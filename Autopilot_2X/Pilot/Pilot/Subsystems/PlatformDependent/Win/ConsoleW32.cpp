#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

// Initialization of a class static members
int ConsoleW32::instancesCount = 0;
OSBase::SemHandle ConsoleW32::_mbsem = 0;
const char ConsoleW32::PSEUDO_INT_NAME[] = "OSirq1";
const char ConsoleW32::SUBS_PREFIX[]          = "ConsoleW32: ";


ConsoleW32::ConsoleW32(void):
    SerialDeviceBase(), work(buf1), lastLine(buf1)
{
    buf1[0] = buf2[0] = 0;
    open();
}

/** \name Method opens the console (handling of input and output)
*/
void ConsoleW32::open(void)
{
	// Bootlog - appends name of a device
    Log.bootPrint ("on Console:\t ");

    // Check the number of class instances
    if (instancesCount > 0)
    {
        Log.tryAbort ("Critical Error: ConsoleW32_open_0.");
        initFailed = true;
        return;
    }

	// Creation of a mailbox of the uC/OS-II system to synchronize with thread reading characters
    _mbsem = Os->semCreate (true);  // Semaphore is locked by default
    if (_mbsem == 0)
    {
        Log.tryAbort ("Critical Error: ConsoleW32_open_1.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the in/out functions
    if (!_vsem.create("ConsoleW32"))
    {
        Log.tryAbort ("Critical Error: ConsoleW32_open_1a.");
        initFailed = true;
        return;
    }

	// Creation of a mutex to control access to the 'lastLine' variable
    hMutex = CreateMutex( 
        NULL,                       // Default attributes
        FALSE,                      // No owner
        NULL);                      // No name
    if (hMutex == NULL) 
    {
        Log.tryAbort ("Critical Error: ConsoleW32_open_2.");
        initFailed = true;
        return;
    }

	// Creation of a new Win32 thread performing reading of a particular chars
	// Note: This thread is not a process of a uC/OS_II (only Win32)
	// uC/OS-II don't control it - this is a potential place for concurrency errors 
	// This thread remains suspended until execution of a 'task' function
    thr = CreateThread(NULL, 0, ConsoleW32::threadFunction , this, CREATE_SUSPENDED, NULL);
    if (thr == NULL) 
    {
        Log.tryAbort ("Critical Error: ConsoleW32_open_3.");
        initFailed = true;
        return;
    }

	// Installation of a function simulates handling of an interrupts in uC/OS-II system
	// In NativeW32 system, execution of this function is necessary (not dangerous)
    PC_IntVectSet(PSEUDO_INT_VECTOR, ConsoleW32::PseudoIsr1);

    instancesCount++;

    Log.bootPrint ("OK" CRLF);
}

/** \name Method reads recently received line. Class posts an event when line is ready.
* \return 'true' if line was read successfully
*/
bool ConsoleW32::getLastLine(char* lineReceived)
{
	// Waiting on access to the variable
	// Windows mutex is required because this variable is written by a Windows thread (not a uC/OS-II thread)
	// From the side of uC/OS-II it will be an active waiting
    DWORD err = WaitForSingleObject( 
        hMutex,      // Handle to mutex
        INFINITE);   // No timeout

    if (err != WAIT_OBJECT_0)
    {
        Log.errorPrint("ConsoleW32_getLastLine_1 [", static_cast<int>(err), "]");
        return false;
    }

    // Copying of a line
    _memccpy(lineReceived, lastLine, 0, LINESIZE);

    // Releasing of a mutex
    if (ReleaseMutex (hMutex) == 0)
    {
        Log.errorPrint("ConsoleW32_getLastLine_2");
        return false;
    }

    return true;
}

/** \name Method sends a text line. Method don't appends end line characters.
* Buffering or creating of a new thread for sending data is not necessary.
*/
bool ConsoleW32::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
{
    if (!_vsem.lock(suppressErrMsg))
        return false;

	// Optional addition of an CRC sum
    char crcBuf[20];
    crcBuf[0] = 0;
    if (withCrc)
        Crc::compute(lineToSend, crcBuf);

    _cprintf_s ("%s%s%s", crcBuf, lineToSend, endline);

    if (!_vsem.unlock(suppressErrMsg))
        return false;

	// Under NIOS 'sendLine' function works on cyclic buffer. If 'withWaiting' flag is set then function waits
	// until a buffer size will be correspondingly small to put to it a line.
	// Under Win32 there is no cyclic buffer and function 'sendTo' is fast. To simulate working similar to NIOS-II
	// (giving control to another task) sleeping is applied to slow down processing.
    if (withWaiting)
        Os->sleepMs(10);

    return true;
}

/** \name Method handles an uC/OS-II operating system task
* Task sends signal when the line is ready to read by the 'GetLastLine' function
*/
void ConsoleW32::taskIn(void* pdata)
{
    // Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrint("ConsoleW32_task_1");
        Os->taskSuspend();
    }

    if (ResumeThread (thr) == static_cast<DWORD>(-1))
        Log.errorPrint("ConsoleW32_task_2");

    for (;;)
    {
        // Version with binary semaphores
        int osErr = 0;
        if (Os->semLock (_mbsem, 0, &osErr) != OSBase::SEM_OK)
        {
            Log.errorPrint("ConsoleW32_task_3 [", osErr, "]");
            Os->sleepMs(ERRWAIT_MS);
        }

		// Sending a signal
        notify (LINE_RECEIVED);
    }
}

/** \name Method handles uC/OS-II system task for sending text line
*/
void ConsoleW32::taskOut(void* pdata)
{
    // Dummy method - on Windows it is not necessary
	Os->taskSuspend();
}

/** \name Method handles Windows thread reading characters from console and collects this data in the line buffer
* Collecting of the characters cannot be done by the 'taskIn' function because waiting for incoming characters by
* 'gets' function doesn't return control to the uC/OS-II system
*/
DWORD WINAPI ConsoleW32::threadFunction( LPVOID pParam ) 
{
	// 'p' holds pointer to the current object ('ThreadFnuction' must be static and accessing its fields directly is not permitted)
    ConsoleW32* p = static_cast<ConsoleW32*>(pParam);

    p->work = p->buf1;
    p->lastLine = p->buf2;

	// Finding a handler to previously defined (in Win32 port for uC/OS-II) Win32 event imitating specified interruption
    HANDLE irqHandle = OpenEvent(EVENT_ALL_ACCESS, FALSE, PSEUDO_INT_NAME);

    for (;;)
    {
        size_t count = 0;
        _cgets_s (p->work, LINESIZE - 1, &count);
        // Skipping empty lines
        if (p->work[0] == '\0')
            continue;

        // Waiting for access to the 'lastLine' variable
        WaitForSingleObject( 
            p->hMutex,   // Handle to mutex
            INFINITE);   // No timeout

        // Switching buffers (working and output)
        if (p->work == p->buf1)
        {
            p->work = p->buf2;
            p->lastLine = p->buf1;
        } else
        {
            p->work = p->buf1;
            p->lastLine = p->buf2;
        }

		// Releasing of the mutex
        ReleaseMutex (p->hMutex);
       
		// Sending a signal
		// 'irHandle' is not NULL only in uc/OS-II. Requested interruption is handled by dedicated thread of uC/OS-II system,
		// which executes dedicated interruption handler procedure 'PseudoIsr1'.
		// I the case of 'NativeW32' system the 'irHandle' is equal to NULL (==NULL) and 'PseudoIsr1' may be executed directly without
		// danger of interference with uC/OS-II system (because there is no uC/OS-II system)
		if (irqHandle != NULL)
            SetEvent(irqHandle);
        else
            PseudoIsr1();
    }
}

/** \name Method handles uC/OS-II operating system interrupts
* Windows imitates request of an interruption.
*/
void ConsoleW32::PseudoIsr1(void)
{
    Os->semUnlock (_mbsem);
}

#endif // PILOT_TARGET

