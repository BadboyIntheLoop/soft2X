#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

// Initialization of static class members
int ConsoleADWin::instancesCount           = 0;
OSBase::SemHandle ConsoleADWin::_mbsem     = 0;
const char ConsoleADWin::PSEUDO_INT_NAME[] = "OSirq1";
const char ConsoleADWin::SUBS_PREFIX[]     = "ConsoleADWin: ";

/** \name Class constructor
* \param 'diag' - when 'true' output is converted to numbers, at input a sequence of numbers in hex format is expected
*/
ConsoleADWin::ConsoleADWin (bool diag):
    IAsyncDevice(), m_work(m_buf1), m_lastLine(m_buf2), m_isUsable(false), m_diag(diag)
{
    m_buf1[0] = '\0';
    m_buf2[0] = '\0';

    //  BootLog - adds name of a device
    Log.bootPrint ("on Console:\t ");

	// Check if there is only one instance of a class
    if (instancesCount > 0)
    {
        Log.tryAbort ("Critical Error: ConsoleADWin_1.");
        return;
    }

	// Creation of a semaphore of uC/OS-II system or native one to synchronize with thread reading characters
    _mbsem = Os->semCreate (true);  // Semaphore is locked by default
    if (_mbsem == 0)
    {
        Log.tryAbort ("Critical Error: ConsoleADWin_2.");
        return;
    }

	// Creation of a mutex controlling access to the 'lastLine' variable
    m_hMutex = CreateMutex( 
        NULL,                       // Default attributes
        FALSE,                      // No parent
        NULL);                      // No name
    if (m_hMutex == NULL) 
    {
        Log.tryAbort ("Critical Error: ConsoleADWin_3.");
        return;
    }

	// Creation of a new Win32 thread performing reading of a particular chars
	// Note: This thread is not a process of a uC/OS_II (only Win32)
	// uC/OS-II don't control it - this is a potential place for concurrency errors 
	// This thread remains suspended until execution of a 'task' function
    m_thr = CreateThread(NULL, 0u, &ConsoleADWin::threadFunction , this, CREATE_SUSPENDED, NULL);
    if (m_thr == NULL) 
    {
        Log.tryAbort ("Critical Error: ConsoleADWin_4.");
        return;
    }

	// Installation of a function simulates handling of an interrupts in uC/OS-II system
	// In NativeW32 system, execution of this function is necessary (not dangerous)
    PC_IntVectSet(PSEUDO_INT_VECTOR, &ConsoleADWin::pseudoIsr1);

    instancesCount++;

    Log.bootPrint ("OK" CRLF);

    m_isUsable = true;
}

/** \name Method sends to console specified sequence of bytes
* \param 'buf' - buffer with data to send ('/0' character nor end line characters aren't required at the end of that sequence. Binary data are allowed)
* \param 'nBytes' - number of bytes to send
* \return 'true' on success, 'false' otherwise
*/
bool ConsoleADWin::sendBufferWait (const unsigned char* buf, int nBytes)
{
    if ((buf == NULL) || (nBytes < 0))
        return false;

    if (!m_diag)
        _cprintf_s ("%s", buf);
	// Use of '_putch(c)' instead of '_cprintf' in a loop causes strange freezing on that function
    else
    {
		// Diagnostic mode - each byte printed as a number
        for (int i=0; i<nBytes; i++)
            _cprintf_s ("%02x ", buf[i]);
        _cprintf_s ("\n");
    }

    return true;
}

/** \name Method reads a sequence of characters from console. Method exits on pressing 'Enter'.
* \param 'buf' - buffer for read characters which will ends with byte of '0' value without CRLF
* \param 'bufSize' - buffer size
* \param nBytes' - returns a number of read bytes
* \return 'true' on success, 'false' otherwise
*/
bool ConsoleADWin::recvBufferWait (unsigned char* buf, int bufSize, int& nBytes)
{
	// Resuming the Windows thread reading a line from console
	// Thread was created as suspendd in the constructor to not make it running before complete creation of an object
    if (ResumeThread (m_thr) == static_cast<DWORD>(-1))
        Log.errorPrint("ConsoleADWin_recvBufferWait_0");

	// Waiting for reading a line from console by a Windows thread
	// 'gets' cannot be use directly because it blocks the uC/OS-II thread
    int osErr = 0;
    if (Os->semLock (_mbsem, 0, &osErr) != OSBase::SEM_OK)
    {
        Log.errorPrint("ConsoleADWin_recvBufferWait_1 [", osErr, "]");
        // protection against immediate exit if an error occurs
        Os->sleepMs(ERRWAIT_MS);
    }

    //  Czekanie na dostêp do zmiennej
    //  Musi byæ mutex Windowsów, bo zmienna jest zapisywana przez w¹tek Windows, a nie uC/OS-II
    //  Oczekiwanie bêdzie mia³o charakter aktywny z punktu widzenia uC/OS-II.

	// Waiting for access to the variable
	// A Windows mutex is required because this variable is written by a Windows thread (not a uC/OS-II thread)
	// From the side of uC/OS-II it will be an active waiting
    DWORD err = WaitForSingleObject( 
        m_hMutex,      // Handle to mutex
        INFINITE);     // No timeout

    if (err != WAIT_OBJECT_0) // parasoft-suppress  JSF-185 "In macro" // parasoft-suppress  JSF-163 "In macro"
    {
        Log.errorPrint("ConsoleADWin_recvBufferWait_2 [", static_cast<int>(err), "]");
        return false;
    }

	// In diagnostic mode it is assumed thad user inserts the numbers separated with spaces
    if (m_diag)
    {
        FParser parser;
        parser.loadLine(m_lastLine);
        for (int i=0; i<parser.count(); i++)
        {
            int c=0;
            sscanf (parser.getToken(i), "%x", &c);
            buf[i] = static_cast<char>(c);
            buf[i+1] = 0;
            nBytes++;
        }
    }
    else
    {
		// Normal mode
		// Copying of a line. Leave space for characters '\r', '\n' and '\0'.
        unsigned char* p = static_cast<unsigned char*>(_memccpy(buf, m_lastLine, 0, bufSize-3));

		// 'p' holds a pointer to the next characters with copied '\0'
		// If p==NULL means that '\0' was not copied (becouse of the size limitation - truncated)
		// 'p' is set at the position of the next character (therefore '-2')
        if (p == NULL)
            p = buf + bufSize-2;

		// Setting a pointer at the ending character ('0')
        p--;

		// Writing an end line characters (overwriting the '0' character)
        p[0] = '\r';
        p[1] = '\n';
        p[2] = '\0';

		// Character zero is not included in the number of read bytes
        nBytes = p-buf + 2;
    }

    // Releasing of a mutex
    if (ReleaseMutex (m_hMutex) == 0)
    {
        Log.errorPrint("ConsoleADWin_recvBufferWait_3");
        return false;
    }

    return true;
}

/** \name Method returns status of an object creation
* \return 'true' is object was created successfully, 'false' otherwise
*/
bool ConsoleADWin::isUsable (void) const
{
    return m_isUsable;
}

/** \name Method handles Windows thread reading lines from console and collects them into the line buffer
* Collecting of the lines cannot be done by the 'TaskIn' function because waiting for incoming char by
* 'gets' function doesn't return control to the uC/OS-II system
*/
DWORD WINAPI ConsoleADWin::threadFunction (LPVOID pParam)  // parasoft-suppress  JSF-114 "Funkcja obs³ugi w¹tku, nigdy nie wychodzi"
{
	// 'p' holds pointer to the current object ('ThreadFnuction' must be static and accessing its fields directly is not permitted)
    ConsoleADWin* p = static_cast<ConsoleADWin*>(pParam);

    p->m_work = p->m_buf1;
    p->m_lastLine = p->m_buf2;

	// Finding a handler to previously defined (in Win32 port for uC/OS-II) Win32 event imitating specified interruption
    HANDLE irqHandle = OpenEvent(EVENT_ALL_ACCESS, FALSE, PSEUDO_INT_NAME);

    while (true) // parasoft-suppress  CODSTA-82 "Infinite loop - handling of a thread"
    {
        size_t count = 0u;

		// Reading of a line - automatically replacement of '\n' with '\0'
        _cgets_s (p->m_work, CON_BUFSIZE - 1, &count);
        // Skipping empty lines
        if (p->m_work[0] == '\0')
            continue;

		// Waiting for an access to the 'lastLine' variable
        WaitForSingleObject( 
            p->m_hMutex,   // Handle to mutex
            INFINITE);     // No timeout

		// Switching buffers (working and output)
        if (p->m_work == p->m_buf1)
        {
            p->m_work = p->m_buf2;
            p->m_lastLine = p->m_buf1;
        } else
        {
            p->m_work = p->m_buf1;
            p->m_lastLine = p->m_buf2;
        }

        // Releasing of a mutex
        ReleaseMutex (p->m_hMutex);

		// Sending a signal
		// 'irHandle' is not NULL only in uc/OS-II. Requested interruption is handled by dedicated thread of uC/OS-II system,
		// which executes dedicated interruption handler procedure 'PseudoIsr1'.
		// I the case of 'NativeW32' system the 'irHandle' is equal to NULL (==NULL) and 'PseudoIsr1' may be executed directly without
		// danger of interference with uC/OS-II system (because there is no uC/OS-II system)
        if (irqHandle != NULL)
            SetEvent(irqHandle);
        else
            pseudoIsr1();
    }
}

/** \name Method handles uC/OS-II operating system interrupts
* Windows imitates request of an interruption.
*/
void ConsoleADWin::pseudoIsr1(void) // parasoft-suppress  JSF-124 "It cannot be specified as inline - executed via address"
{
    Os->semUnlock (_mbsem);
}

#endif // PILOT_TARGET
