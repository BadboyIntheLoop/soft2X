#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

UartW32::UartW32(LPCSTR portName, DWORD baudRate):
    SerialDeviceBase(), work(buf1), lastLine(buf1), _mbsem(0)
{
    buf1[0] = buf2[0] = 0;
    open (portName, baudRate);
}

/** \name Method performs opening of the RS232 serial port
* \param 'portName' - port name e.g. 'COM1'
* \param 'baudRate' - transmission speed e.g. 4800 (must be one of the standard rate)
*/
void UartW32::open(LPCSTR portName, DWORD baudRate)
{
    //  BootLog - adds a port name
    Log.bootPrint ("on "); Log.bootPrint (portName); Log.bootPrint (":\t ");

    // Opening of a serial port
    h = CreateFileA(portName,
            GENERIC_READ|GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            0,
            NULL
            );
    
   if (h == INVALID_HANDLE_VALUE)  // parasoft-suppress  JSF-182_b "Win32 allows comparinson of the pointer and value"
   {
	   // Ends application, depending on the state of the object Log
       Log.tryAbort ("Critical Error: UartW32_open_1.");
       initFailed = true;
       return;
   }

   // Reading of the current parameters
   BOOL fSuccess = GetCommState(h, &dcb);
   if (fSuccess == 0) 
   {
       Log.tryAbort ("Critical Error: UartW32_open_2.");
       initFailed = true;
       return;
   }

   // Setting up of some new parameters
   dcb.BaudRate = baudRate;      // set the baud rate
   dcb.ByteSize = 8;             // data size, xmit, and rcv
   dcb.Parity = NOPARITY;        // no parity bit
   dcb.StopBits = ONESTOPBIT;    // one stop bit
  
   fSuccess = SetCommState(h, &dcb);
   if (fSuccess == 0) 
   {
       Log.tryAbort ("Critical Error: UartW32_open_3.");
       initFailed = true;
       return;
   }

   // Disabling timeout for a port
   ct.ReadIntervalTimeout = 0;
   ct.ReadTotalTimeoutConstant = 0;
   ct.ReadTotalTimeoutMultiplier = 0;
   ct.WriteTotalTimeoutConstant = 10;
   ct.WriteTotalTimeoutMultiplier = 1;
   fSuccess = SetCommTimeouts(h, &ct);
   if (fSuccess == 0) 
   {
       Log.tryAbort ("Critical Error: UartW32_open_4.");
       initFailed = true;
       return;
   }
 
   // Creation of a pre locked semaphore to synchronize with thread reading data
   _mbsem = Os->semCreate (true);   // Semaphore is locked by default
   if (_mbsem == 0)
   {
       Log.tryAbort ("Critical Error: UartW32_open_5.");
       initFailed = true;
       return;
   }

   // Creation of a mutex controling access to the 'lastLine' variable
   hMutex = CreateMutex( 
       NULL,                       // Default attributes
       FALSE,                      // No parent
       NULL);                      // No name mutex
   if (hMutex == NULL) 
   {
       Log.tryAbort ("Critical Error: UartW32_open_7.");
       initFailed = true;
       return;
   }

   // Reset of a port errors and clearing the buffer
   ClearCommBreak(h);
   PurgeComm (h, PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_RXABORT);

	// Creation of a new Win32 thread performing reading of a particular chars
	// Note: This thread is not a process of a uC/OS_II (only Win32)
	// uC/OS-II don't control it - this is a potential place for concurrency errors 
	// This thread remains suspended until execution of a 'task' function
   thr = CreateThread(NULL, 0, UartW32::threadFunction , this, CREATE_SUSPENDED, NULL);
   if (thr == NULL) 
   {
       Log.tryAbort ("Critical Error: UartW32_open_6.");
       initFailed = true;
       return;
   }

   // Setting higher priority such as for the uC/OS-II system
   BOOL b = SetThreadPriority (thr, THREAD_PRIORITY_ABOVE_NORMAL);
   if (b == 0) 
   {
       Log.tryAbort ("Critical Error: UartW32_open_7.");
       initFailed = true;
       return;
   }

   Log.bootPrint ("OK" CRLF);
}

/** \name Method reads lastly received line
* Class posts an event when line is ready
* \return 'true' on success, 'false' otherwise
*/
bool UartW32::getLastLine(char* lineReceived)
{
	// Waiting for an access to the variable
    DWORD err = WaitForSingleObject( 
        hMutex,		// Mutex handle
        INFINITE);	// No timeout

    if (err != WAIT_OBJECT_0)
    {
        Log.errorPrint("UartW32_getLastLine_1 [", static_cast<int>(err), "]");
        return false;
    }

    // Copying of a line
    _memccpy(lineReceived, lastLine, 0, LINESIZE);

    // Releasing of a mutex
    if (ReleaseMutex (hMutex) == 0)
    {
        Log.errorPrintf("UartW32_getLastLine_2");
        return false;
    }

    return true;
}

/** \name Method sends a line to the device
* Implementation for Windows assumes that the buffer system is sufficiently large that it never came to
* waiting in this function. Size of the buffer is set in the 'open' function (constructor). In NIOS an extra
* thread to send data was needed, which was abandoned here.
*/
bool UartW32::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
{
    if (!suppressErrMsg)
        Log.errorPrintf("UartW32_sendLine_1: not implemented");
    return false;
}

/** \name Method handles task of a uC/OS-II system
* Task emits signal when line is ready to read by the 'GetLastLine' function
*/
void UartW32::taskIn(void* pdata)
{
	// Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrintf("UartW32_task_1");
        Os->taskSuspend();
    }

    if (ResumeThread (thr) == (DWORD)-1)
        Log.errorPrintf("UartW32_task_2");

    for (;;)
    {
		// Waiting for releasing of the semaphore (infinite)
        int osErr = 0;
        OSBase::SEMCODE err = Os->semLock (_mbsem, 0, &osErr);
        if (err != OSBase::SEM_OK)
        {
            Log.errorPrint("UartW32_task_3 [", osErr, "]");
            Os->sleepMs(ERRWAIT_MS);
        }

        // Sending a signal
        notify (LINE_RECEIVED);
    }
}

/** \name Method handles uC/OS-II system task for sending text line
*/
void UartW32::taskOut(void* pdata)
{
    // Dummy method - sending data is not implemented
	Os->taskSuspend();
}

/** \name Method handles Windows thread reading characters from port and collects this data in the line buffer
* Collecting of the characters cannot be done by the 'task' function because waiting for incoming characters by
* 'ReadLine' function doesn't return control to the uC/OS-II system
*/
DWORD WINAPI UartW32::threadFunction( LPVOID pParam ) 
{
	// 'p' holds pointer to the current object ('ThreadFnuction' must be static and accessing its fields directly is not permitted)
    UartW32* p = (UartW32*) pParam;
    int buf = 0;
    DWORD nread = 0;
    int pos = 0;    // Position in current buffer
    bool skipToNewLine = false;
    char c = 0;

    p->work = p->buf1;
    p->lastLine = p->buf2;

    for (;;)
    {
		// Reading of a single byte
        if (ReadFile(p->h, &buf, 1, &nread, NULL) == 0)
            Log.errorPrintf("UartW32_threadFunction_1");

        c = static_cast <char> (buf & 0xFF);

        if (pos == LINESIZE)
            pos--;

        if (c == '\n' || c == '\r')
        {
            if (!skipToNewLine)
            {
                p->work[pos] = 0;
                pos = 0;
                skipToNewLine = true;

				// Waiting for access to the 'lastLine' variable
                DWORD err = WaitForSingleObject( 
                    p->hMutex,   // Handle to mutex
                    INFINITE);   // No timeout

                if (err != WAIT_OBJECT_0)
                    Log.errorPrint("UartW32_threadFunction_2 [", static_cast<int>(err), "]");

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

                // Releasing of a mutex
                if (ReleaseMutex (p->hMutex) == 0)
                    Log.errorPrintf("UartW32_threadFunction_3");

                // Sending a signal
                int osErr = 0;
                OSBase::SEMCODE err2 = Os->semUnlock (p->_mbsem, &osErr);
                if (err2 != OSBase::SEM_OK && err2 != OSBase::SEM_FIXED)
                    Log.errorPrint("UartW32_threadFunction_4 [", osErr, "]");
            }
        } else
        {
            skipToNewLine = false;
            p->work[pos++] = static_cast <char> (buf & 0xFF);
        }
    }
}

#endif // PILOT_TARGE
