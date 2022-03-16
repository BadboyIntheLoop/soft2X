#include "PilotIncludes.h"

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

// Initialization of a class static members
int UdpW32::instancesCount = 0;
OSBase::SemHandle UdpW32::_mbsem = 0;
const char UdpW32::PSEUDO_INT_NAME[] = "OSirq2";

UdpW32::UdpW32(u_short inPort, u_short outPort, const char* outIP)
{
    buf1[0] = buf2[0] = 0;
    work = lastLine = buf1;
    open (inPort, outPort, outIP);
}

/** \name Method performs opening of the UDP port
* \param 'inPort' - id of a port receiving data
* \param 'outPort' - id of a port sending data
* \param 'outIp' - IP address on which data will be send
*/
void UdpW32::open(u_short inPort, u_short outPort, const char* outIP)
{
    int err;

    WSADATA wsaData;
    sockaddr_in recvAddr;

    // BootLog - adds the name of the port
    Log.bootPrintf("on UDP (out:%s/%u, in:*/%u):\t ", outIP, outPort, inPort);

	// Check the number of class instances
    if (instancesCount > 0)
    {
        Log.tryAbort ("Critical Error: UdpW32_open_0.");
        initFailed = true;
        return;
    }

    // Winsock initialization
    err = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (err) 
    {
        WSACleanup();
        // Ends the program, depending on the state of the object Log 
        Log.tryAbort ("Critical Error: UdpW32_open_1.");
        initFailed = true;
        return;
    }

	// Creation of a socket for receiving datagrams
    recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (recvSocket == INVALID_SOCKET)
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_2.");
        initFailed = true;
        return;
    }

    int bs;

    // Setting the receive buffer smaller than default
    // Default size is 8192

    bs = 512;
    err = setsockopt(recvSocket, SOL_SOCKET, SO_RCVBUF, (char*)&bs, sizeof(bs));
    if (err != 0)
    {
        Log.tryAbort ("Critical Error: UdpW32_open_3.");
        initFailed = true;
        return;
    }

	// Set 'reusable' for a socket (needed to work together with the simulation controller)
    bs = 1;
    err = setsockopt(recvSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bs, sizeof(bs));
    if (err != 0)
    {
        Log.tryAbort ("Critical Error: UdpW32_open_3a.");
        initFailed = true;
        return;
    }

	// Binding of a receiving socket to any address at specified port
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(inPort);
    recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    err = bind(recvSocket, (SOCKADDR *) &recvAddr, sizeof(recvAddr));
    if (err) 
    {
        WSAGetLastError();
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_4.");
        initFailed = true;
        return;
    }

	// Creating of a socket to send datagrams
    sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sendSocket == INVALID_SOCKET)
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_5.");
        initFailed = true;
        return;
    }

	// Set 'reusable' for a socket (needed to work together with the simulation controller)
    bs = 1;
    err = setsockopt(sendSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bs, sizeof(bs));
    if (err != 0)
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_5a.");
        initFailed = true;
        return;
    }

	// Set the structure of a sending socket with specified address at specified port - no binding
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(outPort);
    sendAddr.sin_addr.s_addr = inet_addr(outIP);

	// Creation of a pre locked semaphore to synchronize with thread reading data
    _mbsem = Os->semCreate (true);
    if (_mbsem == 0)
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_6.");
        initFailed = true;
        return;
    }

	// Semaphore securing access to the socket sending a line
    if (!_vsem.create("UdpW32"))
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_6a.");
        initFailed = true;
        return;
    }

	// Creation of a mutex controling access to the 'lastLine' variable
    hMutex = CreateMutex( 
        NULL,                       // Default attributes
        FALSE,                      // No parent
        NULL);                      // Noname mutex
    if (hMutex == NULL) 
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_7.");
        initFailed = true;
        return;
    }

	// Creation of a new Win32 thread performing reading of a particular chars
	// Note: This thread is not a process of a uC/OS_II (only Win32)
	// uC/OS-II don't control it - this is a potential place for concurrency errors 
	// This thread remains suspended until execution of a 'task' function
    thr = CreateThread(NULL, 0, UdpW32::threadFunction , this, CREATE_SUSPENDED, NULL);
    if (thr == NULL) 
    {
        WSACleanup();
        Log.tryAbort ("Critical Error: UdpW32_open_8.");
        initFailed = true;
        return;
    }

	// Installation of a function simulates handling of an interrupts in uC/OS-II system
	// In NativeW32 system, execution of this function is necessary (not dangerous)
    PC_IntVectSet(PSEUDO_INT_VECTOR, UdpW32::PseudoIsr2);

    instancesCount++;

    Log.bootPrint ("OK" CRLF);
}

/** \name Method reads recently received line. Class posts an event when line is ready.
* \return 'true' if line was read successfully
*/
bool UdpW32::getLastLine(char* lineReceived)
{
    DWORD err;

	// Waiting on access to the variable
	// A Windows mutex is required because this variable is written by a Windows thread (not a uC/OS-II thread)
	// From the side of uC/OS-II it will be an active waiting
    err = WaitForSingleObject( 
        hMutex,		// Handle to mutex
        INFINITE);	// No timeout

    if (err != WAIT_OBJECT_0)
    {
        Log.errorPrint("UdpW32_getLastLine_1 [", static_cast<int>(err), "]");
        return false;
    }

    // Copying of a line
    _memccpy(lineReceived, lastLine, 0, LINESIZE);

    // Releasing the mutex
    if (!ReleaseMutex (hMutex))
    {
        Log.errorPrintf("UdpW32_getLastLine_2");
        return false;
    }

    return true;
}

/** \name Method sends a line of text with line termination character
* Buffering or creation of a separate thread sending data is not neccessary
*/
bool UdpW32::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
{
    char buf[LINESIZE];

    if (!_vsem.lock(suppressErrMsg))
        return false;

    // Optional addition of CRC sum
    char crcBuf[20];
    crcBuf[0] = 0;
    if (withCrc)
        Crc::compute(lineToSend, crcBuf);

	// Addition of CRC sum and end of line - under Windows do not have to worry about performance
    int count = SNPRINTF(buf, sizeof(buf), "%s%s%s", crcBuf, lineToSend, endline);
    if (count == -1)
    {
        if (!suppressErrMsg)
            Log.errorPrintf("UdpW32_sendLine_1");
        _vsem.unlock(suppressErrMsg);
        return false;
    }

	// Send without ending zero
    int n = sendto(sendSocket, 
        buf, 
        strlen(buf), 
        0, 
        (SOCKADDR *) &sendAddr, 
        sizeof(sendAddr));

    if (n == SOCKET_ERROR)
    {
        if (!suppressErrMsg)
            Log.errorPrintf("UdpW32_sendLine_2 [%d]", WSAGetLastError());
        _vsem.unlock(suppressErrMsg);
        return false;
    }

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
void UdpW32::taskIn(void* pdata)
{
	// Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrintf("UdpW32_task_1");
        Os->taskSuspend();
    }

    if (ResumeThread (thr) == static_cast<DWORD>(-1))
        Log.errorPrintf("UdpW32_task_2");

    for (;;)
    {
		// Waiting for the release of the semaphore (infinite)
        int osErr = 0;
        OSBase::SEMCODE err = Os->semLock (_mbsem, 0, &osErr);
        if (err != OSBase::SEM_OK)
        {
            Log.errorPrint("UdpW32_task_3 [", osErr, "]");
            Os->sleepMs(ERRWAIT_MS);
        }

        // Sending a signal
        notify (LINE_RECEIVED);
    }
}

/** \name Method handles uC/OS-II operating system task sending line of a text
*/
void UdpW32::taskOut(void* pdata)
{
    // Dummy method - under Windows it is not necessary
	Os->taskSuspend();
}

/** \name Method handles Windows thread reading characters from port and collects this data in the line buffer
* Collecting of the characters cannot be done by the 'task' function because waiting for incoming characters by
* 'ReadLine' function doesn't return control to the uC/OS-II system
*/
DWORD WINAPI UdpW32::threadFunction( LPVOID pParam ) 
{
	// 'p' holds pointer to the current object ('ThreadFnuction' must be static and accessing its fields directly is not permitted)
	UdpW32* p = static_cast<UdpW32*>(pParam);
    char RecvBuf[UDP_BUFSIZE];
    int count;

    p->work = p->buf1;
    p->lastLine = p->buf2;

	// Finding a handler to previously defined (in Win32 port for uC/OS-II) Win32 event imitating specified interruption
    HANDLE irqHandle = OpenEvent(EVENT_ALL_ACCESS, FALSE, PSEUDO_INT_NAME);

    for (;;)
    {
        // Reading of a datagram
        count = recvfrom(p->recvSocket, 
            RecvBuf, 
            UDP_BUFSIZE, 
            0, 
            (SOCKADDR*)0, 
            (int*)0);
 
		// Copying of the first line from datagram to the line buffer, rest of lines are ignored
		// Instead of 'Count' uses a 'LINESIZE, because 'count' may exceed the buffer size 
        char* f = static_cast <char*> (_memccpy(p->work, RecvBuf, '\n', LINESIZE-1));
        if (f == NULL)
        {
            // Line is broken - remove entire line
            continue;
        }
        else
        {
            // Removing end line character, 'f' points to char after the '\n'
            *(f-1) = (char)0;
            // If CR was before LF
            if ((f - p->work > 1) && (*(f-2) == '\r'))
                *(f-2) = (char)0;
        }

        // Skipping empty rows
        if (*(p->work) == (char)0)
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

        // Releasing of a mutex (only for debugging)
		ReleaseMutex (p->hMutex);
		// From the Win32 thread uC/OS-II functions should not be executed
        
		// Sending a signal
		// 'irHandle' is not NULL only in uc/OS-II. Requested interruption is handled by dedicated thread of uC/OS-II system,
		// which executes dedicated interruption handler procedure 'PseudoIsr1'.
		// I the case of 'NativeW32' system the 'irHandle' is equal to NULL (==NULL) and 'PseudoIsr1' may be executed directly without
		// danger of interference with uC/OS-II system (because there is no uC/OS-II system)
		if (irqHandle != NULL)
            SetEvent(irqHandle);
        else
            PseudoIsr2();
    }
}

/** \name Method handles uC/OS-II operating system interrupts
* Windows imitates request of an interruption.
*/
void UdpW32::PseudoIsr2(void)
{
    Os->semUnlock (_mbsem);
}

#endif // PILOT_TARGET == PT_WIN32

