#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_HARDWARE

/** \name Class constructor
* \param 'portName' - device name in NIOS2IDE environment (e.g. '/dev/uart0')
* \param 'baudRate' - transmission speed (e.g. 115200), default is '0'. When the speed is 0 then speed defined in the hardware is used.
* Speed 0 is required for '/dev/jtag_uart0'.
* \param 'outBufLines' - number of lines in cyclic buffer, 10 by default (optional parameter)
*/
UartNIOS::UartNIOS(CPU_INT32U portID, int baudRate, int outBufLines, bool enable):
SerialDeviceBase (enable)
{
    buf1[0] = buf2[0] = 0;
    work = lastLine = buf1;
    isConfigUartHandler = false;
    portIDUART = portID;
    open(portID, baudRate, outBufLines);
}

/** \name Method opens serial port
* \param 'portName' - device name in NIOS2IDE environment (e.g. '/dev/uart0')
* \param 'baudRate' - transmission speed (e.g. 115200), default is '0'. When the speed is 0 then speed defined in the hardware is used.
* Speed 0 is required for '/dev/jtag_uart0'.
* \param 'outBufLines' - number of lines in cyclic buffer, 10 by default (optional parameter)
*/
void UartNIOS::open(CPU_INT32U portID, int baudRate, int outBufLines)
{
    int err;
    
	// Adding port name to the log
    //Log.bootPrint ("on "); Log.bootPrint (portName); Log.bootPrint (":\t ");

	// Initialization of a buffer for received lines
    work = buf1; lastLine = buf2; buf1[0] = 0; buf2[0] = 0;
    _clearInputFlag = false;

	// Initialization of a cyclic buffer for lines to send
    lUser = 0; lSystem = 0;
    nLines = outBufLines;

    outLinesBuffer = new TLine[nLines];
    if (outLinesBuffer == NULL)
    {
       Log.tryAbort ("Critical Error: UartNIOS_open_1.");
       initFailed = true;
       return;
    }

	// Creation of the mailbox in uC/OS-II system to synchronize the sending task
    mboxSig = OSMboxCreate ((void*)0);
    if (mboxSig == (OS_EVENT*)0)
    {
        Log.tryAbort ("Critical Error: UartNIOS_open_7.");
        initFailed = true;
        return;
    }

	// Semaphore controlling assecc to the buffer (receiving)
    if (!_inBufSem.create("UartNIOS_in"))
    {
        Log.tryAbort ("Critical Error: UartNIOS_open_8.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the cyclic buffer (sending)
    if (!_outBufSem.create("UartNIOS_out"))
    {
        Log.tryAbort ("Critical Error: UartNIOS_open_9.");
        initFailed = true;
        return;
    }

	// Queue length beyond which activate waiting for a send (if 'wait' == true)
   _maxWaitQueue = MAX_WAIT_QUEUE;

//    Log.bootPrint ("OK" CRLF);
}

/** \name Method reads recently received line
* Class notifies observers when line is ready to read
* \return 'true' on success, 'false' otherwise
*/
bool UartNIOS::getLastLine(char* lineReceived)
{
	// Short waiting for access to the variable
    if (!_inBufSem.lock())
    {
		// In the case of timeout method returns an empty string
        lineReceived[0] = '\0';
        return false;
    }

	// Copying of a line (line certainly ends with '\0')
    memccpy(lineReceived, lastLine, 0, LINESIZE);
    
	// Blocking access
    if (!_inBufSem.unlock())
        return false;

    return true;
}

/** \name Method handles an uC/OS-II operating system task
* uC/OS-II task emits a signal when line is ready to read by the 'GetLastLine' function
*/
void UartNIOS::taskIn(void* pdata)
{
    while(true)
    {
		// Reading char by char because 'fgets' expects the 'CRLF' and it is assumed that line may ends with any character. The terminal gives usualy only 'CR'.
        int i = 0;
        int c;
        if(!isConfigUartHandler)
        {
        	    	file = AXIUARTLite_Init(portIDUART);
        	    	AXIUARTLite_IntHandler(file, 0);
        	    	isConfigUartHandler = true;
        }

        while(true)
        {
        	c = AXIUARTLite_RdByte(file);

			// Skipping lines starting with any end-line characters
            if ((i == 0) && (c == '\r' || c == '\n'))
            {
                continue;
            }

			//  Bypassing the handling when channel is disabled
            if (!enabled)
            {
                i = 0;
                continue;
            }

			// Checking for a request of clearing the input buffer
            if (_clearInputFlag)
            {
                i = 0;
                _clearInputFlag = false;
                continue;
            }

            if (i < LINESIZE)
                work[i++] = c;

            if (c == '\r' || c == '\n')
                break;
        }

		// End of the string after getting any end-line character or when the buffer is full
        work[i-1] = '\0';

		// Skipping of an empty or truncated lines
        if (work[0] == '\0' || i == LINESIZE)
            continue;

		// Blocking of an access
        if (!_inBufSem.lock())
            continue;

		// Swithing the buffers (working and output)
        if (work == buf1)
        {
            work = buf2;
            lastLine = buf1;
        } else
        {
            work = buf1;
            lastLine = buf2;
        }

        //  Odblokowanie dostêpu
		// Blocking of an access
        if (!_inBufSem.unlock())
            continue;

		// Emitting the signal
        notify (LINE_RECEIVED);
    }
}

/** \name Method sends a text line
* Sending task adds automatically the end line character ('CRLF' by defualt, however it can be changed)
*/
bool UartNIOS::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
{
	// Handling of an waiting mode
    if (withWaiting)
    {
        int count = 0;
        while (true)
        {
			// Calculation of a current queue size
            int q = (lUser >= lSystem) ? (lUser - lSystem) : (nLines - lSystem + lUser);
			// Queue is less than that causing waiting - go to send data
            if (q < _maxWaitQueue)
                break;

			// When exceeded the maximum number of attempts to send method returns
            if (++count >= MAX_SEND_TRIALS)
                return false;

			// Waiting for the next sampling
            OSTimeDly (SEND_WAIT_TICKS);
        }
    }
    
	// Semaphore protecting variables during execution of this method by other subsystems
    if (!_outBufSem.lock(suppressErrMsg))
        return false;

	// Storing position in buffer (position is associated with a sending task)
    int lSysTmp = lSystem;
    // New position in buffer (position is associated with the user)
    int newLUser = (lUser + 1) % nLines;

	// Checking if buffer is full
    if (newLUser == lSysTmp)
    {
		// Buffer is full, line to be sent  is ignored
        _outBufSem.unlock(suppressErrMsg);
        return false;
    }
    
    char* p = outLinesBuffer[newLUser];
	// Copying of a CRC to the buffer - after the calculation 'p' holds pointer to the '\0' which ends the sring of CRC
    if (withCrc)
        p = Crc::compute(lineToSend, p);

	// Copying of a line to the buffer
    memccpy (p, lineToSend, 0, LINESIZE-1-(p-outLinesBuffer[newLUser]));
    // Protection if the line was too long and was truncated
    outLinesBuffer[newLUser][LINESIZE-1] = '\0';

    lUser = newLUser;

	// Awakening of the sending tasks. 'outLinesBuffer' parameter is dummy
	// Previous notification may not be handled but this is not an error
    INT8U err = OSMboxPost (mboxSig, outLinesBuffer);
    if (err != OS_NO_ERR && err != OS_MBOX_FULL)
    {
        if (!suppressErrMsg)
            Log.errorPrint("UartNIOS_sendLine_1 [", err, "]");
        
        _outBufSem.unlock(suppressErrMsg);
        return false;
    }

    if (!_outBufSem.unlock(suppressErrMsg))
        return false;
    
    return true;         
}

/** \name  Method sends content of specified buffer
* Implementation without a cyclic buffer - method waits for finalizing the operation
* \note Writing is not synchronous. Exetute method only in one task. Do not use 'sendLine' on this channel.
* Method is designed mainly to configure devices after start of the system.
*/
bool UartNIOS::sendBinary (const unsigned char* buf, int size)
{
    bool ret = AXIUARTLite_WrStr (file, (CPU_CHAR*)buf, size);
    
    return ret;
}

/** \name Method clears the input buffer
*/
void UartNIOS::clearInput (void)
{
    _clearInputFlag = true;
}

/** \name Method handles uC/OS-II operating system task sending line of a text
*/
void UartNIOS::taskOut(void* pdata)
{
    INT8U err;
    while (true)
    {
		// Waiting for a signal that something has been added to 'outLinesBuffer'
        OSMboxPend (mboxSig, 0, &err);
        if (err != OS_NO_ERR)
        {
            Log.errorPrint("UartNIOS_taskOut_2 [", err, "]");
            // Wait 10s to not clog up the error log
            OSTimeDlyHMSM (0, 0, ERRWAIT, 0);
            continue;
        }

        // Copying due to concurrency
        int lUTmp = lUser;

		// Sending all of the waiting lines to the cyclic buffer
        while (lSystem != lUTmp)
        {
            int newLSystem = (lSystem + 1) % nLines;
			// Sending of a line (mthod may stop here when buffer is full)
            AXIUARTLite_WrStr (file, outLinesBuffer[newLSystem], strlen (outLinesBuffer[newLSystem]));
			// Adding end line characters
            AXIUARTLite_WrStr (file, endline, strlen (endline));
            lSystem = newLSystem;
        }
    }
}


#endif // PILOT_TARGET

