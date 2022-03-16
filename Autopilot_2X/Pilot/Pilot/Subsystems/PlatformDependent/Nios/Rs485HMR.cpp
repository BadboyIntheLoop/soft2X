#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if (PILOT_TARGET == PT_HARDWARE) && (MAGNETOMETER_TYPE == USE_HMR)

/** \name Class constructor
* \param 'portName' - name of a device in a NIOS2IDE environment (e.g. '/dev/uart0')
* \param 'baudRate' - transmission speed (e.g. 115200), by default is 0.
* When speed is 0 then the default value set in a device is taken into accout (value is constant).
* 0 speed in necessary for a '/dev/jtag_uart'.
* \param 'outBufLines' - number of lines in a cyclic buffer which sends data. 10 lines by default. Optional parameters.
*/
Rs485HMR::Rs485HMR(CPU_INT32U portID, int baudRate, int outBufLines)
{
    hmrRSControl_t control;
    _irqHmrRSTag = -1;    // Initialization in a case of not executing the 'linkObserver' method
    int i, cnt;

    // Setting the speed
    int divisor = 0;     // default=460800
    if      (baudRate == 460800) divisor =  0;
    else if (baudRate == 230400) divisor =  1;
    else if (baudRate == 115200) divisor =  3;
    else if (baudRate ==  57600) divisor =  7;
    else if (baudRate ==  38400) divisor = 11;
    else if (baudRate ==  19200) divisor = 23;
    else if (baudRate ==   9600) divisor = 47;
    else if (baudRate ==   4800) divisor = 95;
    else {
       Log.tryAbort ("Critical Error: HMRRS_open_1.");
       initFailed = true;
       return;
    }

    control.word = 0;
    control.bits.baud_div = divisor;
    HMR_mWriteReg(FLYEYE_HMR_BASE, HMR_RS_DIVISOR_OFFSET, control.word);    //need to fill

    // Reseting of an input buffer
    control.word = HMR_mReadReg(FLYEYE_HMR_BASE, HMR_RS_RD_CONTROL_OFFSET);

    cnt = control.bits.rx_level;
    for (i=0; i<cnt; i++)
    {
        	HMR_mReadReg(FLYEYE_HMR_BASE, HMR_RS_RD_FIFO_OFFSET);
    }

    // Appending of a port name to the log
//    Log.bootPrint ("on "); Log.bootPrint (portName); Log.bootPrint (":\t ");

    // Initialization of a buffers of received lines
    work = buf1;
    lastLine = buf2;
    buf1[0] = 0;
    buf2[0] = 0;
    _clearInputFlag = false;

	// Initialization of a cyclic buffer of lines to send
    lUser = 0; lSystem = 0;
    nLines = outBufLines;

    outLinesBuffer = new TLine[nLines];
    if (outLinesBuffer == NULL)
    {
       Log.tryAbort ("Critical Error: HmrRS_open_2.");
       initFailed = true;
       return;
    }

	// Creation of a uC/OS-II system mailbox to synchronize the sending task
    mboxSig = OSMboxCreate ((void*)0);
    if (mboxSig == (OS_EVENT*)0)
    {
        Log.tryAbort ("Critical Error: HmrRS_open_3.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the buffer (receiving)
    if (!_inBufSem.create("HmrRS_in"))
    {
        Log.tryAbort ("Critical Error: HmrRS_open_4.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the cyclic buffer (transmitting)
    if (!_outBufSem.create("HmrRS_out"))
    {
        Log.tryAbort ("Critical Error: HmrRS_open_5.");
        initFailed = true;
        return;
    }

   // Value of a queue lenght above which waiting for a send (sendLine) is activated (when wait==true)
   _maxWaitQueue = MAX_WAIT_QUEUE;

//    Log.bootPrint ("OK" CRLF);
}

/** \name Method link the observed object. Observed is an interruprion of a FastRS.
*/
void Rs485HMR::linkObserver()
{
    _irqHmrRSTag = registerSubj (this, IRQ_HMR_RS);
}

/** \name Method reads the recently received line.
* Class notifies the observers when line is ready to read.
* \return 'true' on success, 'false' otherwise
*/
bool Rs485HMR::getLastLine(char* lineReceived)
{
	// Short waiting on an access to the variable
    if (!_inBufSem.lock())
    {
		// In the case of timeout method returns an empty string
        lineReceived[0] = '\0';
        return false;
    }

	// Copying of a line which is guarantee to end with an '\0' character
    memccpy(lineReceived, lastLine, 0, LINESIZE);

    // Unlocking of an access
    if (!_inBufSem.unlock())
        return false;

    return true;
}

/** \name Method handles an uC/OS-II system task.
* Task is waiting on a notification from the interruption handler (procedure) and emits a signal
* when the line is ready to read by a 'GetLastLine' function.
*/
void Rs485HMR::taskIn(void* pdata)
{
    int i, j, c, cnt, rx_empty;
    hmrRSControl_t control;

	// Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrintf("HmrRS_taskIn_1");
        OSTaskSuspend (OS_PRIO_SELF);
    }

    j = 0;
    while(true)
    {
		// Waiting for the receive of a notice
        OSBase::EvtMask f = waitForAnyAspect ();

        if (!checkAspect(f, _irqHmrRSTag))
            continue; // Other notifications are ignored
        control.word = HMR_mReadReg(FLYEYE_HMR_BASE, HMR_RS_RD_CONTROL_OFFSET);
        cnt = control.bits.rx_level; // Number of bites in an input buffer
        rx_empty = control.bits.rxe;

        if (rx_empty)
            continue;  // This is also when cnt > 0

        // If there is something to receive (rxe == 0)
        if (cnt == 0)
            cnt = 1000; // If there was an overflow

        // Reading data from the input buffer
        for (i=0; i<cnt; i++) {
        		c = HMR_mReadReg(FLYEYE_HMR_BASE, HMR_RS_RD_FIFO_OFFSET);

            if (j < (LINESIZE-1))
            {
                work[j] = c;
                j++;
            }
            if (c == '\r') {
                // end of line
                work[j-1] = '\0';
                if (work[0] == '\0') {
                    j = 0;  // ignoring of an empty line
                    continue;
                }
                // Locking of an access
                if (!_inBufSem.lock())
                    continue;
                // Switching the buffers (working and output)
                if (work == buf1)
                {
                    work = buf2;
                    lastLine = buf1;
                } else
                {
                    work = buf1;
                    lastLine = buf2;
                }
                j = 0;
                // Unlocking the access
                if (!_inBufSem.unlock())
                    continue;
                // Sending a signal
                notify (LINE_RECEIVED);
            }
        }
    }
}

/** \name Method sends a text line.
* The sending task adds automatically an end-line character (CRLF by default)
*/
bool Rs485HMR::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
{
	// Handling of a mode with waiting
    if (withWaiting)
    {
        int count = 0;
        while (true)
        {
			// Calculation of a current queue length
            int q = (lUser >= lSystem) ? (lUser - lSystem) : (nLines - lSystem + lUser);
            // Value of a queue length is too small to activate the waiting - move to send data
            if (q < _maxWaitQueue)
                break;

			// Method returns when maximum number of tries has been exceeded
            if (++count >= MAX_SEND_TRIALS)
                return false;

			// Waiting for a next sampling
            OSTimeDly (SEND_WAIT_TICKS);
        }
    }

    // Semaphore protects variables during execution of this method from other subsystems
    if (!_outBufSem.lock(suppressErrMsg))
        return false;

	// Storing possition in a buffer associated with the task of sending (due to concurrency)
    int lSysTmp = lSystem;
	// New position in buffer associated with the user
    int newLUser = (lUser + 1) % nLines;

	// check if the buffer is full
    if (newLUser == lSysTmp)
    {
		// Buffer is full, sending line is ignored
        _outBufSem.unlock(suppressErrMsg);
        return false;
    }

    char* p = outLinesBuffer[newLUser];
	// Copying of an CRC to the buffer - after the calculation 'p' holds a pointer to the '\0' character of an CRC string
    if (withCrc)
        p = Crc::compute(lineToSend, p);

	// Copying of a line to the buffer
    memccpy (p, lineToSend, 0, LINESIZE-1-(p-outLinesBuffer[newLUser]));
	// Protection if the line was too long and was truncated
    outLinesBuffer[newLUser][LINESIZE-1] = '\0';

    lUser = newLUser;

	// Awakening of the task of sending. Parameter 'outLinesBuffer' is unimportant (dummy).
	// Previous notification may not be handled but this is not an error
    INT8U err = OSMboxPost (mboxSig, outLinesBuffer);
    if (err != OS_NO_ERR && err != OS_MBOX_FULL)
    {
        if (!suppressErrMsg)
            Log.errorPrint("FastRS_sendLine_1 [", err, "]");

        _outBufSem.unlock(suppressErrMsg);
        return false;
    }

    if (!_outBufSem.unlock(suppressErrMsg))
        return false;

    return true;
}

/** \name Method sends a binary buffer to the communication channel (FastRS)
* \note
* - Method must be executed only within one task
* - Do not use 'sendLine' for this communication channel
* - Buffer filling is not controlled - it is assumed that commands are sent so rarely that FastRS have time to send the previous command
* .
*/
bool Rs485HMR::sendBinary (const unsigned char* buf, int size)
{
	int i;

    for (i=0; i<size; i++)
	{
		HMR_mWriteReg(FLYEYE_HMR_BASE, HMR_RS_WR_FIFO_OFFSET, buf[i]);
	}
    return true;
}

/** \name Method handles the uC/OS-II system task of sending the line
*/
void Rs485HMR::taskOut(void* pdata)
{
    INT8U err;
    char *p;
    hmrRSControl_t control;
    int cnt;

	// Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrintf("HmrRS_taskOut_1");
        OSTaskSuspend (OS_PRIO_SELF);
    }

    while (true)
    {
		// Waiting for a signal that something has been appended to the 'outLinesBuffer'
        OSMboxPend (mboxSig, 0, &err);
        if (err != OS_NO_ERR)
        {
            Log.errorPrint("HmrRS_taskOut_2 [", err, "]");
            // Waiting 10 seconds in order to not clog up an error log
            OSTimeDlyHMSM (0, 0, ERRWAIT, 0);
            continue;
        }

		// Copying due to concurrency
        int lUTmp = lUser;

		// Sending of an all waiting lines from the cyclic buffer
        while (lSystem != lUTmp)
        {
			// Check if the line can be send
            	control.word = HMR_mReadReg(FLYEYE_HMR_BASE, HMR_RS_RD_CONTROL_OFFSET);
            cnt = control.bits.tx_level; // Number of characters not send yet
            if (cnt >= (HMR_RS_BUFFER_SIZE - LINESIZE - 2)) {
                // If there is too much to sent then wait for next sampling
                OSTimeDly (1);  // Sending of an entire buffer takes about 15 ms
                continue;
            }
            int newLSystem = (lSystem + 1) % nLines;
            for (p = outLinesBuffer[newLSystem]; *p !=0; p++)
            		HMR_mWriteReg(FLYEYE_HMR_BASE, HMR_RS_WR_FIFO_OFFSET, *p);
            // Appending of an end-line characters
            for (p = endline; *p !=0; p++) {
                	HMR_mWriteReg(FLYEYE_HMR_BASE, HMR_RS_WR_FIFO_OFFSET, *p);
            }
            lSystem = newLSystem;
        }
    }
}

#endif // PILOT_TARGET
