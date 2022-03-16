#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_HARDWARE

/** \name Class constructor
* \param 'portName' - name of a device in a NIOS2IDE environment (e.g. '/dev/uart0')
* \param 'baudRate' - transmission speed (e.g. 115200), by default is 0.
* When speed is 0 then the default value set in a device is taken into accout (value is constant).
* 0 speed in necessary for a '/dev/jtag_uart'.
* \param 'outBufLines' - number of lines in a cyclic buffer which sends data. 10 lines by default. Optional parameters.
*/
UartCAM::UartCAM(CPU_INT32U portID, int baudRate, int outBufLines)
{
    camControl_t control;
    _irqCamTag = -1;    // Initialization in a case of not executing the 'linkObserver' method
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
    else if (baudRate ==   921600) divisor = 127;
    else {
       Log.tryAbort ("Critical Error: Cam_open_1.");
       initFailed = true;
       return;
    }

    control.word = 0;
    control.bits.baud_div = divisor;
#if USE_CAM == CAM_ENABLE
    VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, CAM_DIVISOR_OFFSET, control.word);
#if USE_WSM == WSM_ENABLE
    VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, CAM_SET_IRQ_CHARACTER, 0x24);
#else
    VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, CAM_SET_IRQ_CHARACTER, 0x24);
#endif

    // Reseting of an input buffer
    control.word = VIETTEL_FASTRS_mReadReg(FLYEYE_CAM_BASE, CAM_RD_CONTROL_OFFSET);


    cnt = control.bits.rx_level;
    for (i=0; i<cnt; i++)
    {
        	VIETTEL_FASTRS_mReadReg(FLYEYE_CAM_BASE, CAM_RD_FIFO_OFFSET);
    }
#endif

    // Appending of a port name to the log
//    Log.bootPrint ("on "); Log.bootPrint (portName); Log.bootPrint (":\t ");

    // Initialization of a buffers of received lines
    work = buf1; lastLine = buf2; buf1[0] = 0; buf2[0] = 0;
    _clearInputFlag = false;

	// Initialization of a cyclic buffer of lines to send
    lUser = 0; lSystem = 0;
    nLines = outBufLines;

    outLinesBuffer = new TLine[nLines];
    if (outLinesBuffer == NULL)
    {
       Log.tryAbort ("Critical Error: Cam_open_2.");
       initFailed = true;
       return;
    }

	// Creation of a uC/OS-II system mailbox to synchronize the sending task
    mboxSig = OSMboxCreate ((void*)0);
    if (mboxSig == (OS_EVENT*)0)
    {
        Log.tryAbort ("Critical Error: Cam_open_3.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the buffer (receiving)
    if (!_inBufSem.create("Cam_in"))
    {
        Log.tryAbort ("Critical Error: Cam_open_4.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the cyclic buffer (transmitting)
    if (!_outBufSem.create("Cam_out"))
    {
        Log.tryAbort ("Critical Error: Cam_open_5.");
        initFailed = true;
        return;
    }

	// Semaphore controlling access to the cyclic buffer (transmitting)
    if (!_vSem.create("Cam_data"))
    {
        Log.tryAbort ("Critical Error: Cam_open_6.");
        initFailed = true;
        return;
    }

   // Value of a queue lenght above which waiting for a send (sendLine) is activated (when wait==true)
   _maxWaitQueue = MAX_WAIT_QUEUE;

   cam_msg_res.mode 		= 0;
   cam_msg_res.px 			= 0;
   cam_msg_res.py 			= 0;
   cam_msg_res.width 		= 0;
   cam_msg_res.height 		= 0;
   cam_msg_res.hfov 		= 0.0f;
   cam_msg_res.gimbalPan 	= 0.0f;
   cam_msg_res.gimbalTilt 	= 0.0f;
   cam_msg_res.gimbalRoll 	= 0.0f;

#if USE_WSM == WSM_ENABLE
   wsm_msg_res.magX = 0;
   wsm_msg_res.magY = 0;
   wsm_msg_res.magZ = 0;
   wsm_msg.read = 0;
   wsm_msg.wsm_state = wsm_msg_parse_t::SYNC1;
#endif

   //
   crc_error_counter = 0;
   crc_counter = 0;
   cam_msg.cam_state = cam_msg_parse_t::SYNC1;
}

/** \name Method link the observed object. Observed is an interruprion of a FastRS.
*/
void UartCAM::linkObserver()
{
    _irqCamTag = registerSubj (this, IRQ_CAM);
}

/** \name Method reads the recently received line.
* Class notifies the observers when line is ready to read.
* \return 'true' on success, 'false' otherwise
*/
bool UartCAM::getLastLine(char* lineReceived)
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

/** \name Method handles an uC/OS-II operating system task
* uC/OS-II task emits a signal when line is ready to read by the 'GetLastLine' function
*/
void UartCAM::taskIn(void* pdata)
{
    int i, c, cnt, rx_empty;
    camControl_t control;

	// Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrintf("Cam_taskIn_1");
        OSTaskSuspend (OS_PRIO_SELF);
    }

    bool ret = false;
    while(true)
    {
		// Waiting for the receive of a notice
        OSBase::EvtMask f = waitForAnyAspect ();

        if (!checkAspect(f, _irqCamTag))
            continue; // Other notifications are ignored
#if USE_CAM == CAM_ENABLE
        control.word = VIETTEL_FASTRS_mReadReg(FLYEYE_CAM_BASE, CAM_RD_CONTROL_OFFSET);
#endif
        cnt = control.bits.rx_level; // Number of bites in an input buffer
        rx_empty = control.bits.rxe;
        if (rx_empty)
            continue;  // This is also when cnt > 0

        // If there is something to receive (rxe == 0)
        if (cnt == 0)
            cnt = 1000; // If there was an overflow
        // Reading data from the input buffer
        for (i=0; i<cnt; i++) {
#if USE_CAM == CAM_ENABLE
        	c = VIETTEL_FASTRS_mReadReg(FLYEYE_CAM_BASE, CAM_RD_FIFO_OFFSET);
#endif
#if USE_WSM == WSM_ENABLE
        	ret |= parse_wsm(c);
#else
        	ret |= parse_cam(c);
#endif
        }
    }
}

/** \name Method sends a text line.
* The sending task adds automatically an end-line character (CRLF by default)
*/
bool UartCAM::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
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

/** \name  Method sends content of specified buffer
* Implementation without a cyclic buffer - method waits for finalizing the operation
* \note Writing is not synchronous. Exetute method only in one task. Do not use 'sendLine' on this channel.
* Method is designed mainly to configure devices after start of the system.
*/
bool UartCAM::sendBinary (const unsigned char* buf, int size)
{
	int i;

    for (i=0; i<size; i++)
	{
#if USE_CAM == CAM_ENABLE
		VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, CAM_WR_FIFO_OFFSET, buf[i]);
#endif
	}
    return true;
}

/** \name Method clears the input buffer
*/
void UartCAM::clearInput (void)
{
    _clearInputFlag = true;
}

/** \name Method handles uC/OS-II operating system task sending line of a text
*/
void UartCAM::taskOut(void* pdata)
{
    INT8U err;
    char *p;
    camControl_t control;
    int cnt;

	// Initialization was unsuccessful - stop the process
    if (initFailed)
    {
        Log.errorPrintf("FastRS_taskOut_1");
        OSTaskSuspend (OS_PRIO_SELF);
    }

    while (true)
    {
		// Waiting for a signal that something has been appended to the 'outLinesBuffer'
        OSMboxPend (mboxSig, 0, &err);
        if (err != OS_NO_ERR)
        {
            Log.errorPrint("FastRS_taskOut_2 [", err, "]");
            // Waiting 10 seconds in order to not clog up an error log
            OSTimeDlyHMSM (0, 0, ERRWAIT, 0);
            continue;
        }

		// Copying due to concurrency
        int lUTmp = lUser;
#if USE_CAM == CAM_ENABLE
		// Sending of an all waiting lines from the cyclic buffer
        while (lSystem != lUTmp)
        {
			// Check if the line can be send
            control.word = VIETTEL_FASTRS_mReadReg(FLYEYE_CAM_BASE, CAM_RD_CONTROL_OFFSET);
            cnt = control.bits.tx_level; // Number of characters not send yet
            if (cnt >= (CAM_BUFFER_SIZE - LINESIZE - 2)) {
                // If there is too much to sent then wait for next sampling
                OSTimeDly (1);  // Sending of an entire buffer takes about 15 ms
                continue;
            }
            int newLSystem = (lSystem + 1) % nLines;
            for (p = outLinesBuffer[newLSystem]; *p !=0; p++)
            	VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, CAM_WR_FIFO_OFFSET, *p);
            // Appending of an end-line characters
            for (p = endline; *p !=0; p++) {
                VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, CAM_WR_FIFO_OFFSET, *p);
            }
            lSystem = newLSystem;
        }
#endif
    }
}

bool UartCAM::getObjectTrackInfor(cam_msg_response_t &msg)
{
    if (!_vSem.lock ())
    {
        return false;
    }
	msg = cam_msg_res;

	if (!_vSem.unlock ())
	{
        return false;
	}

	return true;
}

#if USE_WSM == WSM_ENABLE
bool UartCAM::getWsmData(wsm_msg_res_t &msg)
{
    if (!_vSem.lock ())
    {
        return false;
    }
	msg = wsm_msg_res;

	if (!_vSem.unlock ())
	{
        return false;
	}

	return true;
}

bool UartCAM::parse_wsm(uint8_t temp)
{
    switch (wsm_msg.wsm_state)
    {
    	default:
    	case wsm_msg_parse_t::SYNC1:
    		if(temp == WSM_SYNC1)
    			wsm_msg.wsm_state = wsm_msg_parse_t::SYNC2;
    		wsm_msg.read = 0;
    		break;
    	case wsm_msg_parse_t::SYNC2:
            if (temp == WSM_SYNC2)
            {
            	wsm_msg.wsm_state = wsm_msg_parse_t::SYNC3;
            }
            else
            {
            	wsm_msg.wsm_state = wsm_msg_parse_t::SYNC1;
            }
            break;
    	case wsm_msg_parse_t::SYNC3:
            if (temp == WSM_SYNC3)
            {
            	wsm_msg.wsm_state = wsm_msg_parse_t::DATALENGTH;
            }
            else
            {
            	wsm_msg.wsm_state = wsm_msg_parse_t::SYNC1;
            }
            break;
    	case wsm_msg_parse_t::DATALENGTH:
        	wsm_msg.dataLength = temp;
            wsm_msg.wsm_state = wsm_msg_parse_t::DATA;
            break;
    	case wsm_msg_parse_t::DATA:
            wsm_msg.data.bytes[wsm_msg.read] = temp;
            wsm_msg.read++;
            if (wsm_msg.read >= wsm_msg.dataLength)
            {
            	wsm_msg.wsm_state = wsm_msg_parse_t::CRC;
            }
            break;
    	case wsm_msg_parse_t::CRC:
        	wsm_msg.crc = temp;
        	wsm_msg.wsm_state = wsm_msg_parse_t::SYNC1;

            uint8_t crc = calcCRC(wsm_msg.data.bytes, wsm_msg.read);

            if (wsm_msg.crc == crc)
            {
            	crc_counter++;
    			const wsmMsgBuffer_t &wsmmsg = wsm_msg.data.wsmMsg;
    			wsm_msg_res.magX = wsmmsg.magX;
    			wsm_msg_res.magY = wsmmsg.magY;
    			wsm_msg_res.magZ = wsmmsg.magZ;
    			return true;
            }
            else
            {
                crc_error_counter++;
            }
            break;
    }
    return false;
}
#endif


bool UartCAM::parse_cam(uint8_t temp)
{
    switch (cam_msg.cam_state)
    {
        default:
        case cam_msg_parse_t::SYNC1:
            if (temp == CAM_SYNC1)
            	cam_msg.cam_state = cam_msg_parse_t::SYNC2;
            cam_msg.read = 0;
            break;
        case cam_msg_parse_t::SYNC2:
            if (temp == CAM_SYNC2)
            {
            	cam_msg.cam_state = cam_msg_parse_t::SYNC3;
            }
            else
            {
            	cam_msg.cam_state = cam_msg_parse_t::SYNC1;
            }
            break;
        case cam_msg_parse_t::SYNC3:
            if (temp == CAM_SYNC3)
            {
            	cam_msg.cam_state = cam_msg_parse_t::DATALENGTH;
            }
            else
            {
            	cam_msg.cam_state = cam_msg_parse_t::SYNC1;
            }
            break;
        case cam_msg_parse_t::DATALENGTH:
        	cam_msg.dataLength = temp;
            cam_msg.cam_state = cam_msg_parse_t::DATA;
            break;
        case cam_msg_parse_t::DATA:
            cam_msg.data.bytes[cam_msg.read] = temp;
            cam_msg.read++;
            if (cam_msg.read >= cam_msg.dataLength)
            {
            	cam_msg.cam_state = cam_msg_parse_t::CRC;
            }
            break;
        case cam_msg_parse_t::CRC:
        	cam_msg.crc = temp;
        	cam_msg.cam_state = cam_msg_parse_t::SYNC1;

            uint8_t crc = calcCRC(cam_msg.data.bytes, cam_msg.read);

            if (cam_msg.crc == crc)
            {
            	crc_counter++;
                return process_message_cam();
            }
            else
            {
                crc_error_counter++;
            }
            break;
    }

    return false;
}

bool UartCAM::process_message_cam()
{
	const camMsgBuffer_t &cammsg = cam_msg.data.camMsg;

	cam_msg_res.mode 		= cammsg.mode;
	cam_msg_res.px 			= cammsg.px;
	cam_msg_res.py 			= cammsg.py;
	cam_msg_res.width 		= cammsg.width;
	cam_msg_res.height 		= cammsg.height;
	cam_msg_res.hfov 		= cammsg.hfov;
	cam_msg_res.gimbalPan 	= cammsg.gimbalPan;
	cam_msg_res.gimbalTilt 	= cammsg.gimbalTilt;
	cam_msg_res.gimbalRoll 	= cammsg.gimbalRoll;

	//Send notify to PStateData
	notify(IRQ_CAM_RECEIVED);

	return true;
}

uint8_t UartCAM::calcCRC(uint8_t * data, int length)
{
	short csum = 0;
	for(int i=0; i < length; i++)
	{
		csum += (short)data[i];
	}
	csum = (255-(csum%255));
	return  (uint8_t)csum;
}


#endif // PILOT_TARGET

