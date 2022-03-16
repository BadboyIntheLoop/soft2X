#include <PilotIncludes.h>

CommChannel::CommChannel (IAsyncDevice& device, int nElemsQS, int nElemsQR, bool useLinkLayer, const char* deviceName):
    SerialDeviceBase(),
    m_device(device), m_deviceName(deviceName), m_useLinkLayer(useLinkLayer),
    m_queueS(new DynQueue<QElement, BufferBase>(nElemsQS)),     // parasoft-suppress  JSF-206 "Object will not be released"
    m_evtgQSEmpty(0), m_evtgQSFull(0),
    m_queueR(new DynQueue<ChannelData>(nElemsQR))      // parasoft-suppress  JSF-206 "Object will not be released"
{
	// Semaphore controlling access to the sending queue
    if (!m_semS.create("CommChannel_S"))
    {
        Log.abort ("Critical Error: CommChannel_1.");
        return;
    }

	// Semaphore controlling access to the receiving queue
    if (!m_semR.create("CommChannel_R"))
    {
        Log.abort ("Critical Error: CommChannel_1a.");
        return;
    }

	// Bit flags to handling the sending queue
	// Two groups of flags are required due to restrictions of an OSBase interface
	// For the MicroC/OS-II a single group of flags and two bits (Full/Empty) is sufficient,
	// but it is very difficult to adjust the WindowsNative to such an interface - there is no EventFlags
    int err = 0;
    m_evtgQSEmpty = Os->evtCreate (&err);
    m_evtgQSFull = Os->evtCreate (&err);
    if ((m_evtgQSEmpty == 0) || (m_evtgQSFull == 0))
    {
        Log.abort ("Critical Error: CommChannel_2.");
        return;
    }

	// By default, event flags are reset
	// Setting of a bit, meaning that the queue is not full
	// During waiting on an event it is possibility only to check if bits are set - therefore the logic is reversed
	if (Os->evtPost (m_evtgQSFull, EVT_Q_NOT_FULL, true, &err) != OSBase::EVT_OK)
    {
        Log.abort ("Critical Error: CommChannel_3.");
        return;
    }
}

/** \name Method sends raw data to the buffered communication channel.
* The buffering has been implemented as a DynQueue queue of items (objects) with BufferBase interface.
* The queue is created on a heap (within constructor), and it stores a copies of an objects. After creation it
* do not realocate or release the memory. Method is protected by semaphores.
* \param 'dataToSend' - data to be send
* \param 'suppressErrMsg' - 'true' disables sending of error messages (to prevent recurency when data are send by an error handling mechanism )
* \param 'withWaiting' - 'true' enables waiting for a free space when queue is full (e.g. sending to log)
* \return 'true' on success, 'false' when writing to the queue was unsuccessful or other error occured.
*/
bool CommChannel::sendRawData (const BufferBase& dataToSend, bool suppressErrMsg, bool withWaiting)
{
    int err = 0;
    int putCnt = 0;
    bool bok = false;
    bool bPutOk = false;

    do
    {
        if (withWaiting)
        {
			// Waiting for free space in the queue (without reseting the event flag)
            OSBase::EvtMask f = 0u;
            Os->evtWaitAny (m_evtgQSFull, f, &err, false);

			// In the queue there is enough space by other thread can it use, so writing may not be successful
        }

        if (!m_semS.lock (suppressErrMsg))
            return false;

		// Writing data to the queue. When queue is full 'false' is returned
        bPutOk = m_queueS->put (&dataToSend);

		// Event flag indicating that there is space in the queue is reset when write
		// to the queue was unsuccessful (queue was full) (rare case).
        if (!bPutOk)
            Os->evtPost (m_evtgQSFull, EVT_Q_NOT_FULL, false, &err);

		// Counter of a trying to write to the queue
        putCnt++;

        m_semS.unlock ();

		// Loop continues when:
		//	- sending with waiting has been chosen
		//	- and last write to the queue was unsuccessful (queue was full)
		//	- and the limit for trying to write was not reached
    } while (withWaiting && (!bPutOk) && (putCnt < PUT_MAX_TRY));

    if (bPutOk)
        // Setting the flag indicating that queue is not empty on writting success
		if (Os->evtPost (m_evtgQSEmpty, EVT_Q_NOT_EMPTY, true, &err) == OSBase::EVT_OK)
            bok = true;

    return bok;
}

/** \name Method sends data wrapped with a frame of link layer protocol.
* Method is intended to support devices connected to the RS485 bus.
* \param 'dataToSend' - data to be send
* \param 'devTo'- identifier of a destination device
* \param 'devFrom' - identifier of a source device
* \return 'true' on success, 'false' otherwise
*/
bool CommChannel::sendData485 (const BufferBase& dataToSend, int devTo, int devFrom, int& debugMsg)
{
    if ((devTo < 0) || (devTo > 16) || (devFrom < 0) || (devTo > 16))
        return false;

    FrameBuf frame;

	// Frame identifier
    bool bok = frame.concatB (P485Id);
    // Address of a device
    bok = bok && frame.concatB (static_cast<unsigned char>(devFrom *16  + devTo));
    // Data
    bok = bok && frame.concat (dataToSend);
    // Appending of an CRC
    bok = bok && addFrameCRC (frame);
    // Escaping
    bok = bok && escapeFrame (frame);
    // Appending of a frame termination character
	bok = bok && frame.concatB (FRAME_END_BYTE);

    // Sending
    bok = bok && sendRawData (frame, false, false);
    
	if (debugMsg == RS485_SEND_DATA)
    {
        debugMsg = NONE;
        Log.msgPrintf ("%s", frame.getData());
    }
    
    return bok;
}

/** \name Method reads data from communication channel (without waiting).
* Data are grouped in frames and inserting to the queue of frames.
* The queue is created on the heap (within constructor), and it stores a copies of an objects.
* After creation it do not realocate or release the memory. Method is protected by semaphores.
* \param 'dataReceived' - reference to an object by which data will be returned
* \return 'true' on success, 'false' when queue was full or other error has occured
*/
bool CommChannel::getData (ChannelData& dataReceived)
{
    if (!m_semR.lock ())
        return false;

	// Getting data frame from queue (object copying)
    bool err = m_queueR->get (dataReceived);

    m_semR.unlock ();

    return err;
}

/** \name Dummy method which always returns an error. Do not use it.
*/
bool CommChannel::getLastLine (char* lineReceived)
{
    lineReceived[0] = '\0';
    Log.errorPrint("CommChannel_getLastLine_1");
    return false;
}

/** \name Method sends text line to the buffered communication channel.
* Method is similar to the 'sendData' method - it puts the text line to the BufferBase object and executes 'sendData' method.
*/
bool CommChannel::sendLine (const char* lineToSend, bool suppressErrMsg, bool withWaiting, bool withCrc)
{
    if (lineToSend == NULL)
        return false;

    QElement sb;

	// Optional adding a CRC sum at the beginning of a line
    if (withCrc)
    {
        char crcBuf[CRC_BUF_SIZE];
        crcBuf[0] = '\0';
        Crc::compute(lineToSend, crcBuf);
        sb.concatS (crcBuf);
    }

    sb.concatS (lineToSend);
	// Appending of a end line characters
    sb.concatS ("\r\n");
    sendRawData (sb, suppressErrMsg, withWaiting);
    return true;
}

/** \name Method handles tasks of an operating system for receiving data
*/
void CommChannel::taskIn(void* pdata)
{
    while(true)
    {
//    	OSTimeDlyHMSM(0, 0, 2, 0);
#if PILOT_TARGET == PT_HARDWARE
    	usleep(1000000);
#endif
    	Log.msgPrintf("CommChannel TaskIn\r\n");
    }
}

/** \name Method handles tasks of an operating system for sending data
*/
void CommChannel::taskOut(void* pdata)
{
    while(true)
    {
//    	OSTimeDlyHMSM(0, 0, 2, 0);
#if PILOT_TARGET == PT_HARDWARE
    	usleep(1000000);
#endif
    	Log.msgPrintf("CommChannel TaskOut\r\n");
    }
}

/** \name Method processes the received complete frame of an link layer protocol.
* After processing the payload processing method is executed.
* If value of a first byte is less then 128 then this is not a protocol frame but a text line.
* \param 'frame' - protocol frame (without termination character 'LF')
* \param 'cd' - the output data to be set
* \return 'true' on success, 'false' otherwise
*/
bool CommChannel::processFrame (FrameBuf& frame)
{
    const unsigned char* fPtr = frame.getData();

	// Checking if it is a frame or a text line
    if (fPtr[0] <= ASCII_LAST)
    {
		// A text line
        processPayload (fPtr, frame.getBufferUsed(), 0u);
    }
    else if (P485Id == fPtr[0])
    {
		// Link layer protocol
		// Unescaping of a frame bytes
        unEscapeFrame (frame);

		// Frame CRC summ checking
        if (!checkFrameCRC (frame))
        {
            Log.msgPrintf ("%s: Bad Link Layer CRC", m_deviceName.getData());
            return false;
        }

		// Payload extracting:
		//	- skipping leading fields: protocol type (1B) and address (1B)
		//	- skipping trailing CRC field (2B)
        processPayload (&fPtr[2], frame.getBufferUsed()-4, static_cast<unsigned char>(fPtr[1] >> 4u));
    }
    else
    {
		// Unknown protocol
        Log.msgPrintf ("%s: Bad Link Layer Protocol Id", m_deviceName.getData());
        return false;
    }
    
    return true;
}

/** \name Method processes a payload's data field. After processing it executes method that writes that separated data.
* If first byte is in range of <10;127> then this is interpreted as a text line and this byte is its starting byte.
* Otherwise first byte describes the content of a field (byte is not a part of this content)
* \param  'rbuf' - buffer with payload field
* \param 'bytesReceived' - number of bytes inside this buffer
* \param 'senderNo' - identifier of the device that sent the data
*/
bool CommChannel::processPayload (const unsigned char* rbuf, int bytesReceived, unsigned char senderNo)
{
    if (bytesReceived <= 0)
        return false;

	// Obtaining of a working item
	// Queue should have enough space. If there is no space in the queue then working item will not be available
	// and it is necessary to wait until another subsystem empties the queue.
	// At this time, data may be lost from the device.
    ChannelData* cd = NULL;
    while (true)
    {
        cd = m_queueR->work ();
        if (cd != NULL)
            break;
        Log.errorPrint("CommChannel_processPayload_1");
        Os->sleepMs (WRK_WAIT_MS);
    }

	// Reseting of a obtained (new) working item
    cd->reset ();
    
	// Device number is used only for RS485 devices
    cd->m_senderNo = senderNo;
    int dataBeg = 0;    // Beginning of a data in buffer

    if (rbuf[0] <= ASCII_LAST)
    {
        // Text line
        cd->m_fType = ChannelData::DT_TEXT;
    }
    else
    {
		// Data of a PayloadType type
        cd->m_fType = rbuf[0];
		// Skipping the first byte, because it is not a part of a data
        dataBeg = 1;
    }

	// Skipping the CR and LF characters at the end of a buffer, if buffer contains a text
    if (ChannelData::DT_TEXT == cd->m_fType)
        while (((bytesReceived - dataBeg) > 0) && ((ASCII_LF == rbuf[bytesReceived-1]) || (ASCII_CR == rbuf[bytesReceived-1])))
        {
            --bytesReceived;
        }

	// After erasing the CR and LF characters the data are empty, and method returns
    if (bytesReceived <= 0)
        return false;

	// Copying the data to the resulting buffer
	// In case of error or truncation method returns
    if (!cd->m_data.concatBytes (&rbuf[dataBeg], bytesReceived-dataBeg))
        return false;

    if (!m_semR.lock (true))
        return false;

	// Putting the frame prepared in the working item to the queue
	// No error checking because if queue wasn't full while obtainig the working item then now it will not be
	// No error checking, because if the queue was not full when obtaining a work item so and now it will not be
    m_queueR->putWork ();

    m_semR.unlock ();

	// Sending notification to the subsystem receiving data
    notify (CH_DATA_RECEIVED);

    return true;
}

/** \name Method verifies the frame checksum
*/
bool CommChannel::checkFrameCRC (const FrameBuf& frame) const
{
    int len = frame.getBufferUsed();

    if (len <= 2)
        return false;

    // Calculation of a frame checksum without two last bytes (in which there is a checksum calculated by sender)
    INT16U crc = Crc::computeBin (frame.getData(), len-2);

    bool bok = (((crc >> 8u) & 0xFFu) == frame.getData()[frame.getBufferUsed()-2]);
    bok = bok && ((crc & 0xFFu) == frame.getData()[frame.getBufferUsed()-1]);

    return bok;
}

/** \name Method appends checksum to the end of a frame
*/
bool CommChannel::addFrameCRC (FrameBuf& frame) const
{
    INT16U crc = Crc::computeBin (frame.getData(), frame.getBufferUsed());

    bool bok = frame.concatB (static_cast<unsigned char>((crc >> 8u) & 0xFFu));
    bok = bok && frame.concatB (static_cast<unsigned char>(crc & 0xFFu));

    return bok;
}

/** \name Method puts before each byte of proprietary value a special byte ESC_BYTE, and same byte is xoring with ESC_XOR_BYTE
*/
bool CommChannel::escapeFrame (FrameBuf& frame) const
{
    FrameBuf newFrame;
    int len = frame.getBufferUsed();
    const unsigned char* ptr = frame.getData();
    bool bok = true;

    for (int i = 0; i < len; i++)
    {
		// Escaping of an ending frame byte and same escaping byte
        if ((FRAME_END_BYTE == ptr[i]) || (ESC_BYTE == ptr[i]))
        {
            bok = bok && newFrame.concatB (ESC_BYTE);
			// After the escaping byte, source byte is also xored to do not occur anywhere in the data output
            bok = bok && newFrame.concatB (static_cast<unsigned char>(ptr[i] ^ ESC_XOR_BYTE));
        }
        else
            bok = bok && newFrame.concatB (ptr[i]);
    }

    // Copying the new frame in place of the old
    if (bok)
        frame = newFrame;

    return bok;
}

/** \name Method restores the original data which, for the purpose of transmission, has coded bytes with proprietary values.
* Coding involves putting before restricted byte the value of ESC_BYTE and xoring same byte by ESC_XOR_BYTE value.
* \param 'frame' - data frame (no matter what kind of and what it contains)
*/
bool CommChannel::unEscapeFrame (FrameBuf& frame) const
{
    unsigned char* fPtr = const_cast<unsigned char*>(frame.getData()); // parasoft-suppress  MISRA2008-5_2_5 "Wyj¹tkowo zdejmujemy const"
    int fBytes = frame.getBufferUsed();
    unsigned char esc = 0u;

	// NOTE: Omitting of a first byte (frame identifier)
    int j = 1;
    for (int i=1; i < fBytes; i++)
    {
        if ((ESC_BYTE != fPtr[i]) || (0 != esc))
        {
			// Copying character if:
			//  - the escaping mode is enabled and additionally the character is xored
			//  - the escaping mode is disabled and current character is not an escaping character
            fPtr[j] = static_cast<unsigned char>(fPtr[i] ^ esc);
            j++;

			// After copying a character the escaping mode is disabling (escaping mode was valid for a single character)
            esc = 0u;
        }
        else
        {
			// Enabling the escaping mode
            esc = ESC_XOR_BYTE;
        }
    }

	// Correction of a number of important characters in the buffer
    frame.rawSetBufferUsed (j);
    fPtr[j] = 0;

	// If the end of data has been reached and escaping mode is enabled that's mean that an format error has occurred
    return (0 == esc);
}
