#ifndef COMMCHANNEL_H
#define COMMCHANNEL_H

class IAsyncDevice;

/** \file
* \brief Declartion of a bidirectional comminication channel for binary and text data serial devices
*/

/** \name Class implements bidirectional comminication channel for binary and text data serial devices (e.g. UART, UDP, Console).
* Class uses driver to the device which is compatible with IAsyncDevice interface. Data to be sent are buffered in a queue of size
* specified by an constructors argument. Received data are also buffered. Class uses two system threads, one per receiver and transmitter.
* Inheritance from SerialDeviceBase is temporary for easier collaboration with other subsystems which not need to use devices based
* on CommChannel class in a different way.
*/
/// Class implements bidirectional comminication channel for binary and text data serial devices
class CommChannel: public SerialDeviceBase
{
public:
    CommChannel (IAsyncDevice& device, int nElemsQS, int nElemsQR, bool useLinkLayer, const char* deviceName);

	/// Method sends data without wrapping with protocol frame
    virtual bool sendRawData (const BufferBase& dataToSend, bool suppressErrMsg, bool withWaiting);
    /// Method sends data wrapped with protocol frame of connection layer
    virtual bool sendData485 (const BufferBase& dataToSend, int devTo, int devFrom, int& debugMsg);
	/// Method reads data
    virtual bool getData (ChannelData& dataReceived);

	// Interface for compliance with the SerialDeviceBase

    /// Dummy method which always returns an error
    virtual bool getLastLine (char* lineReceived);
	/// Method sends the line which must be terminated with '\0'
    virtual bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false, bool withCrc = false);
   
	/// Method handles tasks of an operating system for receiving data
    virtual void taskIn (void* pdata);
	/// Method handles tasks of an operating system for sending data
    virtual void taskOut (void* pdata);
	
    static const int ELEM_BUF_SIZE  = 256;   ///< Buffer size for single element
    static const int FRAME_BUF_SIZE = 256;   ///< Frame buffer size

protected:
	/// Destructor is disabled (object should never be destroyed)
    virtual ~CommChannel(void) {};

private:
    typedef FixBuffer<ELEM_BUF_SIZE> QElement;      ///< Type of an element stored in the queue
    typedef FixBuffer<FRAME_BUF_SIZE> FrameBuf;     ///< Frame buffer

	/// Copy constructor is disabled
    CommChannel (const CommChannel&);

    bool processFrame (FrameBuf& frame);
    bool processPayload (const unsigned char* rbuf, int bytesReceived, unsigned char senderNo);
    bool checkFrameCRC (const FrameBuf& frame) const;
    bool addFrameCRC (FrameBuf& frame) const;
    bool escapeFrame (FrameBuf& frame) const;
    bool unEscapeFrame (FrameBuf& frame) const;

	/// Copy operator is disabled
    CommChannel& operator= (const CommChannel&);

    static const int CRC_BUF_SIZE  =  20;   ///< CRC calculations buffer size
    static const int PUT_MAX_TRY   =  10;   ///< Number of attempt to write to the queue of waiting
    static const int ERR_WAIT_MS   = 100;   ///< Time [ms] of loop locking which reads the queue if an error occurred
    static const int WRK_WAIT_MS   =  10;   ///< Waiting time [ms] for item from queue
    static const int RCV_BUF_SIZE  =  50;   ///< Size of the buffer to read data from the device

	/** \name Bit flags of events queue ('m_evth' events group)
	* \{
	*/
    static const int EVT_Q_NOT_EMPTY = 1;
    static const int EVT_Q_NOT_FULL  = 2;
	///\}

    static const unsigned char P485Id       = 0x80u;    ///< P485 protocol identifier in the link layer frame
    static const unsigned char ASCII_LF     =   10u;    ///< LF character ('\n')
    static const unsigned char ASCII_CR     =   13u;    ///< CR character ('\r')
    static const unsigned char ASCII_LAST   =  127u;    ///< CR character ('\r')
    static const unsigned char ESC_BYTE     = 0xDCu;    ///< Escaping byte
    static const unsigned char ESC_XOR_BYTE = 0x80u;    ///< Escaping byte
    static const unsigned char FRAME_END_BYTE = 10u;    ///< End of frame byte

    IAsyncDevice& m_device;                         ///< I/O device driver handled by this communication channel
    const FixBuffer<20> m_deviceName;               ///< Device name
    const bool m_useLinkLayer;                      ///< Flag enabling usage of a link layer (For RS232,RS485 true. For UDP false)

    DynQueue<QElement, BufferBase>* m_queueS;       ///< Pointer to the queue of items to be send
    Semaphore m_semS;                               ///< Semaphore controling access to the queue of items to be send
    OSBase::EvtHandle m_evtgQSEmpty;                ///< Handle to the read events group that supports the sending queue
    OSBase::EvtHandle m_evtgQSFull;                 ///< Handle to the write events group that supports the sending queue

    /** Pointer to the queue of received items (and completed)
	* The queue of received items has no waiting mechanism so it is not synchronized by an events.
	*/
	DynQueue<ChannelData>* m_queueR;
    Semaphore m_semR;                               ///< Semaphore controlling access to the queue of received items
};

#endif // COMMCHANNEL_H
