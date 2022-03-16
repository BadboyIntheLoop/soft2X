#ifndef IASYNCDEVICE_H
#define IASYNCDEVICE_H

/** \file
* \brief Declaration of an iterface class to the asynchronous devices
*/

/** Class implements asynchronous device (UART, UDP) interface for sending and receiving a binary data.
* Class is compatible with all platforms. None of the member functions MUST NOT support multi-threading.
*/
/// Class implements an interface to the asynchronous devices
class IAsyncDevice
{
public:
	/** \name Method sends binary data to the device.
	* Method does not break current transmission and does not destroy the data.
	* Before sending method may waits on finalization of previous transmission or appends data to it.
	* Before returning method may wait on finalizing of the current transmission. 
	* Sending the data is not equivalent to their instant appearance on the transmission medium because data can be queued.
	* Method may return with an error after timeout.
	* \param 'buf' - buffer with data to send
	* \param 'nBytes' - number of bytes to send (0-n)
	* \return 'true' on success, 'false' on error
	*/
    virtual bool sendBufferWait (const unsigned char* buf, int nBytes) = 0;

	/** \name Method receives binary data from device.
	* Method waits witout time linit to buffer overflow or information about the end of the data packet
	* \param 'buf' - buffer to which to store the data
	* \param 'bufSize' - buffer size (>0)
	* \param 'nBytes' - number of read bytes
	* \return 'true' on success, 'false' on error
	*/
    virtual bool recvBufferWait (unsigned char* buf, int bufSize, int& nBytes) = 0;

protected:
	// Default constructor is disabled
    IAsyncDevice(void) {};
    // Destructor is disabled (releasing the object by pointer to its base class)
    virtual ~IAsyncDevice(void) {};

private:
    // Copy constructor is disabled
    IAsyncDevice(IAsyncDevice&);
	// Copy operator is disabled
    IAsyncDevice& operator=(const IAsyncDevice&);
};

#endif  // IAsyncDevice
