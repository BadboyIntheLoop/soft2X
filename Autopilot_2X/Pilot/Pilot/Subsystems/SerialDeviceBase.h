#ifndef SERIALDEVICEBASE_H
#define SERIALDEVICEBASE_H

class ChannelData;

/** \file
* \brief Base class for classes implementing serial devices handling communication channels
*/

/** Base class for classes implementing communication channels like Gps, Tty0, Tty1, etc. 
* Class implements basic functionality to handle a UDP ports and a serial devices like read and write data of a communication device
* and formating its output text lines. Class notifies the observers to read the line of text when the end of line sign has been read.
* Class allows sending a line of text to the device asynchronously (without caching). This class is universal for Win32 and NIOS-II.
*/
/// Base class for classes implementing serial devices handling communication channels
class SerialDeviceBase : public ODTSubject
{
public:
    virtual bool getLastLine(char* lineReceived) = 0; ///< Method gets last received line of text from the device (asynchronously). End line marks are omitted. Line is terminated with '\0'.
	virtual bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false, bool withCrc = false) = 0; ///< Method sends a line of text. Line must be terminated with '\0'.
    virtual bool sendBinary (const unsigned char* buf, int size);	///< Method sends binary data.
    virtual void clearInput (void);	///< Method erases content of an input buffer.
    virtual void enable (bool en); ///< Method enables or disables channel.

    virtual void taskIn(void* pdata) = 0;	///< Method handles receiving of a data within MicroC/OS-II system
    virtual void taskOut(void* pdata) = 0;	///< Method handles sending data within MicroC/OS-II system
  
    virtual void linkObserver(void);

    virtual bool getData (ChannelData& dataReceived);	///< Default implementation for channel data getter method.
  
protected:
    // Constructor is disabled
    explicit SerialDeviceBase(bool enable=true);
    // Destructor is disabled to prevent releasing of an object by pointer to its base class.
	// Parasoft-suppress OOP-31, NIOS-2 environment report an warning while destructor is not a virtual.
    virtual ~SerialDeviceBase(void){};

	bool isInitFailed (void);	///< Method checks if the initialization is not successful

    bool initFailed;    ///< Object initialization error flag
    bool enabled;       ///< Channel activation flag
    char endline[3];    ///< End-line characters

private:
	// Copy constructor is disabled
    SerialDeviceBase(SerialDeviceBase&);
	// Assignment operator is disabled
    SerialDeviceBase& operator=(const SerialDeviceBase&);
};

#endif //SERIALDEVICEBASE_H
