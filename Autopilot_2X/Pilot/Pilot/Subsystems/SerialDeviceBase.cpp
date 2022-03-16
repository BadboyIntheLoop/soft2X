#include "PilotIncludes.h"

SerialDeviceBase::SerialDeviceBase (bool enable)
{
    initFailed = false;
    enabled = enable;
    endline[0] = '\r';
    endline[1] = '\n';
    endline[2] = '\0';
}

//  Czy inicjalizacja siê uda³a
/** \name Method check if initialization was successful or not.
* \return 'true' if initialization was unsuccessful, 'false' otherwise.
*/
bool SerialDeviceBase::isInitFailed (void)
{
    return initFailed;
}

/** \name Method of the derived class sends binary data. Default implementation (of virtual method) is empty.
*/
bool SerialDeviceBase::sendBinary (const unsigned char* buf, int size)
{
    return false;
}

/** \name Method of the derived class clears the input buffer. Default implementation (of virtual method) is empty.
*/
void SerialDeviceBase::clearInput (void)
{
}

/** \name Method toggles the channels activity
* \param Passing 'true' as argument activates the channel, 'false' deactivates it.
*/
void SerialDeviceBase::enable (bool en)
{
    enabled = en;
}

void SerialDeviceBase::linkObserver (void)
{
}

/** \name Method implements default channel data getter.
* Appropriate functionality is implemented in the CommChanel class.
*/
bool SerialDeviceBase::getData (ChannelData& dataReceived)
{
    return false;
}
