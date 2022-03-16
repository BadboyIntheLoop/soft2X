/**
*                                                                                     
* @class ChannelData                                                                         
*                                                                                     
*  Class stores data returned from the communication channel.
*                                                                                     
* 2013 Witold Kruczek @ Flytronic                                                     
*/

#include <PilotIncludes.h>


/**
* Constructor
*/
ChannelData::ChannelData (void):
m_senderNo(0u), m_fType(DT_UNKNOWN)
{}


/**
* Copy constructor
*/
ChannelData::ChannelData (const ChannelData& cd):
m_senderNo(cd.m_senderNo), m_fType(cd.m_fType), m_data(cd.m_data)
{}


/**
* Reset object data.
*/
void ChannelData::reset (void)
{
    m_senderNo = 0u;
    m_fType = DT_UNKNOWN;
    m_data.reset ();
}


/**
* Returns the device number sending data (important only for RS485)
*/
unsigned char ChannelData::getSenderNo (void) const
{
    return m_senderNo;
}


/**
* Return frame type.
* If the frame did not have byte of the type return value corresponding to the text data.
*/
unsigned char ChannelData::getFrameType (void) const
{
    return m_fType;
}


/**
* Return frame contents (without type byte if it was used in that frame)
*/
const BufferBase& ChannelData::getPayload (void) const
{
    return m_data;
}


/**
* Assigment operator.
*/
ChannelData& ChannelData::operator= (const ChannelData& src)
{
    // if it is the same object then skip function.
    if (this != &src)
    {
        m_senderNo = src.m_senderNo;
        m_fType = src.m_fType;
        m_data = src.m_data;
    }
    return *this;
}
