/**
*                                                                                     
* @class ChannelData                                                                         
*                                                                                     
*  Class stores data returned from the communication channel.
*                                                                                     
* 2013 Witold Kruczek @ Flytronic                                                     
*/

#ifndef CHANNELDATA_H
#define CHANNELDATA_H


class ChannelData
{
public:
    friend class CommChannel; /* parasoft-suppress  OOP-11 "Friend forbid a. */

    ChannelData (void);                                 ///< Constructor
    explicit ChannelData (const ChannelData& cd);       ///< Konstruktor kopiuj¹cy
    void reset (void);                                  ///< Reset objet data

    unsigned char getSenderNo (void) const;
    unsigned char getFrameType (void) const ;
    const BufferBase& getPayload (void) const ;

    ChannelData& operator= (const ChannelData& src);    ///< Assignment operatora

    static const int DATA_BUF_SIZE = 256;   ///< Buffer size for one element.

    // Field dType value (in accordance with the field PayloadType protocol frame)
    static const unsigned char DT_485 = 0u;
    static const unsigned char DT_TEXT = 1u;
    static const unsigned char DT_UNKNOWN = 255u;

private:
    unsigned char m_senderNo;
    unsigned char m_fType;
    FixBuffer<DATA_BUF_SIZE> m_data;
};

#endif  // CHANNELDATA_H
