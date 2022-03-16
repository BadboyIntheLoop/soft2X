/**
*                                                                  
* @class Ublox                                                             
*                                                                   
* @brief Static function fo GPS ublox.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>


/**
* Create binary package incuding binary GPS command.
* /param cl - command class
* /param id - command ID
* /param payload - command text (binarry)
* /param payloadSize - text size
* /param outBuf - output buffer
* /param outBufSize - size of output buffer
* Returns true when ok
*/
bool Ublox::prepareUbxPacket (unsigned char cl, unsigned char id,
                              const unsigned char* payload, int payloadSize,
                              unsigned char* outBuf, int &outBufSize)
{
    //  buffer size control
    if (outBufSize < payloadSize + 8)
    {
        Log.errorPrintf("Ublox_prepareUbxPacket_1");
        return false;
    }

    unsigned int ups = static_cast<unsigned int> (payloadSize);
        
    outBuf[0] = 0xb5;   // sync char 1
    outBuf[1] = 0x62;   // sync char 2
    outBuf[2] = cl;     // class
    outBuf[3] = id;     // id
    outBuf[4] = static_cast<unsigned char>(ups & 0xff);          // little endian
    outBuf[5] = static_cast<unsigned char>((ups >> 8) & 0xff);
    memcpy (outBuf+6, payload, ups); // payload    
    
    unsigned char ck_a = 0, ck_b = 0;
    
    for (int i = 0; i < payloadSize + 4; i++)
    {
        ck_a += outBuf[i+2];
        ck_b += ck_a;
    }
    
    outBuf[payloadSize+6] = ck_a;   // control sum
    outBuf[payloadSize+7] = ck_b;
    
    outBufSize = payloadSize + 8;

    return true;
}
                                    

/**
* ON/OFF sending NMEA command with specified cl and id.
*/
bool Ublox::setNmea (unsigned char cl, unsigned char id, bool enable,
                     unsigned char* outBuf, int &outBufSize)
{
    unsigned char buf[3];
    buf[0] = cl;
    buf[1] = id;
    buf[2] = enable ? 1:0;
    
    //  CFG-MSG command
    return prepareUbxPacket (0x06, 0x01, buf, sizeof(buf), outBuf, outBufSize);
}


/**
* Set measurements interval (miliseconds)
*/
bool Ublox::setInterval (unsigned int msec,
                         unsigned char* outBuf, int &outBufSize)
{
    unsigned char buf[6];
    buf[0] = static_cast<unsigned char>(msec & 0xff);
    buf[1] = static_cast<unsigned char>((msec >> 8) & 0xff);
    buf[2] = 1;
    buf[3] = 0;
    buf[4] = 1;
    buf[5] = 0;

    //  CFG-RATE command
    return prepareUbxPacket (0x06, 0x08, buf, sizeof(buf), outBuf, outBufSize);
}

#if GPS_TYPE == UBLOX_8
/**
* Set config gps filter
*/
bool Ublox::setGpsFilter(unsigned char* outBuf, int &outBufSize)
{
    unsigned char buf[12];
    buf[0] = 0x00;
    buf[1] = 0x23;
    buf[2] = 0x00;
    buf[3] = 0x00;
    
    buf[4] = 0xFC;
    buf[5] = 0xFF;
    buf[6] = 0xFF;
    buf[7] = 0xFF;
    
    buf[8] = 0x00;
    buf[9] = 0x01;
    buf[10]= 0x01;
    buf[11]= 0x00;

    //  CFG-NMEA command
    return prepareUbxPacket (0x06, 0x17, buf, sizeof(buf), outBuf, outBufSize);
}
#endif
   
/**
* Set dynamic platform (0-8) 
*/
bool Ublox::setDynPlatform (int platform,
                            unsigned char* outBuf, int &outBufSize)
{
    unsigned char buf[36] = {0};
    buf[0] = 1;                 // mask  - change to dynModel
    buf[1] = 0;
    buf[2] = static_cast<unsigned char>(platform & 0x0f);   // dynModel

    //  CFG-NAV5 command
    return prepareUbxPacket (0x06, 0x24, buf, sizeof(buf), outBuf, outBufSize);
}


/**
* Set SBAS 
*/
bool Ublox::setSBAS (bool enable,
                            unsigned char* outBuf, int &outBufSize)
{
    unsigned char buf[8] = {0};
    buf[0] = enable ? 3 : 0;    //  enabled + test
    buf[1] = enable ? 7 : 0;    //  range + diffCorr + integrity
    buf[2] = 3;                 //  max number of SBAS

    // CFG-SBAS command
    return prepareUbxPacket (0x06, 0x16, buf, sizeof(buf), outBuf, outBufSize);
}


/**
* Reset module
*/
bool Ublox::reset (unsigned char* outBuf, int &outBufSize)
{
    unsigned char buf[4] = {0};
    buf[2] = 0;                 // Hardware Reset

    //  CFG-RST command
    return prepareUbxPacket (0x06, 0x04, buf, sizeof(buf), outBuf, outBufSize);
}


/**
*  If it is a first line in NMEA in package of lines
*/
bool Ublox::isPacketBegin (const char* nmeaLine)
{
    return (STRNICMP (nmeaLine, "$GPRMC", 6) == 0);
}


/**
* If it is a last line in NMEA in package of lines
*/
bool Ublox::isPacketEnd (const char* nmeaLine)
{
    return (STRNICMP (nmeaLine, "$GPGSA", 6) == 0);
}

/**
* Send sequence to reset GPS module (restore default settings)
*/
bool Ublox::pilotReset (SerialDeviceBase* dev)
{
    unsigned char buf[20];

    //  Soft Reset - restore default settings
    int bs = sizeof(buf);
    if (reset (buf, bs))
    {
        //  bs include now command size
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotReset_1");
                return false;
            }
    }      
    
    return true;   
}


/**
* Send all needed initializations for autopilot to the GPS communication chanel.
*/
bool Ublox::pilotInit (SerialDeviceBase* dev, bool useSBAS)
{
    unsigned char buf[50];
            
    //  Turn off GSV command
    int bs = sizeof(buf);
    if (setNmea (0xf0, 0x03, false, buf, bs))
    {
        //  bs include now command size
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotInit_1");
                return false;
            }
    } 

    //  Turn off GLL command
    bs = sizeof(buf);

    if (setNmea (0xf0, 0x01, false, buf, bs))
    {
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotInit_2");
                return false;
            }
    }
     
    //  Turn of VTG command
    bs = sizeof(buf);

    if (setNmea (0xf0, 0x05, false, buf, bs))
    {
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotInit_3");
                return false;
            }
    }

    //  Set 4 Hz
    bs = sizeof(buf);

    if (setInterval (250, buf, bs))
    {
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotInit_4");
                return false;
            }
    }

#if GPS_TYPE == UBLOX_8
    //  Set config gps filter
    bs = sizeof(buf);

    if (setGpsFilter (buf, bs))
    {
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotInit_5");
                return false;
            }
    }
#endif

    //  Set SBAS
    bs = sizeof(buf);

    if (setSBAS (useSBAS, buf, bs))
    {
        if (dev != NULL)
            if (!dev->sendBinary (buf, bs))
            {
                Log.errorPrintf("Ublox_pilotInit_6");
                return false;
            }
    }
    
    return true;
}
