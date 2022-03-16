/**
*                                                                  
* @class Ublox                                                             
*                                                                   
* @brief Static function fo GPS ublox.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef UBLOX_H
#define UBLOX_H

class SerialDeviceBase;

class Ublox
{
public:   
    /// Create binary package incuding binary GPS command.
    static bool prepareUbxPacket (unsigned char cl, unsigned char id,
                                 const unsigned char* payload, int payloadSize,
                                 unsigned char* outBuf, int &outBufSize);
                                 
    /// ON/OFF sending NMEA command with specified cl and id.
    static bool setNmea (unsigned char cl, unsigned char id, bool enable,
                         unsigned char* outBuf, int &outBufSize);

    /// Set measurements interval (miliseconds)
    static bool setInterval (unsigned int msec,
                             unsigned char* outBuf, int &outBufSize);
#if GPS_TYPE == UBLOX_8
    static bool setGpsFilter (unsigned char* outBuf, int &outBufSize);
#endif
    /// Set dynamic platform (0-8) 
    static bool setDynPlatform (int platform,
                                unsigned char* outBuf, int &outBufSize);

    ///  Set SBAS
    static bool setSBAS (bool enable,
                            unsigned char* outBuf, int &outBufSize);

    /// Module reset
    static bool reset (unsigned char* outBuf, int &outBufSize);

    /// If it is a first line in NMEA in package of lines
    static bool isPacketBegin (const char* nmeaLine);

    /// If it is a last line in NMEA in package of lines
    static bool isPacketEnd (const char* nmeaLine);
        
    /// Send sequence to reset GPS module (restore default settings)
    static bool pilotReset (SerialDeviceBase* dev);

    /// Send all needed initializations for autopilot to the GPS communication chanel.
    static bool pilotInit (SerialDeviceBase* dev, bool useSBAS);

private:    
    Ublox(void);
};

#endif //UBLOX_H
