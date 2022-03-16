/**
*                                                                   
* @class GpsData                                                           
*                                                                   
* Stores data that has been read from GPS (position, speed etc)
* Parsing NMEA sequence, normalize obtained data.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef GPSDATA_H
#define GPSDATA_H

class GpsData
{
public:
    enum NavModeCode
    {
        NAV_MODE_UNKNOWN       = 0,
        NAV_MODE_NO_VALID_DATA = 1,
        NAV_MODE_2D            = 2,
        NAV_MODE_3D            = 3,
    };

    GpsData(void);

    void resetData (void);
    bool parseNmea (const char* nmeaLine);
    bool predict (const GpsData &prev, const GpsData &curr, int predTime);

    GpsPosition  position;
    float        amsl;               ///< [m]
    float        groundSpeed;        ///< [kph] - NOTE: RMC is in kntos
    float        track;              ///< [degrees]
    int          utcHour;            ///< hours UTC
    int          utcMin;             ///< minutes UTC
    int          utcSec;             ///< seconds UTC
    int          utcMsec;            ///< miliseconds UTC
    int          day;                ///< number of day in the month
    int          month;              ///< month number
    int          year;               ///< 2 last digits of the year
    float        hDop;               ///< horizonatl accuracy [m]
    float        vDop;               ///< vertical accuracy [m]
    NavModeCode  navMode;            ///< 1-lack of data, 2- 2D mode, 3- 3D mode
    bool         amslPresent;        ///< amsl has non empty value (important in mode navMode==3)
    bool         trackPresent;       ///< track has non empty value 
    bool         datePresent;        ///< date has non empty value 
    bool         timePresent;        ///< time has non empty value 
    bool         parseError;         ///< Parse error occered all lines are invalid
    int          userTime;           ///< Time when user made reading [*100us]

private:
    bool parseGpsTime (const char* time);
    bool parseGpsDate (const char* date);
    bool parseGpsPosition (const char* lat, const char* latSign,
        const char* lon, const char* lonSign);
    bool parseGpsSpeed (const char* speed);
    bool parseGpsTrack (const char* track);
    bool parseGpsAltitude (const char* alt);
    bool parseGpsNavMode (const char* nm);
    bool parseGpsHDop (const char* hDopStr);
    bool parseGpsVDop (const char* vDopStr);
};

#endif  // GPSDATA_H
