/**
*                                                                  
* @class PstateHealth                                                      
*                                                                    
* Class representing data shared by the subsystem based on the PhysicalState class.                                   
* It is used as the interface beetwen susbsystems.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef PSTATEHEALTH_H
#define PSTATEHEALTH_H

class PStateHealth
{
public:
    PStateHealth(void):
        gpsError(gpsErrorCode::ERR_GPS_NO_OUTPUT),
#if USE_DGPS == 1
		 dgpsError(ERR_DGPS_NO_OUTPUT), 
#endif
		 sensorsError(sensorsErrorCode::ERR_SENSORS_NOT_READY)
    {};

    ///  GPS Errors
    enum class gpsErrorCode
    {
        ERR_GPS_OK = 0,             ///< No errors.
        ERR_GPS_NO_OUTPUT = 1,      ///< Lack of NMEA sequence from GPS (after system start)
        ERR_GPS_2D_MODE = 2,        ///< 2D Mode
        ERR_GPS_BAD_OUTPUT = 3,     ///< wrong NMEA sequence
        ERR_GPS_NO_VALID_DATA = 4,  ///< NMEA data out of date
        ERR_GPS_BAD_PRECISION = 5,  ///< Precision less than demanded 
		ERR_GPS_JAMMING_POSITION = 6,
    };
#if USE_DGPS == 1
	///  DGPS Errors
    enum class dgpsErrorCode
    {
        ERR_DGPS_OK            = 0,  ///< No errors.
        ERR_DGPS_NO_OUTPUT     = 1,  ///< Lack of NMEA sequence from DGPS (after system start)
        ERR_DGPS_NO_VALID_DATA = 2,  ///< NMEA data out of date
        ERR_DGPS_BAD_OUTPUT    = 3,  ///< Precision less than demanded 
    };
#endif   
    ///  Error bit flags detected by the subsystem - flags could be summed.
    enum class sensorsErrorCode
    {
        ERR_SENSORS_OK = 0,            ///< No errors.
        ERR_SENSORS_NOT_READY = 1,     ///< No error data (after system start)
        ERR_SENSORS_SPRESS_LOCKED = 2, ///< Static pressure sensor lock
    };


    //  public fields.
    gpsErrorCode     gpsError;
#if USE_DGPS == 1
	dgpsErrorCode    dgpsError;
#endif
    sensorsErrorCode sensorsError;
};

#endif  //  PSTATEHEALTH_H
