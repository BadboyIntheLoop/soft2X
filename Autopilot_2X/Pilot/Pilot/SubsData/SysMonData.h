/**                                                               
* @class SysMonData                                                        
*                                                                   
* Class representing data shared by the subsystem based on the SystemMonitor class.                                   
* It is used as the interface beetwen susbsystems.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef SYSMONDATA_H
#define SYSMONDATA_H

class PStateHealth;

class SysMonData
{
public:
    enum HwButton
    {
        BTN_UNKNOWN = 0,           //  initial value
        BTN_START = 1,
        BTN_STOP = 2
    };
    
    // Error bits flags detected by the subsystem - can be summed.
    enum SysErrorFlag
    {
        ERR_OK = 0,                 ///< Lack of error
        ERR_DATA_NOT_READY = 1,     ///< Lack data about errors (after system started)
        ERR_GPS_BAD = 2,            ///< GPS not started jet or does not work.
        ERR_SENSORS_FAILURE = 4,    ///< Sensors malfunction.
        ERR_VBAT_NO_TAKEOFF = 8,    ///< To low voltage level of main battery lock the takeoff.
        ERR_IBAT_NO_TAKEOFF = 16,   ///< Current on the buffer battery is incorect - lock takeoff.
		ERR_LOW_BATTERY_LEVEL = 32
#if USE_DGPS == 1
		, ERR_DGPS_BAD = 64          ///< DGPS not started jet or does not work.
#endif
    };
       
    SysMonData(void);

    //  public fields
    SysErrorFlag errFlag;
};


//  structure packing
#pragma pack(1)

/**  
*  Structure for the telemetric data associated with SystemMonitor subsystem.
*   Must be packed. For portability types uC/OS-II system were used.
*/
class SysMonTlm
{
public:
    INT32U time;         // Relative time [ms]
    INT16S uUpper;       // Buffer battery voltage [V*1000]
    INT16S iUpper;       // Buffer battery current [A*1000]
    INT8U  gpsError;     // GPS error code according to PStateHealth::GpsErrorCode
    INT8U  sensorsError; // Sensors error code according to PStateHealth::SensorsErrorCode
    INT8U  cpu;          // CPU usage [%]
    INT16U uLower;       // Main battery voltage [V*1000]
    INT16U iLower;       // Main battery current [A*100] (NOTICE: different scale)
    INT16U qLower;       // Charge consumed from main battery [A*s]
    INT16U lastLinkIntv; // Time from the last command "smon ping" [s]
    INT16S timeLeft;     // Remaining time of flight [s]
    INT16S userTimeLeft; // Remaining time of flight for use by the user [s]
    INT8U  dtYear;       // Date - year (last two digits over 2000)
    INT8U  dtMonth;      // month
    INT8U  dtDay;        // day
    INT8U  utcHours;     // Time UTC from GPS (hours)
    INT8U  utcMins;      // minutes
    INT8U  utcSecs;      // seconds
    INT8U  utcMsecs;     // miliseconds
    INT8U  flags;        // Status flags:
                         //     1 - Plane on the ground (SystemNowOnGround = 1)
                         //     2 - UTC time is Czas UTC jest valid
    INT8U  fuel;         // The amount of fuel in the tank [%]

    SysMonTlm (int time100us, float iUp, float uUp, const PStateHealth &psh,
        float iLow, float uLow, float qLow, int lastLinkTime100, float tLeft, float utLeft,
        int pDtYear, int pDtMonth, int pDtDay,
        int pUtcHours, int pUtcMins, int pUtcSecs, int pUtcMsecs, bool onGround, bool pUtcValid, int fuel)
    {
        fillFrom (time100us, iUp, uUp, psh, iLow, uLow, qLow, lastLinkTime100, tLeft, utLeft,
            pDtYear, pDtMonth, pDtDay,
            pUtcHours, pUtcMins, pUtcSecs, pUtcMsecs, onGround, pUtcValid, fuel);
    }

private:
    void fillFrom (int time100us, float iUp, float uUp, const PStateHealth &psh,
        float iLow, float uLow, float qLow, int lastLinkTime100, float tLeft, float utLeft,
        int pDtYear, int pDtMonth, int pDtDay,
        int pUtcHours, int pUtcMins, int pUtcSecs, int pUtcMsecs, bool onGround, bool pUtcValid, int fuel);
};

//  end of packing
#pragma pack()

#endif  //  SYSMONDATA_H
