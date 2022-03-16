/**                                                               
* @class SysMonData                                                        
*                                                                   
* Class representing data shared by the subsystem based on the SystemMonitor class.                                   
* It is used as the interface beetwen susbsystems.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>


SysMonData::SysMonData(void)
{
    errFlag = ERR_DATA_NOT_READY;
}

void SysMonTlm::fillFrom (int time100us, float iUp, float uUp, const PStateHealth &psh,
                          float iLow, float uLow, float qLow, int lastLinkTime100, float tLeft, float utLeft,
                          int pDtYear, int pDtMonth, int pDtDay,
                          int pUtcHours, int pUtcMins, int pUtcSecs, int pUtcMsecs, bool onGround, bool pUtcValid,
                          int pFuel)
{
    // Integer division
    time    = time100us / 10;   // time in ms, but is delivered as 100us
    //  Buffer battery (Called upper)
//    uUpper    = static_cast<INT16S>(uUp * 1000.0f);
//    iUpper    = static_cast<INT16S>(iUp * 1000.0f);
    uUpper    = static_cast<INT16S>(uUp * 100.0f);
    iUpper    = static_cast<INT16S>(iUp * 100.0f);
    //  Main battery (called lower) - protection from negative values
    uLower    = static_cast<INT16U>(uLow>0.0f ? uLow * 1000.0f : 0.0f);
    iLower    = static_cast<INT16U>(iLow>0.0f ? iLow * 100.0f : 0.0f); //  different scale
    qLower    = static_cast<INT16U>(qLow>0.0f ? qLow : 0.0f);
    // Error flags.
    gpsError     = static_cast<INT8U>(psh.gpsError);
    sensorsError = static_cast<INT8U>(psh.sensorsError);
    //  When OS_TASK_STAT_EN == 0 OSCPUUsage variable does not exist
#if OS_TASK_STAT_EN == 1
    cpu          = OSCPUUsage;
#else
    cpu          = 0;
#endif
    // Time from last "smon ping" command [s]
    lastLinkIntv = static_cast<INT16U>(0xFFFF & ((time100us - lastLinkTime100) / 10000));
    timeLeft     = static_cast<INT16S>(tLeft);
    userTimeLeft = static_cast<INT16S>(utLeft);

    // Date
    dtYear = static_cast<INT8U>(pDtYear);
    dtMonth = static_cast<INT8U>(pDtMonth);
    dtDay = static_cast<INT8U>(pDtDay);

    // UTC time
    utcHours = static_cast<INT8U>(pUtcHours);
    utcMins  = static_cast<INT8U>(pUtcMins);
    
    utcSecs  = static_cast<INT8U>(pUtcSecs);
    // integer division
    utcMsecs = static_cast<INT8U>(pUtcMsecs / 10);

    // Flags
    flags = (onGround  ? 1 : 0) +
            (pUtcValid ? 2 : 0);

    // Data from external sensors (fuel, ...)
    fuel = static_cast<INT8U>(pFuel);
}

