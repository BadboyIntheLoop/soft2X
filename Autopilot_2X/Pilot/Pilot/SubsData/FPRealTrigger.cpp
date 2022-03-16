/**
*                                                                   
* @class FPRealTrigger                                                     
*                                                                  
* @brief Class describes conditions that could appear asynchronously in any time during the flight.                                
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                  
*/

#include "PilotIncludes.h"

/**
* Constructor
*/
FPRealTrigger::FPRealTrigger (void) :
_accX(0.0f), _lastAccX(0.0f), _gSpeed(0.0f)
{
    reset();
}

/**
* Reset all conditions.
*/
void FPRealTrigger::reset (void)
{
    _bEnabled = false;
    _bAccX = false;
    _bGSpeed = false;
}


/**
* Checks if all triggers conditions are met.
* /param psd - data received from PState subsystem
* returns true when all trigger conditions had been met; false when they had not met or had been turned off
* After conditions had been met, they would be reset.
*/
bool FPRealTrigger::checkTrigger (const PStateData& psd)
{
    if (!_bEnabled)
        return false;

    //  Condition for acceleration changes in X axis (accX)
    bool bret = false;
    if (_bAccX)
    {
        if (fabsf(psd.accX) > fabsf(_accX) && fabsf(_lastAccX) < fabsf(_accX))
        {
            bret = true;
        }

        _lastAccX = psd.accX;

        //  return when condition had not been met.
        if (!bret)
            return false;
    }

    //  Groundspeed decrease condition.
    if (_bGSpeed)
    {
        //  return when condition had not been met.
		if (psd.groundspeed >= _gSpeed)
            return false;
    }

    return true;
}

/**
* Sets condition related to the achievement of the acceleration in the X axis
* The condition detect passing through the leading edge of the reference value.
* /param accX - reference acceleration (G)
*/
void FPRealTrigger::setAccX (float accX)
{
    _bEnabled = true;
    _bAccX = true;

    _accX = accX;
    _lastAccX = 0.0f;
}


/**
* Sets condition related to drcrease of the groundspeed under given value. Condition detects value less that reference.
* /param gSpeed - reference ground speed (kph)
*/
void FPRealTrigger::setGSpeed (float gSpeed)
{
    _bEnabled = true;
    _bGSpeed = true;

    _gSpeed = gSpeed;
}
