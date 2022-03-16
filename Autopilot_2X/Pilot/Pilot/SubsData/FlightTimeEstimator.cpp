/**                                                              
* @class FlightTimeEstimator                                               
*                                                                   
* Class to estimate remaining flight time taking into account the power consumption, distance to landing place, wind and altitude.
* 
* NOTICE: Class object has been binary saved in the memory. It could not store any pointers or virtual functions.                                    
*                                                                   
* 2010 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>


const float FlightTimeEstimator::EQ_TIME = 600.0f;

/**
* Function make the calculations, results are available by using function "getTimeLeft". 
* /param time100 - current system time [100us]
* /param currentPos - current plane geographical position.
* /param landingPos - geographical position of the landing place.
* /param wind - curremt wind parameters
* /param excessHeigh - difference beetwen current altitude and altittude of the circle over the landing point [m]
* /param chargeUsed - consumed charge from the start of the autopilot [A*s]
* /param onGround - plane is on the ground
*  In case of an error function returns false and sets output times to 0.
*/
bool FlightTimeEstimator::calculate (int time100, const GpsPosition& currentPos, const GpsPosition& landingPos,
        const WindData& wind, float excessHeight, float chargeUsed, bool onGround)
{
    _lastTime = time100;
    _lastChargeUsed = chargeUsed;

    // On the ground average discharge current in not measured (variable "a" = 0)
    if (onGround)
        startMeasure();

    //  Time in seconds from the beggining of counting
    float t = static_cast<float>(time100 - _startTime) * 0.0001f;

    //  Measured average discharging current [A*s]
    float mDisRate = (t > 0.0f) ? ((chargeUsed - _startChargeUsed) / t) : 0.0f;

	// The weighting factor specifying which part of the average discharge current is to be the default, and the measured
    float a = t / (t + EQ_TIME);

    // Average weighting discharging current [A]
    float wCurrent = (a * mDisRate) + ((1-a) * _pDfDischargeRate);

    // Calculating left flight time [s]
    _outTimeLeft = ((_pMaxCharge - chargeUsed) / wCurrent) + (excessHeight / _pRetSink);

    // Calculating distane and track to the landing place [m, deegres]
    float d = GpsPosition::distance (currentPos, landingPos);
    float tr = GpsPosition::track (currentPos, landingPos);

    //  Calculate the travel speed of the return
    //  http://williams.best.vwh.net/avform.htm#Wind
    float e = GpsPosition::subTrack(wind.from, tr) * DEG_2_RAD;         // wind angle [rad]
    float swc = (wind.speed / _pRetAirspeed) * sinf(e);
    float w = _pRetAirspeed * sqrtf(1-swc*swc) - wind.speed * cosf(e);  // travel speed [kph]

    // Calculate return time [s]
    float tRet = d / (w * KPH_2_MS);

    //  Calculate left time at the current position.
    _outTimeLeftAtDistant = _outTimeLeft - tRet;

    // Check corectness of the results
    if (!Numbers::isValid (_outTimeLeft) && !Numbers::isValid (_outTimeLeftAtDistant))
    {
        _outTimeLeft = 0.0f;
        _outTimeLeftAtDistant = 0.0f;
        return false;
    }

    return true;
}


/**
* Function returns resuls of calculations made in the "calculate" function.
* /param  timeLeft - estimated remaining time of flight
* /param  timeLeftAtDistance - estimated remaining time of flight being in position.
*/
void FlightTimeEstimator::getTimeLeft (float& timeLeft, float& timeLeftAtDistant) const
{
    timeLeft = _outTimeLeft;
    timeLeftAtDistant = _outTimeLeftAtDistant;
}

/**
* Start measure of power consumption needed to calculate average speed of discharging.
* It should be called after the take-off (during takeoff charge consumption is unreliable).
*/
void FlightTimeEstimator::startMeasure (void)
{
    _startTime = _lastTime;
    _startChargeUsed = _lastChargeUsed;
}

/**
* Setting calculations parameters. They could be changed in any moment, but after changing function "calculate" must be executed.
* /param maxCharge - maximum battery charge [A*s]
* /param dfDischargeRate - default average discharging current maintaining aircraft in straight flight. mAh/s 
* /param retAirspeed - founded returned speed -IAS to the landing place [kph]
* /param retSink - descending speed at retAirspeed [m/s]
*/
void FlightTimeEstimator::setParameters (float maxCharge, float dfDischargeRate, float retAirspeed, float retSink)
{
    _pMaxCharge = maxCharge;
    _pDfDischargeRate = dfDischargeRate;
    _pRetAirspeed = retAirspeed;
    _pRetSink = retSink;
}

