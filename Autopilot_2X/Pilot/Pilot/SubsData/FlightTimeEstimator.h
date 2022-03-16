/**
*                                                                   
* @class FlightTimeEstimator                                               
*                                                                   
* Class to estimate remaining flight time taking into account the power consumption, distance to landing place, wind and altitude.
* 
* NOTICE: Class object has been binary saved in the memory. It could not store any pointers or virtual functions.                                    
*                                                                   
* 2010 Witold Kruczek @ Flytronic                                   
*/

#ifndef FLIGHTTIMEESTIMATOR_H
#define FLIGHTTIMEESTIMATOR_H

class GpsPosition;
class WindData;

class FlightTimeEstimator
{
public:
    FlightTimeEstimator (void):
      _pMaxCharge(0.0f), _pDfDischargeRate(0.0f), _pRetAirspeed(0.0f), _pRetSink(0.0f),
      _outTimeLeft(0.0f), _outTimeLeftAtDistant(0.0f),
      _lastTime(0), _startTime(0), _lastChargeUsed(0.0f), _startChargeUsed(0.0f)
    {};

    bool calculate (int time100, const GpsPosition& currentPos, const GpsPosition& landingPos,
        const WindData& wind, float excessHeight, float chargeUsed, bool onGround);
    void getTimeLeft (float& timeLeft, float& timeLeftAtDistant) const;
    void startMeasure (void);
    void setParameters (float maxCharge, float dfDischargeRate, float retAirspeed, float retSink);

private:

    static const float EQ_TIME;  ///< Time after which weight of the measured and default current is equal.

	//  Parameters
	float _pMaxCharge;			///< Battery initial charge [A*s]
	float _pDfDischargeRate;	///< Default discharging velocity. (average current) [A]
	float _pRetAirspeed;		///< Assumed return airspeed. [kph]
	float _pRetSink;			///< Rate of descent at _pRetAirspeed airspeed [m/s]
	//  Output
	float _outTimeLeft;			///< Left time of flight. [s]
	float _outTimeLeftAtDistant;///< Left time of flight at current plane position (include distance, wind and altitude) [s]
	//  Working variables
	int   _lastTime;			///< System time of last calculations. [100us]
	int   _startTime;			///< System time from discharging counting has started. [100us]
	float _lastChargeUsed;		///< Consumed charge from last calculations. [A*s]
	float _startChargeUsed;		///< Consumed charge in the moment of starting the discharging velocity counting. [A*s]
};

#endif  // FLIGHTTIMEESTIMATOR_H
