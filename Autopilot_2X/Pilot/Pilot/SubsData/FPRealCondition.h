/**                                                                  
* @class FPRealCondition                                                   
*                                                                   
* @brief Class describes conditions of ending current fligh phase.             
* With the condition could be related user parameter returned after fulfilment of the condition (funId).
* Could be used to create a chain of function call (phases that composed to one flight plan instruction)                                                           
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef FPREALCONDITION_H
#define FPREALCONDITION_H

class PStateData;

class FPRealCondition
{
public:
    FPRealCondition (void);

    void reset (void);
    bool checkCondition (const PStateData& psd, int& funId);
    void setChainFunction (int funId);
    void setMainTimeout (float tout);

    void setInterval (float intv);
    void setFlyFinish (const GpsPosition& fromPos, const GpsPosition& toPos, float maxError);
    void setAirspeed (float airspeed, bool greaterThan);
	void setAccXOrAirspeedInterval(float airspeed, float range, bool greaterThanASpeed, float time, float accXThres, bool greaterThanAccX);
    void setGroundspeed (float airspeed, bool greaterThan);
    void setAgl (float agl, bool greaterThan);
    void setAglRange (float aglMin, float aglMax);
	void setAglRange (float aglBase, float aglRange, bool inRange);
    void setEmrgLandAgl (float aglMin, float aglMargin);
    void setTrackRange (float tracLeft, float tracRight);
    void setTrackToPoint (const GpsPosition& toPos, float leftMargin, float rightMargin, float radius, bool left,
                          int circleMode, float circleModePar);
    void setFlyDistance (const GpsPosition& toPos, float ldx, float advanceDist, float refTrack, float minAgl);

    void setGlidePath (const GpsPosition& toPos, float ldx, float aglOffset, float aglMin, float refTrack, bool above);

    void setPointDistance (const GpsPosition& toPos, float distance, bool greaterThan);
    void setContainerTrigger (void);
    void setAccX (float accX, bool greaterThan);
	void setAbsAccX(float accXThres, bool greaterThan);
    void setPhiOrP (float phi, float p);
    void setAccOrDeltaV (float accX, float airspeed, float range);
#if USE_DGPS == 1
	void setDgpsRange (float dgpsRange, bool greaterThan);
#endif

private:
    
    static const int MAX_CYCLE_INTERVAL_100 = 10000; ///< Maximum time interval between the time cycles upper which data are ignored [seconds]

    int          _funId;             ///< Function identificator set by the setChainFunction
    bool         _bEnabled;          ///< Turned on condition exists.

    bool         _bTimeout;          ///< Turned on timeout (including all conditions)
    int          _toutLastTime100;   ///< Time from previous cycle (actualized in checkCondition) [100us]
    int          _toutTimeToEnd100;  ///< Time left to end of timeout [100us]

    bool         _bInterval;         ///< Interval condition turned on.
    int          _lastTime100;       ///< Time from previous cycle (actualized in checkCondition) [100us]
    int          _timeToEnd100;      ///< Time left to end of interval [100us]

    bool         _bFlyFinish;        ///< Turned on condition for  waypoint achievement. 
    float        _trackToFinish;     ///< Track angle from beggining point to end point (constant)
    GpsPosition  _triggerPoint;      ///< End point moved by the given distance in the begining point direction.

	bool    _bAccXOrAirspeedInterval;			 ///< (Airspeed greater than ... for ... time100) condition 
    bool    _bAirspeed;              ///< Turned on condition for Airspeed.
    float   _airspeed;               ///< Aispeed margin value.
    bool    _airspeedGreaterThan;    ///< True: condition fullfilled when current speed is greater than false: less than
	bool    _airspeedConditionGreaterThan;		 ///< True: condition fullfilled when current speed is greater than false: less than
	int     _airspeedLastTime100;       ///< Time from previous cycle (actualized in checkCondition) [100us]
	int     _airspeedTimeToEnd100;      ///< Time left to end of interval [100us]
	int     _savedAirspeedTimeToEnd100;
	float   _airspeedThres;				///< Airspeed threshold  
	float   _airspeedRange;				///< Airspeed range


    bool    _bGroundspeed;           ///< Groundspeed condition.
    float   _groundspeed;            ///< Groundspeed margin value.
    bool    _groundspeedGreaterThan; ///< True: condition fullfilled when current speed is greater than false: less than

    bool    _bAgl;                   ///< Altitude AGL condition turned on.
    float   _agl;                    ///< Altitude margin value.
    bool    _aglGreaterThan;         ///< True: condition fullfilled when current altitude is greater than false: less than

    bool    _bAglRange;              ///< AGL range condition turned on.
    float   _aglRMin;                ///< Minimum altitude to fullfill the condition.
    float   _aglRMax;                ///< Maximum altitude to fullfill the condition.

	bool    _bAglInRange;
	float   _aglBase;
	float   _aglRange;
	bool    _bInRange;


    bool    _bEmrgLandAgl;           ///< Emergency landing condition turned on. (after fall of the altitude)
    bool    _bEmrgLandActive;        ///< Condition is active (plane wasover _emrgAglMin + _emrgAglMargin
    float   _emrgAglMin;             ///< Altitude under which the condition is fullfiled (if it is active)
    float   _emrgAglMargin;          ///< Altiude margin relative to _emrgAglMin over which condition has been activated.

    bool    _bTrackRange;            ///< Condition for track range is turned on.
    float   _trackLeft;              ///< Left side of the range.
    float   _trackRight;             ///< Right side of the range.

    bool    _bTrackToPoint;          ///< Condition for track angle to desired point turned on.
    GpsPosition _trackPToPos;        ///< Desire waypoint.
    float   _trackPLeftMargin;       ///< Left margin of track angle.
    float   _trackPRightMargin;      ///< Right margin of track angle.
    float   _trackPCRadius;          ///< Radius around the desire waypoint.
    bool    _trackPCLeft;            ///< Direction of circle.
    int     _trackPCMode;            ///< Type of circle.
    float   _trackPCModePar;         ///< Circle parameter.

    bool    _bFlyDistance;           ///< Condition for flight range turned on (used in landing)
    GpsPosition _flyDistToPos;       ///< Position of the touchdown
    float   _flyDistMinAgl;          ///< Altitude under which condition FlyDistance is always fullfiled.
    float   _ldx;                    ///< Excellence in given configuration and wind conditions.
    float   _advanceDist;            ///< Distance of the acceleration when condition worked.
    float   _refTrack;               ///< Landing track (it is not a current track)

    bool    _bPointDistance;         ///< Condition for discance from waypoint turned on.
    GpsPosition _pointDistToPos;     ///< Position to which the distance is measured.
    float   _pointDistValue;         ///< Desired distance.
    bool    _pointDistGreaterThan;   ///< True: when condition is fullfilled (if current distance is greater than)

    bool    _bAccX;                  ///< Turn on condition for acceleration in X axis.
    float   _accX;                   ///< Value threshold of acceleration.
    bool    _accXGreaterThan;        ///< True: condition fulfilled when acc_x is higher than threshold    
    
	bool    _bAbsAccX;               ///< Turn on condition for Acc_x angle.
	float   _accXThres;               ///< Abs_Acc_x threshold

#if USE_DGPS == 1
	bool    _bDgpsRange;             ///< Turn on condition for range
    float   _dgpsRange;              ///< Value threshold of range
	bool    _dgpsRangeGreaterThan;
#endif

    bool    _bPhiOrP;                ///< Turn on condition for Phi angle or P speed.
    float   _phi;                    ///< Desire roll angle Phi
    float   _p;                      ///< Desire angular velocity in the longitudinal axis P
};

#endif  // FPREALCONDITION_H
