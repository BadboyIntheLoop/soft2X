/**                                                                  
* @class FPRealCondition                                                   
*                                                                   
* @brief Class describes conditions of ending current fligh phase.             
* With the condition could be related user parameter returned after fulfilment of the condition (funId).
* Could be used to create a chain of function call (phases that composed to one flight plan instruction)                                                           
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

//  Constructor
FPRealCondition::FPRealCondition (void) :
_funId(0),
_toutLastTime100(0),
_toutTimeToEnd100(0),
_lastTime100(0),
_timeToEnd100(0),
_trackToFinish(0.0f),
_airspeed(0.0f),
_airspeedGreaterThan(false),
_airspeedThres(0.0f),
_airspeedConditionGreaterThan(false),
_airspeedRange(0.0f),
_airspeedLastTime100(0),
_airspeedTimeToEnd100(0),
_savedAirspeedTimeToEnd100(0),
_groundspeed(0.0f) ,
_groundspeedGreaterThan(false),
_agl(0.0f),
_aglGreaterThan(false),
_aglRMin(0.0f),
_aglRMax(0.0f),
_bEmrgLandActive(false),
_emrgAglMin(0.0f),
_emrgAglMargin(0.0f),
_trackLeft(false),
_trackRight(false), 
_trackPLeftMargin(0.0f),

_trackPRightMargin(0.0f), 
_trackPCRadius(0.0f), 
_trackPCLeft(false), 

_trackPCMode(0),
_trackPCModePar(0.0f),
_flyDistMinAgl(0.0f),
_ldx(0.0f),
_advanceDist(0.0f),
_refTrack(0.0f),
_pointDistValue(0.0f),
_pointDistGreaterThan(0.0f),
_accX(0.0f),
#if USE_DGPS == 1
_dgpsRange(0.0f),
#endif
_phi(0.0f),
_p(0.0f),
_accXThres(0.0f),
_accXGreaterThan(false)
{
    reset();
}


/**
* Function that reset all conditions.
*/
void FPRealCondition::reset (void)
{
    _bEnabled = false;
    _funId = -1;
    _bTimeout = false;
    _bInterval = false;
    _bFlyFinish = false;
    _bAirspeed = false;
    _bGroundspeed = false;
    _bAgl = false;
    _bAglRange = false;
	_bAglInRange = false;
    _bEmrgLandAgl = false;
    _bTrackRange = false;
    _bTrackToPoint = false;
    _bFlyDistance = false;
    _bPointDistance = false;
    _bAccX = false;
#if USE_DGPS == 1
	_bDgpsRange = false;
#endif
    _bPhiOrP = false;
	_bAccXOrAirspeedInterval = false;
    _lastTime100 = _toutLastTime100 = 0; ///< _lastTime has to be reset because measurements are inaccurate when setInterval is being called to fast.
	_airspeedLastTime100 = 0;
}

/**
* Function checks if all conditions are met.
* \param psd data received from PState subsystem
* \param pf reference to the function id that would be executed after condition had been met.
* \return true when alle conditions are met; false when are not met or are turned off. When all conditions are met they are reset and ID function saved in setChainFunction will be return.
*/
bool FPRealCondition::checkCondition (const PStateData& psd, int& funId)
{
    if (!_bEnabled)
        return false;

    //  Timeout - This is only one parent condition.
    if (_bTimeout)
    {
        //  Passing invalid time readings.
        //  First enetering propably will be over the range.
        if (psd.time100 <= _toutLastTime100 || psd.time100 > _toutLastTime100 + MAX_CYCLE_INTERVAL_100)
        {
            _toutLastTime100 = psd.time100;
            return false;
        }

        _toutTimeToEnd100 -= (psd.time100 - _toutLastTime100);
        _toutLastTime100 = psd.time100;

        //  If timeout the same action is executed like after condition has been met.
        if (_toutTimeToEnd100 < 0)
        {
            //  set function id to be axecuted after condition has been met.
            funId = _funId;
            // reset all conditions.
            reset();

            return true;
        }
    }

    // Interval condition
    if (_bInterval)
    {
        //  Passing invalid time readings.
        //  First enetering propably will be over the range.
        if (psd.time100 < _lastTime100 || psd.time100 > _lastTime100 + MAX_CYCLE_INTERVAL_100)
        {
            _lastTime100 = psd.time100;
            return false;
        }

        _timeToEnd100 -= (psd.time100 - _lastTime100);
        _lastTime100 = psd.time100;
        // Time has not expired
        if (_timeToEnd100 >= 0)
            return false;
    }

    // The condition for the achievement of the waypoint
    if (_bFlyFinish)
    {
        // Track angle to the waypoint (that is desire to be achieve) set in setFlyFinish function.
        float trackToTrigger = GpsPosition::track (psd.position, _triggerPoint);
        // Track angle difference (0,+180)
        float dtrack = fabsf (GpsPosition::subTrack (_trackToFinish, trackToTrigger));
        // Condition for achieving point - the intersection of the straight line perpendicular to a given track passing by the _triggerPoint
        if (dtrack < 90.0f)
            return false;
     }

    // The condition for the airspeed achievement (desired value has to be passed)
    if (_bAirspeed)
    {
        if ((_airspeedGreaterThan && psd.airspeed <= _airspeed) ||
            (!_airspeedGreaterThan && psd.airspeed >= _airspeed))
            return false;
    }

	// Condition for airspeed for an interval of time

	if (_bAccXOrAirspeedInterval)
	{
		if ((_accXGreaterThan  && fabsf(psd.accX) <= fabsf(_accXThres)) ||
			(!_accXGreaterThan && fabsf(psd.accX) >= fabsf(_accXThres)))

			{
				if ((_airspeedConditionGreaterThan && psd.airspeed <= (_airspeedThres + _airspeedRange)) ||
					(!_airspeedConditionGreaterThan && psd.airspeed >= (_airspeedThres + _airspeedRange)))
				{
					_airspeedTimeToEnd100 = _savedAirspeedTimeToEnd100;
					_airspeedLastTime100 = psd.time100;
					return false;
				}
				//  Passing invalid time readings.
				//  First enetering propably will be over the range.
				if (psd.time100 <= _airspeedLastTime100 || psd.time100 > _airspeedLastTime100 + MAX_CYCLE_INTERVAL_100)
				{
					_airspeedLastTime100 = psd.time100;
					return false;
				}
				Log.msgPrintf("Auto catapult takeoff phase 1 current airspeed: %.2f pass Delta airspeed condition: %.2f and range: %.2f with time: %d", psd.airspeed, _airspeedThres, _airspeedRange, _airspeedTimeToEnd100);

				_airspeedTimeToEnd100 -= (psd.time100 - _airspeedLastTime100);
				_airspeedLastTime100 = psd.time100;

				if (_airspeedTimeToEnd100 >= 0)
				{
					return false;
				}
			}
		else
		{
			Log.msgPrintf("Auto catapult takeoff phase 1 current theta: %.1f (deg) pass accX condition: %.2f", psd.theta * RAD_2_DEG, psd.accX);
		}

	}

    // The condition for the ground speed (desired value has to be passed)
    if (_bGroundspeed)
    {
        if ((_groundspeedGreaterThan && psd.groundspeed <= _groundspeed) ||
            (!_groundspeedGreaterThan && psd.groundspeed >= _groundspeed))
            return false;
    }

    // The condition for achieving the alitutde AGL.
    if (_bAgl)
    {
        if ((_aglGreaterThan && psd.altitude <= _agl) ||
            (!_aglGreaterThan && psd.altitude > _agl))
            return false;
    }

    //  The condition for being in the altitude range.
    if (_bAglRange)
    {
        if (psd.altitude < _aglRMin || psd.altitude > _aglRMax)
            return false;
    }

	//  The condition for being in the altitude range.
    if (_bAglInRange)
    {
		if (((psd.altitude < (_aglBase - _aglRange) || psd.altitude > (_aglBase + _aglRange)) && _bInRange) ||
			((psd.altitude > (_aglBase - _aglRange) && psd.altitude < (_aglBase + _aglRange)) && !_bInRange))
            return false;
    }

    // The condition for the emergency landing after altitude decrease.
    if (_bEmrgLandAgl)
    {
        // Condition activation.
        if (psd.altitude > _emrgAglMin + _emrgAglMargin)
            _bEmrgLandActive = true;

        if (!_bEmrgLandActive || (psd.altitude > _emrgAglMin))
            return false;
    }

    // The condition for being in the track angle range.
    // Range could not be graeter than 180 degrees.
    if (_bTrackRange)
    {
        if (GpsPosition::subTrack(psd.track, _trackLeft) < 0.0f ||
            GpsPosition::subTrack(_trackRight, psd.track) < 0.0f)
            return false;
    }

    // The condition for the track angle to the given waypoint.
    if (_bTrackToPoint)
    {
        float tr = GpsPosition::track (psd.position, _trackPToPos, _trackPCRadius, _trackPCLeft, _trackPCMode, _trackPCModePar);
        float dtr = GpsPosition::subTrack (psd.track, tr);  // dtr positive deviation in the right

        // Margins could be + or -  eg Left=-15, Right=15 gives simetrical angle
        if (dtr < _trackPLeftMargin ||
            dtr > _trackPRightMargin)
            return false;
    }

    // The condition for the flight range. (used for landing)
    if (_bFlyDistance)
    {
        float d = GpsPosition::distance(psd.position, _flyDistToPos);
        float trackToPos = GpsPosition::track (psd.position, _flyDistToPos);

		// The condition for the flight range i met when plane is fly away from waypoint or it fly to waypoint and distance is less than calulated one.
		// Previous condition was based on distance measurement but in case of incorrect GPS positions it fails. 
		// Condition for approaching to waypoint - angle to waypoint < 90 degree
        if (fabsf (GpsPosition::subTrack (_refTrack, trackToPos)) < 90.0f)
            // Minimum altitude condition
            if (psd.altitude > _flyDistMinAgl )
                // Distance condition
                // _ldx could be negative
                if (d >= psd.altitude * _ldx + _advanceDist)
                    return false;
    }

  
    // Distance from given waypoint condition.
    if (_bPointDistance)
    {
        float d = GpsPosition::distance(psd.position, _pointDistToPos);
        if ((_pointDistGreaterThan && d < _pointDistValue) ||
            (!_pointDistGreaterThan && d > _pointDistValue))
            return false;
    }

    // Lower/Higher than acceleration in x axis condition (the absolute value)
    if (_bAccX)
    {           
        if ((_accXGreaterThan  && fabsf(psd.accX) < fabsf(_accX)) ||
            (!_accXGreaterThan && fabsf(psd.accX) > fabsf(_accX)))
            return false;
    }
#if USE_DGPS == 1
	// Lower/Higher than acceleration in x axis condition (the absolute value)
    if (_bDgpsRange)
    {           
		if ((_dgpsRangeGreaterThan  && fabsf(psd.dgpsLonRange) < fabsf(_dgpsRange)) ||
            (!_dgpsRangeGreaterThan && fabsf(psd.dgpsLonRange) > fabsf(_dgpsRange)))
            return false;
    }
#endif

    //  Exceeding angle Phi or angular speed P condition (roll deepening)
    if (_bPhiOrP)
    {
        if ((fabsf(psd.phi) < _phi && fabsf(psd.P) < _p) ||
            (psd.phi * psd.P < 0.0f))
            return false;
    }

     //  set function id to be axecuted after condition has been met.
    funId = _funId;
    //  Reset all conditions.
    reset();

    return true;
}

/**
* Function sets condition related with timeout
* /param tout - time in seconds
* The condition set this function is superior to the other, enough that only it was met.
*/
void FPRealCondition::setMainTimeout (float tout)
{
    _bEnabled = true;
    _bTimeout = true;
    _toutTimeToEnd100 = static_cast<int>(tout * 10000.0f);
}

/**
* This function sets the condition relating to the expiry of the specified time.
* /param intv - time in seconds
*/
void FPRealCondition::setInterval (float intv)
{
    _bEnabled = true;
    _bInterval = true;
    _timeToEnd100 = static_cast<int>(intv * 10000.0f);
}

#if USE_DGPS == 1
/**
* This function sets the condition relating to the exceed of dgps range
*/
void FPRealCondition::setDgpsRange (float dgpsRange, bool greaterThan)
{
    _bEnabled   = true;
    _bDgpsRange = true;

	_dgpsRange  = dgpsRange;
	_dgpsRangeGreaterThan = greaterThan;
}
#endif

/**
* Function sets condition related to achieving given waypoint.
* Condition is met when it is intersect perpendicular line to the route "from" "to" passing by the point moved by maxError in direction of beggining point.
* /param fromPos - beggining waypoint (used to calculate track direction)
* /param toPos - end waypoint (desired)
* /param maxError maximum acceptable error in distance do desire waypoint [meters]
*/
void FPRealCondition::setFlyFinish (const GpsPosition& fromPos, const GpsPosition& toPos, float maxError)
{
    _bEnabled = true;
    _bFlyFinish = true;

    //  Calculate track angle from begginig poitn to end waypoint [degree: 0-360]
    _trackToFinish = GpsPosition::track (fromPos, toPos);

    // Calculation of the proximal point of maxError meters used to check the condition of achieving the target.
	// Plane flies to original desire waypoint, but condition for its achievements will be met earlier intersect perpendicular line to the point calculated below.
    float backTrack = fmodf (_trackToFinish + 180.0f, 360.0f);
    GpsPosition::movePosition (toPos, backTrack, maxError, _triggerPoint);
}

/*
* Function sets condition related to achieving airspeed less or greater than given.
* /param airspeed - reference speed [kph]
* /param greaterThan - true: when current airspeed is greater than reference one, false: opposed
*/
void FPRealCondition::setAirspeed (float airspeed, bool greaterThan)
{
    _bEnabled = true;
    _bAirspeed = true;

    _airspeed = airspeed;
    _airspeedGreaterThan = greaterThan;
}

void FPRealCondition::setAccXOrAirspeedInterval(float airspeed, float range, bool greaterThanASpeed, float time, float accXThres, bool greaterThanAccX)
{
	_bEnabled = true;
	_bAccXOrAirspeedInterval = true;

	_airspeedThres = airspeed;
	_airspeedRange = range;

	_airspeedConditionGreaterThan = greaterThanASpeed;

	_airspeedTimeToEnd100 = static_cast<int>(time * 10000.0f);
	_savedAirspeedTimeToEnd100 = _airspeedTimeToEnd100;

	_accXGreaterThan = greaterThanAccX;
	_accXThres = accXThres;
}

/**
* Function sets condition related to achieving groundspeed less or greater than given.
* /param groundspeed - reference speed [kph]
* /param greaterThan - true: when current groundspeed is greater than reference one, false: opposed 
*/
void FPRealCondition::setGroundspeed (float groundspeed, bool greaterThan)
{
    _bEnabled = true;
    _bGroundspeed = true;

    _groundspeed = groundspeed;
    _groundspeedGreaterThan = greaterThan;
}

/**
* Function sets condition related to achieving altitude above takeoff altitude less or greater than given.
* /param agl - reference altitude [meters]
* /param greaterThan - true: when current altitude is greater than reference one, false: opposed 
*/
void FPRealCondition::setAgl (float agl, bool greaterThan)
{
    _bEnabled = true;
    _bAgl = true;

    _agl = agl;
    _aglGreaterThan = greaterThan;
}

/**
* Function sets condition related to plane being in range of altitude above takeoff altitude.
* /param aglMin - altitude range [meters]
* /param aglMax - altitude range [meters] 
*/
void FPRealCondition::setAglRange (float aglMin, float aglMax)
{
    _bEnabled = true;
    _bAglRange = true;

    _aglRMin = aglMin;
    _aglRMax = aglMax;
}

/**
* Function sets condition related to plane being in range of altitude above takeoff altitude.
* /param aglMin - altitude range [meters]
* /param aglMax - altitude range [meters] 
*/
void FPRealCondition::setAglRange (float aglBase, float aglRange, bool inRange)
{
    _bEnabled = true;
    _bAglInRange = true;

    _aglBase = aglBase;
    _aglRange = aglRange;
	_bInRange = inRange;
}

/**
* Function sets condition related to emergency landing after altitude decrease.
* Condition is met when planes altitude at least once were greater than aglMin+aglMargin and then decrease under aglMin 
*/
void FPRealCondition::setEmrgLandAgl (float aglMin, float aglMargin)
{
    _bEnabled = true;
    _bEmrgLandAgl = true;
    _bEmrgLandActive = false;

    _emrgAglMin = aglMin;
    _emrgAglMargin = aglMargin;
}

/**
* Function sets condition related to plane being in range of track angles.
*/
void FPRealCondition::setTrackRange (float trackLeft, float trackRight)
{
    _bEnabled = true;
    _bTrackRange = true;

    _trackLeft = trackLeft;
    _trackRight = trackRight;
}

/**
* This function sets the condition for the compatibility of the plane track angle with an track angle to a specified point
*/
void FPRealCondition::setTrackToPoint (const GpsPosition& toPos, float leftMargin, float rightMargin, float radius, bool left,
                                       int circleMode, float circleModePar)
{
    _bEnabled = true;
    _bTrackToPoint = true;

    _trackPToPos = toPos;
    _trackPLeftMargin = leftMargin;
    _trackPRightMargin = rightMargin;
    _trackPCRadius = radius;
    _trackPCLeft = left;
    _trackPCMode = circleMode;
    _trackPCModePar = circleModePar;
}


/**  
* Function sets the condition for the range to a specified point at a specified excellence.
* Condition is met when distance from point is less than required or it is increasing.
* /param toPos - desire point
* /param ldx - excellence in current wind conditions
* /param advanceDist - distance acceleration execute condition
* /param refTrack - reference track angle 
* /param minAgl - altitude under which condition is always meet.
*/
void FPRealCondition::setFlyDistance (const GpsPosition& toPos, float ldx, float advanceDist, float refTrack, float minAgl)
{
    _bEnabled = true;
    _bFlyDistance = true;

    _flyDistToPos = toPos;
    _ldx = ldx;
    _advanceDist = advanceDist;
    _refTrack = refTrack;
    _flyDistMinAgl = minAgl;
}

/**  
* Function sets the condition for distance to a specified point.
* /param toPos - desired point
* /param distance - set distance
* /param greaterThan - true: when distance altitude is greater than, false: less than 
*/
void FPRealCondition::setPointDistance (const GpsPosition& toPos, float distance, bool greaterThan)
{
    _bEnabled = true;
    _bPointDistance = true;

    _pointDistToPos = toPos;
    _pointDistValue = distance;
    _pointDistGreaterThan = greaterThan;
}

/**  
* Function sets the condition for exceeding the acceleration in x axis limit.
*/
void FPRealCondition::setAccX(float accX, bool greaterThan)
{
    _bEnabled        = true;
    _bAccX           = true;
    
    _accXGreaterThan = greaterThan;
    _accX            = accX;
}

/**  
* Function sets the condition for exceeding the absolute value of the the roll angle or angular velocity Phi P.
*/
void FPRealCondition::setPhiOrP (float phi, float p)
{
    _bEnabled = true;
    _bPhiOrP = true;

    _phi = phi;
    _p = p;
}


/**
* Function sets function ID returned after condition had been met by the checkCondition function.
*/
void FPRealCondition::setChainFunction (int funId)
{
    _funId = funId;
}
