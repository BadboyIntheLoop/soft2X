/**
*                                                                  
* @ class WindResolver                                                      
*                                                                   
* Class calculates wind speed and direction based on the differences between airspeed and groundspeed at different angles of the track.                                                  
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

/*
* Passing new set data needet to wind calculations.
*  Wind is not calculated in every function call.
*  Returns number of algorithm used in wind calculationsc (if it was calculated in this cycle) or 0.
*/
int WindResolver::putData (const PStateData& psd, const PStateHealth& psh)
{
    static const int   FORCE_INTERVAL_LONG =   static_cast<int>(60.0f * 10000.0f);    //  Maximum of time after which measurement has to be done. [s]
    static const int   FORCE_INTERVAL_SHORT =  static_cast<int>(30.0f * 10000.0f);    //  Optimal time after which measurement has to be done. [s]
    static const float MIN_DTRACK          =  30.0f;   // Minimum difference between track angles for measurement correctness.
    static const float MAX_DTRACK          = 150.0f;   // Maximum difference between track angles for measurement correctness.
    static const float DTRACK_1            =  60.0f;   // Lower limit of the optimal differnece between tracks anglee.
    static const float DTRACK_2            = 120.0f;   // Upper limit of the optimal differnece between tracks anglee.
    //  Warunki wykonania pomiaru
    static const float MIN_AIRSPEED        = 40.0f;   //  Minimum airspeed [kph]
    static const float MIN_GROUNDSPEED     = 10.0f;   //  Minimum groundspeed [kph]
    static const float MIN_ALTITUDE        = 50.0f;   //  Minimum altitude [m]
    static const float MIN_THETA           = -15.0f * DEG_2_RAD;    //  Tilt in down not greater than (- down, + up)
    static const float MAX_THETA           =   5.0f * DEG_2_RAD;    //  Tilt in up not greater than (- down, + up)
    static const float MAX_PHI_ABS         =  20.0f * DEG_2_RAD;    //  Tilt
    static const float MAX_Q_ABS           =   0.1f;
    static const float MAX_P_ABS           =   0.1f;
   
    //  Lack of GPS data.
    if (psh.gpsError != PStateHealth::gpsErrorCode::ERR_GPS_OK &&
        psh.gpsError != PStateHealth::gpsErrorCode::ERR_GPS_2D_MODE)
    {
        _a2SampleCnt = 0;   // Algorithm 2 reset.
        _lastSampleTime100 = psd.time100;
        return 0;
    }
    
    //  Algorithm 2
    bool a2Computed = false;
    // If there are met the required physical conditions for the algorithm 2
    if (psd.airspeed > MIN_AIRSPEED &&
        psd.groundspeed > MIN_GROUNDSPEED &&
        psd.altitude > MIN_ALTITUDE &&
        fabsf (psd.phi) < MAX_PHI_ABS)
    {
        //  Return true when wind calculation was made (once per tens of seconds)
        a2Computed = a2Compute (psd);
    }
    else
        _a2SampleCnt = 0;   // Algorithm 2 reset.


    //  Algorithm 1 (old one)
    bool a1Computed = false;
    //  Difference between current track angle and last used in calculations (absolute value)
    float dTrackAbs = fabsf (GpsPosition::subTrack (psd.track, dataNew.track));
  
    //  Wind calculations could be made:
    //      - only when angles difference is big enough
    //      - at the optimum time at the optimum angle difference tracks or
    //      - at lower difference, if the maximum time is exceeded since the last calculation or
    //      - when it was not calculated (or last measurement was incorrect)
    if (dTrackAbs >= MIN_DTRACK && dTrackAbs <= MAX_DTRACK)
    {
        if ((dTrackAbs > DTRACK_1 && dTrackAbs < DTRACK_2 && psd.time100 > dataNew.time100 + FORCE_INTERVAL_SHORT) ||
            psd.time100 > dataNew.time100 + FORCE_INTERVAL_LONG ||
            !_isWindValid)
        {
            //  If phisical conditions was met.
            if (psd.airspeed > MIN_AIRSPEED &&
                psd.groundspeed > MIN_GROUNDSPEED &&
                psd.altitude > MIN_ALTITUDE &&
                psd.theta > MIN_THETA && psd.theta < MAX_THETA &&
                fabsf (psd.phi) < MAX_PHI_ABS &&
                fabsf (psd.Q) < MAX_Q_ABS &&
                fabsf (psd.P) < MAX_P_ABS)
            {
                dataOld = dataNew;

                dataNew.time100 = psd.time100;
                dataNew.tas = psd.tas;
                dataNew.groundspeed = psd.groundspeed;
                dataNew.track = psd.track;
                dataNew.isValid = true;

                if (dataOld.isValid)
                {
                    _isWindValid = compute ();
                    a1Computed = _isWindValid;
                }
            }
        }
    }

        _lastSampleTime100 = psd.time100;

        //  returns algorithms number that calculated the wind.
        if (a1Computed)
            return 1;
        else if (a2Computed)
            return 2;

        return 0;
}

/*
* Get wind data.
* /param from - direction from which wind blows [degrees]
* /param speed - speed [kph]
* /return true, when data are correct. False, when values are 0.
*/
bool WindResolver::getWind (float &from, float &speed) const
{
    return _wavg.getAvgWind (from, speed);
}

/*
* Reset averager state.
*/
void WindResolver::reset (void)
{
    _wavg.reset();
}

bool WindResolver::compute (void)
{
    static const float EPS = 5.0f;

    // The transformation of the velocity vectors relative to the ground with polar coordinates to Cartesian.
    // Assuming that the vectors have beginnings at the center of the coordinate system, they are also the coordinates of their ends.
    float  xOld = dataOld.groundspeed * cos (dataOld.track * DEG_2_RAD);
    float  yOld = dataOld.groundspeed * sin (dataOld.track * DEG_2_RAD);

    float  xNew = dataNew.groundspeed * cos (dataNew.track * DEG_2_RAD);
    float  yNew = dataNew.groundspeed * sin (dataNew.track * DEG_2_RAD);

    //  Designation of points of intersection of the circles with radius "airspeed1" and "airspeed2" and center points calculated obove.
    //  Points of intersection of the circles are the coordinates of the end of the wind vector
    //  (2 solutions).
    //  The algorithm designate the points of intersection of circles is based on:
    //  http://www.metaphorical.net/note/on/circle_intersection
    float dx = xOld - xNew;
    float dy = yOld - yNew;
    float d2 = dx*dx + dy*dy;
    // Distance between centers of the circles.
    float d = sqrtf (d2);

    //  If the circle are not intersecting then exit function.
    //  EPS added due to airspeed relative to groundspeed calibration errors.
    //  For larger discrepancies do not calculate due to the low accuracy.
    if (d > dataOld.tas + dataNew.tas + EPS ||
        d < fabsf (dataOld.tas - dataNew.tas))
        return false;

    float a = (dataOld.tas*dataOld.tas - dataNew.tas*dataNew.tas + d2) / (2.0f*d);
    float b = dataOld.tas*dataOld.tas - a*a;
    float h = 0.0f;
    if (b >= 0.0f)
    {
        //  At airspeed relative to groundspeed calibration errors square root argument could be negative
        //  (circles do not intersects there are too far from each other). In that case it assumed that they contact in one point.
        h = sqrtf (b);
        // Float value correctness control.
        Numbers::assure (h, 0.0f);
    }

    // Coordinates of centre point
    float x2 = xOld + a*(xNew - xOld)/d;
    float y2 = yOld + a*(yNew - yOld)/d;

    //  Solution 1
    float paX = x2 + h*(yNew - yOld)/d;
    float paY = y2 - h*(xNew - xOld)/d;
    float paR = sqrtf (paX*paX + paY*paY);

    //  Solution 2
    float pbX = x2 - h*(yNew - yOld)/d;
    float pbY = y2 + h*(xNew - xOld)/d;
    float pbR = sqrtf (pbX*pbX + pbY*pbY);
   
    WindVector w;
    if (paR < pbR)
    {
        w._x = paX;
        w._y = paY;
    }
    else
    {
        w._x = pbX;
        w._y = pbY;
    }

    _wavg.putWind (w);

    return true;
}


/*  
* Function provides data from every sample to the algorithm 2.
* Algorithm 2 sum vectors of the track traveled in time of every sample (relative to air).
* After a specified time accumulated vector is added to the initial coordinates and compared with the coordinates of the GPS. The difference is the vector wind.
* Calculated wind vector is averaged from couple of samples and different algorithms.
* Return true if in current cycle wind had been calculated.
*/
bool WindResolver::a2Compute (const PStateData& psd)
{
    static const int   SAMPLES_TO_SUM = 30*100;          // number of samples to sum.
    static const float MAX_SAMPLE_LENGTH = 2.0f/100.0f;  // maximum time of sample [s]

    //  Initialization
    if (_a2SampleCnt == 0)
    {
        _a2GpsStart = psd.position;
        _a2Trace.x = _a2Trace.y = 0.0f;
        _a2StartTime100 = _lastSampleTime100;
    }

    //  tas speed (approximately the leading edge angle alfa = 0)
    float te = psd.tas * cos (psd.theta) * KPH_2_MS;

    //  duration of current sample [s]
    float curSampleLength = (psd.time100 - _lastSampleTime100) * 0.0001f;
    // exit and start new counting when values are beyond the scope.
    if (curSampleLength <= 0.0f || curSampleLength > MAX_SAMPLE_LENGTH)
    {
        _a2SampleCnt = 0;
        return false;
    }

    // Vector  distance traveled in time of one sample (relative to air)
    Vector2f delta = Vector2f::fromRadial (te, psd.psi);
    delta.multC (curSampleLength);

    // Summing vectors from each samples.
    _a2Trace.add (delta);

    //  Calculate wind vector from given amount of samples.
    if (++_a2SampleCnt >= SAMPLES_TO_SUM)
    {
        //  Esimated geographic position (x north, y east)
        GpsPosition a2GpsEnd;
        if (GpsPosition::movePositionXY (_a2GpsStart, _a2Trace.y, _a2Trace.x, a2GpsEnd))
        {
            //  track vector between GPS position and estimated position.
            Vector2f d;
            GpsPosition::getVectorXY (a2GpsEnd, psd.position, d.y, d.x);

            float tt = 10000.0f * MS_2_KPH / (psd.time100 - _a2StartTime100);
            Numbers::assure (tt, 0.0f);
            d.multC (tt);

            // add calculated wind to averager.
            WindVector w;
            w._x = d.x; w._y = d.y;
            _wavg.putWind (w);
        }

        _a2SampleCnt = 0;
        return true;
    }

    return false;
}


/*
* Add given vector to the current one.
*/
void WindResolver::WindVector::add (const WindVector &w)
{
    _x += w._x;
    _y += w._y;
}

/*
* Reset state of wind averager.
*/
void WindResolver::WindAverager::reset (void)
{
    _usedItems = 0;
    _newPos    = 0;
    _avgSpeed  = 0.0f;
    _avgFrom   = 0.0f;
}

/*
* Add new element to the averager and calculate average.
*/
void WindResolver::WindAverager::putWind (const WindVector &w)
{
    _items[_newPos] = w;
    _newPos = (_newPos + 1) % NITEMS;
    if (_usedItems < 0)     // for safety
        _usedItems = 0;
    _usedItems = (_usedItems < NITEMS) ? (_usedItems + 1) : NITEMS;

    WindVector wr;

    //  Sum
    for (int i=0; i<_usedItems; i++)
        wr.add(_items[i]);

    //  Averaging
    wr._x /= static_cast<float>(_usedItems);
    wr._y /= static_cast<float>(_usedItems);

    _avgSpeed = sqrtf (wr._x*wr._x + wr._y*wr._y);
    float _avgTo = atan2 (wr._y, wr._x) * RAD_2_DEG;

    //  conversion to the meteorological direction.
    _avgFrom = GpsPosition::addTrack (_avgTo, 180.0f);
}


/**
* Get average speed and wind direction.
* /param speed - speed in kph
* /param from - direction from which wind blows
*  Return false where there is no elements in the queue.
*/
bool WindResolver::WindAverager::getAvgWind (float &from, float &speed) const
{
    if (_usedItems == 0)
    {
        from = 0.0f;
        speed = 0.0f;
        return false;
    }

    speed = _avgSpeed;
    from = _avgFrom;
    return true;
}
