/**
*                                                                   
* @clacc GpsPosition                                                       
*                                                                   
* Class stores geographical position and makes operations related to the position calculations.                                  
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

// Multiplication data in degrees factor to store data in int type.
const float GpsPosition::scaleFactor = 10000000.0f;
const int GpsPosition::scaleFactorI = 10000000;
const double GpsPosition::scaleFactorD = GpsPosition::scaleFactor;  // Types conversion.
const float GpsPosition::EARTH_RADIUS = 6372797.560856f;


/**
* Set Latitude.
*/
void GpsPosition::setLat (double lat)
{
    _latitude = static_cast<int>(lat * scaleFactorD);
}


/** 
* Set longitude
*/
void GpsPosition::setLon (double lon)
{
    _longitude = static_cast<int>(lon * scaleFactorD);
}


/**
* Get Latitude
*/
void GpsPosition::getLat (double &lat) const
{
    lat = getLat();
}

double GpsPosition::getLat (void) const
{
    return _latitude / scaleFactorD;
}


/**
* Get Longitude
*/
void GpsPosition::getLon (double &lon) const
{
    lon = getLon();
}

double GpsPosition::getLon (void) const
{
    return _longitude / scaleFactorD;
}


/**
*  Get Latitude and Longitude in the telemetric data format (degrees * 1000000).
*/
void GpsPosition::getPosTlm (INT32S &lat, INT32S &lon) const
{
    //  scaling to the telemetry format.
    static const int TSCALE = scaleFactorI / 1000000;
    lat = _latitude / TSCALE;
    lon = _longitude / TSCALE;
}


/**
* Check if two position is equal.
*/
bool GpsPosition::isEqual (const GpsPosition &pos) const
{
    if (_latitude == pos._latitude && _longitude == pos._longitude)
        return true;

    return false;
}


/** 
* Function calculates track in degrees between two positions (in WGS-84 format). Is is quick function that is no need to be precisse.
*/
float GpsPosition::track (const GpsPosition& fromPos, const GpsPosition& toPos)
{
    float f = trackFlatEarth (fromPos, toPos);
    //  correctness control of the float
    Numbers::assure (f, 0.0f);
    return f;
}

/**
* Function calculates track in centi-degrees between two positions (in WGS-84 format). Is is quick function that is no need to be precisse.
*/
int32_t GpsPosition::track_cd(const GpsPosition& fromPos, const GpsPosition& toPos)
{
    float f = trackFlatEarth(fromPos, toPos);
    //  correctness control of the float
    Numbers::assure(f, 0.0f);
    return static_cast<int32_t>(f * 100);
}


/**  Function calculates track in degrees between two positions (in WGS-84 format). Second point is the tangent to the circle whose center is the point of destination 
* /param  fromPos - beggining point
* /param  toPos - destination pointy (circle center)
* /param  circleRadius - circle aorund destination point radius (meters)
* /param  circleLeft - true: tangent from right side (circle will be from left)
* /param  circleMode - 0: old algorithm (correction angle depending on the distance from the circuit), 1: new algorithm (correction angle depends on the point of overtaking with a constant angle)
* /param  circlemodePar - algorithm parameter (for 0, 1 is a distance to the circle when algorithm is switched from flight to the tangent to the circle to the algrotihms "circleMode"
*  
* Function is used in circle around instruction. Wind drift is eliminated automatically.
*/
float GpsPosition::track (const GpsPosition& fromPos, const GpsPosition& toPos,
                          float circleRadius, bool circleLeft, int circleMode, float circleModePar)
{
    static const float DIST_MIN = 0.01f;

    // Track angle to the point
    float tr = trackFlatEarth (fromPos, toPos);

    // Flight straight to the point.
    if (circleRadius == 0.0f)
        return tr;

    //  Parameteres control.
    if (circleMode < 0 || circleMode > 1)
        circleMode = 0;

    if (!(circleModePar > DIST_MIN))
        circleModePar = 20.0f;

    //  Distance to the point.
    float dist = distanceFlatEarth (fromPos, toPos);


    //  Track correction.
    float dtr = 0.0f;

    // algorithm test 
    if (circleMode == 1)
    {
        float cosA = circleRadius / (circleRadius + circleModePar);  //  cosinus overtake angle A
        float sinA = sqrtf (1.0f - cosA*cosA);                  //  sinus overtake angle A
        dtr = atanf (circleRadius*sinA / (dist - circleRadius*cosA));
        //  correctness control of the float
        Numbers::assure (dtr, PI2);
        
        //  Inside circle, when dist < circleRadius - MARG, dtr is negative
        //  Has to be corrected with adding PI
        if (dtr < 0.0f)
            dtr += PI;

        //  Plane far away from circle - flying to tangent
        if (dist > circleRadius + circleModePar)
            dtr = asinf (circleRadius / dist);

        //  Conversion to degrees
        dtr *= RAD_2_DEG;

        //  When circle in left, correction is made in right (adding correction to the track)
        if (circleLeft)
            tr = GpsPosition::addTrack(tr, dtr);
        else
            tr = GpsPosition::addTrack(tr, -dtr);

        //  correctness control of the float
        Numbers::assure (tr, 0.0f);

        return tr;
    }
    //*******


    float b = 0.0f;
    float asb = 0.0f, adr = 0.0f;

    float g = asinf(circleRadius / (circleRadius + circleModePar));
    float a = (PI2-g) / circleModePar;


    if (circleRadius <= dist)
    {
		// Plane outside the circle - flying at a tangent
		if (dist > DIST_MIN)
			b = circleRadius / dist;

        asb = asinf (b);
        adr = PI2 - a * (dist-circleRadius);

        if (asb > adr)
            dtr = asb;
        else
            dtr = adr;
    }
    else
    {
		// When plane is inside the circle, correction is made by substracting correction from track
		if (dist > DIST_MIN)
            b = circleRadius / (2.0f * circleRadius - dist);
    
        asb = asinf (b);
        adr = PI2 + a * (dist-circleRadius);

        if (asb > adr)
            dtr = PI - asb;
        else
            dtr = PI - adr;
    }

    //  Conversion to degrees
    dtr *= RAD_2_DEG;

	//  When circle in left, correction is made in right (adding correction to the track)
    if (circleLeft)
        tr = GpsPosition::addTrack(tr, dtr);
    else
        tr = GpsPosition::addTrack(tr, -dtr);

    //  correctness control of the float
    Numbers::assure (tr, 0.0f);

	return tr;
}

/**
* Version that compute on flat Earth model.
* Source: http://williams.best.vwh.net/avform.htm#LL
*/
float GpsPosition::trackFlatEarth (const GpsPosition& fromPos, const GpsPosition& toPos)
{
    static const double RD1 = DEG_2_RAD / scaleFactor;

    float dLon = static_cast<float>(toPos._longitude - fromPos._longitude);
    float dLat = static_cast<float>(toPos._latitude - fromPos._latitude);

    //  Calculating lat0 is made in double precision. When it was made in single the precision was to small (there was ailerons vibrations during turn)
    //  Multiplication of two double numbers (software) is 10 time slower than multiplication of two float numbers (hardware)
    float lat0 = static_cast<float>(fromPos._latitude * RD1);

    float tr = atan2f (dLon * cosf (lat0), dLat);
    // Normalization to the range 0-360 degrees
    tr = fmodf (tr * RAD_2_DEG + 360.0f, 360.0f);

    return tr;
}


/**
* Function calculate coordinates of the new points. Is based on given point, track and distance.
* /param origPos - origin point,
* /param  track - angle from orogin point to the calculated one (degrees 0-360)
* /param  distance - distance from origin point to the calculated one (meters)
* /param  newPos - new point 
*  return true when ok
*/
bool GpsPosition::movePosition (const GpsPosition& origPos, float pTrack, float pDistance, GpsPosition& newPos)
{
    return movePositionFlatEarth (origPos, pTrack, pDistance, newPos);
}

/**
* Function calculate coordinates of the new points. Is based on given point, track and distance. Flat Earth model.
* /param origPos - origin point,
* /param  track - angle from orogin point to the calculated one (degrees 0-360)
* /param  distance - distance from origin point to the calculated one (meters)
* /param  newPos - new point 
*  return true when ok
*/
bool GpsPosition::movePositionFlatEarth (const GpsPosition& origPos, float pTrack, float pDistance, GpsPosition& newPos)
{
    //  to radians conversion
    float trr = pTrack * DEG_2_RAD;

    float distLat = pDistance * cosf (trr);  // w metrach
    float distLon = pDistance * sinf (trr);

    return movePositionXY (origPos, distLon, distLat, newPos);
}


/**  
* Function calculate coordinates of the new points based on given point and distance to the East and North.
* /param origPos - origin point,
* /param distLon - distance in the East direction [m]
* /param distLat - distance in the North direction [m]
* /param newPos - new point
*/  
bool GpsPosition::movePositionXY (const GpsPosition& origPos, float distLon, float distLat, GpsPosition& newPos)
{
    static const float RD1 = DEG_2_RAD / scaleFactor;
    static const float DR3 = RAD_2_DEG * (scaleFactor / EARTH_RADIUS);
    static const float MAX_LAT = 89.0f * scaleFactor;

    //  Limitation to the latitude range <-89,+89>
    if (origPos._latitude > MAX_LAT || origPos._latitude < -MAX_LAT)
    {
        newPos = origPos;
        return false;;
    }

    //  latitude in radians
    float lat0 = static_cast<float>(origPos._latitude) * RD1;

    //  Meters conversion to degrees and scale
    int dLat = static_cast<int>(distLat * DR3);
    int dLon = static_cast<int>((distLon * DR3) / cosf (lat0));

    // new coordinates
    newPos._longitude = origPos._longitude + dLon;
    newPos._latitude = origPos._latitude + dLat;

    //  correctness control of the float
    bool ret1 = Numbers::assure(distLat, 0.0f);
    bool ret2 = Numbers::assure(distLon, 0.0f);
    return (ret1 && ret2);
}


/**
* Function returns distance vector between given two positions.
* /param fromPos - beggining position
* /param toPos - end position
* /param distLon - distance in the East direction [m]
* /param distLat - distance in the North direction [m]
*/
void GpsPosition::getVectorXY (const GpsPosition& fromPos, const GpsPosition& toPos, float& distLon, float& distLat)
{
    static const float RD1 = DEG_2_RAD / scaleFactor;
    static const float DM1 = DEG_2_RAD  * (EARTH_RADIUS / scaleFactor);

    // Latitude in radians
    float lat0 = static_cast<float>(fromPos._latitude) * RD1;

    float dLon = static_cast<float>(toPos._longitude - fromPos._longitude);
    float dLat = static_cast<float>(toPos._latitude - fromPos._latitude);

    // Degrees conversion to meters.
    distLon = (dLon * DM1) * cosf (lat0);
    distLat = dLat * DM1;
}

/**
* Function returns distance vector2 between given two positions.
* /param fromPos - beggining position
* /param toPos - end position
* /param distance - distance in the vector2 format: (lat,lon) [m]
*/
void GpsPosition::getVector2XY(const GpsPosition& fromPos, const GpsPosition& toPos, Vector2f& distance)
{
    static const float RD1 = DEG_2_RAD / scaleFactor;
    static const float DM1 = DEG_2_RAD * (EARTH_RADIUS / scaleFactor);

    // Latitude in radians
    float lat0 = static_cast<float>(fromPos._latitude) * RD1;

    float dLon = static_cast<float>(toPos._longitude - fromPos._longitude);
    float dLat = static_cast<float>(toPos._latitude - fromPos._latitude);

    // Degrees conversion to meters.
    Vector2f _dist(dLat * DM1, (dLon * DM1) * cosf(lat0));
    distance = _dist;
}

/**
* Calculate distance between two points (meters)
*/
float GpsPosition::distance (const GpsPosition& fromPos, const GpsPosition& toPos)
{
    float f = distanceFlatEarth (fromPos, toPos);
    // correctness control of the float
    Numbers::assure (f, 0.0f);

    return f;
}

/**
* Calculate distance between two points (meters)
*/
Vector2f GpsPosition::distanceVector (const GpsPosition& fromPos, const GpsPosition& toPos)
{
    Vector2f d;
    float f = distanceFlatEarth (fromPos, toPos);
    // correctness control of the float
    Numbers::assure (f, 0.0f);
    float tr = track(fromPos, toPos);
    d = Vector2f(f * cosf(Numbers::radians(tr)), f * sinf(Numbers::radians(tr)));
    return d;
}

/**
* Version that compute on the flat model of the Earth.
*  Source: http://williams.best.vwh.net/avform.htm#LL
*/
float GpsPosition::distanceFlatEarth (const GpsPosition& fromPos, const GpsPosition& toPos)
{
    static const float RD1 = DEG_2_RAD / scaleFactor;
    static const float RD2 = (EARTH_RADIUS * DEG_2_RAD) / scaleFactor;

    float dLon = static_cast<float>(toPos._longitude - fromPos._longitude);
    float dLat = static_cast<float>(toPos._latitude - fromPos._latitude);

    float lat0 = static_cast<float>(fromPos._latitude) * RD1;
	float cLat0 = cosf (lat0);
    float d = RD2 * sqrtf (dLat*dLat + dLon*dLon*cLat0*cLat0);

    return d;
}


/**
* Add to the given angle of track second angle and normalize the result to range (0-360) degrees
* /param baseTrack - base angle (0;360) degrees
* /param toAdd - component to be added (-180;+180) degrees
*/
float GpsPosition::addTrack (float baseTrack, float toAdd)
{
    float f = fmodf (baseTrack + toAdd + 360.0f, 360.0f);
        // correctness control of the float
    Numbers::assure (f, 0.0f);
    return f;
}


/**
* Calculate angle ("track1" and "track2") difference and normalize result to range (-180,+180) degrees
*/
float GpsPosition::subTrack (float track1, float track2, bool inRadians)
{
    float a = (inRadians) ? PI : 180.0f;

    float t = track1 - track2;
    if (t > a)
        t = t - 2.0f * a;
    else if (t < -a)
        t = t + 2.0f * a;

    // correctness control of the float
    Numbers::assure (t, 0.0f);

    return t;
}


/**
* Normalize angle to the range -180 to +180
*/
float GpsPosition::normalize180 (float pTrack, bool inRadians)
{
    float a = (inRadians) ? PI : 180.0f;

    // Convert angle to the range -360 to +360
    float b = fmodf (pTrack, 2.0f*a);
    if (b > a)
        b = b - 2.0f*a;
    else if (b < -a)
        b = b + 2.0f*a;

    // correctness control of the float
    Numbers::assure (b, 0.0f);

    return b;
}


/**
* Calculates coordinates of the predPos point based on current position, previous position and time factor.
* /param prevPos - previous coordinates
* /param curPos - current coordinates
* /param timeFactor - time factor  0- time sample curPos, 1- time sample after curPos sample (curPos-PrevPos)
*  predPos - new coordinates (prediction)
*/
void GpsPosition::predict (const GpsPosition& prevPos, const GpsPosition& curPos, float timeFactor, GpsPosition& predPos)
{
    static const int sc180 = 180 * scaleFactorI;

    float lat1 = static_cast<float>(curPos._latitude - prevPos._latitude) * timeFactor;
    // Do not make correction - around the Pole algorithms does not work.
    predPos._latitude = curPos._latitude + static_cast<int>(lat1);

    float lon1 = static_cast<float>(curPos._longitude - prevPos._longitude);
    //  A simplified way of ignoring the passage over the meridian + -180
    if (lon1 > scaleFactor)
        lon1 = 0.0f;
    predPos._longitude = curPos._longitude + static_cast<int>(lon1 * timeFactor);
    //  Meridian correction +-180
    if (predPos._longitude > sc180)
    {
        // There is overflow so substraction is made in two steps.
        predPos._longitude -= sc180;
        predPos._longitude -= sc180;
    }
    else if (predPos._longitude < -sc180)
    {
        // There is overflow so addition is made in two steps.
        predPos._longitude += sc180;
        predPos._longitude += sc180;
    }
}


/**
* Calculates distance (meters) from desired way. Desired way is defined by the track and points coordinates that it passing by.
* /param pointOnRoute - any point lying on the way
* /param fromPos - point in which calculations is made
* /param routeAngle - desired route angle. 
*  Positive value means deviation in right, negative in left.
*  formPos point could be located from either side of pointOnRoute point (does not change result of the function)
*/
float GpsPosition::crossTrackError (const GpsPosition& pointOnRoute, const GpsPosition& fromPos, float routeAngle)
{
    // Track from current point to the point on the route.
    float tr = track(fromPos, pointOnRoute);
    // Angles difference 
    float dtr = subTrack (routeAngle, tr);
    // Distance from current point to the point on the route.
    float dist = distance(fromPos, pointOnRoute);

    // Calculated deviation from track
    float xtd = dist * sinf(dtr * DEG_2_RAD);

     // correctness control of the float
    Numbers::assure (xtd, 0.0f);

    return xtd;
}
