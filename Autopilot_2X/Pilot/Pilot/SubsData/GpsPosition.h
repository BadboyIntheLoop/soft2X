/**
*                                                                   
* @clacc GpsPosition                                                       
*                                                                   
* Class stores geographical position and makes operations related to the position calculations.                                  
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef GPSPOSITION_H
#define GPSPOSITION_H


class GpsPosition
{
public:
    typedef void (GpsPosition::*setDblFun)(double);      // Types specifies pointers to functions
    typedef void (GpsPosition::*getDblFun)(double&) const;

    GpsPosition(void): _latitude(0), _longitude(0) {};

    void setLat (double lat);
    void setLon (double lon);

    void getLat (double &lat) const;
    void getLon (double &lon) const;

    double getLat (void) const;
    double getLon (void) const;

    void getPosTlm (INT32S &lat, INT32S &lon) const;

    bool isEqual (const GpsPosition &pos) const;

    static float track (const GpsPosition& fromPos, const GpsPosition& toPos);
    static int32_t track_cd(const GpsPosition& fromPos, const GpsPosition& toPos);
    static float track (const GpsPosition& fromPos, const GpsPosition& toPos,float circleRadius, bool circleLeft, int circleMode, float circleModePar);
    static bool movePosition (const GpsPosition& origPos, float track, float distance, GpsPosition& newPos);
    static bool movePositionXY (const GpsPosition& origPos, float distLon, float distLat, GpsPosition& newPos);
    static void getVectorXY (const GpsPosition& fromPos, const GpsPosition& toPos, float& distLon, float& distLat);
    static void getVector2XY(const GpsPosition& fromPos, const GpsPosition& toPos, Vector2f& distance);
    static float distance (const GpsPosition& fromPos, const GpsPosition& ToPos);
    static Vector2f distanceVector (const GpsPosition& fromPos, const GpsPosition& ToPos);
    static float addTrack (float baseTrack, float toAdd);
    static float subTrack (float track1, float track2, bool inRadians=false);
    static float normalize180 (float track, bool inRadians=false);
    static void predict (const GpsPosition& prevPos, const GpsPosition& curPos, float timeFactor, GpsPosition &predPos);
    static float crossTrackError (const GpsPosition& pointOnRoute, const GpsPosition& fromPos, float routeAngle);

private:
    static const float scaleFactor;     // scale factor for internal use
    static const int scaleFactorI;      //  scale factor for internal use - used in expressions with int type to avoid conversion
    static const double scaleFactorD;   //  scale factor for internal use - used in expressions with double type to avoid conversion
    static const float EARTH_RADIUS;    //  Earth radius in meters.

    static float trackFlatEarth (const GpsPosition& fromPos, const GpsPosition& toPos);

    static bool movePositionFlatEarth (const GpsPosition& origPos, float track, float distance, GpsPosition& newPos);

    static float distanceFlatEarth (const GpsPosition& fromPos, const GpsPosition& toPos);

    int _latitude;      // degrees * 10000000
    int _longitude;     // degrees * 10000000
};

#endif  // GPSPOSITION_H
