/**
*                                                                   
* @class GpsData                                                           
*                                                                   
* Stores data that has been read from GPS (position, speed etc)
* Parsing NMEA sequence, normalize obtained data.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef DGPSDATA_H
#define DGPSDATA_H

class DGpsInterface
{
public:
    DGpsInterface(void)
    :   tmpData(), currData(), virtData(), predictData(),
        virtDist(0.0f), numSats(0), PDOP(0.0f),
        prevBaroAlt(0.0f), parseError(false), userTime(0)
        {};

    class DGpsData
    {
    public:
        class DGpsRawData
        {
        public:
            DGpsRawData()
            :   north(0.0f), east(0.0f), up(0.0f)
                {};
            float north;    // [meter]
            float east;     // [meter]
            float up;       // [meter]
        };
        
        class DGpsMeasureData
        {
        public:
            DGpsMeasureData()
            :   lonRange(0.0f), track(0.0f), altitude(0.0f)
                {};
            float lonRange;  // [meter]
            float track;     // [rad]
            float altitude;  // [meter]
        };
    
        DGpsData()
        :   rawData(), measureData()
            {};
        
        DGpsRawData     rawData;
        DGpsMeasureData measureData;
        
        void compute (void);
    };

    DGpsData tmpData;
    DGpsData currData;
    DGpsData virtData;
    DGpsData predictData;
    float virtDist;
    int   numSats;
    float PDOP;
    
    float prevBaroAlt;
    bool  parseError;         ///< Parse error occured all lines are invalid
    int   userTime;
        
    virtual void resetData(void) = 0;
    virtual bool parseNmea(const char* nmeaLine) = 0;
    void prediction (float gpsDistance, float gpsTrack, float altitude);
    
    bool parseDgpsNorth (const char* pNorth);
    bool parseDgpsEast  (const char* pEast);
    bool parseDgpsUp    (const char* pUp);
    
    bool parseDgpsNumSatellite (const char* pNumSat);
    bool parseDgpsPDOP (const char* pPDOP);
};

class Trimble: public DGpsInterface
{
public:
	enum VgkModeCode
    {
        INVALID        = 0,
        AUTO_FIX       = 1,
		RTK_FLOAT      = 2,
		RTK_FIX        = 3,
		DGPS           = 4,
		SBAS           = 5,
		RTK_FLOAT_3D   = 6,
		RTK_FIX_3D     = 7,
		RTK_FLOAT_2D   = 8,
		RTK_FIX_2D     = 9,
		OMNISTAR_HP_XP = 10,
		OMNISTAR_VBS   = 11,
        RTK_LOCATION   = 12
    };

	virtual void resetData (void);
    virtual bool parseNmea (const char* nmeaLine);

	VgkModeCode vgkMode;

private:
	bool parseDgpsVgkMode (const char* pVgkMode);	
};

class Novatel: public DGpsInterface
{
public:
    enum PosTypeCode
    {
        NONE                 = 0,
        FIXEDPOS             = 1,
        FIXEDHEIGHT          = 2,
        DOPPLER_VELOCITY     = 8,
        SINGLE               = 16,
        PSRDIFF              = 17,
        WAAS                 = 18,
        PROPAGATED           = 19,
        OMNISTAR             = 20,
        L1_FLOAT             = 32,
        IONOFREE_FLOAT       = 33,
        NARROW_FLOAT         = 34,
        L1_INT               = 48,
        NARROW_INT           = 50,
        OMNISTAR_HP          = 64,
        OMNISTAR_XP          = 65,
        PPP_CONVERGING       = 68,
        PPP                  = 69,
        OPERATIONAL          = 70,
        WARNING              = 71,
        OUT_OF_BOUNDS        = 72,
        PPP_BASIC_CONVERGING = 77,
        PPP_BASIC            = 78
    };
    
    virtual void resetData (void);
    virtual bool parseNmea (const char* nmeaLine);

    PosTypeCode posType;

private:
    bool parseDgpsPosType (const char* pPosType);   
};

#endif  // DGPSDATA_H
