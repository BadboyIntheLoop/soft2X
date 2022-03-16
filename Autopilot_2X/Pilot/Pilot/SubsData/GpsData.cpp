/*
*                                                                   
* @class GpsData                                                           
*                                                                   
* Stores data that has been read from GPS (position, speed etc)
* Parsing NMEA sequence, normalize obtained data.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

//  constructor
GpsData::GpsData(void)  :
amsl(0.0f), groundSpeed(0.0f), track(0.0f), utcHour(0), utcMin(0), utcSec(0), utcMsec(0),
day(0), month(0), year(0), hDop(0.0f), vDop(0.0f), userTime(0)
{
    resetData();
}


/**
* Set invalid data flags.
*/
void GpsData::resetData(void)
{
    navMode = GpsData::NAV_MODE_NO_VALID_DATA;    // flaga braku danych z GPS
    amslPresent = false;
    trackPresent = false;
    datePresent = false;
    timePresent = false;
    parseError = false;
}


/**
* Parsing received NMEA 2.3 line
* Returns true when there was not parsing error.
* Data could be invalid.
*/
bool GpsData::parseNmea(const char* nmeaLine)
{
    const int MAX_TOKENS = 30;
    char*     tokens[MAX_TOKENS];
    int       nTokens = 0;

    parseError = true;
    
    // Divide NMEA line to tokens
    char tmpLine[LINESIZE];
    int len = strlen(nmeaLine);
    MEMCCPY (tmpLine, nmeaLine, 0, LINESIZE);

    tokens[nTokens++] = tmpLine;
    for (int i=0; i<len; i++)
    {
        if ((tmpLine[i] == ',' || tmpLine[i] == '*') && nTokens < MAX_TOKENS)
        {
            tmpLine[i] = 0;
            tokens[nTokens++] = tmpLine + i + 1;
        }
    }

    //Message Structure:
    //$GPRMC,hhmmss,status,latitude,N,longitude,E,spd,cog,ddmmyy,mv,mvE,mode*cs<CR><LF>
    //Example:
    //$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A*57
    if (STRICMP (tokens[0], "$GPRMC") == 0)
    {
        if (nTokens != 14)
        {
            Log.errorPrintf("GpsData_parseNmea_3");
            return false;
        }

        //  Time - or meesages in the calling function
        //  Time may occur even when data are invalid (from external timer)
        if (!parseGpsTime(tokens[1]))
            return false;

        //  Date
        //  Date may occur even when data are invalid (from external timer)
        if (!parseGpsDate(tokens[9]))
            return false;

        //  If data are invalid exit without message (it is not an error)
        if (tokens[2][0] != 'A')
            return false;

        //  Position
        if (!parseGpsPosition(tokens[3], tokens[4],
            tokens[5], tokens[6]))
            return false;

        //  Speed
        if (!parseGpsSpeed(tokens[7]))
            return false;

        //  Track
        if (!parseGpsTrack(tokens[8]))
            return false;
    }

    //Message Structure:
    //$GPGGA,hhmmss.ss,Latitude,N,Longitude,E,FS,NoSV,HDOP,msl,m,Altref,m,DiffAge,DiffStation*cs<CR><LF>
    //Example:
    //$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,8,1.01,499.6,M,48.0,M,,0*5B
    else if (STRICMP (tokens[0], "$GPGGA") == 0)
    {
        if (nTokens != 16)
        {
            Log.errorPrintf("GpsData_parseNmea_4");
            return false;
        }

        //  If data are invalid exit without message (it is not an error)
        if (tokens[6][0] < '1')
            return false;

        //  Altitude
        if (!parseGpsAltitude(tokens[9]))
            return false;
    }

    //Message Structure:
    //$GPGSA,Smode,FS{,sv},PDOP,HDOP,VDOP*cs<CR><LF>
    //Example:
    //$GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54*0D
    else if (STRICMP (tokens[0], "$GPGSA") == 0)
    {
        if (nTokens != 19)
        {
            Log.errorPrintf("GpsData_parseNmea_5");
            return false;
        }

        //  If data are invalid exit without message (it is not an error)
        if (tokens[2][0] < '2')
            return false;

        //  NavMode
        if (!parseGpsNavMode(tokens[2]))
            return false;

        //  HDOP (Horizontal Dilution of precision)
        if (!parseGpsHDop(tokens[16]))
            return false;

        //  VDOP (Vertical Dilution of precision)
        if (!parseGpsVDop(tokens[17]))
            return false;
    }

    else if (STRICMP (tokens[0], "$GPTXT") == 0)
    {
        Log.msgPrintf(nmeaLine);
    }

    parseError = false;
    return true;
}


/**
* The linear extrapolation is made of the selected object fields at time based on current and previous values​​.
* /param prev - previous data
* /param curr - current data
* /param predTime -time for which define values
* Object fields must be filed with "curr" object elements before calling function.
* Return false when "prev" object has invalid data. Do not delete prev and curr data.
*/
bool GpsData::predict (const GpsData &prev, const GpsData &curr, int predTime)
{
    if (prev.parseError ||
        prev.navMode == GpsData::NAV_MODE_NO_VALID_DATA ||
        prev.navMode == GpsData::NAV_MODE_UNKNOWN)
        return false;

    float w1 = static_cast<float>(curr.userTime - prev.userTime);
    float w2 = static_cast<float>(predTime - curr.userTime);
    float w3 = (w1 == 0.0f) ? 1.0f : w2/w1;

    // During the normal work w3 could be up to 1.4 due to asynchronizm.
    // In case of larger discrepanciespredition is turned on.
    if (w3 > 2.0f)
        return false;
            
    //  track
    if (curr.trackPresent && prev.trackPresent)
    {
        float sbt = GpsPosition::subTrack (curr.track, prev.track);
        track = GpsPosition::addTrack(curr.track, sbt * w3);
    }

    //  position
    GpsPosition::predict (prev.position, curr.position, w3, position); 
    
    // groundspeed
    groundSpeed = curr.groundSpeed + (curr.groundSpeed - prev.groundSpeed) * w3;
    if (groundSpeed < 0.0f)
        groundSpeed = 0.0f;

    return true;
}


/**
* Parse string that consist time in NMEA format.
*/
bool GpsData::parseGpsTime(const char* time)
{
    char buf[3] = {0};

    // Lenght control could be null string
    if (strlen(time) < 1)
        return true;

	//  Hours
    buf[0] = time[0]; buf[1] = time[1];
    if (!TypeParser::toInt(buf, utcHour))
    {
        Log.errorPrintf("GpsData_parseGpsTime_2");
        return false;
    }

    //  Minutes
    buf[0] = time[2]; buf[1] = time[3];
    if (!TypeParser::toInt(buf, utcMin))
    {
        Log.errorPrintf("GpsData_parseGpsTime_3");
        return false;
    }

    //  Seconds
    buf[0] = time[4]; buf[1] = time[5];
    if (!TypeParser::toInt(buf, utcSec))
    {
        Log.errorPrintf("GpsData_parseGpsTime_4");
        return false;
    }

    //  Miliseconds - omnit dot.
    buf[0] = time[7]; buf[1] = time[8];
    if (!TypeParser::toInt(buf, utcMsec))
    {
        Log.errorPrintf("GpsData_parseGpsTime_5");
        return false;
    }
    
    utcMsec *= 10;  // scaling to ms
    timePresent = true;
    
    return true;
}

/**
* Parse string that consist date in NMEA format.
*/
bool GpsData::parseGpsDate(const char* date)
{
    char buf[3] = {0};

    // Lenght control could be null string
    if (strlen(date) < 1)
        return true;

    //  Day
    buf[0] = date[0]; buf[1] = date[1];
    if (!TypeParser::toInt(buf, day))
    {
        Log.errorPrintf("GpsData_parseGpsDate_2");
        return false;
    }

    //  Month
    buf[0] = date[2]; buf[1] = date[3];
    if (!TypeParser::toInt(buf, month))
    {
        Log.errorPrintf("GpsData_parseGpsDate_3");
        return false;
    }

    //  Year
    buf[0] = date[4]; buf[1] = date[5];
    if (!TypeParser::toInt(buf, year))
    {
        Log.errorPrintf("GpsData_parseGpsDate_4");
        return false;
    }

    datePresent = true;
    
    return true;
}


/**
* Strings consiting position in NMEA format parsing.
* /param lat - latitude
* /param latSign - "N" or "S"
* /param lon - longitude
* /param lonSign - "E" or "W"
*/
bool GpsData::parseGpsPosition (const char* lat, const char* latSign,
        const char* lon, const char* lonSign)
{
    char buf1[3] = {0};
    double deg = 0.0, minutes = 0.0;

    // Lenght control could be null string
    if (strlen(lat) < 4)
    {
        Log.errorPrintf("GpsData_parseGpsPosition_1");
        return false;
    }

    //  latitude - whole degrees
    buf1[0] = lat[0]; buf1[1] = lat[1];
    if (!TypeParser::toDouble(buf1, deg))
    {
        Log.errorPrintf("GpsData_parseGpsPosition_2");
        return false;
    }

    //  latitude - minutes fractional
    if (!TypeParser::toDouble(lat+2, minutes))
    {
        Log.errorPrintf("GpsData_parseGpsPosition_3");
        return false;
    }

    //  latitude - calculate fractional degrees and sign.
    deg += minutes * MIN_2_DEG;
    if (latSign[0] == 'S')
        deg *= -1.0;
    else if (latSign[0] != 'N')
    {
        Log.errorPrintf("GpsData_parseGpsPosition_4");
        return false;
    }
    position.setLat(deg);


    //  String control - could not be empty because previously there was a altitude.
    if (strlen(lon) < 4)
    {
        Log.errorPrintf("GpsData_parseGpsPosition_5");
        return false;
    }

    //  longitude - whole degrees
    char buf2[4] = {0};
    buf2[0] = lon[0]; buf2[1] = lon[1]; buf2[2] = lon[2];
    if (!TypeParser::toDouble(buf2, deg))
    {
        Log.errorPrintf("GpsData_parseGpsPosition_6");
        return false;
    }

    //  longitude - minutes fractional
    if (!TypeParser::toDouble(lon+3, minutes))
    {
        Log.errorPrintf("GpsData_parseGpsPosition_7");
        return false;
    }

    //  longitude - calculate fractional degrees and sign.
    deg += minutes * MIN_2_DEG;
    if (lonSign[0] == 'W')
        deg *= -1.0;
    else if (lonSign[0] != 'E')
    {
        Log.errorPrintf("GpsData_parseGpsPosition_8");
        return false;
    }
    position.setLon(deg);

    return true;
}


/**
* Parsing string consisting speed in NMEA format.
* NMEA speed is in knots. Function makes the conversion to the kph.
*/
bool GpsData::parseGpsSpeed (const char* speed)
{
    float ts = 0.0f;

    // Lenght control could be null string
    if (strlen(speed) < 1)
    {
        Log.errorPrintf("GpsData_parseGpsSpeed_1");
        return false;
    }

    if (!TypeParser::toFloat(speed, ts))
    {
        Log.errorPrintf("GpsData_parseGpsSpeed_2");
        return false;
    }

    groundSpeed = ts * KNOTS_2_KPH;

    return true;
}

/**
* Parsing string consisting track angle in NMEA format. 
* That value could be omnited in valid NMEA line (at small speed).
* If it exist flag "trackPresent" has been set.
*/
bool GpsData::parseGpsTrack (const char* trackStr)
{
    // Lenght control could be null string
    if (strlen(trackStr) < 1)
        return true;

    if (!TypeParser::toFloat(trackStr, track))
    {
        Log.errorPrintf("GpsData_parseGpsTrack_1");
        return false;
    }

    trackPresent = true;

    return true;
}

/**
* Parsing string consisting altitude in NMEA format. 
* That value could be omnited in valid NMEA line.
* If it exist flag "amslPresent" has been set.
*/
bool GpsData::parseGpsAltitude (const char* alt)
{
    // Lenght control could be null string
    if (strlen(alt) < 1)
        return true;

    if (!TypeParser::toFloat(alt, amsl))
    {
        Log.errorPrintf("GpsData_parseGpsAltitude_1");
        return false;
    }

    amslPresent = true;

    return true;
}


/**
* Parsing string consisting Navigation Mode in NMEA format. 
*/
bool GpsData::parseGpsNavMode (const char* nm)
{
    // Lenght control could be null string
    if (strlen(nm) < 1)
        return false;

    int tmp = 0;
    if (!TypeParser::toInt(nm, tmp))
    {
        Log.errorPrintf("GpsData_parseGpsNavMode_1");
        return false;
    }
    navMode = static_cast<GpsData::NavModeCode>(tmp);

    return true;
}

/**
* Parsing string consisting HDOP in NMEA format. 
*/
bool GpsData::parseGpsHDop (const char* hDopStr)
{
    // Lenght control could be null string
    if (strlen(hDopStr) < 1)
        return false;

    if (!TypeParser::toFloat(hDopStr, hDop))
    {
        Log.errorPrintf("GpsData_parseGpsHDop_1");
        return false;
    }

    return true;
}

/**
* Parsing string consisting VDOP in NMEA format. 
*/
bool GpsData::parseGpsVDop (const char* vDopStr)
{
    // Lenght control could be null string i 2D mode
    if (strlen(vDopStr) < 1)
    {
        vDop = 0.0f;
        return true;
    }

    if (!TypeParser::toFloat(vDopStr, vDop))
    {
        Log.errorPrintf("GpsData_parseGpsVDop_1");
        return false;
    }

    return true;
}

