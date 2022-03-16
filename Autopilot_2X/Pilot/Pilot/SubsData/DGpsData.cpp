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

void DGpsInterface::DGpsData::compute(void)
{    
    measureData.lonRange = sqrtf(rawData.east * rawData.east + rawData.north * rawData.north);
    
    measureData.track = atan2f(rawData.east, rawData.north);
    measureData.track = fmodf(measureData.track + 3.0f * PI, 2.0f * PI);
    
    measureData.altitude = rawData.up; 
}

void DGpsInterface::prediction(float gpsDistance, float gpsTrack, float altitude)
{
    predictData.rawData.east  = currData.rawData.east  - gpsDistance * sinf(gpsTrack);
    predictData.rawData.north = currData.rawData.north - gpsDistance * cosf(gpsTrack);
    
    predictData.rawData.up = predictData.rawData.up * (1 + (altitude - prevBaroAlt)/prevBaroAlt);
    if ((predictData.rawData.up < 0.1f) && (predictData.rawData.up > (-0.1f)))
    {
        if (predictData.rawData.up > 0.0f)  predictData.rawData.up = 0.1f;
        else predictData.rawData.up = -0.1f;
    }
    prevBaroAlt = altitude;
    predictData.compute();
    currData = predictData;
}

bool DGpsInterface::parseDgpsEast(const char* pEast)
{
    float ts = 0.0f;

    // Lenght control could be null string
    if (strlen(pEast) < 1)
    {
        Log.errorPrintf("DGps_parseEast_1");
        return false;
    }

    if (!TypeParser::toFloat(pEast, ts))
    {
        Log.errorPrintf("DGps_parseEast_2");
        return false;
    }

    tmpData.rawData.east = ts;
    
    return true;
}

bool DGpsInterface::parseDgpsNorth(const char* pNorth)
{
    float ts = 0.0f;

    // Lenght control could be null string
    if (strlen(pNorth) < 1)
    {
        Log.errorPrintf("DGps_parseNorth_1");
        return false;
    }

    if (!TypeParser::toFloat(pNorth, ts))
    {
        Log.errorPrintf("DGps_parseNorth_2");
        return false;
    }

    tmpData.rawData.north = ts;
    
    return true;
}

bool DGpsInterface::parseDgpsUp(const char* pUp)
{
    float ts = 0.0f;

    // Lenght control could be null string
    if (strlen(pUp) < 1)
    {
        Log.errorPrintf("DGps_parseUp_1");
        return false;
    }

    if (!TypeParser::toFloat(pUp, ts))
    {
        Log.errorPrintf("DGps_parseUp_2");
        return false;
    }

    tmpData.rawData.up = ts;
    
    return true;
}

bool DGpsInterface::parseDgpsNumSatellite (const char* pNumSat)
{
    // Lenght control could be null string
    if (strlen(pNumSat) < 1)
    {
        Log.errorPrintf("DGps_parseNumSatellite_1");
        return false;
    }

    int tmp = 0;
    if (!TypeParser::toInt(pNumSat, tmp))
    {
        Log.errorPrintf("DGps_parseNumSatellite_2");
        return false;
    }

    numSats = tmp;

    return true;
}

bool DGpsInterface::parseDgpsPDOP (const char* pPDOP)
{
    // Lenght control could be null string
    if (strlen(pPDOP) < 1)
    {
        Log.errorPrintf("DGps_parsePDOP_1");
        return false;
    }

    float tmp = 0.0f;
    if (!TypeParser::toFloat(pPDOP, tmp))
    {
        Log.errorPrintf("DGps_parsePDOP_2");
        return false;
    }

    PDOP = tmp;

    return true;
}

/**
* Trimble DGps module
*/

/**
* Set invalid data flags.
*/
void Trimble::resetData(void)
{
    vgkMode = INVALID;
    parseError = false;
}

/**
* Parsing received NMEA 2.3 line
* Returns true when there was not parsing error.
* Data could be invalid.
*/
bool Trimble::parseNmea(const char* nmeaLine)
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
    
    if ((STRICMP (tokens[0], "$PTNL") == 0 && STRICMP (tokens[1], "VGK") == 0))
    {        
        if (nTokens != 12)
        {
            Log.errorPrint ("Trimble_parserNmea_1");
            return false;
        }
        
        if (!parseDgpsEast(tokens[4]))
            return false;

        if (!parseDgpsNorth(tokens[5]))
            return false;

        if (!parseDgpsUp(tokens[6]))
            return false;
            
        if (!parseDgpsVgkMode(tokens[7]))
            return false;
            
        if (!parseDgpsNumSatellite(tokens[8]))
            return false;
            
        if (!parseDgpsPDOP(tokens[9]))
            return false;
    }
    
    tmpData.compute();

    parseError = false;
    return true;
}

bool Trimble::parseDgpsVgkMode (const char* pVgkMode)
{    
    // Lenght control could be null string
    if (strlen(pVgkMode) < 1)
    {
        Log.errorPrintf("Trimble_parserVgkMode_1");
        return false;
    }

    int tmp = 0;
    if (!TypeParser::toInt(pVgkMode, tmp))
    {
        Log.errorPrintf("Trimble_parserVgkMode_2");
        return false;
    }

    vgkMode = static_cast<Trimble::VgkModeCode>(tmp);

    return true;
}

/**
* Novatel DGps module
*/

/**
* Set invalid data flags.
*/
void Novatel::resetData(void)
{
    posType = NONE;
    parseError = false;
}

/**
* Parsing received NMEA 2.3 line
* Returns true when there was not parsing error.
* Data could be invalid.
*/
bool Novatel::parseNmea(const char* nmeaLine)
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
        if ((tmpLine[i] == ',' || tmpLine[i] == '*' || tmpLine[i] == ';') && nTokens < MAX_TOKENS)
        {
            tmpLine[i] = 0;
            tokens[nTokens++] = tmpLine + i + 1;
        }
    }
    
    if (STRICMP (tokens[0], "#ALIGNBSLNENUA") == 0)
    {        
        if (nTokens != 30)
        {
            Log.errorPrint ("Novatel_parserNmea_1");
            return false;
        }
        
        if (!parseDgpsPosType(tokens[11]))
            return false;
        
        if (!parseDgpsEast(tokens[12]))
            return false;

        if (!parseDgpsNorth(tokens[13]))
            return false;

        if (!parseDgpsUp(tokens[14]))
            return false;
            
        if (!parseDgpsNumSatellite(tokens[20]))
            return false;
    }
    
    tmpData.compute();

    parseError = false;
    return true;
}

bool Novatel::parseDgpsPosType (const char* pPosType)
{    
    // Lenght control could be null string
    if (strlen(pPosType) < 1)
    {
        Log.errorPrintf("Novatel_parserPosType_1");
        return false;
    }

    int tmp = 0;
    if (!TypeParser::toInt(pPosType, tmp))
    {
        Log.errorPrintf("Novatel_parserPosType_2");
        return false;
    }

    posType = static_cast<Novatel::PosTypeCode>(tmp);

    return true;
}

