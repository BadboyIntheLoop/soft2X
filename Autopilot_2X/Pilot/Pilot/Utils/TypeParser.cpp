/**
*                                                                   
* @class TypeParser                                                        
*                                                                   
* @brief Class shares static functions to text data parsing to selected data types.                                       
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

/**
* Function converting number in text format to float value.
* /param text - number in text format (dot . as decimal point)
* /param fval - returned value
* In case of an error fval is undefined.
*/
bool TypeParser::toFloat (const char* text, float& fval)
{
    char* ptr=0;

    if (text != NULL)
    {
        fval = static_cast<float>(strtod (text, &ptr));

		// The function strtod in 2 parameter returns a pointer to the location that was interrupted by the conversion.
		// If there is a 0 it means that the conversion is faultless.
        if (*ptr == 0)
            return true;
    }

    return false;
}

/**
* Function converting number in text format to double value.
* /param text - number in text format (dot . as decimal point)
* /param dval - returned value
* In case of an error dval is undefined.
*/
bool TypeParser::toDouble (const char* text, double& dval)
{
    char* ptr=0;

    if (text != NULL)
    {
        dval = strtod (text, &ptr);
		// The function strtod in 2 parameter returns a pointer to the location that was interrupted by the conversion.
		// If there is a 0 it means that the conversion is faultless.
        if (*ptr == 0)
            return true;
    }

    return false;
}

/**
* Function converting number in text format to int value.
* /param text - number in text format
* /param ival - returned value
* In case of an error ival is undefined.
*/
bool TypeParser::toInt (const char* text, int& ival)
{
    char* ptr=0;

    if (text != NULL)
    {
        ival = static_cast<int>(strtol (text, &ptr, 10));
		// The function strtod in 2 parameter returns a pointer to the location that was interrupted by the conversion.
		// If there is a 0 it means that the conversion is faultless.
        if (*ptr == 0)
            return true;
    }

    return false;
}


/**
* Converts geographicas coordinates as a text to the GpsPosition type.
* Latitude and longitude is given as degres (float) ended with N,E,W,S (without space).
* /param p1, p2 - Latitude and longitude as text.
* /param pos - reference to the object which fields will be set
* In case of an error pos is undefined.
*/
bool TypeParser::toGpsPosition (const char* p1, const char* p2, GpsPosition& pos)
{
    static const int BUFSIZE = 20;

    if (p1 == NULL || p2 == NULL)
        return false;

    // Arguments copy prevent destroying
    char p1c[BUFSIZE], p2c[BUFSIZE];
    MEMCCPY (p1c, p1, 0, BUFSIZE-1); p1c[BUFSIZE-1] = 0;
    MEMCCPY (p2c, p2, 0, BUFSIZE-1); p2c[BUFSIZE-1] = 0;

    // read and delete last letter
    char p1sfx = static_cast<char>(toupper (p1c[strlen(p1c)-1])); p1c[strlen(p1c)-1] = 0;
    char p2sfx = static_cast<char>(toupper (p2c[strlen(p2c)-1])); p2c[strlen(p2c)-1] = 0;

    char *ptr1=0, *ptr2=0;

    double d1 = strtod (p1c, &ptr1);
    double d2 = strtod (p2c, &ptr2);

    // Number has to be correct
    if (*ptr1 != 0 || *ptr2 != 0)
        return false;

    // Numbers have to be positive
    if (d1 < 0.0 || d2 < 0.0)
        return false;

    //  Arguments could be in any order
    if ((p1sfx == 'N' || p1sfx == 'S') && (p2sfx == 'E' || p2sfx == 'W'))
    {
        if (p1sfx == 'S')
            d1 = -d1;
        if (p2sfx == 'W')
            d2 = -d2;

        pos.setLat (d1);
        pos.setLon (d2);
    }
    else if ((p1sfx == 'E' || p1sfx == 'W') && (p2sfx == 'N' || p2sfx == 'S'))
    {
        if (p1sfx == 'W')
            d1 = -d1;
        if (p2sfx == 'S')
            d2 = -d2;

        pos.setLon (d1);
        pos.setLat (d2);
    }
    else
        return false;

    return true;
}


bool TypeParser::toGpsPositionRel (const char* dist1, const char* dist2, const GpsPosition& base, GpsPosition& pos)
{
    static const int BUFSIZE = 20;

    if (dist1 == NULL || dist2 == NULL)
        return false;

    // Arguments copy prevent destroying
    char d1c[BUFSIZE], d2c[BUFSIZE];
    MEMCCPY (d1c, dist1, 0, BUFSIZE-1); d1c[BUFSIZE-1] = 0;
    MEMCCPY (d2c, dist2, 0, BUFSIZE-1); d2c[BUFSIZE-1] = 0;

    // read and delete last letter
    char d1sfx = static_cast<char>(toupper (d1c[strlen(d1c)-1])); d1c[strlen(d1c)-1] = 0;
    char d2sfx = static_cast<char>(toupper (d2c[strlen(d2c)-1])); d2c[strlen(d2c)-1] = 0;

    char *ptr1=0, *ptr2=0;

    int d1i = static_cast<int>(strtod (d1c, &ptr1));
    int d2i = static_cast<int>(strtod (d2c, &ptr2));

    // Numbers have to be correct
    if (*ptr1 != 0 || *ptr2 != 0)
        return false;

    // Numbers have to be positive
    if (d1i < 0 || d2i < 0)
        return false;

    //  Arguments could be in any order
    if ((d1sfx == 'N' || d1sfx == 'S') && (d2sfx == 'E' || d2sfx == 'W'))
    {
        if (d1sfx == 'S')
            d1i = -d1i;
        if (d2sfx == 'W')
            d2i = -d2i;

        GpsPosition::movePositionXY (base, static_cast<float>(d2i), static_cast<float>(d1i), pos);
    }
    else if ((d1sfx == 'E' || d1sfx == 'W') && (d2sfx == 'N' || d2sfx == 'S'))
    {
        if (d1sfx == 'W')
            d1i = -d1i;
        if (d2sfx == 'S')
            d2i = -d2i;

        GpsPosition::movePositionXY (base, static_cast<float>(d1i), static_cast<float>(d2i), pos);

    }
    else
        return false;

    return true;
}

/**
* Convert number to text.
* /param  val - value to be converted
* /param  buf - output buffer
* /param  bufsize - size of a buffer
* In case of an error buffer is undefined.
*/
bool TypeParser::toStr (int val, char* buf, int bufsize)
{
    int pos = 0;
    int begDig = 0;

    //  Minimum size of buffer
    if (bufsize <= 3 )
        return false;

    if (val < 0)
    {
        buf[pos++] = '-';
        val = abs (val);
        begDig = 1;     //  Beginning of number (skip -)
    }

    //  0 has to be separate serviced 
    if (val == 0)
        buf[pos++] = '0';

    while (val > 0 && pos < bufsize-1)
    {
        char rem = static_cast<char>(val % 10);
        buf[pos++] = '0' + rem;
        val /= 10;
    }

    //  End of string 
    buf[pos] = '\0';

    //  Change order of digits (skip -) 
    int j = strlen (buf) - 1;
    for (int i = begDig; i < j; i++, j--)
    {
        char c = buf[i];
        buf[i] = buf[j];
        buf[j] = c;
    }

    //  If buffer was to small return false.
    if (val == 0)
        return true;

    return false;
}
