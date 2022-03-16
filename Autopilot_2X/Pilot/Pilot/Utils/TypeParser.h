/**
*                                                                   
* @class TypeParser                                                        
*                                                                   
* @brief Class shares static functions to text data parsing to selected data types.                                       
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef TYPEPARSER_H
#define TYPEPARSER_H

class GpsPosition;

class TypeParser
{
public:
    static bool toFloat (const char* text, float& fval);
    static bool toDouble (const char* text, double& dval);
    static bool toInt (const char* text, int& ival);
    static bool toGpsPosition (const char* p1, const char* p2, GpsPosition& pos);
    static bool toGpsPositionRel (const char* dist1, const char* dist2, const GpsPosition& base, GpsPosition& pos);
    static bool toStr (int val, char* buf, int bufsize);

private:
    //  Can not make objects of this class
    TypeParser (void);
};

#endif  // TYPEPARSER_H
