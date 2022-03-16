/**
*                                                                  
* @class Crc                                                               
*                                                                   
* @brief Class handling controll sum CRC used during transmision.
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#ifndef CRC_H
#define CRC_H

class Crc
{
public:
    static const char crcPrefix = '%';

    static char*  compute (const char* data, char* outBuf);
    static INT16U computeBin (const unsigned char* data, int len);
private:
    static const char hexMap[];

    static INT16U crcUpdate (INT16U crc, INT8U c);
};

#endif  //CRC_H
