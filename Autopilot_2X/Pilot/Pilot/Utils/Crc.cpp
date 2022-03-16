/**
*                                                                  
* @class Crc                                                               
*                                                                   
* @brief Class handling controll sum CRC used during transmision.
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

const char Crc::hexMap[] = "0123456789abcdef";

/**
* Calculates CRC for specified string.
* CRC is return as string hex (0000 - FFFF)  with prefix at beginning and space at the end.
* NOTE: outBuf buffer has at least 7 bytes length (prefix + CRC + space + 0)
*/
char* Crc::compute (const char* data, char* outBuf)
{
    INT16U crc = 0xffffu;    //  Wartoœæ pocz¹tkowa CRC
    size_t n = strlen(data);

    //  Calculate CRC
    for (size_t i=0u; i<n; i++)
	{
        crc = crcUpdate (crc, data[i]);
    }
  
    outBuf[0] = crcPrefix;
    outBuf[1] = hexMap[(crc >> 12) & 0x0fu];
    outBuf[2] = hexMap[(crc >> 8) & 0x0fu];
    outBuf[3] = hexMap[(crc >> 4) & 0x0fu];
    outBuf[4] = hexMap[crc & 0x0fu];
    outBuf[5] = ' ';
    outBuf[6] = 0;

    return (outBuf + 6);
}


/**
* Calculate CRC for binary data from defined buffer.
*/
INT16U Crc::computeBin (const unsigned char* data, int len)
{
    INT16U crc = 0xffffu;    // Start CRC value

    //  Calculate CRC
    for (int i=0; i<len; i++)
	{
        crc = crcUpdate (crc, data[i]);
    };

    return crc;
}

/**
* Modify defined CRC including new byte.
* /param crc - previous CRC
* /param c - new byte to add
*/
INT16U Crc::crcUpdate (INT16U crc, INT8U c)
{
    // All implicit type conversions have been eliminated

    c ^= static_cast<INT8U>(crc & 0xffu);
    c ^= static_cast<INT8U>((c << 4) & 0xffu);

    INT16U d1 = static_cast<INT16U>(static_cast<INT16U>(c) << 8);
    INT16U d2 = static_cast<INT16U>(crc >> 8);
    INT16U e  = static_cast<INT16U>(c >> 4);
    INT16U f  = static_cast<INT16U>(c << 3);

    crc = static_cast<INT16U>(static_cast<INT16U>(static_cast<INT16U>(d1 | d2) ^ e) ^ f);
    
    return crc;
}
