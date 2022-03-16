/**
*                                                                   
* @class Base64                                                            
*                                                                  
* @brief Class controlling encoding and decoding in Base64 standard
*                                                                   
* 2008-2012 Witold Kruczek @ Flytronic                              
*/

#ifndef BASE64_H
#define BASE64_H

class Base64
{
public:
    static bool encode (const void* dataToEncode, int dataSize, char* outBuf, int outBufSize);
    static bool decode (const char* textToDecode, unsigned char* outBuf, int outBufSize, int &outSize);

private:
    Base64(void);   ///< Static class lock constructor

    static const char cb64[];
    static const char cd64[];
    
    static void encodeBlock (const unsigned char in[], char* out, int len);
    static void decodeBlock (const unsigned char in4[], unsigned char* out, int len);
};

#endif  //BASE64_H
