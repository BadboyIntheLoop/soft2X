/**
*                                                                   
* @class Base64                                                            
*                                                                  
* @brief Class controlling encoding and decoding in Base64 standard
*                                                                   
* 2008-2012 Witold Kruczek @ Flytronic                              
*/

#include <PilotIncludes.h>

//Translation Table as described in RFC1113
const char Base64::cb64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
//Translation Table to decode 
const char Base64::cd64[]="|$$$}rstuvwxyz{$$$$$$$>?@ABCDEFGHIJKLMNOPQRSTUVW$$$$$$XYZ[\\]^_`abcdefghijklmnopq";

//encode 3 8-bit binary bytes as 4 '6-bit' characters
void Base64::encodeBlock(const unsigned char in[], char* out, int len)
{
    out[0] = cb64[ in[0] >> 2 ];
    out[1] = cb64[ ((in[0] & 0x03u) << 4) | ((in[1] & 0xf0u) >> 4) ];
    out[2] = (len > 1) ? cb64[ ((in[1] & 0x0fu) << 2) | ((in[2] & 0xc0u) >> 6) ] : '=';
    out[3] = (len > 2) ? cb64[ in[2] & 0x3fu ] : '=';
}


//decode 4 '6-bit' characters into 3 8-bit binary bytes
void Base64::decodeBlock (const unsigned char in4[], unsigned char* out, int len)
{
    out[0] = static_cast<unsigned char>((in4[0] << 2 | in4[1] >> 4) & 0xff);
    if (len > 1)
        out[1] = static_cast<unsigned char>((in4[1] << 4 | in4[2] >> 2) & 0xff);
    if (len > 2)
        out[2] = static_cast<unsigned char>((((in4[2] << 6) & 0xc0u) | in4[3]) & 0xff);
}


bool Base64::encode (const void* dataToEncode, int dataSize, char* outBuf, int outBufSize)
{
    int blockCnt = 0;
    int j = 0;
    unsigned char in[3];

    // Control of the required space
    if ((((dataSize / 3) + 1) * 4) > outBufSize)
    {
        Log.errorPrintf("Base64_encode_1");
        return false;
    }

    for (int i = 0; i < dataSize; i++)
    {
        j = i % 3;
        in[j] = (static_cast<const unsigned char*>(dataToEncode))[i];
        if (j == 2)
        {
            encodeBlock (in, outBuf + (blockCnt * 4), 3);
            blockCnt++;
        }
    }
    //  Last not full block
    if (j < 1)
        in[1] = 0u;
    if (j < 2)
    {
        in[2] = 0u;
        encodeBlock (in, outBuf + (blockCnt * 4), j+1);
        blockCnt++;
    }
    // End with 0
    outBuf[blockCnt * 4] = 0;

    return true;
}


/**
* Decoding line coded with Base64
* /param textToDecode - input text in Base64 format
* /param outBuf - binary buffer for result
* /param outBufSize - size of result buffer
* /param outSize - number of decoded bytes
* return false wwhen there is an error.
* In case of an error data in buffer and outSize param are invalid.
*/
bool Base64::decode (const char* textToDecode, unsigned char* outBuf, int outBufSize, int &outSize)
{
    int textSize = strlen (textToDecode);
    unsigned char in[4] = {0};
    int blockCnt = 0;
    int pos = 0;
    outSize = 0;

    while (pos < textSize)
    {
        int len = 0;
        for (int i = 0; (i < 4) && (pos < textSize); i++)
        {
            unsigned char v = 0;
            while (pos < textSize && v == 0)
            {
                v = textToDecode[pos++];
                v = static_cast<unsigned char>((v < 43 || v > 122) ? 0 : cd64[ v - 43 ]);
                if (v != 0)
                    v = static_cast<unsigned char>((v == '$') ? 0 : (v - 61));
            }

            if (pos <= textSize)
            {
                if (v != 0)
                {
                    len++;
                    in[i] = static_cast<unsigned char>(v - 1);
                }

            }
            else
                in[i] = 0;
        }

        if (len > 0)
        {
            if (blockCnt*3 + len-1 > outBufSize)
                return false;
            decodeBlock (in, outBuf + blockCnt*3, len-1);
            blockCnt++;
            outSize += (len-1);
        }
    }

    return true;
}
