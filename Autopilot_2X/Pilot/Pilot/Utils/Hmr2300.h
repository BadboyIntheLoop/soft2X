/*
 * Hmr2300.h
 *
 *  Created on: Aug 8, 2018
 *      Author: truongnt2
 */

#ifndef HMR2300_H
#define HMR2300_H

#if MAGNETOMETER_TYPE == USE_HMR
class SerialDeviceBase;

class Hmr2300
{
public:
	Hmr2300(void);
	static bool setFormat(bool binary_on, unsigned char* outBuf, int& outBufSize);
	static bool setSampleRate(int sampleRate, unsigned char* outBuf, int& outBufSize);
	static bool readDeviceID(unsigned char* outBuf, int& outBufSize);
	static bool setDeviceID(unsigned char* outBuf, int& outBufSize);
	static bool setBaudrate(bool fast, unsigned char* outBuf, int& outBufSize);
	static bool WriteEnable(unsigned char*outBuf, int& outBufSize);
	static bool WriteEnable_ID(unsigned char*outBuf, int& outBufSize);
	static bool StoreParams(unsigned char* outBuf, int& outBufSize);
	static bool reResponse(unsigned char* outBuf, int& outBufSize);
	static bool setAvgReading(bool turn_on, unsigned char* outBuf, int& outBufSize);


	static bool HmrInit (SerialDeviceBase* dev);

    static bool isConfigured;

private:

};

#endif


#endif /* PILOT_UTILS_HMR2300_H_ */
