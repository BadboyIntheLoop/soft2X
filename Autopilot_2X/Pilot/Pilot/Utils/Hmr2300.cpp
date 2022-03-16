/*
 * Hmr2300.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: truongnt2
 */


#include <PilotIncludes.h>

#if MAGNETOMETER_TYPE == USE_HMR

bool Hmr2300::isConfigured = true;
bool Hmr2300::setFormat(bool binary_on, unsigned char* outBuf, int& outBufSize){
	//cmd = *00B\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	if(binary_on)
		outBuf[3] = 'B';
	else
		outBuf[3] = 'A';
	outBuf[4] = '\r';
	outBufSize = 5;
	return true;
}

bool Hmr2300::setSampleRate(int sampleRate, unsigned char* outBuf, int& outBufSize){
	//cmd = *00R=nnn\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = 'R';
	outBuf[4] = '=';
	switch(sampleRate) {
	case 10:				//10Hz
		outBuf[5] = '1';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	case 20:				//20Hz
		outBuf[5] = '2';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	case 30:				//30Hz
		outBuf[5] = '3';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	case 40:				//40Hz
		outBuf[5] = '4';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	case 50:				//50Hz
		outBuf[5] = '5';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	case 60:				//60Hz
		outBuf[5] = '6';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	case 100:				//100Hz
		outBuf[5] = '1';
		outBuf[6] = '0';
		outBuf[7] = '0';
		outBuf[8] = '\r';
		outBufSize = 9;
		break;
	case 123:				//123Hz
		outBuf[5] = '1';
		outBuf[6] = '2';
		outBuf[7] = '3';
		outBuf[8] = '\r';
		outBufSize = 9;
		break;
	case 154:				//154Hz
		outBuf[5] = '1';
		outBuf[6] = '5';
		outBuf[7] = '4';
		outBuf[8] = '\r';
		outBufSize = 9;
		break;
	default:				//20Hz
		outBuf[5] = '2';
		outBuf[6] = '0';
		outBuf[7] = '\r';
		outBufSize = 8;
		break;
	}
	return true;
}

bool Hmr2300::readDeviceID(unsigned char* outBuf, int& outBufSize){
	//cmd: *99ID=\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = 'I';
	outBuf[4] = 'D';
	outBuf[5] = '\r';
	outBufSize = 6;
	return true;
}

bool Hmr2300::reResponse(unsigned char* outBuf, int& outBufSize){
	//cmd: *99N\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = 'N';
	outBuf[4] = '\r';
	outBufSize = 5;
	return true;
}

bool Hmr2300::setAvgReading(bool turn_on, unsigned char* outBuf, int& outBufSize)
{
	//cmd = *99VN\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	if(turn_on)
	{
		outBuf[3] = 'V';
		outBuf[4] = 'N';
	}
	else
	{
		outBuf[3] = 'V';
		outBuf[4] = 'F';
	}
	outBuf[6] = '\r';
	outBufSize = 6;
	return true;
}

bool Hmr2300::setDeviceID(unsigned char* outBuf, int& outBufSize){
	//cmd: *99ID=00\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = 'I';
	outBuf[4] = 'D';
	outBuf[5] = '=';
	outBuf[6] = '0';
	outBuf[7] = '0';
	outBuf[8] = '\r';
	outBufSize = 9;
	return true;
}

bool Hmr2300::setBaudrate(bool fast, unsigned char* outBuf, int& outBufSize){
	//cmd: *99!BR=S/F
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = '!';
	outBuf[4] = 'B';
	outBuf[5] = 'R';
	outBuf[6] = '=';
	if(fast)
		outBuf[7] = 'F';
	else
		outBuf[7] = 'S';
	outBuf[8] = '\r';
	outBufSize = 9;
	return true;
}

bool Hmr2300::WriteEnable(unsigned char*outBuf, int& outBufSize){
	//cmd: *99WE\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = 'W';
	outBuf[4] = 'E';
	outBuf[5] = '\r';
	outBufSize = 6;
	return true;
}

bool Hmr2300::WriteEnable_ID(unsigned char*outBuf, int& outBufSize){
	//cmd: *00WE\r
	outBuf[0] = '*';
	outBuf[1] = '0';
	outBuf[2] = '0';
	outBuf[3] = 'W';
	outBuf[4] = 'E';
	outBuf[5] = '\r';
	outBufSize = 6;
	return true;
}

bool Hmr2300::StoreParams(unsigned char* outBuf, int& outBufSize){
	//cmd: *00SP\r
	outBuf[0] = '*';
	outBuf[1] = '9';
	outBuf[2] = '9';
	outBuf[3] = 'S';
	outBuf[4] = 'P';
	outBuf[5] = '\r';
	outBufSize = 6;
	return true;
}

bool Hmr2300::HmrInit(SerialDeviceBase* dev)
{
	if(!isConfigured)
	{
		unsigned char buf[20];
		int bs = sizeof(buf);
		//Set ID = 00
		if(WriteEnable(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_WE1");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(setDeviceID(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_setDeviceID");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		//Set to Binary output
		if(WriteEnable(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_WE2");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(setFormat(true, buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_setFormat");
					return false;
				}
			}
		}
		//Set SampleRate
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(setSampleRate(100, buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_setSampleRate");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		//Average Reading ON
		if(WriteEnable(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_WE3");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(setAvgReading(true, buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_setAvgReading");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		//Re-Enter Response
		if(WriteEnable(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_WE4");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(reResponse(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_reResponse");
					return false;
				}
			}
		}
//		//Change baudrate = 19200 bps
//		if(WriteEnable(buf, bs))
//		{
//			if(dev != NULL)
//			{
//				if (!dev->sendBinary (buf, bs))
//				{
//					Log.errorPrintf("Hmr_Init_WE5");
//					return false;
//				}
//			}
//		}
//		usleep(1000000);
//		if(setBaudrate(true, buf, bs))
//		{
//			if(dev != NULL)
//			{
//				if (!dev->sendBinary (buf, bs))
//				{
//					Log.errorPrintf("Hmr_Init_setBaudrate");
//					return false;
//				}
//			}
//		}
		//Save parameters to EEPROM
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(WriteEnable(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_WE6");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		if(StoreParams(buf, bs))
		{
			if(dev != NULL)
			{
				if (!dev->sendBinary (buf, bs))
				{
					Log.errorPrintf("Hmr_Init_storeParams");
					return false;
				}
			}
		}
//		usleep(1000000);
		OSTimeDlyHMSM(0, 0, 1, 0);  //delay 1s
		isConfigured = true;
	}
	//enable loop
	HMR_mWriteReg(FLYEYE_HMR_BASE, 12, 0);
	return true;
}

Hmr2300::Hmr2300()
{
//	isConfigured = true;
}

#endif
