/**
*                                                                 
* Gauge
*
* 2008 Grzegorz Tyma @ Flytronic
*/

#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif


float Gauge::currentMain = 0.0f; 
float Gauge::voltageMain = 0.0f;


/**
* Constructor
*/
Gauge::Gauge(void)
{
    gOverflowXYZ = 0;

	gRawMeasurement.accYL = 0;
	gRawMeasurement.accXL = 0;    
	gRawMeasurement.accZL = 0;    
	gRawMeasurement.accYH = 0;
	gRawMeasurement.accXH = 0;
	gRawMeasurement.accZH = 0;
	// gyro
	gRawMeasurement.gyroXL = 0;
	gRawMeasurement.gyroYL = 0;
	gRawMeasurement.gyroZL = 0;
	gRawMeasurement.gyroXH = 0;
	gRawMeasurement.gyroYH = 0;
	gRawMeasurement.gyroZH = 0;
	// Gyro and accelerometers overflow
	gRawMeasurement.accOverflow = 0;
	gRawMeasurement.gyroOverflow = 0;

	gRawMeasurement.gyroXTemperature = 250;
	gRawMeasurement.gyroYTemperature = 250;
	gRawMeasurement.gyroZTemperature = 250;

	// accelerometers
	gMeasurement.accYL = 0.0f;
	gMeasurement.accXL = 0.0f;    
	gMeasurement.accZL = 0.0f;    
	gMeasurement.accYH = 0.0f;
	gMeasurement.accXH = 0.0f;
	gMeasurement.accZH = 0.0f;
	// gyro
	gMeasurement.gyroXL = 0.0f;
	gMeasurement.gyroYL = 0.0f;
	gMeasurement.gyroZL = 0.0f;
	gMeasurement.gyroXH = 0.0f;
	gMeasurement.gyroYH = 0.0f;
	gMeasurement.gyroZH = 0.0f;

	// filter
	gMeasurement.accFilterXL1 = 0.0f;
	gMeasurement.accFilterXL2 = 0.0f;
	gMeasurement.accFilterYL1 = 0.0f;
	gMeasurement.accFilterYL2 = 0.0f;
	gMeasurement.accFilterZL1 = 0.0f;
	gMeasurement.accFilterZL2 = 0.0f;

	phi_k = 0.788699902680461f/180.0f*PI;
	theta_k = -1.853956229709380f/180.0f*PI;
	psi_k = -0.971407237619020f/180.0f*PI;

	float sintheta = sin(theta_k);
	float costheta = cos(theta_k);
	float sinphi = sin(phi_k);
	float cosphi = cos(phi_k);
	float sinpsi = sin(psi_k);
	float cospsi = cos(psi_k);

	mag_kor_11 = costheta*cospsi; 
	mag_kor_21 = costheta*sinpsi;
	mag_kor_31 = -sintheta;
	mag_kor_12 = sinphi*sintheta*cospsi-cosphi*sinpsi; 
	mag_kor_22 = sinphi*sintheta*sinpsi+cosphi*cospsi;
	mag_kor_32 = sinphi*costheta;
	mag_kor_13 = cosphi*sintheta*cospsi+sinphi*sinpsi;
	mag_kor_23 = cosphi*sintheta*sinpsi-sinphi*cospsi; 
	mag_kor_33 = cosphi*costheta;

    initMagnetometer(); 

	reCalibrateCounter = 0;;
	reCalibrateDuration = 0;
	reCalibrateFlag = false;
	gyroXLOffNew = 0.0f;
	gyroYLOffNew = 0.0f;
	gyroZLOffNew = 0.0f;
	gyroXHOffNew = 0.0f; 
	gyroYHOffNew = 0.0f;
	gyroZHOffNew = 0.0f;
	accXUnderCalib = 0.0f;
	accYUnderCalib = 0.0f;
	accZUnderCalib = 0.0f;
	recalibrationStatus = 0;

	for(int i=0; i<2; i++)
	{
		acclXDly[i] = 0.0f;
		acclYDly[i] = 0.0f;
		acclZDly[i] = 0.0f;
		filterAcclXDly[i] = 0.0f;
		filterAcclYDly[i] = 0.0f;
		filterAcclYDly[i] = 0.0f;

//		gyroXDly[i] = 0.0f;
//		gyroYDly[i] = 0.0f;
//		gyroZDly[i] = 0.0f;
//		filterGyroXDly[i] = 0.0f;
//		filterGyroYDly[i] = 0.0f;
//		filterGyroYDly[i] = 0.0f;
	}
    
    lockMagPressureSensor = false;
    //init barometer variables
    grawBaroMeasurement.Pre_digit = 0;
    grawBaroMeasurement.Pressure = 0;
    grawBaroMeasurement.Temp_digit = 0;
    grawBaroMeasurement.Temperature = 0.0f;

    DatIsStarted = false;
    isAltValid = false;
    updatePreCounter = 0;
    for(int i=0; i<8; i++)
    	Coeff[i] = 0;

    //init diff pressure variables
    grawDiffPressureMeasurement.Pressure = 0.0f;
    grawDiffPressureMeasurement.Temperature = 0.0f;

    grawAdsMeasurement.tempEnv = 0;
    grawAdsMeasurement.rhEnv = 0;
    grawAdsMeasurement.fuelLevel = 0;

    _pressure_sum = 0.0f;
    _temp_sum = 0.0f;
    _diffPress_Cnt = 0;
    _pressure_offset = 0.0f;
}


/**
* Read default config data from flash memory and set them. 
*/
void Gauge::setDefaultConfig(void)
{
    gGaugeParam.config = 0;                                 // default: bit0 = 0 - temperature for accelerometeres from scp1 sensor.
    // accelerometers
	gGaugeParam.accXLOff = 14547.231f;						// zero offset
    gGaugeParam.accXLSens = -6198.5f;						// sensitivity
    gGaugeParam.accXLSensT = -2.6981f;						// temperature sensitivity
	gGaugeParam.accYLOff = 14514.726f;                       
    gGaugeParam.accYLSens = -5998.8f;                        
    gGaugeParam.accYLSensT = -2.334f;                        
    gGaugeParam.accZLOff = 15390.6454f;
    gGaugeParam.accZLSens = -6268.2f;
    gGaugeParam.accZLSensT = -2.5f;
    gGaugeParam.accXHOff = 0;
    gGaugeParam.accXHSens = 1;
    gGaugeParam.accXHSensT = 1;
    gGaugeParam.accYHOff = 0;
    gGaugeParam.accYHSens = 1;
    gGaugeParam.accYHSensT = 1;
    gGaugeParam.accZHOff = 0;
    gGaugeParam.accZHSens = 1;
    gGaugeParam.accZHSensT = 1;
    // gyro
    gGaugeParam.gyroXLOff = 15257.66f;
    gGaugeParam.gyroXLSens = 148.404f;
    gGaugeParam.gyroXLSensT = 2.26f;
    gGaugeParam.gyroYLOff = 15007.164f;
    gGaugeParam.gyroYLSens = -153.839f;
    gGaugeParam.gyroYLSensT = 3.144f;
    gGaugeParam.gyroZLOff = 15105.0f;
    gGaugeParam.gyroZLSens = 153.127f;
    gGaugeParam.gyroZLSensT = 8.549f;
    gGaugeParam.gyroXHOff = 15335.149f;
    gGaugeParam.gyroXHSens = 23.249f;
    gGaugeParam.gyroXHSensT = 3.705f;
    gGaugeParam.gyroYHOff = 15383.014f;
    gGaugeParam.gyroYHSens = -23.065f;
    gGaugeParam.gyroYHSensT = -0.289875f;
    gGaugeParam.gyroZHOff = 15178.727f;
    gGaugeParam.gyroZHSens = 25.127f;
    gGaugeParam.gyroZHSensT = 3.202f;

    //swap accl
	gGaugeParam.acclXIndex = 1;
	gGaugeParam.acclYIndex = 2;
	gGaugeParam.acclZIndex = 3;
	gGaugeParam.acclXFactor = 1;
	gGaugeParam.acclYFactor = 1;
	gGaugeParam.acclZFactor = 1;

    //swap gyro
	gGaugeParam.gyroXIndex = 1;
	gGaugeParam.gyroYIndex = 2;
	gGaugeParam.gyroZIndex = 3;
	gGaugeParam.gyroXFactor = 1;
	gGaugeParam.gyroYFactor = 1;
	gGaugeParam.gyroZFactor = 1;


	// magnetometer parameters
	gGaugeParam.magnetX0ff = 3.266593356521972e+4f;
	gGaugeParam.magnetY0ff = 3.249509082717756e+4f;
	gGaugeParam.magnetZ0ff = -3.291866724221400e+4f;
	gGaugeParam.magnetR11 = 0.611754671958110e-4f;
	gGaugeParam.magnetR12 = 0.005311706901443e-4f;
	gGaugeParam.magnetR13 = 0.005568312500548e-4f;
	gGaugeParam.magnetR21 = 0.005311706901443e-4f;
	gGaugeParam.magnetR22 = 0.605170808630765e-4f;
	gGaugeParam.magnetR23 = 0.001453982864486e-4f;
	gGaugeParam.magnetR31 = 0.005568312500548e-4f;
	gGaugeParam.magnetR32 = 0.001453982864486e-4f;
	gGaugeParam.magnetR33 = 0.656599030192586e-4f;

	gGaugeParam.magnetTheta = 0.0f;
	gGaugeParam.magnetPhi = 0.0f;
	gGaugeParam.magnetPsi = 0.0f; 

	phi_k = 0.788699902680461f/180.0f*PI;
	theta_k = -1.853956229709380f/180.0f*PI;
	psi_k = -0.971407237619020f/180.0f*PI;

	float sintheta = sin(theta_k);
	float costheta = cos(theta_k);
	float sinphi = sin(phi_k);
	float cosphi = cos(phi_k);
	float sinpsi = sin(psi_k);
	float cospsi = cos(psi_k);

	mag_kor_11 = costheta*cospsi; 
	mag_kor_21 = costheta*sinpsi;
	mag_kor_31 = -sintheta;
	mag_kor_12 = sinphi*sintheta*cospsi-cosphi*sinpsi; 
	mag_kor_22 = sinphi*sintheta*sinpsi+cosphi*cospsi;
	mag_kor_32 = sinphi*costheta;
	mag_kor_13 = cosphi*sintheta*cospsi+sinphi*sinpsi;
	mag_kor_23 = cosphi*sintheta*sinpsi-sinphi*cospsi; 
	mag_kor_33 = cosphi*costheta;

	gGaugeParam.currentOffset = 43.78f;
	gGaugeParam.currentSens = 0.0025668f;
	gGaugeParam.voltageOffset = 0.0033635f;


	gGaugeParam.diffPressOffset = 2.494f;
	gGaugeParam.magEnable = 1;
	gGaugeParam.gyroTemperatureEnable = 0;
	gGaugeParam.filterActive = 0;
	gGaugeParam.filterAcclCoff_a = 0.12355f;
	gGaugeParam.filterAcclCoff_b = -0.026622f;
	gGaugeParam.filterAcclCoff_c = 0.12355f;
	gGaugeParam.filterAcclCoff_d = 1.2584f;
	gGaugeParam.filterAcclCoff_e = -0.47887f;


	gGaugeParam.magneXIndex = 1;
	gGaugeParam.magneYIndex = 2;
	gGaugeParam.magneZIndex = 3;
	gGaugeParam.magneXFactor = 1;
	gGaugeParam.magneYFactor = 1;
	gGaugeParam.magneZFactor = 1;
}

void Gauge::initMagnetometer(void)
{
    gMagnetRawMeasurement.magnetometerPresent = 0;
	gMagnetRawMeasurement.configurationRead = 0;
    gMagnetRawMeasurement.magX = 0;
    gMagnetRawMeasurement.magY = 0;
    gMagnetRawMeasurement.magZ = 0;
    gMagnetRawMeasurement.diffPress  = 0.0f;
    gMagnetRawMeasurement.magPressure  = 0;
    gMagnetRawMeasurement.magTemperature  = 0;

    gMagnetMeasurement.magX = 0.0f;
    gMagnetMeasurement.magY = 0.0f;
    gMagnetMeasurement.magZ = 0.0f;
    gMagnetMeasurement.diffPress  = 0.0f;
    gMagnetMeasurement.magPressure  = 0;
    gMagnetMeasurement.magTemperature  = 0;

    last_magX = 0;
    last_magY = 0;
    last_magZ = 0;
    mag_Chk_Cnt = 0;
    magErrorCnt = 0;
    _magnet_cnt = 0;


    gMagnetConfiguration.dac1 = 30000;
    gMagnetConfiguration.dac2 = 28300;
    gMagnetConfiguration.dac3 = 30600;
    gMagnetConfiguration.dac4 = 0;

	gMagnetConfiguration.xoff = 3.266593356521972e+4f;
	gMagnetConfiguration.yoff = 3.249509082717756e+4f;
	gMagnetConfiguration.zoff = -3.291866724221400e+4f;
	gMagnetConfiguration.r11 = 0.611754671958110e-4f;
	gMagnetConfiguration.r12 = 0.005311706901443e-4f;
	gMagnetConfiguration.r13 = 0.005568312500548e-4f;
	gMagnetConfiguration.r21 = 0.005311706901443e-4f;
	gMagnetConfiguration.r22 = 0.605170808630765e-4f;
	gMagnetConfiguration.r23 = 0.001453982864486e-4f;
	gMagnetConfiguration.r31 = 0.005568312500548e-4f;
	gMagnetConfiguration.r32 = 0.001453982864486e-4f;
	gMagnetConfiguration.r33 = 0.656599030192586e-4f;
	gMagnetConfiguration.phi = 0.0f;
	gMagnetConfiguration.theta = 0.0f;
	gMagnetConfiguration.psi = 0.0f;
}



/**
* Return true when magnetometer exist and is connected to the autopilot.
*/
bool Gauge::isMagnetometerPresent (void) const
{
    return (gMagnetRawMeasurement.magnetometerPresent==1);
}

/**
* Read data from converters (in every interrupion)
*/
void Gauge::readData(void)
{
    bool ret_validProm = false;
    u32  isPresValid;
    u32  rdPromDone;


	// NOTE! In hardware version 0.1 axis X<->Y are swapped
    gRawMeasurement.gyroXL = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));
    gRawMeasurement.gyroYL = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));
    gRawMeasurement.gyroZL = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));
    gRawMeasurement.accXL = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));
    gRawMeasurement.accYL = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));
    gRawMeasurement.accZL = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));

	gRawMeasurement.gyroXTemperature = static_cast<unsigned short>(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG));
	VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, IMU_RDDATA_REG);
	gRawMeasurement.gyroYTemperature = gRawMeasurement.gyroXTemperature;
	gRawMeasurement.gyroZTemperature = gRawMeasurement.gyroXTemperature;

    gRawMeasurement.gyroXH = gRawMeasurement.gyroXL;
    gRawMeasurement.gyroYH = gRawMeasurement.gyroYL;
    gRawMeasurement.gyroZH = gRawMeasurement.gyroZL;
    gRawMeasurement.accXH = gRawMeasurement.accXL;
    gRawMeasurement.accYH = gRawMeasurement.accYL;
    gRawMeasurement.accZH = gRawMeasurement.accZL;

	gRawMeasurement.accOverflow = 0;
	gRawMeasurement.gyroOverflow = 0;
    
	_magnet_cnt++;
	if(_magnet_cnt == Magnet_Divisor)
	{
		_magnet_cnt = 0;
		magnetProcessData();
	}

	// read AGL
    gRawMeasurement.agl = 0;

	//Read data differential pressure
	diffPressProcessData();

	if(!DatIsStarted)
	{
		rdPromDone = Baro_ReadReg(BAROMETER_BASE,BARO_IS_VALID) & 0x00000002;
		if(rdPromDone){
			baro_read_prom();
			ret_validProm = baro_promisValid();
			if(!ret_validProm)
			{
				Log.msgPrintf("Gauge: Barometer-Prom CRC failed");
#if BARO_TYPE == BARO_SPI
				Baro_WriteReg(BAROMETER_BASE, BARO_READPROM_REG, 1);
#if PILOT_TARGET == PT_HARDWARE
				usleep(10);
#endif
				Baro_WriteReg(BAROMETER_BASE, BARO_IP_ENABLE_REG, 1);
#elif BARO_TYPE == BARO_I2C
#endif
				return;
			}
			//Enable Loop Data
			Baro_WriteReg(BAROMETER_BASE, BARO_LOOP_ENABLE_REG, 1);
			DatIsStarted = true;
		}
	}
	if(!isAltValid){
		isPresValid = Baro_ReadReg(BAROMETER_BASE,BARO_IS_VALID) & 0x00000001;
		if(isPresValid)
			isAltValid = true;
		else
			Baro_WriteReg(BAROMETER_BASE, BARO_LOOP_ENABLE_REG, 1);
	}
	if(DatIsStarted && isAltValid)
	{
		if(++updatePreCounter % BARO_UPDATE_DIVISOR == 0) {
			baro_processData();
			updatePreCounter = 0;
		}
	}

}

void Gauge::requestMagnetometerConfiguration(void)
{
    Log.msgPrintf("requestMagnetometerConfiguration do nothing");
    
}
bool Gauge::storeMagnetometer(void)
{
		Log.msgPrintf("storeMagnetometer do nothing");
		return true;
}


bool Gauge::verifyMagnetometer(void)
{
    if(gMagnetRawMeasurement.configurationRead == 0) return false;
    return true;
}

#if MAGNETOMETER_TYPE == USE_HMR

bool Gauge::isPacketHmrData(const char* hmrLine)
{
	return (STRNICMP (hmrLine, "*00P", 4) == 0);
}

bool Gauge::hmrParseData(const char* hmrLine)
{
    return true;
}
#endif

/**
* If function readMagnetometerConfiguration() returned true values: xoff, yoff, zoff, r11,... should be write to structure gGaugeParam elements
*/
bool Gauge::readMagnetometerConfiguration(float& xoff, float& yoff, float& zoff, float& r11, float& r12, float& r13, float& r21, float& r22, float& r23,
										   float& r31,  float& r32,  float& r33,  float& phi,  float& theta, float& psi, int& magnetometerPresent) const
{

    magnetometerPresent = gMagnetRawMeasurement.magnetometerPresent;
	if(gMagnetRawMeasurement.configurationRead == 0) return false;

	xoff = gMagnetConfiguration.xoff;
    yoff = gMagnetConfiguration.yoff;
    zoff = gMagnetConfiguration.zoff;
    r11 = gMagnetConfiguration.r11; 
    r12 = gMagnetConfiguration.r12; 
    r13 = gMagnetConfiguration.r13; 
    r21 = gMagnetConfiguration.r21; 
    r22 = gMagnetConfiguration.r22; 
    r23 = gMagnetConfiguration.r23; 
    r31 = gMagnetConfiguration.r31; 
    r32 = gMagnetConfiguration.r32; 
    r33 = gMagnetConfiguration.r33; 
    phi = gMagnetConfiguration.phi; 
    theta = gMagnetConfiguration.theta;
    psi = gMagnetConfiguration.psi;  
 
	return true;
}



void Gauge::calibrateMagnetometerData(void)
{
	gaugeParamT* p = &gGaugeParam;
	gMagnetMeasurement.magnetometerPresent = gMagnetRawMeasurement.magnetometerPresent;
	//gMagnetMeasurement.magX = MAG_SCALE_FACTOR * (gMagnetRawMeasurement.magX - p->magnetX0ff);
	//gMagnetMeasurement.magY = MAG_SCALE_FACTOR * (gMagnetRawMeasurement.magY - p->magnetY0ff);
	//gMagnetMeasurement.magZ = MAG_SCALE_FACTOR * (gMagnetRawMeasurement.magZ - p->magnetZ0ff);

#if USE_WSM == WSM_ENABLE
//	// axis swap
//    gMagnetMeasurement.magZ = (float)(-gMagnetRawMeasurement.magZ);
//    float tX = gMagnetRawMeasurement.magX;
//	gMagnetMeasurement.magX = (float)gMagnetRawMeasurement.magY;
//    gMagnetMeasurement.magY = tX;
//
//	tX = gMagnetMeasurement.magX - p->magnetX0ff;
//	float tY = gMagnetMeasurement.magY - p->magnetY0ff;
//	float tZ = gMagnetMeasurement.magZ - p->magnetZ0ff;
	// axis swap
	gMagnetMeasurement.magX = (float)gMagnetRawMeasurement.magX;
	gMagnetMeasurement.magY = (float)gMagnetRawMeasurement.magY;
    gMagnetMeasurement.magZ = (float)gMagnetRawMeasurement.magZ;


	float tX = gMagnetMeasurement.magX - p->magnetX0ff;
	float tY = gMagnetMeasurement.magY - p->magnetY0ff;
	float tZ = gMagnetMeasurement.magZ - p->magnetZ0ff;
#else
	// axis swap
	gMagnetMeasurement.magX = (float)gMagnetRawMeasurement.magX;
	gMagnetMeasurement.magY = (float)gMagnetRawMeasurement.magY;
    gMagnetMeasurement.magZ = (float)gMagnetRawMeasurement.magZ;


	float tX = gMagnetMeasurement.magX - p->magnetX0ff;
	float tY = gMagnetMeasurement.magY - p->magnetY0ff;
	float tZ = gMagnetMeasurement.magZ - p->magnetZ0ff;
#endif


	gMagnetMeasurement.magX = p->magnetR11*tX + p->magnetR12*tY + p->magnetR13*tZ;
	gMagnetMeasurement.magY = p->magnetR21*tX + p->magnetR22*tY + p->magnetR23*tZ;
	gMagnetMeasurement.magZ = p->magnetR31*tX + p->magnetR32*tY + p->magnetR33*tZ;

	// assembly in airframe correction

	tX = gMagnetMeasurement.magX;
	tY = gMagnetMeasurement.magY;
	tZ = gMagnetMeasurement.magZ;

	gMagnetMeasurement.magX = mag_kor_11*tX + mag_kor_12*tY + mag_kor_13*tZ;
	gMagnetMeasurement.magY = mag_kor_21*tX + mag_kor_22*tY + mag_kor_23*tZ;
	gMagnetMeasurement.magZ = mag_kor_31*tX + mag_kor_32*tY + mag_kor_33*tZ;
}


void Gauge::initGauge(void)
{

}

/**
* Converters initialization
*/
void Gauge::initADC(void)
{
     gRawMeasurement.gyroXTemperature = 250;		//25C
     gRawMeasurement.gyroYTemperature = 250;
     gRawMeasurement.gyroZTemperature = 250;
}


/**
* Turn on gyroscpes test mode.
* onOff = 0 : mode turned off
* onOff = 1 : testing mode turned on
*/
void Gauge::setST1(int onOff)
{

}


/**
* Read battery voltage and current
*/
void Gauge::readBattery(float& current, float& voltage, float& currentDown, float& voltageDown, int& status)
{
    //  Without simulation (except HIL mode)s
    if (PState->getSimLevel() == 0 || PState->getSimLevel() >= 1)
    {
        u8 data[4];

        data[0]=0;data[1]=0;

        current = ((((unsigned int)data[0]<<8)| (unsigned int)data[1]) - 26214.4f)*5.0f/65536.0f;

		data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 0;

        // buffer battery voltage
        voltage = (((unsigned int)data[2]<<8)| (unsigned int)data[3])*15.0f/65536.0f;

        // change for fuel engine main battery is the buffer one
        currentDown = ServMan->getFuelConsumption();
        voltageDown = voltageMain;
    }
    else    //  simlevel > 0
    {
        current = 0.0f;
        voltage = 0.0f;
        // RF: VT1
		currentDown = ServMan->getFuelConsumption(); // fuel consumption algorithm simulation
        voltageDown = 0.0f;
        status = 0;
    }

}

/************************************************************************************/
/*                                                                                  */
/*                                  NIOS-II (hardware)                              */
/*                                                                                  */
/************************************************************************************/

/**
* Compute data from sensors.
*/
void Gauge::calibrateData(void)
{
    measurementT* m = &gMeasurement;
    gaugeParamT* p = &gGaugeParam;
    rawMeasurementT* r = &gRawMeasurement;

    float acclX_tmp;
    float acclY_tmp;
    float acclZ_tmp;
    float gyroX_tmp;
    float gyroY_tmp;
    float gyroZ_tmp;
	// accelerations [g]
    // rotate the axes
    acclX_tmp = (static_cast<float>((s16)r->accXL));
    acclY_tmp = (static_cast<float>((s16)r->accYL));
    acclZ_tmp = (static_cast<float>((s16)r->accZL));

    // offset compensation
    acclX_tmp = (acclX_tmp - p->accXLOff)*p->accXLSens * ACCL_SCALE_FACTOR;
    acclY_tmp = (acclY_tmp - p->accYLOff)*p->accYLSens * ACCL_SCALE_FACTOR;
    acclZ_tmp = (acclZ_tmp - p->accZLOff)*p->accZLSens * ACCL_SCALE_FACTOR;

	// filter the acc data
    if(p->filterActive)
	{
		float coff[5];
		coff[0] = p->filterAcclCoff_a;
		coff[1] = p->filterAcclCoff_b;
		coff[2] = p->filterAcclCoff_c;
		coff[3] = p->filterAcclCoff_d;
		coff[4] = p->filterAcclCoff_e;

		acclX_tmp = chebyshev_lowpass_filter_accl(acclX_tmp, acclXDly[0], acclXDly[1], filterAcclXDly[0], filterAcclXDly[1], coff);
		acclY_tmp = chebyshev_lowpass_filter_accl(acclY_tmp, acclYDly[0], acclYDly[1], filterAcclYDly[0], filterAcclYDly[1], coff);
		acclZ_tmp = chebyshev_lowpass_filter_accl(acclZ_tmp, acclZDly[0], acclZDly[1], filterAcclZDly[0], filterAcclZDly[1], coff);
	}

    float acclX_calib;
    float acclY_calib;
    float acclZ_calib;
	//X-axis
	if(p->acclXIndex == 1)
	{
		acclX_calib = acclX_tmp;
	}else if(p->acclXIndex == 2)
	{
		acclX_calib = acclY_tmp;
	}else if(p->acclXIndex == 3)
	{
		acclX_calib = acclZ_tmp;
	}else
	{
		acclX_calib = acclX_tmp;
	}
	//Y-axis
	if(p->acclYIndex == 1)
	{
		acclY_calib = acclX_tmp;
	}else if(p->acclYIndex == 2)
	{
		acclY_calib = acclY_tmp;
	}else if(p->acclYIndex == 3)
	{
		acclY_calib = acclZ_tmp;
	}else
	{
		acclY_calib = acclY_tmp;
	}
	//Z-axis
	if(p->acclZIndex == 1)
	{
		acclZ_calib = acclX_tmp;
	}else if(p->acclZIndex == 2)
	{
		acclZ_calib = acclY_tmp;
	}else if(p->acclZIndex == 3)
	{
		acclZ_calib = acclZ_tmp;
	}else
	{
		acclZ_calib = acclZ_tmp;
	}

	m->accXL = acclX_calib * (int16_t)p->acclXFactor;
	m->accYL = acclY_calib * (int16_t)p->acclYFactor;
	m->accZL = acclZ_calib * (int16_t)p->acclZFactor;

    m->accYH = m->accYL;
    m->accXH = m->accXL;
    m->accZH = m->accZL;

    // gyro (rad/s)
    // rotate the axes

    //bias_gyro = a*x^2 + b*x + c
    //x: imu temperature
    //a: gyroXHSensT
    //b: gyroXLSensT
    //c: gyroXLOff
    float imu_temperature = (float)r->gyroXTemperature*0.001f;
    gyroX_tmp = static_cast<float>((s16)(r->gyroXL) - (p->gyroXLOff + imu_temperature * p->gyroXLSensT + imu_temperature* imu_temperature*p->gyroXHSensT)) * DEG_2_RAD * GYRO_SCALE_FACTOR;
    gyroY_tmp = static_cast<float>((s16)(r->gyroYL) - (p->gyroYLOff + imu_temperature * p->gyroYLSensT + imu_temperature* imu_temperature*p->gyroYHSensT)) * DEG_2_RAD * GYRO_SCALE_FACTOR;
    gyroZ_tmp = static_cast<float>((s16)(r->gyroZL) - (p->gyroZLOff + imu_temperature * p->gyroZLSensT + imu_temperature* imu_temperature*p->gyroZHSensT)) * DEG_2_RAD * GYRO_SCALE_FACTOR;
	gyroX_tmp *= p->gyroXLSens;
	gyroY_tmp *= p->gyroYLSens;
	gyroZ_tmp *= p->gyroZLSens;

    float gyroX_calib;
    float gyroY_calib;
    float gyroZ_calib;
    //swap gyroscope
	//X-axis
	if(p->gyroXIndex == 1)
	{
		gyroX_calib = gyroX_tmp;
	}else if(p->gyroXIndex == 2)
	{
		gyroX_calib = gyroY_tmp;
	}else if(p->gyroXIndex == 3)
	{
		gyroX_calib = gyroZ_tmp;
	}else
	{
		gyroX_calib = gyroX_tmp;
	}

	//Y-axis
	if(p->gyroYIndex == 1)
	{
		gyroY_calib = gyroX_tmp;
	}else if(p->gyroYIndex == 2)
	{
		gyroY_calib = gyroY_tmp;
	}else if(p->gyroYIndex == 3)
	{
		gyroY_calib = gyroZ_tmp;
	}else
	{
		gyroY_calib = gyroY_tmp;
	}

	//Z-axis
	if(p->gyroZIndex == 1)
	{
		gyroZ_calib = gyroX_tmp;
	}else if(p->gyroZIndex == 2)
	{
		gyroZ_calib = gyroY_tmp;
	}else if(p->gyroZIndex == 3)
	{
		gyroZ_calib = gyroZ_tmp;
	}else
	{
		gyroZ_calib = gyroZ_tmp;
	}
	m->gyroXL = gyroX_calib*(int16_t)p->gyroXFactor;
	m->gyroYL = gyroY_calib*(int16_t)p->gyroYFactor;
	m->gyroZL = gyroZ_calib*(int16_t)p->gyroZFactor;


	m->gyroXH = m->gyroXL;
	m->gyroYH = m->gyroYL;
	m->gyroZH = m->gyroZL;
	
    if(gMagnetRawMeasurement.magnetometerPresent)
    {
        calibrateMagnetometerData();
    }
    
    //FIXME
    m->agl = (static_cast<float>(r->agl&0xFFF))* 2.54f / 14.7f / 100.0f;
}

/**
* Call AHRS
*/
void Gauge::ahrs(void)
{
	Log.msgPrintf("Gauge calls ahrs");
}

/**
* Start recalibration.
* Status "recalibrationStatus" is changed:  
*							0 recalibration has not been done after turning on.
*							1 under recalibration
*							2 recalibration failed
*							3 recalibration succed
*/
bool Gauge::startRecalibration()
{
	//if not under recalibration process start it
	if(recalibrationStatus != 1)
	{
		reCalibrateFlag = true;
		recalibrationStatus = 1;
	}

	return true;
}

/**
* Calculate new zero offset for gyroscopes
* During calibrationDuration [number of interruptions] average measurement (gryroscopes remaining at rest) is calculated.
* In addition temeperatures changings are taken into account
* Remaining at rest state is checked by accelerometers indications.
* Returns : 0 recalibration finished successfully.
*			1 under recalibration
*			2 Recalibration failed object has moved
*/
int Gauge::reCalibrateGyros(int calibrationDuration)
{
	gaugeParamT* p = &gGaugeParam;
	rawMeasurementT* r = &gRawMeasurement;
	measurementT* m = &gMeasurement;

	// First entrance to the recalibration
	if(reCalibrateCounter == 0)
	{
		gyroXLOffNew = 0.0f;
		gyroYLOffNew = 0.0f;
		gyroZLOffNew = 0.0f;

		gyroXHOffNew = 0.0f;
		gyroYHOffNew = 0.0f;
		gyroZHOffNew = 0.0f;

		accXUnderCalib = m->accXL;
		accYUnderCalib = m->accYL;
		accZUnderCalib = m->accZL;

		reCalibrateDuration = calibrationDuration;
	}

	// Check if it's still (not moving)
	if((abs(accXUnderCalib - m->accXL)<0.05)&&(abs(accYUnderCalib - m->accYL)<0.05)&&(abs(accZUnderCalib - m->accZL)<0.05))
	{
		// Temaperature from gryoscopes measurement.
		gyroXLOffNew += (static_cast<float>(r->gyroXL) - (static_cast<float>(r->gyroXTemperature) * p->gyroXLSensT));
		gyroYLOffNew += (static_cast<float>(r->gyroYL) - (static_cast<float>(r->gyroYTemperature) * p->gyroYLSensT));
		gyroZLOffNew += (static_cast<float>(r->gyroZL) - (static_cast<float>(r->gyroZTemperature) * p->gyroZLSensT));

		gyroXHOffNew += (static_cast<float>(r->gyroXH) - (static_cast<float>(r->gyroXTemperature) * p->gyroXHSensT));
		gyroYHOffNew += (static_cast<float>(r->gyroYH) - (static_cast<float>(r->gyroYTemperature) * p->gyroYHSensT));
		gyroZHOffNew += (static_cast<float>(r->gyroZH) - (static_cast<float>(r->gyroZTemperature) * p->gyroZHSensT));
        
		recalibrationStatus = 1;
	}
	else
	{
		reCalibrateFlag = false;
		reCalibrateCounter = 0;
		recalibrationStatus = 2;
		return 2;	// Recalibration breaks because of object has moved
	}


	reCalibrateCounter++;

	// All samples gathered, rewrite averages
	if(reCalibrateCounter >= reCalibrateDuration)
	{
		p->gyroXLOff = gyroXLOffNew/(static_cast<float>(reCalibrateCounter));
		p->gyroYLOff = gyroYLOffNew/(static_cast<float>(reCalibrateCounter));
		p->gyroZLOff = gyroZLOffNew/(static_cast<float>(reCalibrateCounter));

		p->gyroXHOff = gyroXHOffNew/(static_cast<float>(reCalibrateCounter));
		p->gyroYHOff = gyroYHOffNew/(static_cast<float>(reCalibrateCounter));
		p->gyroZHOff = gyroZHOffNew/(static_cast<float>(reCalibrateCounter));

		reCalibrateFlag = false;
		reCalibrateCounter = 0;
		recalibrationStatus = 3;
		return 0;	// Recalibration finished successfully.

	}

	return 1;		// Under calibration
}

bool Gauge::imu_is_ready(void)
{
	int spi_ready;
	spi_ready = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE,IMU_RDCFIDLE_REG );
	if(spi_ready)
		return true;
	else
		return false;
}
bool Gauge::imu_not_receive_data(void)
{
	int spi_loop_ready;
	spi_loop_ready = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE,IMU_RDDATAIDLE_REG );
	if(spi_loop_ready)
		return true;
	else
		return false;
}

void Gauge::start_imu_data(void)
{
	VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE, IMU_WRENABLE_REG, 1);
}
void Gauge::stop_imu_data(void)
{
	VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE, IMU_WRDISABLE_REG, 1);
}

void Gauge::imu_wr_cmd(u32 cmd)
{
	u32 i;
	//wait until adis ready
	i = 50000000;
	while(!imu_is_ready())
	{
		i--;
		if(i==0)
		{
			Log.msgPrintf("\r\nwrite cmd timeout 1");
			return;
		}
	}
	i = 50000000;
	while(!imu_not_receive_data())
	{
		i--;
		if(i==0)
		{
			Log.msgPrintf("\r\nwrite cmd timeout 2");
			return;
		}
	}
	//write cmd to adis
	VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE,IMU_WRCFDATA_REG, cmd );
	VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE,IMU_WRCFCMD_REG, 1 );
}
s16 Gauge::imu_rd_data(void)
{
	s16 data_ret;
	u32 i;
	i = 50000000;
	while(!imu_is_ready())
	{
		i--;
		if(i==0)
		{
			Log.msgPrintf("\r\nread cmd timeout 1");
			return 0;
		}
	}
	data_ret = (s16)(VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE,IMU_RDCFDATA_REG ) & 0x0000ffff);
	return data_ret;
}


u16 Gauge::imu_prod_id(void)
{
	u16 prod_id;
	imu_wr_cmd(PROD_ID);
	prod_id = imu_rd_data();
	return prod_id;
}

void Gauge::imu_reset(void)
{
	imu_wr_cmd(0xE880);
	imu_wr_cmd(0xE900);
#if PILOT_TARGET == PT_HARDWARE
	usleep(SOFTWARE_RESET_TIME);
#endif
}
void Gauge::imu_sample_rate_set(u16 sam_rate)
{
	const u16 max_sam_rate = 2000;//Hz
	if(sam_rate>max_sam_rate) return;
	u16 dec_rate = max_sam_rate/sam_rate - 1;
	u16 reg = 0xE400|dec_rate;
	imu_wr_cmd(reg);
	imu_wr_cmd(0xE500);
}
void Gauge::imu_filt_ctrl(u16 numOfTab)
{
	u16 reg = 0xCC00|numOfTab;
	imu_wr_cmd(reg);
	imu_wr_cmd(0xCD00);
}
void Gauge::imu_flash_update_cmd(void)
{
	imu_wr_cmd(0xE808);
	imu_wr_cmd(0xE900);
#if PILOT_TARGET == PT_HARDWARE
	usleep(FLASH_MEMORY_UPDATE_TIME);
#endif
}
void Gauge::imu_bias_correction_update(void)
{
	imu_wr_cmd(0xE801);
	imu_wr_cmd(0xE900);
}
void Gauge::imu_factory_calibration_restore(void)
{
	imu_wr_cmd(0xE802);
	imu_wr_cmd(0xE900);
}
void Gauge::imu_bias_null_cfg(void)
{
	//default NULL_CFG = 0x070A: enable bias null for gyro, null time ~32s
	imu_wr_cmd(0xE60A);
	imu_wr_cmd(0xE707);
#if PILOT_TARGET == PT_HARDWARE
	usleep(32*1000000);
#endif
}

void Gauge::imu_wr_dvsr(u8 dvsr)
{
	VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE, IMU_DIVISOR_REG, dvsr);
}

void Gauge::imu_handle_int()
{
	VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG3_OFFSET, 0);
	if (PState != NULL && OSRunning == OS_TRUE){
		PState->notify (IRQ_ADC);
	}

}

void Gauge::imu_init_interrupt(uint32_t Int_Id, UCOS_INT_FNCT_PTR Handler, uint8_t Priority, UCOS_INT_TYPE Type)
{
#if PILOT_TARGET == PT_HARDWARE
	UCOS_IntSrcEn(Int_Id);
	UCOS_IntTypeSet(Int_Id, Type);
	UCOS_IntVectSet(Int_Id, Priority, 0, Handler , 0);
#endif
}


void Gauge::initImu(void)
{
	u16 prod_id;
	u32 timeout = 50000000;

	imu_wr_dvsr(33);   //sclk = 100Mhz/(2*dvsr)
	stop_imu_data();

//	imu_reset();
	prod_id = imu_prod_id();
	while(prod_id != 16477){
		prod_id = imu_prod_id();
		timeout--;
		imu_reset();
		if (timeout == 0)
		{
			Log.msgPrintf("ADIS16375 initialize failed");
			return;
		}
	}
#if PILOT_TARGET == PT_HARDWARE
	usleep(1000000); // Sleep 1s
#endif
//	imu_bias_null_cfg();
	imu_sample_rate_set(100);
	imu_init_interrupt(ADIS_INT_ID,(UCOS_INT_FNCT_PTR)imu_handle_int,0xa1,UCOS_INT_TYPE_LEVEL);
	start_imu_data();

}

//--------------------------------------------------------------------------
// Barometer function declarations
//--------------------------------------------------------------------------
void Gauge::initBarometer(void){
#if BARO_TYPE == BARO_SPI
	Baro_WriteReg(BAROMETER_BASE, BARO_DVS_REG, 4);
	Baro_WriteReg(BAROMETER_BASE, BARO_DATARATE_REG, 10000000);
#elif BARO_TYPE == BARO_I2C
#endif
	//Enable IP
	Baro_WriteReg(BAROMETER_BASE, BARO_IP_ENABLE_REG, 1);
}

void Gauge::baro_read_prom(void){
	for(int i=0; i<8; i++)
		Coeff[i] = Baro_ReadReg(BAROMETER_BASE,(i+1)*4);
}

bool Gauge::baro_promisValid(void){
	int i, j;
	u16 crc = 0, crc_origin = Coeff[7] & 0x000F;
	Coeff[7] &= 0xFF00;

	for (i = 0; i < 8 * 2; i++) {
		if (i % 2 == 1)
			crc ^= Coeff[i >> 1] & 0x00FF;
		else
			crc ^= Coeff[i >> 1] >> 8;

		for (j = 8; j > 0; j--) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x3000;
			else
				crc <<= 1;
		}
	}

	crc = (crc >> 12) & 0x000F;

	return ((crc_origin != 0x0000) && (crc == crc_origin));
}

void Gauge::baro_processData(void){
	s32 dT;
	s32 TEMP;
	s64 OFF;
	s64 SENS;
	s32 T1    = 0;
	s32 OFF1  = 0;
	s64 SENS1 = 0;

	grawBaroMeasurement.Pre_digit = static_cast<u32>(Baro_ReadReg(BAROMETER_BASE,BARO_PRES_REG));
	grawBaroMeasurement.Temp_digit = static_cast<u32>(Baro_ReadReg(BAROMETER_BASE,BARO_TEMP_REG));
//============================================================================================================================
	 dT   = (s32)grawBaroMeasurement.Temp_digit - ((s32)Coeff[5] << 8);


	 OFF  = (((s64)Coeff[2]) << 16) + ((((s64)dT * (s64)Coeff[4]) >> 7));
	 SENS = (((s64)Coeff[1]) << 15) + ((((s64)dT * (s64)Coeff[3]) >> 8));

	 TEMP = (s32)((((u64)Coeff[6] * dT) >> 23) + 2000);


	 if(TEMP < 2000) // if temperature lower than 20 Celsius
	 {
	   T1    = (s32)(((s64)dT *(s64)dT) >> 31);
	   OFF1  = ( 5 * ( ( ( TEMP - 2000 ) * ( TEMP - 2000 ) ) >> 1 ) );
	   SENS1 = ( 5 * ( ( ( TEMP - 2000 ) * ( TEMP - 2000 ) ) >> 2 ) );

	   if(TEMP < -1500) // if temperature lower than -15 Celsius
	   {
		   	OFF1  = OFF1  + 7  * ( ( ( TEMP + 1500 ) * ( TEMP + 1500 ) ) );
		   	SENS1 = SENS1 + 11 * ( ( ( TEMP + 1500 ) * ( TEMP + 1500 ) ) >> 1);
	   }

	   TEMP -= T1;
	   OFF -= OFF1;
	   SENS -= SENS1;
	 }

	 grawBaroMeasurement.Temperature = (float)(TEMP - T1)/100.0f;
	 grawBaroMeasurement.Pressure  = (u32)(((((grawBaroMeasurement.Pre_digit) * SENS) >> 21) - OFF) >> 15);
}

#if MAGNETOMETER_TYPE == USE_MMC

void Gauge::initMMC(void)
{
	VT_MMC5883_MA_mWriteReg(FLYEYE_MMC_BASE, 7*4, 1000000);   //sample rate 100hz
	VT_MMC5883_MA_mWriteReg(FLYEYE_MMC_BASE, 3*4, 250);   //fast mode: 400khz
	VT_MMC5883_MA_mWriteReg(FLYEYE_MMC_BASE, 1*4, 1);   //Enable IP core
	VT_MMC5883_MA_mWriteReg(FLYEYE_MMC_BASE, 5*4, 1);   //Enable request data
}
#endif


void Gauge::magnetProcessData(void)
{
	gaugeParamT* p = &gGaugeParam;

#if USE_WSM == WSM_ENABLE
//	uint16_t magX_raw, magY_raw, magZ_raw;
//	float magX_tmp, magY_tmp, magZ_tmp;
	int16_t magX_tmp, magY_tmp, magZ_tmp;
	if(TtyCAM != NULL)
	{
		wsm_msg_res_t wsmData;
		if(TtyCAM->getWsmData(wsmData))
		{

//		    unsigned int input = 0;
////		    float* fptr = (float*)&input;
//			magX_raw = (uint16_t)((wsmData.magX + 2048)) << 4;
//			input = (unsigned int)magX_raw;
//			magX_tmp = (float)input;
//			magY_raw = (uint16_t)((wsmData.magY + 2048)) << 4;
//			input = (unsigned int)magY_raw;
//			magY_tmp = (float)input;
//			magZ_raw = (uint16_t)((wsmData.magZ + 2048)) << 4;
//			input = (unsigned int)magZ_raw;
//			magZ_tmp = (float)input;

			magX_tmp = wsmData.magX;
			magY_tmp = wsmData.magY;
			magZ_tmp = wsmData.magZ;
			//X-axis
			if(p->magneXIndex == 1)
			{
				gMagnetRawMeasurement.magX = magX_tmp;
			}else if(p->magneXIndex == 2)
			{
				gMagnetRawMeasurement.magX = magY_tmp;
			}else if(p->magneXIndex == 3)
			{
				gMagnetRawMeasurement.magX = magZ_tmp;
			}else
			{
				gMagnetRawMeasurement.magX = magX_tmp;
			}
			//Y-axis
			if(p->magneYIndex == 1)
			{
				gMagnetRawMeasurement.magY = magX_tmp;
			}else if(p->magneYIndex == 2)
			{
				gMagnetRawMeasurement.magY = magY_tmp;
			}else if(p->magneYIndex == 3)
			{
				gMagnetRawMeasurement.magY = magZ_tmp;
			}else
			{
				gMagnetRawMeasurement.magY = magY_tmp;
			}
			//Z-axis
			if(p->magneZIndex == 1)
			{
				gMagnetRawMeasurement.magZ = magX_tmp;
			}else if(p->magneZIndex == 2)
			{
				gMagnetRawMeasurement.magZ = magY_tmp;
			}else if(p->magneZIndex == 3)
			{
				gMagnetRawMeasurement.magZ = magZ_tmp;
			}else
			{
				gMagnetRawMeasurement.magZ = magZ_tmp;
			}

			gMagnetRawMeasurement.magX *= (int16_t)p->magneXFactor;
			gMagnetRawMeasurement.magY *= (int16_t)p->magneYFactor;
			gMagnetRawMeasurement.magZ *= (int16_t)p->magneZFactor;
		}else
		{
			Log.msgPrintf ("Failed to get data from WSM Comm");
		}
	}
#else
#if MAGNETOMETER_TYPE == USE_HMR
	int16_t magX_tmp, magY_tmp, magZ_tmp;
	magX_tmp = static_cast<int16_t>(HMR_mReadReg(FLYEYE_HMR_BASE, MAG_MAGX) & 0xffff);
	magY_tmp = static_cast<int16_t>(HMR_mReadReg(FLYEYE_HMR_BASE, MAG_MAGY) & 0xffff);
	magZ_tmp = static_cast<int16_t>(HMR_mReadReg(FLYEYE_HMR_BASE, MAG_MAGZ) & 0xffff);
#else
	int32_t magX_tmp, magY_tmp, magZ_tmp;
	magX_tmp = static_cast<int32_t>(VT_MMC5883_MA_mReadReg(FLYEYE_MMC_BASE, MAG_MAGX) & 0x0003ffff);
	magY_tmp = static_cast<int32_t>(VT_MMC5883_MA_mReadReg(FLYEYE_MMC_BASE, MAG_MAGY) & 0x0003ffff);
	magZ_tmp = static_cast<int32_t>(VT_MMC5883_MA_mReadReg(FLYEYE_MMC_BASE, MAG_MAGZ) & 0x0003ffff);
#endif
	if((abs(magX_tmp) > MAG_RAW_RANGE) || (abs(magY_tmp) > MAG_RAW_RANGE) || (abs(magZ_tmp) > MAG_RAW_RANGE))
	{
		gMagnetRawMeasurement.magnetometerPresent = 0;
		if(mag_Chk_Cnt % 20 == 0)
			Log.msgPrintf("HMR out of range");
		magErrorCnt++;
		mag_Chk_Cnt++;
	}
	else
	{
#endif

	mag_Chk_Cnt = 0;
	gMagnetRawMeasurement.magnetometerPresent = 1;
	gMagnetRawMeasurement.configurationRead = 1;

#if USE_WSM == WSM_ENABLE

#else
		//X-axis
		if(p->magneXIndex == 1)
		{
			gMagnetRawMeasurement.magX = magX_tmp;
		}else if(p->magneXIndex == 2)
		{
			gMagnetRawMeasurement.magX = magY_tmp;
		}else if(p->magneXIndex == 3)
		{
			gMagnetRawMeasurement.magX = magZ_tmp;
		}else
		{
			gMagnetRawMeasurement.magX = magX_tmp;
		}
		//Y-axis
		if(p->magneYIndex == 1)
		{
			gMagnetRawMeasurement.magY = magX_tmp;
		}else if(p->magneYIndex == 2)
		{
			gMagnetRawMeasurement.magY = magY_tmp;
		}else if(p->magneYIndex == 3)
		{
			gMagnetRawMeasurement.magY = magZ_tmp;
		}else
		{
			gMagnetRawMeasurement.magY = magY_tmp;
		}
		//Z-axis
		if(p->magneZIndex == 1)
		{
			gMagnetRawMeasurement.magZ = magX_tmp;
		}else if(p->magneZIndex == 2)
		{
			gMagnetRawMeasurement.magZ = magY_tmp;
		}else if(p->magneZIndex == 3)
		{
			gMagnetRawMeasurement.magZ = magZ_tmp;
		}else
		{
			gMagnetRawMeasurement.magZ = magZ_tmp;
		}

		gMagnetRawMeasurement.magX *= (int16_t)p->magneXFactor;
		gMagnetRawMeasurement.magY *= (int16_t)p->magneYFactor;
		gMagnetRawMeasurement.magZ *= (int16_t)p->magneZFactor;
	}
#endif

	last_magX =  magX_tmp;
	last_magY =  magY_tmp;
	last_magZ =  magZ_tmp;

}

void Gauge::initDiffPressure(void)
{
	DiffPressureWrite(DIFF_PRESSURE_BASE, DiffPress_DataRate_Reg, 1000000);    //100Hz
	DiffPressureWrite(DIFF_PRESSURE_BASE, DiffPress_EnableIP_Reg, 1);		   //Enable Ip
}

void Gauge::diffPressProcessData(void)
{
	const float P_max = PRESS_RANGE_MAX;
	const float P_min = -P_max;
	uint8_t raw_bytes[4];

	raw_bytes[0] =  static_cast<uint8_t>(DiffPressureRead(DIFF_PRESSURE_BASE, DiffPress_RD_BYTE_ONE));
	raw_bytes[1] =  static_cast<uint8_t>(DiffPressureRead(DIFF_PRESSURE_BASE, DiffPress_RD_BYTE_TWO));
	raw_bytes[2] =  static_cast<uint8_t>(DiffPressureRead(DIFF_PRESSURE_BASE, DiffPress_RD_BYTE_THREE));
	raw_bytes[3] =  static_cast<uint8_t>(DiffPressureRead(DIFF_PRESSURE_BASE, DiffPress_RD_BYTE_FOUR));

    uint32_t data = (raw_bytes[0] << 24) |
                    (raw_bytes[1] << 16) |
                    (raw_bytes[2] << 8)  |
                    raw_bytes[3];


    if ((data >> STATUS_SHIFT)) {
        // anything other then 00 in the status bits is an error
//        Debug("DLVR: Bad status read %d", data >> STATUS_SHIFT);
    	Log.msgPrintf("Diff Pressure: Bad status read %d", data >> STATUS_SHIFT);
        return;
    }

    uint32_t pres_raw = (data >> PRESSURE_SHIFT)    & PRESSURE_MASK;
    uint32_t temp_raw = (data >> TEMPERATURE_SHIFT) & TEMPERATURE_MASK;

    float diff_press_h2o = (pres_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min;
    float temp = temp_raw*(200.0f/2047.0f) - 50.0f;

    _pressure_sum += INCH_OF_H2O_TO_PASCAL*diff_press_h2o;
    _temp_sum += temp;
    _diffPress_Cnt++;

    if(_diffPress_Cnt == DiffPress_Divisor)
    {
    	grawDiffPressureMeasurement.Pressure = _pressure_sum/DiffPress_Divisor;
    	grawDiffPressureMeasurement.Temperature = _temp_sum/DiffPress_Divisor;
    	_pressure_offset = _pressure_sum/DiffPress_Divisor;
    	_diffPress_Cnt = 0;
    	_pressure_sum = 0.0f;
    	_temp_sum = 0.0f;
    	// differential pressure, cut negative values.
    	grawDiffPressureMeasurement.Pressure -= gGaugeParam.diffPressOffset;
        if (grawDiffPressureMeasurement.Pressure < 0.0f)
        	grawDiffPressureMeasurement.Pressure = 0.0f;
    }
}

bool Gauge::diffPressureGetOffset(void)
{
	gGaugeParam.diffPressOffset = _pressure_offset;
	Log.msgPrintf("Diff Pressure: Get Offset %f --> need ps save gauge config", gGaugeParam.diffPressOffset);
	return true;
}

//--------------------------------------------------------------------------
// ADS1015 function declarations
//--------------------------------------------------------------------------
void Gauge::initADS(void)
{
	// software reset
	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_SOFTWARE_RESET_OFFSET, 0);
	// setting param
	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_PGA_OFFSET, 1);

	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_OP_MODE_OFFSET, 0);

	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_DATA_RATE_OFFSET, 4);

	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_WR_TIMEOUT_OFFSET, 3);

	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_RD_TIMEOUT_OFFSET, 10);

	// ip enable

	ADS1015_mWriteReg(FLYEYE_ADS_BASE, ADS_IP_EN_OFFSET, 1);

}

void Gauge::adsProcessData(void)
{
	grawAdsMeasurement.tempEnv 		= static_cast<uint16_t>(ADS1015_mReadReg(FLYEYE_ADS_BASE, ADS_CHANNEL_4));
	grawAdsMeasurement.rhEnv 		= static_cast<uint16_t>(ADS1015_mReadReg(FLYEYE_ADS_BASE, ADS_CHANNEL_5));
	grawAdsMeasurement.fuelLevel 	= static_cast<uint16_t>(ADS1015_mReadReg(FLYEYE_ADS_BASE, ADS_CHANNEL_6));
}

float Gauge::chebyshev_lowpass_filter_accl_WB (float input, float &state1, float &state2, float *coeff_a, float *coeff_b, float gainInput)
{
	float output;
	float d;

	d = gainInput * input - coeff_a[0] * state1 - coeff_a[1] * state2;

	output = d*coeff_b[0] + coeff_b[1] * state1 + coeff_b[2] * state2;
	state2 = state1;
	state1 = d;

	return output;
}

float Gauge::chebyshev_lowpass_filter_accl(float input, float &state1, float &state2, float &out1, float &out2, float *coff)
{
	float output;
	output = coff[0] * input + coff[1] * state1 + coff[2] * state2 + coff[3] * out1 + coff[4] * out2;
	state2 = state1;
	state1 = input;
	out2 = out1;
	out1 = output;
	return output;
}


