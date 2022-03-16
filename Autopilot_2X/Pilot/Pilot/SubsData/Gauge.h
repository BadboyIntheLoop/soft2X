/**
*                                                                 
* Gauge
*
* 2008 Grzegorz Tyma @ Flytronic
*/

#ifndef GAUGE_H
#define GAUGE_H


#define ADIS_INT_ID_OFFSET			3
#define ADIS_INT_ID					XPAR_FABRIC_XLCONCAT_0_DOUT_INTR + ADIS_INT_ID_OFFSET
/* ADIS16375 Addresses of registers */
/*	Name				R/W		Flash	PAGE_ID		Address			Default			Register Description								Format */
#define	DIAG_STS		/*R		No		0x00*/		0x0200			/*0x0000		Output self test error flags 						Table 2*/
#define TEMP_OUT		/*R		No		0x00*/		0x1C00			/*N/A			Output temperature									Table 4*/
#define TIME_STAMP		/*R		No		0x00*/		0x1E00			/*N/A			Output time stamp									Table 4*/
#define DATA_CNTR		/*R		No		0x00*/		0x2200			/*N/A			New data counter									Table 4*/

#define X_GYRO_LOW		/*R		No		0x00*/		0x0400			/*N/A			Output x-axis gyroscope	low word					Table 5	*/
#define	X_GYRO_OUT		/*R		No		0x00*/		0x0600			/*N/A			Output x-axis gyroscope	High word					Table 6	*/
#define Y_GYRO_LOW		/*R		No		0x00*/		0x0800			/*N/A			Output y-axis gyroscope	low word					Table 7	*/
#define	Y_GYRO_OUT		/*R		No		0x00*/		0x0A00			/*N/A			Output y-axis gyroscope	High word					Table 8*/
#define Z_GYRO_LOW		/*R		No		0x00*/		0x0C00			/*N/A			Output z-axis gyroscope	low word					Table 9		*/
#define	Z_GYRO_OUT		/*R		No		0x00*/		0x0E00			/*N/A			Output z-axis gyroscope	High word					Table 10*/

#define X_ACCL_LOW		/*R		No		0x00*/		0x1000			/*N/A			Output x-axis accelerometer low word				Table 11*/
#define	X_ACCL_OUT		/*R		No		0x00*/		0x1200			/*N/A			Output x-axis accelerometer	High word				Table 12*/
#define Y_ACCL_LOW		/*R		No		0x00*/		0x1400			/*N/A			Output y-axis accelerometer	low word				Table 13*/
#define	Y_ACCL_OUT		/*R		No		0x00*/		0x1600			/*N/A			Output y-axis accelerometer	High word				Table 14*/
#define Z_ACCL_LOW		/*R		No		0x00*/		0x1800			/*N/A			Output z-axis accelerometer	low word				Table 15*/
#define	Z_ACCL_OUT		/*R		No		0x00*/		0x1A00			/*N/A			Output z-axis accelerometer	High word				Table 16*/

#define	X_DELTA_ANG_L	/*R		No		0x00*/		0x2400			/*N/A			Output x-axis delta angle low word					Table 17*/
#define	X_DELTA_ANG_H	/*R		No		0x00*/		0x2600			/*N/A			Output x-axis delta angle high word					Table 18*/
#define	Y_DELTA_ANG_L	/*R		No		0x00*/		0x2800			/*N/A			Output y-axis delta angle low word					Table 19*/
#define	Y_DELTA_ANG_H	/*R		No		0x00*/		0x2A00			/*N/A			Output y-axis delta angle high word					Table 20*/
#define	Z_DELTA_ANG_L	/*R		No		0x00*/		0x2C00			/*N/A			Output z-axis delta angle low word					Table 21*/
#define	Z_DELTA_ANG_H	/*R		No		0x00*/		0x2E00			/*N/A			Output z-axis delta angle high word					Table 22*/

#define	X_DELTA_VEL_L	/*R		No		0x00*/		0x3000			/*N/A			Output x-axis delta velocity low word				Table 17*/
#define	X_DELTA_VEL_H	/*R		No		0x00*/		0x3200			/*N/A			Output x-axis delta velocity high word				Table 18*/
#define	Y_DELTA_VEL_L	/*R		No		0x00*/		0x3400			/*N/A			Output y-axis delta velocity low word				Table 19*/
#define	Y_DELTA_VEL_H	/*R		No		0x00*/		0x3600			/*N/A			Output y-axis delta velocity high word				Table 20*/
#define	Z_DELTA_VEL_L	/*R		No		0x00*/		0x3800			/*N/A			Output z-axis delta velocity low word				Table 21*/
#define	Z_DELTA_VEL_H	/*R		No		0x00*/		0x3A00			/*N/A			Output z-axis delta velocity high word				Table 22*/

#define X_GYRO_OFF_L	/*R/W	Yes		0x02*/		0x4000			/*0x0000		Calibration offset x-axis gyroscope low word		Table 28*/
#define X_GYRO_OFF_H	/*R/W	Yes		0x02*/		0x4200			/*0x0000		Calibration offset x-axis gyroscope high word		Table 29*/
#define Y_GYRO_OFF_L	/*R/W	Yes		0x02*/		0x4400			/*0x0000		Calibration offset y-axis gyroscope low word		Table 30*/
#define Y_GYRO_OFF_H	/*R/W	Yes		0x02*/		0x4600			/*0x0000		Calibration offset y-axis gyroscope high word		Table 31*/
#define Z_GYRO_OFF_L	/*R/W	Yes		0x02*/		0x4800			/*0x0000		Calibration offset z-axis gyroscope low word		Table 32*/
#define Z_GYRO_OFF_H	/*R/W	Yes		0x02*/		0x4A00			/*0x0000		Calibration offset z-axis gyroscope high word		Table 33*/

#define X_ACCL_OFF_L	/*R/W	Yes		0x02*/		0x4C00			/*0x0000		Calibration offset x-axis accelerometer low word	Table 34*/
#define X_ACCL_OFF_H	/*R/W	Yes		0x02*/		0x4E00			/*0x0000		Calibration offset x-axis accelerometer high word	Table 35*/
#define Y_ACCL_OFF_L	/*R/W	Yes		0x02*/		0x5000			/*0x0000		Calibration offset x-axis accelerometer low word	Table 36*/
#define Y_ACCL_OFF_H	/*R/W	Yes		0x02*/		0x5200			/*0x0000		Calibration offset x-axis accelerometer high word	Table 37*/
#define Z_ACCL_OFF_L	/*R/W	Yes		0x02*/		0x5400			/*0x0000		Calibration offset x-axis accelerometer low word	Table 38*/
#define Z_ACCL_OFF_H	/*R/W	Yes		0x02*/		0x5600			/*0x0000		Calibration offset x-axis accelerometer high word	Table 39*/

#define FILT_CTRL		/*R/W	Yes		0x00*/		0x5C00			/*0x0000		Control, Bartlett window FIR filter 				Table 27*/
#define RANG_MDL		/*R		No		0x00*/		0x5E00			/*N/A			Measurement range (model specific) identifier		Table 27*/
#define MSC_CTRL		/*R/W	Yes		0x00*/		0x6000			/*0x00C1		Control, input/output and other miscellaneous 		Table 27*/
#define UP_SCALE		/*R/W	Yes		0x00*/		0x6200			/*0x07D0		Control, scale factor for input clock, PPS mode 	Table 27*/
#define DEC_RATE 		/*R/W	Yes		0x03*/		0x6400			/*0x0000		Control, output samples rate decimation			Table 53*/
#define NULL_CFG		/*R/W	Yes		0x03*/		0x6600			/*0x070A		Control, automatic bias correction configuration	Table 54*/
#define	GLOB_CMD		/*W		No		0x03*/		0x6800			/*N/A			Control, global commands							Table 49*/

#define	FIRM_REV		/*R		No		0x03*/		0x6C00			/*N/A			Identification, firmware revision					Table 49*/
#define	FIRM_DM			/*R		No		0x03*/		0x6E00			/*N/A			Identification, date code, day and month			Table 49*/
#define	FIRM_Y			/*R		No		0x03*/		0x7000			/*N/A			Identification, date code, year						Table 49*/
#define PROD_ID			/*R		Yes		0x00*/		0x7200			/*0x4053		Output Product identification number				Table 27*/
#define	SERIAL_NUM		/*R		Yes		0x02*/		0x7400			/*N/A			Serial number										Table 46*/
#define USER_SCR_1		/*R/W	Yes		0x03*/		0x7600			/*N/A			User Scratch Register 1								N/A*/
#define USER_SCR_2		/*R/W	Yes		0x03*/		0x7800			/*N/A			User Scratch Register 2								N/A*/
#define USER_SCR_3		/*R/W	Yes		0x03*/		0x7A00			/*N/A			User Scratch Register 3								N/A*/

#define	FLSH_CNT_L		/*R		Yes		0x02*/		0x7C00			/*N/A			Output, flash memory write counter, lower word		Table 47*/
#define	FLSH_CNT_H		/*R		Yes		0x02*/		0x7E00			/*N/A			Output, flash memory write counter, upper word		Table 48*/

#define GYRO_SCALE_FACTOR						0.025f   	/* LSB/deg/sec */
#define ACCL_SCALE_FACTOR   					0.00125f  	/* g */
#define TEMP_SCALE_FACTOR						0.1f	 	/* degC */

#define GYRO_SCALE_MAX	 						22887							/* Max scale for gyroscope       		   */
#define ANG_SCALE_MAX	 						32767							/* Max scale for angle           		   */
#define ACC_SCALE_MAX	 						21973						   	/* Max scale for Accelerometers            */
#define VELO_SCALE_MAX	 						32767							/* Max scale for Velocity             	   */

#define TEMP_MIN		   						-40										    /* Temperature min             */
#define TEMP_MAX		    					85									        /* Temperature max             */

#define TEMP_SCALE_MAX	 						10619			/* Full Scale for temperature from 25 to 80 degree             */
#define TEMP_SCALE_MIN							-11504			/* Full Scale for temperature from - 40 to 25 degree           */

#define IMU_GYRO_UNIT_RADIAN_PER_SECOND 		1 						     /* Unit for data output   */
#define IMU_GYRO_UNIT_DEGREE_PER_SECOND 		0

#define CORRECT_ID 								1
#define WRONG_ID 					    		0

#define SOFTWARE_RESET_TIME         			74000		/* Delay time (us) as specified in Data-sheet  */
#define FACTORY_CALIBRATION_RESTORE_TIME 		50000
#define FLASH_MEMORY_UPDATE_TIME 		   		375000
#define FLASH_MEMORY_TEST_TIME 			    	50000
#define FLASH_SELF_TEST 						10000
#define IMU_CALIBRATION_NUMBER 				 	1000

/* Define for Diff Pressure */
#define DIFF_PRESSURE_BASE						XPAR_SSC_I2C_PRESSURE_SENSOR_0_S00_AXI_BASEADDR
#define DiffPressureWrite						SSC_I2C_PRESSURE_SENSOR_mWriteReg
#define DiffPressureRead						SSC_I2C_PRESSURE_SENSOR_mReadReg
#define DiffPressure_I2C_ADDR					0x28

#define STATUS_SHIFT 							30
#define TEMPERATURE_SHIFT 						5
#define TEMPERATURE_MASK 						((1 << 11) - 1)
#define PRESSURE_SHIFT 							16
#define PRESSURE_MASK 							((1 << 14) - 1)
#define INCH_OF_H2O_TO_PASCAL					248.84f
#define PRESS_RANGE_MAX							5.0f     /* inH2O */


/* Define for Barometer */

#if BARO_TYPE == BARO_SPI
	#define BAROMETER_BASE						XPAR_VIETTEL_SPI_BARO_CONTROLLER_0_S_AXI_BASEADDR
	#define Baro_WriteReg						VIETTEL_SPI_BARO_CONTROLLER_mWriteReg
	#define Baro_ReadReg						VIETTEL_SPI_BARO_CONTROLLER_mReadReg
#elif BARO_TYPE == BARO_I2C
	#define BAROMETER_BASE						XPAR_VIETTEL_BARO_CONTROLLER_0_S_AXI_BASEADDR
	#define BARO_WriteReg						VIETTEL_BARO_CONTROLLER_mWriteReg
	#define Baro_ReadReg						VIETTEL_BARO_CONTROLLER_mReadReg
#endif
#define BARO_UPDATE_DIVISOR						4				/* updateRate = 25 Hz */



class Gauge
{
public:
    //  Clas SystemMonitor has access to the private elements - readBattery
    friend class SystemMonitor;

    Gauge(void);

    static void initGauge(void); //if success returns 0;
    
    bool readMagnetometerConfiguration(float& xoff, float& yoff, float& zoff, float& r11, float& r12, float& r13, float& r21, float& r22, float& r23,
                                       float& r31,  float& r32,  float& r33,  float& phi,  float& theta, float& psi, int& magnetometerPresent) const;
    static float chebyshev_lowpass_filter_accl(float input, float &state1, float &state2, float &out1, float &out2, float *coff);
    void setDefaultConfig(void);
    void readData(void);	
    void calibrateData(void);
    void initADC(void);
    void setST1(int onOff);
    void initImu(void);
    void initBarometer(void);
    void initADS(void);
#if MAGNETOMETER_TYPE == USE_MMC
    void initMMC(void);
#endif
    void initDiffPressure(void);
    bool diffPressureGetOffset(void);

	bool storeMagnetometer(void);
    bool verifyMagnetometer(void);
#if MAGNETOMETER_TYPE == USE_HMR
    static bool isPacketHmrData(const char* hmrLine);
	bool hmrParseData(const char* hmrLine);
#endif
    // this structure contains calibrated and raw data (to be logged)
	struct measurementT
	{
		//accelerometers
		float accYL;
		float accXL;    
		float accZL;    
		float accYH;
		float accXH;
		float accZH;
		//gyro
		float gyroXL;
		float gyroYL;
		float gyroZL;
		float gyroXH;
		float gyroYH;
		float gyroZH;	
        //kamera
//#ifdef CAMERA_WB
		float cameraPan;
		float cameraTilt;
		float cameraZoom;
//#endif        

        float agl;
		float accFilterXL1;
		float accFilterXL2;
		float accFilterYL1;
		float accFilterYL2;
		float accFilterZL1;
		float accFilterZL2;
	};

    
	struct rawMeasurementT
	{
		//accelerometers
		unsigned short accYL;
		unsigned short accXL;    
		unsigned short accZL;    
		unsigned short accYH;
		unsigned short accXH;
		unsigned short accZH;
		//gyro
		unsigned short gyroXL;
		unsigned short gyroYL;
		unsigned short gyroZL;
		unsigned short gyroXH;
		unsigned short gyroYH;
		unsigned short gyroZH;
		//Gyro and accelerometers overflow
		int accOverflow;	//if 0 use L gauge, if 1 use H gauge
		int gyroOverflow;	//if 0 use L gauge, if 1 use H gauge
        //camera
//#ifdef CAMERA_WB
		int cameraPan;
		int cameraTilt;
		int cameraZoom;
//#endif

        int agl;

		// temperature measurement in gyroscopes.
		unsigned short gyroXTemperature;
		unsigned short gyroYTemperature;
		unsigned short gyroZTemperature;
	};

    // this structure contains data needed for guages calibration
	struct gaugeParamT
	{

		//accelerometer
		int   config;         // configuration variable: 
                              //   - bit15..bit1 - not used
                              //   - bit0:  accelerometer temperature compensation based on temperatures: 0 - scp1, 1 - gyroZL
        float accXLOff;       // zero offset
		float accXLSens;      // sensitivity
		float accXLSensT;     // temperature sensitivity
		float accYLOff;
		float accYLSens;
		float accYLSensT;
		float accZLOff;
		float accZLSens;
		float accZLSensT;
		float accXHOff;
		float accXHSens;
		float accXHSensT;
		float accYHOff;
		float accYHSens;
		float accYHSensT;
		float accZHOff;
		float accZHSens;
		float accZHSensT;
		//gyro
		float gyroXLOff;
		float gyroXLSens;
		float gyroXLSensT;
		float gyroYLOff;
		float gyroYLSens;
		float gyroYLSensT;
		float gyroZLOff;
		float gyroZLSens;
		float gyroZLSensT;
		float gyroXHOff;
		float gyroXHSens;
		float gyroXHSensT;
		float gyroYHOff;
		float gyroYHSens;
		float gyroYHSensT;
		float gyroZHOff;
		float gyroZHSens;
		float gyroZHSensT;
		//Swap accelerometer
		int	  acclXIndex;
		int	  acclYIndex;
		int	  acclZIndex;
		int   acclXFactor;
		int   acclYFactor;
		int   acclZFactor;
		
		//Swap gyroscope
		int	  gyroXIndex;
		int	  gyroYIndex;
		int	  gyroZIndex;
		int   gyroXFactor;
		int   gyroYFactor;
		int   gyroZFactor;

		// Magnetometer calibration parameters.
		float magnetX0ff;
		float magnetY0ff;
		float magnetZ0ff;
				
		float magnetR11;
		float magnetR12;
		float magnetR13;
		float magnetR21;
		float magnetR22;
		float magnetR23;
		float magnetR31;
		float magnetR32;
		float magnetR33;
		
		// assembly in airframe correction [degrees]
	    float magnetTheta;
		float magnetPhi;
		float magnetPsi;   

		float voltageOffset;
		float currentOffset;
		float currentSens;

		float diffPressOffset;

		int magEnable;
		int gyroTemperatureEnable;	// place of temperature measurement used in gyroscopes corections : 0 - SCP, 1 - inside gyroscopes

		float filterAcclCoff_a;
		float filterAcclCoff_b;
		float filterAcclCoff_c;
		float filterAcclCoff_d;
		float filterAcclCoff_e;
		bool  filterActive;

		//swap magnetometer
		int	  magneXIndex;
		int	  magneYIndex;
		int	  magneZIndex;
		int   magneXFactor;
		int   magneYFactor;
		int   magneZFactor;
	};

    struct magnetMeasurementT
	{
        float magX;
        float magY;
        float magZ;
        float diffPress;    //differential pressure.
        float magPressure;
        float magTemperature;
        int magnetometerPresent; //if 0: magnetometer not connected
    }; 

	struct rawMagnetMeasurementT
	{
#if MAGNETOMETER_TYPE == USE_HMR
        int16_t magX;
        int16_t magY;
        int16_t magZ;
#else
        int16_t magX;
        int16_t magY;
        int16_t magZ;
#endif
        float diffPress;
        unsigned int magPressure;
        unsigned short magTemperature;
        int magnetometerPresent; //if 0: magnetometer not connected
		int configurationRead;	 //if 0: calibration data not read from magnetometer
    };

	struct rawBaroMeasurementT
	{
		float Temperature;
		u32   Pressure;
		u32   Temp_digit;
		u32   Pre_digit;
	};
	struct rawDiffPressureMeasurementT
	{
		float Temperature;		//degC
		float Pressure;			//psi
	};
	struct rawAdsMeasurementT
	{
		uint16_t	rhEnv;
		uint16_t	tempEnv;
		uint16_t	fuelLevel;
	};
	
	struct gaugeParamT gGaugeParam;
	struct measurementT gMeasurement;
	struct rawMeasurementT gRawMeasurement;
    struct magnetMeasurementT gMagnetMeasurement;
    struct rawMagnetMeasurementT gMagnetRawMeasurement;
    struct rawBaroMeasurementT  grawBaroMeasurement;
    struct rawDiffPressureMeasurementT	grawDiffPressureMeasurement;
    struct rawAdsMeasurementT	grawAdsMeasurement;
	//low gyro overflow signal 0b000ZYX
	int gOverflowXYZ;
    
	float mag_kor_11, mag_kor_12, mag_kor_13;
	float mag_kor_21, mag_kor_22, mag_kor_23;
	float mag_kor_31, mag_kor_32, mag_kor_33;

	u32 Coeff[8];
	bool DatIsStarted;
	bool isAltValid;
	int updatePreCounter;
	
	// recalibrationStatus: 0 recalibration has not been done after turning on.
	//						1 under recalibration
	//						2 recalibration failed
	//						3 recalibration succed
	int recalibrationStatus;	
	bool startRecalibration();

    //  Lock static pressure measurement (for test purposes).
    bool lockMagPressureSensor;

private:    
    static const int GAUGE_RECALIBRATION_TIME = 15*100; //15 [s]

    static float currentMain; 
    static float voltageMain;

    struct magnetConfigurationT
	{
        unsigned int dac1;
        unsigned int dac2;
        unsigned int dac3;
        unsigned int dac4;
        float xoff;
        float yoff;
        float zoff;
        float r11;
        float r12;
        float r13;
        float r21;
        float r22;
        float r23;
        float r31;
        float r32;
        float r33;
		float phi;	// assembly in windg correction.
		float theta;
		float psi;
    }; 

	// magnetometer assembly in airframe corrections
	float theta_k;
	float phi_k;
	float psi_k;

    void initMagnetometer(void);
    struct magnetConfigurationT gMagnetConfiguration;

	void ahrs(void);
	bool isMagnetometerPresent (void) const;
	// This function reads current and voltage of battery  
    static void readBattery(float& current, float& voltage, float& currentDown, float& voltageDown, int& status);

    //-------------------------------------------------------
    // GYRO defines
    //-------------------------------------------------------
    static const int GYRO_BASE_ADDRESS          = 0x2000;
    static const int GYRO_CONTROL_ADDRESS       = 0*4;

    // We are able to inject a suitable current to SUMJ pin for null adjustment (6 DAC)
    // We have to control VDD for three gyros working in high range mode (3 DAC)
    // DACs are controlled via registers provided by Daniel

    static const int GYRO_X_L_SUMJ_ADC_ADDR     = 1*4;
    static const int GYRO_Y_L_SUMJ_ADC_ADDR     = 5*4;
    static const int GYRO_Z_L_SUMJ_ADC_ADDR     = 9*4;
    static const int GYRO_X_H_SUMJ_ADC_ADDR     = 2*4;
    static const int GYRO_Y_H_SUMJ_ADC_ADDR     = 6*4;
    static const int GYRO_Z_H_SUMJ_ADC_ADDR     = 10*4;

    static const int GYRO_X_H_VDD_ADC_ADDR      = 3*4;
    static const int GYRO_Y_H_VDD_ADC_ADDR      = 7*4;
    static const int GYRO_Z_H_VDD_ADC_ADDR      = 11*4;
    
    static const int GYRO_X_TEST_ADC_ADDR       = 4*4;
    static const int GYRO_Y_TEST_ADC_ADDR       = 8*4;
    static const int GYRO_Z_TEST_ADC_ADDR       = 12*4;

    //-------------------------------------------------------
    // MAGNETOMETER defines
    //-------------------------------------------------------

    // Address offsets, base address FLYEYE_MAGNET_BASE
    static const int MAG_CONTROL = 0;

    // Measurements
#if MAGNETOMETER_TYPE == USE_HMR
    static const int MAG_RAW_RANGE = 30000;
    static const int MAG_MAGX = 2*4;
    static const int MAG_MAGY = 3*4;
    static const int MAG_MAGZ = 4*4;
#else
    static const int MAG_RAW_RANGE = 256000;
    static const int MAG_MAGX = 0*4;
    static const int MAG_MAGY = 1*4;
    static const int MAG_MAGZ = 2*4;
    static const int MAG_TEMP = 3*4;
#endif

    //-------------------------------------------------------
    // ADS1015 defines
    //-------------------------------------------------------
	static const int ADS_SOFTWARE_RESET_OFFSET 		= 0*4;
	static const int ADS_SCL_CYCLE_OFFSET 			= 1*4;
	static const int ADS_PGA_OFFSET 				= 2*4;
	static const int ADS_OP_MODE_OFFSET 			= 3*4;
	static const int ADS_DATA_RATE_OFFSET  			= 4*4;
	static const int ADS_IP_EN_OFFSET 				= 5*4;
	static const int ADS_WR_TIMEOUT_OFFSET  		= 7*4;
	static const int ADS_RD_TIMEOUT_OFFSET  		= 8*4;

	static const int ADS_CHANNEL_0 					= 0*4;
	static const int ADS_CHANNEL_1 					= 1*4;
	static const int ADS_CHANNEL_2 					= 2*4;
	static const int ADS_CHANNEL_3 					= 3*4;
	static const int ADS_CHANNEL_4 					= 4*4;
	static const int ADS_CHANNEL_5					= 5*4;
	static const int ADS_CHANNEL_6					= 6*4;
	static const int ADS_CHANNEL_7					= 7*4;
	static const int ADS_STATUS						= 8*4;

#if MAGNETOMETER_TYPE == USE_HMR
    int16_t last_magX, last_magY, last_magZ;
#else
    int32_t last_magX, last_magY, last_magZ;
#endif
    uint16_t mag_Chk_Cnt;
    uint32_t magErrorCnt;
    static const int Magnet_Divisor = 1;
    int _magnet_cnt;
	#define MAG_SCALE_FACTOR  (1.0f/15.0f)
    // DAC offsets
    static const int MAG_DAC1 = 7*4;
    static const int MAG_DAC2 = 8*4;
    static const int MAG_DAC3 = 9*4;
    static const int MAG_DAC4 = 10*4;

    // magnetometer calibration
    static const int MAG_XOFF = 11*4;
    static const int MAG_YOFF = 12*4;
    static const int MAG_ZOFF = 13*4;
    static const int MAG_R11  = 14*4;
    static const int MAG_R12  = 15*4;
    static const int MAG_R13  = 16*4;
    static const int MAG_R21  = 17*4;
    static const int MAG_R22  = 18*4;
    static const int MAG_R23  = 19*4;
    static const int MAG_R31  = 20*4;
    static const int MAG_R32  = 21*4;
    static const int MAG_R33  = 22*4;
    static const int MAG_PHI  = 23*4;
    static const int MAG_THETA  = 24*4;
    static const int MAG_PSI  = 25*4;

    //-------------------------------------------------------
    // imu defines
    //-------------------------------------------------------
    static const int IMU_DIVISOR_REG	=	1*4;
    static const int IMU_WRCFCMD_REG	=	2*4;
    static const int IMU_CLR_IRQ_REG	=	3*4;
    static const int IMU_RDDATA_REG		=	4*4;
    static const int IMU_WRDATRATE_REG	=	7*4;
    static const int IMU_WRCFDATA_REG	=   8*4;
    static const int IMU_WRENABLE_REG	=	9*4;
    static const int IMU_WRDISABLE_REG	=	10*4;
    static const int IMU_RDCFDATA_REG	=	15*4;
    static const int IMU_RDDATAIDLE_REG	=	16*4;
    static const int IMU_RDCFIDLE_REG	=	17*4;
    //--------------------------------------------------------
    // Barometer defines
    //--------------------------------------------------------
    //write
#if BARO_TYPE == BARO_SPI
    static const int BARO_DVS_REG		= 1*4;
    static const int BARO_READPROM_REG	= 2*4;
#elif BARO_TYPE == BARO_I2C
    static const int  BARO_ADDR_REG		=	1*4;
    static const int BARO_PRESCALE		=	2*4;
#endif
	static const int BARO_IP_ENABLE_REG	=	3*4;
	static const int BARO_LOOP_ENABLE_REG =	4*4;
	static const int BARO_DATARATE_REG	=	5*4;

    //read
	static const int BARO_STATUS_REG	=	0*4;
	static const int BARO_COEFF_1_REG	=	1*4;
	static const int BARO_COEFF_2_REG	=	2*4;
	static const int BARO_COEFF_3_REG	=	3*4;
	static const int BARO_COEFF_4_REG	=	4*4;
	static const int BARO_COEFF_5_REG	=	5*4;
	static const int BARO_COEFF_6_REG	=	6*4;
	static const int BARO_COEFF_7_REG	=	7*4;
	static const int BARO_COEFF_8_REG	=	8*4;
	static const int BARO_PRES_REG		=	9*4;
	static const int BARO_TEMP_REG		=	10*4;
	static const int BARO_IS_VALID		=   11*4;

    //--------------------------------------------------------
    // Different Pressure defines
    //--------------------------------------------------------
	static const int DiffPress_Sensor_Addr_Reg 	= 1*4;
	static const int DiffPress_DataRate_Reg		= 2*4;
	static const int DiffPress_FreqDivisor_Reg	= 3*4;
	static const int DiffPress_EnableIP_Reg		= 4*4;
	static const int DiffPress_RD_BYTE_ONE		= 0*4;
	static const int DiffPress_RD_BYTE_TWO		= 1*4;
	static const int DiffPress_RD_BYTE_THREE	= 2*4;
	static const int DiffPress_RD_BYTE_FOUR		= 3*4;
	static const int DiffPress_RD_ACK_REG		= 4*4;

	static const int DiffPress_Divisor = 2;
	int _diffPress_Cnt;
	float _pressure_sum;
	float _temp_sum;
	float _pressure_offset;



    typedef union magnetStatus_t
	{
        struct bits 
		{
            u32 mf           : 1; // magnetometer flag (1 - valid data, write 1 to clear)
            u32 ptf          : 1; // press, temperature flag (1 - valid data, write 1 to clear)
            u32 cf           : 1; // config flag (1 - valid data, write 1 to clear)
            u32 rc           : 1; // read config (write 1 to force configuration read)
            u32 dummy        : 28;
        } bits;
        u32 word;
    } magnetStatus_t;


    void calibrateMagnetometerData(void);
    void requestMagnetometerConfiguration(void);

    //Answer codes
    static const int RS485_OK			= 0x0;
    static const int RS485_ERROR		= 0x1;


	//recalibration
	int reCalibrateCounter;
	int reCalibrateDuration;
	bool reCalibrateFlag;
	float gyroXLOffNew, gyroYLOffNew, gyroZLOffNew;
	float gyroXHOffNew, gyroYHOffNew, gyroZHOffNew;
	float accXUnderCalib;
	float accYUnderCalib;
	float accZUnderCalib;
	float acclXDly[2];				///< For chebyshev filter
	float acclYDly[2];				///< For chebyshev filter
	float acclZDly[2];				///< For chebyshev filter
	float filterAcclXDly[2];		///< For chebyshev filter
	float filterAcclYDly[2];		///< For chebyshev filter
	float filterAcclZDly[2];		///< For chebyshev filter

//	float gyroXDly[2];				///< For chebyshev filter
//	float gyroYDly[2];				///< For chebyshev filter
//	float gyroZDly[2];				///< For chebyshev filter
//	float filterGyroXDly[2];		///< For chebyshev filter
//	float filterGyroYDly[2];		///< For chebyshev filter
//	float filterGyroZDly[2];		///< For chebyshev filter

	int reCalibrateGyros(int calibrationDuration);

	//Adis function defines
    bool imu_is_ready(void);
    bool imu_not_receive_data(void);
    void start_imu_data(void);
    void stop_imu_data(void);
    void imu_wr_cmd(u32 cmd);
    void imu_wr_dvsr(u8 dvsr);
    void imu_wr_freq_irq(u32 imu_spi_base, u16 freq_irq);
    s16 imu_rd_data(void);
    u16 imu_prod_id(void);
    void imu_reset(void);
    void imu_sample_rate_set(u16 sam_rate);
    void imu_filt_ctrl(u16 numOfTab);
    void imu_flash_update_cmd(void);
    void imu_bias_correction_update(void);
    void imu_factory_calibration_restore(void);
    void imu_bias_null_cfg(void);
    static void imu_handle_int();
    void imu_init_interrupt(uint32_t Int_Id, UCOS_INT_FNCT_PTR Handler, uint8_t Priority, UCOS_INT_TYPE Type);

    //Barometer function
    void baro_read_prom(void);
    bool baro_promisValid(void);
    void baro_processData(void);
    //Diff-Pressure function defines
    void diffPressProcessData(void);
    void magnetProcessData(void);
    void adsProcessData(void);

    float chebyshev_lowpass_filter_accl_WB(float input, float &state1, float &state2, float *coff_a, float *coff_b, float gainInput);
	//float chebyshev_lowpass_filter_accl(float input, float &state1, float &state2, float &out1, float &out2, float *coff);

};

#endif  // GAUGE_H
