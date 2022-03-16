/**
*                                                                  
* @class PStateData                                                                                                                                
* Class representing data shared by the subsystem based on the PhysicalState class.                                   
* It is used as the interface beetwen susbsystems.                                                      
*                                                                   
* Class including data for communication and log.
*                                                                   
* 2008-2010 Witold Kruczek @ Flytronic                              
*/

#ifndef PSTATEDATA_H
#define PSTATEDATA_H


class DateTime
{
public:
    int  year;      ///< year (two last digits)
    int  month;     ///< month 
    int  day;       ///< day
    int  hours;     ///< hours
    int  mins;      ///< minutes
    int  secs;      ///< seconds
    int  msecs;     ///< miliseconds
    bool valid;     ///< time is valid (set)

    DateTime(void):
       year(0), month(1), day(1), hours(0), mins(0), secs(0), msecs(0), valid(false)
    {};
};


class PStateData
{
public:
    //  public fileds
    int time100;          ///< Relative time [*100us]
    float airspeed;       ///< Airspeed [kph]
    float tas;            ///< True Air Speed: IAS corrected for pressure and temperature [kph]
    float altitude;       ///< Altitude relative to place of takeoff (AAL) or AGL when the elevationOffset i set. During landing it is always altitude relative to landing place [m]
    Vector3f velned;
    GpsPosition position; ///< Geographic position [degrees] 
    GpsPosition rawGpsData;
    float psi;            ///< Euler angle (yaw) relative to ground [rad]
    float theta;          ///< Euler angle (pitch) relative to ground  [rad]
    float phi;            ///< Euler angle (roll) relative to ground  [rad]
    float psiDot;         ///< Angular velocity psi [rad/s]
    float thetaDot;       ///< Angular velocity theta [rad/s]
    float phiDot;         ///< Angular velocity phi [rad/s]
    float R;              ///< Angular velocity in Z plane axis [rad/s]
    float Q;              ///< Angular velocity in Y plane axis [rad/s]
    float P;              ///< Angular velocity in X plane axis [rad/s]
    float accZ;           ///< Acceleration in Z axis [g]
    float accY;           ///< Acceleration in Y axis [g]
    float accX;           ///< Acceleration in X axis [g]
    WindData wind;        ///< Wind data
    float groundspeed;    ///< Ground speed [kph]
    float track;          ///< Track angle [degree]
    float staticPressure; ///< Static pressure [hPa]
    float outerAirTemp;   ///< Outer temperature [C]
	float cameraPan;	  ///< Camera pan [degrees 0-360 in right]
	float cameraTilt;	  ///< Camera tilt [degrees 0:straight, -90: down, +-180: back, x: like -x but up]
	float cameraZoom;	  ///< Camera zoom [degrees]
	int irCamera;		  ///< Daylight camera: 0, thermovision 1
    DateTime dtUtc;       ///< Date and time UTC (valid from GPS has fixed)
	float gpsAmsl;		  ///< Altidute Amsl from Gps (ofrom version 0.485)
    float f32VertSpeed;
    float f32Altitude;
	float f32Rpm;
	float cpuTemp;
#if USE_DGPS == 1    
    float dgpsAltitude;
    float dgpsLonRange;
    float dgpsTrack;
    int   dgpsNumSat;
    float dgpsNorth;
    float dgpsEast;
    
    float dgpsAltitudePredict;
    float dgpsLonRangePredict;
    float dgpsTrackPredict;
#endif
	float elv;
	float leftAlr;
	float rightAlr;
	float rdr;
	float flp;
#if USE_CAM == CAM_ENABLE
	uint8_t cam_mode;			///<Che do hoat dong cua Camera, value = 0x00 [FREE], value = 0x02 [TRACK]
	int32_t object_px;			///<Toa do truc X [pixel] doi tuong bam tren anh
	int32_t object_py;			///<Toa do truc Y [pixel] doi tuong bam tren anh
	int32_t object_width;		///<Chieu rong anh
	int32_t object_height;		///<Chieu cao anh
	float hfov;					///<Truong nhin theo phuong ngang cua Camera [degrees]
	float gimbalPan;			///<Goc Pan [degrees]
	float gimbalTilt;			///<Goc Tilt [degrees]
	float gimbalRoll;			///<Goc Roll [degrees]
#endif

    PStateData(void) :
        time100(0), airspeed(0.0f), tas(0.0f), altitude(0.0f),
        psi(0.0f), theta(0.0f), phi(0.0f), psiDot(0.0f), thetaDot(0.0f), phiDot(0.0f),
        R(0.0f), Q(0.0f), P(0.0f), accZ(0.0f), accY(0.0f), accX(0.0f),
        groundspeed(0.0f), track(0.0f), staticPressure(0.0f), outerAirTemp(0.0f),
        cameraPan(0.0f), cameraTilt(0.0f), cameraZoom(0.0f), irCamera(0), gpsAmsl(0.0f),
		f32VertSpeed(0.0f), f32Altitude(0.0f), f32Rpm(0.0f), cpuTemp(0.0f)
#if USE_DGPS == 1
		, dgpsAltitude(0.0f), dgpsLonRange(0.0f), dgpsTrack(0.0f)
		, dgpsNumSat(0), dgpsNorth(0.0f), dgpsEast(0.0f)
        , dgpsAltitudePredict(0.0f), dgpsLonRangePredict(0.0f), dgpsTrackPredict(0.0f)
#endif
		, elv(0.0f), leftAlr(0.0f), rightAlr(0.0f), rdr(0.0f), flp(0.0f)
#if USE_CAM == CAM_ENABLE
		,cam_mode(0), object_px(0), object_py(0), object_width(0), object_height(0), hfov(0.0f), gimbalPan(0.0f), gimbalTilt(0.0f), gimbalRoll(0.0f)
#endif
    {};
};


/**
* Structure storing additionl sensors data
*/
#define MAX_ENGINE_SPEED 10000
#define MIN_ENGINE_SPEED 0

typedef union hubStatus_t
{
	struct bits
	{
  /* 0*/bool bPressureStatus     : 1;  // 1: error 0: norm
  /* 1*/bool bAccStatus          : 1;  // 1: error 0: norm
  /* 2*/bool bFuelStatus         : 1;  // 1: error 0: norm
  /* 3*/bool bPitotStatus        : 1;  // 1: error 0: norm
  /* 4*/bool bPitotHeater        : 1;  // 1: off 0: on
  /* 5*/bool bTemperatureOnBoard : 1;  // 1: > 55oC 0: < 55oC
  /* 6*/bool bRHStatus           : 1;  // 1: error 0: norm
  /* 7*/bool bTemperatureStatus  : 1;  // 1: error 0: norm
  /* 8*/bool bOverCurrent3       : 1;  // 1: over 0: norm
  /* 9*/bool bOverCurrent2       : 1;  // 1: over 0: norm
  /*10*/bool bOverCurrent1       : 1;  // 1: over 0: norm
  /*11*/bool bVoltageIgnition    : 1;  // 1: out of range 6.4-8.4V 0: in range
  /*12*/bool b5VOut              : 1;  // 1: out of range 4-5.2V 0: in range
  /*13*/bool b5VOut_2            : 1;  // 1: out of range 4-5.2V 0: in range
  /*14*/bool b5VOut_1            : 1;  // 1: out of range 4-5.2V 0: in range
  /*15*/bool bPitotHeaterMode    : 1;  // 1: manual mode 0: auto mode
	} bits;
	INT16U u16_status;
} hubStatus_t;

class AuxSensorData
{
public:
	int   i_engineSpeed;      //    0 - 10000  [rpm]
	int   i_fuelLevel;        //    0 - 100    [%]
	int   i_env_rh;		      //    0 - 99     [%]
	int   i_env_temp;         //  -40 - 100    [C]
	float f_voltage_1;        // 0.00 - 8.40   [V]
    float f_voltage_2;        // 0.00 - 8.40   [V]
	float f_current;          // 0.00 - 2.55   [A]
    int   i_pressure;         // 1000 - 120000 [Pa]
	int   i_engine_temp;	  //    0 - 100    [C]
	bool  b_hubDataAvailable;
	float festFuelLevel;       // Estimate
	int   timeFuelLevel;
	bool  bAllowEstTimeLeft;
	int   estFuelcount;
	hubStatus_t  t_hubStatus;
#if PILOT_TYPE == VUA_SL
	int   i_coolingOilTemp;   //  -40 - 400    [C]
	int   i_coolingWaterTemp; //  -40 - 400    [C]
	int   i_cylinderTemp;     //  -40 - 400    [C]
	int   i_oilPressure;      //    0 -        [Pa]
	int   i_orangeLed;        // status from TCU: 0x00: off, 0x01: solid, 0x02: blink
	int   i_redLed;           // status from TCU: 0x00: off, 0x01: solid, 0x02: blink
	int   i_engineSpeedTCU;   //    0 - 10000  [rpm]
	int   i_yellowLed;        // status from Rectifier: 0x00: off, 0x01: solid, 0x02: blink
#endif
	AuxSensorData (void) :
		i_engineSpeed (0), i_fuelLevel (0), i_env_rh (0), i_env_temp (0), 
		f_voltage_1 (0.0f), f_voltage_2 (0.0f), f_current (0.0f), i_pressure (0), i_engine_temp (0), b_hubDataAvailable(false),
		festFuelLevel (0.0f), timeFuelLevel (0), bAllowEstTimeLeft (false), estFuelcount(0)
#if PILOT_TYPE == VUA_SL
		, i_coolingOilTemp (38), i_coolingWaterTemp (12), i_cylinderTemp (45), i_oilPressure (50)
		, i_orangeLed (1), i_redLed (2), i_engineSpeedTCU (4000), i_yellowLed (2)
#endif
	{
		t_hubStatus.u16_status = 0;
	};
};

// structure packing
#pragma pack(1)

class PStateAuxSensorTlm
{
public:
	INT32U      u32_time;             //               [ms]
	INT16U      u16_engineSpeed;      //    0 - 10000  [rpm] x1
	INT8U       u8_fuelLevel;         //    0 - 100    [%]   x1
	INT8U       u8_env_rh;			  //    0 - 100    [%]   x1
	INT8U       u8_env_temp;          //  -40 - 100    [C]   x1
	INT16U      u16_voltage_1;        // 0.00 - 8.40   [V]   x100
    INT16U      u16_voltage_2;        // 0.00 - 8.40   [V]   x100
	INT8U       u8_current;           // 0.00 - 2.55   [A]   x100
	INT32U      u32_pressure;         // 1000 - 120000 [Pa]  x1
	INT8U		u8_engine_temp;		  //  -40 - 100    [C]   x1
	INT16U      u16_hubStatus;
#if PILOT_TYPE == VUA_SL
	INT16U      u16_coolingOilTemp;   //  -40 - 400    [C]   x1
	INT16U      u16_coolingWaterTemp; //  -40 - 400    [C]   x1
	INT16U      u16_cylinderTemp;     //  -40 - 400    [C]   x1
	INT16U      u16_oilPressure;      //    0 -        [Pa]  x1
	INT8U       u8_orangeLed;         // status from TCU: 0x00: off, 0x01: solid, 0x02: blink
	INT8U       u8_redLed;            // status from TCU: 0x00: off, 0x01: solid, 0x02: blink
	INT16U      u16_engineSpeedTCU;   //    0 - 10000  [rpm] x1
	INT8U       u8_yellowLed;         // status from Rectifier: 0x00: off, 0x01: solid, 0x02: blink
#endif

    void fillFrom (int time100us, const AuxSensorData &auxSensorData);
};


/**
* Structure for the telemetry data - short format(1) (used in radio communication)
*  NOTE: Has to be packed. For portability there are used types from the uC/OS-II system.
*/
class PStateTlmShort 
{
public:
    INT32U time;         ///< Relative time [ms]
    INT32S altitude;     ///< Alitude referenced to the takeoff place (AGL) [m * 100]
    INT32S posLat;       ///< Latitude [degrees * 1000000]
    INT32S posLon;       ///< Longitude [degrees * 1000000]
    INT16S psi;          ///< Euler angle (yaw) relative to ground [rad * 1000]
    INT16S theta;        ///< Euler angle (pitch) relative to ground [rad * 1000]
    INT16S phi;          ///< Euler angle (roll) relative to ground [rad * 1000]
    INT16S windFrom;     ///< Wind direction [degrees * 10]
    INT16S windSpeed;    ///< Wind speed [kph * 10]
    INT16S airspeed;     ///< Airspeed [kph * 10]
    INT16S groundspeed;  ///< Groundspeed [kph * 10]
    INT16S track;        ///< Track [degrees * 10] (relative to the geographical North)
    INT16S amslCorr;     ///< Alitude field correction to obtain amsl (amsl = altitude + amslCorr)
	INT32S gpsAmsl;      ///< GPS altitude (without prediction) [m * 100]

    void fillFrom (const PStateData &psd, float pAmslCorr, float pAalCorr);
};


/**
* Structure for the telemetry data - long format(2) (being saved in the log file)
*  NOTE: Has to be packed. For portability there are used types from the uC/OS-II system.
*/
class PStateTlmLong 
{
public:
    INT32U time;         ///< Relative time [ms]
    INT32S altitude;     ///< Alitude referenced to the takeoff place (AGL) [m * 100]
    INT32S posLat;       ///< Latitude [degrees * 1000000]
    INT32S posLon;       ///< Longitude [degrees * 1000000]
    INT16S psi;          ///< Euler angle (yaw) relative to ground [rad * 1000]
    INT16S theta;        ///< Euler angle (pitch) relative to ground [rad * 1000]
    INT16S phi;          ///< Euler angle (roll) relative to ground [rad * 1000]
    INT16S R;            ///< Angular velocity in Z planes axis [rad/s * 1000]
    INT16S Q;            ///< Angular velocity in Y planes axis  [rad/s * 1000]
    INT16S P;            ///< Angular velocity in X planes axis  [rad/s * 1000]
    INT16S accZ;         ///< Acceleration in Z axis [g * 1000]
    INT16S accY;         ///< Acceleration in Y axis [g * 1000]
    INT16S accX;         ///< Acceleration in X axis [g * 1000]
    INT16S windFrom;     ///< Wind direction (from which direction blows) [degrees * 10]
    INT16S windSpeed;    ///< Wind speed [kph * 10]
    INT16S airspeed;     ///< Airspeed [kph * 10]
    INT16S groundspeed;  ///< Groundspeed [kph * 10]
    INT16S track;        ///< Track [degrees * 10] (relative to the geographical North)
    INT16S amslCorr;     ///< Alitude field correction to obtain amsl (amsl = altitude + amslCorr)
    INT16S TAS;          ///< True Airspeed (airspeed corrected with the pressure and temperature)[kph * 10]
    INT8S  OAT;          // Outer Air Temperature [C]
	INT32S gpsAmsl;      ///< GPS altitude (without prediction) [m * 100]

    void fillFrom (const PStateData &psd, float pAmslCorr, float pAalCorr);
};


/**
* Structure for the telemetry from sensors - raw sensors data format
*  NOTE: Has to be packed. For portability there are used types from the uC/OS-II system.
*/
class PStateRawGaugeTlm /* parasoft-suppress  INIT-06 "Rekord telemetrii jest zawsze ustawiany przez fillFrom" */
{
public:
    INT32U time;         ///< Relative time [ms]
    INT16S accZL;        ///< Acceleration in Z axis (sensor L)
    INT16S accYL;        ///< Acceleration in Y axis (sensor L)
    INT16S accXL;        ///< Acceleration in X axis (sensor L)
    INT16S accZH;        ///< Acceleration in Z axis (sensor H)
    INT16S accYH;        ///< Acceleration in Y axis (sensor H)
    INT16S accXH;        ///< Acceleration in X axis (sensor H)
    INT16S gyroZL;       ///< Angular velocity in Z axis (sensor L)
    INT16S gyroYL;       ///< Angular velocity in Y axis (sensor L)
    INT16S gyroXL;       ///< Angular velocity in X axis (sensor L)
    INT16S gyroZH;       ///< Angular velocity in Z axis (sensor H)
    INT16S gyroYH;       ///< Angular velocity in Y axis (sensor H)
    INT16S gyroXH;       ///< Angular velocity in X axis (sensor H)
    INT32U press1;       ///< Pressure (sensor 1)
    INT32U press2;       ///< pressure (sensor 2)
	INT16U temp1;        ///< Temperature (sensor 1)
    INT16U temp2;        ///< Temperature (sensor 2)
	INT8U  accOverflow;  ///< Accelerometer overflow (1-H, 0-L)
	INT8U  gyroOverflow; ///< Gyro overflow (1-H, 0-L)
	INT32S magX;		 ///< Magnetometer X axis
	INT32S magY;		 ///< Magnetometer Y axis
	INT32S magZ;		 ///< Magnetometer Z axis
	INT32S diffPress;	 ///< Differential pressure
	INT32U staticPress;  ///< Static pressure
	INT16U gyroZTemperature;	///< Temperature (gyroscope Z)
	INT16U gyroYTemperature;	///< Temperature (gyroscope Y)
	INT16U gyroXTemperature;	///< Temperature (gyroscope X)
	INT8U  ahrsUpdateFlag;		///< Flag to show if in current cycle the correction of the ahrs from accelerometers and magnetometer was done bit0 - phi correction, bit1 - theta correction, bit2 - psi correction
	INT16S inclError;	 ///< Magnectic vector inclination error position [rad * 1000]

	void fillFrom (int time100us, const Gauge::rawMeasurementT &rm,const Gauge::rawDiffPressureMeasurementT &dp,const Gauge::rawBaroMeasurementT &ba, const Gauge::rawMagnetMeasurementT &mm, const Ahrs::ahrs_dataT &ahrs, const PStateData &psd, const AuxSensorData &auxSensorData);
};


//  end of packing
#pragma pack()

#endif  //  PSTATEDATA_H
