#ifndef PHYSICALSTATE_H
#define PHYSICALSTATE_H

/** \file
* \brief Declartion of a physical system state accessor class
*/

/** Class provides information about current physical state of a flying platform. Subsystem is responsible for reading and maintaining raw data
* from all sensors (pre-filtered in hardware), readig  goegraphical coordinates, air speed, ground speed, wind speed, way angle, and GPS altitude.
* At the time of reading, data are extrapolated. Subsystem also normalizes values by converting them to floats that matches the assumed physical units,
* prepares a set of data form a multiple data sources, detects a sensors malfunction and exceeds of the measurement range. It also makes a convertion of
* acquired angular values in to ground coordinates system and make a calculations of an angles in it. Subsystem also calculates a vector of a wind (several
* algorithms).
*/
/// Class implements the physical system state accessor
#define DGPS_UNKNOWN 0
#define DGPS_NOVATEL 1
#define DGPS_TRIMBLE 2

class PhysicalState: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:
    PhysicalState (void);
    void  task (const void* pdata);              ///< System tasks handler method
    void  linkObserver();                        ///< Observed objects linker method
    bool  getPStateData (PStateData &psd);       ///< Physical state data getter method
    bool  getPStateHealth (PStateHealth &psh);   ///< Entire subsystem condition (status) checker method
    bool  getWindData (WindData &w);             ///< Wind parameters getter method
    bool  getAuxSensorData (AuxSensorData &asd); ///< Auxiliary sensors data getter method
    int   getSimLevel (void) const;              ///< Simulation level getter method
    int   getTime100 (void) const;               ///< Time since the start of the system getter method [*100us]
    bool  setAglZero (bool pUseTakeoffAgl);      ///< Current atlitude as AGL reference setter method
    void  setAglOffset (float offset);           ///< Offset for AGL altitude setter method [m]
    float getAglOffset (void) const;             ///< Offset for AGL altitude getter method
    int   getNotifyCyclesPerSecond (void) const; ///< Number of subsystem notifications per second getter method
    virtual bool isReadyForTakeoff (ClassifiedLine *cl, bool fLogComm);	///< Subsystem readiness for takeoff checker method
    virtual bool putLine (ClassifiedLine &cl);   ///< New command line to command queue inserter method
    void  useElevationOffsetPar (bool use);      ///< Parameter 'ElevationOffset' usage switcher method
    bool  isElevationOffsetOn (void) const;      ///< Parameter 'ElevationOffset' useage checker method
    float getElevationOffset (void) const;       ///< Elevation offset getter method
    void  setIasFailMode (bool fail);            ///< Method sets appropriate working mode when an incorrect speed reading occurs
#if EMRG_LOSS_ENGINE == 1
	bool  getTestLostEngine (void) const;
#endif	
	/// Number of data reads per second (to set according to the hardware or simulator setting)
    static const int GAUGE_CYCLES_PER_SECOND = 100;
#if USE_DGPS == ENABLED
	bool  getUseDGpsData (void) const;			 ///< Method to use the data from DGps
    int   getDgpsPredictionEnable (void);
#endif

protected:
    // Destructor is disabled - object should never be destroyed
    virtual ~PhysicalState(void){};

private:
    /// Time of a single period of reading data form a sensors [in 100*us]
    static const int GAUGE_TICK_INTERVAL_100 = 10000 / GAUGE_CYCLES_PER_SECOND;
    /// Flag indicates connection of a pressure sensor 1 to the static pressure when it's set to 'ture'
    static const bool STATIC_PRESSURE_FROM_GAUGE_1 = true;
    /** \name Minimum and maximum value of a ground stations pressure considered valid
	* \{
	*/
    static const float MIN_BASE_PRESSURE;
    static const float MAX_BASE_PRESSURE;
	///\}

    /// Value of a speed used in calculations when speed sensor is invalid
    static const float SENS_FAIL_IAS;
    
	/** \name Values of an inclination and declination used in detection of the lack of an initialization
	* \{
	*/
    static const float INCLINATION_NOT_SET;
    static const float DECLINATION_NOT_SET;
	///\}
    
	/// Period when ARHS correction is inactive (after receiving the FPR_SMOOTH_ON notification) [s]
    static const int AHRS_LOCK_TIME = 300;
    /// Log divisor value used for forcing the logging of telemetry (if it was off or slower ) 
    static const int FORCED_LOG_DIVISOR = 5;   //max = 20hz
    /// Subsystem name used as a prefix
    static const char SUBS_PREFIX[];
    /// Configuration data file name
    static const char CONF_FILE_NAME[];
    /// Sensors configuration file name
    static const char GAUGE_CONF_FILE_NAME[];

	/** \name Error messages
	* \{
	*/
    static const char ERR_OK[];
    static const char ERR_UCOMMAND[];
    static const char ERR_CLOAD[];
    static const char ERR_CSAVE[];
    static const char ERR_BADPAR[];
    static const char ERR_SYS[];
    static const char ERR_GCLOAD[];
    static const char ERR_GCSAVE[];
    static const char ERR_MAG_VERIFY[];
    static const char ERR_READONLY[];
    static const char MSG_NO_CONFIG[];
    static const char MSG_NO_GAUGE_CONFIG[];
    static const char MSG_NO_GPS[];
    static const char MSG_GPS_BAD_PRECISION[];
    static const char MSG_SENSORS_ERROR[];
    static const char MSG_BASE_PRESSURE[];
    static const char MSG_MAG_ERROR[];
    static const char MSG_INCLDECL_NOT_SET[];
    static const char MSG_SENSORS_SPRESS_LOCKED_ERROR[];
    static const char ERR_CMD_BUF_FULL[];
    static const char MSG_DIFF_PRESSURE_OFFSET[];
	///\}

	/** \name User Action Flag
	* Bit flags passed when definig the parameter (insertion) and returned after the particular parameter has been set.
    * Flags identify the function wchich must be executed after setting the parameter.
	* \{
	*/
    static const unsigned int UAF_BASE_PRESS       =  1; ///< Set the reference pressure of the base station if it is not set (out of range)
	static const unsigned int UAF_AHRS_DECL        =  2; ///< Change of magnetic declination
	static const unsigned int UAF_AHRS_INCL        =  4; ///< Change of magnetic inclination
    static const unsigned int UAF_GPS_FAIL         =  8; ///< Activate the GPS failure checking
	///\}

    /// Class representing the data from the simulator
    class SimData /* parasoft-suppress  INIT-06 "Fields do not have to be initialized because they are set in scanLine" */
    {
    public:
        float time;           ///< Relative time [s]
        float airspeed;       ///< Air speed [kph]
        float heading;        ///< Heading [deg] (angle relative to the longitudinal axis of the true north)
        float amsl;           ///< Above mean sea level (AMSL) [m]
        GpsPosition position; ///< Latitude and longitude [deg]
        float psi;            ///< Euler angle (yaw) relative to ground [rad]
        float theta;          ///< Euler angle (pitch) relative to ground [rad]
        float phi;            ///< Euler angle (roll) relative to ground [rad]
        float psiDot;         ///< Angular velocity of psi [rad/s]
        float thetaDot;       ///< Angular velocity of theta [rad/s]
        float phiDot;         ///< Angular velocity of phi [rad/s]
        float R;              ///< Angular velocity in Z axis of the plane [rad/s]
        float Q;              ///< Angular velocity in Y axis of the plane [rad/s]
        float P;              ///< Angular velocity in X axis of the plane [rad/s]
        float accZ;           ///< Acceleration in Z axis [g]
        float accY;           ///< Acceleration in Y axis [g]
        float accX;           ///< Acceleration in X axis [g]
        float windFrom;       ///< Wind direction (geographical direction whence the wind blows) [deg]
        float windSpeed;      ///< Wind speed [kph]
        float speedEast;      ///< Speed projected onto the ground (eastward) [kph]
        float speedNorth;     ///< Speed projected onto the ground (northward) [kph]
        float staticPressure; ///< Static pressure [inHg]
        float cameraPan;      ///< Angle of rotation of the camera in the vertical axis relative to the plane [deg: 0 to 360 to the left]
        float cameraTilt;     ///< Angle of rotation of the camera relative to x-y surface [deg: -90 to 90, 0:forward, -90:vertically downwards]
        float cameraZoom;     ///< Angle of view [deg]
        float outerAirTemp, verticalSpeed;
		float elv;
		float leftAlr;
		float rightAlr;
		float rdr;
		float flp;
		float rpm;
		float previousTime;
		GpsPosition prevPosition;

        bool scanLine (const char* line); ///< Method scans the line with data and sets the class attributes
        SimData(void) :
            time(0), airspeed(0.0f), heading(0.0f), amsl(0.0f),
            psi(0.0f), theta(0.0f), phi(0.0f), psiDot(0.0f), thetaDot(0.0f), phiDot(0.0f),
            R(0.0f), Q(0.0f), P(0.0f), accZ(0.0f), accY(0.0f), accX(0.0f),
            windFrom(0.0f), windSpeed(0.0f), speedEast(0.0f), speedNorth(0.0f),
            staticPressure(0.0f), cameraPan(0.0f), cameraTilt(0.0f), cameraZoom(0), outerAirTemp(0.0f),
            verticalSpeed(0.0f), elv(0.0f), leftAlr(0.0f), rightAlr(0.0f),
            rdr(0.0f), flp(0.0f), rpm(0.0f), previousTime(0.0f)
        {};
    };

	/// PhysicalState subsystem configuration data class
    class ConfigData
    {
    public:
        int   notifyDivisor;       ///< Number of samples after which PSTATE_CHANGED(1-n) notification will be send
		int   commDivisor;         ///<	Number of samples after which data will be send to the communication channel (0 to n, 0 disables sending)
		int   commAuxSensorDivisor;///<	Number of samples after which auxiliary data will be send to the communication channel (0 to n, 0 disables sending)
		int   logDivisor;          ///< Number of samples after which data will be send to the log (0 to n; 0 disables logging)
		int   logAuxSensorDivisor; ///< Number of samples after which auxiliary data will be send to the log (0 to n; 0 disables logging)
        int   nmeaCommDivisor;     ///< Number of GPS reading after which data will be send to the communication channel (0 to n)
        int   nmeaLogDivisor;      ///< Number of GPS reading after which data will be send to the log (0 to n)
        int   commFormat;          ///< Identifier of a data format send to the communication channel
        int   logFormat;           ///< Identifier of a data format send to the log
        bool  stateLogEnable;      ///< Flag enables logging the data of the subsystem state (0 or 1)
        bool  rawGaugeLogEnable;   ///< Flag enables logging the raw data from sensors (0 or 1)
		bool  rawGaugeCommEnable;  ///< Flag enables sending to the communication channel a row data from sensors (0 or 1)
		bool  auxSensorLogEnable;  ///< Flag enables logging the auxiliary data from sensors (0 or 1)
		bool  auxSensorCommEnable; ///< Flag enables sending to the communication channel a auxiliary data from sensors (0 or 1)
        float maxHdop;             ///< Maximum possible error of precision (Horizontal Dilution of Precision)(0 to 1000)
#if USE_DGPS == ENABLED
        float maxPdop;             ///< Maximum possible error of precision (Horizontal Dilution of Precision)(0 to 1000)
#endif
        float gpsTimeout;          ///< Maximum time the GPS is nonoperational[s] (0.25 to 100)
#if USE_DGPS == ENABLED
		float dgpsTimeout;         ///< Maximum time the DGPS is nonoperational[s] (0.25 to 100)
#endif
        bool  gpsPredictionEnable; ///< Flag enables prediction of GPS (extrapolation)(0 or 1)
        bool  gpsSbasEnable;       ///< Flag enables SBAS
        bool  gpsFailNavEnable;    ///< Flag enables alternative navigation to GPS after its breakdown
        int   gpsFailTestTimeout;  ///< Maximum time for simulation of an GPS malfunction after which GPS is restarted [s]
        int   simLevel;            ///< Level of simulation (0 to 4)
        bool  asFilter;			   ///< Flag enables airspeed filter
        bool  altFilter;           ///< Flag enables altitude filter
        int   pressureSource;      ///< Pressure source: 0-internal autopilots sensor, 1-external sensor
        bool  useBasePressure;     ///< Flag enables using the pressure of the base ground station in calculation of an altitude
        float takeoffAgl;          ///< AGL altitude during take-off (used in correction of an altitude) [m]
        float elevationSetDelay;   ///< Determination of elevation delay after power on (heating of the sensors) [s]
		int   ahrsVersion;         ///< Version of Ahrs
        bool  spSensorSwitchEnable;///<	Flag enables or disables the mechanism of switching the static pressure sensors when lock has been detected
		bool  bUseKFAlt;		   ///< Flag enables to use the Altitude from the Kalman filter
		float fuelLevelRollPitchRange; // Maximum Roll Pitch to allow to estimate Fuel Level
		float gpsUereConfig;
		float gpsPositionErrOffset;
		bool  gpsTestJammingEnable;
		float gpsJammingTimeOut;
		float filterAirspeedCoff_a;
		float filterAirspeedCoff_b;
		float filterAirspeedCoff_c;
		float filterAirspeedCoff_d;
		float filterAirspeedCoff_e;
		float filterAirspeedThreshold;
		bool filterAirspeedActive;
		float airSpdScaleRatio;
#if USE_DGPS == 1
		int   dgpsPredictionEnable;///< Flag enables to use prediction DGPS Data
		bool  bUseDGps;			   ///< Flag enable to use the data from DGPS
        int   dgpsDevice;
#endif
        int   debugMsg;

        ConfigData (void);         // Class constructor
        void setDefault (void);    ///< Default parameters value setter method
    };
    
    /// Class represents the state of the subsystem which is stored in non-volatile memory.
    class StateData
    {
    public:
        float elevationAmslSim;   ///< AMLS altitude of an elevation (simulation only!)[m]
        float elevationPressure;  ///< Pressure at ground [hPa]
        bool elevationIsSet;      ///< Flag of setting elevation and pressure at the start position
        bool useTakeoffAgl;       ///< Flag of  elevation of the plane during take-off  //Flaga uwzglêdniaj¹ca wysokoœæ wyrzutu samolotu podczas startu 
        float offsetAgl;          ///< Offset for a returning AGL altitude [m]
        float refBasePressure;    ///< Reference pressure of a base station
        float currentBasePressure;///< Current pressure of a base station [hPa]
        float tlmOriginAmsl;      ///< AMSL altitude of the take-off position (set manually, for telemetry only)
        int gaugeTicks;           ///< Incremented after each IRQ_ADC interruption
        int time100;              ///< [time * 100us]
        Ahrs ahrsObject;          ///< Object calculating the Euler angles
        WindResolver windRes;     ///< Object calculating direction and speed of a wind
        float elevationOffset;    ///< Offset of a reference elevation ('+' means up)
        bool useElvOffsetPar;     ///< Flag enables the 'ElevationOffset' parameter
        bool iasFailMode;         ///< Flag indicates a failure of reading of a speed (set externally)
		float inclination;		  ///< Magnetic inclination [rad]
		float declination;		  ///< Magnetic declination [rad]
        bool spSensorLocked;      ///< Flag indicates blocking of a static pressure sensor (flag is manually resetted)
        float spSensorDelta;      ///< Difference between static pressure measured on the wing and by the autopilot
        Vector2f gpsFailIncrS;    ///< Vector represents the way from the point of failure of GPS to the current position (estimated)
        GpsPosition gpsFailLastPos;///< Last valid coordinates before failure of a GPS

        StateData (void);
    };

	/** \name Tags obtained when registering of the observed object
	* \{
	*/
    int  _simTag,
         _nmeaTag,
         _auxNmeaTag,
		 _hmrTag,
         _irqAdcTag,
         _smoothOnTag,
         _smoothOffTag,
         _fPRealLaunchTag,
         _fPRealStandbyTag,
         _sysMonTag,
         _extCmdTag,
#if USE_CAM == CAM_ENABLE
		 _cameraTag,
#endif
         _cTty485Tag;

	///\}

    int  _notifyCounter,            
         _commCounter,
		 _commAuxSensorCounter,
         _logCounter,
         _hubCounter,
		 _logAuxSensorCounter,
#if USE_CAM == CAM_ENABLE
		 _camCounter,
#endif
         _nmeaCommCounter,
         _nmeaLogCounter;             ///< Counters


    CmdQueue<4>   _cmdq;               ///< Subsystems input command line queue (4 lines)
    PStateData    _psd;                ///< Subsystems data (sharing with other subsystems)
    PStateHealth  _psh;                ///< Subsystems   Dane o kondycji podsystemu
    SimData       _sim;                ///< Data from simulator
    ConfigData    _conf;               ///< Subsystem configuration data
    StateData     _stateMem;           ///< Subsystem state data (to the non-volatile memory)
    AuxSensorData _auxSensorData;     ///< Auxiliary data structure (e.g. fuel level)
    
	Semaphore     _vSem;               ///< Semaphore controlling access to the subsystems data (_psd and _psh)
    	
    FParser       _parser;             ///< Parser of a textual commands form communication channel
    Gauge         _gauge;              ///< Object handling the sensors
    bool          _gpsConfigured;      ///< Flag indicates that GPS module is configured
    bool          _gpsPacketStarted;   ///< Flag indicates that the first line from NMEA package has been received (processing has been started)
    int           _gpsBeforeInitLines; ///< Number of lines received from GPS between reset and initialization
    int           _elevationDlyCnt;    ///< Counter of delaying the calculation of an elevation after power on
    bool          _datetimeSet;        ///< Flag indicates that a date and time of a session has been specified
    GpsData       _currentGps;         ///< Current GPS data
    GpsData       _previousGps;        ///< Previous GPS data (needed for extrapolation)
    GpsData       _predictedGps;       ///< Predicted GPS data (extrapolated)
    GpsData       _prevPredictedGps;   ///< Predicted GPS data (extrapolated)
    GpsData       _tmpGps;             ///< Temporary buffer collecting NMEA lines
#if USE_DGPS == ENABLED
	Trimble       _DGpsTrimble;		  ///< DGPS Trimble data
    Novatel       _DGpsNovatel;        ///< DGPS Novatel data
    DGpsInterface *_DGpsData;
#endif
	bool          _isConfigLoaded;     ///< Flag indicates that configuration has been loaded from flash
    bool          _isGaugeConfigLoaded;///< Flag indicates that configuration of a magnetometer has been loaded from flash
	bool		  _isGaugeMagnetometerOK; ///< Flag indicates that the magnetometer is operational
    float		  _previousIas;		  ///< Previous speed value
    float		  _previousRawIas;		  ///< Previous raw aspeed value
	int 		  _iasCounter;		  ///< Decimation counter
	float		  _ias_1;			  ///< ias[-1]
	float		  _ias_2;			  ///< ias[-2]
    unsigned int  _lastRawMagPress;    ///< Raw data from static pressure sensor(at the wing)
    int           _lastRawMagPressCnt; ///< Counter of a same samples from the static pressure sensor (at the wing)
    bool          _gpsFailTest;        ///< Flag enables simulation of a GPS failure
    int           _gpsFailTestTOff100; ///< Given system time off of the GPS failure test [us*100]
	bool		  _gpsPredictionEnable; ///< Flag enables the prediction of groundspeed
	float		  _gpsMaxDistance;		///< max distance between previous gps position and current gps position
    float         _gpsLastDistance;
#if EMRG_LOSS_ENGINE == 1
	bool		  _bTestLostEngine;
#endif
    unsigned char _485seq;            ///< Sequence number for the RS485 protocol frame


#if PILOT_TARGET == PT_HARDWARE
    //declare for xADC_PS
    XAdcPs_Config *xAdcPs_ConfigPtr;
    XAdcPs XAdcInst;      /* XADC driver instance */
    bool _isConfigureXAdc;
    int  xadcReadingDivisor;
#endif

    StorageBase* _confMem;            ///< Subsystem configuration storage
    StorageBase* _gaugeConfMem;       ///< Sensors configuration storage
    ParameterNames* _confNames;       ///< Object associates names with the configuration parameters

	/// Test of the line delaying speed
	float delayLine[48];


    PhysicalState(PhysicalState&);					///< Copy constructor is disabled
	PhysicalState& operator=(const PhysicalState&);	///< Assignment operator is disabled

    void useCmdLine (ClassifiedLine &cl);         ///< Method interprets and executes 'cl' commands
    void useSimLine (const ClassifiedLine &cl);   ///< Method interprets a line of data from simulator
    void useNmeaLine (const ClassifiedLine &cl);  ///< Method interprets a line of NMEAs data from GPS subsystem
    void useAuxNmeaLine (const ClassifiedLine &cl);///< Method interprets a line of NMEAs data from additional GPS
    void useGaugeData (void);                     ///< Method uses data from sensors
    void useLaunchTrig (void);                    ///< Handler method of a take-off notification
    void use485Data (void);                       ///< Handler method of a RS485 data ready to read notification
#if USE_CAM == CAM_ENABLE
    void useCameraData(void);
    void initCameraRequest(void);
#endif
#if MAGNETOMETER_TYPE == USE_HMR
    void useHmrData(const ClassifiedLine &cl);						  ///< Handler method of a RS485 data ready for HMR
#endif
    bool xAdcPSInit(void);
    void usexadcpsData(void);
    int xAdcFractionToInt(float FloatNum);
    bool confLoad (void);                         ///< Method loads subsystem configuration
    bool confSave (void);                         ///< Method saves subsystem configuration
    bool gaugeConfLoad (void);                    ///< Method loads gauges configuration
    bool gaugeConfSave (void);                    ///< Method saves gauges configuration
    void nslData (void);                          ///< Notify Send Log Data
	void sendAuxTlm (void);                       ///< Sending/saving auxilliary telemetry method
    bool prepareTlmLine (int format, char* buf, int bufSize);      ///< Method prepares text line with telemetry data
    bool prepareRawGaugeTlmLine (char* buf, int bufSize) const;    ///< Method prepares text line with raw data from sensors
	bool prepareAuxSensorTlmLine (char* buf, int bsize) const;     ///< Preparing the line of auxilliary sensors telemetry data method
    ParameterNames::ERRCODE setParameters (const char* nameValueItems);///< Parameters setter method
    bool getParameter (const char* pName, ClassifiedLine& cl) const;   ///< Method sends specified parameter or all paramters to the specified device
    float getStaticPressureFromSensor (void) const; ///< Method reads static pressure from the specified sensor
    float getStaticPressure (void) const; ///< Method reads static pressure from specified sensor which is automatically switched to another in case of damage
    static float altFromPress (float press);      ///< Method calculates altitude based on the pressure (standard model of an athmosphere)
    static float computeTas (float ias, float staticPressure, float outerAirTemp);
    bool setAglZeroInternal (void);               ///< Method sets current altitude as a referecne AGL (version without the semaphore)
    float processedAltitude (void) const;         ///< Method calculates the final altitude including all amendments
    void setRefBasePressure (void);               ///< Method sets reference pressure of a base station
    void setParamUserAction (unsigned int flags); ///< Method performs action when parameter value has been changed
    bool decodeRtkPart (const char* rtkPartB64);  ///< Method decodes a part of an RTK correction and sends it to the GPS
    void gpsFailNavigation (void);                ///< Method calculates plane navigation data (e.g. possition) after failure of a GPS
    void gpsFailTestCheckTout (void);             ///< Method resets the 'gpsFail' flag after specified timeout
    void init485requests (void);                  ///< Method initializes a sequence of data requests to a RS485 serial device
	void heater485requests (int heaterModeVal);                ///< Method send a sequence of data config heater mode to a RS485 serial device
	void estFuelLevel (void);					  ///< Method estimated fuel level during flight
	void checkGpsData(bool &windUpdated);		  ///< Method to reject the gps data with abnormal jamming
	float distanceLowpassNonlinearFilter (float input, float &previousDistance, float maxDistance);
#if USE_LEAFLETS == 1
	void loaddropon(void);
	void loaddropoff(void);
#endif
#if USE_DGPS == 1
    bool _dgpsGood;
#endif
public:
#if USE_DGPS == 1
	float getNetOffset (void);
#endif


	float airspeedDly[2];				///< For chebyshev filter
	float filterAirspeedDly[2];		///< For chebyshev filter
};

#endif  // PHYSICALSTATE_H
