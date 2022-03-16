/**
* @class FlightController                                                
* @brief Class that calculate control surfaces deflection, camera positioning and other actuator mechanisms.
*
* Subsystem developes data based on data received from the FPlanRealizer. Current controllers config are taken from the FPlanRealizer subsystem.
* Data are shared with the ServMan subsystem (servo manager) by the 'notify' mechanism.
* 2008 Roman Filipowski @ Flytronic
* 
*/

#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

/** \file
* \brief Class calculates actuators deflection, controlls gimball and other servo mechanisms
*                                                                 
* 
*/
class FlightController: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:
    FlightController(void);
    void task(const void* pdata);										///< Operating system task.
    void linkObserver();												///< Linking observed objects.
    bool getOutputData(OutputControls &oc);								///< Return Controllers data.
	bool getFlightRefLowLevelData(FPRealData::FlightReference::FlightReferenceLowLevel &refLowLevel);			///< Return Flight Reference data.
    virtual bool isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm);	///< If subsystem is ready for takeoff.
    virtual bool putLine (ClassifiedLine &cl);							///< Put new line to the commands queue.
    float GetSimId(void) const;											///< Get value of the simulated current.
	bool getThrAlert (void);

protected:
    ///  Destructor protection. Object should not be destroyed.
    virtual ~FlightController(void){};

private:

    ///  Log divisor is used to force logging the telemetry when it was turned of or was set too slower.
    static const int FORCED_LOG_DIVISOR = 2;

    static const char CMD_OK[];
    static const char ERR_UCOMMAND[];
    static const char ERR_CLOAD[];
    static const char ERR_CSAVE[];
    static const char ERR_BADPAR[];
    static const char ERR_CMDEXEC[];
    static const char MSG_NO_CONFIG[];
    static const char ERR_CMD_BUF_FULL[];

    /**
	* Class storing csubsystems configuration data.
    */
    class ConfigData
    {
    public:
        ConfigData (void);          ///< Constructor
        void setDefault (void);     ///< Set default parameters.

        int   commDivisor;			///< Number of subsystem cycles after which subsystem data should be send to communication chanel (0-n; 0 turn off)
        int   logDivisor;			///< Number of subsystem cycles after which subsystem data should be written to the log file (0-n; 0 turn off)
        int   commFormat;			///< Format identyficator of the data being send to the communication chanel.
        int   logFormat;			///< Format identyficator of the data being log.
		float Vref;					///< Reference speed used in SpeedFactor calculations.
        float trimThetaFun;         ///< Trimming function factor of the Theta [rad] (how many radians add in full throtlle)
        float trimThetaFunNoSpd;    ///< Trimming function factor of the Theta [rad] (how many radians add in full throtlle) in case of speed indicator malfunction.
		float alrDeadZone;			///< Insensibility region of the ailerons (PidModifierAlr)
        float zeroPhiThrottle;      ///< Minimum of thrust (0-1) at which Phi angle is reset.
		float flapsVref;			///< Speed at which flaps are switch off.
        float minQCompV;            ///< Minimum speed to calculate Q in turn compensation. [kph]
        float maxQCompPhi;          ///< Maximum roll Phi that could be compensated with elevator. [rad]
        float alrRedMin;            ///< Ailerons reduction factor at Phi=0.
        float alrRed1Phi;           ///< Angle  Phi at which ailerons reduction factor = 1.
		bool  pqrFilter;				///< Activation filter for angular speed flag.
		bool  trackErrFilter;		///< Activation filter for Track Error flag.
        bool  configAutosave;        ///< Flag that cause automatic execution of "save config" after landing.
        float simIdCoeff;           ///< Factor that is used in current consuption simulation.(A parameter from formula: Id = A*throtle + B)
        float simIdZero;            ///< Current consumption at turned off engine. (B parameter from formula: Id = A*throtle + B)
		int   thrZeroForceN;			///< Time [tick count] when "zero" must be forced during engine controller reset.
		float idFullThrottle;		///< Expected current at full steering at the engine (to calculate Theta from I correction not from throttle)
		bool  measThrottleMode;      ///< Modificators theta, phi and elevator working mode - 0 from engine triggering, 1 from current on engine measurement.
        float thrModeSwitchRef;     ///< value of the motor under which modificator works according to control (always regardless of the measThrottleMode)

		float trimThrV1;            ///< Controller Thr_Alt modificator factor.
		float trimThrV2;			///< Controller Thr_Alt modificator factor.
		float trimThrT1;			///< Controller Thr_Alt modificator factor.
		float trimThrT2;			///< Controller Thr_Alt modificator factor.

#if LAND_MODE == SEMI_LAND
		float altStep;
		float netPosStep;
		float altRefMin;
		float netPosRefMax;
		bool  invDirAlr;
		bool  invDirElv;
		float virtualDistNet;
#endif
        float thrAlertTime;         ///< Time [0] from setting engine to max value to the control altitude measurement(default: 5s) set to 0 - turns off mechanism that controls engine
		float chkAlertThr;          ///< Value from which engine malfunction is testing 0..1 (default: 0.99)
        float optAirspeed;          ///< Optimum speed
        float optAltMrgLow;         ///< Altitude margin that turns on flight with optimum speed.
        float optAltMrgHigh;        ///< Altitude margin that turns off flight with optimum speed.
		bool  bOptVg;
        bool  bOptFlightPhase;

		PidParamsSet pp; ///< Controllers parameters
		L1Params l1Conf;
		TECSParams tecsConf;

	private:
        static const int COMM_DIVISOR_DEFAULT = 0;    
        static const int LOG_DIVISOR_DEFAULT  = 0;
		static const int PID_FORMAT_DEFAULT   = TLM_CONTROLLER_SHORT;
    };
   
    /// Work mode (Automatic/Manual)
    enum PicCicMode
    {
        CIC_MODE = 0, ///< Automatic (CIC)
        PIC_MODE = 1  ///< Manual (PIC)
    };
   
    /// Configuration data file name.
    static const char CONF_FILE_NAME[];
    /// Subsystem name.
    static const char SUBS_PREFIX[];
    /// Reference speed [km/h]
    static const float V_REF_DEFAULT;
    /// Speed at which flaps should be switched off [km/h]
    static const float FLAPS_V_REF_DEFAULT;

	/// Maximum amount of parameters that could be set by the commands or flight plan.
    static const int MAX_PARAMETERS = 67;
    ParameterNames* _pars;  ///< Object pointer to the array of names and parameters description.
	
	int _fPRealTag, _pStateTag, _servoTag, 
		_fPRealLaunchTrigTag, _fPRealTkfP2Tag, _sysMonTag, _sysMonEngCtrlTag,
        _extCmdTag,                                 // Tags that are given during the observation object registartion.
#if LAND_MODE == SEMI_LAND
		_servManTriggerElvTag, _servManTriggerAlrTag, _servManTriggerElvAlrTag,
#endif
		_fPRealAttentionTag, _fpRealNoAttentionTag, _fPRealParachuteTag; // Tags about the interesting maneuvers.

    CmdQueue<2> _cmdq;				///< Subsystems input lines (commands) queue. (2 lines)
	PStateData _psd;				///< Copy of the subsystem PState data.
	FPRealData _fprd;				///< Copy of the subsystem FPReal data.
	ServManData _servoData;			///< Copy of the subsystem ServMan data.

	PidStateSet _ss;				///< Controllers state.

    StorageBase* _confMem;			///< Controllers configuration memory.
	Semaphore _vSem;				///< Semaphor controlling acces to the subsystems data (_fcd)
    FParser _parser;				///< Text commands from communication chanel parser.

	ConfigData _conf;				///< Subsystems Dane konfiguracyjne podsystemu

    int _commCounter;				///< Counter for the sending telemetry data.
    int _logCounter;				///< Counter for the saving data to log file.
	int _previousLogDivisor;		///< Pravious log divisor after forced to logging.

	float _previousP;				///< NLLP filter state.
	float _previousQ;				///< NLLP filter state.
	float _previousR;				///< NLLP filter state.

    float _lastTrackError;			///< Track angle error from previous cycle.

	bool  _parachuteChanged;		///< Parachute state changed?
    float _aerodynamic_load_factor; 	// the aerodymamic load factor. This is calculated from the demanded
    										// roll before the roll is clipped, using 1/sqrt(cos(nav_roll))
	float _throttle;				///< variable used by the Theta and Phi controllers modyficators.
	float _measThrottle;			///< Calculated based on current consuption(value idFullTrhottle)
	float _calcThrottle;			///< engine control calculated by the controllers.
		
	Iir TrackErrorFilter;   
    bool _isConfigLoaded;			///< Configuration has been loaded from flash flag.

	int _thrZeroForceCounter;		///< Counter forcing for setting 0 on the engine controller (reset engine)
	bool _forceReadFromRealFlag;	///< Flag set when data from realizer must be read.
	bool _bSendMsgEngineFail;		///<

#if LAND_MODE == SEMI_LAND
	bool _bUpdateAltRef;
	bool _bUpdateNetPosRef;
	float _distFromOriginalNetPos;
#endif
	
	bool _bAutoZerosThrOut;

#if EMRG_LOSS_ENGINE == 1
	bool _bTestThrAlert;
#endif
	/**  @name Definiction and initialization stayic elements.
	* @{
	*/
	INT16U _PicCicMode;      ///< Variable beeing set by the notification from ServoManager.
	bool _thrAlert;          ///< Flag if alert is set.
	float _alertAltitude;    ///< Saved altitude during first alertzapami�tana wysoko�� przy pierwszym alercie
	int _alertTime100;       ///< Saved time of first alert.
	bool _waitingForClimb;   ///< Flag to wait for climbing.
	///@}
    
	// variables used in flight with optimal speed
    bool _optAirspeedFlag;   ///< Flight with optimal speed flag.

    Pid
		// Controllers level 0
		Alr_P,			 	///< Aileron from P (angular speed in X axis)
		Elv_Q,				///< Elevator from Q (angular speed in Y axis)
		Rdr_R,				///< Rudder from R (pr�dko�� k�towa w osi Z)
		Rdr_Yacc,			///< Rudder from Yacc (kulka)
		Thr_Speed,          ///< Throttle from Airspeed
		Thr_Alt,			///< Throttle from Altitude
		Thr_Alt_2State,		///< Throttle from Altitude (2 stane with hysteresis)
		Btfly_Alt_2State,   ///< Butterfly from Altitude (2 stane with hysteresis)
		Flp_Speed,			///< Flaps From Airspeed 
		Abr_GPErr,			///< Airbrakes from Glide Path Error
		FAlr_Alr,			///< Flaps As Aileron from Ailerons
		Btfly_GPath_2State, ///< Butterfly from Glide Path (2 state)
		
		// Controllers level 1
		P_Phi,				///< angular speed in X axis from Phi (roll)
		Q_Theta,			///< angular speed in Y axis from Theta (pitch)
		R_Psi,	   		    ///< angular speed in Z axis from Psi (yaw)
		R_CoordExp,			///< angular speed in Z axis from calculated value of the turn coordination.
		R_Track,			///< angular speed in Z axis from track angle error.
		
		// Controllers level 2
		Theta_Alt,			///< Theta from Altitude
		Theta_Speed,		///< Theta from Speed
		Phi_Track,			///< Phi from Track - roll angle from track angle error.
		Phi_CTrack,			///< Phi from CTrack - roll angle from track angle error in circle.
		Phi_Psi,			///< Phi from Psi - roll angle from Psi error.
		Theta_GPath_2State, ///< Theta from Glide Path (2 stane with hysteresis)
		Theta_VertSpeed,
		VertSpeed_Alt,

		// Controllers level 3
		Track_TrackCorr,    ///< Track from CrossTrack Error 
		Track_Wpt,			///< Track from Waypoint 

		// Controllers level 4
		TrackCorr_Cte;      ///< Track angle correction from deviation from desire track.

	TECS TECS_controller;
	L1_Control Phi_L1;
	Navigation *nav_l1 = &Phi_L1;
	SpdHgtControl *spd_hgt_ctr = &TECS_controller;

    /// PID outputs modificatos
    PidModifierPhi   _modPhi;
    PidModifierTheta _modRefTheta;
    PidModifierAlr   _modAlr;
	PidModifierThrottle _modThrottle;

    //  Lock coping constructor and assignment operator.
    FlightController(FlightController&);
    FlightController& operator=(const FlightController&);

	// Operations
	bool setupControllers();						///< Controllers settings.
	bool computeAll();								///< Compute all controllers settings.
	void ClearControllers();						///< Reset all controllers.

private:
    void useCmdLine(ClassifiedLine &cl);			///< Commands interpretation.
	bool getParameter(const char* pName, ClassifiedLine& cl) const;
	void elevonMixer(const float aileron, const float elevator, float& leftElevon, float& rightElevon);
    ParameterNames::ERRCODE setParameters (const char* nameValueItems); ///< Set many of the parameters.
    bool confSave(void);
    bool confLoad(void);
	void sendLogAndTelemetryData();
    float CalcTrackError(float curTrack, float refTrack);
    float CalcPsiError(float curPsi, float refPsi) const;
	float CalcR(float v, float curPhi, float curTheta) const;
    float CalcGPath (float altitude, float distance, float curTrack, float trackToWpt) const; 
	float NLFilter(float input, float *state) const;

	void useAttentionTrigs(bool flagON);

	int _flightPhase;
	void checkAltOpt (void);

#if LAND_MODE == SEMI_LAND
	void adjustAltRef (float& altRef);
	void adjustNetPosRef (GpsPosition& oldNetPos, float track, GpsPosition& newNetPos);
#endif
};

#endif  // FLIGHTCONTROLLER_H

