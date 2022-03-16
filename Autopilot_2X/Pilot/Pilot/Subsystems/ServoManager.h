#ifndef SERVOMANAGER_H
#define SERVOMANAGER_H

#pragma pack(1)

/// Notification of controll acquisition by pilot/computer
struct EventPicCicChange
{
	INT32U  time;		///< Time in milliseconds
	INT16U	PicnCic;	///< 0 or 1, like in data read from altera
};

struct TlmDataServo
{
	INT32U 			time;   ///< time in milliseconds
	INT16S			servoData[SERVO_COUNT];	///< Telemetry data
};

/// Telemetry of a current vlaues of RC channels
struct TlmDataRCIN
{
	INT32U  time;   ///< Time in milliseconds
	INT16U	rcinControl;
	INT16U	rcinData[8];
	INT16U	rcinPicnCic;
	INT16U   rcArmStatus;
};

/// Telemetry of a current servos PWM values
struct TlmDataServoIN
{
	INT32U  time;   ///< Time in milliseconds
	INT16U  servoControl;
	INT16U  servoData[16];
};

struct RCStatus
{
	INT8U	DCH1;
	INT8U	DCH2;
	INT8U	frameLost;
	INT8U	failsafeActivated;
	INT32U	totalFrameError;
	INT32U	lastGoodFrame;
	INT8U	NoRcInput;
};
#pragma pack()

/** \file
* \brief Declartion of a servomechanism conroller class
*/

/** Class is designed to process input control values to generate apropriate signals for the servo mechanisms and can read current states of the servos.
* ServoManager class is also a component of the system (subsystem) because it publicly inherits from the SubsystemBase class. Class implements the observer pattern
* by inheriting base classes ODTSubject and ODTObserver representing the objects of observation and observer respectively. Thanks to this ServoManager is both the observer
* and object of observation, in particular, it may watch itself. The registration of the observed objects takes place in "linkObserver()" method, and the registered and
* observed system components are flight control (FControl), and the servo manager component itself. Notifications for this system derived from LDisp (SERVO_LINE, CMD_LINE)
* and FlightController (CONTROLS_COMPUTED).
*/
///Class implements conroller of a servomechanisms
class ServoManager: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:

    ServoManager(void);
    void	task(void* pdata);					                       ///< System task handler method.
    void	linkObserver(void);					                       ///< Observed objects linking method.
    bool	getServoData(ServManData &smd);		                       ///< System monitor data getter method.
    virtual bool isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm); ///< Method returns 'true' if system is ready for takeoff, 'false' otherwise. Possible errors are send to 'cl'.
    virtual bool putLine (ClassifiedLine &cl);                         ///< Method puts new line to the command queue.
	float   getFuelConsumption();                                      ///< RF: VT1 Instantaneous fuel consumption getter mehtod.
	void  setRcControlToFG (float& alr, bool bAlr, float& rdr, bool bRdr,
							float& elv, bool bElv, float& thr, bool bThr,
							float& flp, bool bFlp);

#if LAND_MODE == SEMI_LAND
	int getValueTriggerElv (void);
	int getValueTriggerAlr (void);
#endif
	bool    isArm(void);

	/** Names of an output parameters send by FlightPlanRealizer and their indexes form ServoConf.coeffs[] array.
	* Parameters indexes are mapped form FlightControl.
	* \{
	*/
    enum InputNames
    {
        InAilerons = 0,			///< [float	ailerons] control in the x-axis (usually the ailerons)					
        InElevator = 1,			///< [float	elevator] control in the y-axis (usually the elevator)
        InRudder = 2,			///< [float	rudder] control in the z-axis (usually the rudder)
        InThrottle = 3,			///< [float	throttle] throttle control {0;1}
        InFlaps = 4,			///< [float	flaps] flaps control (can be mapped to the the ailerons) {0;1}
        InAirbreakes = 5,		///< [float	airbrakes] control of air brakes {0;1}
        InContainerDrop = 6,	///< [float	containerDrop] control of the container drop {0;1}
        InButterfly = 7,		///< [float	butterfly] control of the "butterfly" mode (the ailerons up and the flaps down) {0;1}
        InFlapsAsAilerons = 8,	///< [float	flapsAsAilerons] control of the aileron powersteering (flaps tilted proportionally to the ailerons) {0;1}
		InParachute = 9,          ///< [float antenna] control of the antenna pantograph {0;1}
        InCustom1 = 10,			///< [float	customOutput[0]] control of additional servo
        InCustom2 = 11,			///< [float	customOutput[1]] control of additional servo
        InCustom3 = 12,			///< [float	customOutput[2]] control of additional servo
        InCustom4 = 13			///< [float	customOutput[3]] control of additional servo
    };
	///\}

protected:
    /// Destructor is disabled - object should never be destroyed.
    virtual ~ServoManager(void){};

private:
	/// Configuration data file name
    static const char CONF_FILE_NAME[];
    /// Subsystem name used as a prefix
    static const char SUBS_PREFIX[];

	/** \name Subsystem messages
	* \{
	*/
	static const char CMD_OK[];
    static const char ERR_UCOMMAND[];
    static const char ERR_CLOAD[];
    static const char ERR_CSAVE[];
    static const char ERR_BADPAR[];
    static const char ERR_CMDEXEC[];
    static const char ERR_SYS[];
    static const char ERR_READONLY[];
    static const char MSG_NO_CONFIG[];
    static const char ERR_CMD_BUF_FULL[];
    static const char ERR_ALREADY_ARM[];
    static const char ERR_ARM[];
    static const char ERR_DISARM[];
	static const char ERR_ARM_NOT_ALLOW[];

    static const INT32U MASK_PWM_VALUE  = 0xFFF;
    static const INT32U MASK_OFL_BIT    = 0x8000;
    static const INT32U MASK_UFL_BIT    = 0x4000;
    static const INT32U MASK_PIC_N_CIC  = 0x1;
    static const INT32U MASK_TRIG_MAN   = 0x8000;

    //-------------------------------------------------------
    // SERVO defines
    //-------------------------------------------------------

    // Address offsets, base address FLYEYE_SERVO_BASE
    static const int SERVO1_a 			= 1*4;
    static const int SERVO2_a 			= 2*4;
    static const int SERVO3_a 			= 3*4;
    static const int SERVO4_a 			= 4*4;
    static const int SERVO5_a 			= 5*4;
    static const int SERVO6_a 			= 6*4;
    static const int SERVO7_a 			= 7*4;
    static const int SERVO8_a 			= 8*4;
    static const int SERVO9_a 			= 9*4;
	static const int SERVO_PARACHUTE_a	= 10 * 4;
    static const int SERVO_ARM_a 		= 11*4;

    //-------------------------------------------------------
    // RC defines
    //-------------------------------------------------------

    // Address offsets, base address FLYEYE_RC_BASE
    static const int RC1_a		= 1*4;
    static const int RC2_a		= 2*4;
    static const int RC3_a		= 3*4;
    static const int RC4_a		= 4*4;
    static const int RC5_a		= 5*4;
    static const int RC6_a		= 6*4;
    static const int RC7_a		= 7*4;
    static const int RC8_a		= 8*4;
    static const int RC9_a		= 9*4;
    static const int RC10_a		= 10*4;
    static const int RCDCH1_a	= 17*4;
    static const int RCDCH2_a	= 18*4;
    static const int RCFrLost_a	= 19*4;
    static const int RCFSActv_a	= 20*4;
    static const int RCFrErr_a	= 21*4;
    static const int RCmode_a	= 22*4;
    static const int RClastGoodFr_a	= 23*4;
    static const int RCNoRCInput_a	= 24*4;


	/** Static constant used to convert the logical values to the PWM values of a timer controling servos.
	* Current value based on the assumption that operating range of a typical servomechanism is in range 1000 - 2000 (microseconds)
	* and neutral position set to 1500. The difference between the lower and upper limits of the range and neutral position is equal to 500.
	* This value is only used to determine the approximate value of the adjustment. The exact conversion is achieved using the appropriate
	* gain for the servo.
	*/
	static const float	SERVO_CONST;

    /// ServoManager subsystem configuration data class
    class ConfigData
    {
    public:
        int commDivisor;	///< Samples count after which data will be send to the communication channel (0-n, 0=no sending)
        int logDivisor;     ///< Samples count after which data will be send to log (0-n, 0=no logging)
        int commDivisorRC;
        int logDivisorRC;

#if LAND_MODE == SEMI_LAND
		int triggerDivisor;
		int triggerZone;
#endif

		/** (RF: VT1) Parametrs used to calculation of fuel consumption.
		* Characteristics of a fuel consumption is determined by the points (T1,F1) and (T2,F2).
		* Calculation is linear with upper limit ('maxThrCons').
		* \{
		*/
		float minFuelCons;				///< Lower fuel limit
		float maxFuelCons;				///< Upper fuel limit
		float fuelConsT1;				///< Point T1 of the characteristic
		float fuelConsT2;				///< Point T2 of the characteristic
		float fuelConsF1;				///< Point F1 of the characteristic
		float fuelConsF2;				///< Point F2 of the characteristic
		bool  bSpinWhenArmed;			///<Allow Spin Engine when armed
		int   armProcessTimeout;		///<[ms]
		int   enginePwmWhenArmed;		///<pwm engine when initial armed
		int   parachutePwmTurnon;		///<pwm parachute when turning on parachute servo
		int   parachutePwmTurnoff;		///<pwm parachute when turning off parachute servo
		///\}

        ConfigData (void);
        void setDefault (void);	///< Method sets default parameters for the subsystem

    private:
		/** Default values which are set when they cannot be read from the mass memory.
		* \{
		*/
        static const int COMM_DIVISOR_DEFAULT = 0;      ///< Telemetry is disabled to not send messages to the console
        static const int LOG_DIVISOR_DEFAULT = 0;
        static const int COMM_DIVISOR_RCIN_DEFAULT = 0;
        static const int LOG_DIVISOR_RCIN_DEFAULT = 0;
		///\}
    }; 
	
	/// ServoManager subsystem configuration data class and servomechanisms confiruration data
	class ServManConfig
	{
	public:
		ConfigData			subsConf;	///< Subsystem configuration
		ServoConf			servo[SERVO_COUNT];		///< An array of all accessible servomechanisms

		void setDefaults (void);
	};

	/// Class provides access to the registers of servomechanisms. Object of this class should never be destroyed.
	class ServoIO
	{
	public:
		ServoRegs	*servoRegs;
		RCinRegs	*rcinRegs;

		ServoIO(void);

	private:
        // Copy constructor is disabled
        ServoIO(ServoIO&);
		// Copy operator is disabled
        ServoIO& operator=(const ServoIO&);
    };

	/** Private variables
	* \{
	*/
	int					_fctrlTag;
    int                 _extCmdTag;
	int					_fPRealParachuteTag;
#if PILOT_TARGET == PT_WIN32
    CmdQueue<8>         _cmdq;                  ///< Queue of a subsystems input command lines (two command lines)
#endif
#if PILOT_TARGET == PT_HARDWARE
    CmdQueue<2>         _cmdq;                  ///< Queue of a subsystems input command lines (two command lines)
#endif
	ServoIO				_servoIO;				///< Object of direct access to the hardware registers
	ServManConfig		_conf;					///< Subsystem configuration
	ParameterNames*		_pars;					///< A pointer to the object that holds an array of names and descriptions of parameters
	Semaphore			_vSem;
	StorageBase*		_confMem;				///< Configuration memory
	FParser				_parser;				///< Communication channel commands parser
	TlmDataServo		_tlmServo;
	TlmDataRCIN			_tlmRCIN;
	RCStatus			_tRcStatus;
	EventPicCicChange	_eventPicnCicChange;	///< Event send to the base station
	ServManData			_servoData;				///< Data for other subsystems
	int					_servoPWM[16];			///< Array of the PWM values for servomechanisms
    int					_commCounter;           ///< Counters
	int					_logCounter;			
    int                 _commCounterRC;
    int                 _logCounterRC;
#if LAND_MODE == SEMI_LAND
	int                 _triggerCounter;
#endif
	INT16U				_prevPICnCIC;			///< Previously read value
	int					_waitingForAckID;
	int					_eventAckID;
    bool                _isConfigLoaded;
    // RF: VT1
	int					_currentThrPWM;         ///< Engine current control value (PWM) form Futaba RC or autopilot
	bool                _throttleRotate;		///< Flag indicates the engine is spinning. There is no tachometer so it is assumed that engine is spinning still since it was started.
#if LAND_MODE == SEMI_LAND
	int                 _valueTriggerElv;
	int                 _valueTriggerAlr;
#endif
	///\}

    float               _fSimAil;
	float               _fSimElv;
	float               _fSimThr;
	float               _fSimRdr;
	INT16U              _leftSwitch;
	INT16U              _rightSwitch;

	float               _fSimRefRoll;
	float               _fSimRefPitch;
	float               _fSimRefVz;
	float               _fSimRefYaw;
	INT32U              _iArmDisArmTime;
	bool                _bPreArmDisArm;
	bool                _bArmSaveTime;
	bool                _bDisarmSaveTime;
	bool                _bArmed;
	bool                _spinWhenArmed;
	bool				_parachuted;


    // Copy constructor is disabled
    ServoManager(ServoManager&);
	// Copy operator is disabled
    ServoManager& operator=(const ServoManager&);

	bool	getParameter(const char* pName, ClassifiedLine& cl) const; ///< Parameter getter method
    ParameterNames::ERRCODE setParameters (const char* nameValueItems);	///< Parameters setter method
    void	useCmdLine(ClassifiedLine &cl);
	bool	saveConf(void);
	bool	loadConf(void);
	void	readRCIN(TlmDataRCIN& t, RCStatus& s) const;								///< Reading data from RC radio registers method
	void	setServos(void);											///< Applying calculated PWM values to the servos method
	void	computeAll(const OutputControls& oc);						///< Calculation of a servo settings method
	void	sendServoTlmData(bool fLog, bool fComm);					///< Sending telemetry method
	void	sendRCINTlmData(bool fLog, bool fComm);						///< Sending telemetry method
	void	fireEventPicnCicChange(const EventPicCicChange& e);
	float   calcServoValue (ServoConf servoConf, INT32U servo, float coeffs_p, float coeffs_n);
	void	parachuteAction(void);
#if LAND_MODE == SEMI_LAND
	int calcValueTriggerElv (int triggerZone);
	int calcValueTriggerAlr (int triggerZone);
#endif
	void    resetParams(void);
	void	armEngine(bool isArm);
};

#endif  // SERVOMANAGER_H


