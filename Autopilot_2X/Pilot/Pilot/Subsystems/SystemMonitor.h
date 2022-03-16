#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

/** \file
* \brief Declartion of a system state monitor class
*/

/** Class is designed to monitor the state of whole system by reading the state of each subsystem and making decisions about their impact on flight.
* Class also supports reading state of the "start" and "stop" switches (buttons) by an interrupt service routine, which emits theirs "notify" function.
* Class performs cyclic reading the state of each subsystem conponents (pooling technique) which provides dedicated diagnostic functionality. The cyclic
* state checking is performed once every few notices 'PSTATE_CHANGED' sent form the physical state (PState) subsystem.
* SystemMonitor class is a kind of a data accessor, because it is responsible for maintaining and providing information about the system state
* and decisions taken to other subsystem components. Storing data by SystemMonitor involves copying that data in the right place in the class. That class
* has, amoung others, two private members of the class "SmonConfigData" and "SmonStateData" representing the configuration data and the status data respectively,
* both stored in nonvolatile memory.
*	Class provides information about whether it is possible to go to 'running' mode (isReadyForRun(bool)), the system is ready to takeoff (isReadyForTakeoff(ClassifiedLine*, bool)),
* or a radio connection is active (isLinkBroken(void) const), battery status (getBattery(float&, float&, float&, float&, int&) const) and registered the GPS position
* and height of the time of last call. Class also controls the hardware 'watchdog' - counter causes an automatic reset when the CPU does not respond on
* cyclic stimulation.
*	SystemMonitor class is also a component of the system (subsystem) because it publicly inherits from the SubsystemBase base class. Class implements the observer pattern
* by inheriting base classes ODTSubject and ODTObserver representing the object of observation and observer respectively. Thanks to this SystemMonitor is both the observer
* and object of observation, in particular, it may watch itself. The registration of the observed objects takes place in "linkObserver()" method, and the registered and
* observed system components are physical state component (PState), flight plan realizer comnponent (FPReal), and the system monitor component itself.
*	The MicroC/OS-II operating system tasks are handled by SystemMonitors "task( const void* )" method, where the following sequence of operations is performed: loading
* subsystem configuration data (confLoad()), initializing the FlightTimeEstimator and processing the system events loop.
*/
/// Class implements system state monitoring
class SystemMonitor: public ODTSubject, public ODTObserver, public SubsystemBase
{
public:
    SystemMonitor (void);

    void task (const void* pdata);              ///< System tasks handler method
	void linkObserver (void);                   ///< Observed objects linking method
    bool getSysMonData (SysMonData &smd);       ///< System monitor data getter
    bool isReadyForRun (bool toLog);            ///< Method checks if system is ready to enter the "Running" mode
    bool isLinkBroken (void) const;             ///< Method checks if radio connection has been broken
    bool lastLinkPos (GpsPosition& pos, float& altitude);                ///< Method sets 'pos' and 'altitude' arguments, lately received GPS position and altitudde (with offset) respectively
    virtual bool isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm);   ///< Method returns 'true' if system is ready for takeoff, 'false' otherwise. Possible errors are send to 'cl'
    virtual bool putLine (ClassifiedLine &cl);                           ///< Method puts new line to the command queue
    void getBattery(float& currentUp, float& voltageUp, float& currentDown, float& voltageDown, int& status) const;
	void getFuelConsumption(float &fuelConsumption);

protected:
    ///Destructor is disabled. Object should never be destroyed
	virtual ~SystemMonitor(void){};

private:
	/** \name User Action Flag
	* Bit flags passed when definig the parameter (insertion) and returned after the particular parameter has been set.
    * Flags identify the function wchich must be executed after setting the parameter.
	* \{
	*/
    static const unsigned int UAF_FTE_INIT = 1; ///< value, returned by set, indicates that calculation parameters require to be refreshed
    static const unsigned int UAF_SET_TTY  = 2; ///< value, returned by set, indicates the TTY associated to 'msg' require to be changed, etc.
	///\}

	/** \name Constant prefixes
	* \{
	*/
    static const char SUBS_PREFIX[];		///< Subsystem name used as a prefix
    static const char CONF_FILE_NAME[];		///< Configuration data file name
	///\}

	/** \name Error messages
	* \{
	*/
    static const char ERR_OK[];
    static const char ERR_UCOMMAND[];
    static const char ERR_CLOAD[];
    static const char ERR_CSAVE[];
    static const char ERR_STATE_MEM_TEST[];
    static const char ERR_CONFIG_MEM_TEST[];
    static const char ERR_BADPAR[];
    static const char ERR_LOG_MEM_TEST[];
    static const char ERR_READONLY[];
    static const char ERR_REBOOT[];
    static const char MSG_NO_CONFIG[];
    static const char MSG_VBAT_NO_TAKEOFF[];
    static const char MSG_AUXBAT_DCHARGE[];
    static const char MSG_NOT_READY[];
    static const char MSG_LANDED_REBOOT[];
    static const char MSG_NO_GPARS[];
    static const char MSG_UPG_IN_PRGRS[];
    static const char ERR_CMD_BUF_FULL[];
	///\}

	/** \name Flash programmer error messages
	* \{
	*/
    static const char ERR_FLASH_NOT_PREP[];
    static const char ERR_FLASH_BAD_LEN[];
    static const char ERR_FLASH_BAD_ADDR[];
    static const char ERR_FLASH_INTERNAL[];
    static const char ERR_FLASH_CRC_SDRAM[];
    static const char ERR_FLASH_CRC[];
    static const char ERR_FLASH_WRITE[];
	///\}

    /// SysteMonitor subsystem configuration data class
    class SmonConfigData
    {
    public:
		int commDivisor;           ///< number of received PSTATE_CHANGED notifications after which data is sent to communication channel. Value scope is 0 to n while 0 disable the sending
        int logDivisor;            ///< as above in the aspect of sending data to log
        int checkDivisor;          ///< as above in the aspect of checking the state of entire system and '*Cond' conditions
        float airborneAglCond;     ///< minimum AGL height needed to determine whether system is airborn, if so SystemNowOnGround is set to "false" [meters]
        float airborneIasCond;     ///< as above in the aspect of speed [kph]
        float maxUpperIBat;        ///< Upper battery maximum discharge current [A]. Value less then 0 means that accu is discharged
        float minLowerVBat;        ///< Main (lower) battery minimum voltage
        float linkBrokenTimeout;   ///< Time of lately received ping message [s]. This allows to determine if the connection is broken
        float lastLinkPosOffset;   ///< Offset distance of the last connection point along one section of the route return
        float lastLinkAltOffset;   ///< Increasing the height of the point of the last connection
        float fteMaxCharge;        ///< FlightTimeEstimator (FTE): Maximum battery charge [Ah]
        float fteDfDischargeRate;  ///< FTE: The default discharge rate in straight flight [mAh/s]
        float fteRetAirspeed;      ///< FTE: Returning speed (IAS) [kph]
        float fteRetSink;          ///< FTE: rate of descent at the time of return [m/s]
        float fteEndAgl;           ///< FTE: Assumpted AGL height at the entrance to the descent zone [m]
        bool  retOnLowEnergy;      ///< Flag signaling the return of the low-energy (fte returns 0 of the reserved time)

        int   scriptNo;            ///< Installation script identifier. (no influence, just storing)
        bool  upgrInProgress;      ///< Flag indicates that systems upgrade is in progress. Take-off is disabled for this time
        float engCtrlMinI;         ///< Mininum current of a main battery over which the battery is considered to be efficient [A]
        int   engCtrlNChk;         ///< Number of current samples checked during inspections of motor controller
        float gpsBadTimeout;       ///< Time [s] after which the GPS malfunction will be detected
#if USE_DGPS == 1
		float dgpsBadTimeout;      ///< Time [s] after which the DGPS malfunction will be detected
#endif
        float pingIgnoreTimeout;   ///< Time difference value between received ping command response and current system time [s] above which ping commands are ignored
		
		bool  testLostLink;

#if EMRG_LOSS_ENGINE == 1
		float f32FocusLandAltitude;
		int   emrgRpm;
		float lossEngineTimeout;
		bool  useRpmforLostEngine;
#endif
        bool bTestHIL;
		float fWorkingTime;			///< Expectation working time of system
		float fTimeMaxbyBatt;       ///< Maximum expection working time estimated by battery
		float fFuelConsumption;		///< Average Fuel consumption by engine
		float fTimeUpdateTimeLeft;  ///< time minimum to allow to update new timeleft

        SmonConfigData (void);  // Constructor
        //  Class uses compiler-generated copy constructor (comment disables the Parasofts warning)
        void setDefault (void); ///< Method sets the default parameters
    };


    /// Class represents the state of the subsystem which is stored in non-volatile memory
    class SmonStateData
    {
    public:
        int          lastLinkTime100;     ///< System time of a lately executed ping command. [time * 100us]
        GpsPosition  tmpLastLinkPos;      ///< System time of lately received GPS position
        GpsPosition  lastLinkPos;         ///< System time of lately latched the GPS position with offset
        float        lastLinkAltitude;    ///< System time of lately latched altitude
        bool         linkBroken;          ///< Flag indicates the connection has been broken
        bool         landed;              //  
        float        qd;                  ///< Load obtained from the lower battery from the time the system has been started [A*s]
        FlightTimeEstimator fte;          ///< It is an object that estimates the remaining flight time
        int          lastGpsGoodTime100;  ///< The system time of the last valid GPS reading
        bool         gpsBad;              ///< Flag indicates that the GPS is treated as malfunctioned. Flag is set after certain time of disability
#if USE_DGPS == 1
		int          lastDGpsGoodTime100; ///< The system time of the last valid DGPS reading
        bool         dgpsBad;             ///< Flag indicates that the DGPS is treated as malfunctioned. Flag is set after certain time of disability
#endif

        //  Class uses compiler-generated copy constructor (comment disables the Parasofts warning)
		//  Constructor
        SmonStateData() :
            lastLinkTime100(0),
            lastLinkAltitude(150.0f),
            linkBroken (false),
            landed (false),
            qd(0.0f),
            lastGpsGoodTime100(0),
            gpsBad(false)
#if USE_DGPS == 1
			, lastDGpsGoodTime100(0)
            , dgpsBad(false)
#endif
        {};
    };


	/** \name Tags received when registering the object observed
	* \{
	*/
    int _pStateTag,
        _fPRealLndTag,
        _fPRealTkfTag,
        _extCmdTag;
	///\}

	/** \name Counters
	* \{
	*/
    int  _commCounter,
         _logCounter,
#if EMRG_LOSS_ENGINE == 1
		 _lossEngineCounter,
#endif
         _checkCounter;
	///\}

    CmdQueue<4>     _cmdq;              ///< Subsystems input command line queue (two lines)
    SysMonData      _smData;            ///< Subsystems data
    SmonConfigData  _conf;              ///< Subsystems configuration data
    SmonStateData   _state;             ///< Subsystem state data (to the non-volatile memory)
    PStateData      _psd;               ///< Data read from the PState subsustem
    PStateHealth    _psh;               ///< Data read from the PState subsystem in the aspect of error status
    OutputControls  _oc;                ///< Data read from the Control subsystem
    AuxSensorData   _auxSensorData;     ///< Auxilary sensors data
    bool            _pStateInitialized; ///< Flag determining the values _psd and _psh are valid (at least has been read once)
    float           _vg, _ig;           ///< Voltage and current of the buffer battery
    float           _vd, _id;           ///< Voltage and current of the main battery
	int             _rs485Status;       ///< RS485 status read together with the currents and voltages
    int             _idCounter;         ///< Current samples counter for the motor controller control
    Semaphore       _vsem;              ///< Semaphore controlling access to the variables
    StorageBase*    _confMem;           ///< Subsystem configuration memory
    StorageBase*    _configMemTest;     ///< Object to test the configuration memory (flash)
    FParser         _parser;
    ParameterNames* _pars;              ///< Pointer to the object that holds an array of names and descriptions of parametes
	FlashProgrammer _flashProgrammer;   ///< Flash programmer
    bool            _isConfigLoaded;    ///< Flag indicates if configuration had been loaded from flash
    bool            _mainVoltageDrop;   ///< Flag indicates voltage drop (useful in detection of incorrect contact)
	bool			_fteIsCalculated;	///< Flag indicates if flight times had been calculated
	float			_timeMax;			///< Maximum estimated working time of system
	bool			_bTmaxEstimated;	///< Flag indicates Tmax is calculated or not
	int				_startTime100;		///< Time when the system starts operation
	bool			_bUpdateTimeLeft;   ///< Flag indicates to allow update Time Left
	int				_bufferTime100;     ///< Time when fuel go down under buffer level

    SystemMonitor(SystemMonitor&);					///< Copy contructor is disabled
    SystemMonitor& operator=(const SystemMonitor&);	///< Assignment operator is disabled

    void useCmdLine (ClassifiedLine &cl);                               ///< Method interprets and executes 'cl' command
    void usePstate (void);                                              ///< PSTATE_CHANGED notification handler
    void useFPRealLnd (void);                                           ///< FPR_LANDED notifications handler
    void useFPRealTkf (void);                                           ///< FPR_TAKEOFF_CPL notifications handler
    ParameterNames::ERRCODE setParameters (const char* nameValueItems); ///< Parameters setter method
    bool getParameter (const char* pName, ClassifiedLine& cl) const;    ///< The sending method of one or all of the parameters
    bool confLoad (void);                                               ///< Method loads the subsystem configuration
    bool confSave (void);                                               ///< Method saves the subsystem configuration
    bool gParSave (void);                                               ///< Saving global parameters method
    bool serialSave (const char* pValue);                               ///< Serial number saving method
    bool resetCntSave(void);
    void sendTlm (void);                                                ///< Sending/saving telemetry method
    bool prepareTlmLine (char* buf, int bsize) const;                   ///< Preparing the line of telemetry data method
    bool setPSErrorFlag (const PStateHealth& psh, float iUb, float vLb);///< Setting PState error flags method
    bool initErrorFlag (void);                                          ///< Initialization of error flags before setting them
    void checkSystemHealth (void);                                      ///< Checking the entire system state and setting error flags
    void checkSystemSlow (void);                                        ///< Checking the entire system state not performed in each cycle
    bool checkLink (void);                                              ///< Checking of rupture and restore the connection
    void checkLowEnergy (void);                                         ///< Checking of low battery
    void checkGps (void);                                               ///< Checking the correctness of GPS
    void checkDisarmEngine (void);                                              ///< Check if we can disarm the system
#if EMRG_LOSS_ENGINE == 1
	void checkEngine (void);
#endif
    void checkAllStacks (ClassifiedLine* pcl, bool toLog) const;        ///< Sending diagnostic of a processe's stacks
    void checkStack (ClassifiedLine* pcl, bool toLog, INT8U taskId,
                     const char* taskName) const;                       ///< Auxilary functionality for 'checkAllStacks' method
    bool ping (bool checkTimeFlag, int lastResponseTime);               ///< Ping command handler
    void monitorLink (const ClassifiedLine &cl);                        ///< Monitoring all transmissions method
    bool _isReadyForRun (ClassifiedLine* pcl, bool fLogComm);           ///< Method checks if system is ready to enter the "Running" mode (on the ground)
    void initFTE (void);                                                ///< Initializing the FTE (Flight Time Estimator) method
    void checkEngineController (void);                                  ///< Method checks whether the motor controller is not suspended
#if EMRG_LOSS_ENGINE == 1
	bool _bLossEngine;
	bool _bFocusLand;
	bool _bCheckLostEngine;
	bool _bLostEngineAlert;
#endif
public:
#if EMRG_LOSS_ENGINE == 1
	bool getLossEngine (void);
#endif
    bool getTestHIL (void);
	void calculateMaxWorkingTime(float &Tmax);
};

#endif  // SYSTEMMONITOR_H
