#ifndef FLIGHTPLANREALIZER_H
#define FLIGHTPLANREALIZER_H

/**
 * @class  FlightPlanRealizer
 *
 *  The FlightPlanRealizer class was designed to realize each flight plan elements and flight commands. To ensure this, class has functions to interpret passed flight plan lines/instructions. 
 *  Each of interpreted flight command are splitted into elemental flight phases. For every phase are defined parameters and end conditions. 
 *  Flight phase is a part of the flight that could be realized by the set of properly configured controllers. (connections, referece values).
 *  The FlightPlanRealizer class are able to perform "manual turn" mode - manual turning by the changing desired value of the appropriate controller.
 *  Class are storing state variables (defining flight phases; desired altitude, speed, etc.). These parameters are initialized as default from the mass memory but can be changed by the flight plan instruction or external commands.
 *   @brief  A flight plan realizer class.
 */
class FlightPlanRealizer: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:
  FlightPlanRealizer(void);
  void task(const void* pdata);					///< Operating system task.
  void linkObserver(void);						///< Linking observed objects.
  bool getBaseData (BaseStationData& based);	///< Returning Base Station data (position, elevation).
  bool getFPRealData (FPRealData& fprd);		///< Returning set of variables of the FlightPlanRealizer subsystem.
  bool isBasePositionSet (void) const;			///< If the position of the base station is set.
  bool isNotBreakable (void) const;				///< Can be current isntruction stopped.

  void clearNetLandFlag (void);					///< Clear all net landing flag (landing net)
  void clearLandParachuteFlag(void);			///< Clear all landing parachute flag (landing parachute)
  void clearTkfFlag(void);						///< Clear all takeoff flag (landing parachute)
  bool isInCGMode (void) const;					///< If the Camera Guide mode is enabled.
  bool isAllowEngineSignal (void) const;        ///< If allow engine failure signal.
  bool isUseThetaModifier (void) const;         ///< If ThetaModifier is allowed to use.
  void enableThetaModifier (void);              ///< Allow to use theta modifier
  void disableThetaModifier (void);             ///< Disable theta modifier
  bool isInMTMode (void) const;					///< If the Manual Turn mode is enabled.
  bool turnOffEngine (void);					///< Check if command to stop engine being issued
  bool inLandMode (void) const;					///< Check if in land mode
  bool inAfterPreApproachPhase (void) const;    ///< Check if in approach, descent and land mode
  bool inLandPhase (void) const;				///< Check if in land mode
  bool inPostLandPhase (void) const;			///< Check if in post land mode
  bool inPreLandPhase(void) const;              ///< Check if in  preland
  bool inParachutePhase(void) const;			///< Check if in  parachute phase
  bool inTkfCatapultPhase(void) const;			///< Check if in  takeoff catapult phase
  bool getTuningControllers (int &tuningLat, int &tuningLon, int &tuningSpd); ///< Get value of tuning controllers.
  bool getTuneEnable (void);
  bool getUseCorrTrack (void);
  
  virtual bool isReadyForTakeoff(ClassifiedLine* cl, bool fLogComm);	///< Checking if the subsystem is ready for takeoff. Erros are send to the ClassifiedLine class.
  virtual bool putLine (ClassifiedLine &cl);							///< Put new line to the commands queue.

protected:
  ///  destructor protection. Object should not be destroyed.
  virtual ~FlightPlanRealizer(void){};

private:
  ///Copy constructor and assigment operator protection.
  FlightPlanRealizer(FlightPlanRealizer&);
  FlightPlanRealizer& operator=(const FlightPlanRealizer&);

  /** @name Error messages
  * @{
  */
  static const char ERR_OK[];
  static const char ERR_UCOMMAND[];
  static const char ERR_CLOAD[];
  static const char ERR_CSAVE[];
  static const char ERR_BADPAR[];
  static const char ERR_CMDEXEC[];
  static const char ERR_SYS[];
  static const char ERR_NO_CONFIG[];
  static const char ERR_FLAND[];
  static const char ERR_CMD_BUF_FULL[];
  static const char SUBS_PREFIX[];
  static const char ERR_LAND_UNPREPARED[];

  static const char CONF_FILE_NAME[]; ///< Name of the file with configuration data.

  /**  @name ID of functions supporting phases of flight. 
  * @{
  */
  static const int RESERVED			=  0;
  static const int FLY_PHASE_2		=  3;
  
  // Auto takeoff using catapult    
  static const int AUTO_TAKEOFF_CATAPULT_P1 = 6;
  static const int AUTO_TAKEOFF_CATAPULT_P2 = 7;
  static const int AUTO_TAKEOFF_CATAPULT_P3 = 8;
  static const int AUTO_BEGIN_MISSION       = 9;

  static const int PRELANDING_P2	= 10;
  static const int APPROACH			= 11;
  static const int DESCENT			= 12;
  static const int NET_LAND			= 13;
  static const int POST_LAND	    = 14;
  static const int ABORT_LAND		= 15;
  static const int GOAROUND_NET		= 16;
  static const int MT_OFF			= 17;
  static const int STALL_ON			= 18;
  static const int STALL_OFF		= 19;
  static const int CRC_PHASE_2		= 20;
  static const int FLND_PHASE_2		= 34;
  static const int FLND_PHASE_3		= 35;
  static const int GOTO_PHASE_2		= 36;
  static const int ESC_PHASE_1		= 38;
  static const int ESC_PHASE_2		= 39;
  static const int LOW_GSPD_ON		= 40;
  static const int LOW_GSPD_OFF		= 41;
  ///@}

  // Auto parachute landing
  static const int CRC_BASE_PARA_PHASE_2	= 43;
  static const int CRC_PARA_PHASE_1			= 44;
  static const int CRC_PARA_PHASE_2			= 45;
  static const int CRC_PARA_PHASE_3			= 46;

  //Camguide
  static const int CRC_CAMGUIDE			= 47;
  static const int CRC_CRC_CAMGUIDE_PHASE_2 = 48;
  static const int SEARCH = 49;
  static const int MAX_PHASE_FUN	= 50;  ///< The maximum number of the functions controling flight phases.

  /** @name User Action Flag.  
  * Bit flags passing during defining parameter (insert) and returning after parameter being set. 
  * They are used for function (function - executed after parameter being set) identyfication.   
  * @{  
  */
  static const unsigned int UAF_REDO_CIRCLE		=  2; ///< If set returned this value and Circle instruction are being currently executed, Circle instruction must be executed again. 
  static const unsigned int UAF_SET_DFLT_ALT	=  4; ///< Set ref.Altitude as default values (alg.dfAltitude).
  static const unsigned int UAF_SET_DFLT_AIRSPD =  8; ///< Set ref.Airspeed as default values (alg.dfAirspeed).
  static const unsigned int UAF_SET_CRC_AIRSPD  = 16; ///< Set ref.Airspeed in Circle as default values (alg.crcAirspeed).
  ///@}
  
  static const int CG_DIVISOR = 2;  ///< Divisor defining at which cycle loop, data for camera guide mode must be processed.

  typedef void (FlightPlanRealizer::*phaseFunction)(void);  ///<  Pointer to the function operating next flight phase.

  ///  Class storing takeoff point parameters.
  class OriginProperties
  {
  public:
    OriginProperties()
      :   takeoffTrack(0.0f)
    {};

    // Copy constructor is compiler generated

    GpsPosition position;
    float takeoffTrack;
  };
  
   ///  Class storing landing point parameters.
  class NetLandingProperties
  {
  public:
	  class NetPropertises
	  {
	  public:
		  GpsPosition position;
		  float width;
		  float height;
		  float offSet;
		  float internalVector;
	  
		  NetPropertises () :
			position (), width (0.0f), height (0.0f), offSet (0.0f), internalVector (0.0f) {}
	  };

    NetLandingProperties()
      :   net (), circleLeft(false), circleRadius (0.0f), distToCircle (0.0f), trackToFollow (0.0f), landPrepared(false)
    {};

    // Copy constructor is compiler generated

    // Position of the recovery net
	NetPropertises net;

    // Position of a virtual waypoint
    GpsPosition tempPosition;
    
    // Position of the circle to decrease altitude
    GpsPosition circleLowerAltitude;

    // Direction to circle. True - left
    bool    circleLeft;   

    // Radius of turn
    float circleRadius;

    // Distance to circle
    float distToCircle;
    
    // Track to follow
    float trackToFollow;

    // Flag allow to change phase
    bool landPrepared;
  };
  
  ///  Class storing base station parameters.
  class BaseStationProperties
  {
  public:
    BaseStationProperties()
      :   elvOffset(0.0f), prevTrackSysTime(0), track(0.0f), speed(0.0f), prevTrack(0.0f)
    {};

  // Copy constructor is compiler generated

  /** @name Basic elements.
  * @{  
  */
  GpsPosition position;    ///< Current gps position of the base station
  float elvOffset;    ///< Altitude offset relative to the takeoff level.
  ///@}

  /** @name Calculated elements.
  * @{  
  */
  GpsPosition prevTrackPosition;	///< Last base station position needed to calculate base track.
  int prevTrackSysTime;				///< System time from the system start to the last time of base track calculations.[*100us]
  float track;						///< The angle of the base station movement track. (0-360 degrees)
  float speed;						///< Movement speed of the base station (kph)
  float prevTrack;					///< The previously calculated track.
  ///@}
  };

///  Class storing parachuting point parameters.
  class ParachutingProperties
  {
  public:
	  ParachutingProperties()
		  : circleLeft(0), circleRadius(0.0f)
	  {};

	  // Copy constructor is compiler generated

	  /** @name Basic elements.
	  * @{
	  */
	  GpsPosition positionParachuting;    ///< Current gps position of the position parachuting.
	  GpsPosition positionCircleCenter;   ///< Current gps position of the circle center of the circle fly before parachuting.
	  bool circleLeft;					  ///<
	  float circleRadius;				  ///< The radius of the circle fly before parachuting [met].
	  float circleBaseTime;				  ///< The time of the circle base process before landing [s].
	  float altitude;					  ///< The altitude of the parachuting point [met].
	  float airspeed;					  ///< The airspeed before parachuting [km/h].
	  WindData wind;					  ///< The wind data facilitate to calculate the drift distance.
  };

  ///  Class storing camguide point parameters.
  class CamguideProperties
  {
  public:
	  CamguideProperties()
		  : circleCamguideTime(0.0f)
	  {};

	  // Copy constructor is compiler generated

	  /** @name Basic elements.
	  * @{
	  */
	  GpsPosition positionCamguide;			///< Current gps position of the position camguide.
	  float circleCamguideTime;				///< Timeout [s].
  };

  class FlightConstraintPropertises
  {
  public:
	  float minVal;
	  float maxVal;

	  FlightConstraintPropertises()
	  :		minVal(0.0f), maxVal(0.0f)
	  {};

	  virtual ~FlightConstraintPropertises() {};
  };
  /// Class storing subsystems configuration parameters.
  class FlightConstraint
  {
	public:
      FlightConstraintPropertises Phi;
	  FlightConstraintPropertises P;
	  FlightConstraintPropertises Alr;
	  bool    rudderYacc;		///< true: algorith using acceleration in Y axis, false: algorithm using desired Phi value and airspeed.
	  FlightConstraintPropertises Rdr;
	  FlightConstraintPropertises R;
	  FlightConstraintPropertises ThetaSpd;
	  FlightConstraintPropertises ThetaAlt;
	  FlightConstraintPropertises Q;
	  FlightConstraintPropertises Elv;
	  FlightConstraintPropertises Thr;
	  float   mrgLowThr;		///< marginLow value  of the Thr_Alt_2State controller.
	  float   mrgHighThr;		///< marginHigh value  of the Thr_Alt_2State controller.
	  float   minBtfly;			///< minimum value  of the Btfly_Alt_2State controller.
	  float   maxBtfly;			///< maximum value  of the Btfly_Alt_2State controller.
	  float   mrgLowBtfly;		///< marginLow value  of the Btfly_Alt_2State controller.
	  float   mrgHighBtfly;		///< marginHigh value  of the Btfly_Alt_2State controller
	  FlightConstraintPropertises Flp;
	  FlightConstraintPropertises VertSpd;
	  FlightConstraintPropertises TrackCorr;
	  bool    bChanged;
	  bool    bChangedMode;

	  FlightConstraint() {};
	  virtual void SetDefault() = 0;

	  virtual ~FlightConstraint() {};
  };

  class FlyConstraint: public FlightConstraint
  {
  public:
	float   finishError; ///< Enought distance from the point that could be achieved.
   
	FlyConstraint ();
	virtual void SetDefault();
  };

  class CrcConstraint: public FlightConstraint
  {
  public:
	float   radius;		    ///< Default circle radius.
	float   radiusErr;      ///< Radius Error compasation
	bool    left;			///< Direction fo circle. True - left
	float   airspeed;		///< Airspeed in circle.
	float   baseTime;		///< Time of circle in instruction "circle base".
    float   normTime;
	float   baseAltitude;	///< Altitude relative to the place of the takeoff in the "fly base".

	CrcConstraint ();
	virtual void SetDefault ();
  };

  class SemiLandConstraint: public FlightConstraint
  {
  public:
	  float airspeed;
	  float approachAlt;
	  float approachDeltaAlt;
	  float trackRange;
	  bool  bUseFlap;
	  float descentAlt;
	  float descentDeltaAlt;
	  float accX;
	  float abortAlt;
	  float abortRadius;	  
	  
	  SemiLandConstraint ();
	  virtual void SetDefault ();
  };

  class ConfigData
  {
  public:
    // Copy constructor is compiler generated
     
  /** @name Default reference values.
  * @{  
  */
    float  dfAirspeed;			///< Default reference value of the airspeed [kph]
    float  dfAltitude;			///< Default reference value of the  altitude [m AGL]
	float  dfAltitudeDelta;
	float  linkBrokenAlt;
    int    dfCircleMode;		///< Circle algorithm variant (0-old, 1-new)
    float  dfCircleModePar;		///< Parameter of the selected circle algorithm. (distance from circumference [m])
	float  dfAutoGndWindCoeff;	///< Ground Wind coefficient

  ///@}
    
  /** @name Default onHold mode reference values.
  * @{ 
  */ 
	float   holdTime;			///< Max time in OnHold mode [s]
	float   holdRadius;			///< Radius of the circle [m]
	bool    holdLeft;			///< True: circle in left
  ///@}

  /** @name Manual Turn mode parameters.
  * @{  
  */
	float   manTimeout;			///< Time after which Manual turn stops. Czas po kt�rym nast�puje wyj�cie z trybu
	float   manPhiCoeff;		///< Value mapping factor "turn" to the angle Phi
  ///@}

	/** @name Return Home Lost Link mode parameters.
  * @{  
  */
	float   linkBrokenMaxAlt;			///< Time after which Manual turn stops. Czas po kt�rym nast�puje wyj�cie z trybu
	float   linkBrokenAltZone;			///< Time after which Manual turn stops. Czas po kt�rym nast�puje wyj�cie z trybu
	float   linkBrokenRadius;		///< Value mapping factor "turn" to the angle Phi
  ///@}
  
  /** @name Camera Guide mode parameters.
  * @{ 
  */ 
	float   cgPhiCoeff;			///< Value mapping factor. Angle "Pan" on the angle Phi
	float   cgPanOffset;		///< "Pan" offset angle (Flight with the rotated camera ) [degrees]
	bool    cgUseCamTlm;		///< Flag of the input tlemetry usage with the camera status.
	
	FlyConstraint      flyConstraint;
	CrcConstraint      crcConstraint;
	SemiLandConstraint semiLandConstraint;

  /** @name Force Land mode instruction parameters.
  * @{ 
  */
	float   fLndThrOffTime;		///< Time of enging work before the drop of container. [s]
	float   fLndTheta;			///< Constant pitch value [rad]
	float   fLndMaxAbsPhi;		///< maximum value of Phi_Psi controller.
	float   fLndBtfly;			///< Constant value of the butterfly during landing.
	float   fLndLandedAccX;		///< Minimum acceleration in X axis to recognise that landing had finished.
  ///@}
     
  /** @name Emergency Land mode instruction parameters. (automatic emergency landing after altitude drop)
  * @{ 
  */
    bool    emrgLandEnable;		///< Flag to enable emergency landing procedure after altitude drop.
    float   emrgLandAgl;		///< Altitude (AGL) at which emergency landing is turning on. [m]
    float   emrgLandMrg;		///< Altitude margin relative to the emrgLandAgl at which altitude control is turning on.
#if EMRG_LOSS_ENGINE == 1
	float   emrgMinPhiDeg;
	float   emrgMaxPhiDeg;
	float   emrgMinThetaDeg;
	float   emrgMaxThetaDeg;
	float   emrgAirspeedRef;
	float   emrgAltitude;
	float   emrgRadius;
	float   emrgThetaDeg;
#endif
  ///@}


  /** @name Stall Recovery mode instruction parameters. 
  * @{ 
  */
	bool    stallRecEnable;		///< Flag to enable stall recovery procedure.
	float   stallRecElevator;   ///< Position of the elevator during stall recovery procedure.
	float   stallRecAirspeedOn; ///< Necessary condition (airspeed) at which stall recovery procedure starts.
	float   stallRecPOn;		///< Optional condition (angular velocity) at which stall recovery procedure starts.
	float   stallRecPhiOn;		///< Optional condition (roll Phi) at which stall recovery procedure starts.
	float   stallRecTimeOff;	///< Maximum time to stop the procedure.
	float   stallRecAirspeedOff;///< Enough airspeed to terminate the stall recovery procedure.
  ///@}
    
    /** @name Auto Takeoff Catapult mode instruction parameters. 
    * @{ 
    */
    bool    autoTkfCatapultEnable;        ///< Enable using auto takeoff from catapult.
    float   autoTkfCatapultPitch;         ///< Pitch to hold [deg].
    float   autoTkfCatapultTimeout;       ///< Timeout to turn off update AHRS from accelerometers [sec].
    float   autoTkfCatapultThetaMaxErr;   ///< Max error for theta to turn off AHRS udate [deg]     
	float   autoTkfCatapultClimbAlt;      ///< Altitude to proceed to phase 4. [m]
    float   autoTkfCatapultAirspeedRange; ///< Delta airspeed range [km/h]
    ///@}

	/** @name Auto Parachute landing mode instruction parameters.
	* @{
	*/
	float    autoParaRadius;					///< The radius of the circle parachute approaching phase.
	float    autoParaAltitude;				///< Altitude of the parachuting phase. [met]
	float    autoParaAirspeed;				///< Airspeed of the parachuting phase. [km/h]
	float    autoParaStableVelZ;				///< Z stable velocity of parachute. [m/s]  
	float    autoParaDelTrack;
	float	 autoParaBaseTime;
	float	 autoParaApproachTime;
	///@}

  /** @name Pressure sensors error - procedures parameters. 
  * @{ 
  */    
	float   pressFaultTheta;	///< Theta value during the pressure sensore malfunction [rad]
    float   pressFaultIas;		///< Ias (indicated airspeed)  during the pressure sensore malfunction (for the auxiliary functions) [kph]
  ///@}

    //  Parametry zwi�zane z prze��czaniem Track-Psi przy zmniejszeniu pr�dko�ci
    float   gSpeedMinValue;		///< Pr�dko�� poni�ej kt�rej jest prze��czany regulator Phi_xTrack na Phi_Psi
  
	float	timeIntervalAspeed;

  /** @name Other parameters. 
  * @{ 
  */
    bool  configAutosave;		///< Flag to automatic turn on "save config" after landing.
    int   mainDivisor;			///< Number of notifications from the PS after which perform the cycle calculation of the subsystem (1-n)
    int   commDivisor;			///< Number of subsystem cycles after which subsystem data should be send to comunication chanel (0-n; 0 turn off)
    int   logDivisor;			///< Number of subsystem cycles after which subsystem data should be written to the log file (0-n; 0 turn off)
	bool  bUseCorrTrack;
  ///@}

    ConfigData (void);      ///< Constructor
    void setDefault (void);   ///< Setting default values.
  };

  /// Class storing subsystem state data in volatile memory. (in case of failure)
  class RamStorageData
  {
  public:
		int   tuningLat;            ///< Methods indicates the stages of Tuning Latitude Controller. (Elevator)
		int   preTuningLat;
		int   tuningLon;            ///< Methods indicates the stages of Tuning Longitude Controller.(Aileron )
		int   preTuningLon;
		int   tuningSpd;            ///< Methods indicates the stages of Tuning Speed Controller.    (Throttle)
		int   preTuningSpd;
		bool  bTuneEnable;
		RamStorageData (): bTuneEnable (false)
		{
			reset ();
		};

		void reset (void)
		{
			tuningLat = preTuningLat = LAT_TUNING_UNCHANGED;
			tuningLon = preTuningLon = LON_TUNING_UNCHANGED;
			tuningSpd = preTuningSpd = SPD_TUNING_UNCHANGED;
		}
  };

  /// Class storing subsystem state data in non-volatile memory. (in case of failure)
  class StateData
  {
  public:
    // Copy constructor is compiler generated
	enum NLAND_PHASE_FLAG
	{
		UNKNOWN,
		PRE_LAND,
		PRE_APPROACH,
		APPROACH,
		DESCENT,
		LAND,
		POST_LAND,
		ABORT_LAND
	};

	enum LAND_PHASE_FLAG
	{
		LND_UNKNOWN,
		LND_PRELAND,
		LND_ESTIMATE_HEADING,
		LND_PARACHUTING,
		LND_PARACHUTED
	};

  enum CAMGUIDE_PHASE_FLAG
	{
		HEADING_ALIGNING,
		CAMERA_ALIGNING,
	};

	enum TKF_PHASE_FLAG
	{
		// Takeoff
		TKF_P1,
		TKF_P2,
		TKF_P3,
		TKF_UNKNOWN
	};
  
    OriginProperties		    origin;				        ///< Takeoff position properties.
    BaseStationProperties	  base;				          ///< Base station properties.
	  ParachutingProperties   para;				          ///< Parachuting point properties.
    NetLandingProperties	  nLand;				        ///< Landing properties.
	  CamguideProperties		  camguide;		          ///< Camguide properties.
	  BaseStationData			    based;				        ///< Base station position and elevation
    FPRealCondition			    exitCondition;		    ///< Exit condition from the flight phase.
    FPRealCondition			    exitConditionMT;	    ///< Exit condition from the "manual turn" mode.
    FPRealTrigger			      launchTrigger;		    ///< Asynchronous condition (trigger) launching the plane.
    FPRealData				      fprd;				          ///< Subsystems data.
    int						          gotoLineId;			      ///< Number of line to go to in the flight plan. (Phase 2 isnstruction goto)
    int						          lastGotoTimeStamp;	  ///< Duration of the last go to instruction (wait could be added before goto)
    bool					          doNotBreak;			      ///< Do not break current phase instruction.
    bool					          doNotBreakByRetMode;  ///< Do not break current phase instruction in the "return" mode.
    bool					          doNotManualTurn;	    ///< Flag to forbid "manual turn" mode.
    bool					          manualTurnMode;		    ///< Flag to turn on "manual turn" mode.
    bool					          doNotCameraGuide;	    ///< Flag to forbid "camera guide" mode.

    NLAND_PHASE_FLAG nLandPhaseFlag;
    TKF_PHASE_FLAG tkfPhaseFlag;
    LAND_PHASE_FLAG landParachutePhaseFlag;
    CAMGUIDE_PHASE_FLAG camGuidePhaseFlag;

    bool cameraGuideMode;      ///< Flag to turn on "camera guide" mode.
    bool bAllowEngineMsg;      ///< Flag to supress engine failure message during takeoff.
    bool useThetaMod;          ///< Flag to indicate the system uses ThetaModifier.
    bool useUafRedoCircle;     ///< use with UAF_REDO_CIRCLE flag.
    bool useUafSetDfltAlt;     ///< use with UAF_SET_DFLT_ALT flag.
    bool useUafSetDfltAirSpd;  ///< use with UAF_SET_DFLT_AIRSPD flag.
    bool useUafSetCrcAirSpd;   ///< use with UAF_SET_CRC_AIRSPD flag.
    bool lastPhiTrackState;    ///< State of the Phi_Track controller in the moment of turning on "manual turn" mode.
    bool lastPhiCTrackState;   ///< State of the Phi_CTrack controller in the moment of turning on "manual turn" mode.
    bool doNotFinishPhaseInMT; ///< Do not break current phase instruction in the "manual turn" mode.
    bool doNotFinishPhaseInCG; ///< Do not break current phase instruction in the "camera guide" mode.
    bool skipCommandInMT;      ///< Flag - force to skip to the next flight instruction after turning on "manual turn" mode.
    bool skipCommandInCG;      ///< Flag - force to skip to the next flight instruction after turning on"camera guide" mode.
    bool originPosIsSet;       ///< Origin position has been set.
    bool basePosIsSet;         ///< Base posiiton has been set.
    bool observationMode;      ///< Flag to turn on observation mode (different set of controllers parameters).

    /** @name Stall recovery mode variables. 
  * @{ 
  */
    bool			stall;
    FPRealCondition stallCondition;		///<  Stall recovery function begining condition.
    bool			stallSavedDoNotMT;
    bool			stallSavedDoNotCG;
    FPRealCondition stallSavedExitCondition;
    FPRealData		stallSavedFprd;		///< Copy of the fprd saved before stall recovery function.
    ///@}

  /** @name Variables associated with operating the flight on low speed relative to the ground.
  * @{
  */
    bool			lowGSpeed;			///< Flag switching to the Phi_Psi controller.
    FPRealCondition lowGSpeedCondition;
    FPRealData::ControllerPropertiesSet lowGSpeedSavedCtrlProps;
  ///@}
 
    FPRealCondition emrgLandingCondition;   ///< The emergency landing conditions.
    float       circleTime; ///< Desired circle time in circle|hold instruction.

    ///  Constructor
    StateData() :
      gotoLineId(0),
      lastGotoTimeStamp(0),
      doNotBreak(false),
      doNotBreakByRetMode(false),
      doNotManualTurn(false),
      manualTurnMode(false),
      doNotCameraGuide(false),
      nLandPhaseFlag(UNKNOWN),
	  tkfPhaseFlag(TKF_UNKNOWN),
	  landParachutePhaseFlag(LND_UNKNOWN),
      cameraGuideMode(false),
	  bAllowEngineMsg(true),
      useThetaMod(true),
      useUafRedoCircle(false),
      useUafSetDfltAlt(false),
      useUafSetDfltAirSpd(false),
      useUafSetCrcAirSpd(false),
      lastPhiTrackState(false),
      lastPhiCTrackState(false),
      doNotFinishPhaseInMT(false),
      doNotFinishPhaseInCG(false),
      skipCommandInMT(false),
      skipCommandInCG(false),
      originPosIsSet(false),
      basePosIsSet(false),
      observationMode(false),
      stall(false),
      stallSavedDoNotMT(false),
      stallSavedDoNotCG(false),
      lowGSpeed(false),
      circleTime(0.0f)
    {};

    void resetMode (void);
    void resetConditions (void);        ///< Delete the set of ending phases conditions.
    void setDoNotBreakFlag (bool newValue);   ///< Change variable doNotBreak and send event.
  };
  
  /** @name Tags received at observer object registration.
  * @{
  */
  int _fPlanTag,
    _pStateTag,
    _sysMonTag,
    _redoTag,
    _extCmdTag,
	_servomanErmParachuteTag,
	_sysMonNoDgpsTag;
  ///@}

  int	_mainCounter;               ///< Main counter based on notification from the PhysicalState.
  int	_cgCounter;                 ///< Camera guide loop counter.
  int	_commCounter;               ///< Telemtry counter.
  int	_logCounter;                ///< Log telemetry counter.
  bool	_isConfigLoaded;            ///< Flag showing that configuration from flash is loaded.

  ParameterNames*	_pars;          ///< Pointer to the object storing array of names and parameters descroptions.
  ParameterNames*	_alg;			///< Pointer to the object storing array of names and parameters descroptions.
  FlightConstraint* _flightConstraint;
  Semaphore			_vSem;          ///< Semaphor controlling access to the subsystems data (_fprd).
  FParser			_parser;        ///< Command parser from the communication channel.
  StorageBase*		_confMem;       ///< Subsystem configuration memory.
  phaseFunction		_funTab[MAX_PHASE_FUN];     ///< Array of pointers to the flight phases functions.

  /**  @name zmienne stanu podsystemu. 
  * @{
  */
  CmdQueue<4>    _cmdq;            ///< Queue input lines (subsystems commands 4 lines).
  PStateData     _psd;             ///< Copy of the data from PState subsystem. 
  FPlanData      _fpd;             ///< Copy of the data from FPlan subsystem.
  ConfigData     _conf;            ///< Subsystem configuration data.
  StateData      _state;           ///< Subsystem data with subsystem state needed to reproduce after malfuction.
  RamStorageData _ramStorage;

  TlmCamShort   _tlmCam;          ///< Input telemtry with camera status.
  bool			_tlmCamGood;      ///< Camera telemetry correctness flag.
  ///@}

  void useCmdLine (ClassifiedLine &cl); ///< Command interpretation.
  void useFPlan (void);					///< Flight plan instruction interpretation. 
  void usePState (void);				///< Handling the notice from the PState about new values.
  bool confLoad (void);					///< Read subsystems configuration.
  bool confSave (void);					///< Save subsystems configuration.
  void fpCmdCompleted (void);           ///< Things to do after flight plan command had been completed.(notify, etc.)
  float cgComputePhi (float cameraPan, float cameraTilt) const;   ///< Calculate angle of the Phi Oblicza k�t Phi according to the gimbal position. (Camera guide mode)
  void cgSetPhi (void);					///< Set calculated value of the Phi angle.
  void redoCurrentFPLine (void);        ///< Redo current flight plan instruction (send notice).
  void setParamUserAction (unsigned int flags); ///< Execute some functions after parameter change.
  void sendTelemetry (void);			///< Send telemetry to the comunication chanel and to the log.
  bool prepareTlmLine (char* buf, int bufSize) const;  ///< Preparing telemetry line.
  
  /**  @name External commands|flight plan instructions.
  * @{
  */
  bool setParameters (const char* nameValueItems, bool fromFPlan);      
  bool getParameter (const char* pName, ClassifiedLine& cl) const;       
  bool disableControllers (bool fromFPlan);
  void tlmCamShort (const char* tlmData);
  bool turn (const char* pvalue);
  void turnOff (void);

  bool camGuideOn (void);
  void camGuideOff (void);
  void wait (const char* pTime);
  void waitCalcPara(void);
  void fly (const char* pLon, const char* pLat, const char* pMode, const char* pAlt);
  void flyPhase2 (void);
  void flyCamguide (const char* pLon, const char* pLat, const char* pMode, const char* pAlt);
  void flySearch ();
  void searchObj ();
#if EMRG_LOSS_ENGINE == 1
  void focusLand (void);
#endif
  void flyLastLink (void);
  void flyBase (void);
  void flyBaseLand(void);
  void flyParaPoint(void);
  void takeoff (void);
  void circle (const char* pTime, const char* pRadius, const char* pDir);
  void circlePhase1 (const GpsPosition* midPos, float radius, bool circleLeft, float time, float altitude=0.0f);
  void circleBaseParaPhase1(const GpsPosition* midPos, float radius, bool circleLeft, float time, float altitude = 0.0f);
  void circleCamguidePhase1(const GpsPosition* midPos, float radius, bool circleLeft, float time, float altitude = 0.0f);
  void circleParaPhase1 (void);
  void circleBase (void);
  void circleBasePara(void);
  void circleBasePreEstimateHeading(void);
  void headingAligning(void);
  void circleCamguide(void);
  void circleAround (const char* p1, const char* p2, const char* pMode, const char* pRadius, const char* pDir, const char* pAlt);
  void circleToClimb (double lat, double lon, float radius, float alt);
  void circleToClimbLand (double lat, double lon, float radius, float alt);
#if EMRG_LOSS_ENGINE == 1
  void circleToDescent (double lat, double lon, float radius, float alt = 0.0f);
#endif
  void circlePhase2 (void);
  void circleBaseParaPhase2 (void);
  void circleParaPhase2(void);
  void circleCamguidePhase2(void);
  bool forceLand (void);
  void forceLandPhase2 (void);
  void forceLandPhase3 (void);
  void fpGoto (const char* pLineId);
  void fpGotoPhase2 (void);
  void standby (void);
  void onHold (void);
  void stallOn (void);
  void stallOff (void);
  void lowGSpeedOn (void);
  void lowGSpeedOff (void);
  void iasFault(bool on);

  void parachuting(void);

  ///@}

  /**  @name Functions used to set controllers and conditions.
  * @{
  */
  void setStallRecovery (void);
  void setLowGSpeedCheck (void);
  void resetLowGSpeedCheck (void);
  void setEmergencyLandingCheck (void);

    // autoTakeoff catapult
  void autoTkfCatapultPhase1(void);
  void autoTkfCatapultPhase2(void);
  void autoTkfCatapultPhase3(void);
  void autoBeginMission(void);    

  void setLatControllers (CONTROLLER_CHANNEL_ID ID);
  void setElvControllers (CONTROLLER_CHANNEL_ID ID);
  void setQControllers (CONTROLLER_CHANNEL_ID ID);
  void setThetaControllers (CONTROLLER_CHANNEL_ID ID);
  void setVertSpeedControllers (CONTROLLER_CHANNEL_ID ID);

  void setLonControllers (CONTROLLER_CHANNEL_ID ALR_ID, CONTROLLER_CHANNEL_ID RDR_ID = LON_UNCHANGED_RDR_ID);
  void setAlrControllers (CONTROLLER_CHANNEL_ID ALR_ID);
  void setPControllers (CONTROLLER_CHANNEL_ID ALR_ID);
  void setPhiControllers (CONTROLLER_CHANNEL_ID ALR_ID);
  void setTrackControllers (CONTROLLER_CHANNEL_ID ALR_ID);
  void setTrackCorrControllers (CONTROLLER_CHANNEL_ID ALR_ID);

  void setRdrControllers (CONTROLLER_CHANNEL_ID RDR_ID);
  void setRControllers (CONTROLLER_CHANNEL_ID RDR_ID);
  
  void setSpdControllers (CONTROLLER_CHANNEL_ID THR_ID, CONTROLLER_CHANNEL_ID FLP_ID = SPD_UNCHANGED_FLP_ID);
  void setThrControllers (CONTROLLER_CHANNEL_ID THR_ID);
  void setFlpControllers (CONTROLLER_CHANNEL_ID FLP_ID);

  void setTuneControllers (void);
  void setTuneLatControllers (void);
  void setTuneLonControllers (void);
  void setTuneSpdControllers (void);

  enum FLIGHT_MODE
  {
	  UNKNOWN = 0,
	  FLY_MODE,
	  CRC_MODE
  };
  void setControllers (FLIGHT_MODE flightMode = FLY_MODE);
  ///@}

  // landing
  void netPrelandCompute(const char* p1, const char* p2, const char* p3, const char* p4, const char* p5, const char* p6);
  void netPrelandPhase2(void);
  void netReadyApproach (void);
  void netApproach (void);
  void netDescent (void);
  void netLand (void);
  void netPostLand (void);
  void netAbort (void);
  void netGoAround (void);

  // Optimize insert param
  bool setControlPropertiseParams (int numIndex);
  bool setFlightControlParams (int numIndex);
  bool setFlightReferenceParams (int numIndex);
  bool setGpsPosition (int numIndex);
  bool setDefaultParams (int numIndex);
  bool setHoldModeParams (int numIndex);
  bool setManualTurnModeParams (int numIndex);
  bool setCameraGuideModeParams (int numIndex);
  bool setLinkBrokenParams (int numIndex);
  bool setFlyModeParams (int numIndex);
  bool setCrcModeParams (int numIndex);
  bool setSemiLandModeParams (int numIndex);
  bool setNetParams (int index);
  bool setForceLandParams (int numIndex);
  bool setEmergencyLandParams (int numIndex);
  bool setStallRecoveryParams (int numIndex);
  bool setAutoTkfCatapultParams (int index);
  bool setAutoParachutingParams(int index);
  bool setSubsystemParams (int numIndex);
  void setFlightPhase (void);

  // Optimize set reference for flight phase
  bool setRefWaypoint (const char* lat, const char* lon, const char* mode);
  bool setRefAltitude (const char* alt);
  bool _circleToClimb;

  bool _bTuneMode;
  bool _bFirstSetControllers;

#if EMRG_LOSS_ENGINE == ENABLED
  bool _circleToDescent;
#endif
};

#endif  // FLIGHTPLANREALIZER_H

