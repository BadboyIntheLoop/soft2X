/**
*                                                                  
* @class Defs                                                              
*                                                                   
* @brief Various declarations and definitions
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/
 
#ifndef DEFS_H
#define DEFS_H


#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#define CRLF "\r\n"

#define INPUT
#define OUTPUT
#define INOUT

//  Global constans
//const int LINESIZE = 20;          ///< Buffer size for text lines from communication chanel
const int LINESIZE = 250;          ///< Buffer size for text lines from communication chanel
const int SERVO_COUNT = 13;        ///< Handled servos count //truongnt
const int CUSTOM_OUTPUT_COUNT = 4; ///< Number of extra outputs (mapped to the servos)
const int SW_VER_MAJOR = 2;
const int SW_VER_MINOR = 2;

#if PILOT_TYPE == VUA_SC_3TH
    const char PILOT_TYPE_STR[]  = "VUA_SC_3TH";
#elif PILOT_TYPE == VUA_SM
    const char PILOT_TYPE_STR[]  = "VUA_SM";
#elif PILOT_TYPE == VUA_SL
    const char PILOT_TYPE_STR[]  = "VUA_SL";
#elif PILOT_TYPE == VUA_SC
    const char PILOT_TYPE_STR[]  = "VUA_SC";
#elif PILOT_TYPE == VUA_SC_6G
    const char PILOT_TYPE_STR[]  = "VUA_SC_6G";
#endif

#if ENGINE_TYPE == VUA_G
    const char ENGINE_TYPE_STR[]  = "_G";
#elif ENGINE_TYPE == VUA_E
    const char ENGINE_TYPE_STR[]  = "_E";
#endif

const int BANK_COUNT = 2;           ///< Number of bank controllers parameters

const float PI = 3.14159265358979323846f;
const float PI2 = PI*0.5f;
const float RAD_2_DEG = 180.0f / PI;
const float DEG_2_RAD = PI / 180.0f;
const float INHG_2_HPA = 33.8639f;      ///< inches mercury column -> hektopascale
const float KNOTS_2_KPH = 1.852f;       ///< knots -> kmph
const float MS_2_KPH = 3.6f;            ///< mps -> kmph
const float KPH_2_MS = 1.0f / 3.6f;     ///< kmph -> mps
const float G = 9.80665f;

const float RO = 1.168f;                ///< air density (1013.25, 30C)
const float STA_T = 15.0f;
const float STA_P = 1013.25f;
const float STA_RO = 1.225f;            ///< air density (1013.25, 15C)
const float KELVIN_ADD = 273.15f;       ///< Celsius -> kelvin

const double MIN_2_DEG = 1.0/60.0;

const char LOG_FILE_NAME[] = "log";

/**
* OBSERVERs template handling notifications types.
*/
enum ASPECT
{
    UNKNOWN_ASPECT,     ///< Reserved do not use.
    LINE_RECEIVED,      ///< Text line received by the communication chanel ready to be read.
    CH_DATA_RECEIVED,   ///< Data received by the communication chanel ready to be read.u (send by CommChannel)
    NMEA_LINE,          ///< Standard line NMEA (GPS) ready to be read by LDisp subsystem
    AUX_NMEA_LINE,      ///< Standard line NMEA (GPS) ready to be read by LDisp subsystem from extra GPS
	HMR_LINE,
    SIM_LINE,           ///< Command line from simulator ready to be read by LDisp subsystem.
    CMD_LINE,           ///< Command line ready to be read by LDisp subsystem.
    FP_CHANGED,         ///< Flight plan has beed changed
    FP_INTERNAL_GOTO,   ///< Internal FPlan notification.
    FPEL_COMPLETED,     ///< FPReal ended flight plan instruction execution.
    FPR_CHANGED,        ///< FPReal changed variable stettings.
    FPR_LANDED,         ///< FPReal done last landing phase.
    FPR_LAUNCHED,       ///< FPReal detect takeoff
	FPR_TAKEOFF_CPL,    ///< FPReal ended takeoff instruction
    FPR_SMOOTH_ON,      ///< FPReal perform flight plan phase that demand smoooth angles measurements
    FPR_SMOOTH_OFF,     ///< FPR_SMOOTH_ON turn off.
    FPR_INTERNAL_REDO,  ///< Internal FPReal notification
	FPR_ATTENTION,      ///< Pay attention interesting maneuver.
	FPR_NO_ATTENTION,   ///< Notify that something interesting has finished.
    FPR_STANDBY,        ///< FPReal switched to Stanby mode
	FPR_TKF_P2,			///< FPReal into Takeoff Catapult phase 2
	FPR_PARACHUTE,      ///< FPReal into Parachuting phase
	FPR_BASE_CHANGED,
    CONTROLS_COMPUTED,  ///< FControl calculated output signals
	CONTROLS_CLIMB,
	CONTROLS_CRUISE,
	CONTROLS_LEVELFLIGHT,
    PSTATE_CHANGED,     ///< New physical measurements from PState subsystem
    PSTATE_WIND,        ///< Calculated wind vector from PState subsyste.
    SYSMON_MSG,         ///< System monitor data has changed
    SYSMON_LANDED,      ///< Sysmon performed afterlanding procedure.
    SYSMON_LINK,        ///< Radio link status
    SYSMON_LOW_ENERGY,  ///< Energy supply sufficient only to return 
    SYSMON_NO_GPS,      ///< GPS malfunction
#if USE_DGPS == 1
	SYSMON_NO_DGPS,     ///< DGPS malfunction
#endif
    SYSMON_ENG_CTRL,    ///< Engine controller malfunction
#if EMRG_LOSS_ENGINE == 1
	SYSMON_LOSS_ENGINE,
	SYSMON_FOCUS_LAND,
#endif
    IRQ_ADC,            ///< Internal PState notification.
	PIC_N_CIC_CHANGED,	///< Pilot in Control / Computer in Control
	IRQ_FAST_RS,        ///< Interruption from FAST_RS
	IRQ_HMR_RS,
	IRQ_CAM,
	IRQ_CAM_RECEIVED,
#if LAND_MODE == SEMI_LAND
	SERVMAN_TRIGGER_ELV,
	SERVMAN_TRIGGER_ALR,
	SERVMAN_TRIGGER_ELV_ALR,
#endif
	SERVMAN_DISARM,
	SERVMAN_ERM_PARACHUTE,
    EXT_CMD             ///< Command line has been added to the queue of the subsystem (another subsystem called the function Notify the current subsystem)
};

enum DEBUG_MSG
{
	NONE = 0,
	HUB_FRAME_TYPE,
	HUB_VERIFY_HEADER,
	HUB_RAW_BYTES,
	HUB_PHY_DATA,
	RS485_SEND_DATA,
	HIGH_LOW_GYRO,
	FUEL_LEVEL
};

enum LAT_TUNING_ID
{
	LAT_TUNING_UNCHANGED = 0,
	LAT_TUNING_UNUSED,
	LAT_TUNING_HOLD_THETA,
	LAT_TUNING_THETA_ALT,
	LAT_TUNING_HOLD_VZ,
	LAT_TUNING_VZ_ALT,
	LAT_TUNING_HOLD_SPD
};
enum LON_TUNING_ID
{
	LON_TUNING_UNCHANGED = 0,
	LON_TUNING_UNUSED,
	LON_TUNING_HOLD_PHI,
	LON_TUNING_TRACK_WPT,
	LON_TUNING_TRACK_PATH
};
enum SPD_TUNING_ID
{
	SPD_TUNING_UNCHANGED = 0,
	SPD_TUNING_UNUSED,
	SPD_TUNING_HOLD_SPD,
	SPD_TUNING_HOLD_ALT
};

/**
* Telemetry records types (NOTE: do not change numbers they are used in base station)
*/
enum TLM_FORMAT
{
    TLM_PS_SHORT  = 1,          ///< Physical state - short format
    TLM_PS_LONG   = 2,          ///< Physical state - long format

	TLM_CONTROLLER_SHORT = 3,
	TLM_CONTROLLER_LONG = 4,
	
	TLM_CAMERA = 5,

    TLM_RAW_GAUGE = 6,          ///< Raw data from indicators
    TLM_NMEA      = 7,          ///< NMEA text sequence - basic GPS
    TLM_AUX_NMEA  = 8,          ///< NMEA text sequence - extra GPS
	
	TLM_SERVO     = 11,
    TLM_RC_IN	  = 12,
	TLM_SERVO_IN  = 13,
    TLM_SMON      = 15,         ///< SystemMonitor data
    TLM_REAL      = 16,         ///< FlightPlanRealizer - current subsystem state
    TLM_TARGET    = 17,         ///<  CameraManager - data
    TLM_CAM_SHORT = 18,         ///< Camera position output telemetry
    TLM_AUX_INFO  = 68
};


/**
* Events (NOTE: do not change numbers they are used in base station)
*/
enum EVT
{
    EVT_PIC_CIC    = 1,          ///< PIC/CIC mode
    EVT_FPR_REJECT = 2,          ///< Reject by FPReal FPlan's instruction or mode.
    EVT_FPR_ACCEPT = 3,          ///< Accept by FPReal FPlan's instruction or mode.
    EVT_FPR_MANUAL = 4,          ///< Turn ON / OFF manual turn mode
    EVT_FP_LINECHANGE = 6,       ///<  FPlan - new flightplan line
    EVT_FPR_CG = 7,              ///<  Turn ON / OFF camera guide mode
    EVT_SM_LANDED = 8,           ///< Plane had landed
    EVT_FPR_NOBREAK = 9,         ///< Do not break current instruction.
    EVT_FPR_TODROP = 11,         ///< Container shutdown
	EVT_CTRL_THRALERT = 12,      ///< engine malfunction
    EVT_ENGINE_ARM = 13,           ///< engine arm
    EVT_ENGINE_DISARM = 14         ///< engine disarm
};

#if PILOT_TARGET == PT_WIN32
    #define SNPRINTF Log.secureSnprintf
    #define STRNICMP _strnicmp
    #define STRICMP _stricmp
    #define MEMCCPY _memccpy
#else
    #define SNPRINTF Log.secureSnprintf
//    #define STRNICMP strnicmp
//    #define STRICMP stricmp
	#define STRNICMP strncasecmp
	#define STRICMP strcasecmp
    #define MEMCCPY memccpy
#endif  // PILOT_TARGET

/// Operating system type.

#define OS_UCOSII   1       ///< uC/OS-II on NIOS and W32 platform
#define OS_WIN      2       ///< Windows (only on W32 platform)

#define PILOT_OS OS_WIN

#endif //DEFS_H
