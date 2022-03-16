/**
*                                                                   
* @class GlobVars                                                          
*                                                                   
* @brief Global variables
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef GLOBVARS_H
#define GLOBVARS_H

extern OSBase* Os; ///< Autopilot's operating system

extern SystemLog Log; ///< System log

/**  @name Pointers to the global objects (subsystems)
* @{
*/
extern SerialDeviceBase* 		Gps;
extern SerialDeviceBase* 		Tty0;
extern SerialDeviceBase* 		Tty1;
extern SerialDeviceBase* 		Tty2;
#if MAGNETOMETER_TYPE == USE_HMR
extern SerialDeviceBase* 		Hmr;
#endif
extern CommChannel*      		CTty485;
#if USE_CAM == CAM_ENABLE
extern UartCAM*					TtyCAM;
#endif
extern SerialDeviceBase* 		SimTty;    ///< Alternative pointer
extern LineDispatcher* 			LDisp;
extern FlightPlan* 				FPlan;
extern FlightPlanRealizer* 		FPReal;
extern FlightController* 		FControl;
extern ServoManager* 			ServMan;
extern PhysicalState* 			PState;
extern SystemMonitor* 			SysMon;
extern StorageBase* 			LogStor;
extern LogManager* 				LogMan;
//extern ADIS16467* 				IMU;
///@}


/**  @name Proceeses priorities (ID)
* @{
*/
extern const INT8U RootTaskPriority;
extern const INT8U Tty0InTaskPriority;
extern const INT8U Tty1InTaskPriority;
extern const INT8U Tty2InTaskPriority;
extern const INT8U GpsInTaskPriority;
extern const INT8U LDispTaskPriority;
extern const INT8U PStateTaskPriority;
extern const INT8U FControlTaskPriority;
extern const INT8U ServManTaskPriority;
extern const INT8U FPRealTaskPriority;
extern const INT8U FPlanTaskPriority;
extern const INT8U CamManTaskPriority;
extern const INT8U SysMonTaskPriority;
extern const INT8U Tty0OutTaskPriority;  
extern const INT8U Tty1OutTaskPriority;
extern const INT8U Tty2OutTaskPriority;
extern const INT8U GpsOutTaskPriority;
extern const INT8U LogStorTaskPriority;
extern const INT8U LogManTaskPriority;
///@}

/**  @name Variables that specify if the system was turned on on the ground
* @{
*/
extern bool SystemStartedOnGround;
extern bool SystemNowOnGround;
extern bool GlobalParamsLoaded;
///@}

extern bool allowDisarmEngine;

/**
* Global parameters read from flash before start of multitasking
*/
class GlobalParams 
{
public:
    int  commTtyNo;
    int  simTtyNo;
    bool blueToothEnable;
    bool commCrc;       ///< Turn on CRC in telemetry and events
    bool commMsgCrc;    ///< Turn on CRC to the messages (msg, err, utc)
    bool rs485New;      ///< Turn on new RS485 mode

    GlobalParams (void);    ///< Constructor
    void setDefault (void); ///< Set default parameters
};

extern GlobalParams GPar;
extern const char GPAR_FILE_NAME[];

/// Simulator Object (could be XPlane or Flightgear
class Simulator
{
public:
    static void setAileron  (SerialDeviceBase* device, float value);
    static void setRudder   (SerialDeviceBase* device, float value);
    static void setElevator (SerialDeviceBase* device, float value);
    static void setThrottle (SerialDeviceBase* device, float value);
    static void setFlap     (SerialDeviceBase* device, float value);
 
private:
    static void sendCmd (SerialDeviceBase* device, char* string, float value);
};

/**  @name Serial number of autopilot (read from flash before start of multitasking)
* @{
*/
extern unsigned int SerialNo;
extern const char SERIALNO_FILE_NAME[];

extern int ResetCounter;
extern const char RESETCNT_FILE_NAME[];
///@}

#endif //GLOBVARS_H
