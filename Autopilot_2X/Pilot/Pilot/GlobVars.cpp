/**
*                                                                   
* @class GlobVars                                                          
*                                                                   
* @brief Global variables
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

/// Autopilots operatin system
OSBase* Os = NULL; 

/// System LOG
SystemLog Log;

// Pointers to the global objects (subsystems)
SerialDeviceBase*   Gps      = NULL;
SerialDeviceBase*   Tty0     = NULL;
SerialDeviceBase*   Tty1     = NULL;
SerialDeviceBase*   Tty2     = NULL;
#if MAGNETOMETER_TYPE == USE_HMR
SerialDeviceBase*   Hmr      = NULL;
#endif
CommChannel*        CTty485  = NULL;
#if USE_CAM == CAM_ENABLE
UartCAM*   			TtyCAM   = NULL;
#endif
SerialDeviceBase*   SimTty   = NULL;   // alternative pointer
LineDispatcher*     LDisp    = NULL;
FlightPlan*         FPlan    = NULL;
FlightPlanRealizer* FPReal   = NULL;
FlightController*   FControl = NULL;
ServoManager*       ServMan  = NULL;
PhysicalState*      PState   = NULL;
SystemMonitor*      SysMon   = NULL;
StorageBase*        LogStor  = NULL;
LogManager*         LogMan   = NULL;
//ADIS16467*			IMU		 = NULL;


//  If system was turned on on the ground.
bool SystemStartedOnGround = false;
bool SystemNowOnGround     = false;
bool GlobalParamsLoaded    = false;

bool allowDisarmEngine	   = true;

//  Global parameters read from flash before start of multitasking
GlobalParams GPar;
const char GPAR_FILE_NAME[] = "globalparams";

// Serial number of autopilot (read from flash before start of multitasking)
unsigned int SerialNo = 0;
const char SERIALNO_FILE_NAME[] = "serialno";

// Reset counter of autopilot (read from flash before start of multitasking)
int ResetCounter = 0;
const char RESETCNT_FILE_NAME[] = "resetcnt";


/**
* GlobalClass - constructor with configuration data
*/
GlobalParams::GlobalParams(void)
{
    setDefault ();
}


/**
* Sets default configuration data.
*/
void GlobalParams::setDefault(void)
{
    commTtyNo       = 1;
    simTtyNo        = 1;
    blueToothEnable = false;
    commCrc         = true;
    commMsgCrc      = false;
    rs485New        = false;
}

void Simulator::setAileron (SerialDeviceBase* device, float value)
{
    sendCmd(device, "set controls/flight/aileron %f ", value);
}

void Simulator::setRudder (SerialDeviceBase* device, float value)
{
    sendCmd(device, "set controls/flight/rudder %f ", value);
}

void Simulator::setElevator (SerialDeviceBase* device, float value)
{
    sendCmd(device, "set controls/flight/elevator %f ", value);
}

void Simulator::setThrottle (SerialDeviceBase* device, float value)
{
    sendCmd(device, "set controls/engines/engine/throttle %f ", value);
}

void Simulator::setFlap (SerialDeviceBase* device, float value)
{
    sendCmd(device, "set controls/flight/flaps %f ", value);
}

void Simulator::sendCmd (SerialDeviceBase* device, char* string, float value)
{
    char buf[LINESIZE];

    SNPRINTF (buf, sizeof(buf), string, value);
    device->sendLine(buf);
}
