/**
*                                                                  
* @brief Autopilot - main file                                           
*                                                                   
*                                                                   
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_HARDWARE
static void StartupTask (void *p_arg);
#endif

// Stacks sizes for tasks
// Note: In Win32 enviroment are ignored

static const int RootTaskStkSize        = 6000;
static const int GpsInTaskStkSize       = 6000;
static const int GpsOutTaskStkSize      = 6000;
static const int Tty0InTaskStkSize      = 6000;
static const int Tty0OutTaskStkSize     = 6000;
static const int Tty1InTaskStkSize      = 6000;
static const int Tty1OutTaskStkSize     = 6000;
static const int Tty2InTaskStkSize      = 6000;
static const int Tty2OutTaskStkSize     = 6000;
#if USE_CAM == CAM_ENABLE
static const int TtyCAMInTaskStkSize    = 6000;
static const int TtyCAMOutTaskStkSize   = 6000;
#endif
#if MAGNETOMETER_TYPE == USE_HMR
static const int HmrInTaskStkSize 		= 6000;
static const int HmrOutTaskStkSize   	= 6000;
#endif
static const int LDispTaskStkSize       = 6000;
static const int SysMonTaskStkSize      = 6000;
static const int PStateTaskStkSize      = 6000;
static const int ServManTaskStkSize     = 6000;
static const int FControlTaskStkSize    = 6000;
static const int FPRealTaskStkSize      = 6000;
static const int FPlanTaskStkSize       = 6000;
static const int LogStorTaskStkSize     = 6000;
static const int LogManTaskStkSize      = 6000;
static const int IMUTaskStkSize  	 = 6000;


//  Processes priorities (ID) 
//  Gaps in the numbering for diagnostic purposes
const INT8U IMUTaskPriority 	 = 3;
const INT8U RootTaskPriority     = 5;
const INT8U Tty0InTaskPriority   = 8;   //  Receive data
const INT8U Tty1InTaskPriority   = 9;
const INT8U GpsInTaskPriority    = 10;
const INT8U Tty2InTaskPriority   = 11;
//const INT8U Tty485InTaskPriority = 13;
#if MAGNETOMETER_TYPE == USE_HMR
const INT8U HmrInTaskPriority 	= 13;
#endif
const INT8U LDispTaskPriority    = 14;
#if USE_CAM == CAM_ENABLE
const INT8U TtyCAMInTaskPriority = 15;
#endif
const INT8U PStateTaskPriority   = 16;

const INT8U FControlTaskPriority = 18;

const INT8U ServManTaskPriority  = 20;

const INT8U FPRealTaskPriority   = 22;

const INT8U FPlanTaskPriority    = 24;
#if USE_CAM == CAM_ENABLE
const INT8U TtyCAMOutTaskPriority= 25;
#endif
#if MAGNETOMETER_TYPE == USE_HMR
const INT8U HmrOutTaskPriority   = 26;
#endif

const INT8U Tty2OutTaskPriority  = 28;
const INT8U Tty0OutTaskPriority  = 29;  //  Send data
const INT8U Tty1OutTaskPriority  = 30;
const INT8U GpsOutTaskPriority   = 31;
 

const INT8U SysMonTaskPriority   = 32;  // System monitor (includes active loops to SPI handling)

const INT8U LogStorTaskPriority  = 34;  // Saving to log (includes active loops to Flash handlinga)
const INT8U LogManTaskPriority   = 35;  // Read/clear log must be > LogStorTaskPriority (access to flash'a)

CPU_STK StartupTaskStk[RootTaskStkSize];
// Space reservation for processes stacks.
OS_STK  RootTaskStk     [RootTaskStkSize];
OS_STK  GpsInTaskStk    [GpsInTaskStkSize];
OS_STK  GpsOutTaskStk   [GpsOutTaskStkSize];
OS_STK  Tty0InTaskStk   [Tty0InTaskStkSize];
OS_STK  Tty0OutTaskStk  [Tty0OutTaskStkSize];
OS_STK  Tty1InTaskStk   [Tty1InTaskStkSize];
OS_STK  Tty1OutTaskStk  [Tty1OutTaskStkSize];
OS_STK  Tty2InTaskStk   [Tty2InTaskStkSize];
OS_STK  Tty2OutTaskStk  [Tty2OutTaskStkSize];
//OS_STK  Tty485InTaskStk [Tty485InTaskStkSize];
//OS_STK  Tty485OutTaskStk[Tty485OutTaskStkSize];
#if USE_CAM == CAM_ENABLE
OS_STK  TtyCAMInTaskStk [TtyCAMInTaskStkSize];
OS_STK  TtyCAMOutTaskStk[TtyCAMOutTaskStkSize];
#endif
#if MAGNETOMETER_TYPE == USE_HMR
OS_STK  HmrInTaskStk [HmrInTaskStkSize];
OS_STK  HmrOutTaskStk[HmrOutTaskStkSize];
#endif
OS_STK  LDispTaskStk    [LDispTaskStkSize];
OS_STK  FPlanTaskStk    [FPlanTaskStkSize];
OS_STK  FPRealTaskStk   [FPRealTaskStkSize];
OS_STK  FControlTaskStk [FControlTaskStkSize];
OS_STK  ServManTaskStk  [ServManTaskStkSize];
OS_STK  PStateTaskStk   [PStateTaskStkSize];
OS_STK  SysMonTaskStk   [SysMonTaskStkSize];
OS_STK  LogStorTaskStk  [LogStorTaskStkSize];
OS_STK  LogManTaskStk   [LogManTaskStkSize];
OS_STK  IMUTaskStk   	[IMUTaskStkSize];


/**  @name Autopilot's operating system processes functions.
* It is not possible to directly call a class method C++ as the main function of the process
* Autopilot can operate under a system uC / OS-II (NIOS platform or Windows), or under Windows (mechanisms of native-only on Windows)
* Since the uC / OS-II and Windows require a different syntax main thread function, preprocessor definitions have been applied TSKPROC and TSKRET different for uC / OS-II and Windows
* @{
*/
TSKPROC GpsInTaskWrapper (void *p_arg)
{
    Gps->taskIn(p_arg);
    TSKRET;
}

TSKPROC GpsOutTaskWrapper (void *p_arg)
{
    Gps->taskOut(p_arg);
    TSKRET;
}

#if MAGNETOMETER_TYPE == USE_HMR
TSKPROC HmrInTaskWrapper (void *p_arg)
{
    Hmr->taskIn(p_arg);
    TSKRET;
}

TSKPROC HmrOutTaskWrapper (void *p_arg)
{
    Hmr->taskOut(p_arg);
    TSKRET;
}
#endif

TSKPROC Tty0InTaskWrapper (void *p_arg)
{
    Tty0->taskIn(p_arg);
    TSKRET;
}

TSKPROC Tty0OutTaskWrapper (void *p_arg)
{
    Tty0->taskOut(p_arg);
    TSKRET;
}

TSKPROC Tty1InTaskWrapper (void *p_arg)
{
    Tty1->taskIn(p_arg);
    TSKRET;
}

TSKPROC Tty1OutTaskWrapper (void *p_arg)
{
    Tty1->taskOut(p_arg);
    TSKRET;
}

TSKPROC Tty2InTaskWrapper (void *p_arg)
{
    Tty2->taskIn(p_arg);
    TSKRET;
}

TSKPROC Tty2OutTaskWrapper (void *p_arg)
{
    Tty2->taskOut(p_arg);
    TSKRET;
}

TSKPROC Tty485InTaskWrapper (void *p_arg)
{
    CTty485->taskIn(p_arg);
    TSKRET;
}

TSKPROC Tty485OutTaskWrapper (void *p_arg)
{
    CTty485->taskOut(p_arg);
    TSKRET;
}
#if USE_CAM == CAM_ENABLE
TSKPROC TtyCAMInTaskWrapper (void *p_arg)
{
    TtyCAM->taskIn(p_arg);
    TSKRET;
}

TSKPROC TtyCAMOutTaskWrapper (void *p_arg)
{
	TtyCAM->taskOut(p_arg);
    TSKRET;
}
#endif

TSKPROC LDispTaskWrapper (void *p_arg)
{
    LDisp->task(p_arg);
    TSKRET;
}

TSKPROC FPlanTaskWrapper (void *p_arg)
{
    FPlan->task(p_arg);
    TSKRET;
}

TSKPROC FPRealTaskWrapper (void *p_arg)
{
    FPReal->task(p_arg);
    TSKRET;
}

TSKPROC FControlTaskWrapper (void *p_arg)
{
    FControl->task(p_arg);
    TSKRET;
}

TSKPROC SysMonTaskWrapper (void *p_arg)
{
    SysMon->task(p_arg);
    TSKRET;
}

TSKPROC ServManTaskWrapper (void *p_arg)
{
	ServMan->task(p_arg);
    TSKRET;
}

TSKPROC PStateTaskWrapper (void *p_arg)
{
    PState->task(p_arg);
    TSKRET;
}

TSKPROC LogStorTaskWrapper (void *p_arg)
{
    LogStor->task(p_arg);
    TSKRET;
}

TSKPROC LogManTaskWrapper (void *p_arg)
{
    LogMan->task(p_arg);
    TSKRET;
}

///@}

/**
* Main task that create other processes
*/
TSKPROC RootTask (void *p_arg)
{

    int tcErr = 0;

    //  Initialize the collection of statistics (global variable OSCPUUsage gives load in %)
    //  When there was an reset in the air, function is not being executed due to long time of calibration (100ms)
    //  Not use for Win32
    if (SystemStartedOnGround)
        Os->statInit();

    //  Gps
    //  Communication chanels are optional, depending on configuration
    if (Gps != NULL)
    {
        if (!Os->taskCreate(GpsInTaskWrapper, (void *)0, &GpsInTaskStk[0], GpsInTaskStkSize, GpsInTaskPriority, &tcErr))
            // Ends or resetes program
            Log.abort ("Critical Error: RootTask_GpsIn [", tcErr, "].");

        if (!Os->taskCreate(GpsOutTaskWrapper, (void *)0, &GpsOutTaskStk[0], GpsOutTaskStkSize, GpsOutTaskPriority, &tcErr))
            // Ends or resetes program
            Log.abort ("Critical Error: RootTask_GpsOut [", tcErr, "].");
    }
#if MAGNETOMETER_TYPE == USE_HMR
    //  Hmr2300
    //  Communication chanels are optional, depending on configuration
    if (Hmr != NULL)
    {
        if (!Os->taskCreate(HmrInTaskWrapper, (void *)0, &HmrInTaskStk[0], HmrInTaskStkSize, HmrInTaskPriority, &tcErr))
            // Ends or resetes program
			Log.abort("Critical Error: RootTask_HmrIn [", tcErr, "].");

        if (!Os->taskCreate(HmrOutTaskWrapper, (void *)0, &HmrOutTaskStk[0], HmrOutTaskStkSize, HmrOutTaskPriority, &tcErr))
            // Ends or resetes program
			Log.abort("Critical Error: RootTask_HmrOut [", tcErr, "].");
    }
#endif
#if PILOT_TARGET == PT_WIN32
    //  Tty0
    if (Tty0 != NULL)
    {
        if (!Os->taskCreate(Tty0InTaskWrapper, (void *)0, &Tty0InTaskStk[0], Tty0InTaskStkSize, Tty0InTaskPriority, &tcErr))
            Log.abort ("Critical Error: RootTask_Tty0In [", tcErr, "].");

        if (!Os->taskCreate(Tty0OutTaskWrapper, (void *)0, &Tty0OutTaskStk[0], Tty0OutTaskStkSize, Tty0OutTaskPriority, &tcErr))
            Log.abort ("Critical Error: RootTask_Tty0Out [", tcErr, "].");
    }
#endif

    //  Tty1
    if (Tty1 != NULL)
    {
        if (!Os->taskCreate(Tty1InTaskWrapper, (void *)0, &Tty1InTaskStk[0], Tty1InTaskStkSize, Tty1InTaskPriority, &tcErr))
			Log.abort("Critical Error: RootTask_Tty1In [", tcErr, "].");

        if (!Os->taskCreate(Tty1OutTaskWrapper, (void *)0, &Tty1OutTaskStk[0], Tty1OutTaskStkSize, Tty1OutTaskPriority, &tcErr))
			Log.abort("Critical Error: RootTask_Tty1Out [", tcErr, "].");
    }

    //  Tty2
    if (Tty2 != NULL)
    {
        if (!Os->taskCreate(Tty2InTaskWrapper, (void *)0, &Tty2InTaskStk[0], Tty2InTaskStkSize, Tty2InTaskPriority, &tcErr))
			Log.abort("Critical Error: RootTask_Tty2In [", tcErr, "].");

        if (!Os->taskCreate(Tty2OutTaskWrapper, (void *)0, &Tty2OutTaskStk[0], Tty2OutTaskStkSize, Tty2OutTaskPriority, &tcErr))
			Log.abort("Critical Error: RootTask_Tty2Out [", tcErr, "].");
    }
#if USE_CAM == CAM_ENABLE
    //  TtyCAM
    if (TtyCAM != NULL)
    {
        if (!Os->taskCreate(TtyCAMInTaskWrapper, (void *)0, &TtyCAMInTaskStk[0], TtyCAMInTaskStkSize, TtyCAMInTaskPriority, &tcErr))
			Log.abort("Critical Error: RootTask_TtyCAMIn [", tcErr, "].");

        if (!Os->taskCreate(TtyCAMOutTaskWrapper, (void *)0, &TtyCAMOutTaskStk[0], TtyCAMOutTaskStkSize, TtyCAMOutTaskPriority, &tcErr))
			Log.abort("Critical Error: RootTask_TtyCAMOut [", tcErr, "].");
    }
#endif

	//  LDisp
    if (!Os->taskCreate(LDispTaskWrapper, (void *)0, &LDispTaskStk[0], LDispTaskStkSize, LDispTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_LDisp [", tcErr, "].");

    //  FPlan
    if (!Os->taskCreate(FPlanTaskWrapper, (void *)0, &FPlanTaskStk[0], FPlanTaskStkSize, FPlanTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_FPlan [", tcErr, "].");

    //  FPReal
    if (!Os->taskCreate(FPRealTaskWrapper, (void *)0, &FPRealTaskStk[0], FPRealTaskStkSize, FPRealTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_FPReal [", tcErr, "].");

    //  FControl
    if (!Os->taskCreate(FControlTaskWrapper, (void *)0, &FControlTaskStk[0], FControlTaskStkSize, FControlTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_FControl [", tcErr, "].");

    //  SysMon
    if (!Os->taskCreate(SysMonTaskWrapper, (void *)0, &SysMonTaskStk[0], SysMonTaskStkSize, SysMonTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_SysMon [", tcErr, "].");

    //  ServMan
    if (!Os->taskCreate(ServManTaskWrapper, (void *)0, &ServManTaskStk[0], ServManTaskStkSize, ServManTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_ServMan [", tcErr, "].");

	//  PState
    if (!Os->taskCreate(PStateTaskWrapper, (void *)0, &PStateTaskStk[0], PStateTaskStkSize, PStateTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_PState [", tcErr, "].");
//    //IMU
//	if (!Os->taskCreate(IMUTaskWrapper, (void *)0, &IMUTaskStk[0], IMUTaskStkSize, IMUTaskPriority, &tcErr))
//		UCOS_Printf ("Critical Error: RootTask_IMU [", tcErr, "].");
    //  LogStor
    if (!Os->taskCreate(LogStorTaskWrapper, (void *)0, &LogStorTaskStk[0], LogStorTaskStkSize, LogStorTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_LogStor [", tcErr, "].");

    //  LogMan
    if (!Os->taskCreate(LogManTaskWrapper, (void *)0, &LogManTaskStk[0], LogManTaskStkSize, LogManTaskPriority, &tcErr))
		Log.abort("Critical Error: RootTask_LogMan [", tcErr, "].");
#if PILOT_TARGET == PT_WIN32
    Log.msgPrintf("Serial number: %u", SerialNo);
    Log.msgPrintf("S/W version  : %d.%d", SW_VER_MAJOR, SW_VER_MINOR);
    Log.msgPrintf("OS Type      : %s", Os->osName());
	Log.startPrint();
#endif

    //  Suspend main task - should create only child tasks
    //  From this point works created tasks
    Os->taskSuspend();
    TSKRET;
}

/**
* Defines if plane is on ground. It must work quickly before multitasking starts.
* It has been called only during system has been starting.
*/
static bool isOnGround (void)
{
    return true;
}


/**
* Load serial number ang global parameters from flash to the global variables.
* Note: Do not call many times - reserves and does not release memory.
*/
static bool loadGlobalParams (void)
{
    bool ret1 = false;
    bool ret2 = false;

    // Subsystems configuration memory.
    // Created in singleThread mode.
    StorageBase* confMem = StorageFactory::createConfigMemory(true);
    if(confMem == NULL)
    {
        Log.tryAbort ("Critical Error: loadGlobalParams_1.");
        return false;
    }

    //  Read serial number
    bool result = confMem->open(SERIALNO_FILE_NAME);

    if(result)
    {
        if (!confMem->load(&SerialNo, sizeof(SerialNo)))
        {
            SerialNo = 0;   // Load function could set random value.
            //Log.bootPrintf("Error: Can not load serial number (loadGlobalParams_2)." CRLF);
			Log.bootPrintf("Error: Can not load serial number (loadGlobalParams_2).\r\n");
        }
        else
            ret1 = true;
    }
    else
        //Log.bootPrintf("Error: Can not load serial number (loadGlobalParams_3)." CRLF);
		Log.bootPrintf("Error: Can not load serial number (loadGlobalParams_3).");

    confMem->close();

    result = confMem->open(GPAR_FILE_NAME);

    if (result)
    {
        if (!confMem->load(&GPar, sizeof(GPar)))
        {
            // In case of an error restore default values.
            GPar.setDefault();
//            Log.bootPrintf("Error: Can not load global parameters (loadGlobalParams_4)." CRLF);
			Log.bootPrintf("Error: Can not load global parameters (loadGlobalParams_4).\r\n");
        }
        else
        {
        	ret2 = true;
        }

    }
    else
//        Log.bootPrintf("Error: Can not load global parameters (loadGlobalParams_5)." CRLF);
		Log.bootPrintf("Error: Can not load global parameters (loadGlobalParams_5).\r\n");

    confMem->close();
#if PILOT_TARGET == PT_HARDWARE
	bool ret3 = false;
    //  Read reset counter
    result = confMem->open(RESETCNT_FILE_NAME);

    if(result)
    {
        if (!confMem->load(&ResetCounter, sizeof(ResetCounter)))
        {
        	ResetCounter = 0;   // Load function could set random value.
            //Log.bootPrintf("Error: Can not load serial number (loadGlobalParams_2)." CRLF);
			Log.bootPrintf("Error: Can not load reset counter (loadGlobalParams_6).\r\n");
        }
        else
        {
            ret3 = true;
        }
    }
    else
		Log.bootPrintf("Error: Can not load reset counter (loadGlobalParams_7).");

    confMem->close();
#endif
#if PILOT_TARGET == PT_HARDWARE
    return (ret1 && ret2 && ret3);
#elif PILOT_TARGET == PT_WIN32
	return (ret1 && ret2);
#endif
}


/**
* Initializes the hardware based on global parameters.
*/
static void initHardware (void)
{
    PlatformLayer::setBlueTooth (GPar.blueToothEnable);
}



int main(void)
{
    // Initialization basic hardware.
    PlatformLayer::initPlatform ();

    // Create object that handles functions of the autopilot's operating system
    Os = PlatformLayer::OSPrepare();
    if (Os == NULL)
		Log.abort("Critical Error: main_5.");

    //  Initialization of the operating systems structures and working threads.
    Os->init();

    //  Initiate object of the SystemLog class, that could not been done in constructor (before OSInit())
    Log.Init(); 

    //  Inicjalizacja przetworników (wymagana dla funkcji isOnGround)
    PlatformLayer::initSensors1();

    //  Defines if the plane is on the ground and save in the variable.
    SystemStartedOnGround = isOnGround();
    SystemNowOnGround = SystemStartedOnGround;

    // If the plane is on the ground send message to the console.
    Log.bootPrintEnabled = SystemStartedOnGround;

    Log.bootPrint ("<<< Viettel Autopilot (C) 2020-2025 >>>" CRLF CRLF);

#if PILOT_TARGET == PT_WIN32
    // Read global parameters and serial number from flash.
    GlobalParamsLoaded = loadGlobalParams ();
	Log.startPrint();
#endif

    //  Initialization of hardware that uses global parameters
    initHardware ();

    // Create and initialize subsystems that depend on the platform
    Log.bootPrint (CRLF "Starting subsystems:" CRLF);
//    print (CRLF "Starting subsystems:" CRLF);

    // Communication chanels initialization.
#if MAGNETOMETER_TYPE == USE_HMR
    PlatformLayer::initTty (Gps,Hmr, Tty0, Tty1, Tty2, CTty485);
#else
    PlatformLayer::initTty (Gps, Tty0, Tty1, Tty2, CTty485);
#endif

#if USE_CAM == CAM_ENABLE
    TtyCAM = new UartCAM(XPAR_UART_CAM_S_AXI_BASEADDR, 115200, 295);
#endif

	//  Create and initialize rest of the subsystems
    LogStor = StorageFactory::createLogMemory ();
    if(LogStor == NULL)
    {
        Log.abort ("Critical Error: main_1 [Cannot create log memory]");
        return 0;
    }

	Log.bootPrint("  - LDisp:\t ");
    LDisp = new LineDispatcher();

	Log.bootPrint("  - FPlan:\t ");
    FPlan = new FlightPlan();

	Log.bootPrint("  - FPReal:\t ");
    FPReal = new FlightPlanRealizer();

	Log.bootPrint("  - FControl:\t ");
    FControl = new FlightController();

	Log.bootPrint("  - ServMan:\t ");
    ServMan = new ServoManager();

	Log.bootPrint("  - PState:\t ");
    PState = new PhysicalState();

	Log.bootPrint("  - SysMon:\t ");
    SysMon = new SystemMonitor();

	Log.bootPrint("  - LogMan:\t ");
    LogMan = new LogManager();

//    IMU = ADIS16467::get_instance();
//    // Bind objects for Observer design pattern
    LDisp->linkObserver();
    FPlan->linkObserver();
    FPReal->linkObserver();
    FControl->linkObserver();
	ServMan->linkObserver();
	PState->linkObserver();
    SysMon->linkObserver();
    LogMan->linkObserver();

    if (Tty2 != NULL)   //Tty2 do not occur on every platform, but is an observer
    	Tty2->linkObserver();

#if PILOT_TARGET == PT_HARDWARE
#if MAGNETOMETER_TYPE == USE_HMR
	if(Hmr != NULL)
		Hmr->linkObserver();
#endif

#if USE_CAM == CAM_ENABLE
	if (TtyCAM != NULL)   //Tty2 do not occur on every platform, but is an observer
		TtyCAM->linkObserver();
#endif
#elif PILOT_TARGET == PT_WIN32
	// Set output for errors and messages
	// Parameters being set in "smon" subsystem configuration.
	Log.setCommDeviceByNo(GPar.commTtyNo);

	if (!Log.setLogFile(LogStor, "log"))
		Log.tryAbort("Error: main_3 [Cannot open log]");
	Log.bootPrintf(CRLF "Messages redirected to         : Tty%d, LogStor" CRLF, GPar.commTtyNo);

	//  Set the control output for the simulator
	if (GPar.simTtyNo == 1)
		SimTty = Tty1;
	else
		SimTty = Tty0;

	if (GPar.simTtyNo == 2)
		SimTty = Tty2;

	Log.bootPrintf("Simulator control redirected to: Tty%d" CRLF CRLF, GPar.simTtyNo);
#endif
//    // Initialization of interruptions handling
    initIrq ();

    int rootErr = 0;
#if PILOT_TARGET == PT_HARDWARE
    if (!Os->taskCreate(StartupTask, (void *)RootTask, &StartupTaskStk[RootTaskStkSize - 1], RootTaskStkSize, UCOS_START_TASK_PRIO, &rootErr))
		// Ends or resets program
		Log.abort("Critical Error: main_2 [", rootErr, "].");
#elif PILOT_TARGET == PT_WIN32

	if (!Os->taskCreate(RootTask, (void *)0, &RootTaskStk[0], RootTaskStkSize, RootTaskPriority, &rootErr))
		// Ends or resets program
		Log.abort("Critical Error: main_2 [", rootErr, "].");
#endif
	//  Start multitasking.
	Log.bootPrint("Start multitasking." CRLF CRLF);
    Os->start();
    //  Here can never reach!!!
    Log.abort ("Critical Error: main_4.");
    return 0; 
}

#if PILOT_TARGET == PT_HARDWARE

static void StartupTask (void *p_arg)
{
    KAL_ERR kal_err;
    CPU_INT32U tick_rate;
#if (UCOS_START_DEBUG_TRACE == DEF_ENABLED)
    MEM_SEG_INFO seg_info;
    LIB_ERR lib_err;
#endif
#if (APP_OSIII_ENABLED == DEF_ENABLED)
#if (OS_CFG_STAT_TASK_EN == DEF_ENABLED)
    OS_ERR  os_err;
#endif
#endif


    UCOS_IntInit();                                             /* Initialize interrupt controller.                     */

#if (APP_OSIII_ENABLED == DEF_ENABLED)
    tick_rate = OS_CFG_TICK_RATE_HZ;
#endif

#if (APP_OSII_ENABLED == DEF_ENABLED)
    tick_rate = OS_TICKS_PER_SEC;
#endif

    UCOS_TmrTickInit(tick_rate);                                /* Configure and enable OS tick interrupt.              */

#if (APP_OSIII_ENABLED == DEF_ENABLED)
#if (OS_CFG_STAT_TASK_EN == DEF_ENABLED)
    OSStatTaskCPUUsageInit(&os_err);
#endif
#endif

    KAL_Init(DEF_NULL, &kal_err);

    UCOS_StdInOutInit();
    GlobalParamsLoaded = loadGlobalParams ();

    Log.msgPrintf ("Serial number: %u" CRLF, SerialNo);
    Log.msgPrintf ("S/W version  : %d.%d" CRLF, SW_VER_MAJOR, SW_VER_MINOR);
    Log.msgPrintf ("OS Type      : %s" CRLF, Os->osName());
    //    // Set output for errors and messages
    //    // Parameters being set in "smon" subsystem configuration.
    Log.setCommDeviceByNo (GPar.commTtyNo);

    //  Set the control output for the simulator
    if (GPar.simTtyNo == 1)
    	SimTty = Tty1;
    else
        SimTty = Tty0;

    if (GPar.simTtyNo == 2)
    	SimTty = Tty2;

#if EXTERNAL_MEMORY_TYPE == USE_MMC
    char fileName[LINESIZE];
    if (snprintf (fileName, sizeof(fileName),"1:/log_default_uav_%i",ResetCounter) >= 0)
#elif EXTERNAL_MEMORY_TYPE == USE_SDCARD
    	char fileName[LINESIZE];
    	if (snprintf (fileName, sizeof(fileName),"log_default_uav_%d",ResetCounter) >= 0)
#endif
        {
    	    if (!Log.setLogFile (LogStor, fileName))
    	        Log.tryAbort ("Error: main_3 [Cannot open log]");
        }
//#if EXTERNAL_MEMORY_TYPE == USE_MMC
//    if (!Log.setLogFile (LogStor, "1:/log_default_uav"))
//        Log.tryAbort ("Error: main_3 [Cannot open log]");
//#elif EXTERNAL_MEMORY_TYPE == USE_SDCARD
//    if (!Log.setLogFile (LogStor, "log_default_uav"))
//        Log.tryAbort ("Error: main_3 [Cannot open log]");
//#endif
#if (UCOS_START_DEBUG_TRACE == DEF_ENABLED)
    Log.msgPrintf("UCOS - uC/OS Init Started");
    Log.msgPrintf("UCOS - STDIN/STDOUT Device Initialized");
#endif

#if (APP_SHELL_ENABLED == DEF_ENABLED)
    UCOS_Shell_Init();
#endif

#if ((APP_FS_ENABLED == DEF_ENABLED) && (UCOS_CFG_INIT_FS == DEF_ENABLED))
    UCOS_FS_Init();
#endif

#if ((APP_TCPIP_ENABLED == DEF_ENABLED) && (UCOS_CFG_INIT_NET == DEF_ENABLED))
    UCOS_TCPIP_Init();
#endif /* (APP_TCPIP_ENABLED == DEF_ENABLED) */

#if ((APP_USBD_ENABLED == DEF_ENABLED) && (UCOS_CFG_INIT_USBD == DEF_ENABLED) && (UCOS_USB_TYPE == UCOS_USB_TYPE_DEVICE))
    UCOS_USBD_Init();
#endif /* #if (APP_USBD_ENABLED == DEF_ENABLED) */

#if ((APP_USBH_ENABLED == DEF_ENABLED) && (UCOS_CFG_INIT_USBH == DEF_ENABLED) && (UCOS_USB_TYPE == UCOS_USB_TYPE_HOST))
    UCOS_USBH_Init();
#endif /* #if (APP_USBH_ENABLED == DEF_ENABLED) */

#if (UCOS_START_DEBUG_TRACE == DEF_ENABLED)
    Mem_SegRemSizeGet(DEF_NULL, 4, &seg_info, &lib_err);
    Log.msgPrintf ("UCOS - UCOS init done");
    Log.msgPrintf ("UCOS - Total configured heap size. %d", seg_info.TotalSize);
    Log.msgPrintf ("UCOS - Total used size after init. %d", seg_info.UsedSize);
#endif

    (*((CPU_FNCT_PTR)(p_arg)))(DEF_NULL);

}

#endif


