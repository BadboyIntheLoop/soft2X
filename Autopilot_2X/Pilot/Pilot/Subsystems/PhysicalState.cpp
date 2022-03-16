#include <PilotIncludes.h>

// Static members initialization
const char PhysicalState::SUBS_PREFIX[]           = "ps: ";
const char PhysicalState::CONF_FILE_NAME[]        = "pstatecfg";
const char PhysicalState::GAUGE_CONF_FILE_NAME[]  = "gaugecfg";

const char PhysicalState::ERR_OK[]                = "ps: ok";
const char PhysicalState::ERR_SYS[]               = "ps: PS01 System error";
const char PhysicalState::ERR_CLOAD[]             = "ps: PS02 Cannot load config data";
const char PhysicalState::ERR_CSAVE[]             = "ps: PS03 Cannot save config data";
const char PhysicalState::ERR_UCOMMAND[]          = "ps: PS04 Unrecognized command or wrong number of parameters";
const char PhysicalState::ERR_BADPAR[]            = "ps: PS05 Bad parameter name or value";
const char PhysicalState::MSG_NO_CONFIG[]         = "ps: PS07 Config data not loaded from flash";
const char PhysicalState::ERR_READONLY[]          = "ps: PS08 Cannot set ReadOnly parameter";
const char PhysicalState::ERR_GCLOAD[]            = "ps: PS20 Cannot load gauge config data";
const char PhysicalState::ERR_GCSAVE[]            = "ps: PS21 Cannot save gauge config data";
const char PhysicalState::ERR_MAG_VERIFY[]        = "ps: PS22 wrong magnetometer configuration";
const char PhysicalState::MSG_NO_GAUGE_CONFIG[]   = "ps: PS23 Gauge config data not loaded from flash";
const char PhysicalState::MSG_NO_GPS[]            = "ps: PS24 GPS not ready";
const char PhysicalState::MSG_GPS_BAD_PRECISION[] = "ps: PS25 GPS bad precision or 2D mode";
const char PhysicalState::MSG_SENSORS_ERROR[]     = "ps: PS26 Sensors error";
const char PhysicalState::MSG_BASE_PRESSURE[]     = "ps: PS27 Base station pressure out of range";
const char PhysicalState::MSG_MAG_ERROR[]         = "ps: PS28 Magnetometer error";
const char PhysicalState::MSG_INCLDECL_NOT_SET[]  = "ps: PS29 Inclination or declination not set";
const char PhysicalState::MSG_SENSORS_SPRESS_LOCKED_ERROR[] = "ps: PS30 Static pressure sensor locked";
const char PhysicalState::ERR_CMD_BUF_FULL[]      = "ps: PS30 Command buffer full";

const char PhysicalState::MSG_DIFF_PRESSURE_OFFSET[] = "ps: PS33 Get diffpressure offset failed";

const float PhysicalState::MIN_BASE_PRESSURE = 470.0f;  // 4000m
const float PhysicalState::MAX_BASE_PRESSURE = 1100.0f;
const float PhysicalState::SENS_FAIL_IAS = 70.0f;

const float PhysicalState::INCLINATION_NOT_SET = 10.0f;
const float PhysicalState::DECLINATION_NOT_SET = 10.0f;

// Default class constructor
PhysicalState::PhysicalState(void):
ODTObserver(), ODTSubject(), SubsystemBase(),
// Initialization in case of no execution the 'linkObserver' method
_simTag(-1), _nmeaTag(-1), _auxNmeaTag(-1),_hmrTag(-1), _irqAdcTag(-1), _smoothOnTag(-1), _smoothOffTag(-1),
_fPRealLaunchTag(-1), _fPRealStandbyTag(-1), _sysMonTag(-1), _extCmdTag(-1),
#if USE_CAM == CAM_ENABLE
_cameraTag(-1),
#endif
_cTty485Tag(-1),


// Counters initialization (-1 means sending a notification at the first iteration, which is important in FG simulation because FG can be paused
_notifyCounter(-1), _commCounter(-1), _commAuxSensorCounter(-1), _logCounter(-1), _hubCounter(-1), _logAuxSensorCounter(-1),
#if USE_CAM == CAM_ENABLE
		 _camCounter(-1),
#endif
_nmeaCommCounter(-1), _nmeaLogCounter(-1),
_cmdq(Log),

_gpsConfigured(false),
_gpsPacketStarted(false),
_gpsBeforeInitLines(0),
_elevationDlyCnt(0),
_datetimeSet(false),
#if USE_DGPS == ENABLED
_DGpsData(NULL),
#endif
_isConfigLoaded(false),
_isGaugeConfigLoaded(false),
_isGaugeMagnetometerOK(false),
_previousIas(0.0f),
_previousRawIas(0.0f),
_iasCounter(0),
_ias_1(0.0f),
_ias_2(0.0f),
_lastRawMagPress(0),
_lastRawMagPressCnt(0),
_gpsFailTest(false),
_gpsFailTestTOff100(0),
_gpsPredictionEnable(false),
_gpsMaxDistance(100.0f),
_gpsLastDistance(0.0f),
#if EMRG_LOSS_ENGINE == 1
_bTestLostEngine(false),
#endif
_485seq(0u),

#if PILOT_TARGET == PT_HARDWARE
xAdcPs_ConfigPtr(NULL),
_isConfigureXAdc(false),
xadcReadingDivisor(0),
#endif

_confMem(NULL),
_gaugeConfMem(NULL),
_confNames(NULL)
{
#if USE_DGPS == ENABLED
    _dgpsGood = false;
#endif
    
//    Log.bootPrint ("(SimLevel 0,1,3,4 only):\t ");
  
	// Test of the line delaying speed
	for(int i=0; i<48; i++)
    {
		delayLine[i] = 0.0f;
    }

	for (int i = 0; i < 2; i++)
	{
		airspeedDly[i] = 0.0f;
		filterAirspeedDly[i] = 0.0f;
	}

    //Creation of a subsystem data access controll semaphore. If semaphore is not created correctly a particular error message is logged.
	if (!_vSem.create("PhysicalState"))
    {
        Log.abort ("Critical Error: PhysicalState_1.");
        return;
    }

    // Creation of a memory configuration data
    _confMem = StorageFactory::createConfigMemory();
    if( _confMem == NULL)
    {
        Log.abort ("Critical Error: PhysicalState_2.");
        return;
    }

    // Creation of a sensors configuration data
    _gaugeConfMem = StorageFactory::createConfigMemory();
    if( _gaugeConfMem == NULL)
    {
        Log.abort ("Critical Error: PhysicalState_2a.");
        return;
    }

	// Initialization of a mapping of parameter names to objects. If initialization failed error message is logged.
    _confNames = new ParameterNames(SUBS_PREFIX, 77);
    if (_confNames == NULL)
    {
        Log.abort ("Critical Error: PhysicalState_3.");
        return;
    }

    // Initialization of a variables used by 'ps set/get..' commands
	// 'insert' function ends with abort in the case of error
    /* 1*/_confNames->insert("notifyDivisor", &_conf.notifyDivisor, 1, 1000);
	/* 2*/_confNames->insert("commDivisor",         &_conf.commDivisor, 0, 1000);
	/* 3*/_confNames->insert("commAuxSensorDivisor",&_conf.commAuxSensorDivisor, 0, 1000);
    /* 4*/_confNames->insert("logDivisor",          &_conf.logDivisor,  0, 1000);
	/* 5*/_confNames->insert("logAuxSensorDivisor", &_conf.logAuxSensorDivisor,  0, 1000);
    /* 6*/_confNames->insert("nmeaCommDivisor", &_conf.nmeaCommDivisor, 0, 1);
    /* 7*/_confNames->insert("nmeaLogDivisor", &_conf.nmeaLogDivisor, 0, 1);
    /* 8*/_confNames->insert("commFormat", &_conf.commFormat, 1, 2);
    /* 9*/_confNames->insert("logFormat", &_conf.logFormat, 1, 2);
    /*10*/_confNames->insert("stateLogEnable", &_conf.stateLogEnable);
    /*11*/_confNames->insert("rawGaugeLogEnable", &_conf.rawGaugeLogEnable);
	/*10*/_confNames->insert("rawGaugeCommEnable", &_conf.rawGaugeCommEnable);
	/*11*/_confNames->insert("auxSensorLogEnable", &_conf.auxSensorLogEnable);
	/*10*/_confNames->insert("auxSensorCommEnable", &_conf.auxSensorCommEnable);
    /*15*/_confNames->insert("gpsPredictionEnable", &_conf.gpsPredictionEnable);
    /*16*/_confNames->insert("gpsSbasEnable", &_conf.gpsSbasEnable);
    /*17*/_confNames->insert("gpsFailTest", &_gpsFailTest, false, UAF_GPS_FAIL);
    /*18*/_confNames->insert("gpsFailTestTimeout", &_conf.gpsFailTestTimeout, 0, 1000);
    /*19*/_confNames->insert("gpsFailNavEnable", &_conf.gpsFailNavEnable);
    /*20*/_confNames->insert("maxHdop", &_conf.maxHdop, 0.0f, 1000.0f, 1);
#if USE_DGPS == ENABLED
    /*20*/_confNames->insert("maxPdop", &_conf.maxPdop, 0.0f, 1000.0f, 1);
#endif
    /*21*/_confNames->insert("gpsTimeout", &_conf.gpsTimeout, 0.25f, 100.0f, 2);
#if USE_DGPS == ENABLED
	/*22*/_confNames->insert("dgpsTimeout", &_conf.dgpsTimeout, 0.25f, 1000.0f, 2);
#endif
    /*23*/_confNames->insert("simLevel", &_conf.simLevel, 0, 4);
	/*24*/_confNames->insert("asFilter", &_conf.asFilter);
	/*25*/_confNames->insert("altFilter", &_conf.altFilter);
    /*26*/_confNames->insert("pressureSource", &_conf.pressureSource, 0, 1);
    /*27*/_confNames->insert("useBasePressure", &_conf.useBasePressure);
    /*28*/_confNames->insert("takeoffAgl", &_conf.takeoffAgl, 0.0f, 100.0f, 2);
    /*29*/_confNames->insert("elevationSetDelay", &_conf.elevationSetDelay, 0.0f, 1000.0f, 1);

	/*30*/_confNames->insert("ahrsVersion", &_conf.ahrsVersion, 0, 100);
    /*31*/_confNames->insert("spSensorSwitchEnable", &_conf.spSensorSwitchEnable);
	/*32*/_confNames->insert("useKFAlt", &_conf.bUseKFAlt);
    /*44*/_confNames->insert("fuelLevelRollPitchRange", &_conf.fuelLevelRollPitchRange, 0.0f, 100.0f, 4);
	/*44*/_confNames->insert("gpsUereConfig", &_conf.gpsUereConfig, 0.0f, 100.0f, 4);
	/*44*/_confNames->insert("gpsPositionErrOffset", &_conf.gpsPositionErrOffset, 0.0f, 100.0f, 4);
	/*32*/_confNames->insert("gpsTestJammingEnable", &_conf.gpsTestJammingEnable);
	/*44*/_confNames->insert("gpsJammingTimeOut", &_conf.gpsJammingTimeOut, 0.0f, 1000.0f, 4);
#if USE_DGPS == 1
	/*33*/_confNames->insert("dgpsPredictionEnable", &_conf.dgpsPredictionEnable, 0, 30);
	/*34*/_confNames->insert("useDGps", &_conf.bUseDGps);
    /*34*/_confNames->insert("dgpsDevice", &_conf.dgpsDevice, 0, 2);
#endif
    /*35*/_confNames->insert("debugMsg", &_conf.debugMsg, 0, 100);

    // Variables of a state, stored in RAM only
	/*34*/_confNames->insert("inclination", &_stateMem.inclination, -10.0f, 10.0f, 8, false, UAF_AHRS_INCL);
	/*35*/_confNames->insert("declination", &_stateMem.declination, -10.0f, 10.0f, 8, false, UAF_AHRS_DECL);
    /*36*/_confNames->insert("elevationPressure", &_stateMem.elevationPressure, 100.0f, 1500.0f, 3, true);
    /*37*/_confNames->insert("refBasePressure", &_stateMem.refBasePressure, 100.0f, 1500.0f, 3, true);
    /*38*/_confNames->insert("currentBasePressure", &_stateMem.currentBasePressure, 0.0f, MAX_BASE_PRESSURE, 3, false, UAF_BASE_PRESS);
	/*39*/_confNames->insert("tlmOriginAmsl", &_stateMem.tlmOriginAmsl, -1000.0f, 10000.0f, 0);
    /*40*/_confNames->insert("elevationOffset", &_stateMem.elevationOffset, -10000.0f, 10000.0f, 2);
    /*41*/_confNames->insert("spSensorLocked", &_stateMem.spSensorLocked);
    /*42*/_confNames->insert("testLockSpSensor", &_gauge.lockMagPressureSensor);
	
	/*43*/_confNames->insert("filterAirspeedCoffA", &_conf.filterAirspeedCoff_a, -15.0f, 15.0f, 6);
	/*44*/_confNames->insert("filterAirspeedCoffB", &_conf.filterAirspeedCoff_b, -15.0f, 15.0f, 6);
	/*45*/_confNames->insert("filterAirspeedCoffC", &_conf.filterAirspeedCoff_c, -15.0f, 15.0f, 6);
	/*46*/_confNames->insert("filterAirspeedCoffD", &_conf.filterAirspeedCoff_d, -15.0f, 15.0f, 6);
	/*47*/_confNames->insert("filterAirspeedCoffE", &_conf.filterAirspeedCoff_e, -15.0f, 15.0f, 6);
	/*48*/_confNames->insert("filterAirspeedThreshold", &_conf.filterAirspeedThreshold, -100.0f, 100.0f);
	/*49*/_confNames->insert("filterAirspeedActive", &_conf.filterAirspeedActive, 0, 0xFFFF);
	/*47*/_confNames->insert("airSpdScaleRatio", &_conf.airSpdScaleRatio, 0.0f, 5.0f, 6);
    // Sensors configuration parameters (separate container)
    ParameterNames* gp = NULL;
    /*43*/_confNames->insertContainer("gauge.", 92, gp); //@@TODO
    if (gp == NULL)
    {
        Log.abort ("Critical Error: PhysicalState_3a.");
        return;
    }

    // Accelerometers
    /* 1*/gp->insert("accXLOff",   &_gauge.gGaugeParam.accXLOff,      -100000.0f, 100000.0f, 8);
    /* 2*/gp->insert("accXLSens",  &_gauge.gGaugeParam.accXLSens,  -100000.0f, 100000.0f, 8);
    /* 3*/gp->insert("accXLSensT", &_gauge.gGaugeParam.accXLSensT, -100.0f,    100.0f, 8);

    /* 4*/gp->insert("accYLOff",   &_gauge.gGaugeParam.accYLOff,      -100000.0f, 100000.0f, 8);
    /* 5*/gp->insert("accYLSens",  &_gauge.gGaugeParam.accYLSens,  -100000.0f, 100000.0f, 8);
    /* 6*/gp->insert("accYLSensT", &_gauge.gGaugeParam.accYLSensT, -100.0f,    100.0f, 8);
 
    /* 7*/gp->insert("accZLOff",   &_gauge.gGaugeParam.accZLOff,      -100000.0f, 100000.0f, 8);
    /* 8*/gp->insert("accZLSens",  &_gauge.gGaugeParam.accZLSens,  -100000.0f, 100000.0f, 8);
    /* 9*/gp->insert("accZLSensT", &_gauge.gGaugeParam.accZLSensT, -100.0f,    100.0f, 8);
 
    /*10*/gp->insert("accXHOff",   &_gauge.gGaugeParam.accXHOff,      -100000.0f, 100000.0f, 6);
    /*11*/gp->insert("accXHSens",  &_gauge.gGaugeParam.accXHSens,  -100000.0f, 100000.0f);
    /*12*/gp->insert("accXHSensT", &_gauge.gGaugeParam.accXHSensT, -100.0f,    100.0f, 8);
 
    /*13*/gp->insert("accYHOff",   &_gauge.gGaugeParam.accYHOff,      -100000.0f, 100000.0f, 6);
    /*14*/gp->insert("accYHSens",  &_gauge.gGaugeParam.accYHSens,  -100000.0f, 100000.0f);
    /*15*/gp->insert("accYHSensT", &_gauge.gGaugeParam.accYHSensT, -100.0f,    100.0f, 8);
 
    /*16*/gp->insert("accZHOff",   &_gauge.gGaugeParam.accZHOff,      -100000.0f, 100000.0f, 6);
    /*17*/gp->insert("accZHSens",  &_gauge.gGaugeParam.accZHSens,  -100000.0f, 100000.0f);
    /*18*/gp->insert("accZHSensT", &_gauge.gGaugeParam.accZHSensT, -100.0f,    100.0f, 8);
 
    //  gyro
    /*19*/gp->insert("gyroXLOff",   &_gauge.gGaugeParam.gyroXLOff,      -100000.0f, 100000.0f, 8);
    /*20*/gp->insert("gyroXLSens",  &_gauge.gGaugeParam.gyroXLSens,  -100000.0f, 100000.0f, 8);
    /*21*/gp->insert("gyroXLSensT", &_gauge.gGaugeParam.gyroXLSensT, -1000.0f,    1000.0f, 8);
 
    /*22*/gp->insert("gyroYLOff",   &_gauge.gGaugeParam.gyroYLOff,      -100000.0f, 100000.0f, 8);
    /*23*/gp->insert("gyroYLSens",  &_gauge.gGaugeParam.gyroYLSens,  -100000.0f, 100000.0f, 8);
    /*24*/gp->insert("gyroYLSensT", &_gauge.gGaugeParam.gyroYLSensT, -1000.0f,    1000.0f, 8);
 
    /*25*/gp->insert("gyroZLOff",   &_gauge.gGaugeParam.gyroZLOff,      -100000.0f, 100000.0f, 8);
    /*26*/gp->insert("gyroZLSens",  &_gauge.gGaugeParam.gyroZLSens,  -100000.0f, 100000.0f, 8);
    /*27*/gp->insert("gyroZLSensT", &_gauge.gGaugeParam.gyroZLSensT, -1000.0f,    1000.0f, 8);
 
    /*28*/gp->insert("gyroXHOff",   &_gauge.gGaugeParam.gyroXHOff,      -100000.0f, 100000.0f, 6);
    /*29*/gp->insert("gyroXHSens",  &_gauge.gGaugeParam.gyroXHSens,  -100000.0f, 100000.0f);
    /*30*/gp->insert("gyroXHSensT", &_gauge.gGaugeParam.gyroXHSensT, -1000.0f,    1000.0f, 8);
 
    /*31*/gp->insert("gyroYHOff",   &_gauge.gGaugeParam.gyroYHOff,      -100000.0f, 100000.0f, 6);
    /*32*/gp->insert("gyroYHSens",  &_gauge.gGaugeParam.gyroYHSens,  -100000.0f, 100000.0f);
    /*33*/gp->insert("gyroYHSensT", &_gauge.gGaugeParam.gyroYHSensT, -1000.0f,    1000.0f, 8);
 
    /*34*/gp->insert("gyroZHOff",   &_gauge.gGaugeParam.gyroZHOff,      -100000.0f, 100000.0f, 6);
    /*35*/gp->insert("gyroZHSens",  &_gauge.gGaugeParam.gyroZHSens,  -100000.0f, 100000.0f);
    /*36*/gp->insert("gyroZHSensT", &_gauge.gGaugeParam.gyroZHSensT, -1000.0f,    1000.0f, 8);

	//swap accelerometer
	/*37*/gp->insert("acclXIndex",        &_gauge.gGaugeParam.acclXIndex,     1, 3);
	/*38*/gp->insert("acclYIndex",        &_gauge.gGaugeParam.acclYIndex,     1, 3);
	/*39*/gp->insert("acclZIndex",        &_gauge.gGaugeParam.acclZIndex,     1, 3);
	/*49*/gp->insert("acclXFactor",       &_gauge.gGaugeParam.acclXFactor,    -1, 1);
	/*41*/gp->insert("acclYFactor",       &_gauge.gGaugeParam.acclYFactor,    -1, 1);
	/*42*/gp->insert("acclZFactor",       &_gauge.gGaugeParam.acclZFactor,    -1, 1);

	//swap gyroscope
	/*43*/gp->insert("gyroXIndex",        &_gauge.gGaugeParam.gyroXIndex,     1, 3);
	/*44*/gp->insert("gyroYIndex",        &_gauge.gGaugeParam.gyroYIndex,     1, 3);
	/*45*/gp->insert("gyroZIndex",        &_gauge.gGaugeParam.gyroZIndex,     1, 3);
	/*46*/gp->insert("gyroXFactor",       &_gauge.gGaugeParam.gyroXFactor,    -1, 1);
	/*47*/gp->insert("gyroYFactor",       &_gauge.gGaugeParam.gyroYFactor,    -1, 1);
	/*48*/gp->insert("gryoZFactor",       &_gauge.gGaugeParam.gyroZFactor,    -1, 1);

	// Calibration parameters for magnetometers
	/*49*/gp->insert("magnetXOff",   &_gauge.gGaugeParam.magnetX0ff, -100000.0f, 100000.0f);
	/*50*/gp->insert("magnetYOff",   &_gauge.gGaugeParam.magnetY0ff, -100000.0f, 100000.0f);
	/*51*/gp->insert("magnetZOff",   &_gauge.gGaugeParam.magnetZ0ff, -100000.0f, 100000.0f);
	/*52*/gp->insert("magnetR11",   &_gauge.gGaugeParam.magnetR11, -10000.0f, 10000.0f,19);
	/*53*/gp->insert("magnetR12",   &_gauge.gGaugeParam.magnetR12, -10000.0f, 10000.0f,19);
	/*54*/gp->insert("magnetR13",   &_gauge.gGaugeParam.magnetR13, -10000.0f, 10000.0f,19);
	/*55*/gp->insert("magnetR21",   &_gauge.gGaugeParam.magnetR21, -10000.0f, 10000.0f,19);
	/*56*/gp->insert("magnetR22",   &_gauge.gGaugeParam.magnetR22, -10000.0f, 10000.0f,19);
	/*57*/gp->insert("magnetR23",   &_gauge.gGaugeParam.magnetR23, -10000.0f, 10000.0f,19);
	/*58*/gp->insert("magnetR31",   &_gauge.gGaugeParam.magnetR31, -10000.0f, 10000.0f,19);
	/*59*/gp->insert("magnetR32",   &_gauge.gGaugeParam.magnetR32, -10000.0f, 10000.0f,19);
	/*60*/gp->insert("magnetR33",   &_gauge.gGaugeParam.magnetR33, -10000.0f, 10000.0f,19);

	/*61*/gp->insert("magnetTheta", &_gauge.gGaugeParam.magnetTheta, -1000.0f, 1000.0f);
	/*62*/gp->insert("magnetPhi",   &_gauge.gGaugeParam.magnetPhi, -1000.0f, 1000.0f);
	/*63*/gp->insert("magnetPsi",   &_gauge.gGaugeParam.magnetPsi, -1000.0f, 1000.0f);

	/*64*/gp->insert("voltageOffset",   &_gauge.gGaugeParam.voltageOffset, -100.0f, 100.0f);
	/*65*/gp->insert("currentOffset",   &_gauge.gGaugeParam.currentOffset, -200.0f, 200.0f);
	/*66*/gp->insert("currentSens",   &_gauge.gGaugeParam.currentSens, -1.0f, 1.0f, 19);

	/*67*/gp->insert("diffPressOffset",   &_gauge.gGaugeParam.diffPressOffset, -15.0f, 15.0f);
	
	/*68*/gp->insert("magEnable",   &_gauge.gGaugeParam.magEnable, 0, 5);
	/*69*/gp->insert("gyroTemperatureEnable",   &_gauge.gGaugeParam.gyroTemperatureEnable, 0, 1);
    /*70*/gp->insert("filterAcclCoffA",     &_gauge.gGaugeParam.filterAcclCoff_a, -15.0f, 15.0f, 6);
    /*71*/gp->insert("filterAcclCoffB",     &_gauge.gGaugeParam.filterAcclCoff_b, -15.0f, 15.0f, 6);
    /*72*/gp->insert("filterAcclCoffC",	   &_gauge.gGaugeParam.filterAcclCoff_c, -15.0f, 15.0f, 6);
    /*73*/gp->insert("filterAcclCoffD",	   &_gauge.gGaugeParam.filterAcclCoff_d, -15.0f, 15.0f, 6);
    /*74*/gp->insert("filterAcclCoffE",	   &_gauge.gGaugeParam.filterAcclCoff_e, -15.0f, 15.0f, 6);
	/*75*/gp->insert("filterActive",      &_gauge.gGaugeParam.filterActive,         0, 0xFFFF);

	//swap magneto
	/*76*/gp->insert("magneXIndex",        &_gauge.gGaugeParam.magneXIndex,     1, 3);
	/*77*/gp->insert("magneYIndex",        &_gauge.gGaugeParam.magneYIndex,     1, 3);
	/*78*/gp->insert("magneZIndex",        &_gauge.gGaugeParam.magneZIndex,     1, 3);
	/*79*/gp->insert("magneXFactor",       &_gauge.gGaugeParam.magneXFactor,    -1, 1);
	/*80*/gp->insert("magneYFactor",       &_gauge.gGaugeParam.magneYFactor,    -1, 1);
	/*81*/gp->insert("magneZFactor",       &_gauge.gGaugeParam.magneZFactor,    -1, 1);
    
    // Configuration variable
    /*77*/gp->insert( "config", &_gauge.gGaugeParam.config, 0, 0xFFFF);

	Log.bootPrint ("OK" CRLF);
}

/** \name Method gets PhysicalState subsystem data by copying a content of a PStateData to 'psd'.
*/
bool PhysicalState::getPStateData (PStateData &psd)
{
    if (!_vSem.lock ())
        return false;

    // Copying of a data
    psd = _psd;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

/** \name Method copies the data about subsystem condition to an object 'psh' passed as an argument. The object is a reference to a PStateHealth.
*/
bool PhysicalState::getPStateHealth (PStateHealth &psh)
{
    if (!_vSem.lock ())
        return false;

    // Copying of a data
    psh = _psh;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

/** \name Method copies wind data to an object 'w' of type reference to an WindData
*/
bool PhysicalState::getWindData (WindData &w)
{
    if (!_vSem.lock ())
        return false;

    w = _psd.wind;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

/** \name Method copies data from auxiliary sensors to the object 'asd' of type reference to a AuxSensorsData
*/
bool  PhysicalState::getAuxSensorData (AuxSensorData &asd)
{
    if (!_vSem.lock ())
        return false;
    
    asd = _auxSensorData;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

/** \name Method returns value of a simulation level
*/
int PhysicalState::getSimLevel (void) const
{
	// Code is unsecured by semaphore because of execution in a single code instruction
    return _conf.simLevel;
}

#if EMRG_LOSS_ENGINE == 1
/** \name Method returns value of a Test Engine Fail
*/
bool PhysicalState::getTestLostEngine (void) const
{
	// Code is unsecured by semaphore because of execution in a single code instruction
    return _bTestLostEngine;
}
#endif

/** \name Methos returns time since the start of the system [*100us]
*/
int PhysicalState::getTime100 (void) const
{
    // Code is unsecured by semaphore because of execution in a single code instruction
    return _stateMem.time100;
}

/** \name Method sets current altitude value as a AGL reference. Method is called synchronously by othes
* subsystem component so it cannot be driven by notifications (because execution wouldn't be guaranteed)
* \param If 'useTakeOffAgl' flag is set to 'true' then the calculations of an altitude will take into account the altitude of a hand-start
*/
bool PhysicalState::setAglZero (bool pUseTakeoffAgl)
{
    bool ret = false;

    if (!_vSem.lock ())
        return false;

    ret = setAglZeroInternal ();

	// Reset the reference pressure of a base station to force the setting of a new value if available.
	// Zero (0) value disables the mechanism of correction until 'refBasePressure' will be set to some significiant value.
    _stateMem.refBasePressure = 0.0f;

    // Storing of a current pressure value from a base station
    if (ret && _conf.useBasePressure)
    {
        setRefBasePressure ();
    }

    if (!_vSem.unlock ())
        return false;

    _stateMem.useTakeoffAgl = (ret & pUseTakeoffAgl);

    return ret;
}

/** \name Method sets offset value adding to currently calculated AGL [m]
*
*/
void PhysicalState::setAglOffset (float offset)
{
	// Code is unsecured by semaphore because of execution in a single code instruction
    if (_stateMem.offsetAgl != offset)
        Log.msgPrintf ("%sChange AGL reference from: %.1f to: %.1f", SUBS_PREFIX, _stateMem.offsetAgl, offset);
    _stateMem.offsetAgl = offset;
}

/** \name Method returns value of current offset value added to AGL [m]
*/
float PhysicalState::getAglOffset (void) const
{
    return _stateMem.offsetAgl;
}

/** \name Method returns the number of subsystem notifications per second
*/
int  PhysicalState::getNotifyCyclesPerSecond(void) const
{
    return GAUGE_CYCLES_PER_SECOND / _conf.notifyDivisor;
}

/** \name Method returns 'true' if subsystem is ready for take-off (system is fully functional).
* Method sends error messages to the communication channel 'cl' if it is not null.
* \note Method is executed in the context of different subsystem, pay attention while concurency programming.
*/
bool PhysicalState::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    bool bok = true;

    //  Check the configuration is loaded from flash
    if (!_isConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_CONFIG);
        bok = false;
    }

	// Check the gauges configurations are loaded from flash
    if (!_isGaugeConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_GAUGE_CONFIG);
        bok = false;
    }

	// Check the GPS accuracy
    if (_psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_BAD_PRECISION ||
        _psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_2D_MODE)
    {
        Log.msgPrintf (cl, fLogComm, MSG_GPS_BAD_PRECISION);
        bok = false;
    }

	// Check other GPS errors
    else if (_psh.gpsError != PStateHealth::gpsErrorCode::ERR_GPS_OK)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_GPS);
        bok = false;
    }

	// Check the lock of a static pressure sensor
    if (_psh.sensorsError == PStateHealth::sensorsErrorCode::ERR_SENSORS_SPRESS_LOCKED)
    {
        Log.msgPrintf (cl, fLogComm, MSG_SENSORS_SPRESS_LOCKED_ERROR);
        bok = false;
    }

	// Check errors from the sensors
    else if (_psh.sensorsError != PStateHealth::sensorsErrorCode::ERR_SENSORS_OK)
    {
        Log.msgPrintf (cl, fLogComm, MSG_SENSORS_ERROR);
        bok = false;
    }

	// Check the magnetometer
    if (!_isGaugeMagnetometerOK)
    {
        Log.msgPrintf (cl, fLogComm, MSG_MAG_ERROR);
        bok = false;
    }

	// Check the initilization of inclination and declination (in simulation mode it is not necessary)
    if (_conf.simLevel == 0 &&
        (_stateMem.inclination >= INCLINATION_NOT_SET ||
        _stateMem.declination >= DECLINATION_NOT_SET))
    {
        Log.msgPrintf (cl, fLogComm, MSG_INCLDECL_NOT_SET);
        bok = false;
    }

    return bok;
}

/** \name Method adds new command line to the commands queue and sends internal notification.
* \return 'true' if adding of a new command was successful, 'false' if queue was full or due to some errors
*/
bool PhysicalState::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("PhysicalState_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }

  	// Send notification about adding command to the queue
	// NOTE: Method 'putLine' is executed by other subsystem component, but notification goes from PhysicalState subsystem.
	// Observer treats it as PhysicalState notification.
    notify (EXT_CMD);

    return ret;
}

/** \name Method switches usage of an 'ElevationOffset' parameter ('useElvOffsetPar')
* \param If 'true' enables parameter, 'false' disables it.
*/
void  PhysicalState::useElevationOffsetPar (bool use)
{
    if (_stateMem.useElvOffsetPar && !use)
        Log.msgPrintf ("%sIgnore elevationOffset", SUBS_PREFIX);
    else if (!_stateMem.useElvOffsetPar && use)
        Log.msgPrintf ("%sUse elevationOffset", SUBS_PREFIX);

    _stateMem.useElvOffsetPar = use;
}

/** \name Method returns status of 
* \return 'true' if parameter is enabled, 'false' otherwise.
*/
bool  PhysicalState::isElevationOffsetOn (void) const
{
    return _stateMem.useElvOffsetPar;
}

/** \name  Method returns value of an offset for elevation ('ElevationOffset').
*/
float PhysicalState::getElevationOffset (void) const
{
    return _stateMem.elevationOffset;
}

/** \name Method sets appropriate working mode when an incorrect speed reading occurs
*/
void PhysicalState::setIasFailMode (bool fail)
{
    _stateMem.iasFailMode = fail;
}

/** \name Method sets current altitude as a referecne AGL.
* \note Using semaphore isn't necessary because method is executed from code section which is already protected by semaphore.
*/
bool PhysicalState::setAglZeroInternal (void)
{
    bool ret = false;

    if (_psh.sensorsError != PStateHealth::sensorsErrorCode::ERR_SENSORS_OK)
    {
        Log.msgPrintf ("%sElevation setting error: sensorsError = %d", SUBS_PREFIX, _psh.sensorsError);
        return false;
    }

    _stateMem.elevationPressure = _psd.staticPressure;

    if (_conf.simLevel > 0)
    {
		// AGL from simulator
        _stateMem.elevationAmslSim = _sim.amsl;
    }
    else
    {
        _stateMem.elevationAmslSim = 0.0f;
    }

    _stateMem.elevationIsSet = true;
    ret = true;

    Log.msgPrintf ("%sSet elevation pressure: %.3f hPa", SUBS_PREFIX, _stateMem.elevationPressure);

    return ret;
}

/** \name Method registers objects for observation.
*/
void PhysicalState::linkObserver()
{
	// Self registration which allows to insert command lines (putLine(..)) into the command queue.
	// Method 'putLine' is executed with the context of different subsystem but it is assumed by the observer as it is a FlightPlanRealizer method.
    _extCmdTag = registerSubj (this, EXT_CMD);
	// Registration of LDisp subsystem which sends simulators data
    _simTag = registerSubj (LDisp, SIM_LINE);
	// Registration of LDisp subsystem passing frames of NMEA protocol from primary GPS
    _nmeaTag = registerSubj (LDisp, NMEA_LINE);
    // Registration of LDisp subsystem passing frames of NMEA protocol from auxiliary GPS
    _auxNmeaTag = registerSubj (LDisp, AUX_NMEA_LINE);    
	// Self registration - notifications are send by the 'notify' method of this object called as a response to an FLYEYE_ADC_IRQ interruption (IRQ.C).
    _irqAdcTag = registerSubj (this, IRQ_ADC);
    // Registration of FPReal subsystem sending notifications about the requirements for measurements
    _smoothOnTag = registerSubj (FPReal, FPR_SMOOTH_ON);
    _smoothOffTag = registerSubj (FPReal, FPR_SMOOTH_OFF);
	// Registration of FPReal subsystem which sends launched notification (the aircraft was released).
	_fPRealLaunchTag = registerSubj(FPReal, FPR_LAUNCHED);
    // FPReal set to standby mode
    _fPRealStandbyTag = registerSubj(FPReal, FPR_STANDBY);
	// Registration of SysMon subsystem which sends notification of finalize the landing procedure.
    _sysMonTag = registerSubj (SysMon, SYSMON_LANDED);
#if MAGNETOMETER_TYPE == USE_HMR
    if(Hmr != NULL)
//    	_hmrTag = registerSubj (LDisp, HMR_LINE);
    	_hmrTag = registerSubj (Hmr, HMR_LINE);
#endif
    // Registration of a RS485 channel
    if (CTty485 != NULL)
        _cTty485Tag = registerSubj (CTty485, CH_DATA_RECEIVED);
#if USE_CAM == CAM_ENABLE
    _cameraTag = registerSubj(TtyCAM, IRQ_CAM_RECEIVED);
#endif

}

/** \name Method performs specific MicroC/OS-II operating system task.
*/
void PhysicalState::task(const void* pdata)
{
	// Loading of a subsystem configuration data. Possible errors are printed out by 'confLoad' function.
	// If error occures function 'confLoad' restores default values.
    confLoad();
    gaugeConfLoad();
#if PILOT_TARGET == PT_HARDWARE
    _gauge.initImu();
    _gauge.initBarometer();
    _gauge.initDiffPressure();

    if (Gps != NULL)
    {
        // GPS reset (restoring default values)
        Ublox::pilotReset(Gps);
        // Turning on the GPS channel
        Gps->enable (true);
    }
#if MAGNETOMETER_TYPE == USE_HMR
    if(Hmr != NULL)
    {
    	if(!Hmr2300::HmrInit(Hmr))
    	{
    		Log.errorPrintf("Init HMR failed");
    	}
    }
#else
    _gauge.initMMC();
#endif
	// Resetting all of waiting notifications (GPS was sending data before reset)
    resetAllAspects ();

    //init xADC
    _isConfigureXAdc = xAdcPSInit();
    //init Altimeter
#endif
    for (;;)
    {

        // Waiting for a notification
        OSBase::EvtMask f = waitForAnyAspect ();
        // Line of data from simulator
        if (checkAspect (f, _simTag))
        {
            ClassifiedLine simCl;
            if (LDisp->getSimLine (simCl))
            {
                useSimLine(simCl);
            }
        }
#if MAGNETOMETER_TYPE == USE_HMR
        if (checkAspect (f, _hmrTag))
        {
            ClassifiedLine hmrCl;
            if (LDisp->getHmrLine (hmrCl))
            {
            	useHmrData(hmrCl);
            }
        }
#endif

        // Line of NMEA data from GPS
        if (checkAspect (f, _nmeaTag))
        {
			// Parsing of a NMEA command
            ClassifiedLine nmeaCl;
            if (LDisp->getNmeaLine (nmeaCl))
            {
                useNmeaLine(nmeaCl);
            }

            if (!_gpsConfigured)
            {
				// GPS configuration is critical for system operation (reset is performed when error occurs).
				// The configuration of GPS is done here because it is sure that GPS works correctly.
				// Initialization is performed after receiving ten lines of data after reseting of the GPS (before ten lines it loses command).
                if (_gpsBeforeInitLines++ >= 10)
                {
                    if (!Ublox::pilotInit(Gps, true))
                    {
                        Log.abort ("Critical Error: PhysicalState_task_1 [ublox].");
                        return;
                    }
                    _gpsConfigured = true;
                }
            }

        }
        // Line of NMEA data from auxiliary GPS
        if (checkAspect (f, _auxNmeaTag))
        {
            // Parsing of a NMEA command
            ClassifiedLine auxNmeaCl;
            if (LDisp->getAuxNmeaLine (auxNmeaCl))
            {
                useAuxNmeaLine(auxNmeaCl);
            }
        }

		// Putting of a command form communication channel into the command queue
        if (checkAspect (f, _extCmdTag))
        {
            ClassifiedLine cmdCl;

            while (_cmdq.cmdGet (cmdCl))
            {
                useCmdLine (cmdCl);
            }
        }

		// Interruption from ADC converters
        if (checkAspect (f, _irqAdcTag))
        {
			// Number of ticks required to time estimation (sample timestamps)
            _stateMem.gaugeTicks++;
            // Reading data form sensors
            _gauge.readData();
//            // Converting to standard units
            _gauge.calibrateData();
//
            useGaugeData();
#if PILOT_TARGET == PT_HARDWARE

            if(_isConfigureXAdc)
            {
            	if(++xadcReadingDivisor % 200 == 0)     //read cpu Temperature, frequency = 2 seconds
            	{
            		xadcReadingDivisor = 0;
            		usexadcpsData();
            	}
            }
#endif

        }

        // Enable request for a smooth data
        if (checkAspect (f, _smoothOnTag))
        {
			// Turning on correction for Ahrs
            Log.msgPrintf ("%sAHRS airspeed condition off.", SUBS_PREFIX);
            _stateMem.ahrsObject.lockUpdate (AHRS_LOCK_TIME);
        }

        // Disable request for a smooth data
		// This request is also disbaled by timeout but usually later
        if (checkAspect (f, _smoothOffTag))
        {
            if (_stateMem.ahrsObject.isUpdateLocked())
            {
                Log.msgPrintf ("%sAHRS airspeed condition on.", SUBS_PREFIX);
                _stateMem.ahrsObject.unlockUpdate ();
            }
        }

        // Take-off commencement notification (plane starts moving, acceleration was detected)
        if (checkAspect (f, _fPRealLaunchTag))
        {
            useLaunchTrig ();
        }

        // Finalization of landing notification
        if (checkAspect (f, _sysMonTag))
        {
            Log.msgPrintf ("%sDeclination: %f, Inclination: %f", SUBS_PREFIX, _stateMem.declination, _stateMem.inclination);
			// Ahrs2 is set to 'Earth' mode (just in case it is not known whether the notification to set the autopilot in 'Standby' mode already occurred)
            //_stateMem.ahrs2.setEarthMode (true);
			// Recalibration of a Ahrs2 for diagnostic purposes (to compare bias at flight to the bias when aircraft was on ground )
            //_stateMem.ahrs2.calibrate ();
        }

		// Notification of setting FPRealizer to standby mode.
        if (checkAspect (f, _fPRealStandbyTag))
        {
            // Setting Ahrs2 to Earth mode and turning on accelerometers and magnetometer (if they were disabled)
            //_stateMem.ahrs2.setAccMagOff (false);
            //_stateMem.ahrs2.setEarthMode (true);
        }

		// Notification of receiving RS485 data
        if (checkAspect (f, _cTty485Tag))
        {
            use485Data ();
        }
#if USE_CAM == CAM_ENABLE
        if(checkAspect(f, _cameraTag))
        {
        	useCameraData();
        }
#endif
    }
}

/** \name Method parses and executes 'cl' commands from communication channel
*/
void PhysicalState::useCmdLine(ClassifiedLine &cl)
{
    char buf[LINESIZE] = {0};

	if (!_parser.loadLine(cl.getLine()))
    {
        Log.errorPrintf("PhysicalState_useCmdLine_1");
        cl.answer(ERR_SYS);
        return;
    }

	// Parsing of a 'ps load gauge config' command
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "load") == 0 &&
        STRICMP (_parser.getToken(2), "gauge") == 0 &&
        STRICMP (_parser.getToken(3), "config") == 0)
    {
        if (gaugeConfLoad())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_GCLOAD);
	
        return;
    }

	// Parsing of a 'ps gauge recalibration' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "gauge") == 0 &&
        STRICMP (_parser.getToken(2), "recalibration") == 0)
        
    {
        if (_gauge.startRecalibration())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_UCOMMAND);
				
        return;
    }

	// Parsing of a 'ps gauge recalibration status' command
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "gauge") == 0 &&
        STRICMP (_parser.getToken(2), "recalibration") == 0 &&
        STRICMP (_parser.getToken(3), "status") == 0)
    {
		SNPRINTF(buf, sizeof(buf), "%srecalibration status = %d", SUBS_PREFIX, _gauge.recalibrationStatus);
        buf[LINESIZE-1] = 0;    // Ensure the '0' ends the buffer
	    cl.answer(buf, true, false);
		cl.answer(ERR_OK);
        return;
    }

	// Parsing of a 'ps gauge st1 <val>' command
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "gauge") == 0 &&
        STRICMP (_parser.getToken(2), "st1") == 0 )
    {
		_gauge.setST1(_parser.getTokenAsInt(3));
        cl.answer(ERR_OK);
		
        return;
    }

	// Parsing of a 'ps save config' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "save") == 0 &&
        STRICMP (_parser.getToken(2), "config") == 0)
    {
        if (confSave())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_CSAVE);
        return;
    }

	// Parsing of a 'ps save gauge config' command
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "save") == 0 &&
        STRICMP (_parser.getToken(2), "gauge") == 0 &&
        STRICMP (_parser.getToken(3), "config") == 0)
    {
        if (gaugeConfSave())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_GCSAVE);
        return;
    }

	// Parsing of a 'ps save gauge magnetometer' command
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "save") == 0 &&
        STRICMP (_parser.getToken(2), "gauge") == 0 &&
        STRICMP (_parser.getToken(3), "magnetometer") == 0)
    {
        if (_gauge.storeMagnetometer())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_GCSAVE);
        return;
    }

  	// Parsing of a 'ps verify magnetometer' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "verify") == 0 &&
        STRICMP (_parser.getToken(2), "magnetometer") == 0)
    {
        if (_gauge.verifyMagnetometer())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_MAG_VERIFY);
        return;
    }

  	// Parsing of a 'ps diffpressure offset' command
    else if (_parser.count() == 3 &&
        STRICMP (_parser.getToken(1), "diffpressure") == 0 &&
        STRICMP (_parser.getToken(2), "offset") == 0)
    {
        if (_gauge.diffPressureGetOffset())
            cl.answer(ERR_OK);
        else
            cl.answer(MSG_DIFF_PRESSURE_OFFSET);
        return;
    }

	// Parsing of a 'ps set <name> <value>' command
    else if (_parser.count() >= 4 && 
        STRICMP (_parser.getToken(1), "set") == 0)
    {
        ParameterNames::ERRCODE errc = setParameters (cl.getLine() + _parser.getTokenIndex(2));
        if (errc == ParameterNames::ERR_OK)
            cl.answer(ERR_OK);
        else if (errc == ParameterNames::ERR_READONLY)
            cl.answer(ERR_READONLY);
        else
            cl.answer(ERR_BADPAR);
        return;
    }
    
	// Parsing of a 'ps get <name>' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "get") == 0)
    {
		// If OK then 'getParameter' returns one or many values ('ps: ok' is not necessary)
        if (getParameter (_parser.getToken(2), cl))
            cl.answer(ERR_OK, false, true);
        else
            cl.answer(ERR_BADPAR);
        return;
    }

	// Parsing of a 'ps rtk <string>' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "rtk") == 0)
    {
        // RF: no response
        decodeRtkPart (_parser.getToken(2));
        return;
	}

#if EMRG_LOSS_ENGINE == 1
	// Parsing the 'ps test engine fail' command
    else if (_parser.count() == 4 &&
        STRICMP (_parser.getToken(1), "test") == 0 &&
        STRICMP (_parser.getToken(2), "engine") == 0 &&
		STRICMP (_parser.getToken(3), "fail") == 0)
    {
		_bTestLostEngine = !_bTestLostEngine;
		Log.errorPrintf("Test Engine Fail: %d", _bTestLostEngine);
		cl.answer(ERR_OK);
		return;
	}
#endif
    
#if USE_LEAFLETS == ENABLED
	// Parsing the 'truyendon on' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "bomb") == 0 &&
        STRICMP (_parser.getToken(2), "on") == 0)
    {
		loaddropon();
		Log.msgPrintf("Truyen don ON");
        cl.answer(ERR_OK);
        return;
    } 
	// Parsing the 'truyendon off' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "bomb") == 0 &&
        STRICMP (_parser.getToken(2), "off") == 0)
    {
		loaddropoff();
		Log.msgPrintf("Truyen don OFF");
        cl.answer(ERR_OK);
        return;
    } 
#endif
	// Parsing of a 'ps heater <string> <string>' command
    else if (_parser.count() >= 3 && 
        STRICMP (_parser.getToken(1), "heater") == 0)
    {
        if (STRICMP (_parser.getToken(2), "auto") == 0)
		{
			heater485requests (0x00);
		}
		else if ((STRICMP (_parser.getToken(2), "manual") == 0) && (STRICMP (_parser.getToken(3), "on") == 0))
		{
			heater485requests (0x10);
		}
		else if ((STRICMP (_parser.getToken(2), "manual") == 0) && (STRICMP (_parser.getToken(3), "off") == 0))
		{
			heater485requests (0x11);
		}
		cl.answer(ERR_OK);
        return;
	}

    cl.answer(ERR_UCOMMAND);
}

/** \name Method interprets a line of data from simulator
*/
void PhysicalState::useSimLine(const ClassifiedLine &cl)
{
	// 'lastTime' is set to '-1.0' to not omit first sample after setting of an initial condition.
	// This first sample is sent continuously by FG while FG is paused by SimCon and has 0 time value.
    static float lastTime = -1.0f;  

    bool windUpdated = false;

    // Just in case the data line from simulator is ignored when simulation is running
    if (_conf.simLevel < 1)
        return;

	// Check for conversion error of data line from simulator
    if (!_sim.scanLine (cl.getLine()))
        return;


    // Time [*100us]
    int t100 = static_cast<int>(_sim.time * 10000.0f);

    // Disabling the GPS fail test after specified timeout
    gpsFailTestCheckTout ();

    if (!_vSem.lock ())
        return;

	// Resetting the error flag
    _psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_NO_VALID_DATA;
#if USE_DGPS == ENABLED
	_psh.dgpsError = PStateHealth::ERR_DGPS_NO_VALID_DATA;
#endif
    _psh.sensorsError = PStateHealth::sensorsErrorCode::ERR_SENSORS_OK;
    _isGaugeMagnetometerOK = true;

    //  Data from the simulator
    _stateMem.time100 = t100;
    _psd.time100 = t100;
    _psd.airspeed = _sim.airspeed;
    _psd.R = _sim.R;
    _psd.Q = _sim.Q;
    _psd.P = _sim.P;
    _psd.accZ = _sim.accZ;
    _psd.accY = _sim.accY;
    _psd.accX = _sim.accX;
	
    // Simulation of a GPS fail - some records aren't set just like in the case of real GPS fail
	if(_conf.gpsTestJammingEnable)
	{
		float gpsTrack = fmodf(_psd.track + 2.0f * _psd.airspeed, 360.0f);
		float gpsDistance = fmodf(gpsTrack * 30.0f + 150.0f, 5000.0f);
		GpsPosition::movePosition(_sim.position, gpsTrack, gpsDistance, _sim.position);
	}

    if (!_gpsFailTest)
    {
		// check GPS jamming
		// Speed and ground track aren't directly sent from simulator
		float simGroundSpeed = sqrtf ((_sim.speedEast * _sim.speedEast) + (_sim.speedNorth * _sim.speedNorth));
		float maxDistance;
		float dDist = (_psd.groundspeed + simGroundSpeed) * KPH_2_MS * (_sim.time - _sim.previousTime);
		maxDistance = dDist + 20.0f * _conf.gpsUereConfig + _conf.gpsPositionErrOffset;
		float currentDist = GpsPosition::distance(_sim.prevPosition, _sim.position);

		if((currentDist > maxDistance) && (_psd.groundspeed > 10.0f))
		{
			_psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_JAMMING_POSITION;
			Log.msgPrintf ("%sGPS Jamming Position at latitude %f longitude %f.", SUBS_PREFIX, _sim.position.getLat(), _sim.position.getLon());
		}
		else
		{
			_psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_OK;
			_psd.position = _sim.position;
        
			_psd.groundspeed = simGroundSpeed;
			_psd.track = atan2f (_sim.speedEast,_sim.speedNorth) * RAD_2_DEG;
		}
        if (_psd.track < 0.0f)
            _psd.track = 360.0f + _psd.track;
        _psd.gpsAmsl = _sim.amsl;

    }

	_sim.previousTime = _sim.time;
	_sim.prevPosition = _sim.position;
    _psd.staticPressure = _sim.staticPressure * INHG_2_HPA;
	//_gauge.gMagnetRawMeasurement.magPressure = 400.0f * _psd.staticPressure;
    _psd.outerAirTemp = 15.0f;

	// Setting of data and UTC time in _psd structure
    _psd.dtUtc.year = 0;
    _psd.dtUtc.month = 1;
    _psd.dtUtc.day = 1;
    _psd.dtUtc.valid = false;
    _psd.dtUtc.hours = 0;
    _psd.dtUtc.mins  = 0;
    _psd.dtUtc.secs  = 0;
    _psd.dtUtc.msecs = 0;

	// Single setting of an elevation after power on if elevation wasn't set previously (after delay)
	// In 'takeoff' procedure elevation is set again.
	// The static pressure must be known.
    if (!_stateMem.elevationIsSet && _elevationDlyCnt-- == 0)
    {
        // NOTE: do not use the 'setElevation' method - it locks the semaphores
        setAglZeroInternal();
    }

    // All data come from the simulator
    if (_conf.simLevel == 4)
    {
        // AGL altitude, 'elevationAmsl' is set to 0
        _psd.altitude = _sim.amsl - _stateMem.elevationAmslSim + _stateMem.offsetAgl;
        if (_stateMem.useElvOffsetPar)
            _psd.altitude -= _stateMem.elevationOffset;
        if (_stateMem.useTakeoffAgl)
            _psd.altitude += _conf.takeoffAgl;
        _psd.psi = _sim.psi;
        _psd.theta = _sim.theta;
        _psd.phi = _sim.phi;
        _psd.psiDot = _sim.psiDot;
        _psd.thetaDot = _sim.thetaDot;
        _psd.phiDot = _sim.phiDot;
        _psd.wind.from = _sim.windFrom;
		_psd.wind.speed = _sim.windSpeed*KNOTS_2_KPH;
        _psd.f32Rpm   = _sim.rpm;
        _psd.f32VertSpeed = _sim.verticalSpeed;
        _psd.velned = Vector3f(_sim.speedNorth * KPH_2_MS, _sim.speedEast * KPH_2_MS, -_sim.verticalSpeed);
        // Executing of a method computed wind. At the simulation level set to 4, wind's data come directly from the simulator,
		// and this execution is only necessary to send notification.
        if (_stateMem.windRes.putData(_psd, _psh) != 0)
        {
            Log.msgPrintf ("%sWind (from FG): %.1f kph, %.1f deg", SUBS_PREFIX, _psd.wind.speed, _psd.wind.from);
            windUpdated = true;
        }
    }

    // All data come from simulator except of wind and altitude which are calculated
    else if (_conf.simLevel == 3)
    {
        // AGL altitude, 'elevationAmsl' is set to 0
        if (_stateMem.elevationIsSet)
        {
            _psd.altitude = processedAltitude();
        }
        else
            _psd.altitude = 0.0f;

        _psd.psi = _sim.psi;
        _psd.theta = _sim.theta;
        _psd.phi = _sim.phi;
        _psd.psiDot = _sim.psiDot;
        _psd.thetaDot = _sim.thetaDot;
        _psd.phiDot = _sim.phiDot;

		_psd.f32Rpm   = _sim.rpm;
        // Calculation of a wind
        int alg = _stateMem.windRes.putData(_psd, _psh);
        if (alg > 0)
        {
            _stateMem.windRes.getWind (_psd.wind.from, _psd.wind.speed);
            Log.msgPrintf ("%sWind: %.1f kph, %.1f deg [%s]", SUBS_PREFIX, _psd.wind.speed, _psd.wind.from, alg==1 ? "a1":"a2");
            windUpdated = true;
        }
    }

	// Simulator is not sendig Euler angles nor their derivatives - they are calculated
    else if (_conf.simLevel == 1)
    {
        _psd.wind.from = _sim.windFrom;
		_psd.wind.speed = _sim.windSpeed*KNOTS_2_KPH;
		_psd.f32Rpm   = _sim.rpm;

        // psd altitude will be calculated from simulator air presssure
        if (_stateMem.elevationIsSet)
        {
            _psd.altitude = processedAltitude();
        }
        else
		{
            _psd.altitude = 0.0f;
		}
        
        // Set up the '_gauge' object with data necessary for calculations
        _gauge.gMeasurement.gyroXL = _gauge.gMeasurement.gyroXH = _psd.P;
        _gauge.gMeasurement.gyroYL = _gauge.gMeasurement.gyroYH = _psd.Q;
        _gauge.gMeasurement.gyroZL = _gauge.gMeasurement.gyroZH = _psd.R;

        _gauge.gMeasurement.accXL = _gauge.gMeasurement.accXH = _psd.accX;
        _gauge.gMeasurement.accYL = _gauge.gMeasurement.accYH = _psd.accY;
        _gauge.gMeasurement.accZL = _gauge.gMeasurement.accZH = _psd.accZ;

		_gauge.gMeasurement.agl = _psd.altitude;

        // Calculation of an Euler angles and their derivatives
        // Binh edit
		_stateMem.ahrsObject.ahrsMainFG(&_gauge.gMeasurement, (_stateMem.iasFailMode ? SENS_FAIL_IAS : _psd.airspeed),
			_psd.time100, _sim.phi, _sim.theta, _sim.psi, 0, _psd.groundspeed, _psd.track);
        // Binh end
        
        // Set the calculated  
        _psd.psi = _stateMem.ahrsObject.gAhrsData.psi;
        _psd.theta = _stateMem.ahrsObject.gAhrsData.theta;
        _psd.phi = _stateMem.ahrsObject.gAhrsData.phi;
        _psd.psiDot = _stateMem.ahrsObject.gAhrsData.psiDot;
        _psd.thetaDot = _stateMem.ahrsObject.gAhrsData.thetaDot;
        _psd.phiDot = _stateMem.ahrsObject.gAhrsData.phiDot;
		_psd.f32Rpm   = _sim.rpm;		        
		_psd.f32VertSpeed = _stateMem.ahrsObject.gAhrsData.f32EstVertSpeed;
		_psd.f32Altitude = _stateMem.ahrsObject.gAhrsData.f32EstAlt;

		if (_conf.bUseKFAlt)
		{
			_psd.altitude = _psd.f32Altitude; 
		}

        // Wind calculation
        int alg = _stateMem.windRes.putData(_psd, _psh);
        if (alg > 0)
        {
            _stateMem.windRes.getWind (_psd.wind.from, _psd.wind.speed);
            Log.msgPrintf ("%sWind: %.1f kph, %.1f deg [%s]", SUBS_PREFIX, _psd.wind.speed, _psd.wind.from, alg==1 ? "a1":"a2");
            windUpdated = true;
        }
    }
#if EMRG_LOSS_ENGINE == 1
	if(_bTestLostEngine)
		_auxSensorData.i_engineSpeed = 0;
	else
		_auxSensorData.i_engineSpeed = static_cast<int>(_sim.rpm);
#else
	_auxSensorData.i_engineSpeed = 3000;
#endif

	//// Test of delay line, RF: 2 sec
	//for (int i=47; i>0; i--)
	//{
	//	delayLine[i] = delayLine[i-1];
	//}
	//delayLine[0] = _psd.airspeed;

	//// Beginning of a NL filter
	//static const float FILTER_S = 0.008f;
	//static const float FILTER_L = 2.0f;
	//	
	//float delta = _psd.airspeed - _previousIas;
	//float filterG = FILTER_S * (1.0f + (delta / FILTER_L) * (delta / FILTER_L));
	//float iasF = _previousIas + delta * filterG;
	//if (fabsf(iasF-_psd.airspeed) < 20.0f)
	//{
 //		_psd.airspeed = iasF;
	//	if (_psd.airspeed < 0.0f) _psd.airspeed = 0.0f;
	//	
	//}
 //   _previousIas = _psd.airspeed;

	//end of test

	if (_conf.filterAirspeedActive)
	{
		if (fabsf(_previousRawIas - _psd.airspeed) > _conf.filterAirspeedThreshold)
		{
			_psd.airspeed = _previousRawIas;
		}
		_previousRawIas = _psd.airspeed;
	}

	if (_conf.filterAirspeedActive)
	{
		float coff[5];
		coff[0] = _conf.filterAirspeedCoff_a;
		coff[1] = _conf.filterAirspeedCoff_b;
		coff[2] = _conf.filterAirspeedCoff_c;
		coff[3] = _conf.filterAirspeedCoff_d;
		coff[4] = _conf.filterAirspeedCoff_e;

		_psd.airspeed = Gauge::chebyshev_lowpass_filter_accl(_psd.airspeed, airspeedDly[0], airspeedDly[1], filterAirspeedDly[0], filterAirspeedDly[1], coff);
	}

	// TAS speed calculation (after the IAS filtration)
    _psd.tas = computeTas (_psd.airspeed, _psd.staticPressure, _psd.outerAirTemp);

	// Estimation of a groungspeed, position, track, gpsAmsl (in the case of GPS failure)
	// '_psd' must holds current data (except data mentioned above which must be latched in the case of a GPS failure)
	// '_psd' must holds current GPS error status
    gpsFailNavigation ();

    if (!_vSem.unlock ())
        return;

    // Sending the telemetry and notifications to other subsystems. Sending the RS485 request.
    nslData();

	sendAuxTlm();

    if (windUpdated)
        notify (PSTATE_WIND);

    lastTime = _sim.time;
}

/** \name Method uses data from sensors and extrapolates data from GPS
*/
void PhysicalState::useGaugeData(void)
{
    // Sensors data are ignored while simulation is running
    if (_conf.simLevel > 0)
        return;

    // Flag determining whether the wind data has been updated in current iteration
    bool windUpdated = false;
	
    // Relative time calculation
    _stateMem.time100 = _stateMem.gaugeTicks * GAUGE_TICK_INTERVAL_100;

	// IAS speed calculation
//    float dp = _gauge.gMagnetMeasurement.diffPress;
    float dp = _gauge.grawDiffPressureMeasurement.Pressure;
    
	// If the magnetometer is present then it must have an differential pressure sensor connected to them.
	// This differential pressure sensor is used to determine the IAS.

    // Optimization
    static const float exp1 = sqrtf (2.0f / RO) * MS_2_KPH;
    float ias = exp1 * sqrtf (dp);

	if (_conf.filterAirspeedActive)
	{
		if (fabsf(_previousRawIas - ias) > _conf.filterAirspeedThreshold)
		{
			ias = _previousRawIas;
		}
		_previousRawIas = ias;
	}

    // Speed filtration
	// Decimation and interpolation with factor set to 12
	if (_conf.asFilter) {
		_iasCounter--;
		if(_iasCounter <= 0)
		{
			_iasCounter = 3;
			_ias_2 = _ias_1;
			_ias_1 = ias;
		}
		// 0.5s delay	
		//ias = _ias_2 + ((_ias_1-_ias_2)/3.0f) * static_cast<float>(3-_iasCounter);
	
		static const float FILTER_S = 0.015f;
		static const float FILTER_L = 10.0f;
		
		float delta = ias - _previousIas;
	    float filterG = FILTER_S * (1.0f + (delta / FILTER_L) * (delta / FILTER_L));
	    if(filterG > 1.0f)
	    	filterG = 1.0f;
	    float iasF = _previousIas + delta * filterG;
	    if (fabsf(iasF-ias) < 20.0f)
	    {
 			ias = iasF;
		}

	    if (ias < 0.0f) ias = 0.0f;
	}



	if (_conf.filterAirspeedActive)
	{
		float coff[5];
		coff[0] = _conf.filterAirspeedCoff_a;
		coff[1] = _conf.filterAirspeedCoff_b;
		coff[2] = _conf.filterAirspeedCoff_c;
		coff[3] = _conf.filterAirspeedCoff_d;
		coff[4] = _conf.filterAirspeedCoff_e;

		ias = Gauge::chebyshev_lowpass_filter_accl(ias, airspeedDly[0], airspeedDly[1], filterAirspeedDly[0], filterAirspeedDly[1], coff);
	}
    _previousIas = ias;

    // Reading the static pressure
    float statPress = getStaticPressure ();

	// Reading the external temperature
//    float oat = _gauge.gMagnetMeasurement.magTemperature;
    float oat = _gauge.grawBaroMeasurement.Temperature;
    
	// TAS speed calculation
    float tas = computeTas (ias, statPress, oat);

	if (_stateMem.elevationIsSet)
		_gauge.gMeasurement.agl = processedAltitude();
    else
	{
        _gauge.gMeasurement.agl = 0.0f;
	}
    // Calculation of an angles

	// An extra parameter has been added : flightMode
	// flightMode == 1 : previous stringent criterion for theta correction
	// flightMode != 1 : less stringent criterion for theta correction
    _stateMem.ahrsObject.ahrsMain(&_gauge.gMeasurement, &_gauge.gMagnetMeasurement,
        (_stateMem.iasFailMode ? SENS_FAIL_IAS : ias), _stateMem.time100, _conf.pressureSource,
		3, 1, _conf.ahrsVersion, _conf.debugMsg, _psd.groundspeed, _psd.track);
    // Binh end


	// Verify proper operation of the magnetometer based on the deviation of the measured vector magnetic
	if(fabsf(_stateMem.ahrsObject.gAhrsData.incdError)<0.5f)
	{
		_isGaugeMagnetometerOK = true;
	}else{
		_isGaugeMagnetometerOK = false;
	}
	
    // Disabling the GPS fail test after specified timeout
    gpsFailTestCheckTout ();

    if (!_vSem.lock ())
        return;

    // Set up the _'psd' and '_psh' structures
    //=======================================================

    // Static pressure
    _psd.staticPressure = statPress;

    // External temperature
    _psd.outerAirTemp = oat;

	// Single setting of an elevation after power on if elevation wasn't set previously (after delay)
	// In 'takeoff' procedure elevation is set again.
	// The static pressure must be known.
    if (!_stateMem.elevationIsSet && _elevationDlyCnt-- == 0)
    {
        // NOTE: do not use the 'setElevation' method - it locks the semaphores
        setAglZeroInternal();
    }

    // Check if static pressure sensor has been locked (only for the HUB sensor)
    if (_conf.pressureSource == 0)
    {
        if ((unsigned int)_auxSensorData.i_pressure == _lastRawMagPress)
        {
            if (_lastRawMagPressCnt++ > 48)
            {
                // If several consecutive samples has identical values, the error flag is set.
                // NOTE: sensor respons frequency is less then system cycle frequency. The remaining samples are copied from the previous reading so
                // the control periad cannot be too short.
                _psh.sensorsError = PStateHealth::sensorsErrorCode::ERR_SENSORS_SPRESS_LOCKED;
                // Setting of a one-time error flag
                if (!_stateMem.spSensorLocked)
                {
                    _stateMem.spSensorLocked = true;
                    Log.msgPrintf ("%sStatic pressure sensor locked - switched to alternate sensor: %s (dP=%.3f)", SUBS_PREFIX, _conf.spSensorSwitchEnable ? "yes":"no", _stateMem.spSensorDelta);
                }
                _lastRawMagPressCnt = 0;
            }
        }
        else
        {
            // Static presure sensor is unlocked - value of a current pressure sample differs from previous one
            _psh.sensorsError = PStateHealth::sensorsErrorCode::ERR_SENSORS_OK;
            _lastRawMagPressCnt = 0;
            // If the error flag is not set, then difference between the wing sensor(1) value and the autopilot sensor(0) value is stored
            if (!_stateMem.spSensorLocked)
            {
                _stateMem.spSensorDelta = getStaticPressureFromSensor() - getStaticPressureFromSensor();    // w hPa
            }
        }
        
        _lastRawMagPress = _auxSensorData.i_pressure;
    }
    else
        // For the pressure sensor from autopilot error is not checked
        _psh.sensorsError = PStateHealth::sensorsErrorCode::ERR_SENSORS_OK;


    _psd.time100 = _stateMem.time100;

    // IAS speed
    _psd.airspeed = _conf.airSpdScaleRatio*ias;
    // TAS speed
    _psd.tas = tas;

	// Calculation of an AGL altitude
    if (_stateMem.elevationIsSet)
        _psd.altitude = processedAltitude();
    else
        _psd.altitude = 0.0f;

	// Gyroscopes (angular velocities)
    if (_gauge.gRawMeasurement.gyroOverflow == 0)
    {
		// Smaller range
        _psd.P = _gauge.gMeasurement.gyroXL;
        _psd.Q = _gauge.gMeasurement.gyroYL;
        _psd.R = _gauge.gMeasurement.gyroZL;
    }
    else
    {
        // Greater range
        _psd.P = _gauge.gMeasurement.gyroXH;
        _psd.Q = _gauge.gMeasurement.gyroYH;
        _psd.R = _gauge.gMeasurement.gyroZH;
    }
    
    // Accelerometers
    if (_gauge.gRawMeasurement.accOverflow == 0)
    {
        _psd.accX = _gauge.gMeasurement.accXL;
        _psd.accY = _gauge.gMeasurement.accYL;
        _psd.accZ = _gauge.gMeasurement.accZH;  // RF: VT1
    }
    else
    {
        _psd.accX = _gauge.gMeasurement.accXH;
        _psd.accY = _gauge.gMeasurement.accYH;
        _psd.accZ = _gauge.gMeasurement.accZH;
    }

    // Setting the date and time based on GPS (GPS does not have to be traced, but its internal timer had to be operated)
    if (!_datetimeSet && _currentGps.datePresent && _currentGps.timePresent)
    {   
        Log.dateTimePrint(_currentGps.year, _currentGps.month, _currentGps.day,
            _currentGps.utcHour, _currentGps.utcMin, _currentGps.utcSec);
        _datetimeSet = true;
    }

    // Setting the UTC time in the '_psd' structure
    _psd.dtUtc.valid = _currentGps.timePresent && _currentGps.datePresent;
    if (_psd.dtUtc.valid)
    {
        _psd.dtUtc.year  = _currentGps.year;
        _psd.dtUtc.month = _currentGps.month;
        _psd.dtUtc.day   = _currentGps.day;
        _psd.dtUtc.hours = _currentGps.utcHour;
        _psd.dtUtc.mins  = _currentGps.utcMin;
        _psd.dtUtc.secs  = _currentGps.utcSec;
        _psd.dtUtc.msecs = _currentGps.utcMsec;
    }

#if USE_DGPS == 1
    _prevPredictedGps = _predictedGps;
#endif
    //  GPS
	checkGpsData(windUpdated);
#if USE_DGPS == 1
	// DGPS data
	if(_conf.dgpsPredictionEnable == 0)
	{
		_psd.dgpsAltitude = _DGpsData->tmpData.measureData.altitude;
		_psd.dgpsLonRange = _DGpsData->tmpData.measureData.lonRange;
		_psd.dgpsTrack    = _DGpsData->tmpData.measureData.track;
		_psd.dgpsEast	  = _DGpsData->tmpData.rawData.east;
		_psd.dgpsNorth	  = _DGpsData->tmpData.rawData.north;
		_psd.dgpsNumSat   = _DGpsData->numSats;
	}
	//  DGPS
    if (static_cast<float>(_psd.time100 - _DGpsData->userTime) <= _conf.dgpsTimeout * 10000.0f && _conf.dgpsPredictionEnable > 0)
	{
		// DGPS data
		if (_DGpsTrimble.parseError)
        {
            _psh.dgpsError = PStateHealth::ERR_DGPS_BAD_OUTPUT;
        }
        
		if (_dgpsGood)
		{                            
            _psh.dgpsError = PStateHealth::ERR_DGPS_OK;

			_psd.dgpsAltitude = _DGpsData->currData.measureData.altitude;
			_psd.dgpsLonRange = _DGpsData->currData.measureData.lonRange;
			_psd.dgpsTrack	  = _DGpsData->currData.measureData.track * RAD_2_DEG;
			_psd.dgpsEast	  = _DGpsData->currData.rawData.east;
			_psd.dgpsNorth	  = _DGpsData->currData.rawData.north;
			_psd.dgpsNumSat   = _DGpsData->numSats;

            _DGpsTrimble.prevBaroAlt = _psd.altitude;
            
            _psd.dgpsAltitudePredict = _psd.dgpsAltitude;
            _psd.dgpsLonRangePredict = _psd.dgpsLonRange;
            _psd.dgpsTrackPredict    = _psd.dgpsTrack;
		}
		else
		{
            _psh.dgpsError = PStateHealth::ERR_DGPS_NO_VALID_DATA;

			_psd.dgpsNumSat   = _DGpsData->numSats;
			_psd.dgpsEast	  = _DGpsData->currData.rawData.east;
			_psd.dgpsNorth	  = _DGpsData->currData.rawData.north;
			// in case of using prediction with DGPS in low frequency
			if (_conf.dgpsPredictionEnable >= 4)
			{                    				
                _DGpsData->prediction(GpsPosition::distance(_prevPredictedGps.position, _predictedGps.position), _psd.track * DEG_2_RAD, _psd.altitude);
            
                _psd.dgpsLonRange = _DGpsData->tmpData.measureData.lonRange;
                _psd.dgpsTrack    = _DGpsData->tmpData.measureData.track * RAD_2_DEG;
                _psd.dgpsAltitude = _DGpsData->tmpData.measureData.altitude;
                
                _psd.dgpsLonRangePredict = _DGpsData->currData.measureData.lonRange;
                _psd.dgpsTrackPredict    = _DGpsData->currData.measureData.track * RAD_2_DEG;
                _psd.dgpsAltitudePredict = _DGpsData->currData.measureData.altitude;
			}
            else if (_conf.dgpsPredictionEnable == 3)
            {                                    
                _DGpsData->prediction(GpsPosition::distance(_prevPredictedGps.position, _predictedGps.position), _psd.track * DEG_2_RAD, _psd.altitude);
            
                _psd.dgpsLonRange = _DGpsData->tmpData.measureData.lonRange;
                _psd.dgpsTrack    = _DGpsData->tmpData.measureData.track * RAD_2_DEG;
                _psd.dgpsAltitude = _DGpsData->tmpData.measureData.altitude;
            }
		}
	}
    else
    {
		// No communication with DGPS
        _psh.dgpsError = PStateHealth::ERR_DGPS_NO_OUTPUT;
    }
#endif
        // Euler angles
        _psd.phi   = _stateMem.ahrsObject.gAhrsData.phi;
        _psd.theta = _stateMem.ahrsObject.gAhrsData.theta;
        _psd.psi   = _stateMem.ahrsObject.gAhrsData.psi;

        // Derivatives of the Euler angles ('Earth' coordinate system)
        _psd.phiDot = _stateMem.ahrsObject.gAhrsData.phiDot;
        _psd.psiDot = _stateMem.ahrsObject.gAhrsData.psiDot;
        _psd.thetaDot = _stateMem.ahrsObject.gAhrsData.thetaDot;

		_psd.f32VertSpeed   = _stateMem.ahrsObject.gAhrsData.f32EstVertSpeed;
		_psd.f32Altitude    = _stateMem.ahrsObject.gAhrsData.f32EstAlt;

		if (_conf.bUseKFAlt)
		{
			_psd.altitude = _psd.f32Altitude;
		}

	// Estimation of a groungspeed, position, track, gpsAmsl (in the case of GPS failure)
	// '_psd' must holds current data (except data mentioned above which must be latched in the case of a GPS failure)
	// '_psd' must holds current GPS error status
    gpsFailNavigation ();

    if (!_vSem.unlock ())
        return;

   // Sending the telemetry and notifications to other subsystems. Sending the RS485 request.
    nslData();

	sendAuxTlm ();

    if (windUpdated)
        notify (PSTATE_WIND);
}

/** \name Method handles the 'take-off' notification
*/
void PhysicalState::useLaunchTrig (void)
{
    // Force telemetry logging if logging was too slow or disabled.
    if (_conf.logDivisor == 0 || _conf.logDivisor > FORCED_LOG_DIVISOR)
    {
        _conf.logDivisor = FORCED_LOG_DIVISOR;
        Log.msgPrintf ("%sForce telemetry logging.", SUBS_PREFIX);
    }
    _conf.logFormat = TLM_PS_LONG;
    _conf.stateLogEnable = true;
}

/** \name Method interprets a line of NMEAs data from GPS subsystem
*/
void PhysicalState::useNmeaLine(const ClassifiedLine &cl)
{
    // GPS data are taken into account when simulation is running
	if ((_conf.simLevel == 0) || (SysMon->getTestHIL()))
    {
        // Sending data to log
        if (_conf.nmeaLogDivisor != 0)
            if (++_nmeaLogCounter % _conf.nmeaLogDivisor == 0)
            {
                Log.tlmPrint (TLM_NMEA, cl.getLine(), true, false);
                _nmeaLogCounter = 0;
            }
        
		// Sending data to communication channel
        if (_conf.nmeaCommDivisor != 0)
            if (++_nmeaCommCounter % _conf.nmeaCommDivisor == 0)
            {
                Log.tlmPrint (TLM_NMEA, cl.getLine(), false, true);
                _nmeaCommCounter = 0;
            }

		// Beginning of NMEA package
        if (Ublox::isPacketBegin(cl.getLine()))
        {
            _tmpGps.resetData();
            _gpsPacketStarted = true;
        }
            
		// Parsing errors are logged by 'parseNmea' method
		// Parsing correction checking and data validation is done by '_currentGps' object
        _tmpGps.parseNmea(cl.getLine());

		// End of NMEA package
        if (Ublox::isPacketEnd(cl.getLine()) && _gpsPacketStarted)
        {
            _previousGps = _currentGps;
            _currentGps = _tmpGps;
            
			// Time of the reading - needed for prediction
            _currentGps.userTime = _stateMem.time100;            
			// Lines of NMEA data will be omitted since receiving beginning line of NMEA package
			// Entire packege needs to be received because possition is at the first line and status at the last one
            _gpsPacketStarted = false;
        }
    }
}

/** \name Method interprets a line of NMEAs data from additional GPS (telemetry only)
*/
void PhysicalState::useAuxNmeaLine(const ClassifiedLine &cl)
{
	if (_conf.simLevel == 0)
    {
		// Sending data to log
        if (_conf.nmeaLogDivisor != 0)
		{
            if (++_nmeaLogCounter % _conf.nmeaLogDivisor == 0)
            {
                Log.tlmPrint (TLM_AUX_NMEA, cl.getLine(), true, false);
                _nmeaLogCounter = 0;
            }
		}

		// Sending data to communication channel
        if (_conf.nmeaCommDivisor != 0)
		{
            if (++_nmeaCommCounter % _conf.nmeaCommDivisor == 0)
            {
                Log.tlmPrint (TLM_AUX_NMEA, cl.getLine(), false, true);
                _nmeaCommCounter = 0;
            }
		}
#if USE_DGPS == ENABLED
        if (_conf.dgpsDevice == DGPS_TRIMBLE)
        {
            if (STRNICMP (cl.getLine(), "$PTNL,VGK", 9) == 0)
            {
                _DGpsTrimble.resetData();
            }
    
    		// Parsing errors are logged by 'parseNmea' method
    		// Parsing correction checking and data validation is done by '_currentGps' object
            _DGpsTrimble.parseNmea(cl.getLine());
    
    		if (!_DGpsTrimble.parseError)
    		{
    			_DGpsTrimble.userTime = _stateMem.time100;
                if (((_DGpsTrimble.vgkMode == Trimble::RTK_FLOAT) ||
                     (_DGpsTrimble.vgkMode == Trimble::RTK_LOCATION)) &&
                     (_DGpsTrimble.PDOP < _conf.maxPdop))
                {
                    _dgpsGood = true;
                    _DGpsTrimble.currData = _DGpsTrimble.tmpData;
                    _DGpsData = &_DGpsTrimble;
                }
                else
                {
                    _dgpsGood = false;
                }
    		}
        }
        else if (_conf.dgpsDevice == DGPS_NOVATEL)
        {
            if (STRNICMP (cl.getLine(), "#ALIGNBSLNENUA", 14) == 0)
            {
                _DGpsNovatel.resetData();
            }
    
            // Parsing errors are logged by 'parseNmea' method
            // Parsing correction checking and data validation is done by '_currentGps' object
            _DGpsNovatel.parseNmea(cl.getLine());
    
            if (!_DGpsNovatel.parseError)
            {
                _DGpsNovatel.userTime = _stateMem.time100;
                if (_DGpsNovatel.posType == Novatel::NARROW_FLOAT)
                {
                    _dgpsGood = true;
                    _DGpsNovatel.currData = _DGpsNovatel.tmpData;
                    _DGpsData = &_DGpsNovatel;
                }
                else
                {
                    _dgpsGood = false;
                }
            }
        }
#endif
	}
}

/** \name Method handles notification of RS485 data ready to read.
* \note Each device sends data once per system cycle, so this method may be call several times per single system cycle.
* \note After reading the data a request of reading data must be send to the next device.
*/
void PhysicalState::use485Data (void)
{
    if (NULL == CTty485)
        return;

    // In the case of a larger number of devices at this point should be send a request of reading the data to the next device.

    ChannelData recvData;
    if (!CTty485->getData (recvData))
    {
        // Error reading of data form communication channel (e.g. semaphore is locked)
        return;
    }

	if (_conf.debugMsg == HUB_FRAME_TYPE)
	{
		_conf.debugMsg = NONE;
		Log.msgPrintf ("frameType: %x, senderNo: %d", recvData.getFrameType(), recvData.getSenderNo());
	}

    if (recvData.getFrameType () != 0x81)
        // Unknown frame type
        return;

	// Reading of device id from which data are read 
    int senderNo = recvData.getSenderNo ();

	// Pointer to the data
    const unsigned char* pData = recvData.getPayload().getData();

    int inter = pData[1];   // Interface
    int cmd = pData[2];     // Command
    int status = pData[3];  // Response status

	if (_conf.debugMsg == HUB_VERIFY_HEADER)
	{
        _conf.debugMsg = NONE;
		Log.msgPrintf ("0: %d, 1: %d, 2: %d, 3: %d", pData[0], pData[1], pData[2], pData[3]);
	}

    if ((4 == senderNo) && (1 == inter) && (0xAA == cmd) && (0 == status))
    {
        INT16U u16_temp = 0;
#if ((PILOT_TYPE == VUA_SC_3TH) || (PILOT_TYPE == VUA_SC) || (PILOT_TYPE == VUA_SL) || (PILOT_TYPE == VUA_SC_6G))       
		INT8U  u8_temp  = 0;		
        INT32U u32_temp = 0;
#endif
		if (_conf.debugMsg == HUB_RAW_BYTES)
		{
            _conf.debugMsg = NONE;
#if PILOT_TYPE == VUA_SC_3TH
			Log.msgPrintf ("4: %d, 5: %d, 6: %d, 7: %d, 8: %d, 9: %d, 10: %d", 
                pData[4], pData[5], pData[6], pData[7], pData[8], pData[9], pData[10]);
#elif(PILOT_TYPE == VUA_SC && ENGINE_TYPE == VUA_E)
			Log.msgPrintf ("4: %d, 5: %d, 6: %d, 7: %d, 8: %d, 9: %d, 10: %d, 11: %d, 12: %d", 
                pData[4], pData[5], pData[6], pData[7], pData[8], pData[9], pData[10], pData[11], pData[12]);
#elif ((PILOT_TYPE == VUA_SC || PILOT_TYPE == VUA_SL || PILOT_TYPE == VUA_SC_6G) && (ENGINE_TYPE == VUA_G))
			Log.msgPrintf ("4: %d, 5: %d, 6: %d, 7: %d, 8: %d, 9: %d, 10: %d, 11: %d, 12: %d, 13: %d, 14: %d, 15: %d, 16: %d, 17: %d, 18: %d", 
				pData[4], pData[5], pData[6], pData[7], pData[8], pData[9], pData[10], pData[11], 
				pData[12], pData[13], pData[14], pData[15], pData[16], pData[17], pData[18]);
#elif (PILOT_TYPE == VUA_SL)
			Log.msgPrintf ("15: %d, 16: %d, 17: %d, 18: %d, 19: %d, 20: %d, 21: %d, 22: %d, 23: %d, 24: %d, 25: %d, 26: %d, 27: %d", 
				pData[15], pData[16], pData[17], pData[18], pData[19], pData[20], pData[21], pData[22], 
				pData[23], pData[24], pData[25], pData[26], pData[27]);
#endif
		}

        if (!_vSem.lock ())
            return;
      
        // Reading engine speed
		u16_temp = ((pData[4] << 8) & 0xFF00) | (pData[5] & 0xFF);
		_auxSensorData.i_engineSpeed = (u16_temp > MAX_ENGINE_SPEED) ? _auxSensorData.i_engineSpeed : (int)u16_temp;
#if PILOT_TYPE == VUA_SC_3TH
		// Reading of a fuel level [percentage]
		u8_temp = pData[6] & 0xFF;
		_auxSensorData.i_fuelLevel = (u8_temp > 100) ? 100 : (int)u8_temp;

		// Reading air humidity [percentage]
		u8_temp = pData[7] & 0xFF;
        _auxSensorData.i_env_rh = (u8_temp > 99) ? 99 : (int)u8_temp;   
        
        // Reading temperature (removing offset)
		u8_temp = pData[8] & 0xFF;
		_auxSensorData.i_env_temp = (int)u8_temp - 40;

		// Reading Votage Battery 1
		u16_temp = pData[9] & 0xFF;
		_auxSensorData.f_voltage_1 = (float)u16_temp / 10.0f;

		// Reading Current Supply
		u8_temp = pData[10] & 0xFF;
		_auxSensorData.f_current = (float)u8_temp / 100.0f;

#elif (PILOT_TYPE == VUA_SC && ENGINE_TYPE == VUA_E)
		// Reading Votage Battery
		u16_temp = ((pData[6] << 8) & 0xFF00) | (pData[7] & 0xFF);
		_auxSensorData.f_voltage_1 = (float)u16_temp / 100.0f;
        
        // Reading Engine current
        u16_temp = ((pData[8] << 8) & 0xFF00) | (pData[9] & 0xFF);
        _auxSensorData.f_current = (float)u16_temp / 100.0f;

		// Reading Engine temperature
		u8_temp = (pData[10] & 0xFF);
		_auxSensorData.i_engine_temp = (int)u8_temp;

		// Reading Environment temperature
		u8_temp = (pData[11] & 0xFF);
		_auxSensorData.i_env_temp = (int)u8_temp - 40;

		// Reading Environment rh
		u8_temp = (pData[12] & 0xFF);
		_auxSensorData.i_env_rh = (int)u8_temp;

#elif ((PILOT_TYPE == VUA_SC || PILOT_TYPE == VUA_SL || PILOT_TYPE == VUA_SC_6G) && (ENGINE_TYPE == VUA_G))
        // Reading of a fuel level [percentage]
		u8_temp = pData[6] & 0xFF;
		_auxSensorData.i_fuelLevel = (u8_temp > 100) ? 100 : (int)u8_temp;
     
        // Reading air humidity [percentage]
		u8_temp = pData[7] & 0xFF;
        _auxSensorData.i_env_rh = (u8_temp > 99) ? 99 : (int)u8_temp;   
        
        // Reading temperature (removing offset)
		u8_temp = pData[8] & 0xFF;
		_auxSensorData.i_env_temp = (int)u8_temp - 40;        

		// Reading Votage Battery 1
		u16_temp = ((pData[9] << 8) & 0xFF00) | (pData[10] & 0xFF);
		_auxSensorData.f_voltage_1 = (float)u16_temp / 100.0f;
        
        // Reading Votage Battery 2
        u16_temp = ((pData[11] << 8) & 0xFF00) | (pData[12] & 0xFF);
        _auxSensorData.f_voltage_2 = (float)u16_temp / 100.0f;

		// Reading Current Supply
		u8_temp = pData[13] & 0xFF;
		_auxSensorData.f_current = (float)u8_temp / 100.0f;

		// Reading Pressure
		u32_temp = ((pData[14] << 16) & 0xFF0000)   |
                   ((pData[15] <<  8) & 0xFF00)     |
                    (pData[16] & 0xFF);
		_auxSensorData.i_pressure = (unsigned int)u32_temp;

		// Reading Status of Hub
		u16_temp = ((pData[17] << 8) & 0xFF00) | (pData[18] & 0xFF);
		_auxSensorData.t_hubStatus.u16_status = u16_temp;

#elif PILOT_TYPE == VUA_SL
		u16_temp = ((pData[15] << 8) & 0xFF00) | (pData[16] & 0xFF);
		_auxSensorData.i_coolingOilTemp = (int)u16_temp - 40;

		u16_temp = ((pData[17] << 8) & 0xFF00) | (pData[18] & 0xFF);
		_auxSensorData.i_coolingWaterTemp = (int)u16_temp - 40;

		u16_temp = ((pData[19] << 8) & 0xFF00) | (pData[20] & 0xFF);
		_auxSensorData.i_cylinderTemp = (int)u16_temp - 40;

		u16_temp = ((pData[21] << 8) & 0xFF00) | (pData[22] & 0xFF);
		_auxSensorData.i_oilPressure = (int)u16_temp;

		u8_temp = pData[23] & 0xFF;
		_auxSensorData.i_orangeLed = (int)u8_temp;

		u8_temp = pData[24] & 0xFF;
		_auxSensorData.i_redLed = (int)u8_temp;

		u16_temp = ((pData[25] << 8) & 0xFF00) | (pData[26] & 0xFF);
		_auxSensorData.i_engineSpeedTCU = (int)u16_temp;

		u8_temp = pData[27] & 0xFF;
		_auxSensorData.i_yellowLed = (int)u8_temp;
#endif

        _vSem.unlock ();

		if (_conf.debugMsg == HUB_PHY_DATA)
		{
			_conf.debugMsg = NONE;
#if PILOT_TYPE == VUA_SC_3TH
			Log.msgPrintf ("EngineSpeed: %d, FuelLevel: %d, RH: %d, Temp: %d, voltage: %.2f, current: %.2f",
				_auxSensorData.i_engineSpeed, _auxSensorData.i_fuelLevel, _auxSensorData.i_env_rh, _auxSensorData.i_env_temp, _auxSensorData.f_voltage_1, _auxSensorData.f_current);
#elif (PILOT_TYPE == VUA_SC && ENGINE_TYPE == VUA_E)
			Log.msgPrintf ("EngineSpeed: %d, RH: %d, Temp: %d, voltage: %.2f, current: %.2f, Engine Temp: %d",
				_auxSensorData.i_engineSpeed,_auxSensorData.i_env_rh, _auxSensorData.i_env_temp, _auxSensorData.f_voltage_1, _auxSensorData.f_current, _auxSensorData.i_engine_temp);
#elif ((PILOT_TYPE == VUA_SC || PILOT_TYPE == VUA_SL || PILOT_TYPE == VUA_SC_6G) && (ENGINE_TYPE == VUA_G))
			Log.msgPrintf ("EngineSpeed: %d, FuelLevel: %d, RH: %d, Temp: %d, voltage_1: %.2f, voltage_2: %.2f, current: %.2f, pressure: %d, deviceStatus: %d",
				_auxSensorData.i_engineSpeed, _auxSensorData.i_fuelLevel, _auxSensorData.i_env_rh, _auxSensorData.i_env_temp, 
				_auxSensorData.f_voltage_1, _auxSensorData.f_voltage_2, _auxSensorData.f_current, _auxSensorData.i_pressure, _auxSensorData.t_hubStatus.u16_status);
#elif PILOT_TYPE == VUA_SL
			Log.msgPrintf ("coolingOilTemp: %d, coolingWaterTemp: %d, cylinderTemp: %d, oilPressure: %d, orangeLed: %d, redLed: %d, engineSpeedTCU: %d, yellowLed: %d",
				_auxSensorData.i_coolingOilTemp, _auxSensorData.i_coolingWaterTemp, _auxSensorData.i_cylinderTemp, _auxSensorData.i_oilPressure, 
				_auxSensorData.i_orangeLed, _auxSensorData.i_redLed, _auxSensorData.i_engineSpeedTCU, _auxSensorData.i_yellowLed);
#endif
		}
    }     
}

#if USE_CAM == CAM_ENABLE
    void PhysicalState::useCameraData(void)
    {
    	if(TtyCAM != NULL)
    	{
    		cam_msg_response_t camData;
    		if(TtyCAM->getObjectTrackInfor(camData))
    		{
    			_psd.cam_mode = camData.mode;
    			_psd.object_px = camData.px;
    			_psd.object_py = camData.py;
    			_psd.object_width = camData.width;
    			_psd.object_height = camData.height;
    			_psd.hfov = camData.hfov;
    			_psd.gimbalPan = camData.gimbalPan;
    			_psd.gimbalTilt = camData.gimbalTilt;
    			_psd.gimbalRoll = camData.gimbalRoll;
    		}else
    		{
    			Log.msgPrintf ("%sFailed to get data from Camera Comm", SUBS_PREFIX);
    		}
    	}
    }
    void PhysicalState::initCameraRequest(void)
    {
#if USE_WSM == WSM_ENABLE
    	uint8_t payload[5];
    	//sync1
    	payload[0] = 0x2A;
    	//sync2
    	payload[1] = 0x30;
    	//key
    	payload[2] = 0x30;
    	//length
    	payload[3] = 0x50;
    	//checksum
    	payload[4] = 0x0D;

    	if(TtyCAM != NULL)
    	{
    		TtyCAM->sendBinary(payload, 5);
    	}
#else
    	uint8_t payload[5];
    	//sync1
    	payload[0] = 0x24;
    	//sync2
    	payload[1] = 0x40;
    	//key
    	payload[2] = 0x04;
    	//length
    	payload[3] = 0x00;
    	//checksum
    	payload[4] = 0xFF;

    	if(TtyCAM != NULL)
    	{
    		TtyCAM->sendBinary(payload, 5);
    	}
#endif
    }
#endif

#if MAGNETOMETER_TYPE == USE_HMR
void PhysicalState::useHmrData(const ClassifiedLine &cl)
{
    if (_conf.simLevel == 0)
    {
    	if(Gauge::isPacketHmrData(cl.getLine()))
    	{
    		_gauge.hmrParseData(cl.getLine());
    	}
    }
}
#endif

#if PILOT_TARGET == PT_HARDWARE

bool PhysicalState::xAdcPSInit(void)
{
	int Status;
	/*
	 * Initialize the XAdc driver.
	 */
	xAdcPs_ConfigPtr = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID);
	if (xAdcPs_ConfigPtr == NULL) {
		Log.msgPrintf("ps: initialize_xAdc_failed_1");
		return false;
	}
	XAdcPs_CfgInitialize(&XAdcInst, xAdcPs_ConfigPtr,
			xAdcPs_ConfigPtr->BaseAddress);

	Status = XAdcPs_SelfTest(&XAdcInst);
	if (Status != XST_SUCCESS) {
		Log.msgPrintf("ps: initialize_xAdc_failed_2");
		return false;
	}
	XAdcPs_SetSequencerMode(&XAdcInst, XADCPS_SEQ_MODE_SAFE);

	return true;
}

void PhysicalState::usexadcpsData(void)
{
	/*
	 * Read the on-chip Temperature Data (Current/Maximum/Minimum)
	 * from the ADC data registers.
	 */
	uint32_t TempRawData = XAdcPs_GetAdcData(&XAdcInst, XADCPS_CH_TEMP);
	float TempData = XAdcPs_RawToTemperature(TempRawData);
	_psd.cpuTemp = TempData;
}

int PhysicalState::xAdcFractionToInt(float FloatNum)
{
	float Temp;

	Temp = FloatNum;
	if (FloatNum < 0) {
		Temp = -(FloatNum);
	}

	return( ((int)((Temp -(float)((int)Temp)) * (1000.0f))));
}
#endif

/** \name Method loads subsystem configuration. In the case of an error default data are set (error could falsify the actual values)
*/
bool PhysicalState::confLoad(void)
{
    _isConfigLoaded = _confMem->loadFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!_isConfigLoaded)
    {
        // Restoring the default configuration in case of error
        _conf.setDefault ();
        Log.errorPrintf("PhysicalState_confLoad_1");
    }

	// Restoring the default declination and PI/2-inclination to the Ahrs subsystem
    _stateMem.ahrsObject.Declination = _stateMem.declination;
    _stateMem.ahrsObject.Pi_2minusInclination = PI2 - _stateMem.inclination;

    return _isConfigLoaded;
}

/** \name Method loads gauges configuration. In the case of an error default data are set (error could falsify the actual values)
*/
bool PhysicalState::gaugeConfLoad(void)
{
	float xoff = 0.0f, yoff = 0.0f, zoff = 0.0f;
	float r11 = 0.0f, r12 = 0.0f, r13 = 0.0f, r21 = 0.0f, r22 = 0.0f, r23 = 0.0f, r31 = 0.0f, r32 = 0.0f, r33 = 0.0f;
	float phi = 0.0f, theta = 0.0f, psi = 0.0f;
	int result = 0;

    _isGaugeConfigLoaded = _gaugeConfMem->loadFile (GAUGE_CONF_FILE_NAME, &_gauge.gGaugeParam, sizeof(_gauge.gGaugeParam));
    if (!_isGaugeConfigLoaded)
    {
        // Restoring the default configuration in case of error
        _gauge.setDefaultConfig ();
        Log.errorPrintf("PhysicalState_gaugeConfLoad_1");
    }

    // Restoring default data or read data from converters
    _gauge.initADC();

	// Load configuration data from megnetometer
	// Waiting for reading of configuration data from magnetometer device
	// Delay between swich the power on and the first response from the magnetometer device (about 650ms)

	// RF: 1.12.2010 - in simulation mode reading data from magentometer is omitted ('cimLevel' > 0)
	if (_conf.simLevel == 0)	
	{
    	int magnetometerPresent = 1;
        int counter = 0;
		while((counter<200)&&(result==0))
		{
			_gauge.readData(); // Faked data reading
			result = _gauge.readMagnetometerConfiguration(xoff,yoff,zoff,r11,r12,r13,r21,r22,r23,r31,r32,r33,phi,theta,psi,magnetometerPresent);
			counter++;
            Os->sleepMs(10);
		}
	}
	else
	{
		result = 1;
	}
    
	// Data was read successfully, rewriting the configuration data
	if(result==1)
	{

	}else{
		// Failed to read data from the magnetometer
		_isGaugeConfigLoaded = false;
		Log.errorPrintf("PhysicalState_gaugeConfLoad_2");
	}

	// Calculation of transformation matrix for magnetometer position correction based on previously determined angles

	float phi_k = _gauge.gGaugeParam.magnetPhi/180.0f*PI;
	float theta_k = _gauge.gGaugeParam.magnetTheta/180.0f*PI;
	float psi_k = _gauge.gGaugeParam.magnetPsi/180.0f*PI;

	float sintheta = sin(theta_k);
	float costheta = cos(theta_k);
	float sinphi = sin(phi_k);
	float cosphi = cos(phi_k);
	float sinpsi = sin(psi_k);
	float cospsi = cos(psi_k);

	_gauge.mag_kor_11 = costheta*cospsi; 
	_gauge.mag_kor_21 = costheta*sinpsi;
	_gauge.mag_kor_31 = -sintheta;
	_gauge.mag_kor_12 = sinphi*sintheta*cospsi-cosphi*sinpsi; 
	_gauge.mag_kor_22 = sinphi*sintheta*sinpsi+cosphi*cospsi;
	_gauge.mag_kor_32 = sinphi*costheta;
	_gauge.mag_kor_13 = cosphi*sintheta*cospsi+sinphi*sinpsi;
	_gauge.mag_kor_23 = cosphi*sintheta*sinpsi-sinphi*cospsi; 
	_gauge.mag_kor_33 = cosphi*costheta;

    return _isGaugeConfigLoaded;
}

/** \name Method saves subsystem configuration data.
*/
bool PhysicalState::confSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _confMem->saveFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!bok)
        Log.errorPrintf("PhysicalState_confSave_1");

    return bok;
}

/** \name Method saves gauges configuration data.
*/
bool PhysicalState::gaugeConfSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _gaugeConfMem->saveFile (GAUGE_CONF_FILE_NAME, &_gauge.gGaugeParam, sizeof(_gauge.gGaugeParam));
    if (!bok)
        Log.errorPrintf("PhysicalState_gaugeConfSave_1");

    return bok;
}

/** \name Method sends notification of reading the sensors data to other subsystems, and telemetry data to the communication channel and to the log.
*/
void PhysicalState::nslData (void)
{
   // Initialization of requesting the RS485 data
   if (PState->getSimLevel() == 0)
   {
        init485requests ();
   }

#if USE_CAM == CAM_ENABLE
   if (PState->getSimLevel() == 0)
   {
	   initCameraRequest();
   }
#endif


	// Notification about new data
    if (++_notifyCounter % _conf.notifyDivisor == 0)
    {
        notify (PSTATE_CHANGED);
        _notifyCounter = 0;
    }

    char buf[LINESIZE];

	// Sending data to log
    if (_conf.logDivisor != 0)
	{
        if (++_logCounter % _conf.logDivisor == 0)
        {
            if (_conf.stateLogEnable)
                if (prepareTlmLine (_conf.logFormat, buf, sizeof(buf)))
                    Log.tlmPrint (static_cast <enum TLM_FORMAT>(_conf.logFormat), buf, true, false);
                else
                    Log.errorPrintf("PhysicalState_nlsData_1a");

            if (_conf.rawGaugeLogEnable)
                if (prepareRawGaugeTlmLine (buf, sizeof(buf)))
                    Log.tlmPrint (TLM_RAW_GAUGE, buf, true, false);
                else
                    Log.errorPrintf("PhysicalState_nlsData_2a");

            _logCounter = 0;
        }
	}

    // Sending data to communication channel
    if (_conf.commDivisor != 0)
	{
        if (++_commCounter % _conf.commDivisor == 0)
        {
            if (prepareTlmLine (_conf.commFormat, buf, sizeof(buf)))
                Log.tlmPrint (static_cast <enum TLM_FORMAT>(_conf.commFormat), buf, false, true);
            else
                Log.errorPrintf("PhysicalState_nlsData_1b");

			// GT: T06 sending data to a serial device
            if (_conf.rawGaugeCommEnable)
                if (prepareRawGaugeTlmLine (buf, sizeof(buf)))
                    Log.tlmPrint (TLM_RAW_GAUGE, buf, false, true);
                else
                    Log.errorPrintf("PhysicalState_nlsData_2b");

            _commCounter = 0;
        }
	}
}

// Sending/writing auxilliary telemetry data (the line) to the log at the specified time intervals
void PhysicalState::sendAuxTlm (void)
{
    char buf[LINESIZE];

	// Sending data to log
    if (_conf.logAuxSensorDivisor != 0)
	{
        if (++_logAuxSensorCounter % _conf.logAuxSensorDivisor == 0)
        {
            _logAuxSensorCounter = 0;
            if (_conf.auxSensorLogEnable)
			{
                if (prepareAuxSensorTlmLine (buf, sizeof(buf)))
				{
					Log.tlmPrint (TLM_AUX_INFO, buf, true, false);
				}
				else
				{
					Log.errorPrintf("SystemMonitor_sendAuxTlm_1a");
				}
			}
        }
	}

    // Sending data to communication channel
    if (_conf.commAuxSensorDivisor != 0)
	{
        if (++_commAuxSensorCounter % _conf.commAuxSensorDivisor == 0)
        {
            _commAuxSensorCounter = 0;
			if (_conf.auxSensorCommEnable)
			{
				if (prepareAuxSensorTlmLine (buf, sizeof(buf)))
				{
					Log.tlmPrint (TLM_AUX_INFO, buf, false, true);
				}
				else
				{
					Log.errorPrintf("SystemMonitor_sendAuxTlm_1b");
				}
			}
        }
	}
}

/** \name Method prepares textual line of telemetry data.
* \param 'format' - format identifier
* \param 'buf' - output buffer
* \param 'bufSize' - buffer size
*/
bool PhysicalState::prepareTlmLine (int format, char* buf, int bufSize)
{
	// Setting the elevation value of start position (manually by user)
	// getAglOffest() returns value of correction for altimeter before landing in the way that sum of a psd.altitude and tElev procucts an amsl altitude.
    float tElv = _stateMem.tlmOriginAmsl - getAglOffset();

    float tAal = _stateMem.useElvOffsetPar ? _stateMem.elevationOffset : 0.0f;

    if (format == 1)
    {
        PStateTlmShort t1;

        if (!_vSem.lock ())
            return false;

        t1.fillFrom (_psd, tElv, tAal);

        if (!_vSem.unlock ())
            return false;

        if (!Base64::encode (&t1, sizeof(t1), buf, bufSize))
        {
            Log.errorPrintf("PhysicalState_prepareTlmLine_1");
            return false;
        }

        return true;
    }
    else if (format == 2)
    {
        PStateTlmLong t2;

        if (!_vSem.lock ())
            return false;

        t2.fillFrom (_psd, tElv, tAal);
              
        if (!_vSem.unlock ())
            return false;

        if (!Base64::encode (&t2, sizeof(t2), buf, bufSize))
        {
            Log.errorPrintf("PhysicalState_prepareTlmLine_2");
            return false;
        }

        return true;
    }
    else
    {
        Log.errorPrintf("PhysicalState_PrepareTlmLine_3");
    }

    return false;
}

/** \name Method prepares a textual line of row data form sensors.
* \param 'buf' - bufor wyj�ciowy
* \param 'bufSize' - size of a buffer
*/
bool PhysicalState::prepareRawGaugeTlmLine (char* buf, int bufSize) const
{
    PStateRawGaugeTlm rgt;

	// The semaphore protection isn't needed here, because _gauge is written only by this subsystem.
	rgt.fillFrom (_stateMem.time100, _gauge.gRawMeasurement,_gauge.grawDiffPressureMeasurement,_gauge.grawBaroMeasurement, _gauge.gMagnetRawMeasurement, _stateMem.ahrsObject.gAhrsData, _psd, _auxSensorData);

    if (!Base64::encode (&rgt, sizeof(rgt), buf, bufSize))
    {
        Log.errorPrintf("PhysicalState_prepareRawGaugeTlmLine_1");
        return false;
    }

    return true;
}

/** \name Method prepares text line with telemetry data.
* \param 'buf' is an output text buffer.
* \param 'bufSize' is the size of the output buffer.
*/
bool PhysicalState::prepareAuxSensorTlmLine (char* buf, int bufSize) const
{
	PStateAuxSensorTlm ast;
	ast.fillFrom(_stateMem.time100, _auxSensorData);

    if (!Base64::encode (&ast, sizeof(ast), buf, bufSize))
    {
        Log.errorPrintf("PhysicalState_prepareAuxSensorTlmLine_1");
        return false;
    }

    return true;
}

//  Funkcja ustawia warto�ci wielu parametr�w. W przypadku b��du zachowuje niezmienione warto�ci.
//      nameValueItems - pary <nazwa> <warto��> (w postaci tekstowej)
/** \name Method sets parameters with a values passed as an argument 'nameValueItems' of a list of pairs 'param_name' and 'value'.
* \param 'nameValueItems' - textual list of a pairs of parameter name and coresponding value.
*/
ParameterNames::ERRCODE PhysicalState::setParameters (const char* nameValueItems)
{
    unsigned int flags=0;
    ParameterNames::ERRCODE err = _confNames->setParams (nameValueItems, &flags);
    if (err == ParameterNames::ERR_OK)
        setParamUserAction (flags);

    return err;
}

/** \name Method sends to the specified device 'cl' a value of subsystem configuration parameter 'pName' (as a text).
* \param 'pName' - name of parameter to send (or '*' if all of the parameters are expected to be send)
* \param 'cl' - a communication device to which to parameter(s) will be send
*/
bool PhysicalState::getParameter (const char* pName, ClassifiedLine& cl) const
{
    return _confNames->getParam(pName, cl);
}

/** \name Method reads static pressure value [hPa] from specified sensor.
* \param 'sourceNo' - pressure sensor identifier (0-sensor in VT Hub, 1-sensor at the WSM)
*/
float PhysicalState::getStaticPressureFromSensor () const
{
	return static_cast<float>(_gauge.grawBaroMeasurement.Pressure) * 0.01f;
//    if (sourceNo == 0)
//        return static_cast<float>(_auxSensorData.i_pressure) * 0.01f;
//    else
//        return (_gauge.gMagnetMeasurement.magPressure * 0.01f);
}

/** \name Method reads static pressure from specified sensor which is automatically switched to another in case of damage.
* \param 'sourceNo' - 0: sensor from autopilot device, 1: sensor from the wing.
*/
float PhysicalState::getStaticPressure () const
{
	return getStaticPressureFromSensor();
//    // Sensor not locked
//    if (!_stateMem.spSensorLocked)
//        return getStaticPressureFromSensor(sourceNo);
//
//    // Switch from hub to pitot sensor
//    else
//    {
//        if (_conf.spSensorSwitchEnable)
//            return getStaticPressureFromSensor (1) - _stateMem.spSensorDelta;
//
//        // Keep the failed sensor
//        else
//            return getStaticPressureFromSensor(sourceNo);
//    }
}

/** \name Method calculates altitude based on the pressure (standard model of an athmosphere)
* \see http://www.srh.noaa.gov/epz/wxcalc/formulas/pressureAltitude.pdf
*/
float PhysicalState::altFromPress (float press)
{
    static const float a = 1.0f / 1013.25f;
    float h = (1.0f - powf (press * a, 0.190284f)) * 44307.69396f;

    return h;
}

/** \name Method calculates TAS speed.
* \see http://williams.best.vwh.net/avform.htm#Mach
*/
float PhysicalState::computeTas (float ias, float staticPressure, float outerAirTemp)
{
    float tas = ias * sqrtf (STA_P/staticPressure * (outerAirTemp+KELVIN_ADD)/(STA_T+KELVIN_ADD));
    return tas;
}

/** \name Method calculates the final altitude including all amendments
*/
float PhysicalState::processedAltitude (void) const
{
    float press = _stateMem.elevationPressure;

    // Correction of a reference pressure, measured by an aircraft's sensors and stored during take-off, with difference between ground
	// station pressure measured during take-off and current pressure of ground station position
    if (_conf.useBasePressure &&
        _stateMem.refBasePressure >= MIN_BASE_PRESSURE &&
        _stateMem.refBasePressure <= MAX_BASE_PRESSURE &&
        _stateMem.currentBasePressure >= MIN_BASE_PRESSURE&&
        _stateMem.currentBasePressure <= MAX_BASE_PRESSURE)
        press += (_stateMem.currentBasePressure - _stateMem.refBasePressure);	// currentBasePressure: ap suat tai gcs hien tai; refBasePressure: ap suat tai gcs luc cat canh

    float altRef = altFromPress (press);										// altRef: do cao tham chieu tai vi tri gcs
    float altAbs = altFromPress (_psd.staticPressure);							// altAbs: do cao tuyet doi cua UAV
    float a = altAbs - altRef + _stateMem.offsetAgl;							// a:	do cao tuong doi cua UAV = do cao tuyet doi - do cao tham chieu - offset (ban do dia hinh)
	if (_stateMem.useElvOffsetPar)
        a -= _stateMem.elevationOffset;
    if (_stateMem.useTakeoffAgl)
        a += _conf.takeoffAgl;

    return a;
}

/** \name Method sets the reference pressure of the ground station
*/
void PhysicalState::setRefBasePressure (void)
{
    if (_stateMem.currentBasePressure >= MIN_BASE_PRESSURE &&
        _stateMem.currentBasePressure <= MAX_BASE_PRESSURE)
    {
        _stateMem.refBasePressure = _stateMem.currentBasePressure;
        Log.msgPrintf ("%sBase station ref. pressure: %.3f", SUBS_PREFIX, _stateMem.refBasePressure);
    }
}

/** \name Method performs action when parameter value has been changed
*/
void PhysicalState::setParamUserAction (unsigned int flags)
{
	// Current pressure value at a base station has been changed
	// Is reference pressure is not set (preser value is 0) then it should be set
    if ((flags & UAF_BASE_PRESS) != 0)
	{
        if (_stateMem.refBasePressure < MIN_BASE_PRESSURE)
            setRefBasePressure ();
	}
	if ((flags & UAF_AHRS_INCL) != 0)
	{
         _stateMem.ahrsObject.Pi_2minusInclination = PI2 - _stateMem.inclination;
	}
	if ((flags & UAF_AHRS_DECL) != 0)
	{
        _stateMem.ahrsObject.Declination = _stateMem.declination;
	}

    // Turn on/off test of a GPS failure
	if ((flags & UAF_GPS_FAIL) != 0)
	{
        if (_gpsFailTest)
            _gpsFailTestTOff100 = _stateMem.time100 + _conf.gpsFailTestTimeout * 10000;
	}
}

/** \name Method decodes part of a RTK correction and send it to the auxiliary GPS
*/
bool PhysicalState::decodeRtkPart (const char* rtkPartB64)
{
    unsigned char buf[LINESIZE*3/4+10];
    int nBytes = 0;

    if (Base64::decode (rtkPartB64, buf, sizeof(buf), nBytes))
    {
        if (Tty2 != NULL)
        {
            Tty2->sendBinary (buf, nBytes);
            return true;
        }
    }

    return false;
}

/** \name Method calculates plane navigation data (e.g. possition) after failure of a GPS.
* 'psd' and 'psh' must hold current values. Method updates the 'psd' structure but do not change the GPS failure status.
*/
void PhysicalState::gpsFailNavigation (void)
{
	// Method returns when GPS navigation is disabled
    if (!_conf.gpsFailNavEnable)
        return;

	// If GPS works correctly and nothings change then current position is stored
    if (_psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_OK ) //||
        //_psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_2D_MODE ||
        //_psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_BAD_PRECISION)
    {
        _gpsLastDistance = GpsPosition::distance(_stateMem.gpsFailLastPos, _psd.position);
        if(_gpsLastDistance > _gpsMaxDistance)
        {
            _gpsLastDistance = _gpsMaxDistance;
        }
        _stateMem.gpsFailLastPos = _psd.position;
        _stateMem.gpsFailIncrS.x = 0.0f; _stateMem.gpsFailIncrS.y = 0.0f;
        return;
    }

	// Projection of a TAS speed on the surface of the Earth (sideslip angle should be taken into account but it's unknown;
	// at 10 degrees of theta there is 1,5 % of a deviation so it's rather small, it's more to take into account the climbing on the engine)
    float vt = _psd.tas * cos(_psd.theta);
	// Aircraft velocity vector relative to air ('psi' in radians, x - north, y- east)
    Vector2f vecVAir = Vector2f::fromRadial (vt, _psd.psi);

    // Navigational wind direction in radians
    float war = GpsPosition::addTrack (_psd.wind.from, 180.0f) * DEG_2_RAD;
	// Wind speed vector (navigation)
    Vector2f vecVWind = Vector2f::fromRadial (_psd.wind.speed, war);
    
    // Velocity vector relative to the Earth
    if(_psd.groundspeed < 2.0f && _psd.airspeed < 20.0f)
    {
        vecVAir.x = 0.0f;
        vecVAir.y = 0.0f;
    }
    Vector2f vecVEarth = vecVAir;
    vecVEarth.add (vecVWind);

	// Convertion to radial coordinates (in radians)
    vecVEarth.toRadial (_psd.groundspeed, _psd.track);
	// Track normalization (degrees)
    _psd.track *= RAD_2_DEG;

    // GPS altitude
    _psd.gpsAmsl = _psd.altitude + _stateMem.tlmOriginAmsl;

    // Vector of distance traveled during single time sample
    Vector2f deltaS = vecVEarth;
    deltaS.multC (KPH_2_MS / GAUGE_CYCLES_PER_SECOND);
	// Vector of track from the beginning of the flight to the current position
    _stateMem.gpsFailIncrS.add (deltaS);
	// Estimated geographical position (x - north, y - east)
    GpsPosition::movePositionXY (_stateMem.gpsFailLastPos, _stateMem.gpsFailIncrS.y, _stateMem.gpsFailIncrS.x, _psd.position);
}

/** \name Method resets the 'gpsFail' flag after specified timeout
*/
void PhysicalState::gpsFailTestCheckTout (void)
{
    if (_gpsFailTest)
        if (_psd.time100 > _gpsFailTestTOff100)
        {
            Log.msgPrintf ("%sGPS Fail Test auto off.", SUBS_PREFIX);
            _gpsFailTest = false;
        }
}

/** \name Method initializes a sequence of data requests to a RS485 serial device
* The request is send to the first device. Requests to other devices are send in the response handler method.
* \note Temporary version - fuel level indicator only for VTPatrol
*/
void PhysicalState::init485requests (void)
{
    FixBuffer<50> payload;

	// Frame type (Simple Application Layer)
    payload.concatB (0x81u);
	// Command id (in this case - sequence number)
    payload.concatB (_485seq);
    _485seq++;      // Automatic modulo 256
    // Interface
    payload.concatB (0x01u);
    // Command type
    payload.concatB (0xAAu);
    // Status
    payload.concatB (0xFFu);
#if (PILOT_TYPE == VUA_SC && ENGINE_TYPE == VUA_G)|| PILOT_TYPE == VUA_SL || PILOT_TYPE == VUA_SC_6G
    // Day
    payload.concatB (_psd.dtUtc.day & 0xFF);
    // Month
    payload.concatB (_psd.dtUtc.month & 0xFF);
    // Year
    payload.concatB (_psd.dtUtc.year & 0xFF);
    
    // Hour
    payload.concatB (_psd.dtUtc.hours & 0xFF);
    // Minute
    payload.concatB (_psd.dtUtc.mins & 0xFF);
    // Second
    payload.concatB (_psd.dtUtc.secs & 0xFF);
#endif

    if (CTty485 != NULL)
        CTty485->sendData485 (payload, 4, 0, _conf.debugMsg);
}
void PhysicalState::estFuelLevel (void)
{
	float consumption = 0.0f;
	if (_auxSensorData.b_hubDataAvailable)
	{
		if ((_auxSensorData.i_fuelLevel >0) && (_auxSensorData.festFuelLevel < 0.5f))
		{
			_auxSensorData.festFuelLevel = static_cast<float>(_auxSensorData.i_fuelLevel);
			_auxSensorData.estFuelcount = 0;
			_auxSensorData.timeFuelLevel = _psd.time100;
			_auxSensorData.bAllowEstTimeLeft = false;
		}
		else
		{
			if(abs(_psd.theta) < _conf.fuelLevelRollPitchRange * DEG_2_RAD && abs(_psd.phi) < _conf.fuelLevelRollPitchRange * DEG_2_RAD)
			{
				SysMon->getFuelConsumption (consumption);

				if(_auxSensorData.estFuelcount == 0)
				{
					_auxSensorData.festFuelLevel = static_cast<float>(_auxSensorData.i_fuelLevel);
					_auxSensorData.timeFuelLevel = _psd.time100;
				}

				_auxSensorData.festFuelLevel = _auxSensorData.festFuelLevel - static_cast<float>(_psd.time100 - _auxSensorData.timeFuelLevel) * consumption * 0.035f / 36000000.0f;
				_auxSensorData.festFuelLevel = 0.5f * (_auxSensorData.festFuelLevel + static_cast<float>(_auxSensorData.i_fuelLevel));

				_auxSensorData.timeFuelLevel = _psd.time100;
				_auxSensorData.estFuelcount ++;

				if(abs(_auxSensorData.festFuelLevel - static_cast<float>(_auxSensorData.i_fuelLevel)) > 5.0f)
					_auxSensorData.festFuelLevel = static_cast<float>(_auxSensorData.i_fuelLevel);

				if (_auxSensorData.estFuelcount == 1000)
				{
					_auxSensorData.bAllowEstTimeLeft = true;
					_auxSensorData.estFuelcount = 0;
				}
			}
		}
	}
	if(_conf.debugMsg == FUEL_LEVEL)
	{
		Log.errorPrintf("Fuel Level: Hub %d Estimated %0.1f consumption %0.1f", _auxSensorData.i_fuelLevel, _auxSensorData.festFuelLevel, static_cast<double>(static_cast<double>(_psd.time100) - _auxSensorData.timeFuelLevel) * consumption * 0.035 / 36000000);
		_conf.debugMsg = NONE;
	}
}
void PhysicalState::heater485requests (int heaterModeVal)
{
    FixBuffer<50> payload;

	// Frame type (Simple Application Layer)
    payload.concatB (0x81u);
	// Command id (in this case - sequence number)
    payload.concatB (_485seq);
    _485seq++;      // Automatic modulo 256
    // Interface
    payload.concatB (0x01u);
    // Command type
    payload.concatB (0xABu);
    // Status
    payload.concatB (0xFFu);

    // Heater Mode
	payload.concatB (heaterModeVal & 0xFF);
	
    if (CTty485 != NULL)
	{
		_conf.debugMsg = RS485_SEND_DATA;
        CTty485->sendData485 (payload, 4, 0, _conf.debugMsg);
	}
}


#if USE_DGPS == ENABLED
float PhysicalState::getNetOffset (void)
{
    return _DGpsData->currData.measureData.altitude;
}
#endif
#if USE_LEAFLETS == ENABLED
void PhysicalState::loaddropon (void)
{
    FixBuffer<50> payload;

	// Frame type (Simple Application Layer)
    payload.concatB (0x81u);
	// Command id (in this case - sequence number)
    payload.concatB (0x00);
    // Interface
    payload.concatB (0x01u);
    // Command type
    payload.concatB (0xACu);
    // Status
    payload.concatB (0xFFu);

	
    if (CTty485 != NULL)
	{
		_conf.debugMsg = NONE;
        CTty485->sendData485 (payload, 4, 0, _conf.debugMsg);
	}
}

void PhysicalState::loaddropoff (void)
{
    FixBuffer<50> payload;

	// Frame type (Simple Application Layer)
    payload.concatB (0x81u);
	// Command id (in this case - sequence number)
    payload.concatB (0x00);
    // Interface
    payload.concatB (0x01u);
    // Command type
    payload.concatB (0xABu);
    // Status
    payload.concatB (0xFFu);

	
    if (CTty485 != NULL)
	{
		_conf.debugMsg = NONE;
        CTty485->sendData485 (payload, 4, 0, _conf.debugMsg);
	}
}
#endif
//=========================================================================
//  Class SimData
//=========================================================================

/** \name Method interprets a line of data from simulator and set up the structure
* \param 'line' - textual line of data from simulator
* \return 'true' on success, 'false' otherwise
*/
bool PhysicalState::SimData::scanLine(const char* line)
{
    double lat=0.0, lon=0.0;
    int n = 0;
#if EMRG_LOSS_ENGINE == 1
	n = sscanf(line, "#v3,%f,%f,%f,"
            "%lf,%lf,"
            "%f,%f,%f,"
            "%f,%f,%f,"
            "%f,%f,%f,"    
            "%f,%f,%f,"
            "%f,%f,%f,"
            "%f,%f,"
            "%f,"
            "%f,%f,%f,%f",
            &time, &airspeed, &amsl, 
            &lat, &lon,
            &psi, &theta, &phi,
            &psiDot, &thetaDot, &phiDot,
            &R, &Q, &P,
            &accZ, &accY, &accX,
            &windFrom, &windSpeed, &heading,
            &speedEast, &speedNorth,
            &staticPressure,
            &outerAirTemp, &verticalSpeed, &cameraZoom, &rpm);
    
        if (n == 27)
        {
    		// Internal representation of a geographical position does not need to be known (interface of type double)
            position.setLat(lat);
            position.setLon(lon);
            return true;
        }
#else
	n = sscanf(line, "#v3,%f,%f,%f,"
		"%lf,%lf,"
		"%f,%f,%f,"
		"%f,%f,%f,"
		"%f,%f,%f,"
		"%f,%f,%f,"
		"%f,%f,%f,"
		"%f,%f,"
		"%f,"
		"%f,%f,%f,%f",
		&time, &airspeed, &amsl,
		&lat, &lon,
		&psi, &theta, &phi,
		&psiDot, &thetaDot, &phiDot,
		&R, &Q, &P,
		&accZ, &accY, &accX,
		&windFrom, &windSpeed, &heading,
		&speedEast, &speedNorth,
		&staticPressure,
		&outerAirTemp, &verticalSpeed, &cameraZoom, &rpm);

	if (n == 27)
	{
		// Internal representation of a geographical position does not need to be known (interface of type double)
		position.setLat(lat);
		position.setLon(lon);
		return true;
	}
#endif
    Log.errorPrintf("PhysicalState_SimData_scanLine_1 [%d]", n);
    return false;
}


//=========================================================================
//  Class ConfigData
//=========================================================================

// Class constructor with configuration data 
PhysicalState::ConfigData::ConfigData(void)
{
    setDefault ();
}

// Default class constructor
void PhysicalState::ConfigData::setDefault(void)
{
    notifyDivisor       =  1;
    commDivisor         =  0;
	commAuxSensorDivisor=  0;
    logDivisor          =  0;
	logAuxSensorDivisor =  0;
    nmeaCommDivisor     =  0;
    nmeaLogDivisor      =  0;
    commFormat          =  1;
    logFormat           =  2;
    stateLogEnable      = true;
    rawGaugeLogEnable   = false;
    rawGaugeCommEnable  = false;
	auxSensorLogEnable  = false;
    auxSensorCommEnable = false;
    maxHdop             = 10.0f;
#if USE_DGPS == ENABLED
    maxPdop             = 10.0f;
#endif
    gpsTimeout          =  1.5f;
#if USE_DGPS == ENABLED
	dgpsTimeout         =  1.5f;
#endif
    gpsPredictionEnable = true;
    gpsSbasEnable       = false;
    gpsFailTestTimeout  = 120;
    gpsFailNavEnable    = true;
    simLevel            =  0;
	asFilter			= false;
	altFilter			= false;
    pressureSource      =  0;
    useBasePressure     = false;
    takeoffAgl          =  0.0f;
    elevationSetDelay   = 15.0f;
	ahrsVersion			=  0;
    spSensorSwitchEnable = true;
    bUseKFAlt            = 0;
	fuelLevelRollPitchRange = 3.0f;
	gpsUereConfig		 = 7.0f;
	gpsPositionErrOffset = 50.0f;
	gpsJammingTimeOut	 = 60.0f;
	gpsTestJammingEnable = false;
	//chebyshev airspeed filter
	filterAirspeedCoff_a = 0.095466f;
	filterAirspeedCoff_b = -0.17269f;
	filterAirspeedCoff_c = 0.095466f;
	filterAirspeedCoff_d = 1.8090f;
	filterAirspeedCoff_e = -0.82722f;
	filterAirspeedThreshold = 30.0f;
	filterAirspeedActive = false;
	airSpdScaleRatio = 1.0f;
#if USE_DGPS == 1
	dgpsPredictionEnable = 1;
	bUseDGps			 = 0;
    dgpsDevice           = DGPS_UNKNOWN;
#endif
    debugMsg             = NONE;
}

//=========================================================================
//  Class StateData
//=========================================================================
PhysicalState::StateData::StateData(void):
    elevationAmslSim(0.0f),
    elevationPressure(0.0f),
    elevationIsSet(false),
    useTakeoffAgl(false),
    offsetAgl(0.0f),
    refBasePressure(0.0f),
    currentBasePressure(0.0f),
    tlmOriginAmsl(0.0f),
    gaugeTicks(0),
    time100(0),
    ahrsObject(GAUGE_CYCLES_PER_SECOND),
    windRes(),
    elevationOffset(0.0f),
    useElvOffsetPar(false),
    iasFailMode(false),
    inclination(INCLINATION_NOT_SET),
    declination(DECLINATION_NOT_SET),
    spSensorLocked(false),
    spSensorDelta(0),
    gpsFailIncrS (0.0f, 0.0f)
{};

void PhysicalState::checkGpsData(bool &windUpdated)
{
	//  GPS

    if (static_cast<float>(_psd.time100 - _currentGps.userTime) <= _conf.gpsTimeout * 10000.0f)
	{
        if (!_currentGps.parseError)
        {
            if ((_currentGps.navMode == GpsData::NAV_MODE_3D ||
                _currentGps.navMode == GpsData::NAV_MODE_2D) &&
                !_gpsFailTest)
            {
				// There are some important data (data may be incomplete or may have too low accuracy)
                _psd.rawGpsData = _currentGps.position;
				// Setting of an error code
                if (_currentGps.navMode == GpsData::NAV_MODE_2D)
                    _psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_2D_MODE;
                else if (_currentGps.hDop > _conf.maxHdop)
                    _psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_BAD_PRECISION;
                else 
				{
					if ((_previousGps.userTime > 100) && _conf.gpsTestJammingEnable)
					{
						if(static_cast<float>(_currentGps.userTime - _previousGps.userTime) <= _conf.gpsJammingTimeOut * 10000.0f)
						{
							float dDist = (_previousGps.groundSpeed + _currentGps.groundSpeed) * KPH_2_MS * (_currentGps.userTime - _previousGps.userTime) * 0.0001f;
							_gpsMaxDistance = dDist + (_previousGps.hDop + _currentGps.hDop) * _conf.gpsUereConfig + _conf.gpsPositionErrOffset;
							if(_gpsMaxDistance > 100.0f)
								_gpsMaxDistance = 100.0f;
							float currentDist = GpsPosition::distance(_currentGps.position, _previousGps.position);

							if (currentDist > _gpsMaxDistance)
							{
								_currentGps = _previousGps;
								_psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_JAMMING_POSITION;
							}
							else
							{
								_psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_OK;
							}
						}
					}
					else
						_psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_OK;
				}				
            }
            else
            {
				// Data are present but not valid
                _psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_NO_VALID_DATA;
            }
        }
        else
        {
			// Parse error - not data
            _psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_NO_OUTPUT;
        }
	}
    else
    {
		// No communication with GPS
        _psh.gpsError = PStateHealth::gpsErrorCode::ERR_GPS_NO_OUTPUT;
    }

	if(_psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_OK)
	{
		// Prediction of gps indication
        _predictedGps = _currentGps;
        if (_conf.gpsPredictionEnable && _gpsPredictionEnable)
        {
			_predictedGps.predict (_previousGps, _currentGps, _psd.time100);
			// Smoothing the groundspeed
            _predictedGps.groundSpeed = (_predictedGps.groundSpeed + _psd.groundspeed) * 0.5f;
        }
		else
		{
			// wait until the plane move before smoothing ground speed
			if((_psd.groundspeed > 0.1f)&&!_gpsPredictionEnable)
			{
				_gpsPredictionEnable = true;
			}
		}

		// Setting GPS data in '_psd' structure (PStateData)
        _psd.position = _predictedGps.position;
        _psd.groundspeed = _predictedGps.groundSpeed;

        if (_predictedGps.trackPresent)
        {
			_psd.track = _predictedGps.track;
            _psd.velned = Vector3f(_psd.groundspeed * cosf(Numbers::radians(_psd.track)), 
                _psd.groundspeed * sinf(Numbers::radians(_psd.track)), -_psd.f32VertSpeed);
			// Wind calculation
            int alg = _stateMem.windRes.putData(_psd, _psh);
            if (alg > 0)
            {
				_stateMem.windRes.getWind (_psd.wind.from, _psd.wind.speed);
                Log.msgPrintf ("%sWind: %.1f kph, %.1f deg [%s]", SUBS_PREFIX, _psd.wind.speed, _psd.wind.from, alg==1 ? "a1":"a2");
                windUpdated = true;
            }
		}

		// GPS Altitude
		if (_currentGps.amslPresent)
		{
			_psd.gpsAmsl = _currentGps.amsl;
		}
	}
}

float PhysicalState::distanceLowpassNonlinearFilter (float input, float &previousDistance, float maxDistance)
{
	float D = input - previousDistance;
	float S = 0.2f;
	float K = 0.0f;
	float maxDist = maxDistance;

	if (maxDist < 0.0f)
		maxDist = 100.0f;
	
	K = S / maxDist; 
	float _G = S * (1.0f + K * K);
	if (_G > 1.0f)
		_G = 1.0f;

	float fitered_output = previousDistance + D * _G;

	if(fitered_output > maxDist)
		fitered_output = maxDist;
    previousDistance = fitered_output;

	return fitered_output;
}

#if USE_DGPS == 1
bool PhysicalState::getUseDGpsData(void) const
{
	return _conf.bUseDGps;
}

int PhysicalState::getDgpsPredictionEnable (void)
{
    return _conf.dgpsPredictionEnable;
}
#endif
