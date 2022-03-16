#include <PilotIncludes.h>

///Name of the subsystem used as a prefix.
const char SystemMonitor::SUBS_PREFIX[]    = "smon: ";

///Configuration data file name.
const char SystemMonitor::CONF_FILE_NAME[] = "smoncfg";

/** \name System Error Messages
*\{
*/
const char SystemMonitor::ERR_OK[]              = "smon: ok";
const char SystemMonitor::ERR_CLOAD[]           = "smon: SM02 Cannot load config data";
const char SystemMonitor::ERR_CSAVE[]           = "smon: SM03 Cannot save config data";
const char SystemMonitor::ERR_UCOMMAND[]        = "smon: SM04 Unrecognized command or wrong number of parameters";
const char SystemMonitor::ERR_BADPAR[]          = "smon: SM05 Bad parameter name or value";
const char SystemMonitor::MSG_NO_CONFIG[]       = "smon: SM07 Config data not loaded from flash";
const char SystemMonitor::ERR_READONLY[]        = "smon: SM08 Cannot set ReadOnly parameter";
const char SystemMonitor::ERR_STATE_MEM_TEST[]  = "smon: SM20 State memory test failed";
const char SystemMonitor::ERR_CONFIG_MEM_TEST[] = "smon: SM21 Config memory test failed";
const char SystemMonitor::ERR_LOG_MEM_TEST[]    = "smon: SM22 Log memory test failed";
const char SystemMonitor::ERR_REBOOT[]          = "smon: SM23 Airborne reboot not allowed";
const char SystemMonitor::MSG_VBAT_NO_TAKEOFF[] = "smon: SM24 Main battery voltage too low for takeoff";
const char SystemMonitor::MSG_AUXBAT_DCHARGE[]  = "smon: SM25 Auxiliary battery is discharging - reboot needed";
const char SystemMonitor::MSG_NOT_READY[]       = "smon: SM26 System not ready for takeoff";
const char SystemMonitor::MSG_LANDED_REBOOT[]   = "smon: SM27 System landed - reboot needed";
const char SystemMonitor::MSG_NO_GPARS[]        = "smon: SM28 Global parameters not loaded from flash";
const char SystemMonitor::MSG_UPG_IN_PRGRS[]    = "smon: SM29 Upgrade in progress";
const char SystemMonitor::ERR_CMD_BUF_FULL[]    = "smon: SM30 Command buffer full";
///\}

/** \name Flash Memory Programmer Error Messages
*\{
*/
const char SystemMonitor::ERR_FLASH_NOT_PREP[]  = "smon: SM40 Flash programming not prepared";
const char SystemMonitor::ERR_FLASH_BAD_LEN[]   = "smon: SM41 Incorrect flash length";
const char SystemMonitor::ERR_FLASH_BAD_ADDR[]  = "smon: SM42 Incorrect flash address";
const char SystemMonitor::ERR_FLASH_INTERNAL[]  = "smon: SM43 FlashProgrammer internal error";
const char SystemMonitor::ERR_FLASH_CRC_SDRAM[] = "smon: SM44 *** SDRAM CRC Error ***";
const char SystemMonitor::ERR_FLASH_CRC[]       = "smon: SM45 *** FLASH CRC FATAL ERROR ***";
const char SystemMonitor::ERR_FLASH_WRITE[]     = "smon: SM46 *** Write flash FATAL ERROR ***";
///\}

SystemMonitor::SystemMonitor(void):
	// Initialization in case of no execution the 'linkObserver' method
    ODTSubject(), ODTObserver(), SubsystemBase(),
	// Counters initialization (-1 enters at the first notice, what is important with FG, which can be paused )
	_pStateTag(-1), _fPRealLndTag(-1), _fPRealTkfTag(-1), _extCmdTag(-1),
     _commCounter(-1), _logCounter(-1),
#if EMRG_LOSS_ENGINE == 1
	_lossEngineCounter (-1),
#endif
    _checkCounter(-1),
    _cmdq(Log), _pStateInitialized(false),  _vg(0.0f), _ig(0.0f), _vd(0.0f), _id(0.0f), _rs485Status(0), _idCounter(0), _isConfigLoaded(false),
    _mainVoltageDrop(false), _fteIsCalculated(false), _timeMax(0.0f),_bTmaxEstimated(false),_startTime100(0), _bUpdateTimeLeft (false), _bufferTime100(0)
#if EMRG_LOSS_ENGINE == 1
	, _bLossEngine(false)
	, _bFocusLand(false)
	, _bCheckLostEngine(false)
	, _bLostEngineAlert(false)
#endif
{    
	//Creation of a subsystem data access controll semaphore. If semaphore is not created correctly a particular error message is logged.
    if (!_vsem.create("SystemMonitor"))
    {
        Log.abort ("Critical Error: SystemMonitor_1.");
        return;
    }
	    
	//Creation of a configuration data memory. If memory is not created correctly a particular error message is logged.
    _confMem = StorageFactory::createConfigMemory();
    if( _confMem == NULL)
    {
        Log.abort ("Critical Error: SystemMonitor_2.");
        return;
    }
	    
	//Creation of a special memory object designed for memory testing.
    _configMemTest = StorageFactory::createConfigMemory();

    // Set subsystem prefix to FlashProgrammer
    _flashProgrammer.setPrefix(SUBS_PREFIX);
   
	//If memory testing object is not created correctly a particular error message is logged.
    if (_configMemTest == NULL)
    {        
		// After airborn 'abort' cannot be exesuted (memory testing is not necessary)
        Log.tryAbort ("Critical Error: SystemMonitor_3.");
        return;
    }

	//Initialization of a mapping parameter names to objects. If initialization failed error message is logged.
    _pars = new ParameterNames(SUBS_PREFIX, 56);
    if (_pars == NULL)
    {
        Log.abort ("Critical Error: SystemMonitor_4.");
        return;
    }

    /* 1*/_pars->insert("OnGround", &SystemNowOnGround);
    /* 2*/_pars->insert("StartedOnGround", &SystemStartedOnGround);
    /* 3*/_pars->insert("commDivisor", &_conf.commDivisor, 0, 1000);
    /* 4*/_pars->insert("logDivisor", &_conf.logDivisor, 0, 1000);
    /* 5*/_pars->insert("checkDivisor", &_conf.checkDivisor, 10, 1000);
    /* 6*/_pars->insert("airborneAglCond", &_conf.airborneAglCond, 0.0f, 1000.0f, 2);
    /* 7*/_pars->insert("airborneIasCond", &_conf.airborneIasCond, 0.0f, 1000.0f, 1);
    /* 8*/_pars->insert("commTtyNo", &GPar.commTtyNo, 0, 2, false, UAF_SET_TTY);
    /* 9*/_pars->insert("simTtyNo", &GPar.simTtyNo, 0, 2);
    /*10*/_pars->insert("blueToothEnable", &GPar.blueToothEnable);
    /*11*/_pars->insert("commCrc", &GPar.commCrc);
    /*12*/_pars->insert("commMsgCrc", &GPar.commMsgCrc);
    /*13*/_pars->insert("rs485New", &GPar.rs485New);
    /*14*/_pars->insert("maxUpperIBat", &_conf.maxUpperIBat, -10.0f, 0.0f);
    /*15*/_pars->insert("minLowerVBat", &_conf.minLowerVBat,   0.0f, 100.0f, 2);
    /*16*/_pars->insert("linkBrokenTimeout", &_conf.linkBrokenTimeout, 0.0f, 100000.0f);
    /*17*/_pars->insert("lastLinkPosOffset", &_conf.lastLinkPosOffset, 0.0f, 10000.0f, 1);
    /*18*/_pars->insert("lastLinkAltOffset", &_conf.lastLinkAltOffset, 0.0f, 1000.0f, 1);
    /*19*/_pars->insert("lastLink.position.lat", &_state.lastLinkPos, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0, 6, true);
    /*20*/_pars->insert("lastLink.position.lon", &_state.lastLinkPos, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0, 6, true);
    /*21*/_pars->insert("linkBroken", &_state.linkBroken, true);
    /*22*/_pars->insert("fteMaxCharge", &_conf.fteMaxCharge, 0.0f, 100.0f, 2, false, UAF_FTE_INIT);
    /*23*/_pars->insert("fteDfDischargeRate", &_conf.fteDfDischargeRate, 0.0f, 10.0f, 2, false, UAF_FTE_INIT);
    /*24*/_pars->insert("fteRetAirspeed", &_conf.fteRetAirspeed, 0.0f, 150.0f, 1, false, UAF_FTE_INIT);
    /*25*/_pars->insert("fteRetSink", &_conf.fteRetSink, 0.0f, 10.0f, 2, false, UAF_FTE_INIT);
    /*26*/_pars->insert("fteEndAgl", &_conf.fteEndAgl, 0.0f, 1000.0f, 1, false, UAF_FTE_INIT);
    /*27*/_pars->insert("retOnLowEnergy", &_conf.retOnLowEnergy);
    /*28*/_pars->insert("scriptNo", &_conf.scriptNo, 0, 100000);
    /*29*/_pars->insert("upgrInProgress", &_conf.upgrInProgress);
    /*30*/_pars->insert("engCtrlMinI", &_conf.engCtrlMinI, 0.0f, 1000.0f);
    /*31*/_pars->insert("engCtrlNChk", &_conf.engCtrlNChk, 0, 20);
    /*32*/_pars->insert("gpsBadTimeout", &_conf.gpsBadTimeout, 0.0f, 100.0f);
    /*33*/_pars->insert("gpsBad", &_state.gpsBad, true);
#if USE_DGPS == 1
	/*34*/_pars->insert("dgpsBadTimeout", &_conf.dgpsBadTimeout, 0.0f, 100.0f);
	/*35*/_pars->insert("dgpsBad", &_state.dgpsBad, true);
#endif
    /*36*/_pars->insert("pingIgnoreTimeout", &_conf.pingIgnoreTimeout, 0.0f, 100000.0f);
	/*37*/_pars->insert("testLostLink", &_conf.testLostLink);
#if EMRG_LOSS_ENGINE == 1
	/*39*/_pars->insert("focusLandAlt",      &_conf.f32FocusLandAltitude, -10000.0f, 10000.0f);
	/*40*/_pars->insert("emrgRpm",           &_conf.emrgRpm,                      0,    10000);
	/*41*/_pars->insert("lossEngineTimeout", &_conf.lossEngineTimeout,         0.0f, 10000.0f);
    /*47*/_pars->insert("useRpmforLostEngine", &_conf.useRpmforLostEngine);
#endif
    /*42*/_pars->insert("testHIL",           &_conf.bTestHIL);
	/*43*/_pars->insert("workingTime", &_conf.fWorkingTime, 0.0f, 100000.0f);
	/*44*/_pars->insert("timeMaxbyBatt", &_conf.fTimeMaxbyBatt, 0.0f, 100000.0f);
	/*45*/_pars->insert("fuelConsumption", &_conf.fFuelConsumption, 0.0f, 100.0f);
	/*46*/_pars->insert("timeUpdateTimeLeft", &_conf.fTimeUpdateTimeLeft, 0.0f, 10000.0f);
    
	Log.bootPrint ("OK" CRLF);
}

/** \name Method fills an empty object with current subsystem data. Note: Current data subsystem are updated at specified intervals so they can be out of date.
* \param Reference to an SysMonData object in order to set its data by copying the object.
* \return "true" if SysMonData object data is set, "false" when setting the data has been locked by another thread (low probability).
*/
bool SystemMonitor::getSysMonData (SysMonData &smd)
{
    if (!_vsem.lock ())
        return false;

    smd = _smData;

    if (!_vsem.unlock ())
        return false;

    return true;   
}

/** \name Fasade for private implemented method '_isReadyForRun' testing out general auto-pilot system readiness to engage the statup phase of flight.
* \param bool argument enabling error logging while 'true', disabling this ability otherwise.
* \return 'True' if system is ready, 'false' otherwise.
*/
bool SystemMonitor::isReadyForRun (bool toLog)
{
    return _isReadyForRun (NULL, toLog);
}

/** \name Private implementation of a 'isReadyForRun' and is responsible for checking the readiness of ServoManager, PhysicalState, SystemMonitor, FlightControl,
* FlightPlanRealizer, FlightPlan, and CameraManager subsystems to the initiation of the flight.
* \param Pointer to a ClassifiedLine type ('pcl').
* \param Bool argument enabling error logging when 'true', disabling logging otherwise.
* \return 'True' if all subsystems are ready for take-off, 'false' otherwise.
*/
bool SystemMonitor::_isReadyForRun (ClassifiedLine* pcl, bool fLogComm)
{
    bool bok = true;
	if (FPReal->getTuneEnable())
	{
		if (_conf.bTestHIL)
		{
    		if (PState->getSimLevel () > 0)
			{
    			return true;
			}
		}
	}

    // Subsystems readiness checking. Optionally error messages are send by subsystems to flash and communication channel if pcl is null,
	// or to the pcl channel otherwise.
	if (!ServMan->isReadyForTakeoff(pcl, fLogComm))
        bok = false;
    if (!PState->isReadyForTakeoff(pcl, fLogComm))
        bok = false;
    if (!SysMon->isReadyForTakeoff(pcl, fLogComm))
        bok = false;
    if (!FControl->isReadyForTakeoff(pcl, fLogComm))
        bok = false;
    if (!FPReal->isReadyForTakeoff(pcl, fLogComm))
        bok = false;
    if (!FPlan->isReadyForTakeoff(pcl, fLogComm))
        bok = false;
   
    return bok;
}

/** \name Method initializes the Flight Time Estimator parameters by setting the maxCharge, dfDischargeRate, retAirspeed, and retSink.
* It can be called automatically when you change the parameters of the subsystem by SET
*/
void SystemMonitor::initFTE (void)
{
    //  1mAh/s = 0.001*Ah/s = 0.001*3600*As/s = 3.6A
    _state.fte.setParameters (_conf.fteMaxCharge * 3600.0f, _conf.fteDfDischargeRate * 3.6f,
        _conf.fteRetAirspeed, _conf.fteRetSink);
}

/**  \name Method checks if radio link is broken.
* \return 'true' if radio link is broken, 'false' otherwise.
*/
bool SystemMonitor::isLinkBroken (void) const
{
    return _state.linkBroken;
}

/** \name Method sets GPS position in the 'pos' argument and altitude in the 'altitude' argument after receiving last ping command form SystemMonitor subsystem.
* \return 'True' if getting last gps position and altitude was succesufll, 'false' when getting the data was lock by other thread.
*/
bool SystemMonitor::lastLinkPos (GpsPosition& pos, float& altitude)
{
    if (!_vsem.lock ())
        return false;

    pos = _state.lastLinkPos;
    altitude = _state.lastLinkAltitude;

    if (!_vsem.unlock ())
        return false;

    return true;   
}

/** \name Method checks readiness for take-off of SystemMonitor subsystem. Optionally error messages are send to 'cl' if it is not null.
* Method is executed in the context of different subsystem, pay attention while concurency programming.
* \return true if subsystem is ready for take-off, false otherwise.
*/
bool SystemMonitor::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    bool bok = true;

    //  Checking the flags start update.
	if (_conf.upgrInProgress)
    {
        Log.msgPrintf (cl, fLogComm, MSG_UPG_IN_PRGRS);
        bok = false;
    }

    //  Check the flash configuration is loaded.
    if (!_isConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_CONFIG);
        bok = false;
    }
/*
    if (_ig < _conf.maxUpperIBat)
    {
        Log.msgPrintf (cl, fLogComm, MSG_AUXBAT_DCHARGE);
        bok = false;
    }

	// Check the lower battery voltage.
	// Check if the voltage does not drop below the required value, eg due to poor contact. Flag '_mainVoltageDrop' is set to true
	// once the voltage has dropped below required value.
    if (_mainVoltageDrop)
    {
        Log.msgPrintf (cl, fLogComm, MSG_VBAT_NO_TAKEOFF);
        bok = false;
    }
*/

   	// Startup protection after landing (to be sure reboot is required)
    if (_state.landed)
    {
        Log.msgPrintf (cl, fLogComm, MSG_LANDED_REBOOT);
        bok = false;
    }

	// Check if the global parameters has been loaded from flash
    if (!GlobalParamsLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_GPARS);
        bok = false;
    }

    return bok;
}

/** \name Method puts new command line into command queue and sends internal notification.
* \return true if succeeded, false otherwise (e.g. queue is full)
*/
bool SystemMonitor::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("SystemMonitor_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }
    
	// Notification of putting line to the commands queue
	// NOTE: This method exetutes routines from different subsystem (_cmdq.cmdPut(cl)) but notification (EXT_CMD) is sending from SysMon subsystem.
	// Observer treats it as SysMon notification.
    notify (EXT_CMD);

    return ret;
}

/** \name Method checks current and voltage levels from both batteries, using for this purpose previously stored sensors values.
*/
void SystemMonitor::getBattery(float& currentUp, float& voltageUp, float& currentDown, float& voltageDown, int& status) const
{
    currentUp   = _ig;
    voltageUp   = _vg;
    currentDown = _id;
    voltageDown = _vd;
    status      = _rs485Status;
}

/** \name Method get fuel consumption values.
*/
void SystemMonitor::getFuelConsumption(float& fuelConsumption)
{
	fuelConsumption   = _conf.fFuelConsumption;
}

/** \name Method makes specific subsystem component observable by SystemMonitor class (the observer)
*/
void SystemMonitor::linkObserver(void)
{
	// Registration of PState subsystem as a source clock signal
    _pStateTag = registerSubj (PState, PSTATE_CHANGED);

    // Self registration which allows to insert command lines (putLine(..)) into the command queue.
    _extCmdTag = registerSubj (this, EXT_CMD);

	// Registration of FPReal subsystem which sends notification of finalize the landing procedure.
    _fPRealLndTag = registerSubj (FPReal, FPR_LANDED);

	// Registration of FPReal subsystem which sends notification of finalize take-off procedure.
    _fPRealTkfTag = registerSubj (FPReal, FPR_TAKEOFF_CPL);
}

/** \name Method performs specific MicroC/OS-II operating system task.
* First subsystem configuration data are loaded, and flight time estimator is initialized.
*/
void SystemMonitor::task(const void* pdata)
{
	// Loading subsustems configuration data. Errors are printed out by 'confLoad' function if any.
	// If error occurs 'confLoad' sets the default values.
    confLoad();
    initFTE();	///< Flight Time Estimator initialization.
    resetCntSave();

    while(true)
    {
		// Waiting for the notofication.
        OSBase::EvtMask f = waitForAnyAspect ();
	
		// Reading values form sensors.
        if (checkAspect (f, _pStateTag))
        {
            usePstate ();
        }

		// Commnd has been put to the command queue
        if (checkAspect (f, _extCmdTag))
        {
            ClassifiedLine cl;

			// Iterating through the queued commands
            while (_cmdq.cmdGet(cl))
            {
                if (STRNICMP (cl.getLine(), "smon", 4) == 0 && (cl.getLine()[4] == 0 || isspace (cl.getLine()[4]) != 0))
                {
                    useCmdLine (cl);
                }
                else
                {
					// Each transmission monitoring for diagnostic purposes.
                    monitorLink (cl);
                }
            }
        }

        // Finalize of the landing (support for the emergency landing)
        if (checkAspect (f, _fPRealLndTag))
        {
            useFPRealLnd ();
        }

		// Finalize of the take-off
        if (checkAspect (f, _fPRealTkfTag))
        {
            Log.msgPrintf("%sStart calculating time left from take off: %d", SUBS_PREFIX, _startTime100);
            useFPRealTkf ();
        }

    }
}

/** \name Method parses the command line from its argument 'cl'.
*/
void SystemMonitor::useCmdLine (ClassifiedLine &cl)
{
	if (!_parser.loadLine(cl.getLine()))
    {
        Log.errorPrintf("SystemMonitor_useCmdLine_1");
        return;
    }

	// Parsing the 'smon ping' command as first because of the most frequent use of it.
    if (_parser.count() >= 2 &&
        STRICMP (_parser.getToken(1), "ping") == 0)
    {
        if (_parser.count() == 2)
        {
			// Pinging without parameter
            ping(false, 0);
            cl.answer(ERR_OK);
            return;  
        }
        else if (_parser.count() == 3)
        {
			// Pinging with parameter
            int t = 0;
            if (TypeParser::toInt(_parser.getToken(2), t) == true)
            {
                if (ping(true, t) == true)
                {
                    cl.answer(ERR_OK);
                }
                return;  // Pings with incorrect time values or timeouts are ignored.
            }
        }
    }

	// Parsing the 'smon test memory' command
    else if (_parser.count() == 3 &&
        STRICMP (_parser.getToken(1), "test") == 0 &&
        STRICMP (_parser.getToken(2), "memory") == 0)
    {
		// Test configuration memory on a special test file (flash for NIOS).
        if (!_configMemTest->saveTest("test"))
        {
            cl.answer(ERR_CONFIG_MEM_TEST);
            return;
        }
        
		// Test on currently opened and used log file (Flash for NIOS)
        Log.logDisable();       // Log write protection (for other subsystems)
        if (!LogStor->appendTest("log"))
        {
            Log.logEnable();
            cl.answer(ERR_LOG_MEM_TEST);
            return;
        }
        Log.logEnable();       // Log write protection (for other subsystems).
        
        cl.answer(ERR_OK);
        return;
    }

	// Parsing the 'smon save config' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "save") == 0 &&
        STRICMP (_parser.getToken(2), "config") == 0)
    {
        if (confSave() && gParSave())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_CSAVE);
        return;
    }

	// Parsing the 'smon save sn <value>' command
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "save") == 0 &&
        STRICMP (_parser.getToken(2), "sn") == 0)
    {
        if (serialSave(_parser.getToken(3)))
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_CSAVE);
        return;
    }

	// Parsing the 'smon save resetcnt' command
    else if (_parser.count() == 3 &&
        STRICMP (_parser.getToken(1), "save") == 0 &&
        STRICMP (_parser.getToken(2), "resetcnt") == 0)
    {
        if (resetCntSave())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_CSAVE);
        return;
    }

	// Parsing the 'smon set <name> <value>' command
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

	// Parsing the 'smon get <name>' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "get") == 0)
    {
        if (getParameter (_parser.getToken(2), cl))
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_BADPAR);
        return;
    }

	// Parsing the 'stack' command
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "stack") == 0)
    {
        checkAllStacks (&cl, false);
        cl.answer(ERR_OK);
        return;
    }

	// Parsing the 'stack log' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "stack") == 0 &&
        STRICMP (_parser.getToken(2), "log") == 0)
    {
        checkAllStacks (&cl, true);
        cl.answer(ERR_OK);
        return;
    }
    
	// Parsing the 'reboot' command
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "reboot") == 0)
    {
        if (SystemNowOnGround || PState->getSimLevel() > 0)
            Log.abort ("User initiated reboot.");
        else
            cl.answer(ERR_REBOOT);
        
        return;
    }

	// Parsing the 'check' command
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "check") == 0)
    {
        if (_isReadyForRun (&cl, true))
            cl.answer(ERR_OK);
        else
            cl.answer(MSG_NOT_READY);
        return;
    }

	// Parsing the 'smon flash ...' command
    else if (_parser.count() >= 2 && STRICMP (_parser.getToken(1), "flash") == 0)
    {
        if (!SystemNowOnGround)
        {
            Log.msgPrintf ("%sCannot flash program in flight", SUBS_PREFIX);
            return;
        }
		FLASH_PROG_RESULT flash_prog_res = _flashProgrammer.flashCmd(&cl, &_parser);
        if (flash_prog_res == FLASH_PROG_SILENT)
            return; 
        
        switch (flash_prog_res) 
        {
            case FLASH_PROG_OK:				 cl.answer(ERR_OK,              false, true); break;
            case FLASH_PROG_UCOMMAND_ERROR:  cl.answer(ERR_UCOMMAND,        false, true); break; 
            case FLASH_PROG_BADPAR_ERROR:    cl.answer(ERR_BADPAR,          false, true); break;
            case FLASH_PROG_NOT_PREP_ERROR:  cl.answer(ERR_FLASH_NOT_PREP,  false, true); break;
            case FLASH_PROG_LEN_ERROR:       cl.answer(ERR_FLASH_BAD_LEN,   false, true); break; 
            case FLASH_PROG_ADDR_ERROR:      cl.answer(ERR_FLASH_BAD_ADDR,  false, true); break;
            case FLASH_PROG_SDRAM_CRC_ERROR: cl.answer(ERR_FLASH_CRC_SDRAM, false, true); break;
            case FLASH_PROG_WRITE_ERROR:     cl.answer(ERR_FLASH_WRITE,     false, true); break;
            case FLASH_PROG_CRC_ERROR:       cl.answer(ERR_FLASH_CRC,       false, true); break;
            case FLASH_PROG_INTERNAL_ERROR:  cl.answer(ERR_FLASH_INTERNAL,  false, true); break;
            default:                         cl.answer(ERR_FLASH_INTERNAL,  false, true); break;
        }
        return;
    }

	// Parsing the 'smon motorbrake on|off' command
    else if (_parser.count() == 3 && STRICMP (_parser.getToken(1), "motorbrake") == 0)
    {
        if (STRICMP (_parser.getToken(2), "on") == 0)
        {
            PlatformLayer::setMotorBrake (true);
            cl.answer(ERR_OK);
            return;
        }                        
        else if (STRICMP (_parser.getToken(2), "off") == 0)
        {
            PlatformLayer::setMotorBrake (false);   
            cl.answer(ERR_OK);
            return;
        }
        else
        {
            cl.answer(ERR_UCOMMAND);
            return;
        }
    }

	// Parsing the 'bat' command
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "bat") == 0)
    {
        char buf[LINESIZE];

        SNPRINTF (buf, sizeof(buf), "%sVup=%.2fV (%.2fV/c), Iup=%.3fA, Vdown=%.2fV (%.2fV/c), Idown=%.2fA", SUBS_PREFIX, _vg, _vg/3.0f, _ig, _vd, _vd/12.0f, _id);
        cl.answer (buf, false, false);
        cl.answer(ERR_OK);
        return;
    }
    else
        ;   // Null statement as a formality

    // Unknonw command handling
    cl.answer(ERR_UCOMMAND);
}

/** Method handles the PSTATE_CHANGED notification.
*/
void SystemMonitor::usePstate (void)
{  
    float SAMPLE_TIME = 1.0f / PState->getNotifyCyclesPerSecond();

	// read voltages and currents from both batteries (lower and upper one)
    Gauge::readBattery (_ig, _vg, _id, _vd, _rs485Status);

	// Calculation of energy consumed from the start of the system
    _state.qd += _id * SAMPLE_TIME;
	// Check for short voltage drop and set '_mainVoltageDrop' flag if occured
    if (_vd < _conf.minLowerVBat)
        _mainVoltageDrop = true;

	// Check the engines controller
    if (_idCounter > 0)
    {
		// If current level of the lower battery is greater then specified limit value then controller can start the engine.
        if (_id > _conf.engCtrlMinI)
            _idCounter = 0;
        else
            if (--_idCounter == 0)
                notify (SYSMON_ENG_CTRL);
    }

	// Check the status of all subsystems
    checkSystemHealth ();

	// Check the status of system at the specified number of cycles
    if (_conf.checkDivisor != 0 && ++_checkCounter % _conf.checkDivisor == 0)
    {      
        checkSystemSlow ();
        _checkCounter = 0;
    }

	// Check radio connection
    checkLink();

	if(_fteIsCalculated)
	{
		//  Sprawdzenie niskiego poziomu energii
		checkLowEnergy();
	}
 
	//Validation of GPS
    checkGps();

	//Validation arm/disarm
	checkDisarmEngine();
/*
	// Calculate Tmax
	//if(!_bTmaxEstimated && _auxSensorData.b_hubDataAvailable)
	//{
	//	calculateMaxWorkingTime(_timeMax);
	//	_bTmaxEstimated = true;
	//}
	if(!_bUpdateTimeLeft && ((static_cast<float>(_psd.time100 - _startTime100) * 0.0001f) > _conf.fTimeUpdateTimeLeft))
	{
		_bUpdateTimeLeft = true;
	}

	if((_auxSensorData.bAllowEstTimeLeft) && _bUpdateTimeLeft)
	{
		calculateMaxWorkingTime(_timeMax);
		if(!_bTmaxEstimated && (_auxSensorData.i_fuelLevel > 0) && _auxSensorData.b_hubDataAvailable)
			_bTmaxEstimated = true;
		_bUpdateTimeLeft = false;
		_startTime100 = _psd.time100;
	}
*/

	// Sending telemetry at the specified time (other then checking the system status)
    sendTlm();
#if EMRG_LOSS_ENGINE == 1
	checkEngine();
#endif
}

/** \name Method handles 'FPR_LANDED' notification (the end of the landing)
*/
void SystemMonitor::useFPRealLnd (void)
{
	// Setting the 'SystemNowOnGround' flag means that plane is on ground
    SystemNowOnGround = true;
    Log.msgPrintf("%sSystem is on ground", SUBS_PREFIX);
    
	// Turning off 12V power supply for engine controller
    PlatformLayer::setMotorBrake (false);
    Log.msgPrintf("%sDisable motor brake sustainer", SUBS_PREFIX);

	// Notice all the subsystems that activities related to landing have been completed
    notify (SYSMON_LANDED);
    
	// Writing information about remaining stacks capacities to the log
    checkAllStacks (NULL, true);

	// Sending notification of landing to the base station
    Log.eventPrint (EVT_SM_LANDED, "");

	// Blocking the possibility of starting (reboot is needed)
    _state.landed = true;
}

/** \name Method handles the take-off complete ('FPR_TAKEOFF_CPL') notification.
*/
void SystemMonitor::useFPRealTkf (void)
{
	// Calculation initialization of the current consumption during flight.
	// It takes place here because current consumption during take-off is not reliable.
    _state.fte.startMeasure();
/*
	//float Tmax;
	//calculateMaxWorkingTime(Tmax);
	//if(_bTmaxEstimated)
	//	_timeMax = Numbers::minF(Tmax,_timeMax);
	calculateMaxWorkingTime(_timeMax);
	_startTime100 = _psd.time100;
*/
}

/** \name Method sets values for parameters passed as string of pairs of name and coresponding value.
* In the case of a failed initialization value does not change.
* \params String representing the list of pairs of name and value.
* \return Setting parameter result of type 'ParameterNames::ERRCODE', in the case of success 'ParameterNames::ERR_OK' is returned.
*/
ParameterNames::ERRCODE SystemMonitor::setParameters (const char* nameValueItems)
{
    unsigned int flags=0;
    ParameterNames::ERRCODE err = _pars->setParams (nameValueItems, &flags);
	// Executing the functions specified by flags
    if (err == ParameterNames::ERR_OK)
    {
        if ((flags & UAF_FTE_INIT) != 0)
            initFTE();
        if ((flags & UAF_SET_TTY) != 0)
            Log.setCommDeviceByNo (GPar.commTtyNo);
    }

    return err;
}

/** \name Method sends the value of configuration parameter as a text to the specified device.
* \param String ('pName') representing the name of parameter. All parameters are signed by an asterisk ('*').
* \param Reference to communication channel ('cl').
*/
bool SystemMonitor::getParameter (const char* pName, ClassifiedLine& cl) const
{
    return _pars->getParam(pName, cl);
}

/** \name Method reads communication data of a subsystem.
* In case of error default values are restored (error could corrupt actual values).
*/
bool SystemMonitor::confLoad(void)
{
    _isConfigLoaded = _confMem->loadFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!_isConfigLoaded)
    {
		// Restoring default configuration after an error
        _conf.setDefault ();
        Log.errorPrintf("SystemMonitor_confLoad_1");
    }
	_startTime100 = _psd.time100;
	_bUpdateTimeLeft = true;
	Log.msgPrintf("%sStart calculating time left from starting system: %d", SUBS_PREFIX, _startTime100);
    return _isConfigLoaded;
}

// Saving configuration data of a subsystem
bool SystemMonitor::confSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _confMem->saveFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!bok)
        Log.errorPrintf("SystemMonitor_confSave_1");

    return bok;
}

// Saving global parameters
bool SystemMonitor::gParSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool result = _confMem->open(GPAR_FILE_NAME);

    if(!result)
    {
        Log.errorPrintf("SystemMonitor_gParSave_1");
        return false;
    }

    result = _confMem->save(&GPar, sizeof(GPar));
    _confMem->close();

    if(!result)
    {
        Log.errorPrintf("SystemMonitor_gParSave_2");
        return false;
    }

    return true;
}

// Saving the serial number
bool SystemMonitor::serialSave (const char* pValue)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    int v = 0;
    if (!TypeParser::toInt (pValue, v))
        return false;
    SerialNo = static_cast<unsigned int>(v);

    bool result = _confMem->open(SERIALNO_FILE_NAME);

    if(!result)
    {
        Log.errorPrintf("SystemMonitor_serialSave_2");
        return false;
    }

    result = _confMem->save(&SerialNo, sizeof(SerialNo));
    _confMem->close();

    if(!result)
    {
        Log.errorPrintf("SystemMonitor_serial_3");
        return false;
    }

    return true;
}

// Reset Counter
bool SystemMonitor::resetCntSave ()
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool result = _confMem->open(RESETCNT_FILE_NAME);

    if(!result)
    {
        Log.errorPrintf("SystemMonitor_resetCntSave_1");
        return false;
    }

    ResetCounter += 1;
    result = _confMem->save(&ResetCounter, sizeof(ResetCounter));
    _confMem->close();

    if(!result)
    {
        Log.errorPrintf("SystemMonitor_resetCntSave_2");
        return false;
    }

    return true;
}

// Sending/writing telemetry data (the line) to the log at the specified time intervals
void SystemMonitor::sendTlm (void)
{
    char buf[LINESIZE];
    bool linePrepared = false;

	// Sending data to the log
    if (_conf.logDivisor != 0 && ++_logCounter % _conf.logDivisor == 0)
    {
        if (prepareTlmLine (buf, sizeof(buf)))
        {
            Log.tlmPrint (TLM_SMON, buf, true, false);
            linePrepared = true;
        }
        else
            Log.errorPrintf("SystemMonitor_sendTlm_1");

        _logCounter = 0;
    }

	// Sending data to the communication channel
    if (_conf.commDivisor != 0 && ++_commCounter % _conf.commDivisor == 0)
    {
		// If in the same cycle has been prepared data to log, will be used here
        if (linePrepared || prepareTlmLine (buf, sizeof(buf)))
            Log.tlmPrint (TLM_SMON, buf, false, true);
        else
            Log.errorPrintf("SystemMonitor_sendTlm_2");

        _commCounter = 0;
    }
}

/** \name Method prepares text line with telemetry data.
* \param 'buf' is an output text buffer.
* \param 'bufSize' is the size of the output buffer.
*/
bool SystemMonitor::prepareTlmLine (char* buf, int bufSize) const
{
    float timeLeft = 0.0f;
    float userTimeLeft = 0.0f;

	// The semaphore isn't needed here.
    _state.fte.getTimeLeft (timeLeft, userTimeLeft);

//    SysMonTlm smt(PState->getTime100(), _ig, _vg, _psh, _id, _vd, _state.qd, _state.lastLinkTime100,
//        timeLeft, userTimeLeft,
//        _psd.dtUtc.year, _psd.dtUtc.month, _psd.dtUtc.day,
//        _psd.dtUtc.hours, _psd.dtUtc.mins, _psd.dtUtc.secs, _psd.dtUtc.msecs,
//		SystemNowOnGround, _psd.dtUtc.valid, _auxSensorData.i_fuelLevel);



    SysMonTlm smt(PState->getTime100(), _psd.cpuTemp,_vg , _psh, _id, _vd, _state.qd, _state.lastLinkTime100,
        timeLeft, userTimeLeft,
        _psd.dtUtc.year, _psd.dtUtc.month, _psd.dtUtc.day,
        _psd.dtUtc.hours, _psd.dtUtc.mins, _psd.dtUtc.secs, _psd.dtUtc.msecs,
		SystemNowOnGround, _psd.dtUtc.valid, _auxSensorData.i_fuelLevel);

    if (!Base64::encode (&smt, sizeof(smt), buf, bufSize))
    {
        Log.errorPrintf("SystemMonitor_prepareTlmLine_1");
        return false;
    }

    return true;
}

/** \name Method sets error flag of physical state subsystem.
*/
bool SystemMonitor::setPSErrorFlag (const PStateHealth& psh, float ig, float vd)
{
    if (!_vsem.lock ())
        return false;

	// Reseting the DATA_NOT_READY flag (flag is set by default)
    _smData.errFlag = static_cast<SysMonData::SysErrorFlag> 
        (_smData.errFlag & ~SysMonData::ERR_DATA_NOT_READY);

	// It is assumed that 2D mode is allowed during flight, but on the ground the GPS must be fully operational (3D)
    bool bGps = false;
    if (SystemNowOnGround)
        bGps = (psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_OK);
    else
        bGps = (psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_OK ||
                psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_2D_MODE);

    bool bSens = (psh.sensorsError == PStateHealth::sensorsErrorCode::ERR_SENSORS_OK);
#if USE_DGPS == 1
    bool bDGps = (psh.dgpsError == PStateHealth::ERR_DGPS_OK);
#endif
    //  GPS
    _smData.errFlag = static_cast<SysMonData::SysErrorFlag> 
        (bGps ? _smData.errFlag & ~SysMonData::ERR_GPS_BAD :
        _smData.errFlag | SysMonData::ERR_GPS_BAD);

    //  Sensors
    _smData.errFlag = static_cast<SysMonData::SysErrorFlag> 
        (bSens ? _smData.errFlag & ~SysMonData::ERR_SENSORS_FAILURE :
        _smData.errFlag | SysMonData::ERR_SENSORS_FAILURE);
#if USE_DGPS == 1
    //  DGPS
    _smData.errFlag = static_cast<SysMonData::SysErrorFlag> 
        (bDGps ? _smData.errFlag & ~SysMonData::ERR_DGPS_BAD :
        _smData.errFlag | SysMonData::ERR_DGPS_BAD);
#endif
	// Upper battery current
    _smData.errFlag = static_cast<SysMonData::SysErrorFlag> 
        (ig >= _conf.maxUpperIBat ? _smData.errFlag & ~SysMonData::ERR_IBAT_NO_TAKEOFF :
        _smData.errFlag | SysMonData::ERR_IBAT_NO_TAKEOFF);

	// Lower battery voltage.
	// If voltage is to low an apropriate error flag is set. To reset the flags system must be restarted.
    if (vd < _conf.minLowerVBat)
        _smData.errFlag = static_cast<SysMonData::SysErrorFlag> 
        (_smData.errFlag | SysMonData::ERR_VBAT_NO_TAKEOFF);

    if (!_vsem.unlock ())
        return false;

    return true;
}

/** \name Method initializes error flags before system checking.
*/
bool SystemMonitor::initErrorFlag (void)
{
    if (!_vsem.lock ())
    {
        return false;
    }

	_smData.errFlag = static_cast<SysMonData::SysErrorFlag>(_smData.errFlag | SysMonData::ERR_DATA_NOT_READY);

    if (!_vsem.unlock ())
    {
        return false;
    }

    return true;
}

/** \name Diagnostics method performed in each cycle.
*/
void SystemMonitor::checkSystemHealth (void)
{
	// Initializaton of an error flags (as data not ready), other flags remains unchanged.
    initErrorFlag ();

	// Checking condition of physical state subsystem.
    if (PState->getPStateHealth(_psh))
    {
		// Setting of an error flags, '_ig' and '_vd' parameters must have current values.
        setPSErrorFlag(_psh, _ig, _vd);
    }
}

/** \name Method checks the state of the whole system within the time specified by the CheckDivisor parameter.
*/
void SystemMonitor::checkSystemSlow (void)
{
	// Reading the values returned form the FlightController subsystem (these values can be used in different functions, so here are read).
 	// Error handling is not necessary here, 'getOutputData' method has such functionality.
    FControl->getOutputData (_oc);

	// Check there is no conflict with the flag 'LinkBroken' and FlightPlanes 'ReturnMode' state (let's say that state is an inconsistent).
    FPlanData fpd;
    if (FPlan->getFPlanData (fpd))
    {
        if (_state.linkBroken &&
            fpd.itemType != FPlanData::ITEM_TYPE_RETURN && !SystemNowOnGround)
        {
			// Link is broken and current flight plan instruction isn't one of the returning instructions.
			// Such situation can happen right after setting the 'LinkBroken' flag while FPlan has lower priority and
			// did not manage to change the instruction - then 'SYSMON_LINK' notification will be sent redundantly
			// (which in genral is not dangerous)
            notify (SYSMON_LINK);

  			// Inconsistency is a system error
            Log.errorPrintf ("SystemMonitor_checkSystem_1");
        }
    }

	// Reading the values from auxiliary sensors.
	// Caution: these values can be delayed by one cycle of the system
    PState->getAuxSensorData(_auxSensorData);

	// Checking the physical state ('PState') subsystem.
    if (PState->getPStateData(_psd))
    {
		// Reinitialization of the broken link (once performed)
        if (!_pStateInitialized)
        {
            if ((_smData.errFlag & SysMonData::ERR_GPS_BAD) == 0 &&
                (_smData.errFlag & SysMonData::ERR_DATA_NOT_READY) == 0)
            {
                _pStateInitialized = true;
                _state.lastLinkTime100 = _psd.time100;
                _state.tmpLastLinkPos = _psd.position;
                _state.lastLinkPos = _psd.position;
                _state.lastLinkAltitude = _psd.altitude;
            }
        }

		// Check whether the flying platform is on the ground. Reseting 'SystemNowOnGround' flag
		// is performed after the last stage of landing (without altitude and speed validation)
        if (SystemNowOnGround && 
            _psd.airspeed > _conf.airborneIasCond && _psd.altitude > _conf.airborneAglCond)
        {
			// Setting the 'SystemNowOnGround' flag means the plane is in flight.
            SystemNowOnGround = false;
            Log.msgPrintf("%sSystem is airborne", SUBS_PREFIX);
		}

		// Calculation of the time remaining to return to base. Base point must be set.
		if(FPReal->isBasePositionSet())
		{
			BaseStationData baseData;
			FPReal->getBaseData(baseData);

			float excessH = _psd.altitude - baseData.baseElvOffset - _conf.fteEndAgl - PState->getAglOffset()
				+ (PState->isElevationOffsetOn() ? PState->getElevationOffset() : 0.0f);
			_fteIsCalculated = _state.fte.calculate(_psd.time100, _psd.position, baseData.basePosition, _psd.wind, excessH, _state.qd, SystemNowOnGround);
		}

	}

	// Engine controller checking.
    checkEngineController ();
}

//  Sprawdzenie zerwania po³¹czenia
/** \name Method validates the radio link.
* \param 'True' if link is active, 'false' if it's broken.
*/
bool SystemMonitor::checkLink (void)
{
	// On the ground link is not checked
    if (!_state.linkBroken && 
        !SystemNowOnGround &&
        _conf.linkBrokenTimeout > 0.0f &&
		PState->getTime100() - _state.lastLinkTime100 > static_cast<int>(_conf.linkBrokenTimeout * 10000.0f))
    {
		// When GPS is not working there is no possibility to find the return flight path
        if ((_smData.errFlag & SysMonData::ERR_GPS_BAD) != 0)
            return false;

		// Refreshing data of the physical state subsystem
        if (!PState->getPStateData(_psd))
        {
            Log.errorPrintf ("SystemMonitor_checkLink_1");
            return false;
        }

		// Angle away from the current point to the point of the last active link
        float tr1 = GpsPosition::track (_psd.position, _state.tmpLastLinkPos);

        if (!_vsem.lock ())
            return false;

		// Moving an intermediate point along the return flight path (as a reserve distances to eventual obstacles)
        bool b = (GpsPosition::movePosition (_state.tmpLastLinkPos, tr1, _conf.lastLinkPosOffset, _state.lastLinkPos));
		// Increasing of the altitude
        _state.lastLinkAltitude = _psd.altitude + _conf.lastLinkAltOffset;

        if (!_vsem.unlock ())
            return false;

        if (b)
        {
            Log.msgPrintf("%sRadio link broken", SUBS_PREFIX);
            _state.linkBroken = true;
			// Notification of link loss
			notify (SYSMON_LINK);
        }
        else
        {
            Log.errorPrintf ("SystemMonitor_checkLink_2");
            return false;
        }
    }
    return true;
}

/** \name Method checks if the battery is low and if so enforces the return mode.
*/
void SystemMonitor::checkLowEnergy (void)
{
    float tl=0.0f;
    float tld=0.0f;

	// Return mode is set when the battery is too low, what means that there is enough energy only for return to landing point
	// (no operation time). Return mode is set only once but after possible improvement of conditions 'retOnLowEnergy' flag can be
	// set manually
    _state.fte.getTimeLeft(tl, tld);
    if (tld <= 0.0f && _conf.retOnLowEnergy && !SystemNowOnGround)
    {
       Log.msgPrintf("%sLow battery energy", SUBS_PREFIX);
	   // Return mode is set only once
        _conf.retOnLowEnergy = false;
        notify (SYSMON_LOW_ENERGY);
    }
}

/** \name Method checks if GPS connection is valid, and when it in not return mode is set.
* Return mode will be enforced whenever a specified time Gps will be out of order.
* The restoration of the of the Gps does not automatically change the flight path.
*/
void SystemMonitor::checkGps (void)
{
    int t = PState->getTime100();

	// Storing the time of last correct reading the value from GPS and reseting the 'gpsBad' flag.
    if ((_smData.errFlag & SysMonData::ERR_GPS_BAD) == 0)
    {
        _state.lastGpsGoodTime100 = t;
        _state.gpsBad = false;
    }
#if USE_DGPS == 1
	// Storing the time of last correct reading the value from DGPS and reseting the 'dgpsBad' flag.
    if ((_smData.errFlag & SysMonData::ERR_DGPS_BAD) == 0)
    {
        _state.lastDGpsGoodTime100 = t;
        _state.dgpsBad = false;
    }
#endif
	// Return mode is enforced when:
	//	- it is not enforced yet
	//  - and flying platform is in flight
	//	- and time of required malfunction of GPS is > 0
	//	- and time of current malfunction of GPS > time of required malfunction of GPS
    if (!_state.gpsBad && 
        !SystemNowOnGround &&
        _conf.gpsBadTimeout > 0.0f &&
        t - _state.lastGpsGoodTime100 > static_cast<int>(_conf.gpsBadTimeout * 10000.0f))
    {
            Log.msgPrintf("%sBad Gps", SUBS_PREFIX);
            _state.gpsBad = true;
			// Notification that GPS has been lost
            notify (SYSMON_NO_GPS);
    }
#if USE_DGPS == 1
	if (!_state.dgpsBad && 
        !SystemNowOnGround &&
        _conf.dgpsBadTimeout > 0.0f &&
        t - _state.lastDGpsGoodTime100 > static_cast<int>(_conf.dgpsBadTimeout * 10000.0f))
    {
            Log.msgPrintf("%sBad DGps", SUBS_PREFIX);
            _state.dgpsBad = true;
			// Notification that DGPS has been lost
            notify (SYSMON_NO_DGPS);
    }
#endif
}

void SystemMonitor::checkDisarmEngine(void)
{
    allowDisarmEngine = true;
    if (_psd.altitude > 5.0f)
    	allowDisarmEngine = false;

    if (_psd.airspeed > 30.0f && _psd.groundspeed > 15.0f)
    	allowDisarmEngine = false;
}

#if EMRG_LOSS_ENGINE == 1
void SystemMonitor::checkEngine (void)
{
	if(!SystemNowOnGround)
	{
		// check if use the rpm sensor or not
		if(_conf.useRpmforLostEngine)
		{
			if (_auxSensorData.i_engineSpeed >= _conf.emrgRpm)
				_lossEngineCounter = 0;
			else 
				_lossEngineCounter ++;

			// check time out for lost engine
			if(_lossEngineCounter > (int)(_conf.lossEngineTimeout * 24.0f))
				_bCheckLostEngine = true;
			else
				_bCheckLostEngine = false;

			// if throttle alert, need to check RPM to confirm
			if(FControl->getThrAlert())
			{
				if (_bCheckLostEngine)
				{
					_bLostEngineAlert = true;
				}
				else
				{
					_bLostEngineAlert = false;
				}
			}

			// if RPM returns to 0, make alert
			if(_bCheckLostEngine)
			{
				_bLostEngineAlert = true;
			}
		}
		else
		{
			// Set all flags using RPM to defalt
			_lossEngineCounter = 0;
			_bCheckLostEngine = false;

			// Depending on throttle alert
			if(FControl->getThrAlert())
			{
				_bLostEngineAlert = true;
			}
			else
			{
				_bLostEngineAlert = false;
			}
		}
		

	}

	if (_bLostEngineAlert)
	{
		if ((_psd.altitude > _conf.f32FocusLandAltitude) && !_bLossEngine)
		{
			_bLossEngine = true;
			Log.errorPrintf("Lost Engine");
			notify (SYSMON_LOSS_ENGINE);
		}

		if ((_psd.altitude < _conf.f32FocusLandAltitude) && !_bFocusLand)
		{
			_bFocusLand = true;
			Log.errorPrintf("Focus Land");
			notify (SYSMON_FOCUS_LAND);
		}
	}
	else
	{
		_bLossEngine = false;
		_bFocusLand = false;
	}
}
#endif

/** \name Method sends information of remaining stasks capacities as a result to the communication channel and log
* \param Pointer to 'ClassifiedLine' object to which message is send as a result (pointer may be null). 
*/
void SystemMonitor::checkAllStacks (ClassifiedLine* pcl, bool toLog) const
{
    checkStack (pcl, toLog, RootTaskPriority,     "Root    ");
    checkStack (pcl, toLog, Tty0InTaskPriority,   "Tty0In  ");
    checkStack (pcl, toLog, Tty1InTaskPriority,   "Tty1In  ");
    checkStack (pcl, toLog, Tty2InTaskPriority,   "Tty2In  ");
    checkStack (pcl, toLog, GpsInTaskPriority,    "GpsIn   ");
    checkStack (pcl, toLog, LDispTaskPriority,    "LDisp   ");
    checkStack (pcl, toLog, PStateTaskPriority,   "PState  ");
    checkStack (pcl, toLog, FControlTaskPriority, "FControl");
    checkStack (pcl, toLog, ServManTaskPriority,  "ServMan ");
    checkStack (pcl, toLog, FPRealTaskPriority,   "FPReal  ");
    checkStack (pcl, toLog, FPlanTaskPriority,    "FPlan   ");
    checkStack (pcl, toLog, SysMonTaskPriority,   "SysMon  ");
    checkStack (pcl, toLog, Tty0OutTaskPriority,  "Tty0Out ");
    checkStack (pcl, toLog, Tty1OutTaskPriority,  "Tty1Out ");
    checkStack (pcl, toLog, Tty2OutTaskPriority,  "Tty2Out ");
    checkStack (pcl, toLog, GpsOutTaskPriority,   "GpsOut  ");
    checkStack (pcl, toLog, LogStorTaskPriority,  "LogStor ");
    checkStack (pcl, toLog, LogManTaskPriority,   "LogMan  ");
}


void SystemMonitor::checkStack (ClassifiedLine* pcl, bool toLog, INT8U taskId, const char* taskName) const
{
    char buf[LINESIZE];

    int bfree = Os->stackMinFree (taskId);
    if (bfree >= 0)
        SNPRINTF (buf, sizeof(buf), "%s%s %5d bytes free", SUBS_PREFIX, taskName, bfree);
    else
        SNPRINTF (buf, sizeof(buf), "%s%s -- unused or error --", SUBS_PREFIX, taskName);

    if (pcl != NULL)
        pcl->answer(buf, false, false);   
    if (toLog)
        Log.msgPrintf("%s", buf);
}

/** \name Method handles the 'ping' command.
* \param 'checkTimeFlag' determines whether the time of the last response from the 'ping' command is to be controlled.
* \param 'lastResponseTime' determines time of the last response to the 'ping' command received by the ground station.
* \return 'True' if 'ping' command was processed successfully, 'false' otherwise.
*/
bool SystemMonitor::ping (bool checkTimeFlag, int lastResponseTime)
{
	// Read the most recent data from the physical state subsystem (GPS data can be invalid)
    if (!PState->getPStateData(_psd))
    {
        Log.errorPrintf ("SystemMonitor_ping_1");
		// Return in the case of error because time and the position may be outdated
        return false;
    }

	if (_conf.testLostLink)
	{
		return false;
	}

	// Check whether the time is to be controlled
    if (checkTimeFlag)
    {
        int t = lastResponseTime * 10000;
		if (t < 0 || t > _psd.time100 || (_psd.time100 - t) > static_cast<int>(_conf.pingIgnoreTimeout * 10000.0f))
        {
			// Pings with invalid parameters or with too big differences of time are ignored
            return false;  
        }
    }

	// Reset flag is always performed even when the errors of the GPS
    if (_state.linkBroken)
    {
        Log.msgPrintf("%sRadio link restored", SUBS_PREFIX);
        _state.linkBroken = false;
		// Notification that radio link has been established.
		notify (SYSMON_LINK);
    }

	// Store the time
	_state.lastLinkTime100 = _psd.time100;

	// GPS is invalid, current position cannot be stored (method returns)
    if ((_smData.errFlag & SysMonData::ERR_GPS_BAD) != 0)
        return true;

	// Storing the current position (is correct if the program came here)
    _state.tmpLastLinkPos = _psd.position;
    return true;
}

/** \name Method monitors all transmissions.
*/
void SystemMonitor::monitorLink (const ClassifiedLine &cl)
{
    if (!_parser.loadLine(cl.getLine()))
    {
        Log.errorPrintf("SystemMonitor_monitorLink_1");
        return;
    }

	// All FPlan subsystems commands works similarly to a 'ping' command except 'fp status' command. This is necessary because
	// some of the FPlan subsystems commands exits the return mode (_returnMode) and there must ba a way to re-entry thie mode
	// while link is broken. This is possible when 'linkBroken' flag is set to 'true'.
    if (_parser.count() > 0 && STRICMP (_parser.getToken(0), "fp") == 0)
    {
        ping(false, 0);
    }
}

/** \name Method checks the engine controller.
* Engines controller afret reset is setup with current PWM settings as reference(engine is off).
* Reboot the controller in the air stops the engine even when it was turned on before. To make the engine works correctly a low value of the PWM must be set for a certain time.
* The controller resets itself for no reason at about 2500 seconds.
*/
void SystemMonitor::checkEngineController (void)
{
    _idCounter = 0;

	// If the engine has to be turned on  then the number of cycles in which will be checked lower battery current must be set.
	// If the currents value for each of these samples will be lower then threshold current, then an controllers error will be notified.
    if (_oc.fCtrl.throttle > 0.0f)
        _idCounter = _conf.engCtrlNChk;
}

#if EMRG_LOSS_ENGINE == 1
bool SystemMonitor::getLossEngine (void)
{
	return _bLostEngineAlert;
}
#endif
bool SystemMonitor::getTestHIL(void)
{
    return _conf.bTestHIL;
}

//=========================================================================
//  Class SmonConfigData
//=========================================================================

/** \name Configuration data class constructor
*/
SystemMonitor::SmonConfigData::SmonConfigData(void)
{
    setDefault ();
}

void SystemMonitor::calculateMaxWorkingTime (float &Tmax)
{
	float V_servobat = Numbers::minF(_auxSensorData.f_voltage_1 * 0.5f, _auxSensorData.f_voltage_2 * 0.5f);
	//float V_bat = Numbers::minF(V_servobat,_vg * 0.33333f);

	// Polynomial fit from the data table
	float V_per = Numbers::fitBatteryLine(V_servobat);
    
    // Limit to sensible value
    Numbers::constrain_float(V_per, 0.0f, 100.0f);
	// Time limited by battery
	float Tleft_bat = _conf.fTimeMaxbyBatt * 36.0f * V_per;

	if(_bTmaxEstimated)
	{	
		// Time limited by Fuel for engine
		float Tleft_fuel;
		if(_auxSensorData.festFuelLevel > 0)
		{
			Tleft_fuel = 3600.0f * (1.0f + 3.5f * _auxSensorData.festFuelLevel / 100.0f) / _conf.fFuelConsumption;
			_bufferTime100 = _psd.time100;
		}
		else
		{
			Tleft_fuel = 1.5f/_conf.fFuelConsumption - static_cast<float>(_psd.time100 - _bufferTime100) * 0.0001f;
		}
		float minT = Numbers::minF(Tleft_bat, Tleft_fuel);
		Tmax = Numbers::minF(minT, _conf.fWorkingTime);
	}
	else
	{
		if(_auxSensorData.i_fuelLevel == 0)
			Tmax = 0.0f;
	}
}


/** \name Method sets default configuration data.
*/
void SystemMonitor::SmonConfigData::setDefault(void)
{
    commDivisor         = 120;
    logDivisor          = 120;
    checkDivisor        =  60;
    airborneAglCond     =  10.0f;
    airborneIasCond     =  40.0f;
    maxUpperIBat        =  -0.2f;
    minLowerVBat        =  30.0f;
    linkBrokenTimeout   =  50.0f;
    lastLinkPosOffset   = 100.0f;
    lastLinkAltOffset   =  50.0f;
    fteMaxCharge        =  10.0f;
    fteDfDischargeRate  =   1.14f;
    fteRetAirspeed      =  70.0f;
    fteRetSink          =   1.2f;
    fteEndAgl           = 100.0f;
    retOnLowEnergy      = true;
    scriptNo            =   0;
    upgrInProgress      = true;
    engCtrlMinI         =   5.0f;
    engCtrlNChk         =   0;   // default: the engine controllers reseting mechanism is turned off (=0)
    gpsBadTimeout       =  15.0f;
#if USE_DGPS == 1
	dgpsBadTimeout      =  15.0f;
#endif
    pingIgnoreTimeout   =  10.0f;

	testLostLink  =   false;
#if EMRG_LOSS_ENGINE == 1
	f32FocusLandAltitude = 50.0f;
	emrgRpm = 1000;
	lossEngineTimeout = 10.0f;
	useRpmforLostEngine = false;
#endif
    bTestHIL = true;
	fWorkingTime = 10800.0f;
	fTimeMaxbyBatt = 5.0f;
	fFuelConsumption = 0.9798f;
	fTimeUpdateTimeLeft = 300.0f;
}
