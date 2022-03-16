/*********************************************************************/
/*                                                                   */
/* FlightPlanRealizer                                                */
/*                                                                   */
/*********************************************************************/

#include <PilotIncludes.h>

/**  @name Definiction and initialization og the static elements.
* @{
*/
const char FlightPlanRealizer::CONF_FILE_NAME[] = "fprealcfg";
const char FlightPlanRealizer::SUBS_PREFIX[]    = "real: ";
const char FlightPlanRealizer::ERR_OK[]           = "real: ok";
const char FlightPlanRealizer::ERR_SYS[]          = "real: RE01 System error";
const char FlightPlanRealizer::ERR_CLOAD[]        = "real: RE02 Cannot load config data";
const char FlightPlanRealizer::ERR_CSAVE[]        = "real: RE03 Cannot save config data";
const char FlightPlanRealizer::ERR_UCOMMAND[]     = "real: RE04 Unrecognized command or wrong number of parameters";
const char FlightPlanRealizer::ERR_BADPAR[]       = "real: RE05 Bad parameter name or value";
const char FlightPlanRealizer::ERR_CMDEXEC[]      = "real: RE06 Command Execute Error";
const char FlightPlanRealizer::ERR_NO_CONFIG[]    = "real: RE07 Config data not loaded from flash";
const char FlightPlanRealizer::ERR_FLAND[]        = "real: RE22 Cannot force land or system on ground";
const char FlightPlanRealizer::ERR_CMD_BUF_FULL[] = "real: RE23 Command buffer full";
const char FlightPlanRealizer::ERR_LAND_UNPREPARED[] = "real: RE24 Landing not prepared";
///@}

/**
* FlightPlanRealizer Constructor realize initialization an array of mappings parameter names to objects... 
*/
FlightPlanRealizer::FlightPlanRealizer(void):
    // initialization in case of linkObserver function has not been called.
    ODTObserver(), ODTSubject(), SubsystemBase(),
	_fPlanTag(-1), _pStateTag(-1), _sysMonTag(-1), _redoTag(-1), _extCmdTag(-1), _servomanErmParachuteTag(-1),
#if USE_DGPS == 1
	_sysMonNoDgpsTag(-1),
#endif
    _mainCounter(0), _cgCounter(0), _commCounter(0), _logCounter(0),
    _isConfigLoaded(false), _pars(NULL), _flightConstraint(NULL), _confMem(NULL), _cmdq(Log), _tlmCamGood(false)
{  
    _pars = new ParameterNames(SUBS_PREFIX, 115);

    if (_pars == NULL)
    {
        Log.abort ("Critical Error: FlightPlanRealizer_1.");
        return;
    }

	_bTuneMode = false;
	_bFirstSetControllers = true;
	_circleToClimb = false;
#if EMRG_LOSS_ENGINE == 1
	_circleToDescent = false;
#endif

    //Initialization an array of mappings parameter names to objects.     
    
    // Controllers Parameters initializactions.
    /* 38*/setControlPropertiseParams(38);

    // FlightControl Parameters initializactions
    /* 61*/setFlightControlParams(23);

    // FlightReference
    /* 88*/setFlightReferenceParams(27);

    // Gps Position
    /*112*/setGpsPosition (27);

    //  Configuration parameters of the flight plan instructions (Separate container).
    _pars->insertContainer("alg.", 328, _alg);
    if (_alg == NULL)
    {
        Log.abort ("Critical Error: FlightPlanRealizer_2.");
        return;
    }

    /* 17*/setDefaultParams (17);
    /* 30*/setHoldModeParams (13);
    /* 42*/setManualTurnModeParams (12);
	/* 54*/setLinkBrokenParams (12);
    /* 67*/setCameraGuideModeParams (13);
	/* 82*/setNetParams (15);

    /*127*/setFlyModeParams (45);
    /*175*/setCrcModeParams (48);
#if LAND_MODE == SEMI_LAND
	/*227*/setSemiLandModeParams (52);
#endif

    /*242*/setForceLandParams (15);
    /*264*/setEmergencyLandParams (22);
    /*281*/setStallRecoveryParams (17);
	/*297*/setAutoTkfCatapultParams (16);
	/*307*/setAutoParachutingParams(10);

    /*327*/setSubsystemParams (20);
    
    //  array of pointers to the functions controlling flight phases (for safety reasons they are reset).
    setFlightPhase ();

    //  Create semaphor controlling acces to the shared subsystems data.
    if (!_vSem.create("FlightPlanRealizer"))
    {
        Log.abort ("Critical Error: FlightPlanRealizer_3.");
        return;
    }

    // Configuration data memory.
    _confMem = StorageFactory::createConfigMemory();
    if( _confMem == NULL)
    {
        Log.abort ("Critical Error: FlightPlanRealizer_4.");
        return;
    }

    Log.bootPrint ("OK" CRLF);
}

/**
* Sharing of data held by the subsystem
* fprd - reference to the object whose fields will be filled
*/
bool FlightPlanRealizer::getFPRealData (FPRealData &fprd)
{
    if (!_vSem.lock ())
        return false;

    // Object copy
    fprd = _state.fprd;

    if (!_vSem.unlock ())
        return false;

    return true;   
}

/** 
*    Share base station position and elevation.
*/
bool FlightPlanRealizer::getBaseData (BaseStationData& based)
{
    if (!_vSem.lock ())
        return false;

    // Object copy
    based.basePosition = _state.base.position;
    based.baseElvOffset = _state.base.elvOffset;
    
    if (!_vSem.unlock ())
        return false;

    return true;   
}

//-------------------------------------------------------------------------
// get the tuning controllers
//-------------------------------------------------------------------------

bool FlightPlanRealizer::getTuningControllers(int &tuningLat, int &tuningLon, int &tuningSpd)
{
    if (!_vSem.lock ())
    {
        return false;
    }

	tuningLat = _ramStorage.tuningLat;
	tuningLon = _ramStorage.tuningLon;
	tuningSpd = _ramStorage.tuningSpd;

    if (!_vSem.unlock ())
    {
        return false;
    }

    return true;
}

//-------------------------------------------------------------------------
// get the tuning controllers
//-------------------------------------------------------------------------
bool FlightPlanRealizer::getTuneEnable (void)
{
	return _ramStorage.bTuneEnable;
}

//-------------------------------------------------------------------------
// get the Track Path Mode
//-------------------------------------------------------------------------
bool FlightPlanRealizer::getUseCorrTrack (void)
{
	return _conf.bUseCorrTrack;
}


/**
*  Register observer objects.
*/
void FlightPlanRealizer::linkObserver(void)
{
    //  Self registration.  Save command to the queue in putLine function.
    //  putLine function is called from other subsystem but it has been seen by observer like been send by FlightGear.
    _extCmdTag = registerSubj (this, EXT_CMD);

    //  FPlan subsystem registration. Subsytem forward notification about new instruction to be realized.
    _fPlanTag = registerSubj (FPlan, FP_CHANGED);

    //  PState subsystem registration. Subsytem forward notification about phisical state has been change (new data read).
    _pStateTag = registerSubj (PState, PSTATE_CHANGED);

    //  SysMon subsystem registration. Subsytem forward notification about finished landing.
    _sysMonTag = registerSubj (SysMon, SYSMON_LANDED);
	//
	_servomanErmParachuteTag = registerSubj(ServMan, SERVMAN_ERM_PARACHUTE);
    //  Current subsystem registration. Subsystem forward notification that reread of Fplan line is needed.
    _redoTag = registerSubj (this, FPR_INTERNAL_REDO);
#if USE_DGPS == 1
	_sysMonNoDgpsTag = registerSubj (SysMon, SYSMON_NO_DGPS);
#endif
}


/**
* Task of the MicroC/OS-II operating system. 
*/
void FlightPlanRealizer::task(const void* pdata)
{
    // Loading subsystem's configuration data (possible erros are written by confLoad function) 
    // In case of error confLoad function restore default values.
    confLoad();
 
    // Set default valuess.
    _state.fprd.fRef.airspeed = _conf.dfAirspeed;
    _state.fprd.fRef.altitude = _conf.dfAltitude;
    _state.fprd.fRef.circleMode = _conf.dfCircleMode;
    _state.fprd.fRef.circleModePar = _conf.dfCircleModePar;

    for (;;)
    {
        //  Waiting for the coming notice.
        OSBase::EvtMask f = waitForAnyAspect ();

        //  Command has been written to queue.
        if (checkAspect (f, _extCmdTag))
        {
            ClassifiedLine cmdCl;

            while (_cmdq.cmdGet (cmdCl))
            {
                useCmdLine (cmdCl);
            }
        }

        // Notice from FPlan subsystem.
        // could not be "else" here because couple of notice could came at the same time.
        if (checkAspect (f, _fPlanTag) ||
            checkAspect (f, _redoTag))
        {
            //  Remember current executed instruction.
            FPlanData fpdTmp = _fpd;

            if (FPlan->getFPlanData (_fpd))
            {
                char buf[30];
                if (SNPRINTF (buf, sizeof(buf), "%i %i %s", _fpd.currentItem, _fpd.currentStatus, _fpd.fplanName) <= 0)
                    Log.errorPrintf("FlightPlanRealizer_task_2");

                // Check "Stopped" mode - for security resons it has a priority.
                if (_fpd.currentStatus == FPlanConf::Stopped)
                    standby();
                else
                    //  Checking if current instruction could be break by other or if could be flight mode changed. (e.g. OnHold)
                    if (_state.doNotBreak)
                    {
                        Log.msgPrintf ("%sCan not break current flight plan instruction.", SUBS_PREFIX);
                        Log.eventPrint (EVT_FPR_REJECT, buf);
                        //  Restore original instruction.
                        _fpd = fpdTmp;
                    }
                    //  In some cases instruction that normaly could be broke, shouldn't be broken by the "return" mode instruction.
                    else if (_state.doNotBreakByRetMode && (_fpd.itemType == FPlanData::ITEM_TYPE_RETURN))
                    {
                        Log.msgPrintf ("%sCan not break current flight plan instruction by RETURN Mode.", SUBS_PREFIX);
                        Log.eventPrint (EVT_FPR_REJECT, buf);
                        //  Restore original instruction.
                        _fpd = fpdTmp;
                    }
                    else
                    {
                        Log.eventPrint (EVT_FPR_ACCEPT, buf);
                        useFPlan();
                    }
            }
        }

        // Notify from PState subsystem.
        if (checkAspect (f, _pStateTag))
        {
            usePState ();
        }

        // Notify from SysMon subsystem that landing had finished.
        if (checkAspect (f, _sysMonTag))
        {
            if (_conf.configAutosave)
            {
                if (confSave())
                    Log.msgPrintf ("%sSave config after landing", SUBS_PREFIX);
                else
                    Log.msgPrintf ("%sError saving config", SUBS_PREFIX);
            }
        }
		// Notify from SysMon subsystem that landing had finished.
		if (checkAspect (f, _servomanErmParachuteTag))
		{
			Log.msgPrintf ("%sEmergency landing parachute", SUBS_PREFIX);
			parachuting();
		}
#if USE_DGPS == 1
		if (checkAspect (f, _sysMonNoDgpsTag))
        {
			if ((_state.nLandPhaseFlag == _state.DESCENT) ||
				(_state.nLandPhaseFlag == _state.LAND))
			{
				Log.msgPrintf ("%sAbort land cause by lost DGPS", SUBS_PREFIX);
				netAbort ();
			}
        }
#endif
    }
}


/**  
* Could current executing instruction be break?
* Do not check "doNotBreakByRetMode" flag.
*/
bool FlightPlanRealizer::isNotBreakable (void) const
{
    // Do not need to secure with semaphore because instruction fits in one word.
    return _state.doNotBreak && (_fpd.currentStatus == FPlanConf::Running);
}

void FlightPlanRealizer::clearNetLandFlag (void)
{
    if ((_state.nLandPhaseFlag != _state.UNKNOWN) &&
        (_fpd.currentStatus == FPlanConf::Running))
    {
        _state.nLandPhaseFlag = _state.UNKNOWN;
    }
}

void FlightPlanRealizer::clearLandParachuteFlag(void)
{
	if ((_state.landParachutePhaseFlag != _state.LND_UNKNOWN) &&
		(_fpd.currentStatus == FPlanConf::Running))
	{
		_state.landParachutePhaseFlag = _state.LND_UNKNOWN;
	}
}

void FlightPlanRealizer::clearTkfFlag(void)
{
	if ((_state.tkfPhaseFlag != _state.TKF_UNKNOWN) &&
		(_fpd.currentStatus == FPlanConf::Running))
	{
		_state.tkfPhaseFlag = _state.TKF_UNKNOWN;
	}
}

/**
* Is "Camera Guide" mode turned on?
*/
bool FlightPlanRealizer::isInCGMode (void) const
{
    // Do not need to secure with semaphore because instruction fits in one word.
    return _state.cameraGuideMode;
}

/**
* Is "Auto Takeoff" mode turned on?
*/
bool FlightPlanRealizer::isAllowEngineSignal (void) const
{
    // Do not need to secure with semaphore because instruction fits in one word.
    return _state.bAllowEngineMsg;
}

/**
* If allow to use ThetaModifier
*/
bool FlightPlanRealizer::isUseThetaModifier (void) const
{
    // Do not need to secure with semaphore because instruction fits in one word.
    return _state.useThetaMod;
}

/**
* If allow to use ThetaModifier
*/
void FlightPlanRealizer::enableThetaModifier (void)
{
    // Do not need to secure with semaphore because instruction fits in one word.
    if (!_state.useThetaMod)
        _state.useThetaMod = true;
    return;
}

/**
* If allow to use ThetaModifier
*/
void FlightPlanRealizer::disableThetaModifier (void)
{
    // Do not need to secure with semaphore because instruction fits in one word.
    if (_state.useThetaMod)
        _state.useThetaMod = false;
    return;
}
/* Have the base position coordinates been set? Needed to consumed energy calculations.
*/
bool FlightPlanRealizer::isBasePositionSet (void) const
{
    return _state.basePosIsSet;
}

/**
* Is "Manual turn" mode turned on? 
*/
bool FlightPlanRealizer::isInMTMode (void) const
{
    // Do not need to secure with semaphore because instruction fits in one word.
    return _state.manualTurnMode;
}

/**
* Return TRUE if system is ready for takeoff. (fully functional).
* Sends error messages to the cl (if cl != NULL)
* Function is called in the context of other subsystem - note on synchronnous
*/
bool FlightPlanRealizer::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    //  Loading configuration from flash checking.
    if (!_isConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, ERR_NO_CONFIG);
        return false;
    }
    return true;
}

/**
* Function writes to command queue new line and sends internal notice.
* Returns TRUE when ok, FALSE when queue i full  (or the line was not written from other reason).
*/
bool FlightPlanRealizer::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("FlightPlanRealizer_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }
    
    // Sending notification about writting new line to queue.
    // NOTE: putLine funtion has being call from other subsystem, but current subsytem funtion is being executed.
    // Observer treats it as FlightPlanRealizer notification.
    notify (EXT_CMD);

    return ret;
}

/**
* Interpretation of the command sended from the communication channel.
*/
void FlightPlanRealizer::useCmdLine(ClassifiedLine &cl)
{
    if (!_parser.loadLine(cl.getLine()))
    {
        Log.errorPrintf("FlightPlanRealizer_useCmdLine_1");
        cl.answer(ERR_SYS);
        return;
    }

    //  Input camera state telemetry interpretation.
    //  There is an exit from function in case of bad format, to prevent from sending error message.
    if (STRICMP (_parser.getToken(0), "&T18:") == 0)
    {
        if (_parser.count() == 2)
            tlmCamShort (_parser.getToken(1));
        return;
    }

    //  Operator command to abort landing
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "abort") == 0)
    {
        netAbort ();
        cl.answer(ERR_OK);
        return;
    } 
       
    //  Command "real turn <value>|off" interpretation
    //  is at the beggining because it is offen use in manual turn mode.
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "turn") == 0)
    {
        //  turn of "manual turn" mode
        if (STRICMP (_parser.getToken(2), "off") == 0)
        {
            turnOff ();
            cl.answer(ERR_OK);
            //  Redo from current flight plan line (send internal notification)
            redoCurrentFPLine ();
        }
        //  turn on
        else if (turn (_parser.getToken(2)))
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_CMDEXEC);

        return;
    }
    
    // "real cg on|off" command interpretation
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "cg") == 0)
    {
        //  turn on "camera guide" mode
        if (STRICMP (_parser.getToken(2), "off") == 0)
        {
            camGuideOff ();
            cl.answer(ERR_OK);
            //  Redo from current flight plan line (send internal notification)
            redoCurrentFPLine ();
            return;
        }
        //  turn on
        else if (STRICMP (_parser.getToken(2), "on") == 0)
        {
            if (camGuideOn ())
                cl.answer(ERR_OK);
            else
                cl.answer(ERR_CMDEXEC);
            return;
        }
        else ;  // empty instruction for formalities 
    }

    // "real disable controllers" command interpretation
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "disable") == 0 &&
        STRICMP (_parser.getToken(2), "controllers") == 0)
    {
        if (disableControllers(false))
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_CMDEXEC);
        return;
    }

    // "real save config" command interpretation
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

    // "real approach" command interpretation
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "approach") == 0)
    {
        netReadyApproach();
        cl.answer(ERR_OK);
     
        return;
    }

    // "real set <name> <value>" command interpretation
    else if (_parser.count() >= 4 && 
        STRICMP (_parser.getToken(1), "set") == 0)
    {
        if (setParameters (cl.getLine() + _parser.getTokenIndex(2), false))
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_BADPAR);
        return;
    }

    // "real get <name>" command interpretation
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "get") == 0)
    {
        if (getParameter (_parser.getToken(2), cl))
            cl.answer(ERR_OK, false, true);
        else
            cl.answer(ERR_BADPAR);
        return;
    }


    // "real force land" command interpretation
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "force") == 0 &&
        STRICMP (_parser.getToken(2), "land") == 0)
    {
        if (forceLand())
            cl.answer(ERR_OK);
        else
            cl.answer(ERR_FLAND);
        return;
    }

    // "ias fault on" command interpretation
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "ias") == 0 &&
        STRICMP (_parser.getToken(2), "fault") == 0 &&
        STRICMP (_parser.getToken(3), "on") == 0)
    {
        iasFault (true);
        cl.answer (ERR_OK);
        return;
    }

    // "ias fault off" command interpretation
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(1), "ias") == 0 &&
        STRICMP (_parser.getToken(2), "fault") == 0 &&
        STRICMP (_parser.getToken(3), "off") == 0)
    {
        iasFault (false);
        cl.answer (ERR_OK);
        return;
    }

    // "ahrs smooth on" command interpretation
    else if (_parser.count() == 4 &&
        STRICMP (_parser.getToken(1), "ahrs") == 0 &&
        STRICMP (_parser.getToken(2), "smooth") == 0 &&
        STRICMP (_parser.getToken(3), "on") == 0)
    {
    	notify(FPR_SMOOTH_ON);
        cl.answer (ERR_OK);
        return;
    }

    // "ahrs smooth off" command interpretation
    else if (_parser.count() == 4 &&
        STRICMP (_parser.getToken(1), "ahrs") == 0 &&
        STRICMP (_parser.getToken(2), "smooth") == 0 &&
        STRICMP (_parser.getToken(3), "off") == 0)
    {
    	notify(FPR_SMOOTH_OFF);
        cl.answer (ERR_OK);
        return;
    }

	// "tuning lat lon spd" command interpretation
    else if (_parser.count() == 5 && 
        STRICMP (_parser.getToken(1), "tuning") == 0)
    {
		_ramStorage.tuningLat = _parser.getTokenAsInt(2);
		_ramStorage.tuningLon = _parser.getTokenAsInt(3);
		_ramStorage.tuningSpd = _parser.getTokenAsInt(4);
        cl.answer (ERR_OK);
        return;
    }

    else ;  // empty instruction for formalities 

    cl.answer(ERR_UCOMMAND);
}


/**  
*    Flight plan instruction interpretation. (Instruction is in the _fpd variable)
*/
void FlightPlanRealizer::useFPlan(void)
{
    if (_fpd.currentStatus == FPlanConf::OnHold)
    {
        onHold();
        return;
    }
    else if (_fpd.currentStatus != FPlanConf::Running)
    {
        Log.errorPrintf("FlightPlanRealizer_useFPlan_1");
        return;
    }

    //  Running mode
    Log.msgPrintf ("%sFPlan line [id: %d, cmd: %s]", SUBS_PREFIX, _fpd.currentItem, _fpd.command);

    if (!_parser.loadLine(_fpd.command))
    {
        Log.errorPrintf("FlightPlanRealizer_useFPlan_2");
        return;
    }
    
    //  passing comments started with  //
    if (_parser.count() > 0 && 
        STRNICMP (_parser.getToken(0), "//", 2) == 0)
    {
        //  Instruction is treated as made
        fpCmdCompleted ();
        return;
    }
    
    // "preland <net/runway/pilot> <long_from> <lat_from> <long_to> <lat_to> <long_circle> <lat_circle> <abs> <radius> <left/right>" command interpretation
    else if (_parser.count() >= 8 &&
        STRICMP (_parser.getToken(0), "preland") == 0)
    {
        // Circle Radius
        if (!TypeParser::toFloat(_parser.getToken(9), _state.nLand.circleRadius))
        {
            Log.msgPrintf("%s: Unrecoginzed circle radius, use default", SUBS_PREFIX);
            _state.nLand.circleRadius = _conf.crcConstraint.radius * (1.0f - _conf.crcConstraint.radiusErr);
        }

        //  Circle direction
            if (STRICMP (_parser.getToken(10), "left") == 0)
                _state.nLand.circleLeft = true;
            
            else if (STRICMP (_parser.getToken(10), "right") == 0)
                _state.nLand.circleLeft = false;

            else
            {
                Log.msgPrintf("%s: Unrecognized circle direction, use default", SUBS_PREFIX);
                _state.nLand.circleLeft = _conf.crcConstraint.left;
            }

        // Main preland procedure
        netPrelandCompute(_parser.getToken(2), _parser.getToken(3), _parser.getToken(4), _parser.getToken(5), _parser.getToken(6), _parser.getToken(7));
        return;
    }

    // "disable controllers" command interpretation
    else if (_parser.count() == 2 && STRICMP (_parser.getToken(0), "disable") == 0 && STRICMP (_parser.getToken(1), "controllers") >= 0)
    {
        //  Any error messages or performed instructions messages are located inside
        disableControllers(true);
        return;
    }

    // "set <name> <value>" command interpretation
    else if (_parser.count() >= 3 && STRICMP (_parser.getToken(0), "set") == 0)
    {
        setParameters (_fpd.command + _parser.getTokenIndex(1), true);
        return;
    }

    // "wait" command interpretation
    else if ((_parser.count() == 1  || _parser.count() == 2) && 
        STRICMP (_parser.getToken(0), "wait") == 0)
    {
        char* p = NULL;
        if (_parser.count() == 2)
            p = _parser.getToken(1);
        wait(p);
        return;
    }

    // "fly last link" command interpretation
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(0), "fly") == 0 &&
        STRICMP (_parser.getToken(1), "last") == 0 &&
        STRICMP (_parser.getToken(2), "link") == 0)
    {
        flyLastLink ();
        return;
    }

    // "fly base" command interpretation
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(0), "fly") == 0 &&
        STRICMP (_parser.getToken(1), "base") == 0)
    {
        flyBase ();
        return;
    }

	// "fly base land" command interpretation
	else if (_parser.count() == 3 &&
		STRICMP(_parser.getToken(0), "fly") == 0 &&
		STRICMP(_parser.getToken(1), "base") == 0 &&
		STRICMP(_parser.getToken(2), "land") == 0)
	{
		flyBaseLand();
		return;
	}

	// "parachuting" command interpretation
	else if (_parser.count() == 1 &&
	STRICMP(_parser.getToken(0), "parachuting") == 0)
	{
		waitCalcPara();
		return;
	}

    // "search" command interpretation
	else if (_parser.count() == 1 &&
        STRICMP(_parser.getToken(0), "search") == 0)
	{
		flySearch();
		return;
	}

    // "fly <longitude> <latitude> <rel|abs>" command interpretation
    else if (_parser.count() == 4 && 
        STRICMP (_parser.getToken(0), "fly") == 0)
    {
        //fly (_parser.getToken(1), _parser.getToken(2), _parser.getToken(3), NULL);
		flyCamguide(_parser.getToken(1), _parser.getToken(2), _parser.getToken(3), NULL);
        return;
    }

    // "fly <longitude> <latitude> <rel|abs> [altitude]" command interpretation
    else if (_parser.count() == 5 && 
        STRICMP (_parser.getToken(0), "fly") == 0)
    {
        fly (_parser.getToken(1), _parser.getToken(2), _parser.getToken(3), _parser.getToken(4));
        return;
    }

    // "takeoff" command interpretation
    else if (_parser.count() >= 1 && 
        STRICMP (_parser.getToken(0), "takeoff") == 0)
    {
        takeoff ();
        return;
    }
 
    //  "circle base" command interpretation (must be interpreted before "circle <time> ..." command)
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(0), "circle") == 0 &&
        STRICMP (_parser.getToken(1), "base") == 0)
    {
        circleBase ();
        return;
	}

	//  "circle base" command interpretation (must be interpreted before "circle <time> ..." command)
	else if (_parser.count() == 3 &&
		STRICMP(_parser.getToken(0), "circle") == 0 &&
		STRICMP(_parser.getToken(1), "base") == 0 &&
		STRICMP(_parser.getToken(2), "parachute") == 0)
	{
		circleBasePara();
		return;
	}

    // "circle around <longitude> <latitude> abs|rel [radius [left|right]] [altitude]" command interpretation
    else if (_parser.count() >= 5 && _parser.count() <= 8 &&
        STRICMP (_parser.getToken(0), "circle") == 0 &&
        STRICMP (_parser.getToken(1), "around") == 0)
    {
        char* rad = NULL;
        char* dir = NULL;
        char* alt = NULL;

        if (_parser.count() >= 6)
            rad = _parser.getToken (5);
        if (_parser.count() >= 7)
            dir = _parser.getToken (6);
        if (_parser.count() == 8)
            alt = _parser.getToken (7);

        circleAround (_parser.getToken(2), _parser.getToken(3), _parser.getToken(4), rad, dir, alt);
        return;
    }

    //  "circle <time> [radius [left|right]]" command interpretation
    else if (_parser.count() >= 2 && _parser.count() <= 4 && 
        STRICMP (_parser.getToken(0), "circle") == 0)
    {
        char* rad = NULL;
        char* dir = NULL;

        if (_parser.count() >= 3)
            rad = _parser.getToken (2);
        if (_parser.count() == 4)
            dir = _parser.getToken (3);

        circle (_parser.getToken (1), rad, dir);
        return;
    }

	//  "circle <time> [radius [left|right]]" command interpretation
    else if (_parser.count() == 3 &&
        STRICMP (_parser.getToken(0), "climb") == 0 &&
		STRICMP (_parser.getToken(1), "by") == 0 &&
		STRICMP (_parser.getToken(2), "circle") == 0)
    {
		circleToClimb (_psd.position.getLat(), _psd.position.getLon(), _conf.linkBrokenRadius, _conf.linkBrokenMaxAlt);
        return;
    }

	//  "circle base prepare" command interpretation
	else if (_parser.count() == 4 &&
		STRICMP (_parser.getToken(0), "prepare") == 0 &&
		STRICMP (_parser.getToken(1), "circle") == 0 &&
		STRICMP (_parser.getToken(2), "base") == 0 &&
		STRICMP (_parser.getToken(3), "land") == 0)
	{
		circleBasePreEstimateHeading();
		return;
	}

#if EMRG_LOSS_ENGINE == 1
	else if (_parser.count() == 3 &&
        STRICMP (_parser.getToken(0), "descent") == 0 &&
		STRICMP (_parser.getToken(1), "by") == 0 &&
		STRICMP (_parser.getToken(2), "circle") == 0)
    {
		circleToDescent (_psd.position.getLat(), _psd.position.getLon(), _conf.emrgRadius, _conf.emrgAltitude);
        return;
    }

	else if (_parser.count() == 2 &&
        STRICMP (_parser.getToken(0), "focus") == 0 &&
		STRICMP (_parser.getToken(1), "land") == 0)
    {
		focusLand ();
        return;
    }
#endif


    // "goto <line_id>" command interpretation
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(0), "goto") == 0)
    {
        fpGoto (_parser.getToken(1));
        return;
    }
    else ;  // empty instruction for formalities 

    // Not recognized flight plan instruction.
    Log.msgPrintf("%s  -> Unrecognized flight plan instruction", SUBS_PREFIX);
    // Treated like being done.
    fpCmdCompleted ();

}


/**
* Notice from PState about new values handling.
*/
void FlightPlanRealizer::usePState(void)
{
    //  Passing some notice due to performance (there are not needed so often)
    if (++_mainCounter < _conf.mainDivisor)
    {
        return;
    }
    _mainCounter = 0;

    if (!PState->getPStateData (_psd))
    {
        Log.errorPrintf("FlightPlanRealizer_usePState_1");
        return;
    }

    //  Sending telemetry with subsystem state.
    sendTelemetry();

    //  Set up geographical position after autopilot has been turned on. It will be re-set in "takeoff" instruction.
    if (!_state.originPosIsSet)
    {
        PStateHealth psh;
        if (PState->getPStateHealth (psh))
            if (psh.gpsError == PStateHealth::gpsErrorCode::ERR_GPS_OK)
            {
                _state.origin.position = _psd.position;
                _state.originPosIsSet = true;
                if (!_state.basePosIsSet)
                {
                    _state.base.position = _state.origin.position;
                    _state.base.prevTrackPosition = _state.base.position;
                    _state.base.prevTrackSysTime = _psd.time100;
                    _state.basePosIsSet = true;
                }

                Log.msgPrintf ("%sSet coarse origin/base position: lon:%.6f lat:%.6f",
                    SUBS_PREFIX, _state.origin.position.getLon(), _state.origin.position.getLat());
            }
    }

    //  Set Phi angle in "camera guide" mode.
    if (_state.cameraGuideMode)
    {
        if (++_cgCounter >= CG_DIVISOR)
        {
            if (_vSem.lock ())
            {
                //  Set new reference value.
                cgSetPhi ();
                _vSem.unlock ();
                notify (FPR_CHANGED);
            }

            _cgCounter = 0;
        }
    }

    //  Check launch trigger.
    if (_state.launchTrigger.checkTrigger (_psd))
    {
        //  turn off trigger to prevent from being call again.
        _state.launchTrigger.reset();
        notify (FPR_LAUNCHED);
        Log.msgPrintf ("%sLaunch trigger fired.", SUBS_PREFIX);
    }

    int funId = 0;

    // Turn off emergency landing mechanism. (emergency landing after altitude dropped under reference emergency landing altitude).
    if (_conf.emrgLandEnable && (_state.fprd.fRef.altitude <= _conf.emrgLandAgl))
    {
        _conf.emrgLandEnable = false;
        Log.msgPrintf ("%sRef alt = %.1fm, emergency landing monitor off.", SUBS_PREFIX, _state.fprd.fRef.altitude);
    }

    //  Check the turn on condition of the emergency landing. Only when reference altitude is greater than altitude of turning on emergeency landing.
    if (_conf.emrgLandEnable && _state.emrgLandingCondition.checkCondition (_psd, funId))
    {
        Log.msgPrintf ("%sEmergency landing.", SUBS_PREFIX);
        // The same procedure as for the landing launched by the operator
        forceLand ();

        // Exit from procedure to not start handling another condition that the case could be fulfilled. 
        return;
    }

    // Checking the condition of decrease ground speed. 
    if (_state.lowGSpeedCondition.checkCondition (_psd, funId))
    {
        // Call function from the array of pointers.
        if (funId < MAX_PHASE_FUN && funId > 0 && _funTab[funId] != NULL)
            (this->*_funTab[funId])();
        else
            Log.errorPrintf("FlightPlanRealizer_usePState_2");
    }


    //  Check the stall condition.
    if (_state.stallCondition.checkCondition (_psd, funId))
    {
        // Turn off "manual turn" and "camera guide" modes.
        turnOff ();
        camGuideOff ();
        // Go to the stall recovery procedure.
        stallOn ();
    }

    //  Check exit condition from the "manual turn" mode.
    if (_state.exitConditionMT.checkCondition (_psd, funId))
    {
        //  Turn off "manual turn" mode.
        turnOff ();
        //  Redo from current flight plan line (send internal notification)
        redoCurrentFPLine ();
    }

    //  Check the exit condition from the flight phase. (and stall recovery)
    funId = -1;
    if (!(_state.manualTurnMode && _state.doNotFinishPhaseInMT) &&
        !(_state.cameraGuideMode && _state.doNotFinishPhaseInCG) ||
        (_state.stall))
    {
        //  Verify fulfillment of the conditions.
        bool b = _state.exitCondition.checkCondition (_psd, funId);
        if (b)
        {
            //  Delete set flight phase exit conditions and asynchronous triggers.
            _state.resetConditions();
            //  Turn of flight with low ground speed. (Phi_Psi)
            resetLowGSpeedCheck();
            if (funId == -1)
                // Last phase - notify about end of instruction
                fpCmdCompleted ();
            else
                //  Call function from the array of pointers.
                if (funId < MAX_PHASE_FUN && funId > 0 && _funTab[funId] != NULL)
                    (this->*_funTab[funId])();
                else
                    Log.errorPrintf("FlightPlanRealizer_usePState_3");
        }
    }
}


/**  
*  Loading subsystems configuration data. In case of an error default values are being set (error could destroy  current values).
*/
bool FlightPlanRealizer::confLoad(void)
{
    _isConfigLoaded = _confMem->loadFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!_isConfigLoaded)
    {
        // Restoring default configuration data in case of error.
        _conf.setDefault ();
        Log.errorPrintf("FlightPlanRealizer_confLoad_1");
    }

    return _isConfigLoaded;
}


/**  
* Saving subsystems configuration data.
*/
bool FlightPlanRealizer::confSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _confMem->saveFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!bok)
        Log.errorPrintf("FlightPlanRealizer_confSave_1");

    return bok;
}


/**
* Supporting things after completion of the flight plan instruction
*/
void FlightPlanRealizer::fpCmdCompleted (void)
{
    // Reset flag that lock interrrupt of the instruction (it is checked before new instruction, so it couldn't be at the instructions beggining '_state.resetmode()'.
    _state.setDoNotBreakFlag (false);
    _state.doNotBreakByRetMode = false;

    // Turn on AHRS correction (it's important after flight phase with turned off correction had been stopped).
    notify (FPR_SMOOTH_OFF);

    // Send notify that instruction had been completed.
    notify (FPEL_COMPLETED);
}


/** 
* Function to calculate Phi angle according to camera settings realtive to the airframe.
* cameraPan - rotation angle to the airframe vertical axis [radians, count clockwise]
*/
float FlightPlanRealizer::cgComputePhi(float cameraPan, float cameraTilt) const
{
    float tPhi = GpsPosition::normalize180 (cameraPan, true);
    float panOffsetRad = _conf.cgPanOffset * DEG_2_RAD;

    //  Angle pan offset allowing to fly straight with camera turned to the side.
    if (tPhi >= 0.0f)
        tPhi -= (panOffsetRad);
    else
        tPhi += (panOffsetRad);

    //  Mapping turn to phi angle factor value.
    tPhi *= _conf.cgPhiCoeff;  

    // Limiting the entrance to the allowed values
	if (tPhi > _conf.flyConstraint.Phi.maxVal)
        tPhi = _conf.flyConstraint.Phi.maxVal;
    else if (tPhi < _conf.flyConstraint.Phi.minVal)
        tPhi = _conf.flyConstraint.Phi.minVal;

    Numbers::assure (tPhi, 0.0f);

    return tPhi;
}

/** 
* Setting calculated reference value Phi angle used in the Camera Guide mode.
*/
void FlightPlanRealizer::cgSetPhi (void)
{
    _state.fprd.fRef.fRefLowLevel.phi = cgComputePhi (_tlmCam.pan * 0.01f * DEG_2_RAD, _tlmCam.tilt * 0.01f * DEG_2_RAD);
}


/**
* Redo current flight plan instruction (send notify)
*/
void FlightPlanRealizer::redoCurrentFPLine (void)
{
    notify (FPR_INTERNAL_REDO);
}


/** Wykonanie akcji po zmianie wartości parametru
*  flags - flagi bitowe zwrócone
*
*/
void FlightPlanRealizer::setParamUserAction (unsigned int flags)
{
    // Do not need to secure with semaphore because instruction fits in one word.
    if (((flags & UAF_REDO_CIRCLE) != 0 && _state.useUafRedoCircle))
    {
        //  Redo from current flight plan line (send internal notification)
        redoCurrentFPLine ();
    }

    if ((flags & UAF_SET_DFLT_ALT) != 0 && _state.useUafSetDfltAlt)
        _state.fprd.fRef.altitude = _conf.dfAltitude;

    if ((flags & UAF_SET_DFLT_AIRSPD) != 0 && _state.useUafSetDfltAirSpd)
        _state.fprd.fRef.airspeed = _conf.dfAirspeed;

    if ((flags & UAF_SET_CRC_AIRSPD) != 0 && _state.useUafSetCrcAirSpd)
        _state.fprd.fRef.airspeed = _conf.crcConstraint.airspeed;
}


/**  
* Send telemetry to log and/or communication channel.
*/
void FlightPlanRealizer::sendTelemetry (void)
{
    char buf[LINESIZE];
    bool linePrepared = false;

    // Send data to the log.
    if (_conf.logDivisor != 0 && ++_logCounter % _conf.logDivisor == 0)
    {
        if (prepareTlmLine (buf, sizeof(buf)))
        {
            Log.tlmPrint (TLM_REAL, buf, true, false);
            linePrepared = true;
        }
        else
            Log.errorPrintf("FlightPlanRealizer_sendTelemetry_1");

        _logCounter = 0;
    }

    //  Send data to communication channel.
    if (_conf.commDivisor != 0 && (++_commCounter % _conf.commDivisor == 0))
    {
        if (linePrepared || prepareTlmLine (buf, sizeof(buf)))
            Log.tlmPrint (TLM_REAL, buf, false, true);
        else
            Log.errorPrintf("FlightPlanRealizer_sendTelemetry_2");

        _commCounter = 0;
    }

}


/**
*    Function prepares text line with subsystems status.
*      buf - exit bufor
*      bufSize - bufor size
*/
bool FlightPlanRealizer::prepareTlmLine (char* buf, int bufSize) const
{
    FPRealTlm rt;

    // Do not need to secure with semaphore
    //rt.fillFrom (PState->getTime100(), _state.doNotBreak, _state.doNotManualTurn, _state.manualTurnMode, 
    //    (_state.nLandPhaseFlag == _state.PRE_LAND), (_state.nLandPhaseFlag == _state.APPROACH), (_state.nLandPhaseFlag == _state.DESCENT),
    //    (_state.nLandPhaseFlag == _state.LAND), (_state.nLandPhaseFlag == _state.ABORT_LAND),
    //    _state.doNotCameraGuide, _state.cameraGuideMode, _state.observationMode, (_fpd.itemType==FPlanData::ITEM_TYPE_RETURN),
    //    0, _fpd.currentStatus, _state.fprd.fRef.airspeed, _state.fprd.fRef.altitude, _fpd.currentItem);
	rt.fillFrom(PState->getTime100(), _state.doNotBreak, _state.doNotManualTurn, _state.manualTurnMode,
		(_state.landParachutePhaseFlag == _state.LND_PRELAND), (_state.landParachutePhaseFlag == _state.LND_ESTIMATE_HEADING), (_state.landParachutePhaseFlag == _state.LND_PARACHUTING),
		(_state.landParachutePhaseFlag == _state.LND_PARACHUTED), (_state.tkfPhaseFlag == _state.TKF_P1), (_state.tkfPhaseFlag == _state.TKF_P2), (_state.tkfPhaseFlag == _state.TKF_P3),
		_state.doNotCameraGuide, _state.cameraGuideMode, _state.observationMode, (_fpd.itemType == FPlanData::ITEM_TYPE_RETURN),
		0, _fpd.currentStatus, _state.fprd.fRef.airspeed, _state.fprd.fRef.altitude, _fpd.currentItem);

    if (!Base64::encode (&rt, sizeof(rt), buf, bufSize))
    {
        Log.errorPrintf("FlightPlanRealizer_prepareTlmLine_1");
        return false;
    }

    return true;
}


//*****************************************************************************
//  External commands / flight plan instructions.
//*****************************************************************************


/**
* Function sends to the specified device value of the subsystems parameter value (text)
* pName - name of the desire parameter (or * when all parameters are to be send)
* cl - object related with received line.
*/
bool FlightPlanRealizer::getParameter (const char* pName, ClassifiedLine& cl) const
{
    //  There are only readings so semaphore lock is no needed.
    return _pars->getParam(pName, cl);
}

/**  
* Function sets the value of many parameters. In case of an error values remains unchanged.
* nameValueItems - pair <name> <value> (as text)
* fromFPlan - instruction has been launched from flight plan - sends notify switching to next flight plan instruction.
*/
bool FlightPlanRealizer::setParameters (const char* nameValueItems, bool fromFPlan)
{
    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - skipping");
        if (fromFPlan)
            fpCmdCompleted ();
        return false;
    }

    unsigned int flags=0;
    bool err = (_pars->setParams (nameValueItems, &flags) == ParameterNames::ERR_OK);

    bool err2 = _vSem.unlock ();

	if (err && inPreLandPhase())
	{
		if (_parser.loadLine(nameValueItems))
		{
			if (_parser.count() >= 4 &&
				STRICMP(_parser.getToken(0), "base.position.lat") == 0 && 
				STRICMP(_parser.getToken(2), "base.position.lon") == 0)
			{
				Log.msgPrintf("%s  -> lat: %s - lon: %s", SUBS_PREFIX, _parser.getToken(1), _parser.getToken(3));
				notify(FPR_BASE_CHANGED);
			}
		}
		else
		{
			Log.msgPrintf("%s  -> parser real items failed", SUBS_PREFIX);
		}


	}
  
    if (fromFPlan)
    {
        //  Sends notify about FPReal settings changes to other subsystems (even if there is an error)
        notify (FPR_CHANGED);
        //  Notify about flight plan instruction had been completed.
        fpCmdCompleted ();
    }
    else
    {
        //  Execute command that had been set from command line by the user not from flight plan. (If no error occurde during setting)
        if (err)
            setParamUserAction (flags);

        //  Notify to other subsystems obout FPReal setting changes. It has to be a the end because refreshRefAltSpeed() sets variables for other subsystems.
        notify (FPR_CHANGED);
    }

    return err && err2;
}


/** 
* Set field "enable" to false in all controllers and resets the output variables. 
* Returns false in case of sempahors errors, in other cases true.
*/
bool FlightPlanRealizer::disableControllers (bool fromFPlan)
{
    bool ret = true;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - skipping");
        if (fromFPlan)
            fpCmdCompleted ();
        return false;
    }

    _state.fprd.ctrlProps.disableControllers();
    _state.fprd.outCtrl.fCtrl.reset();

    if (!_vSem.unlock ())
        ret = false;

    Log.msgPrintf ("%s  -> Controllers disabled, FlightControl outputs cleared.", SUBS_PREFIX);

    //  Send notify to other subsystems about FPReal settings had been changed (even there are an error).
    notify (FPR_CHANGED);
    //  Notice of completion of the flight plan instruction
    if (fromFPlan)
        fpCmdCompleted ();
    
    return ret;   
}


/**  
* Function that reads status of the camera data from output telemtry.
*/
void FlightPlanRealizer::tlmCamShort (const char* tlmData)
{
    int nBytes = 0;

    _tlmCamGood = false;

    if (!Base64::decode (tlmData, reinterpret_cast<unsigned char*>(&_tlmCam), sizeof(_tlmCam), nBytes))
    {
        Log.msgPrintf("%sBad camera data format: %s", SUBS_PREFIX, tlmData);
        return;
    }

    if (nBytes != sizeof(_tlmCam))
    {
        Log.msgPrintf("%sBad camera data length: %s", SUBS_PREFIX, tlmData);
        return;
    }

    _tlmCamGood = true;
}


/**
* Command "turn <value>" line instruction.
*/
bool FlightPlanRealizer::turn (const char* pValue)
{
    float tPhi = 0.0f;
    // Parameter validation
    if (!TypeParser::toFloat (pValue, tPhi))
        return false;

    //  Check whether manual turn mode could be enter.
    if (_state.doNotManualTurn)
        return true;


    // turn off "camera guide" mode if it was turned on.
    camGuideOff ();

    if (!_state.manualTurnMode)
    {
        // Checking whether to immediately end the current flight instruction after "manual turn" mode had been switched on. 
        if (_state.skipCommandInMT)
        {
            fpCmdCompleted();
            return true;
        }
        _state.manualTurnMode = true;
        _state.lastPhiTrackState  = _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable;
        _state.lastPhiCTrackState = _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable; // RF::
        Log.eventPrint (EVT_FPR_MANUAL, "1");
        Log.msgPrintf ("%sManual Turn Mode: on", SUBS_PREFIX);
    }

    // Setting timeout after which "manual turn" ends automatically. (turnOff function)
    _state.exitConditionMT.setInterval (_conf.manTimeout);

    if (!_vSem.lock ())
        return false;

    // turn off roll controller - Phi_Track
    _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable = false;
    _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable = false; 

    //  Mapping factor turn value to angle phi.
    tPhi *= _conf.manPhiCoeff;

    // Limiting the entrance to the allowed values
    if (tPhi > _conf.flyConstraint.Phi.maxVal)
        tPhi = _conf.flyConstraint.Phi.maxVal;
    else if (tPhi < _conf.flyConstraint.Phi.minVal)
        tPhi = _conf.flyConstraint.Phi.minVal;

    // sets new reference value
    _state.fprd.fRef.fRefLowLevel.phi = tPhi;

    if (!_vSem.unlock ())
        return false;

    notify (FPR_CHANGED);

    return true;
}

/** 
* Auxiliary function realizing "manual turn" mode turn off.
*/
void FlightPlanRealizer::turnOff (void)
{
    if (_state.manualTurnMode)
    {
        if (!_vSem.lock ())
            return;

        _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable = _state.lastPhiTrackState;
        _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable = _state.lastPhiCTrackState; 
        _state.manualTurnMode = false;

        // Restes "manual turn" mode exit condition (timeout), because turnOff function could be executed from command line and condition would continue to be checked.
        _state.exitConditionMT.reset ();

        if (!_vSem.unlock ())
            return;

        Log.eventPrint (EVT_FPR_MANUAL, "0");
        Log.msgPrintf ("%sManual Turn Mode: off", SUBS_PREFIX);

        notify (FPR_CHANGED);
    }
}


/**  
* Function used to turn on "camera guide" mode.
*/
bool FlightPlanRealizer::camGuideOn (void)
{
    // Checking if currently is in this mode.
    if (_state.cameraGuideMode)
        return true;

    // Checking if this mode could be entered.
    if (_state.doNotCameraGuide)
        return true;

    // Turn off "manual turn" mode if it was turned on.
    turnOff ();

    // Checking whether to immediately end the current flight instruction after "camera guide" mode had been switched on. 
    // New instruction turn off "camera guide" mode at the beggining so there it's no use to turn it on yet. 
    if (_state.skipCommandInCG)
    {
        fpCmdCompleted();
        return true;
    }

    _state.cameraGuideMode = true;
    _state.lastPhiTrackState = _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable;
    _state.lastPhiCTrackState = _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable; 
    Log.eventPrint (EVT_FPR_CG, "1");
    Log.msgPrintf ("%sCamera Guide Mode: on", SUBS_PREFIX);

    if (!_vSem.lock ())
        return false;

    // turn off roll controller - Phi_Track
    _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable = false;
    _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable = false; 

    // sets new reference value
    cgSetPhi ();

    if (!_vSem.unlock ())
        return false;

    notify (FPR_CHANGED);

    return true;
}


/**  
* Function used to turn off "camera guide" mode.
*/
void FlightPlanRealizer::camGuideOff (void)
{
    if (_state.cameraGuideMode)
    {
        if (!_vSem.lock ())
            return;

        _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable = _state.lastPhiTrackState;
        _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable = _state.lastPhiCTrackState;
        _state.cameraGuideMode = false;

        if (!_vSem.unlock ())
            return;

        Log.eventPrint (EVT_FPR_CG, "0");
        Log.msgPrintf ("%sCamera Guide Mode: off", SUBS_PREFIX);

        notify (FPR_CHANGED);
    }
}

/**     
* Instruction do nothing - stops flight plan execution.
* pTime - time in miliseconds as text (floating) or NULL (unlimited time)
* Instruction ccan not be executed from command line.
*/
void FlightPlanRealizer::wait (const char* pTime)
{

    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();

    //  Delete all set conditions.
    _state.resetConditions ();

    if (pTime == NULL)
    {
        Log.msgPrintf ("%s  -> Waiting", SUBS_PREFIX);
        return;   
        //  Unlimited time - lack of notice and exit conditions setting.
    }
    else
    {
        float ft = 0.0f;
        bool bok = TypeParser::toFloat (pTime, ft);
        if (bok)
        {
            // Set time condition.
            _state.exitCondition.setInterval (ft);
        }
        else
        {
            // Error - notify about flight plan completion (no to suspend program).
            fpCmdCompleted ();
        }

        Log.msgPrintf ("%s  -> Waiting %.3f sec [%s]", SUBS_PREFIX, ft, bok ? "ok" : "error - no waiting");
    }
}


/**  
* Flight plan instruction "Fly" (flight to a specified point). Point can be given as absolute coordinates 
* or as distance from beggining point in meteres. Distance in N & E or S & W. 
* After fulfill of exit conditions second phase function being executed - flyPhase2
* p1, p2 - coordinates as a text
* pMode - type of coordinates (rel, abs) relative, absolute
* pAlt -points altitude above launch point as text. It can be NULL.
*/
void FlightPlanRealizer::fly (const char* p1, const char* p2, const char* pMode, const char* pAlt)
{
    // reset flags associated with locking breaking instruction and manual turn mode.
    _state.resetMode();

    // Settings flags causing simultaneous ref.altitude & ref.airspeed changings after desired values modification (alg.dfAltitude i alg.dfAirspeed)
    _state.useUafSetDfltAlt = true;
    _state.useUafSetDfltAirSpd = true;

	if (!_vSem.lock ("%s  -> System error (lock) [error - fly skipping]", SUBS_PREFIX))
    {
        fpCmdCompleted ();
        return;
    }
	setRefWaypoint (p1, p2, pMode);
	setRefAltitude (pAlt);
	_state.fprd.fRef.airspeed = _conf.dfAirspeed;

    //  not circling around point (radius 0)
    _state.fprd.fRef.circleRadius = 0.0f;

    setControllers(FLY_MODE);
    _state.fprd.ctrlProps.L1CProp.setControllerParams(L1_CIRCLE_ID, _conf.flyConstraint.Phi.minVal, _conf.flyConstraint.Phi.maxVal);

    _vSem.unlock ();

    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();
    // Condition of point attainment
    _state.exitCondition.setFlyFinish (_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo, _conf.flyConstraint.finishError);
    // Extra time condition to prevent to quickly point attainment when there is a loop in flight plan to the same instruction. In general flight this case is inessential.
    _state.exitCondition.setInterval (0.5f);
    //  Set function that is going to be executed after condition has been fulfil.
    _state.exitCondition.setChainFunction (FLY_PHASE_2);

    // Set stall recovery procedures executing condition.
    setStallRecovery ();

    // Switch on condition that cause using Phi_Psi controller.
    setLowGSpeedCheck ();

    //  Turn on condition that execute emergency landing procedure after altitude has dropped.
    setEmergencyLandingCheck ();

    Log.msgPrintf ("%s  -> Flying to lon:%.6f lat:%.6f [%s]", SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}


/*
*  Second phase of the "Fly" instruction - saving destination point.
*  This point will be used as beggining point in next "Fly" instruction. 
*/
void FlightPlanRealizer::flyPhase2 (void)
{
    Log.msgPrintf ("%s  -> 'To' met at lon:%.6f lat:%.6f err:%.1fm [%s]",
        SUBS_PREFIX, _psd.position.getLon(), _psd.position.getLat(),
        GpsPosition::distance (_psd.position, _state.fprd.fRef.wptTo), "ok");

    //  This phase don't set any conditions, so notify (...) will not be executed automatically.
    fpCmdCompleted ();
}

/**  
* Flight plan instruction "Fly" (flight to a specified point). Point can be given as absolute coordinates 
* or as distance from beggining point in meteres. Distance in N & E or S & W. 
* After fulfill of exit conditions second phase function being executed - flyPhase2
* p1, p2 - coordinates as a text
* pMode - type of coordinates (rel, abs) relative, absolute
* pAlt -points altitude above launch point as text. It can be NULL.
*/
void FlightPlanRealizer::flyCamguide (const char* p1, const char* p2, const char* pMode, const char* pAlt)
{
	// reset flags associated with locking breaking instruction and manual turn mode.
	_state.resetMode();

	// Settings flags causing simultaneous ref.altitude & ref.airspeed changings after desired values modification (alg.dfAltitude i alg.dfAirspeed)
	_state.useUafSetDfltAlt = true;
	_state.useUafSetDfltAirSpd = true;

	if (!_vSem.lock ("%s  -> System error (lock) [error - fly skipping]", SUBS_PREFIX))
	{
		fpCmdCompleted ();
		return;
	}
	setRefWaypoint (p1, p2, pMode);
	setRefAltitude (pAlt);
	_state.fprd.fRef.airspeed = _conf.dfAirspeed;
	_state.camguide.positionCamguide = _state.fprd.fRef.wptTo;

	//  not circling around point (radius 0)
	//_state.fprd.fRef.circleRadius = 0.0f;
	_state.fprd.fRef.circleRadius = 0.0f;//_conf.crcConstraint.radius;
	_state.fprd.fRef.circleLeft = _conf.crcConstraint.left;


	setControllers(FLY_MODE);

	_vSem.unlock ();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();

	float radius = 0.0f;
	radius = _conf.crcConstraint.radius;
    _state.fprd.ctrlProps.L1CProp.setControllerParams(L1_CIRCLE_ID, _conf.crcConstraint.Phi.minVal, _conf.crcConstraint.Phi.maxVal);
	// Condition of point attainment
	_state.exitCondition.setFlyFinish(_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo, 2*radius);
	// Extra time condition to prevent to quickly point attainment when there is a loop in flight plan to the same instruction. In general flight this case is inessential.
	_state.exitCondition.setInterval(0.5f);
	//  Set function that is going to be executed after condition has been fulfil.
	_state.exitCondition.setChainFunction(CRC_CAMGUIDE);

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Flying to lon:%.6f lat:%.6f [%s]", SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/**  
* Flight plan instruction "Fly search" (flight to a specified point). Point can be given as absolute coordinates 
* or as distance from beggining point in meteres. Distance in N & E or S & W. 
* After fulfill of exit conditions second phase function being executed - flyPhase2
* p1, p2 - coordinates as a text
* pMode - type of coordinates (rel, abs) relative, absolute
* pAlt -points altitude above launch point as text. It can be NULL.
*/
void FlightPlanRealizer::flySearch ()
{
    FPlan->getCamGuidePoint(_state.camguide.positionCamguide);
	// reset flags associated with locking breaking instruction and manual turn mode.
	_state.resetMode();
    float radius = 0.0f;
	radius = _conf.crcConstraint.radius;
    
	// Settings flags causing simultaneous ref.altitude & ref.airspeed changings after desired values modification (alg.dfAltitude i alg.dfAirspeed)
	_state.useUafSetDfltAlt = true;
	_state.useUafSetDfltAirSpd = true;

	if (!_vSem.lock ("%s  -> System error (lock) [error - fly skipping]", SUBS_PREFIX))
	{
		fpCmdCompleted ();
		return;
	}
	// setRefWaypoint (p1, p2, pMode);
	// setRefAltitude (pAlt);
    _state.fprd.fRef.wptFrom = _psd.position;
    _state.fprd.fRef.wptTo = _state.camguide.positionCamguide;
	_state.fprd.fRef.airspeed = _conf.dfAirspeed;
    _state.fprd.fRef.altitude = _conf.dfAltitude;
	// _state.camguide.positionCamguide = _state.fprd.fRef.wptTo;
    bool inROI = GpsPosition::distance(_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo) < 2.5 * _psd.altitude;

    _state.fprd.fRef.circleRadius = _conf.crcConstraint.radius;
	_state.fprd.fRef.circleLeft = _conf.crcConstraint.left;

	setControllers(FLY_MODE);
    if (inROI)
    {
        _state.fprd.fRef.circleRadius = 2 * radius;
        _state.fprd.ctrlProps.L1CProp.setControllerParams(L1_CIRCLE_ID, _conf.crcConstraint.Phi.minVal, _conf.crcConstraint.Phi.maxVal);
    }
    else
        _state.fprd.ctrlProps.L1CProp.setControllerParams(L1_HOMING_ID, _conf.flyConstraint.Phi.minVal, _conf.flyConstraint.Phi.maxVal);
    
	_vSem.unlock ();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();
	// Condition of point attainment
    if (inROI)
        _state.exitCondition.setPointDistance(_state.fprd.fRef.wptTo, 2.9f * _psd.altitude, true);
    else
        _state.exitCondition.setFlyFinish(_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo, 2.5f * _psd.altitude);

	// Extra time condition to prevent to quickly point attainment when there is a loop in flight plan to the same instruction. In general flight this case is inessential.
	// _state.exitCondition.setInterval(0.5f);
	//  Set function that is going to be executed after condition has been fulfil.
	_state.exitCondition.setChainFunction(SEARCH);

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Flying to lon:%.6f lat:%.6f [%s]", SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/**  
* Flight plan instruction "Fly" (flight to a specified point). Point can be given as absolute coordinates 
* or as distance from beggining point in meteres. Distance in N & E or S & W. 
* After fulfill of exit conditions second phase function being executed - flyPhase2
* p1, p2 - coordinates as a text
* pMode - type of coordinates (rel, abs) relative, absolute
* pAlt -points altitude above launch point as text. It can be NULL.
*/
void FlightPlanRealizer::searchObj ()
{
	// reset flags associated with locking breaking instruction and manual turn mode.
	_state.resetMode();
    _state.fprd.fRef.wptFrom = _psd.position;

	if (!_vSem.lock ("%s  -> System error (lock) [error - fly skipping]", SUBS_PREFIX) || _psd.altitude < 70.0f)
	{
		fpCmdCompleted ();
		return;
	}
	//setRefAltitude (pAlt);
    _state.fprd.fRef.altitude = 0.0f;
    _state.fprd.fRef.wptTo = _state.camguide.positionCamguide;
	setControllers(FLY_MODE);
    _state.fprd.ctrlProps.L1CProp.setControllerParams(L1_HOMING_ID, _conf.flyConstraint.Phi.minVal, _conf.flyConstraint.Phi.maxVal);
    
	_vSem.unlock ();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();
	// Condition of point attainment
    //_state.exitCondition.setPointDistance(_state.fprd.fRef.wptTo, 10.0f, false);
    _state.exitCondition.setAgl(50.0f, false);
	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Flying to lon:%.6f lat:%.6f [%s]", SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

#if EMRG_LOSS_ENGINE == 1
void FlightPlanRealizer::focusLand (void)
{
	//  Lock the possibility of instruction interruption from the FPlan subsystem.
    _state.setDoNotBreakFlag (true); 

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock_1) [%s]", SUBS_PREFIX, "error - fly skipping");
        fpCmdCompleted ();
        return;
    }

    //  Set default airspeed.   
    _state.fprd.outCtrl.fCtrl.reset();

    //  Controllers settings - butterfly is not allowed.
	_flightConstraint = &_conf.flyConstraint;
	_state.fprd.fRef.fRefLowLevel.phi = 0.0f;
	setLonControllers (LON_HOLD_PHI_ID);
	_state.fprd.fRef.fRefLowLevel.theta = _conf.emrgThetaDeg * DEG_2_RAD;
	setLatControllers (LAT_HOLD_THETA_ID);

    _vSem.unlock ();

    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();

    Log.msgPrintf ("%s  -> Focus Landing [%s]", SUBS_PREFIX, "ok");

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}
#endif

/**
* "Fly Last Link" flight plan instruction. (Fly to last connectivity point.
* Instruction do not has next phases.
*/
void FlightPlanRealizer::flyLastLink (void)
{
    // omit instruction when it was called during the communication was working.
    if (!SysMon->isLinkBroken())
    {
        Log.msgPrintf ("%s  -> Link not broken - skipping", SUBS_PREFIX);
        fpCmdCompleted ();
        return;
    }

    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();

    //  Lock "manual" and "camera guide" modes.
    _state.doNotManualTurn = true;
    _state.doNotCameraGuide = true;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock_1) [%s]", SUBS_PREFIX, "error - fly skipping");
        fpCmdCompleted ();
        return;
    }

    // Set starting point reference value.
    _state.fprd.fRef.wptFrom = _psd.position;
    Log.msgPrintf ("%s  -> Flying from current position - lon:%.6f lat:%.6f",
        SUBS_PREFIX, _state.fprd.fRef.wptFrom.getLon(), _state.fprd.fRef.wptFrom.getLat());

    // Destination point coordinate and altitude.
    GpsPosition pos;
    float alt = 0.0f;
    if (!SysMon->lastLinkPos (pos, alt))
    {
        _vSem.unlock();
        Log.msgPrintf ("%s  -> System error (lock_2) [%s]", SUBS_PREFIX, "error - fly skipping");
        fpCmdCompleted ();
        return;
    }

    // Set destination point reference value.
    _state.fprd.fRef.wptTo = pos;
    // Do not circle around the point ( radius = 0).
    _state.fprd.fRef.circleRadius = 0.0f;

    //  Set default airspeed.
    _state.fprd.fRef.airspeed = _conf.dfAirspeed;
   
    // Set altitude of point where was last communication.
    _state.fprd.fRef.altitude = alt;

    setControllers(FLY_MODE);

    _vSem.unlock ();

    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();

    _state.exitCondition.setFlyFinish (_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo, _conf.flyConstraint.finishError);

    // Set stall recovery procedures executing condition.
    setStallRecovery ();

    // Switch on condition that cause using Phi_Psi controller.
    setLowGSpeedCheck ();

    //  Turn on condition that execute emergency landing procedure after altitude has dropped.
    setEmergencyLandingCheck ();

    Log.msgPrintf ("%s  -> Flying to lon:%.6f lat:%.6f [%s]",
        SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}


/* 
* "Fly Base" fligt plan instruction.
* Flight is a tangent to the circle around the base station (circle has parameters like those in "Circle" instruction)
* Baste station position changings during the flight in "Fly Base" instruction are ignored.
* Instruction don't has next phases. 
*/
void FlightPlanRealizer::flyBase (void)
{
    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();

    //  Lock "manual" and "camera guide" modes.
    _state.doNotManualTurn = true;
    _state.doNotCameraGuide = true;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock_3) [%s]", SUBS_PREFIX, "error - fly skipping");
        fpCmdCompleted ();
        return;
    }

    // Set starting point reference value.
    _state.fprd.fRef.wptFrom = _psd.position;
    Log.msgPrintf ("%s  -> Flying from current position - lon:%.6f lat:%.6f",
        SUBS_PREFIX, _state.fprd.fRef.wptFrom.getLon(), _state.fprd.fRef.wptFrom.getLat());

    // Set reference value of the destination point.
    _state.fprd.fRef.wptTo = _state.base.position;
    // Radius and direction of circle.
    _state.fprd.fRef.circleRadius = 0.0f;//_conf.crcConstraint.radius;
    _state.fprd.fRef.circleLeft = _conf.crcConstraint.left;

    // Set default airpseed.
    _state.fprd.fRef.airspeed = _conf.dfAirspeed;

    // Elevation offset is reset (to lift tje airplane) when airframe fly over the area of elevation below the launch site.
    // When the elevation of the ground was above the starting point, the return is at high altitude, because it is not known how quickly lowered terrain.
    float elvOff = PState->getElevationOffset();
    if (elvOff < 0.0f)
        elvOff = 0.0f;

    // Turn off flight according to AGL. 
    PState->useElevationOffsetPar (false);

    // Set reference altitude ( relative to the takeoff place)
	_conf.dfAltitude = _psd.altitude;
	_state.fprd.fRef.altitude = _psd.altitude;
#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
	{
		_conf.dfAltitude = _conf.linkBrokenMaxAlt;
		_state.fprd.fRef.altitude = _conf.linkBrokenMaxAlt;
	}
#endif
    // Ver 0.26: holding the altitude of UAV after lost link
    setControllers(FLY_MODE);

    _vSem.unlock ();

    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();

	float radius = 0.0f;
#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
		radius = _conf.emrgRadius;
	else
#endif
		radius = _conf.crcConstraint.radius;
	_state.exitCondition.setFlyFinish (_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo, radius * 2.0f);

    // Set stall recovery procedures executing condition.
    setStallRecovery ();

    // Switch on condition that cause using Phi_Psi controller.
    setLowGSpeedCheck ();

    //  Turn on condition that execute emergency landing procedure after altitude has dropped.
    setEmergencyLandingCheck ();

    Log.msgPrintf ("%s  -> Flying to lon:%.6f lat:%.6f [%s]",
        SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}

/*
* "Fly Base Land" fligt plan instruction.
* Flight is a tangent to the circle around the base station (circle has parameters like those in "Circle" instruction)
* Baste station position changings during the flight in "Fly Base" instruction are ignored.
* Instruction don't has next phases.
*/
void FlightPlanRealizer::flyBaseLand(void)
{
	// reset flags associated with locking breaking instruction and manual turn mode. 
	_state.resetMode();

	// Settings flags causing simultaneous ref.altitude & ref.airspeed changings after desired values modification (alg.dfAltitude i alg.dfAirspeed)
	_state.useUafSetDfltAlt = true;
	_state.useUafSetDfltAirSpd = true;

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock_3) [%s]", SUBS_PREFIX, "error - fly skipping");
		fpCmdCompleted();
		return;
	}
	//Set landing parachute flag
	_state.landParachutePhaseFlag = _state.LND_PRELAND;

	// Set starting point reference value.
	_state.fprd.fRef.wptFrom = _psd.position;
	Log.msgPrintf("%s  -> Flying from current position - lon:%.6f lat:%.6f",
		SUBS_PREFIX, _state.fprd.fRef.wptFrom.getLon(), _state.fprd.fRef.wptFrom.getLat());

	// Set reference value of the destination point.
	_state.fprd.fRef.wptTo = _state.base.position;
	// Radius and direction of circle.
	_state.fprd.fRef.circleRadius = 0.0f;//_conf.crcConstraint.radius;
	_state.fprd.fRef.circleLeft = _conf.crcConstraint.left;

	// Set default airpseed.
	_state.fprd.fRef.airspeed = _conf.dfAirspeed;

	// Elevation offset is reset (to lift tje airplane) when airframe fly over the area of elevation below the launch site.
	// When the elevation of the ground was above the starting point, the return is at high altitude, because it is not known how quickly lowered terrain.
	float elvOff = PState->getElevationOffset();
	if (elvOff < 0.0f)
		elvOff = 0.0f;

	// Turn off flight according to AGL. 
	PState->useElevationOffsetPar(false);

#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
	{
		_conf.dfAltitude = _conf.linkBrokenMaxAlt;
		_state.fprd.fRef.altitude = _conf.linkBrokenMaxAlt;
	}
#endif
	// Ver 0.26: holding the altitude of UAV after lost link
	setControllers(FLY_MODE);

	_vSem.unlock();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();

	float radius = 0.0f;
#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
		radius = _conf.emrgRadius;
	else
#endif
		radius = _conf.crcConstraint.radius;
	_state.exitCondition.setFlyFinish(_state.fprd.fRef.wptFrom, _state.fprd.fRef.wptTo, radius * 2.0f);

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Flying to lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/*
* "Waiting to calculate the parachuting position" fligt plan instruction.
* Instruction has next phases.
*/
void FlightPlanRealizer::waitCalcPara(void)
{
	// reset flags associated with locking breaking instruction and manual turn mode. 
	_state.resetMode();

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - circle skipping");
		fpCmdCompleted();
		return;
	}

	//Set landing parachute flag
	_state.landParachutePhaseFlag = _state.LND_PARACHUTING;

	_state.para.circleLeft = _conf.crcConstraint.left;
	_state.para.circleRadius = _conf.autoParaRadius;
	_state.para.altitude = _conf.crcConstraint.baseAltitude;
	_state.para.airspeed = _conf.autoParaAirspeed;
	_state.para.wind.from = _psd.wind.from;
	_state.para.wind.speed = _psd.wind.speed;

	float driftDistance = _psd.wind.speed * KPH_2_MS * (_conf.crcConstraint.baseAltitude / _conf.autoParaStableVelZ);

	// Parachuting position
	GpsPosition::movePosition(_state.base.position, _psd.wind.from, driftDistance, _state.para.positionParachuting);

	// Centre of the circle before parachuting.
	float moveTrack = GpsPosition::addTrack(_psd.wind.from, _state.para.circleLeft ? -90.0f : 90.0f);

	GpsPosition::movePosition(_state.para.positionParachuting, moveTrack, _state.para.circleRadius, _state.para.positionCircleCenter);

	Log.msgPrintf("%s  -> Waiting for Calculate the parachuiting point - with drift distance: %f.2",
		SUBS_PREFIX, driftDistance);

	_vSem.unlock();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();

	// The condition will be satisfied at a distance less than the radius of 1.41, which gives a 45 degree deviation from the route directly to the point. This factor may be different.
	_state.exitCondition.setInterval(0.5f);

	//  Set function that will be executed after fulfilment of the condition.
	_state.exitCondition.setChainFunction(CRC_PARA_PHASE_1);
	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);

}

/**
*  Low level "circle parachuting phase 1" instruction. 
*/
void FlightPlanRealizer::circleParaPhase1(void)
{
	// reset flags associated with locking breaking instruction and manual turn mode. 
	_state.resetMode();

	//  Set flags that cause simultaneous change of ref.altitude and ref.airspeed after default values has been changed (alg.dfAltitude & alg.dfAirspeed)
	//  If altitude was given as a parameter (call from CircleBase) reference altitude could not be change automatically based on default value. It is controlled by the special parameter.
	_state.useUafSetDfltAlt = true;
	_state.useUafSetDfltAirSpd = true;  //  airspeed during the flight to the circle point.

	//  Turn on flag that cause immediate skip to next flight plan instruction after turning on "manual turn" or "camera guide" mode.
	_state.skipCommandInMT = false; // RF:: true;
	_state.skipCommandInCG = false; // RF:: true;

	//  Allow to re-execute Circle instruction and other instructions executing CirclePhase1 instruction after parameters that has UAF_REDO_CIRCLE flag changed.
	_state.useUafRedoCircle = true;

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - circle skipping");
		fpCmdCompleted();
		return;
	}

	_state.fprd.fRef.wptFrom = _state.fprd.fRef.wptTo;
	_state.fprd.fRef.wptTo = _state.para.positionCircleCenter;
	_state.fprd.fRef.circleRadius = _state.para.circleRadius;
	_state.fprd.fRef.circleLeft = _state.para.circleLeft;

	// Parachuting altitude
	_state.fprd.fRef.altitude = _state.para.altitude;

	// Parachuting airspeed
	_state.fprd.fRef.airspeed = _state.para.airspeed;

	//  Set controllers like as those in straight flight (fly).
	if (GpsPosition::distance(_psd.position, _state.para.positionCircleCenter) > _state.para.circleRadius * 1.41f)
	{
		setControllers(FLY_MODE);
	}

	_vSem.unlock();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();

	// The condition will be satisfied at a distance less than the radius of 1.41, which gives a 45 degree deviation from the route directly to the point. This factor may be different.
	_state.exitCondition.setPointDistance(_state.para.positionCircleCenter, _state.para.circleRadius * 2.0f, false);

	//  Set function that will be executed after fulfilment of the condition.
	_state.exitCondition.setChainFunction(CRC_PARA_PHASE_2);
	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Parachute landing: Approach: Circle Phase 1 (fly) around lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/**
* Second phase of the "Circle Parachuting" instruction.
*/
void FlightPlanRealizer::circleParaPhase2(void)
{
	// _state.doNotBreak = true;
	//  Turn off automatic change of ref.Airspeed after dfAirspeed changing (but turn on is after crcAirspeed)
	_state.useUafSetDfltAirSpd = false;  //  airspeed at flight to the circle point.
	_state.useUafSetCrcAirSpd = false;    //  circle airspeed.

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
	}

	_state.fprd.fRef.airspeed = _state.para.airspeed;      //  Circle airspeed
	setControllers(CRC_MODE);

	_vSem.unlock();

	// Delete all set conditions and set time condition given as parameter.
	_state.resetConditions();

	// The condition for a flight in the specified range of angles of the track. Will not be met when the plane goes back.
	_state.exitCondition.setTrackRange(_state.para.wind.from - _conf.autoParaDelTrack, _state.para.wind.from + _conf.autoParaDelTrack);
	_state.exitCondition.setInterval(_conf.autoParaApproachTime);
	//  Set function that will be executed after fulfilment of the condition.
	_state.exitCondition.setChainFunction(CRC_PARA_PHASE_3);

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Parachute landing: Approach: Circle Phase 2 around lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/**
* Second phase of the "Circle Parachuting" instruction.
*/
void FlightPlanRealizer::parachuting(void)
{
	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
	}
	_state.resetMode();
	//  Lock the possibility of instruction interruption from the FPlan subsystem.
	_state.setDoNotBreakFlag(true);

	//Set landing parachute flag
	_state.landParachutePhaseFlag = _state.LND_PARACHUTED;

	//  Turn off all controllers.
	_state.fprd.ctrlProps.disableControllers();

	//  Reset outputs.
	_state.fprd.outCtrl.fCtrl.reset();

	_vSem.unlock();

	Log.msgPrintf("%s  -> Parachute landing: Parachuiting at position - lon:%.6f lat:%.6f",
		SUBS_PREFIX, _state.para.positionParachuting.getLon(), _state.para.positionParachuting.getLat());

	notify(FPR_PARACHUTE);

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}



/**  
* "Takeoff" fligt plan instruction.
* Function is ignored when UAV is flying.
* After fulfilling the exit conditions second phase function has been executed (takeoffPhase2)
*/
void FlightPlanRealizer::takeoff (void)
{    
    //  Ignore "takeoff" instruction when the UAV is in the air.
    if ( !SystemNowOnGround )
    {
        Log.msgPrintf ("%s  -> Aircraft is airborne - takeoff skipping", SUBS_PREFIX);
        fpCmdCompleted ();
        return;
    }

    // Checking if system is ready to takeoff
    if (!SysMon->isReadyForRun (true))
    {
        Log.msgPrintf ("%s  -> System not ready for takeoff - waiting for operator action", SUBS_PREFIX);

        // Notify is not beeing send. Flight plan subsystem waits in this instruction waits forever or until another data reading from fplan subsytem.
        return;
    }

    //  Saves altitude and pressure w Physical State (taking into account altitude of takeoff)
    if (!PState->setAglZero (true))
    {
        Log.msgPrintf ("%s  -> Cannot set origin elevation - waiting for operator action", SUBS_PREFIX);
        // Notify is not beeing send. Flight plan subsystem waits in this instruction waits forever or until another data reading from fplan subsytem.
        return;
    }
        
    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (takeoff,lock) - waiting for operator action", SUBS_PREFIX);
        //  Waits for operator action.
        return;
    }

      //  Set values that defines takeoff and base station position Ustawienie wartości opisujących miejsce startu i położenie stacji bazowej
    _state.origin.position = _psd.position;
    _state.base.position = _state.origin.position;
#if USE_DGPS == 1
	_state.nLand.net.offSet = PState->getNetOffset();
#endif
    _vSem.unlock ();
    
	// Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();

    setStallRecovery ();        // Set stall recovery procedures executing condition. (is only relevant when there is no next instruction in the flight plan)
    notify (FPR_SMOOTH_OFF);    // Release request correction angles off AHRS.
    notify (FPR_TAKEOFF_CPL);    // Notify about end of takeoff procedure. (used in energy consumption durig flight)
    notify (FPR_CHANGED);        // Send notify to other subsystems about FPReal settings had been changed. Unusual situation because the instruction has just ends and will be taken next thar will change the settings.
    // Checking if auto take off is enabled
    if (!_conf.autoTkfCatapultEnable)
    {
        Log.msgPrintf("%s  -> Auto taking off from catapult is not permitted", SUBS_PREFIX);
		Log.msgPrintf("%s  -> Taking off by RC Pilot", SUBS_PREFIX);
		// Notify complete takeoff
		fpCmdCompleted();
        return;
    }
	else
	{
        _state.exitCondition.setInterval (1.0f);
        _state.exitCondition.setChainFunction (AUTO_TAKEOFF_CATAPULT_P1);
		notify (FPR_ATTENTION);        // Notify that something interesting is happening. Pay attention.
		Log.msgPrintf ("%s  -> Auto catapult takeoff starting ...", SUBS_PREFIX);
	}
}

/**
* Auto takeoff catapult phase 1. 
*/
void FlightPlanRealizer::autoTkfCatapultPhase1(void)
{
	_state.resetMode();

    //  Ignore "takeoff" instruction when the UAV is in the air.
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%s  -> Aircraft is airborne - auto takeoff skipping", SUBS_PREFIX);
        fpCmdCompleted ();
        return;
    }
    
    // Checking if system is ready to takeoff
    if (!SysMon->isReadyForRun (true))
    {
        Log.msgPrintf ("%s  -> System not ready for takeoff - waiting for operator action", SUBS_PREFIX);

        // Notify is not beeing send. Flight plan subsystem waits in this instruction waits forever or until another data reading from fplan subsytem.
        return;
    }
    
    //  Saves altitude and pressure w Physical State (taking into account altitude of takeoff)
    if (!PState->setAglZero (true))
    {
        Log.msgPrintf ("%s  -> Cannot set origin elevation - waiting for operator action", SUBS_PREFIX);
        // Notify is not beeing send. Flight plan subsystem waits in this instruction waits forever or until another data reading from fplan subsytem.
        return;
    }
	// Set state flag
	_state.tkfPhaseFlag = _state.TKF_P1;

    // Enable indicating flag to not signal ENGINE FAILURE during takeoff
    _state.bAllowEngineMsg = false;    
    
    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();
    
    //  Turn off all controllers.
    _state.fprd.ctrlProps.disableControllers();
    
    //  Reset outputs.
    _state.fprd.outCtrl.fCtrl.reset();
    
    //  Lock the possibility of instruction interruption from the FPlan subsystem.
    _state.setDoNotBreakFlag (true);    
    
    if (!_vSem.lock ())
    {
        //  Send message to GCS
        Log.msgPrintf ("%s  -> Auto catapult takeoff phase 1 error (semaphore lock)", SUBS_PREFIX);
        //  Waits for operator action.
        return;
    } 
    
    //  Set values that defines takeoff and base station position Ustawienie wartości opisujących miejsce startu i położenie stacji bazowej
    _state.origin.position = _psd.position;
    _state.base.position   = _state.origin.position;
    
    _vSem.unlock ();    
    
    // Exit condition is when detecting error in theta
	_state.exitCondition.setAccXOrAirspeedInterval(_psd.airspeed, _conf.autoTkfCatapultAirspeedRange, true, _conf.timeIntervalAspeed, fabsf(sinf(_psd.theta + _conf.autoTkfCatapultThetaMaxErr * DEG_2_RAD)), true);
    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (AUTO_TAKEOFF_CATAPULT_P2);  

    // Notify about end of takeoff procedure. (used in energy consumption during flight)
    notify (FPR_TAKEOFF_CPL);    

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
    
    // Debug message 
    Log.msgPrintf ("%s  -> Auto catapult takeoff phase 1 current theta: %.1f (deg) max error theta: %.1f (deg)", SUBS_PREFIX, _psd.theta * RAD_2_DEG, _conf.autoTkfCatapultThetaMaxErr);
}
   
/**
* Auto takeoff catapult phase 2. 
*/
void FlightPlanRealizer::autoTkfCatapultPhase2(void)
{
    if (!_vSem.lock ())
    {
        //  Send message to GCS
        Log.msgPrintf ("%s  -> Auto catapult takeoff phase 2 error (semaphore lock)", SUBS_PREFIX);
        //  Waits for operator action.
        return;
    }
	// Set state flag
	_state.tkfPhaseFlag = _state.TKF_P2;
    
	// Turn on Thr_Alt to gradually get throttle max
	_state.fprd.fRef.altitude = _conf.dfAltitude;

	// Make sure throttle at max level
	_flightConstraint = &_conf.flyConstraint;
	setSpdControllers(SPD_THR_SPD_ID);

	notify(FPR_TKF_P2);

    _vSem.unlock ();    
    
    // Turn off update from accelerometers
    notify(FPR_SMOOTH_ON);
   
	_state.resetConditions ();

    // Exit condition is timeOut
    _state.exitCondition.setInterval(_conf.autoTkfCatapultTimeout);
    
    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (AUTO_TAKEOFF_CATAPULT_P3);
    
    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
    
    // Debug message
	Log.msgPrintf ("%s  -> Auto catapult takeoff phase 2 timeout: %.3f (sec) at theta: %.1f (deg)", SUBS_PREFIX, _conf.autoTkfCatapultTimeout, _psd.theta * RAD_2_DEG);
}    

/**
* Auto takeoff catapult phase 3. 
*/
void FlightPlanRealizer::autoTkfCatapultPhase3(void)
{
    if (!_vSem.lock ())
    {
        //  Send message to GCS
        Log.msgPrintf ("%s  -> Auto catapult takeoff phase 3 error (semaphore lock)", SUBS_PREFIX);
        //  Waits for operator action.
        return;
    }

	// Set state flag
	_state.tkfPhaseFlag = _state.TKF_P3;
    
	// Do not use ThetaModifier
    _state.useThetaMod = false;

    // Turn on update from accelerometers
    notify(FPR_SMOOTH_OFF);
    
	notify(FPR_TKF_P2);

	 _flightConstraint = &_conf.flyConstraint;

     // Aileron to keep wings level
     _state.fprd.fRef.fRefLowLevel.phi = 0.0f;
	 setLonControllers (LON_HOLD_PHI_ID);
    _state.fprd.ctrlProps.L1CProp.setControllerParams(L1_HEADING_HOLD_ID, _conf.flyConstraint.Phi.minVal, _conf.flyConstraint.Phi.maxVal);
     // Turn on Vz_Alt Controller
	 _state.fprd.fRef.altitude = _conf.autoTkfCatapultClimbAlt;
     setLatControllers (LAT_VZ_ALT_ID);
    
    _vSem.unlock ();       
       
	_state.resetConditions (); 

    //  Ending condition is AGL altitude in greater than this
	_state.exitCondition.setAglRange (_conf.autoTkfCatapultClimbAlt - _conf.dfAltitudeDelta, _conf.autoTkfCatapultClimbAlt + _conf.dfAltitudeDelta);
    
    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (AUTO_BEGIN_MISSION);
    
    notify (FPR_NO_ATTENTION);
    
    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
    
    // Debug message
	Log.msgPrintf ("%s  -> Auto catapult takeoff phase 3 with target altitude: %.1f (m) from altitude: %.1f (m)", SUBS_PREFIX, _conf.autoTkfCatapultClimbAlt, _psd.altitude);
}

/**
* Going on mission
*/
void FlightPlanRealizer::autoBeginMission(void)
{
    // Allow to use ThetaModifier
    _state.useThetaMod = true;

	//Clear takeoff flag
	clearTkfFlag();
    
    // Enable indicating flag to signal ENGINE FAILURE
    _state.bAllowEngineMsg = true;
    
    // Notify complete takeoff
    fpCmdCompleted();
    
    //  Debug message
    Log.msgPrintf("%s -> Take off finished, begin mission", SUBS_PREFIX);
}

/**
* "Circle" flight plan instruction (circle with wind influence eleimination). 
*/
void FlightPlanRealizer::circle (const char* pTime, const char* pRadius, const char* pDir)
{
    bool bok = true;

    // Time of circle
    float time = 0.0f;
    if (pTime != NULL)
        bok = TypeParser::toFloat (pTime, time);
    else
        time = _conf.holdTime;

    //  Radius
    float radius = _conf.crcConstraint.radius * (1.0f - _conf.crcConstraint.radiusErr);
    if (bok)
        if (pRadius != NULL)
            bok = TypeParser::toFloat (pRadius, radius);

    // circle direction
    bool circleLeft = _conf.crcConstraint.left;;
    if (bok)
        if (pDir != NULL)
        {
            if (STRICMP (pDir, "left") == 0)
                circleLeft = true;
            else if (STRICMP (pDir, "right") == 0)
                circleLeft = false;
            else
                bok = false;
        }

    if (!bok)
    {

        //  Syntax error - notify that flight plan isntruction has been end. (get new one).
        Log.msgPrintf ("%s  -> Bad parameters [%s]", SUBS_PREFIX, "error - circle skipping");
        fpCmdCompleted ();
        return;
    }

    //  define reference value and track from which calculate the centre of the circle.
    GpsPosition basePoint = _psd.position;
    float baseTrack = _psd.track;
    Log.msgPrintf ("%s  -> Circle from current position - lon:%.6f lat:%.6f",
        SUBS_PREFIX, basePoint.getLon(), basePoint.getLat());

    // Centre of the circle.
    float moveTrack = GpsPosition::addTrack (baseTrack, circleLeft ? -90.0f : 90.0f);
    GpsPosition midPos;
    GpsPosition::movePosition (basePoint, moveTrack, radius, midPos);  //RF::
    circlePhase1 (&midPos, radius, circleLeft, time);
}


/** 
*  "Circle base" flight plan instruction. Circle with default parameters around base station.
*/
void FlightPlanRealizer::circleBase (void)
{

    circlePhase1 (&_state.base.position, _conf.crcConstraint.radius, _conf.crcConstraint.left, _conf.crcConstraint.baseTime,
        _conf.crcConstraint.baseAltitude + _state.base.elvOffset);

    //  Lock "manual" and "camera guide" modes.
    _state.doNotManualTurn = true;
    _state.doNotCameraGuide = true;

    // Turn off flight according to AGL. 
    PState->useElevationOffsetPar (false);
}

/**
*  "Circle base parachute" flight plan instruction. Circle with default parameters around base station.
*/
void FlightPlanRealizer::circleBasePara(void)
{

	//Set landing parachute flag
	_state.landParachutePhaseFlag = _state.LND_ESTIMATE_HEADING; 
	circleBaseParaPhase1(&_state.base.position, _conf.autoParaRadius, _conf.crcConstraint.left, _conf.autoParaBaseTime,
		_conf.crcConstraint.baseAltitude);

	//  Lock "manual" and "camera guide" modes.
	_state.doNotManualTurn = true;
	_state.doNotCameraGuide = true;

	// Turn off flight according to AGL. 
	PState->useElevationOffsetPar(false);

}

void FlightPlanRealizer::circleBasePreEstimateHeading(void)
{
	//Set landing parachute flag
	_state.landParachutePhaseFlag = _state.LND_PRELAND;
	circleBaseParaPhase1(&_state.base.position, _conf.autoParaRadius, _conf.crcConstraint.left, _conf.autoParaBaseTime,
		_conf.crcConstraint.baseAltitude);

	//  Lock "manual" and "camera guide" modes.
	_state.doNotManualTurn = true;
	_state.doNotCameraGuide = true;

	// Turn off flight according to AGL. 
	PState->useElevationOffsetPar(false);
}

/**
*  "Circle camguide" flight plan instruction. Circle with default parameters around base station.
*/
void FlightPlanRealizer::circleCamguide(void)
{

	circleCamguidePhase1(&_state.camguide.positionCamguide, _conf.crcConstraint.radius, _conf.crcConstraint.left, _conf.crcConstraint.baseTime,
		_conf.dfAltitude);

	//  Lock "manual" and "camera guide" modes.
	_state.doNotManualTurn = true;
	_state.doNotCameraGuide = true;

	// Turn off flight according to AGL. 
	PState->useElevationOffsetPar(false);
}

/**   
* "circle around..." flight plan instruction (circle around biven coordinates)
*  pRadius - circle radius [m]. when NULL - choose _conf.crcRadius
*  pDir - circle direction ("left" | "right"). when NULL - choose "left"
*  pAlt - desire altitude (change dfAltitude). when NULL - desire altitude will not change
*/
void FlightPlanRealizer::circleAround (const char* p1, const char* p2, const char* pMode, const char* pRadius,
                                       const char* pDir, const char* pAlt)
{
    GpsPosition pos;
    bool bok = true;

    //  Radius
    float radius = _conf.crcConstraint.radius * (1.0f - _conf.crcConstraint.radiusErr);
    if (pRadius != NULL)
        bok = TypeParser::toFloat (pRadius, radius);

    //  Circle direction
    bool circleLeft = _conf.crcConstraint.left;
    if (bok && (pDir != NULL))
    {
        if (STRICMP (pDir, "left") == 0)
            circleLeft = true;
        else if (STRICMP (pDir, "right") == 0)
            circleLeft = false;
        else
            bok = false;
    }

    //  Altitude
    float alt = _conf.dfAltitude;
    if  (bok && (pAlt != NULL))
    {
        bok = TypeParser::toFloat (pAlt, alt);
    }

    if (bok)
    {
        if (STRICMP (pMode, "abs") == 0)
            // parsing geographical coordinates treated as obsolute.
            bok = TypeParser::toGpsPosition (p1, p2, pos);
        else 
        {
            // parsing geographical coordinates treated as relative
            Log.msgPrintf ("%s  -> Relative (REL) position not implemented", SUBS_PREFIX);
            bok = false;
        }
    }

    if (!bok)
    {
        //  Syntax error - notify that flight plan isntruction has been end. (get new one).
        Log.msgPrintf ("%s  -> Bad parameters [%s]", SUBS_PREFIX, "error - circle skipping");
        fpCmdCompleted ();
        return;
    }

    //  Set default altitude.
    if (pAlt != NULL)
    {
        Log.msgPrintf ("%s  -> Set default altitude to: %.1fm (from: %.1fm)",
            SUBS_PREFIX, alt, _conf.dfAltitude);
        _conf.dfAltitude = alt;
    }

    circlePhase1 (&pos, radius, circleLeft, _conf.crcConstraint.normTime);
}

void FlightPlanRealizer::circleToClimb (double lat, double lon, float radius, float alt)
{
    GpsPosition pos;
	pos.setLat(lat);
	pos.setLon(lon);

    //  Circle direction
    bool circleLeft = _conf.crcConstraint.left;
    _circleToClimb = true;

	if (_psd.altitude > alt)
	{
		_circleToClimb = false;
		Log.msgPrintf ("%s  -> circle to climb skipped at %.1f (m) [%s]", SUBS_PREFIX, _psd.altitude, "ok");

		fpCmdCompleted ();
	}
	else
	{
		circlePhase1 (&pos, radius, circleLeft, _conf.crcConstraint.normTime, alt);
	}
}

void FlightPlanRealizer::circleToClimbLand (double lat, double lon, float radius, float alt)
{
	GpsPosition pos;
	pos.setLat(lat);
	pos.setLon(lon);
	//Set landing parachute flag
	_state.landParachutePhaseFlag = _state.LND_PRELAND;

	//  Circle direction
	bool circleLeft = _conf.crcConstraint.left;
	_circleToClimb = true;

	if (_psd.altitude > alt)
	{
		_circleToClimb = false;
		Log.msgPrintf ("%s  -> circle to climb skipped at %.1f (m) [%s]", SUBS_PREFIX, _psd.altitude, "ok");

		fpCmdCompleted ();
	}
	else
	{
		circlePhase1 (&pos, radius, circleLeft, _conf.crcConstraint.normTime, alt);
	}
}

#if EMRG_LOSS_ENGINE == 1
void FlightPlanRealizer::circleToDescent (double lat, double lon, float radius, float alt)
{
    GpsPosition pos;

    //  Circle direction
    bool circleLeft = _conf.crcConstraint.left;

	if ((_state.fprd.fRef.wptTo.getLat() == _state.base.position.getLat()) &&
		(_state.fprd.fRef.wptTo.getLon() == _state.base.position.getLon()))
	{
		pos.setLat(_state.base.position.getLat());
		pos.setLon(_state.base.position.getLon());
	}
	else
	{
		pos.setLat(lat);
		pos.setLon(lon);
		GpsPosition::movePosition(pos, circleLeft ? (_psd.track - 90.0f) : (_psd.track + 90.0f), radius, pos);
	}

    _circleToDescent = true;

	if (_psd.altitude < alt)
	{
		_circleToDescent = false;
		Log.msgPrintf ("%s  -> circle to descent skipped at %.1f (m) [%s]", SUBS_PREFIX, _psd.altitude, "ok");

		fpCmdCompleted ();
	}
	else
	{
		circlePhase1 (&pos, radius, circleLeft, _conf.crcConstraint.normTime);
	}
}
#endif

/**  
*  Low level "circle" instruction. All parametrs are needed except altitude ( when altitude ==0.0f then circle is at the default altitude - dfAltitude)
*/
void FlightPlanRealizer::circlePhase1 (const GpsPosition* midPos, float radius, bool circleLeft, float time, float altitude)
{
    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();
 
    //  Set flags that cause simultaneous change of ref.altitude and ref.airspeed after default values has been changed (alg.dfAltitude & alg.dfAirspeed)
    //  If altitude was given as a parameter (call from CircleBase) reference altitude could not be change automatically based on default value. It is controlled by the special parameter.
    if (!(altitude > 0.0f))
        _state.useUafSetDfltAlt = true;
    _state.useUafSetDfltAirSpd = true;  //  airspeed during the flight to the circle point.

    //  Turn on flag that cause immediate skip to next flight plan instruction after turning on "manual turn" or "camera guide" mode.
    _state.skipCommandInMT = false; // RF:: true;
    _state.skipCommandInCG = false; // RF:: true;

    //  Allow to re-execute Circle instruction and other instructions executing CirclePhase1 instruction after parameters that has UAF_REDO_CIRCLE flag changed.
    _state.useUafRedoCircle = true;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - circle skipping");
        fpCmdCompleted ();
        return;
    }

	_state.fprd.fRef.wptFrom = _state.fprd.fRef.wptTo;
    _state.fprd.fRef.wptTo = *midPos;
	_state.fprd.fRef.circleRadius = radius;
    _state.fprd.fRef.circleLeft = circleLeft;
    // Set circle time needed for 2 phase.
    _state.circleTime = time;

#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
	{
		_state.fprd.fRef.altitude = _conf.linkBrokenMaxAlt;
	}
	else
#endif
		if (altitude > 0.0f)
		{
			// Set new reference altitude.
			_state.fprd.fRef.altitude = altitude;
		}
		else
		{
			_state.fprd.fRef.altitude = _conf.dfAltitude;
		}


    // Default airspeed
    _state.fprd.fRef.airspeed = _conf.dfAirspeed;

    //  Set controllers like as those in straight flight (fly).
	if (GpsPosition::distance(_psd.position, *midPos) > radius * 1.41f)
	{
		setControllers(FLY_MODE);
	}

    _vSem.unlock ();

    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions ();

    //  Warunek będzie spełniony przy odległości mniejszej niż 1,41 promienia, co daje 45 stopni odchyłki
    //  od trasy bezpośrednio do punktu. Współczynnik może być inny.

    // The condition will be satisfied at a distance less than the radius of 1.41, which gives a 45 degree deviation from the route directly to the point. This factor may be different.
    _state.exitCondition.setPointDistance (*midPos, radius * 2.0f, false);

    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (CRC_PHASE_2);

    // Set stall recovery procedures executing condition.
    setStallRecovery ();

    // Switch on condition that cause using Phi_Psi controller.
    setLowGSpeedCheck ();

    //  Turn on condition that execute emergency landing procedure after altitude has dropped.
    setEmergencyLandingCheck ();

    Log.msgPrintf ("%s  -> Circle Phase 1 (fly) around lon:%.6f lat:%.6f [%s]",
        SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}


/** 
* Second phase of the "Circle" instruction.
*/
void FlightPlanRealizer::circlePhase2 (void)
{
    //  Turn off automatic change of ref.Airspeed after dfAirspeed changing (but turn on is after crcAirspeed)
    _state.useUafSetDfltAirSpd = false;  //  airspeed at flight to the circle point.
    _state.useUafSetCrcAirSpd = true;    //  circle airspeed.

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
    }

    Log.msgPrintf ("%s  -> Circle Phase 2 around lon:%.6f lat:%.6f [%s]",
        SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

    _state.fprd.fRef.airspeed = _conf.crcConstraint.airspeed;      //  Circle airspeed
	setControllers (CRC_MODE);

    _vSem.unlock ();

    // Delete all set conditions and set time condition given as parameter.
    _state.resetConditions ();
	if (_circleToClimb)
	{
		_circleToClimb = false;
		_state.exitCondition.setAgl (_conf.linkBrokenMaxAlt - _conf.linkBrokenAltZone, true);
	}
#if EMRG_LOSS_ENGINE == 1
	else if (_circleToDescent)
	{
		_circleToDescent = false;
		//  Lock the possibility of instruction interruption from the FPlan subsystem.
		_state.setDoNotBreakFlag (true); 
		_state.exitCondition.setAgl (_conf.emrgAltitude, false);
	}
#endif
	else
	{
		_state.exitCondition.setInterval (_state.circleTime);
	}

    // Set stall recovery procedures executing condition.
    setStallRecovery ();

    // Switch on condition that cause using Phi_Psi controller.
    setLowGSpeedCheck ();

    //  Turn on condition that execute emergency landing procedure after altitude has dropped.
    setEmergencyLandingCheck ();

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}

/**
*  Low level "circle base parachute" instruction. All parametrs are needed except altitude ( when altitude ==0.0f then circle is at the default altitude - dfAltitude)
*/
void FlightPlanRealizer::circleBaseParaPhase1(const GpsPosition* midPos, float radius, bool circleLeft, float time, float altitude)
{
	// reset flags associated with locking breaking instruction and manual turn mode. 
	_state.resetMode();

	//  Set flags that cause simultaneous change of ref.altitude and ref.airspeed after default values has been changed (alg.dfAltitude & alg.dfAirspeed)
	//  If altitude was given as a parameter (call from CircleBase) reference altitude could not be change automatically based on default value. It is controlled by the special parameter.
	if (!(altitude > 0.0f))
		_state.useUafSetDfltAlt = true;
	_state.useUafSetDfltAirSpd = true;  //  airspeed during the flight to the circle point.

	//  Turn on flag that cause immediate skip to next flight plan instruction after turning on "manual turn" or "camera guide" mode.
	_state.skipCommandInMT = false; // RF:: true;
	_state.skipCommandInCG = false; // RF:: true;

	//  Allow to re-execute Circle instruction and other instructions executing CirclePhase1 instruction after parameters that has UAF_REDO_CIRCLE flag changed.
	_state.useUafRedoCircle = true;

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - circle skipping");
		fpCmdCompleted();
		return;
	}

	_state.fprd.fRef.wptFrom = _state.fprd.fRef.wptTo;
	_state.fprd.fRef.wptTo = *midPos;
	_state.fprd.fRef.circleRadius = radius;
	_state.fprd.fRef.circleLeft = circleLeft;

#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
	{
		_state.fprd.fRef.altitude = _conf.linkBrokenMaxAlt;
	}
	else
#endif
	if (altitude > 0.0f)
	{
		// Set new reference altitude.
		_state.fprd.fRef.altitude = altitude;
	}
	else
	{
		_state.fprd.fRef.altitude = _conf.dfAltitude;
	}


	// Default airspeed
	_state.fprd.fRef.airspeed = _conf.dfAirspeed;

	//  Set controllers like as those in straight flight (fly).
	if (GpsPosition::distance(_psd.position, *midPos) > radius * 1.41f)
	{
		setControllers(FLY_MODE);
	}

	_vSem.unlock();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();

	//  Warunek będzie spełniony przy odległości mniejszej niż 1,41 promienia, co daje 45 stopni odchyłki
	//  od trasy bezpośrednio do punktu. Współczynnik może być inny.

	// The condition will be satisfied at a distance less than the radius of 1.41, which gives a 45 degree deviation from the route directly to the point. This factor may be different.
	_state.exitCondition.setPointDistance(*midPos, radius * 2.0f, false);

	//  Set function that will be executed after fulfilment of the condition.
	_state.exitCondition.setChainFunction(CRC_BASE_PARA_PHASE_2);

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Parachute landing: Circle Base Phase 1 (fly) around lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/**
*  Low level "circle camguide" instruction. All parametrs are needed except altitude ( when altitude ==0.0f then circle is at the default altitude - dfAltitude)
*/
void FlightPlanRealizer::circleCamguidePhase1(const GpsPosition* midPos, float radius, bool circleLeft, float time, float altitude)
{
	// reset flags associated with locking breaking instruction and manual turn mode. 
	_state.resetMode();

	//  Set flags that cause simultaneous change of ref.altitude and ref.airspeed after default values has been changed (alg.dfAltitude & alg.dfAirspeed)
	//  If altitude was given as a parameter (call from CircleBase) reference altitude could not be change automatically based on default value. It is controlled by the special parameter.
	if (!(altitude > 0.0f))
		_state.useUafSetDfltAlt = true;
	_state.useUafSetDfltAirSpd = true;  //  airspeed during the flight to the circle point.

	//  Turn on flag that cause immediate skip to next flight plan instruction after turning on "manual turn" or "camera guide" mode.
	_state.skipCommandInMT = false; // RF:: true;
	_state.skipCommandInCG = false; // RF:: true;

	//  Allow to re-execute Circle instruction and other instructions executing CirclePhase1 instruction after parameters that has UAF_REDO_CIRCLE flag changed.
	_state.useUafRedoCircle = true;

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "error - circle skipping");
		fpCmdCompleted();
		return;
	}

	_state.fprd.fRef.wptFrom = _state.fprd.fRef.wptTo;
	_state.fprd.fRef.wptTo = *midPos;
	_state.fprd.fRef.circleRadius = radius;
	_state.fprd.fRef.circleLeft = circleLeft;
	// Set circle time needed for 2 phase.
	_state.circleTime = time;

#if EMRG_LOSS_ENGINE == 1
	if (SysMon->getLossEngine())
	{
		_state.fprd.fRef.altitude = _conf.linkBrokenMaxAlt;
	}
	else
#endif
	if (altitude > 0.0f)
	{
		// Set new reference altitude.
		_state.fprd.fRef.altitude = altitude;
	}
	else
	{
		_state.fprd.fRef.altitude = _conf.dfAltitude;
	}


	// Default airspeed
	_state.fprd.fRef.airspeed = _conf.dfAirspeed;

	//  Set controllers like as those in straight flight (fly).
	if (GpsPosition::distance(_psd.position, *midPos) > radius * 1.41f)
	{
		setControllers(FLY_MODE);
	}

	_vSem.unlock();

	// Delete all set conditions and set condition determining point attainment.
	_state.resetConditions();

	//  Warunek będzie spełniony przy odległości mniejszej niż 1,41 promienia, co daje 45 stopni odchyłki
	//  od trasy bezpośrednio do punktu. Współczynnik może być inny.

	// The condition will be satisfied at a distance less than the radius of 1.41, which gives a 45 degree deviation from the route directly to the point. This factor may be different.
	_state.exitCondition.setPointDistance(*midPos, radius * 2.0f, false);

	//  Set function that will be executed after fulfilment of the condition.
	_state.exitCondition.setChainFunction(CRC_CRC_CAMGUIDE_PHASE_2);

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Circle Camguide Phase 1 (fly) around lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}


/**
* Second phase of the "Circle base parachute" instruction.
*/
void FlightPlanRealizer::circleBaseParaPhase2(void)
{
	//  Turn off automatic change of ref.Airspeed after dfAirspeed changing (but turn on is after crcAirspeed)
	_state.useUafSetDfltAirSpd = false;  //  airspeed at flight to the circle point.
	_state.useUafSetCrcAirSpd = true;    //  circle airspeed.

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
	}

	_state.fprd.fRef.airspeed = _conf.dfAirspeed;      //  Circle airspeed

	//
	_state.fprd.fRef.altitude = _conf.crcConstraint.baseAltitude;

	setControllers(CRC_MODE);

	_vSem.unlock();

	// Delete all set conditions and set time condition given as parameter.
	_state.resetConditions();

	if (_state.landParachutePhaseFlag == _state.LND_PRELAND)
	{
			//Ending condition is AGL altitude in a range
		_state.exitCondition.setAglRange(_conf.crcConstraint.baseAltitude - _conf.dfAltitudeDelta, _conf.crcConstraint.baseAltitude + _conf.dfAltitudeDelta);
	}
	else
	{
		// The condition will be satisfied after an interval
		_state.exitCondition.setInterval(_conf.autoParaBaseTime);
	}

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	Log.msgPrintf("%s  -> Parachute landing: Circle Base Phase 2 around lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}

/**
* Second phase of the "Circle" instruction.
*/
void FlightPlanRealizer::circleCamguidePhase2(void)
{
	//  Turn off automatic change of ref.Airspeed after dfAirspeed changing (but turn on is after crcAirspeed)
	_state.useUafSetDfltAirSpd = false;  //  airspeed at flight to the circle point.
	_state.useUafSetCrcAirSpd = true;    //  circle airspeed.
	_state.useUafSetDfltAlt = true;

	if (!_vSem.lock())
	{
		Log.msgPrintf("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
	}

	Log.msgPrintf("%s  -> Circle Camguide Phase 2 around lon:%.6f lat:%.6f [%s]",
		SUBS_PREFIX, _state.fprd.fRef.wptTo.getLon(), _state.fprd.fRef.wptTo.getLat(), "ok");

	_state.fprd.fRef.airspeed = _conf.crcConstraint.airspeed;      //  Circle airspeed
	_state.fprd.fRef.altitude = _conf.dfAltitude;
	setControllers(CRC_MODE);

	_vSem.unlock();

	// Delete all set conditions and set time condition given as parameter.
	_state.resetConditions();
	if (_circleToClimb)
	{
		_circleToClimb = false;
		_state.exitCondition.setAgl(_conf.linkBrokenMaxAlt - _conf.linkBrokenAltZone, true);
	}
#if EMRG_LOSS_ENGINE == 1
	else if (_circleToDescent)
	{
		_circleToDescent = false;
		//  Lock the possibility of instruction interruption from the FPlan subsystem.
		_state.setDoNotBreakFlag(true);
		_state.exitCondition.setAgl(_conf.emrgAltitude, false);
	}
#endif
	else
	{
		_state.exitCondition.setInterval(_state.circleTime);
	}

	// Set stall recovery procedures executing condition.
	setStallRecovery();

	// Switch on condition that cause using Phi_Psi controller.
	setLowGSpeedCheck();

	//  Turn on condition that execute emergency landing procedure after altitude has dropped.
	setEmergencyLandingCheck();

	//  Send notify to other subsystems about FPReal settings had been changed.
	notify(FPR_CHANGED);
}



/** 
* Force land instruction. 1 phase.
*/
bool FlightPlanRealizer::forceLand (void)
{
    if (SystemNowOnGround)
    {
        Log.msgPrintf ("%s  -> Aircraft is on ground - forced landing ignored", SUBS_PREFIX);
        return false;
    }

    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();

    //  Lock "manual" and "camera guide" modes.
    _state.doNotManualTurn = true;
    _state.doNotCameraGuide = true;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue landing");
    }

	//  Lock the possibility of instruction interruption from the FPlan subsystem.
    _state.setDoNotBreakFlag (true); 

    //  Turn off all controllers.
    _state.fprd.ctrlProps.disableControllers();
    //  Reset outputs.
    _state.fprd.outCtrl.fCtrl.reset();

    //  Oblong track
    //  Pitch is beeing set after butterfly.
    _state.fprd.fRef.fRefLowLevel.theta = _conf.fLndTheta;

    _state.fprd.ctrlProps.QCProp.Q_Theta.setControllerParams(_conf.flyConstraint.Q.minVal, _conf.flyConstraint.Q.maxVal);
    _state.fprd.ctrlProps.ElvCProp.Elv_Q.setControllerParams(_conf.flyConstraint.Elv.minVal, _conf.flyConstraint.Elv.maxVal);

    //  Cross track - tilt regulation depends on course error.
    //  Set course reference value (psi)
    _state.fprd.fRef.fRefLowLevel.psi = _psd.psi;

    _state.fprd.ctrlProps.PhiCProp.Phi_Psi.setControllerParams(-_conf.fLndMaxAbsPhi, _conf.fLndMaxAbsPhi);
    _state.fprd.ctrlProps.PCProp.P_Phi.setControllerParams(_conf.flyConstraint.P.minVal, _conf.flyConstraint.P.maxVal);
    _state.fprd.ctrlProps.AlrCProp.Alr_P.setControllerParams(_conf.flyConstraint.Alr.minVal, _conf.flyConstraint.Alr.maxVal);

    //  Rudder
    _state.fprd.ctrlProps.RCProp.R_CoordExp.setControllerParams(_conf.flyConstraint.R.minVal, _conf.flyConstraint.R.maxVal);
    _state.fprd.ctrlProps.RudderCProp.Rdr_R.setControllerParams(_conf.flyConstraint.Rdr.minVal, _conf.flyConstraint.Rdr.maxVal);

    _vSem.unlock ();

    // Delete all conditions. 
    _state.resetConditions ();

    // Set time needed to stop engine.
    _state.exitCondition.setInterval (_conf.fLndThrOffTime);

    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (FLND_PHASE_2);

    Log.msgPrintf ("%s  -> Forced landing agl:%.1fm lon:%.6f lat:%.6f", SUBS_PREFIX,
        _psd.altitude, _psd.position.getLon(), _psd.position.getLat());

    // Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);

    // Notify to external subsystems
    Log.eventPrint (EVT_FPR_TODROP, "1");

    // Notify that something interesting is happening. Pay attention.
    notify (FPR_ATTENTION);

    return true;
}


/** 
* Force land 2 phase. (butterfly)
*/
void FlightPlanRealizer::forceLandPhase2 (void)
{
    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue landing");
    }

    //  Turn on butterfly
    _state.fprd.outCtrl.fCtrl.butterfly = _conf.fLndBtfly;

    _vSem.unlock ();

    //  Acceleration in X axis condition.
    _state.exitCondition.setAccX (_conf.fLndLandedAccX, false);

    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (FLND_PHASE_3);

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}


/**
* Force land isntruction 3 phase. (finishing the landing)
*/
void FlightPlanRealizer::forceLandPhase3 (void)
{
    Log.msgPrintf ("%s  -> Forced landing completed agl:%.1fm lon:%.6f lat:%.6f", SUBS_PREFIX,
        _psd.altitude, _psd.position.getLon(), _psd.position.getLat());

    //  Turn off controllers. Set servos in neutral position.
    // Strange behaviour in simulation mode omit this.
    if (PState->getSimLevel() == 0)
        disableControllers (false);
    else
        Log.msgPrintf ("%s  -> SimLevel > 0 - Disable Controllers skipped", SUBS_PREFIX);

    //  Last phase. Lack of exit conditions.
    //  Notify to the subsystems about the landing has been finished.
    notify (FPR_LANDED);

    // Notify that something interesting has finished.
    notify (FPR_NO_ATTENTION);
}


/*
* "goto <line id>" flight plan instruction - 1 phase (delay when it is executing too fast).
*/
void FlightPlanRealizer::fpGoto (const char* pLineId)
{
    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();

    if (TypeParser::toInt(pLineId, _state.gotoLineId))
    {
        // check elapsed time from the previous goto instruction.
        int d = _psd.time100 - _state.lastGotoTimeStamp;

        // If elapsed less than 0.5 second make delay 0.5 second.
        float t = 0.0f;
        if (d < 5000)   // 0.5s
            t = 0.5f;

        // Delete all set conditions.
        _state.resetConditions ();

        //  Set time needed to stop the engine.
        _state.exitCondition.setInterval (t);

        //  Set function that will be executed after fulfilment of the condition.
        _state.exitCondition.setChainFunction (GOTO_PHASE_2);

        Log.msgPrintf ("%sGoto line: %i, wait: %.1fs", SUBS_PREFIX, _state.gotoLineId, t);
    }
    else
    {
        Log.msgPrintf ("%sBad line id - skip to next line", SUBS_PREFIX);
        //  In case of an error go to the next instruction in flight plan.
        fpCmdCompleted ();
    }
}

/** 
* Second phase of the "goto <line id>" instruction.
*/
void FlightPlanRealizer::fpGotoPhase2 (void)
{
    //  Save new time
    _state.lastGotoTimeStamp = _psd.time100;

    FPlan->gotoLine(_state.gotoLineId);
}


void FlightPlanRealizer::standby (void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sSystem is airborne - Standby ignored", SUBS_PREFIX);
        return;
    }

    Log.msgPrintf ("%sStandby", SUBS_PREFIX);

    // Clean subsystem state - settings including only one flight plan istruction.
    _state.resetMode();
    // Delete all set conditions and set condition determining point attainment.
    _state.resetConditions();
    // Clean subsystem settings including many of the flight plan instructions.
    _state.fprd.other.doNotUseAirspeed = false;

    //  Turn off all controllers.
    _state.fprd.ctrlProps.disableControllers();
    //  Reset outputs.
    _state.fprd.outCtrl.fCtrl.reset();

    //  Switch PhysicalState in the mode where airspeed reading are included.(for the AHRS)
    PState->setIasFailMode (false);

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);

    // Notify that something interesting has finished..
    notify (FPR_NO_ATTENTION);

    // Notify that FPReal had been switched to Standby mode.
    notify (FPR_STANDBY);
}


void FlightPlanRealizer::onHold (void)
{
    Log.msgPrintf ("%sOnHold", SUBS_PREFIX);

    //  Define the centre of the circle.
    float moveTrack = GpsPosition::addTrack (_psd.track, _conf.holdLeft ? -90.0f : 90.0f);
    GpsPosition midPos;
    GpsPosition::movePosition (_psd.position, moveTrack, _conf.holdRadius, midPos);

    circlePhase1 (&midPos, _conf.holdRadius, _conf.holdLeft, _conf.holdTime);
}


/**
* stall recovery function. It is executed like the next flight phase after fulfilment of the conditions set in the stallCondition object.
*/
void FlightPlanRealizer::stallOn (void)
{
    if (!_state.stall)
    {
        Log.msgPrintf ("%s  -> Stall recovery on", SUBS_PREFIX);

        _state.stall = true;

        //  Lock "manual" and "camera guide" modes and save previous value.
        _state.stallSavedDoNotMT = _state.doNotManualTurn;
        _state.stallSavedDoNotCG = _state.doNotCameraGuide;
        _state.doNotManualTurn = true;
        _state.doNotCameraGuide = true;

        //  Save all current settings of the controllers and outputs.
        _state.stallSavedFprd = _state.fprd;

        if (!_vSem.lock ())
        {
            Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue stall recovery");
        }

        // Turn off all controllers. (without resetting outputs to not change butterfly and flaps)
        _state.fprd.ctrlProps.disableControllers();

        //  Elevatro set to 0.
        _state.fprd.outCtrl.fCtrl.elevator = _conf.stallRecElevator;

        // Engine throtle set to minimum value.
        _state.fprd.outCtrl.fCtrl.throttle = _conf.flyConstraint.Thr.minVal; 
        // Rudder stabilizes 0 R speed.
        _state.fprd.fRef.fRefLowLevel.R = 0.0f;
        _state.fprd.ctrlProps.RudderCProp.Rdr_R.setControllerParams(_conf.flyConstraint.Rdr.minVal, _conf.flyConstraint.Rdr.maxVal);
        
        // Ailerons stabilizes phi=0
        _state.fprd.fRef.fRefLowLevel.phi = 0.0f;
        _state.fprd.ctrlProps.AlrCProp.Alr_P.setControllerParams(_conf.flyConstraint.Alr.minVal, _conf.flyConstraint.Alr.maxVal);
        _state.fprd.ctrlProps.PCProp.P_Phi.setControllerParams(_conf.flyConstraint.P.minVal, _conf.flyConstraint.P.maxVal);
                
        _vSem.unlock ();

        // Save current exit conditions from the phase and set new one.
        _state.stallSavedExitCondition = _state.exitCondition;
        _state.exitCondition.reset();
        _state.exitCondition.setMainTimeout (_conf.stallRecTimeOff);         // Maximum time of the stall recovery try.
        _state.exitCondition.setAirspeed (_conf.stallRecAirspeedOff, true);  // Speed after which exit.

        //  Set function that will be executed after fulfilment of the condition.
        _state.exitCondition.setChainFunction (STALL_OFF);

        //  Send notify to other subsystems about FPReal settings had been changed.
        notify (FPR_CHANGED);
    }
}


/** 
* Procedure to return to the flight phase from which stall recovery function was called.
*/
void FlightPlanRealizer::stallOff (void)
{
    Log.msgPrintf ("%s  -> Stall recovery off", SUBS_PREFIX);

    _state.stall = false;

    // Restore previous value of the "Manual turn" mode.
    _state.doNotManualTurn = _state.stallSavedDoNotMT;
    _state.doNotCameraGuide = _state.stallSavedDoNotCG;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue stall off");
    }

    // Restore all controllers and outputs settings.
    _state.fprd = _state.stallSavedFprd;

    _vSem.unlock ();

    //  Restore saved phase exit conditions.
    _state.exitCondition = _state.stallSavedExitCondition;

    // re-set stall recovery procedures executing condition.
    setStallRecovery ();

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}


/** 
* Operating procedure for decrease ground speed. It is called upon detection lowGSpeedCondition condition.
*/
void FlightPlanRealizer::lowGSpeedOn (void)
{
    //  Procedure is executed only when one of the controller Phi_Track or Phi_CTrack is turned on.
    if (!_state.lowGSpeed && (_state.fprd.ctrlProps.PhiCProp.Phi_Track.enable || _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable))
    {
        Log.msgPrintf ("%s  -> Low Ground Speed (Phi_Psi): on (gspd: %.1fkph)", SUBS_PREFIX, _psd.groundspeed);

        _state.lowGSpeed = true;

        // Save current controllers settings.
        _state.lowGSpeedSavedCtrlProps = _state.fprd.ctrlProps;

        if (!_vSem.lock ())
        {
            Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
        }

        // Switch controllers from the Track to tje Psi. Min and Max Phi is set the same like in the turned off controller.
        _state.fprd.ctrlProps.PhiCProp.Phi_Psi.reset();
        _state.fprd.ctrlProps.PhiCProp.Phi_Psi.enable = true;
        _state.fprd.ctrlProps.PhiCProp.Phi_Psi.minValue = _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable ?
            _state.fprd.ctrlProps.PhiCProp.Phi_Track.minValue : _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.minValue;
        _state.fprd.ctrlProps.PhiCProp.Phi_Psi.maxValue = _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable ?
            _state.fprd.ctrlProps.PhiCProp.Phi_Track.maxValue : _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.maxValue;

        _state.fprd.ctrlProps.PhiCProp.Phi_Track.enable = false;
        _state.fprd.ctrlProps.PhiCProp.Phi_CTrack.enable = false;

        // Set track (psi) reference value for current value. 
        _state.fprd.fRef.fRefLowLevel.psi = _psd.psi;

        _vSem.unlock ();

        // Change speed condition - now will be incerease of the speed due to restore Phi_Track controller.
        _state.lowGSpeedCondition.reset();
        _state.lowGSpeedCondition.setGroundspeed (_conf.gSpeedMinValue, true);
        // The condition for a flight in the specified range of angles of the track. Will not be met when the plane goes back.
        float trackLeft = GpsPosition::addTrack (_psd.track, -90.0f);
        float trackRight = GpsPosition::addTrack (_psd.track, 90.0f);
        _state.lowGSpeedCondition.setTrackRange (trackLeft, trackRight);

       //  Set function that will be executed after fulfilment of the condition.
        _state.lowGSpeedCondition.setChainFunction (LOW_GSPD_OFF);

        //  Send notify to other subsystems about FPReal settings had been changed.
        notify (FPR_CHANGED);
    }
}


/**  
* Procedure that handle the decrease ground speed - return after restoring speed.
* Function will be execute after lowGSpeedCondition (changed for detection of the speed incerease) condtion has been detected.
*/
void FlightPlanRealizer::lowGSpeedOff (void)
{
    if (_state.lowGSpeed)
    {
        Log.msgPrintf ("%s  -> Low Ground Speed (Phi_Psi): off (gspd: %.1fkph)", SUBS_PREFIX, _psd.groundspeed);

        _state.lowGSpeed = false;

        if (!_vSem.lock ())
        {
            Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
        }

        //  Restore all controllers settings.
        _state.fprd.ctrlProps = _state.lowGSpeedSavedCtrlProps;

        _vSem.unlock ();

        // Set again detection of the decrease of ground speed.
        _state.lowGSpeedCondition.reset();
        _state.lowGSpeedCondition.setGroundspeed (_conf.gSpeedMinValue, false);

        //  Set function that will be executed after fulfilment of the condition.
        _state.lowGSpeedCondition.setChainFunction (LOW_GSPD_ON);

        //  Send notify to other subsystems about FPReal settings had been changed.
        notify (FPR_CHANGED);
    }
}

/* 
* Function that realize "ias fault on | off" commands.
*/
void FlightPlanRealizer::iasFault(bool on)
{
    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
    }

    FPRealData*  pfprd = &_state.fprd;
    // If currently stall recovery function is executing (_state.fprd is swaped) setting will not been change (stall recovery will continue).
    // All changes are set in saved state that will be restore after stall recovery function ends. 
    if (_state.stall)
        pfprd = &_state.stallSavedFprd;

    //  Set flag that prohibit use of speed indications.
    pfprd->other.doNotUseAirspeed = on;

    if (on)
    {
        // Set constant pitch Theta, when currently is being useg Theta_Speed controller.
        // When it is not turned on it's mean that flight is being realized with constant pitch. 
        if (pfprd->ctrlProps.ThetaCProp.Theta_Speed.enable)
        {
            // Flight with constant pitch - Theta.
            pfprd->fRef.fRefLowLevel.theta = _conf.pressFaultTheta;
        }

        if (pfprd->ctrlProps.FlpCProp.Flp_Speed.enable)
        {
            //  Turn off flpas regulator after speed indicator malfunction.
            pfprd->outCtrl.fCtrl.flaps = 0.0f;
        }

        // Turn off condition to detect stall. Condition will not be set again because prohibits it "doNotUseAirspeed" flag in the "setStallRecovery" function.
        // If currently exectuing stall recovery function will be done to the end but will not be reexecuted. (if stall conditions are fulfill)
        _state.stallCondition.reset();

        //  Switch PhysicalState in the mode where airspeed reading are ignored.(for the AHRS)
        PState->setIasFailMode (true);

    }
    else
    {
        //  Switch PhysicalState in the mode where airspeed reading are included.(for the AHRS)
        PState->setIasFailMode (false);
    }

    _vSem.unlock ();

    Log.msgPrintf ("%sIAS fault mode: %s", SUBS_PREFIX, (on ? "on" : "off"));

    //  Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}


/* 
* Function that set cotrollers parameters for straight flight.
* It is used in "fly" instruction and first "circle" phases.
*/
void FlightPlanRealizer::setControllers (FLIGHT_MODE flightMode)
{
	FControl->getOutputData(_state.fprd.outCtrl);
	FControl->getFlightRefLowLevelData(_state.fprd.fRef.fRefLowLevel);

	if (flightMode == FLY_MODE)			
		_flightConstraint = &_conf.flyConstraint;
	else if (flightMode == CRC_MODE)
		_flightConstraint = &_conf.crcConstraint;

    if (_state.fprd.other.doNotUseAirspeed)
		_state.fprd.fRef.fRefLowLevel.theta = _conf.pressFaultTheta;

	if (_state.fprd.other.doNotUseAirspeed)
        _state.fprd.outCtrl.fCtrl.flaps = 0.0f;

	if (_bFirstSetControllers || !_ramStorage.bTuneEnable)
	{
		_bFirstSetControllers = false;
		_bTuneMode = false;

		if (flightMode == CRC_MODE)
			setLonControllers (LON_CRC_WPT_ID, LON_R_COORDEXP_ID);
		else if (flightMode == FLY_MODE && _conf.bUseCorrTrack)
			setLonControllers (LON_TRACK_PATH_ID, LON_R_COORDEXP_ID);
		else
			setLonControllers (LON_TRACK_WPT_ID, LON_R_COORDEXP_ID);

#if EMRG_LOSS_ENGINE == 1
		if (SysMon->getLossEngine())
		{
			_state.fprd.fRef.airspeed = _conf.emrgAirspeedRef;
			setLatControllers (LAT_HOLD_SPD_ID);
			setSpdControllers (SPD_THR_ALT_ID, SPD_FLP_SPD_ID);
			_state.fprd.outCtrl.fCtrl.throttle = 0.0f;
			return;
		}
#endif
		// if (FPlan->getFlightPhase() == 1)
		// {
		// 	setLatControllers (LAT_HOLD_SPD_ID);
		// 	setSpdControllers (SPD_THR_ALT_ID, SPD_FLP_SPD_ID);
		// }
		// if (FPlan->getFlightPhase() == 2)
		// {
		// 	setLatControllers (LAT_HOLD_SPD_ID);
		// 	setSpdControllers (SPD_THR_ALT_ID, SPD_FLP_SPD_ID);
		// }
		// if (FPlan->getFlightPhase() == 3)
		// {
		// 	setLatControllers (LAT_VZ_ALT_ID);
		// 	setSpdControllers (SPD_THR_SPD_ID, SPD_FLP_SPD_ID);
		// }
        setLatControllers(LAT_HOLD_THETA_ID);
        setSpdControllers(TECS_ID, SPD_FLP_SPD_ID);
    }
	else
	{
		if (!_bTuneMode)
		{
			_bTuneMode = true;
			_ramStorage.reset ();
		}
		setTuneControllers ();
	}
}

/**  
* Setting executing stall recovery function conditions.
*/
void FlightPlanRealizer::setStallRecovery (void)
{
    // stall recovery function must be turned on in the configuration and could not be any pressure indicator malfunctions.
    if (_conf.stallRecEnable && !_state.fprd.other.doNotUseAirspeed)
    {
        //  Speed ​​below which entry into the function
        _state.stallCondition.setAirspeed (_conf.stallRecAirspeedOn, false);
        // Additional conditions for pitch and pitch speed (must bee greater than given).
        _state.stallCondition.setPhiOrP (_conf.stallRecPhiOn, _conf.stallRecPOn);
    }
}


/**
* Turn on procedure to maitance decrease of ground speed  (works only to the end of flight phase)
*/
void FlightPlanRealizer::setLowGSpeedCheck (void)
{
    //  Set detection of the decrease of ground speed.
    _state.lowGSpeedCondition.reset();
    _state.lowGSpeedCondition.setGroundspeed (_conf.gSpeedMinValue, false);

    //  Set function that will be executed after fulfilment of the condition.
    _state.lowGSpeedCondition.setChainFunction (LOW_GSPD_ON);
}


/** 
* Turn on procedure to maitance decrease of ground speed (turn on automatically at the end of flight phase)
*/
void FlightPlanRealizer::resetLowGSpeedCheck (void)
{
    // Turn on checking decrease or increase of the ground speed.
    _state.lowGSpeedCondition.reset();

    if (_state.lowGSpeed)
    {
        Log.msgPrintf ("%s  -> Low Ground Speed (Phi_Psi): end phase off (gspd: %.1fkph)", SUBS_PREFIX, _psd.groundspeed);

        _state.lowGSpeed = false;

        if (!_vSem.lock ())
        {
            Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue");
        }

        //  Restore all controllers settings.
        _state.fprd.ctrlProps = _state.lowGSpeedSavedCtrlProps;

        _vSem.unlock ();
    }
}


/**  
*  Turn on condition that execute emergency landing procedure after altitude has dropped.
*/
void FlightPlanRealizer::setEmergencyLandingCheck (void)
{
    //  Emergency landing after decrease of the altitude must be turned on in configuration.
    if (_conf.emrgLandEnable)
    {
        //  Condition for altitude less than given if it was before greater (+ margin)
        _state.emrgLandingCondition.setEmrgLandAgl (_conf.emrgLandAgl, _conf.emrgLandMrg);
    }
}

/**
* Pre land phase 1. 
*/
void FlightPlanRealizer::netPrelandCompute(const char* p1, const char* p2, const char* p3, const char* p4, const char* p5, const char* p6)
{
    // Get postion of the recovery net
    if (!TypeParser::toGpsPosition(p1, p2, _state.nLand.net.position))
    {
        Log.errorPrintf("preLndCompute_1");
        fpCmdCompleted();
        return;
    }

    // Get second position
    if (!TypeParser::toGpsPosition(p3, p4, _state.nLand.tempPosition))
    {
        Log.errorPrintf("preLndCompute_2");
        fpCmdCompleted();
        return;
    }

    // Get position of the circle
    if (!TypeParser::toGpsPosition(p5, p6, _state.nLand.circleLowerAltitude))
    {
        Log.errorPrintf("preLndCompute_3");
        fpCmdCompleted();
        return;
    }

    // reset flags associated with locking breaking instruction and manual turn mode. 
    _state.resetMode();

    // Set state to preland mode
    _state.nLandPhaseFlag = _state.PRE_LAND;

    // Delete all set conditions.
    _state.resetConditions ();

    // Get distance to circle
    _state.nLand.distToCircle = GpsPosition::distance(_state.nLand.net.position, _state.nLand.tempPosition);

    // Calculate track
    _state.nLand.trackToFollow = GpsPosition::track(_state.nLand.tempPosition, _state.nLand.net.position);

    // Set time-out
    _state.exitCondition.setInterval(0.5f);

    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (PRELANDING_P2);

    // Send notify to other subsystems about FPReal settings had been changed.
   notify (FPR_CHANGED);
}

/**
* Pre land phase 2. 
*/
void FlightPlanRealizer::netPrelandPhase2(void)
{
    // Checking if system is ready for landing
    if (!SysMon->isReadyForRun (true))
    {
        Log.msgPrintf ("%s  -> System not ready for land - waiting for operator action", SUBS_PREFIX);

        // Notify is not beeing send. Flight plan subsystem waits in this instruction waits forever or until another data reading from fplan subsytem.
        return;
    }

    // Goto circle to lower altitude
    circlePhase1 (&_state.nLand.circleLowerAltitude, _state.nLand.circleRadius, _state.nLand.circleLeft, _conf.crcConstraint.normTime, _conf.dfAltitude);

    // Settings flags causing simultaneous ref.altitude & ref.airspeed changings after desired values modification (alg.dfAltitude i alg.dfAirspeed)
    _state.useUafSetDfltAlt = true;
    _state.useUafSetDfltAirSpd = true;

    // Land prepare done!
    _state.nLand.landPrepared = true;

    // Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
}

/**
* Approaching
*/
void FlightPlanRealizer::netReadyApproach (void)
{
	// reset flags associated with locking breaking instruction and manual turn mode. 
    //_state.resetMode();

	// Set state to land state
    _state.nLandPhaseFlag = _state.APPROACH;
    
	_state.fprd.fRef.altitude = _conf.semiLandConstraint.approachAlt;
    
    setLatControllers (LAT_VZ_ALT_ID);
    setSpdControllers (SPD_THR_SPD_ID, SPD_FLP_SPD_ID);

    // Delete all set conditions.
    _state.resetConditions ();


	_state.exitCondition.setTrackRange(_state.nLand.trackToFollow - _conf.semiLandConstraint.trackRange, _state.nLand.trackToFollow + _conf.semiLandConstraint.trackRange);
	_state.exitCondition.setFlyFinish(_state.nLand.circleLowerAltitude, _state.nLand.tempPosition,_conf.flyConstraint.finishError);
	_state.exitCondition.setAglRange(_conf.semiLandConstraint.approachAlt, _conf.semiLandConstraint.approachDeltaAlt, true);
    _state.exitCondition.setChainFunction (APPROACH);
    // Send notify to other subsystems about FPReal settings had been changed.
    notify (FPR_CHANGED);
    
    // Debug message
   Log.msgPrintf ("%s  -> Start circle to %fm at ready approach phase", SUBS_PREFIX, _state.fprd.fRef.altitude);
}


void FlightPlanRealizer::netApproach (void)
{
    // Set state to land state
    _state.nLandPhaseFlag = _state.APPROACH;
    
    // Assign next waypoint is recovery net
    _state.fprd.fRef.wptTo    = _state.nLand.net.position;
    _state.fprd.fRef.wptFrom  = _state.nLand.tempPosition;
	_state.fprd.fRef.track    = _state.nLand.trackToFollow;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue Approaching land");
    }

	setLonControllers (LON_TRACK_PATH_ID, LON_R_COORDEXP_ID);

    _vSem.unlock ();

    // Delete all conditions. 
    _state.resetConditions ();

    _state.exitCondition.setTrackRange(_state.nLand.trackToFollow - 0.5f * _conf.semiLandConstraint.trackRange, _state.nLand.trackToFollow + 0.5f * _conf.semiLandConstraint.trackRange);
	_state.exitCondition.setPhiOrP (0.1f, 0.01f);
    _state.exitCondition.setChainFunction (DESCENT);

   // Send notify to other subsystems about FPReal settings had been changed.
   notify (FPR_CHANGED);

   // Debug message
   Log.msgPrintf ("%s  -> Start approaching", SUBS_PREFIX);
}

/**
* Descenting
*/
void FlightPlanRealizer::netDescent(void)
{
	// Set state to land state
    _state.nLandPhaseFlag = _state.DESCENT;

	// Set reference
	_state.fprd.fRef.altitude = _conf.semiLandConstraint.descentAlt;
	_state.fprd.fRef.airspeed = _conf.semiLandConstraint.airspeed;

    if (!_vSem.lock ())
    {
        Log.msgPrintf ("%s  -> System error (lock) [%s]", SUBS_PREFIX, "continue Descenting land");
    }
    
    // Max Flap
	if (!_conf.semiLandConstraint.bUseFlap)
	{
		_state.fprd.outCtrl.fCtrl.flaps = _conf.semiLandConstraint.Flp.maxVal;
		setFlpControllers(SPD_UNUSED_FLP_ID);
	}

    _vSem.unlock ();

    // Set exit conditions. 
    _state.resetConditions ();
	_state.exitCondition.setAglRange(_conf.semiLandConstraint.descentAlt, _conf.semiLandConstraint.descentDeltaAlt, true);
    _state.exitCondition.setChainFunction(NET_LAND);

   // Send notify to other subsystems about FPReal settings had been changed.
   notify (FPR_CHANGED);

   // Debug message
   Log.msgPrintf ("%s  -> Start descenting to %fm", SUBS_PREFIX, _state.fprd.fRef.altitude);
}

/**
* Landing. 
*/
void FlightPlanRealizer::netLand(void)
{
    // Set state to land state
    _state.nLandPhaseFlag = _state.LAND;

	_flightConstraint = &_conf.semiLandConstraint;

	setLatControllers(LAT_VZ_ALT_ID);
	setLonControllers(LON_TRACK_PATH_ID, LON_R_COORDEXP_ID);
	setSpdControllers(SPD_THR_SPD_ID, SPD_FLP_SPD_ID);

    // Delete all conditions. 
    _state.resetConditions ();

    // Set accX trigger
	_state.exitCondition.setAccX(_conf.semiLandConstraint.accX, true);

    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (POST_LAND);

   // Send notify to other subsystems about FPReal settings had been changed.
   notify (FPR_CHANGED);

   // Debug message
   Log.msgPrintf ("%s  -> Land to recovery net at agl:%.1fm lon:%.6f lat:%.6f heading: %.1f deg", SUBS_PREFIX,
	   _psd.altitude, _state.nLand.net.position.getLon(), _state.nLand.net.position.getLat(), _psd.psi);
}

void FlightPlanRealizer::netPostLand (void)
{
	// Set state to land state
	_state.nLandPhaseFlag = _state.POST_LAND;

    // Delete all conditions. 
    _state.resetConditions ();

	// Disable all controllers
	_state.fprd.ctrlProps.disableControllers();

	// Reset all ouputs
	_state.fprd.outCtrl.fCtrl.reset();

	notify (FPR_CHANGED);

   // Debug message
   Log.msgPrintf ("%s  -> Land to recovery net successfull", SUBS_PREFIX);
}

/** 
* Auxiliary function when aborting landing.
*/
void FlightPlanRealizer::netGoAround(void)
{
    // Reset flags
    _state.nLand.landPrepared = false;
    _state.nLandPhaseFlag = _state.ABORT_LAND;

	circlePhase1 (&_state.nLand.net.position, _conf.semiLandConstraint.abortRadius, _conf.crcConstraint.left, _conf.crcConstraint.baseTime,
		_conf.semiLandConstraint.abortAlt);

    Log.msgPrintf ("%sEnter goAround Mode target theta:%.1f", SUBS_PREFIX, _state.fprd.fRef.fRefLowLevel.theta * RAD_2_DEG);

    notify (FPR_CHANGED);

    return;
}

/** 
* Auxiliary function when aborting landing.
*/
void FlightPlanRealizer::netAbort(void)
{
    _state.fprd.fRef.altitude = _conf.semiLandConstraint.abortAlt + _state.nLand.net.offSet;

    // Reset flags
    _state.nLand.landPrepared = false;
    _state.nLandPhaseFlag = _state.ABORT_LAND;

    if (!_vSem.lock ())
        return;
    
    // // Maximum throttle
	_state.fprd.outCtrl.fCtrl.throttle = _conf.flyConstraint.Thr.maxVal;
	setSpdControllers (SPD_UNUSED_THR_ID, SPD_FLP_SPD_ID);
    
    // Hold wings level
    _state.fprd.fRef.fRefLowLevel.phi = 0.0f;
	setLonControllers (LON_HOLD_PHI_ID);

    // Hold pitch
	setLatControllers (LAT_VZ_ALT_ID);
    
    if (!_vSem.unlock ())
        return;

    // Delete all conditions. 
    _state.resetConditions ();

    // Set exit conditons
    _state.exitCondition.setAgl(_conf.semiLandConstraint.abortAlt, true);

    //  Set function that will be executed after fulfilment of the condition.
    _state.exitCondition.setChainFunction (GOAROUND_NET);

   // Send notify to other subsystems about FPReal settings had been changed.
   notify (FPR_CHANGED);

    Log.msgPrintf ("%sEnter goAround Mode target theta:%.1f", SUBS_PREFIX, _state.fprd.fRef.fRefLowLevel.theta * RAD_2_DEG);

    return;
}

/**
* Check if in land mode. 
*/
bool FlightPlanRealizer::inLandMode(void) const
{
	return ((_state.nLandPhaseFlag == _state.PRE_APPROACH)  ||
            (_state.nLandPhaseFlag == _state.APPROACH)      ||
		    (_state.nLandPhaseFlag == _state.DESCENT)       ||
			(_state.nLandPhaseFlag == _state.LAND)          ||
			(_state.nLandPhaseFlag == _state.POST_LAND)     ||
			(_state.nLandPhaseFlag == _state.ABORT_LAND));
}

/**
* Check if in descent + land mode. 
*/
bool FlightPlanRealizer::inAfterPreApproachPhase(void) const
{
	return ((_state.nLandPhaseFlag == _state.APPROACH) ||
            (_state.nLandPhaseFlag == _state.DESCENT)  ||
			(_state.nLandPhaseFlag == _state.LAND)     ||
            (_state.nLandPhaseFlag == _state.POST_LAND));
}

/**
* Check if in land mode. 
*/
bool FlightPlanRealizer::inLandPhase(void) const
{
    return (_state.nLandPhaseFlag == _state.LAND);
}

/**
* Check if in post land mode. 
*/
bool FlightPlanRealizer::inPostLandPhase(void) const
{
    return (_state.nLandPhaseFlag == _state.POST_LAND);
}

bool FlightPlanRealizer::inPreLandPhase(void) const
{
	return (_state.landParachutePhaseFlag == _state.LND_PRELAND);
}

bool FlightPlanRealizer::inParachutePhase(void) const
{
	return (_state.landParachutePhaseFlag == _state.LND_PARACHUTED);
}

bool FlightPlanRealizer::inTkfCatapultPhase(void) const
{
	return ((_state.tkfPhaseFlag == _state.TKF_P1) ||
		(_state.tkfPhaseFlag == _state.TKF_P2) ||
		(_state.tkfPhaseFlag == _state.TKF_P3));
}


//***************************************************************************************
//  Nested classes
//***************************************************************************************

//=========================================================================
//  ConfigData Class
//=========================================================================

/**  
* Class constructor with configuration data
*/
FlightPlanRealizer::ConfigData::ConfigData(void)
{
    setDefault ();
}

FlightPlanRealizer::FlyConstraint::FlyConstraint(void)
{
	SetDefault();
}

FlightPlanRealizer::CrcConstraint::CrcConstraint(void)
{
	SetDefault();
}

FlightPlanRealizer::SemiLandConstraint::SemiLandConstraint(void)
{
	SetDefault();
}

/**
* Function sets defaults configuration data.
*/
void FlightPlanRealizer::ConfigData::setDefault(void)
{
    // Defeault reference values.
    dfAirspeed         =  70.0f;
    dfAltitude         = 100.0f;
	dfAltitudeDelta    =  10.0f;
    dfCircleMode       =      1;
    dfCircleModePar    =  15.0f;
    dfAutoGndWindCoeff =   1.0f;
    //  Default "OnHold" mode parameters.
    holdTime           = 600.0f;
    holdRadius         = 100.0f;
    holdLeft           =   true;
    //  "Manual Turn" mode parameters
    manTimeout         = 30.0f;
    manPhiCoeff        =  1.0f;
	//  "Link Broken" mode parameters
	linkBrokenMaxAlt   = 500.0f;
	linkBrokenAltZone  =  20.0f;
	linkBrokenRadius   = 200.0f;
    //  "Camera Guide" mode parameters
    cgPhiCoeff         =  0.5f;
    cgPanOffset        =  0.0f;
    cgUseCamTlm        =  false;
    
	// flight constraint params
    flyConstraint.SetDefault();
    crcConstraint.SetDefault();
	semiLandConstraint.SetDefault();

    //  "Force Land" mode parameters
    fLndThrOffTime         =   1.0f;
    fLndTheta              =  -0.175f;    // -0.175 rad = 10 degres down
    fLndMaxAbsPhi          =   0.3f;
    fLndBtfly              =   1.0f;
    fLndLandedAccX         =   0.6f;
    
    //  "Emergerncy Land" (after alitude drops) mode parameters
    emrgLandEnable         =    true;
    emrgLandAgl            =   50.0f;
    emrgLandMrg            =   10.0f;
#if EMRG_LOSS_ENGINE == 1
	emrgMinPhiDeg   =  -10.0f;
	emrgMaxPhiDeg   =   10.0f;
	emrgMinThetaDeg =  -20.0f;
	emrgMaxThetaDeg =   20.0f;
	emrgAirspeedRef =   75.0f;
	emrgAltitude    =    5.0f;
	emrgRadius      =  400.0f;
	emrgThetaDeg    =   -5.0f;
#endif

    //  "Stall Recovery" mode parameters
    stallRecEnable         =   true;
    stallRecElevator       =   0.0f;
    stallRecAirspeedOn     =  50.0f;
    stallRecPOn            = 0.872f;  // 50 degree/s
    stallRecPhiOn          = 0.523f;  // 30 degree
    stallRecTimeOff        =   3.0f;
    stallRecAirspeedOff    =  55.0f;

    // "Auto takeoff" catapult mode parameters
    autoTkfCatapultEnable        =  true;
    autoTkfCatapultPitch         = 12.0f;
    autoTkfCatapultTimeout       =  0.5f;
    autoTkfCatapultThetaMaxErr   =  0.0f;
	autoTkfCatapultClimbAlt      = 70.0f;
    autoTkfCatapultAirspeedRange = 45.0f;

	// Parachute Landing mode parameters
	autoParaRadius				= 250.0f;					
	autoParaAltitude			= 150.0f;			
	autoParaAirspeed			= 80.0f;				
	autoParaStableVelZ			= 6.0f;				
	autoParaDelTrack			= 3.0f;
	autoParaBaseTime			= 90.0f;
	autoParaApproachTime		= 30.0f;

    //  Parameters related with service the pressure indicator malfunction.
    pressFaultTheta        =  -0.062f;  // -3.53 degree
    pressFaultIas          =    70.0f;

    //  Parameters related with switching Track-Psi during decreasing the speed
    gSpeedMinValue         =  10.0f;
	// time interval DeltaAirSpeed for takeoff 
	timeIntervalAspeed	   = 0.1f;		//second

    //  Other parameteres
    configAutosave         = false;
    mainDivisor            =     2;   // divisor of the notifications from the PhysicalState
    commDivisor            =    60;   // divisir of the mainDivisor
    logDivisor             =    60;
	bUseCorrTrack          = false;
}

void FlightPlanRealizer::FlyConstraint::SetDefault ()
{
    //  "Fly" 
    Phi.minVal          =   -0.8f;     Phi.maxVal       =    0.8f;
    P.minVal            =   -3.0f;     P.maxVal         =    3.0f;
    Alr.minVal          =   -1.0f;     Alr.maxVal       =    1.0f;
    rudderYacc          =   false;
    Rdr.minVal          =   -1.0f;     Rdr.maxVal       =    1.0f;
    R.minVal            =   -3.0f;     R.maxVal         =    3.0f;
    ThetaSpd.minVal     =   -0.3f;     ThetaSpd.maxVal  =    0.3f;
    ThetaAlt.minVal     =   -0.2f;     ThetaAlt.maxVal  =    0.2f;
    Q.minVal            =   -3.0f;     Q.maxVal         =    3.0f;
    Elv.minVal          =   -1.0f;     Elv.maxVal       =    1.0f;
    Thr.minVal          =   0.07f;     Thr.maxVal       =    1.0f;
    mrgLowThr           =  -10.0f;     mrgHighThr       =   20.0f;
    minBtfly            =    0.0f;     maxBtfly         =    0.8f;
    mrgLowBtfly         =   10.0f;     mrgHighBtfly     =   50.0f;
    finishError         =   50.0f;
    Flp.minVal          =    0.0f;     Flp.maxVal       =    0.4f;
    VertSpd.minVal      =   -5.0f;     VertSpd.maxVal   =    5.0f;
    TrackCorr.minVal    =  -30.0f;     TrackCorr.maxVal =   30.0f;
}

void FlightPlanRealizer::CrcConstraint::SetDefault ()
{
    //  "Circle" mode parameters
    radius              =   100.0f;
	radiusErr           =     0.1f;
    left                =     true;
    Phi.minVal          =    -0.8f;      Phi.maxVal       =    0.8f;
    P.minVal            =    -3.0f;      P.maxVal         =    3.0f;
    Alr.minVal          =    -1.0f;      Alr.maxVal       =    1.0f;
    rudderYacc          =    false;
    Rdr.minVal          =    -1.0f;      Rdr.maxVal       =    1.0f;
    R.minVal            =    -3.0f;      R.maxVal         =    3.0f;
    airspeed            =    70.0f;
    ThetaSpd.minVal     =    -0.3f;      ThetaSpd.maxVal  =    0.3f;
    ThetaAlt.minVal     =    -0.2f;      ThetaAlt.maxVal  =    0.2f;
    Q.minVal            =    -3.0f;      Q.maxVal         =    3.0f;
    Elv.minVal          =    -1.0f;      Elv.maxVal       =    1.0f;
    Thr.minVal          =     0.0f;      Thr.maxVal       =    1.0f;
    mrgLowThr           =   -10.0f;      mrgHighThr       =   20.0f;
    minBtfly            =     0.0f;      maxBtfly         =    0.8f;
    mrgLowBtfly         =    10.0f;      mrgHighBtfly     =   50.0f;
	baseAltitude        =   100.0f;
    baseTime            = 10800.0f;      normTime         = 10800.0f;
    Flp.minVal          =     0.0f;      Flp.maxVal       =    0.4f;
    VertSpd.minVal      =    -5.0f;      VertSpd.maxVal   =    5.0f;
    TrackCorr.minVal    =   -30.0f;      TrackCorr.maxVal =   30.0f;
}

void FlightPlanRealizer::SemiLandConstraint::SetDefault ()
{
    //  "Fly" 
    Phi.minVal          =   -0.8f;     Phi.maxVal       =    0.8f;
    P.minVal            =   -3.0f;     P.maxVal         =    3.0f;
    Alr.minVal          =   -1.0f;     Alr.maxVal       =    1.0f;
    rudderYacc          =   false;
    Rdr.minVal          =   -1.0f;     Rdr.maxVal       =    1.0f;
    R.minVal            =   -3.0f;     R.maxVal         =    3.0f;
    ThetaSpd.minVal     =   -0.3f;     ThetaSpd.maxVal  =    0.3f;
    ThetaAlt.minVal     =   -0.2f;     ThetaAlt.maxVal  =    0.2f;
    Q.minVal            =   -3.0f;     Q.maxVal         =    3.0f;
    Elv.minVal          =   -1.0f;     Elv.maxVal       =    1.0f;
    Thr.minVal          =   0.03f;     Thr.maxVal       =    1.0f;
    mrgLowThr           =  -10.0f;     mrgHighThr       =   20.0f;
    minBtfly            =    0.0f;     maxBtfly         =    0.8f;
    mrgLowBtfly         =   10.0f;     mrgHighBtfly     =   50.0f;
    Flp.minVal          =    0.0f;     Flp.maxVal       =    0.4f;
    VertSpd.minVal      =   -1.0f;     VertSpd.maxVal   =    1.0f;
    TrackCorr.minVal    =  -30.0f;     TrackCorr.maxVal =   30.0f;
	airspeed        =   75.0f;
	approachAlt     =   70.0f;     approachDeltaAlt =   5.0f;
	trackRange      =    5.0f;
	bUseFlap        =   false;
	descentAlt      =   50.0f;     descentDeltaAlt  =   5.0f;
	accX            =    2.0f;
	abortAlt        =   70.0f;     abortRadius      = 200.0f;
}

/**
* Function delete flags related with locking posibility of breaking instructions and "manual turn" mode.
*/
void FlightPlanRealizer::StateData::resetMode(void)
{
    // Reset flag that lock interrrupt of the instruction.
    setDoNotBreakFlag (false);
    doNotBreakByRetMode = false;
    //  Lock "manual" and "camera guide" modes.
    doNotManualTurn = false;
    doNotCameraGuide = false;
    //  Turn off flag forbidding passing to the next flight plan instruction in "manual turn" mode.
    doNotFinishPhaseInMT = false;
    doNotFinishPhaseInCG = false;
    //  Turn off flag causing immediate passing to the next flight plan instruction after turning on  "manual turn" & "camera guide" mode.
    skipCommandInMT = false;
    skipCommandInCG = false;
    //  Turn off flags causing action after parameter change. (Parameters with flag UAF_*).
    useUafRedoCircle = false;
    useUafSetDfltAlt = false;
    useUafSetDfltAirSpd = false;
    useUafSetCrcAirSpd = false;
    // Turn on flag to use thetaModifier
    if (!useThetaMod)
    {
        useThetaMod = true;
        Log.msgPrintf ("%sReuse ThetaModifier", SUBS_PREFIX);
    }  

    if (manualTurnMode)
    {
        //  Turn off flag - manual turn mode.
        manualTurnMode = false;
        Log.eventPrint (EVT_FPR_MANUAL, "0");
        Log.msgPrintf ("%sManual Turn Mode: auto off", SUBS_PREFIX);
    }

    if (cameraGuideMode)
    {
        //  Turn off flag - camera guide mode.
        cameraGuideMode = false;
        Log.eventPrint (EVT_FPR_CG, "0");
        Log.msgPrintf ("%sCamera Guide Mode: auto off", SUBS_PREFIX);
    }

    if (stall)
    {
        //  Turn off flag executing stall recovery mode instructions.
        stall = false;
        Log.msgPrintf ("%sStall recovery cancelled", SUBS_PREFIX);
    }

    if (lowGSpeed)
    {
        //  turn off flag handling low ground speed (Phi_Psi controller).
        lowGSpeed = false;
        Log.msgPrintf ("%sLow Ground Speed (Phi_Psi): auto off", SUBS_PREFIX);
    }

    // Reset elevation offset being set for the landing at other level.
    PState->setAglOffset (0.0f);
    // Include "elevationOffset" in the Physical State (could be turned off earlier).
    // PState->useElevationOffsetPar (true);
}


/**
* Function delete flight phases end conditions and asynchronous triggers.
*/
void FlightPlanRealizer::StateData::resetConditions(void)
{
    //  Delete all set conditions
    exitCondition.reset ();
    exitConditionMT.reset ();
    stallCondition.reset ();
    lowGSpeedCondition.reset ();
    emrgLandingCondition.reset ();
    //  Turn off triggers 
    launchTrigger.reset();
}


/** 
* Change variable doNotBreak and send event.
*/
void FlightPlanRealizer::StateData::setDoNotBreakFlag (bool newValue)
{
    if (doNotBreak != newValue)
    {
        doNotBreak = newValue;
        Log.eventPrint (EVT_FPR_NOBREAK, newValue ? "1" : "0");
    }
}

bool FlightPlanRealizer::setControlPropertiseParams (int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_pars->insert("Alr_P.",              &_state.fprd.ctrlProps.AlrCProp.Alr_P);            
    numTemp++;/* 2*/_pars->insert("Elv_Q.",              &_state.fprd.ctrlProps.ElvCProp.Elv_Q);
    numTemp++;/* 3*/_pars->insert("Rdr_R.",              &_state.fprd.ctrlProps.RudderCProp.Rdr_R);
    numTemp++;/* 4*/_pars->insert("Rdr_Yacc.",           &_state.fprd.ctrlProps.RudderCProp.Rdr_Yacc);
    numTemp++;/* 5*/_pars->insert("Thr_Speed.",          &_state.fprd.ctrlProps.ThrCProp.Thr_Speed);
    numTemp++;/* 6*/_pars->insert("Thr_Alt.",            &_state.fprd.ctrlProps.ThrCProp.Thr_Alt);
    numTemp++;/* 7*/_pars->insert("Thr_Alt_2State.",     &_state.fprd.ctrlProps.Thr_Alt_2State);
    numTemp++;/* 8*/_pars->insert("Btfly_Alt_2State.",   &_state.fprd.ctrlProps.Btfly_Alt_2State);
    numTemp++;/* 9*/_pars->insert("Flp_Speed.",          &_state.fprd.ctrlProps.FlpCProp.Flp_Speed);
    numTemp++;/*10*/_pars->insert("Abr_GPErr.",          &_state.fprd.ctrlProps.Abr_GPErr);
    numTemp++;/*11*/_pars->insert("FAlr_Alr.",           &_state.fprd.ctrlProps.FAlr_Alr);
    numTemp++;/*12*/_pars->insert("P_Phi.",              &_state.fprd.ctrlProps.PCProp.P_Phi);
    numTemp++;/*13*/_pars->insert("Q_Theta.",            &_state.fprd.ctrlProps.QCProp.Q_Theta);
    numTemp++;/*14*/_pars->insert("R_Psi.",              &_state.fprd.ctrlProps.RCProp.R_Psi);
    numTemp++;/*15*/_pars->insert("R_CoordExp.",         &_state.fprd.ctrlProps.RCProp.R_CoordExp);
    numTemp++;/*16*/_pars->insert("R_Track.",            &_state.fprd.ctrlProps.RCProp.R_Track);
    numTemp++;/*17*/_pars->insert("Theta_Alt.",          &_state.fprd.ctrlProps.ThetaCProp.Theta_Alt);
    numTemp++;/*18*/_pars->insert("Theta_Speed.",        &_state.fprd.ctrlProps.ThetaCProp.Theta_Speed);
    numTemp++;/*19*/_pars->insert("Phi_Track.",          &_state.fprd.ctrlProps.PhiCProp.Phi_Track);
    numTemp++;/*20*/_pars->insert("Phi_CTrack.",         &_state.fprd.ctrlProps.PhiCProp.Phi_CTrack);
    numTemp++;/*21*/_pars->insert("Phi_Psi.",            &_state.fprd.ctrlProps.PhiCProp.Phi_Psi);
    numTemp++;/*22*/_pars->insert("Track_TrackCorr.",    &_state.fprd.ctrlProps.TrackCProp.Track_TrackCorr);
    numTemp++;/*23*/_pars->insert("Track_Wpt.",          &_state.fprd.ctrlProps.TrackCProp.Track_Wpt);
    numTemp++;/*24*/_pars->insert("TrackCorr_Cte.",      &_state.fprd.ctrlProps.TrackCorrCProp.TrackCorr_Cte);
    numTemp++;/*25*/_pars->insert("Btfly_GPath_2State.", &_state.fprd.ctrlProps.Btfly_GPath_2State);
    numTemp++;/*26*/_pars->insert("Theta_GPath_2State.", &_state.fprd.ctrlProps.Theta_GPath_2State);
    numTemp++;/*27*/_pars->insert("Theta_VertSpeed.",    &_state.fprd.ctrlProps.ThetaCProp.Theta_VertSpeed);
    numTemp++;/*28*/_pars->insert("VertSpeed_Alt.",      &_state.fprd.ctrlProps.VertSpeedCProp.VertSpeed_Alt);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for control propertise params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setFlightControlParams (int numIndex)
{
	int numTemp = 0;
	
    numTemp++;/* 1*/_pars->insert("fc.ailerons",        &_state.fprd.outCtrl.fCtrl.ailerons, -1.0f, 1.0f);
    numTemp++;/* 2*/_pars->insert("fc.elevator",        &_state.fprd.outCtrl.fCtrl.elevator, -1.0f, 1.0f);
    numTemp++;/* 3*/_pars->insert("fc.rudder",          &_state.fprd.outCtrl.fCtrl.rudder, -1.0f, 1.0f);
    numTemp++;/* 4*/_pars->insert("fc.throttle",        &_state.fprd.outCtrl.fCtrl.throttle, 0.0f, 1.0f);
    numTemp++;/* 5*/_pars->insert("fc.flaps",           &_state.fprd.outCtrl.fCtrl.flaps, -1.0f, 1.0f);
    numTemp++;/* 6*/_pars->insert("fc.airbrakes",       &_state.fprd.outCtrl.fCtrl.airbrakes, -1.0f, 1.0f);
    numTemp++;/* 7*/_pars->insert("fc.butterfly",       &_state.fprd.outCtrl.fCtrl.butterfly, 0.0f, 1.0f);
    numTemp++;/* 8*/_pars->insert("fc.flapsAsAilerons", &_state.fprd.outCtrl.fCtrl.flapsAsAilerons, -1.0f, 1.0f);
    numTemp++;/* 9*/_pars->insert("fc.parachute",         &_state.fprd.outCtrl.fCtrl.parachute, 0.0f, 1.0f);
    numTemp++;/*10*/_pars->insert("fc.custom[0]",       &_state.fprd.outCtrl.fCtrl.customOutput[0], -1.0f, 1.0f);
    numTemp++;/*11*/_pars->insert("fc.custom[1]",       &_state.fprd.outCtrl.fCtrl.customOutput[1], -1.0f, 1.0f);
    numTemp++;/*12*/_pars->insert("fc.custom[2]",       &_state.fprd.outCtrl.fCtrl.customOutput[2], -1.0f, 1.0f);
    numTemp++;/*13*/_pars->insert("fc.custom[3]",       &_state.fprd.outCtrl.fCtrl.customOutput[3], -1.0f, 1.0f);

	if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for flight control params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setFlightReferenceParams (int numIndex)
{
    int numTemp = 0;
	
    numTemp++;/* 1*/_pars->insert("ref.P",              &_state.fprd.fRef.fRefLowLevel.P, -10.0f, 10.0f);
    numTemp++;/* 2*/_pars->insert("ref.Q",              &_state.fprd.fRef.fRefLowLevel.Q, -10.0f, 10.0f);
    numTemp++;/* 3*/_pars->insert("ref.R",              &_state.fprd.fRef.fRefLowLevel.R, -10.0f, 10.0f);
    numTemp++;/* 4*/_pars->insert("ref.theta",          &_state.fprd.fRef.fRefLowLevel.theta, -10.0f, 10.0f);
    numTemp++;/* 5*/_pars->insert("ref.phi",            &_state.fprd.fRef.fRefLowLevel.phi, -10.0f, 10.0f);
    numTemp++;/* 6*/_pars->insert("ref.psi",            &_state.fprd.fRef.fRefLowLevel.psi, -10.0f, 10.0f);
    numTemp++;/* 7*/_pars->insert("ref.airspeed",       &_state.fprd.fRef.airspeed, 54.0f, 201.0f, 1);
    numTemp++;/* 8*/_pars->insert("ref.altitude",       &_state.fprd.fRef.altitude, -5000.0f, 10000.0f, 2);
    numTemp++;/* 9*/_pars->insert("ref.track",          &_state.fprd.fRef.track, 0.0f, 360.0f, 1);
    numTemp++;/*10*/_pars->insert("ref.trackFromTo",    &_state.fprd.fRef.trackFromTo, 0.0f, 360.0f, 1);
    numTemp++;/*11*/_pars->insert("ref.crossTrackCorr", &_state.fprd.fRef.crossTrackCorr, -360.0f, 360.0f, 1);
    numTemp++;/*12*/_pars->insert("ref.circleRadius",   &_state.fprd.fRef.circleRadius, 0.0f, 10000.0f, 1);
    numTemp++;/*13*/_pars->insert("ref.circleLeft",     &_state.fprd.fRef.circleLeft);
    numTemp++;/*14*/_pars->insert("ref.circleMode",     &_state.fprd.fRef.circleMode, 0, 1);
    numTemp++;/*15*/_pars->insert("ref.circleModePar",  &_state.fprd.fRef.circleModePar, 0.0f, 1000.0f, 2);
    numTemp++;/*16*/_pars->insert("ref.glidePath",      &_state.fprd.fRef.glidePath, 0.0f, 100.0f, 2);
	numTemp++;/*17*/_pars->insert("ref.vertSpeed",      &_state.fprd.fRef.fRefLowLevel.f32VertSpeed, -100.0f, 100.0f, 2);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for flight reference params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setGpsPosition (int numIndex)
{
    int numTemp = 0;
    numTemp++;/* 1*/_pars->insert("ref.wptFrom.lat",    &_state.fprd.fRef.wptFrom, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0);
    numTemp++;/* 2*/_pars->insert("ref.wptFrom.lon",    &_state.fprd.fRef.wptFrom, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0);
    numTemp++;/* 3*/_pars->insert("ref.wptTo.lat",      &_state.fprd.fRef.wptTo, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0);
    numTemp++;/* 4*/_pars->insert("ref.wptTo.lon",      &_state.fprd.fRef.wptTo, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0);
    // Origin propertises
    numTemp++;/* 5*/_pars->insert("origin.track",        &_state.origin.takeoffTrack, 0.0f, 360.0f, 1);
    numTemp++;/* 6*/_pars->insert("origin.position.lat", &_state.origin.position, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0);
    numTemp++;/* 7*/_pars->insert("origin.position.lon", &_state.origin.position, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0);
    // BaseStation
    numTemp++;/* 8*/_pars->insert("base.position.lat",   &_state.base.position, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0, 6, false);
    numTemp++;/* 9*/_pars->insert("base.position.lon",   &_state.base.position, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0, 6, false);
    numTemp++;/*10*/_pars->insert("base.elvOffset",      &_state.base.elvOffset, -5000.0f, 10000.0f, 1);
	// Parachuting Position
	numTemp++;/* 11*/_pars->insert("parachute.position.lat", &_state.para.positionParachuting, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0, 6, false);
	numTemp++;/* 12*/_pars->insert("parachute.position.lon", &_state.para.positionParachuting, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0, 6, false);
	numTemp++;/* 13*/_pars->insert("circleParachute.position.lat", &_state.para.positionCircleCenter, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0, 6, false);
	numTemp++;/* 14*/_pars->insert("circleParachute.position.lon", &_state.para.positionCircleCenter, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0, 6, false);

	// BaseStation
	numTemp++;/* 15*/_pars->insert("camguide.position.lat", &_state.camguide.positionCamguide, &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0, 6, false);
	numTemp++;/* 16*/_pars->insert("camguide.position.lon", &_state.camguide.positionCamguide, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0, 6, false);
	numTemp++;/* 17*/_pars->insert("camguide.circleCamguideTime", &_state.camguide.circleCamguideTime, 0.0f, 100000.0f);


    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for gps position", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setDefaultParams (int numIndex)
{
    int numTemp = 0;
    numTemp++;/*  1*/_alg->insert("dfAirspeed",         &_conf.dfAirspeed,            54.0f,   181.0f, 1, false, UAF_SET_DFLT_AIRSPD);
    numTemp++;/*  2*/_alg->insert("dfAltitude",         &_conf.dfAltitude,         -5000.0f, 10000.0f, 1, false, UAF_SET_DFLT_ALT);
	numTemp++;/*  3*/_alg->insert("dfAltitudeDelta",    &_conf.dfAltitudeDelta,        0.0f,   100.0f);
    numTemp++;/*  4*/_alg->insert("dfCircleMode",       &_conf.dfCircleMode,          0, 1);
    numTemp++;/*  5*/_alg->insert("dfCircleModePar",    &_conf.dfCircleModePar,        0.0f,  1000.0f, 2);
    numTemp++;/*  6*/_alg->insert("dfAutoGndWindCoeff", &_conf.dfAutoGndWindCoeff,     0.0f,     1.0f, 3);
    numTemp++;/*  7*/_alg->insert("gSpeedMinValue",     &_conf.gSpeedMinValue,         0.0f,   100.0f, 1);
	numTemp++;/*  8*/_alg->insert("timeIntervalAspeed", &_conf.timeIntervalAspeed,     0.0f,     1.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for gps position", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setHoldModeParams (int numIndex)
{
    int numTemp = 0;
    numTemp++;/* 1*/_alg->insert("holdTime",   &_conf.holdTime,    0.0f, 10000.0f, 1);
    numTemp++;/* 2*/_alg->insert("holdRadius", &_conf.holdRadius, 10.0f, 10000.0f, 1);
    numTemp++;/* 3*/_alg->insert("holdLeft",   &_conf.holdLeft);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for hold mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setManualTurnModeParams (int numIndex)
{
    int numTemp = 0;
    numTemp++;/* 1*/_alg->insert("manTimeout",  &_conf.manTimeout,  0.0f, 10000.0f, 1);
    numTemp++;/* 2*/_alg->insert("manPhiCoeff", &_conf.manPhiCoeff, 0.0f,     2.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for manual turn mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setLinkBrokenParams (int numIndex)
{
    int numTemp = 0;
	numTemp++;/* 1*/_alg->insert("linkBrokenMaxAlt",  &_conf.linkBrokenMaxAlt,  0.0f, 10000.0f);
	numTemp++;/* 2*/_alg->insert("linkBrokenAltZone", &_conf.linkBrokenAltZone, 0.0f,  1000.0f);
	numTemp++;/* 3*/_alg->insert("linkBrokenRadius",  &_conf.linkBrokenRadius,  0.0f,  1000.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for link broken params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setCameraGuideModeParams (int numIndex)
{
    int numTemp = 0;
    numTemp++;/* 1*/_alg->insert("cgPhiCoeff",  &_conf.cgPhiCoeff,    -2.0f,   2.0f);
    numTemp++;/* 2*/_alg->insert("cgPanOffset", &_conf.cgPanOffset, -100.0f, 100.0f, 1);
    numTemp++;/* 3*/_alg->insert("cgUseCamTlm", &_conf.cgUseCamTlm);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for camera guide mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setNetParams (int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_pars->insert("nLand.netposition.lat", &_state.nLand.net.position,  &GpsPosition::setLat, &GpsPosition::getLat, -90.0, 90.0);
    numTemp++;/* 2*/_pars->insert("nLand.netposition.lon", &_state.nLand.net.position, &GpsPosition::setLon, &GpsPosition::getLon, -180.0, 180.0);
	numTemp++;/* 3*/_pars->insert("nLand.netheight",       &_state.nLand.net.height, 0.0f,  100.0f);
	numTemp++;/* 4*/_pars->insert("nLand.netwidth",        &_state.nLand.net.width,  0.0f,  100.0f);
	numTemp++;/* 5*/_pars->insert("nLand.netoffset",       &_state.nLand.net.offSet, 0.0f, 1000.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for land mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setFlyModeParams (int numIndex)
{
    int numTemp = 0;
	
    numTemp++;/* 1*/_alg->insert("flyMinPhi",         &_conf.flyConstraint.Phi.minVal,            -2.0f,      0.0f);
    numTemp++;/* 2*/_alg->insert("flyMaxPhi",         &_conf.flyConstraint.Phi.maxVal,             0.0f,      2.0f);
    numTemp++;/* 3*/_alg->insert("flyMinP",           &_conf.flyConstraint.P.minVal,             -10.0f,      0.0f);
    numTemp++;/* 4*/_alg->insert("flyMaxP",           &_conf.flyConstraint.P.maxVal,               0.0f,     10.0f);
    numTemp++;/* 5*/_alg->insert("flyMinAlr",         &_conf.flyConstraint.Alr.minVal,            -1.0f,      0.0f);
    numTemp++;/* 6*/_alg->insert("flyMaxAlr",         &_conf.flyConstraint.Alr.maxVal,             0.0f,      1.0f);
    numTemp++;/* 7*/_alg->insert("flyRudderYacc",     &_conf.flyConstraint.rudderYacc);
    numTemp++;/* 8*/_alg->insert("flyMinRdr",         &_conf.flyConstraint.Rdr.minVal,            -1.0f,      0.0f);
    numTemp++;/* 9*/_alg->insert("flyMaxRdr",         &_conf.flyConstraint.Rdr.maxVal,             0.0f,      1.0f);
    numTemp++;/*10*/_alg->insert("flyMinR",           &_conf.flyConstraint.R.minVal,             -10.0f,      0.0f);
    numTemp++;/*11*/_alg->insert("flyMaxR",           &_conf.flyConstraint.R.maxVal,               0.0f,     10.0f);
    numTemp++;/*12*/_alg->insert("flyMinThetaSpd",    &_conf.flyConstraint.ThetaSpd.minVal,       -2.0f,      0.0f);
    numTemp++;/*13*/_alg->insert("flyMaxThetaSpd",    &_conf.flyConstraint.ThetaSpd.maxVal,        0.0f,      2.0f);
    numTemp++;/*14*/_alg->insert("flyMinThetaAlt",    &_conf.flyConstraint.ThetaAlt.minVal,       -2.0f,      0.0f);
    numTemp++;/*15*/_alg->insert("flyMaxThetaAlt",    &_conf.flyConstraint.ThetaAlt.maxVal,        0.0f,      2.0f);
    numTemp++;/*16*/_alg->insert("flyMinQ",           &_conf.flyConstraint.Q.minVal,             -10.0f,      0.0f);
    numTemp++;/*17*/_alg->insert("flyMaxQ",           &_conf.flyConstraint.Q.maxVal,               0.0f,     10.0f);
    numTemp++;/*18*/_alg->insert("flyMinElv",         &_conf.flyConstraint.Elv.minVal,            -1.0f,      0.0f);
    numTemp++;/*19*/_alg->insert("flyMaxElv",         &_conf.flyConstraint.Elv.maxVal,             0.0f,      1.0f);
    numTemp++;/*20*/_alg->insert("flyMinThr",         &_conf.flyConstraint.Thr.minVal,             0.0f,      1.0f);
    numTemp++;/*21*/_alg->insert("flyMaxThr",         &_conf.flyConstraint.Thr.maxVal,             0.0f,      1.0f);
    numTemp++;/*22*/_alg->insert("flyMrgLowThr",      &_conf.flyConstraint.mrgLowThr,      -1000.0f,   1000.0f, 2);
    numTemp++;/*23*/_alg->insert("flyMrgHighThr",     &_conf.flyConstraint.mrgHighThr,     -1000.0f,   1000.0f, 2);
    numTemp++;/*24*/_alg->insert("flyMinBtfly",       &_conf.flyConstraint.minBtfly,           0.0f,      1.0f);
    numTemp++;/*25*/_alg->insert("flyMaxBtfly",       &_conf.flyConstraint.maxBtfly,           0.0f,      1.0f);
    numTemp++;/*26*/_alg->insert("flyMrgLowBtfly",    &_conf.flyConstraint.mrgLowBtfly,    -1000.0f,   1000.0f, 2);
    numTemp++;/*27*/_alg->insert("flyMrgHighBtfly",   &_conf.flyConstraint.mrgHighBtfly,   -1000.0f,   1000.0f, 2);
    numTemp++;/*28*/_alg->insert("flyFinishError",    &_conf.flyConstraint.finishError,        0.0f,  10000.0f, 2);
    numTemp++;/*29*/_alg->insert("flyMinFlp",         &_conf.flyConstraint.Flp.minVal,            -1.0f,      1.0f);
    numTemp++;/*30*/_alg->insert("flyMaxFlp",         &_conf.flyConstraint.Flp.maxVal,            -1.0f,      1.0f);
    numTemp++;/*31*/_alg->insert("flyMinVertSpd",     &_conf.flyConstraint.VertSpd.minVal,      -100.0f,      0.0f);
    numTemp++;/*32*/_alg->insert("flyMaxVertSpd",     &_conf.flyConstraint.VertSpd.maxVal,         0.0f,    100.0f);
    numTemp++;/*33*/_alg->insert("flyMinTrackCorr",   &_conf.flyConstraint.TrackCorr.minVal,    -100.0f,      0.0f);
    numTemp++;/*34*/_alg->insert("flyMaxTrackCorr",   &_conf.flyConstraint.TrackCorr.maxVal,       0.0f,    100.0f);    

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for fly mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setCrcModeParams(int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_alg->insert("crcRadius",         &_conf.crcConstraint.radius,             0.0f,  10000.0f, 1, false, UAF_REDO_CIRCLE);
	numTemp++;/* 2*/_alg->insert("crcLeft",           &_conf.crcConstraint.left,              false,  UAF_REDO_CIRCLE);
	numTemp++;/* 1*/_alg->insert("crcRadiusErr",      &_conf.crcConstraint.radiusErr,              0.0f,      1.0f);
    numTemp++;/* 3*/_alg->insert("crcMinPhi",         &_conf.crcConstraint.Phi.minVal,            -2.0f,      0.0f);
    numTemp++;/* 4*/_alg->insert("crcMaxPhi",         &_conf.crcConstraint.Phi.maxVal,             0.0f,      2.0f);
    numTemp++;/* 5*/_alg->insert("crcMinP",           &_conf.crcConstraint.P.minVal,             -10.0f,      0.0f);
    numTemp++;/* 6*/_alg->insert("crcMaxP",           &_conf.crcConstraint.P.maxVal,               0.0f,     10.0f);
    numTemp++;/* 7*/_alg->insert("crcMinAlr",         &_conf.crcConstraint.Alr.minVal,            -1.0f,      0.0f);
    numTemp++;/* 8*/_alg->insert("crcMaxAlr",         &_conf.crcConstraint.Alr.maxVal,             0.0f,      1.0f);
    numTemp++;/* 9*/_alg->insert("crcRudderYacc",     &_conf.crcConstraint.rudderYacc);
    numTemp++;/*10*/_alg->insert("crcMinRdr",         &_conf.crcConstraint.Rdr.minVal,            -1.0f,      0.0f);
    numTemp++;/*11*/_alg->insert("crcMaxRdr",         &_conf.crcConstraint.Rdr.maxVal,             0.0f,      1.0f);
    numTemp++;/*12*/_alg->insert("crcMinR",           &_conf.crcConstraint.R.minVal,             -10.0f,      0.0f);
    numTemp++;/*13*/_alg->insert("crcMaxR",           &_conf.crcConstraint.R.maxVal,               0.0f,     10.0f);
    numTemp++;/*14*/_alg->insert("crcAirspeed",       &_conf.crcConstraint.airspeed,          54.0f,    201.0f, 1, false, UAF_SET_CRC_AIRSPD);
    numTemp++;/*15*/_alg->insert("crcMinThetaSpd",    &_conf.crcConstraint.ThetaSpd.minVal,       -2.0f,      0.0f);
    numTemp++;/*16*/_alg->insert("crcMaxThetaSpd",    &_conf.crcConstraint.ThetaSpd.maxVal,        0.0f,      2.0f);
    numTemp++;/*17*/_alg->insert("crcMinThetaAlt",    &_conf.crcConstraint.ThetaAlt.minVal,       -2.0f,      0.0f);
    numTemp++;/*18*/_alg->insert("crcMaxThetaAlt",    &_conf.crcConstraint.ThetaAlt.maxVal,        0.0f,      2.0f);
    numTemp++;/*19*/_alg->insert("crcMinQ",           &_conf.crcConstraint.Q.minVal,             -10.0f,      0.0f);
    numTemp++;/*20*/_alg->insert("crcMaxQ",           &_conf.crcConstraint.Q.maxVal,               0.0f,     10.0f);
    numTemp++;/*21*/_alg->insert("crcMinElv",         &_conf.crcConstraint.Elv.minVal,            -1.0f,      0.0f);
    numTemp++;/*22*/_alg->insert("crcMaxElv",         &_conf.crcConstraint.Elv.maxVal,             0.0f,      1.0f);
    numTemp++;/*23*/_alg->insert("crcMinThr",         &_conf.crcConstraint.Thr.minVal,             0.0f,      1.0f);
    numTemp++;/*24*/_alg->insert("crcMaxThr",         &_conf.crcConstraint.Thr.maxVal,             0.0f,      1.0f);
    numTemp++;/*25*/_alg->insert("crcMrgLowThr",      &_conf.crcConstraint.mrgLowThr,      -1000.0f,   1000.0f, 2);
    numTemp++;/*26*/_alg->insert("crcMrgHighThr",     &_conf.crcConstraint.mrgHighThr,     -1000.0f,   1000.0f, 2);
    numTemp++;/*27*/_alg->insert("crcMinBtfly",       &_conf.crcConstraint.minBtfly,           0.0f,      1.0f);
    numTemp++;/*28*/_alg->insert("crcMaxBtfly",       &_conf.crcConstraint.maxBtfly,           0.0f,      1.0f);
    numTemp++;/*29*/_alg->insert("crcMrgLowBtfly",    &_conf.crcConstraint.mrgLowBtfly,    -1000.0f,   1000.0f, 2);
    numTemp++;/*30*/_alg->insert("crcMrgHighBtfly",   &_conf.crcConstraint.mrgHighBtfly,   -1000.0f,   1000.0f, 2);
    numTemp++;/*31*/_alg->insert("crcBaseTime",       &_conf.crcConstraint.baseTime,          -0.0f,  18000.0f, 1);
    numTemp++;/*32*/_alg->insert("crcNormTime",       &_conf.crcConstraint.normTime,          -0.0f,  18000.0f, 1);
    numTemp++;/*33*/_alg->insert("crcBaseAltitude",   &_conf.crcConstraint.baseAltitude,   -5000.0f,  10000.0f, 1);
    numTemp++;/*34*/_alg->insert("crcMinFlp",         &_conf.crcConstraint.Flp.minVal,            -1.0f,      1.0f);
    numTemp++;/*35*/_alg->insert("crcMaxFlp",         &_conf.crcConstraint.Flp.maxVal,            -1.0f,      1.0f);
    numTemp++;/*36*/_alg->insert("crcMinVertSpd",     &_conf.crcConstraint.VertSpd.minVal,      -100.0f,      0.0f);
    numTemp++;/*37*/_alg->insert("crcMaxVertSpd",     &_conf.crcConstraint.VertSpd.maxVal,         0.0f,    100.0f);
    numTemp++;/*38*/_alg->insert("crcMinTrackCorr",   &_conf.crcConstraint.TrackCorr.minVal,    -100.0f,      0.0f);
    numTemp++;/*39*/_alg->insert("crcMaxTrackCorr",   &_conf.crcConstraint.TrackCorr.maxVal,       0.0f,    100.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for crc mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setSemiLandModeParams(int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_alg->insert("semiLandMinPhi",         &_conf.semiLandConstraint.Phi.minVal,            -2.0f,      0.0f);
    numTemp++;/* 2*/_alg->insert("semiLandMaxPhi",         &_conf.semiLandConstraint.Phi.maxVal,             0.0f,      2.0f);
    numTemp++;/* 3*/_alg->insert("semiLandMinP",           &_conf.semiLandConstraint.P.minVal,             -10.0f,      0.0f);
    numTemp++;/* 4*/_alg->insert("semiLandMaxP",           &_conf.semiLandConstraint.P.maxVal,               0.0f,     10.0f);
    numTemp++;/* 5*/_alg->insert("semiLandMinAlr",         &_conf.semiLandConstraint.Alr.minVal,            -1.0f,      0.0f);
    numTemp++;/* 6*/_alg->insert("semiLandMaxAlr",         &_conf.semiLandConstraint.Alr.maxVal,             0.0f,      1.0f);
    numTemp++;/* 7*/_alg->insert("semiLandRudderYacc",     &_conf.semiLandConstraint.rudderYacc);
    numTemp++;/* 8*/_alg->insert("semiLandMinRdr",         &_conf.semiLandConstraint.Rdr.minVal,            -1.0f,      0.0f);
    numTemp++;/* 9*/_alg->insert("semiLandMaxRdr",         &_conf.semiLandConstraint.Rdr.maxVal,             0.0f,      1.0f);
    numTemp++;/*10*/_alg->insert("semiLandMinR",           &_conf.semiLandConstraint.R.minVal,             -10.0f,      0.0f);
    numTemp++;/*11*/_alg->insert("semiLandMaxR",           &_conf.semiLandConstraint.R.maxVal,               0.0f,     10.0f);
    numTemp++;/*12*/_alg->insert("semiLandMinThetaSpd",    &_conf.semiLandConstraint.ThetaSpd.minVal,       -2.0f,      0.0f);
    numTemp++;/*13*/_alg->insert("semiLandMaxThetaSpd",    &_conf.semiLandConstraint.ThetaSpd.maxVal,        0.0f,      2.0f);
    numTemp++;/*14*/_alg->insert("semiLandMinThetaAlt",    &_conf.semiLandConstraint.ThetaAlt.minVal,       -2.0f,      0.0f);
    numTemp++;/*15*/_alg->insert("semiLandMaxThetaAlt",    &_conf.semiLandConstraint.ThetaAlt.maxVal,        0.0f,      2.0f);
    numTemp++;/*16*/_alg->insert("semiLandMinQ",           &_conf.semiLandConstraint.Q.minVal,             -10.0f,      0.0f);
    numTemp++;/*17*/_alg->insert("semiLandMaxQ",           &_conf.semiLandConstraint.Q.maxVal,               0.0f,     10.0f);
    numTemp++;/*18*/_alg->insert("semiLandMinElv",         &_conf.semiLandConstraint.Elv.minVal,            -1.0f,      0.0f);
    numTemp++;/*19*/_alg->insert("semiLandMaxElv",         &_conf.semiLandConstraint.Elv.maxVal,             0.0f,      1.0f);
    numTemp++;/*20*/_alg->insert("semiLandMinThr",         &_conf.semiLandConstraint.Thr.minVal,             0.0f,      1.0f);
    numTemp++;/*21*/_alg->insert("semiLandMaxThr",         &_conf.semiLandConstraint.Thr.maxVal,             0.0f,      1.0f);
    numTemp++;/*22*/_alg->insert("semiLandMrgLowThr",      &_conf.semiLandConstraint.mrgLowThr,      -1000.0f,   1000.0f, 2);
    numTemp++;/*23*/_alg->insert("semiLandMrgHighThr",     &_conf.semiLandConstraint.mrgHighThr,     -1000.0f,   1000.0f, 2);
    numTemp++;/*24*/_alg->insert("semiLandMinBtfly",       &_conf.semiLandConstraint.minBtfly,           0.0f,      1.0f);
    numTemp++;/*25*/_alg->insert("semiLandMaxBtfly",       &_conf.semiLandConstraint.maxBtfly,           0.0f,      1.0f);
    numTemp++;/*26*/_alg->insert("semiLandMrgLowBtfly",    &_conf.semiLandConstraint.mrgLowBtfly,    -1000.0f,   1000.0f, 2);
    numTemp++;/*27*/_alg->insert("semiLandMrgHighBtfly",   &_conf.semiLandConstraint.mrgHighBtfly,   -1000.0f,   1000.0f, 2);
    numTemp++;/*28*/_alg->insert("semiLandMinFlp",         &_conf.semiLandConstraint.Flp.minVal,            -1.0f,      1.0f);
    numTemp++;/*29*/_alg->insert("semiLandMaxFlp",         &_conf.semiLandConstraint.Flp.maxVal,            -1.0f,      1.0f);
    numTemp++;/*30*/_alg->insert("semiLandMinVertSpd",     &_conf.semiLandConstraint.VertSpd.minVal,      -100.0f,      0.0f);
    numTemp++;/*31*/_alg->insert("semiLandMaxVertSpd",     &_conf.semiLandConstraint.VertSpd.maxVal,         0.0f,    100.0f);
    numTemp++;/*32*/_alg->insert("semiLandMinTrackCorr",   &_conf.semiLandConstraint.TrackCorr.minVal,    -100.0f,      0.0f);
    numTemp++;/*33*/_alg->insert("semiLandMaxTrackCorr",   &_conf.semiLandConstraint.TrackCorr.maxVal,       0.0f,    100.0f);

	numTemp++;/*34*/_alg->insert("semiLandAirspeed",         &_conf.semiLandConstraint.airspeed,            60.0f,  201.0f);
	numTemp++;/*35*/_alg->insert("semiLandApproachAlt",      &_conf.semiLandConstraint.approachAlt,      -1000.0f, 1000.0f);
	numTemp++;/*36*/_alg->insert("semiLandApproachDeltaAlt", &_conf.semiLandConstraint.approachDeltaAlt,     0.0f,  100.0f);
	numTemp++;/*37*/_alg->insert("semiLandTrackRange",       &_conf.semiLandConstraint.trackRange,         -30.0f,   30.0f);
	numTemp++;/*38*/_alg->insert("semiLandUseFlap",          &_conf.semiLandConstraint.bUseFlap);
	numTemp++;/*39*/_alg->insert("semiLandDescentAlt",       &_conf.semiLandConstraint.descentAlt,       -1000.0f, 1000.0f);
	numTemp++;/*40*/_alg->insert("semiLandDescentDeltaAlt",  &_conf.semiLandConstraint.descentDeltaAlt,      0.0f,  100.0f);
	numTemp++;/*41*/_alg->insert("semiLandAccX",             &_conf.semiLandConstraint.accX,                -5.0f,    5.0f);
	numTemp++;/*42*/_alg->insert("semiLandAbortAlt",         &_conf.semiLandConstraint.abortAlt,         -1000.0f, 1000.0f);
    numTemp++;/*43*/_alg->insert("semiLandAbortRadius",      &_conf.semiLandConstraint.abortRadius,          0.0f, 5000.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for semi land mode params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setForceLandParams (int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_alg->insert("fLndThrOffTime",         &_conf.fLndThrOffTime,          0.0f,    100.0f, 1);
    numTemp++;/* 2*/_alg->insert("fLndTheta",              &_conf.fLndTheta,              -1.0f,      1.0f, 3);
    numTemp++;/* 3*/_alg->insert("fLndMaxAbsPhi",          &_conf.fLndMaxAbsPhi,           0.0f,      1.0f, 3);
    numTemp++;/* 4*/_alg->insert("fLndBtfly",              &_conf.fLndBtfly,               0.0f,      1.0f, 2);
    numTemp++;/* 5*/_alg->insert("fLndLandedAccX",         &_conf.fLndLandedAccX,          0.0f,     10.0f, 2);
    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for force land params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setEmergencyLandParams (int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_alg->insert("emrgLandEnable",         &_conf.emrgLandEnable);
    numTemp++;/* 2*/_alg->insert("emrgLandAgl",            &_conf.emrgLandAgl,             0.0f,    200.0f, 1);
    numTemp++;/* 3*/_alg->insert("emrgLandMrg",            &_conf.emrgLandMrg,             0.0f,    100.0f, 1);
#if EMRG_LOSS_ENGINE == 1
	numTemp++;/* 5*/_alg->insert("emrgMinPhiDeg",          &_conf.emrgMinPhiDeg,         -30.0f,      0.0f, 1);
	numTemp++;/* 6*/_alg->insert("emrgMaxPhiDeg",          &_conf.emrgMaxPhiDeg,           0.0f,     30.0f, 1);
	numTemp++;/* 7*/_alg->insert("emrgMinThetaDeg",        &_conf.emrgMinThetaDeg,       -30.0f,      0.0f, 1);
	numTemp++;/* 8*/_alg->insert("emrgMaxThetaDeg",        &_conf.emrgMaxThetaDeg,         0.0f,     30.0f, 1);
	numTemp++;/* 9*/_alg->insert("emrgAirspeedRef",        &_conf.emrgAirspeedRef,         0.0f,    150.0f, 1);
	numTemp++;/*10*/_alg->insert("emrgAltitude",           &_conf.emrgAltitude,       -10000.0f,  10000.0f, 1);
	numTemp++;/*11*/_alg->insert("emrgRadius",             &_conf.emrgRadius,              0.0f,  10000.0f, 1);
	numTemp++;/*12*/_alg->insert("emrgThetaDeg",           &_conf.emrgThetaDeg,          -50.0f,     50.0f, 1);
#endif
    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for emergency land params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setStallRecoveryParams (int numIndex)
{
    int numTemp = 0;

    numTemp++;/* 1*/_alg->insert("stallRecEnable",         &_conf.stallRecEnable);
    numTemp++;/* 2*/_alg->insert("stallRecAirspeedOn",     &_conf.stallRecAirspeedOn,     0.0f,    100.0f, 1);
    numTemp++;/* 3*/_alg->insert("stallRecPOn",            &_conf.stallRecPOn,            0.0f,     10.0f);
    numTemp++;/* 4*/_alg->insert("stallRecPhiOn",          &_conf.stallRecPhiOn,          0.0f,     10.0f);
    numTemp++;/* 5*/_alg->insert("stallRecElevator",       &_conf.stallRecElevator,      -1.0f,      1.0f);
    numTemp++;/* 6*/_alg->insert("stallRecAirspeedOff",    &_conf.stallRecAirspeedOff,    0.0f,    100.0f, 1);
    numTemp++;/* 7*/_alg->insert("stallRecTimeOff",        &_conf.stallRecTimeOff,        0.0f,     10.0f);
    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for stall recovery params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setAutoTkfCatapultParams (int numIndex)
{
    int numTemp = 0;
    
	numTemp++;/* 1*/_alg->insert("autoTkfCatapultEnable",           &_conf.autoTkfCatapultEnable,           0,      1);
    numTemp++;/* 2*/_alg->insert("autoTkfCatapultPitch",            &_conf.autoTkfCatapultPitch,         0.0f,  20.0f);
    numTemp++;/* 3*/_alg->insert("autoTkfCatapultTimeout",          &_conf.autoTkfCatapultTimeout,       0.0f,  10.0f);
    numTemp++;/* 4*/_alg->insert("autoTkfCatapultThetaMaxErr",      &_conf.autoTkfCatapultThetaMaxErr,   0.0f,  20.0f); 
	numTemp++;/* 5*/_alg->insert("autoTkfCatapultClimbAlt",         &_conf.autoTkfCatapultClimbAlt,      0.0f, 100.0f);  
    numTemp++;/* 6*/_alg->insert("autoTkfCatapultAirspeedRange",    &_conf.autoTkfCatapultAirspeedRange, 0.0f, 100.0f);

    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for auto takeoff catapult params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

bool FlightPlanRealizer::setAutoParachutingParams(int numIndex)
{
	int numTemp = 0;

	numTemp++;/* 1*/_alg->insert("autoParaRadius", &_conf.autoParaRadius, 0.0f, 200.0f);
	numTemp++;/* 2*/_alg->insert("autoParaAltitude", &_conf.autoParaAltitude, 0.0f, 200.0f);
	numTemp++;/* 3*/_alg->insert("autoParaAirspeed", &_conf.autoParaAirspeed, 0.0f, 100.0f);
	numTemp++;/* 4*/_alg->insert("autoParaStableVelZ", &_conf.autoParaStableVelZ, 0.0f, 10.0f);
	numTemp++;/* 5*/_alg->insert("autoParaDelTrack", &_conf.autoParaDelTrack, 0.0f, 10.0f);
	numTemp++;/* 6*/_alg->insert("autoParaBaseTime", &_conf.autoParaBaseTime, 0.0f, 200.0f);
	numTemp++;/* 7*/_alg->insert("autoParaApproachTime", &_conf.autoParaApproachTime, 0.0f, 100.0f);
	
	if (numTemp > numIndex)
	{
		Log.msgPrintf("%sNot enough space for auto parachute landing params", SUBS_PREFIX);
		return false;
	}
	else
	{
		return true;
	}
}

bool FlightPlanRealizer::setSubsystemParams (int numIndex)
{
    int numTemp = 0;
    numTemp++;/* 1*/_alg->insert("pressFaultTheta",        &_conf.pressFaultTheta,       -0.3f,       0.0f, 3);  
    numTemp++;/* 2*/_alg->insert("pressFaultIas",          &_conf.pressFaultIas,          0.0f,     120.0f, 1);
    numTemp++;/* 3*/_alg->insert("configAutosave",         &_conf.configAutosave);  
    numTemp++;/* 4*/_alg->insert("mainDivisor",            &_conf.mainDivisor, 1, 100);
    numTemp++;/* 5*/_alg->insert("commDivisor",            &_conf.commDivisor, 0, 1000);
    numTemp++;/* 6*/_alg->insert("logDivisor",             &_conf.logDivisor, 0, 1000);
	numTemp++;/* 7*/_alg->insert("tuningLat",              &_ramStorage.tuningLat, 0, 10);
	numTemp++;/* 8*/_alg->insert("tuningLon",              &_ramStorage.tuningLon, 0, 10);
	numTemp++;/* 9*/_alg->insert("tuningSpd",              &_ramStorage.tuningSpd, 0, 10);
	numTemp++;/*10*/_alg->insert("tuneEnable",             &_ramStorage.bTuneEnable);
	numTemp++;/*11*/_alg->insert("useCorrTrack",           &_conf.bUseCorrTrack);
    if (numTemp > numIndex)
    {
        Log.msgPrintf ("%sNot enough space for subsystem params", SUBS_PREFIX);
        return false;
    }
    else
    {
        return true;
    }
}

void FlightPlanRealizer::setFlightPhase (void)
{
    //  array of pointers to the functions controlling flight phases (for safety reasons they are reset).
    for (int i = 0; i < MAX_PHASE_FUN; i++)
    {
        _funTab[i] = NULL;
    }

    _funTab[FLY_PHASE_2] = &FlightPlanRealizer::flyPhase2;
	_funTab[AUTO_TAKEOFF_CATAPULT_P1] = &FlightPlanRealizer::autoTkfCatapultPhase1;
    _funTab[AUTO_TAKEOFF_CATAPULT_P2] = &FlightPlanRealizer::autoTkfCatapultPhase2;
    _funTab[AUTO_TAKEOFF_CATAPULT_P3] = &FlightPlanRealizer::autoTkfCatapultPhase3;
	_funTab[AUTO_BEGIN_MISSION]       = &FlightPlanRealizer::autoBeginMission;
    _funTab[MT_OFF] = &FlightPlanRealizer::turnOff;
    _funTab[STALL_ON] = &FlightPlanRealizer::stallOn;
    _funTab[PRELANDING_P2] = &FlightPlanRealizer::netPrelandPhase2;
    _funTab[APPROACH] = &FlightPlanRealizer::netApproach;
    _funTab[DESCENT] = &FlightPlanRealizer::netDescent;
    _funTab[NET_LAND] = &FlightPlanRealizer::netLand;
	_funTab[POST_LAND] = &FlightPlanRealizer::netPostLand;
    _funTab[ABORT_LAND] = &FlightPlanRealizer::netAbort;
    _funTab[GOAROUND_NET] = &FlightPlanRealizer::netGoAround;

    _funTab[STALL_OFF] = &FlightPlanRealizer::stallOff;

    _funTab[CRC_PHASE_2] = &FlightPlanRealizer::circlePhase2;
    _funTab[FLND_PHASE_2] = &FlightPlanRealizer::forceLandPhase2;
    _funTab[FLND_PHASE_3] = &FlightPlanRealizer::forceLandPhase3;
    _funTab[GOTO_PHASE_2] = &FlightPlanRealizer::fpGotoPhase2;
    
    // Functions that support other conditions (not flight phases)
    _funTab[LOW_GSPD_ON] = &FlightPlanRealizer::lowGSpeedOn;
    _funTab[LOW_GSPD_OFF] = &FlightPlanRealizer::lowGSpeedOff;

	// Parachute Landing Mode
	_funTab[CRC_BASE_PARA_PHASE_2] = &FlightPlanRealizer::circleBaseParaPhase2;
	_funTab[CRC_PARA_PHASE_1] = &FlightPlanRealizer::circleParaPhase1;
	_funTab[CRC_PARA_PHASE_2] = &FlightPlanRealizer::circleParaPhase2;
	_funTab[CRC_PARA_PHASE_3] = &FlightPlanRealizer::parachuting;

	//Camguide
	_funTab[CRC_CAMGUIDE] = &FlightPlanRealizer::circleCamguide;
    _funTab[SEARCH] = &FlightPlanRealizer::searchObj;
	_funTab[CRC_CRC_CAMGUIDE_PHASE_2] = &FlightPlanRealizer::circleCamguidePhase2;
}

void FlightPlanRealizer::setLatControllers (CONTROLLER_CHANNEL_ID ID)
{	
	switch (ID)
	{
	case LAT_UNUSED_ID:
		_state.fprd.ctrlProps.disableLatControllers();
		return;

	case LAT_HOLD_Q_ID:
		_state.fprd.ctrlProps.disableLatControllers();
		setElvControllers (ID);
		return;

	case LAT_HOLD_THETA_ID:
		setElvControllers (LAT_HOLD_Q_ID);
		setQControllers (ID);
		setThetaControllers (LAT_UNUSED_THETA_ID);
		return;

	case LAT_HOLD_VZ_ID:
		setElvControllers (LAT_HOLD_Q_ID);
		setQControllers (LAT_HOLD_THETA_ID);
		setThetaControllers (ID);
		setVertSpeedControllers (LAT_UNUSED_VZ_ID);
		return;

	case LAT_VZ_ALT_ID:
		setElvControllers (LAT_HOLD_Q_ID);
		setQControllers (LAT_HOLD_THETA_ID);
		setThetaControllers (LAT_HOLD_VZ_ID);
		setVertSpeedControllers (ID);
		return;

	case LAT_THETA_ALT_ID: case LAT_HOLD_SPD_ID:
		setElvControllers (LAT_HOLD_Q_ID);
		setQControllers (LAT_HOLD_THETA_ID);
		setThetaControllers (ID);
		setVertSpeedControllers (LAT_UNUSED_VZ_ID);
		return;

	case LAT_UNCHANGED_ID:
		return;

	default:
		Log.errorPrint("Error: Undefined Lat Id");
		return;
	}
}

void FlightPlanRealizer::setElvControllers (CONTROLLER_CHANNEL_ID ID)
{
	switch (ID)
    {
	case LAT_UNUSED_ID:
		_state.fprd.ctrlProps.ElvCProp.disableControllers ();
		return;
	case LAT_HOLD_Q_ID:
		_state.fprd.ctrlProps.ElvCProp.setControllerParams (ID, _flightConstraint->Elv.minVal, _flightConstraint->Elv.maxVal);
		return;
	default: return;
	}
}

void FlightPlanRealizer::setQControllers (CONTROLLER_CHANNEL_ID ID)
{
	switch (ID)
    {
	case LAT_UNUSED_Q_ID:
		_state.fprd.ctrlProps.QCProp.disableControllers ();
		return;
	case LAT_HOLD_THETA_ID:
		_state.fprd.ctrlProps.QCProp.setControllerParams (ID, _flightConstraint->Q.minVal, _flightConstraint->Q.maxVal);
		return;
	default: return;
	}
}

void FlightPlanRealizer::setThetaControllers (CONTROLLER_CHANNEL_ID ID)
{
	switch (ID)
    {
	case LAT_UNUSED_THETA_ID:
		_state.fprd.ctrlProps.ThetaCProp.disableControllers ();
		return;
	case LAT_HOLD_VZ_ID: case LAT_HOLD_SPD_ID:
#if EMRG_LOSS_ENGINE == 1
		if (SysMon->getLossEngine())
			_state.fprd.ctrlProps.ThetaCProp.setControllerParams (ID, _conf.emrgMinThetaDeg * DEG_2_RAD, _conf.emrgMaxThetaDeg * DEG_2_RAD);
		else
#endif
			_state.fprd.ctrlProps.ThetaCProp.setControllerParams (ID, _flightConstraint->ThetaSpd.minVal, _flightConstraint->ThetaSpd.maxVal);
		return;
	case LAT_THETA_ALT_ID:
#if EMRG_LOSS_ENGINE == 1
		if (SysMon->getLossEngine())
			_state.fprd.ctrlProps.ThetaCProp.setControllerParams (ID, _conf.emrgMinThetaDeg * DEG_2_RAD, _conf.emrgMaxThetaDeg * DEG_2_RAD);
		else
#endif
			_state.fprd.ctrlProps.ThetaCProp.setControllerParams (ID, _flightConstraint->ThetaAlt.minVal, _flightConstraint->ThetaAlt.maxVal);
		return;
	default: return;
	}
}

void FlightPlanRealizer::setVertSpeedControllers (CONTROLLER_CHANNEL_ID ID)
{
	switch (ID)
    {
	case LAT_UNUSED_VZ_ID:
		_state.fprd.ctrlProps.VertSpeedCProp.disableControllers ();
		return;
	case LAT_VZ_ALT_ID:
		_state.fprd.ctrlProps.VertSpeedCProp.setControllerParams (ID, _flightConstraint->VertSpd.minVal, _flightConstraint->VertSpd.maxVal);
		return;
	default: return;
	}
}

void FlightPlanRealizer::setLonControllers (CONTROLLER_CHANNEL_ID ALR_ID, CONTROLLER_CHANNEL_ID RDR_ID)
{
	switch (ALR_ID)
	{
	case LON_UNUSED_ALR_ID:
		_state.fprd.ctrlProps.disableLonAlrControllers();
		break;

	case LON_HOLD_P_ID:
		_state.fprd.ctrlProps.disableLonAlrControllers();
		setAlrControllers (ALR_ID);
		break;

	case LON_HOLD_PHI_ID:
		setAlrControllers (LON_HOLD_P_ID);
		setPControllers (ALR_ID);
		setPhiControllers (LON_UNUSED_PHI_ID);
		setTrackControllers (LON_UNUSED_TRACK_ID);
		setTrackCorrControllers (LON_UNUSED_TRACKCORR_ID);
		break;

	case LON_HOLD_TRACK_ID: case LON_HOLD_CTRACK_ID: case LON_HOLD_PSI_ID:
		setAlrControllers (LON_HOLD_P_ID);
		setPControllers (LON_HOLD_PHI_ID);
		setPhiControllers (ALR_ID);
		setTrackControllers (LON_UNUSED_TRACK_ID);
		setTrackCorrControllers (LON_UNUSED_TRACKCORR_ID);
		break;

	case LON_CRC_WPT_ID:
		setAlrControllers (LON_HOLD_P_ID);
		setPControllers (LON_HOLD_PHI_ID);
		setPhiControllers (LON_HOLD_CTRACK_ID);
		setTrackControllers (ALR_ID);
		setTrackCorrControllers (LON_UNUSED_TRACKCORR_ID);
		break;

	case LON_TRACK_WPT_ID:
		setAlrControllers (LON_HOLD_P_ID);
		setPControllers (LON_HOLD_PHI_ID);
		setPhiControllers (LON_HOLD_TRACK_ID);
		setTrackControllers (ALR_ID);
		setTrackCorrControllers (LON_UNUSED_TRACKCORR_ID);
		break;

	case LON_TRACK_PATH_ID:
		setAlrControllers (LON_HOLD_P_ID);
		setPControllers (LON_HOLD_PHI_ID);
		setPhiControllers (LON_HOLD_TRACK_ID);
		setTrackControllers (ALR_ID);
		setTrackCorrControllers (ALR_ID);
		break;

	case LON_UNCHANGED_ALR_ID:
		break;

	default:
		Log.errorPrint("Error: Undefined Lon Alr Id");
		break;
	}
    
    switch (RDR_ID)
	{
	case LON_UNUSED_RDR_ID:
		_state.fprd.ctrlProps.disableLonRdrControllers();
		return;

	case LON_HOLD_R_ID: case LON_HOLD_YACC_ID:
		setRdrControllers (RDR_ID);
		setRControllers (LON_UNUSED_R_ID);
		return;

	case LON_R_PSI_ID: case LON_R_COORDEXP_ID: case LON_R_TRACK_ID:
		setRdrControllers (LON_HOLD_R_ID);
		setRControllers (RDR_ID);
		return;

	case LON_UNCHANGED_RDR_ID:
		return;

	 default:
		Log.errorPrint("Error: Undefined Lon Rdr Id");
		return;
	}
}

void FlightPlanRealizer::setAlrControllers (CONTROLLER_CHANNEL_ID ALR_ID)
{
    switch (ALR_ID)
    {
	case LON_UNUSED_ALR_ID:
		_state.fprd.ctrlProps.AlrCProp.disableControllers();
		return;
	case LON_HOLD_P_ID:
		_state.fprd.ctrlProps.AlrCProp.setControllerParams (ALR_ID, _flightConstraint->Alr.minVal, _flightConstraint->Alr.maxVal);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setPControllers (CONTROLLER_CHANNEL_ID ALR_ID)
{
    switch (ALR_ID)
    {
	case LON_UNUSED_P_ID:
		_state.fprd.ctrlProps.PCProp.disableControllers();
		return;
	case LON_HOLD_PHI_ID:
		_state.fprd.ctrlProps.PCProp.setControllerParams (ALR_ID, _flightConstraint->P.minVal, _flightConstraint->P.maxVal);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setPhiControllers (CONTROLLER_CHANNEL_ID ALR_ID)
{
    switch (ALR_ID)
    {
	case LON_UNUSED_PHI_ID:
		_state.fprd.ctrlProps.PhiCProp.disableControllers();
		return;
	case LON_HOLD_TRACK_ID: case LON_HOLD_CTRACK_ID: case LON_HOLD_PSI_ID:
#if EMRG_LOSS_ENGINE == 1
		if (SysMon->getLossEngine())
			_state.fprd.ctrlProps.PhiCProp.setControllerParams (ALR_ID, _conf.emrgMinPhiDeg * DEG_2_RAD, _conf.emrgMaxPhiDeg * DEG_2_RAD);
		else
#endif
			_state.fprd.ctrlProps.PhiCProp.setControllerParams (ALR_ID, _flightConstraint->Phi.minVal, _flightConstraint->Phi.maxVal);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setTrackControllers (CONTROLLER_CHANNEL_ID ALR_ID)
{
    switch (ALR_ID)
    {
	case LON_UNUSED_TRACK_ID:
		_state.fprd.ctrlProps.TrackCProp.disableControllers();
		return;
	case LON_TRACK_WPT_ID: case LON_TRACK_PATH_ID: case LON_CRC_WPT_ID:
		_state.fprd.ctrlProps.TrackCProp.setControllerParams (ALR_ID, 0.0f, 360.0f);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setTrackCorrControllers (CONTROLLER_CHANNEL_ID ALR_ID)
{
    switch (ALR_ID)
    {
	case LON_UNUSED_TRACKCORR_ID:
		_state.fprd.ctrlProps.TrackCorrCProp.disableControllers();
		return;
	case LON_TRACK_PATH_ID:
		_state.fprd.ctrlProps.TrackCorrCProp.setControllerParams (ALR_ID, _flightConstraint->TrackCorr.minVal, _flightConstraint->TrackCorr.maxVal);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setRdrControllers (CONTROLLER_CHANNEL_ID RDR_ID)
{
	switch (RDR_ID)
    {
	case LON_UNUSED_RDR_ID:
		_state.fprd.ctrlProps.RudderCProp.disableControllers();
		return;
	case LON_HOLD_R_ID: case LON_HOLD_YACC_ID:
		_state.fprd.ctrlProps.RudderCProp.setControllerParams (RDR_ID, _flightConstraint->Rdr.minVal, _flightConstraint->Rdr.maxVal);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setRControllers (CONTROLLER_CHANNEL_ID RDR_ID)
{
	switch (RDR_ID)
    {
	case LON_UNUSED_R_ID:
		_state.fprd.ctrlProps.RCProp.disableControllers();
		return;
	case LON_R_PSI_ID: case LON_R_COORDEXP_ID: case LON_R_TRACK_ID:
		_state.fprd.ctrlProps.RCProp.setControllerParams (RDR_ID, _flightConstraint->R.minVal, _flightConstraint->R.maxVal);
        return;
	default: return;
	}
}

void FlightPlanRealizer::setSpdControllers (CONTROLLER_CHANNEL_ID THR_ID, CONTROLLER_CHANNEL_ID FLP_ID)
{
	switch (THR_ID)
	{
	case SPD_UNUSED_THR_ID: case SPD_THR_ALT_ID: case SPD_THR_SPD_ID:
		setThrControllers (THR_ID);
		break;
    
    case TECS_ID:
        setThrControllers (TECS_ID);

	case SPD_UNCHANGED_THR_ID:
		break;

	default:
		Log.errorPrint("Error: Undefined Spd Thr Id");
		break;
	}

	switch (FLP_ID)
	{
	case SPD_UNUSED_FLP_ID: case SPD_FLP_SPD_ID:
		setFlpControllers (FLP_ID);
		return;

	case SPD_UNCHANGED_FLP_ID:
		return;

	default:
		Log.errorPrint("Error: Undefined Spd Flp Id");
		return;
	}
}

void FlightPlanRealizer::setThrControllers (CONTROLLER_CHANNEL_ID THR_ID)
{
	switch (THR_ID)
	{
	case SPD_UNUSED_THR_ID:
		_state.fprd.ctrlProps.ThrCProp.disableControllers();
		return;
	case SPD_THR_ALT_ID: 
		_state.fprd.ctrlProps.ThrCProp.setControllerParams (THR_ID, _flightConstraint->Thr.minVal, 0.95f * _flightConstraint->Thr.maxVal);
		return;
	case SPD_THR_SPD_ID:
		_state.fprd.ctrlProps.ThrCProp.setControllerParams (THR_ID, _flightConstraint->Thr.minVal, _flightConstraint->Thr.maxVal);
		return;
    case TECS_ID:
		_state.fprd.ctrlProps.TECSCProp.setControllerParams (TECS_ID, _flightConstraint->Thr.minVal, _flightConstraint->Thr.maxVal);
		return;
	default: return;
	}
}

void FlightPlanRealizer::setFlpControllers (CONTROLLER_CHANNEL_ID FLP_ID)
{
	switch (FLP_ID)
	{
	case SPD_UNUSED_FLP_ID: 
		_state.fprd.ctrlProps.FlpCProp.disableControllers();
		return;
	case SPD_FLP_SPD_ID:
        _state.fprd.ctrlProps.FlpCProp.setControllerParams (FLP_ID, _flightConstraint->Flp.minVal, _flightConstraint->Flp.maxVal);
		return;
	default: return;
	}
}

bool FlightPlanRealizer::setRefWaypoint (const char* pLat, const char* pLon, const char* pMode)
{
    GpsPosition pos;
    bool bok = false;

    //  coordinates parsing 
    if (STRICMP (pMode, "abs") == 0)
        //  gographical coordinates parsing treated as absolute values.
        bok = TypeParser::toGpsPosition (pLat, pLon, pos);
    else if (STRICMP (pMode, "rel") == 0)
    {
        //  gographical coordinates parsing treated as relative values.
        bok = TypeParser::toGpsPositionRel (pLat, pLon, _state.fprd.fRef.wptFrom, pos);
    }

    if (!bok)
    {
        // Bad syntax error - notify about flight plan completion (load new).
        Log.msgPrintf ("%s  -> Bad position format [%s]", SUBS_PREFIX, "error - skipping");
        fpCmdCompleted ();
        return false;
    }

    // Setting beggining point reference value to current position.
    // It affects the condition  of the rank point. 
	if ((_state.fprd.fRef.wptTo.getLon() == 0) || (_state.fprd.fRef.wptTo.getLat() == 0) || !_state.fprd.ctrlProps.TrackCorrCProp.TrackCorr_Cte.enable)
	{
		_state.fprd.fRef.wptFrom = _psd.position;
	}
	else
	{
		_state.fprd.fRef.wptFrom = _state.fprd.fRef.wptTo;
	}
    Log.msgPrintf ("%s  -> 'From' set to current lon:%.6f lat:%.6f",
        SUBS_PREFIX, _state.fprd.fRef.wptFrom.getLon(), _state.fprd.fRef.wptFrom.getLat());

    // set reference value of the desired point
    _state.fprd.fRef.wptTo = pos;

    // If start coordinates are equal to the end coordinates, current value has been set to start value.
    // It can happen during the flight when current flight instruction would be changed to previous one.
    if (_state.fprd.fRef.wptFrom.isEqual (_state.fprd.fRef.wptTo))
    {
        _state.fprd.fRef.wptFrom = _psd.position;
        Log.msgPrintf ("%s  -> 'From' and 'To' are equal - use current lon:%.6f lat:%.6f",
            SUBS_PREFIX, _state.fprd.fRef.wptFrom.getLon(), _state.fprd.fRef.wptFrom.getLat());
    }
    
    return true;
}

bool FlightPlanRealizer::setRefAltitude (const char* pAlt)
{
    float alt = _conf.dfAltitude;   //initialization with value > 0 (just in case)
    bool bok = true;

    //  altitude parsing
    if  (pAlt != NULL)
    {
        bok = TypeParser::toFloat (pAlt, alt);
    }

    if (!bok)
    {
        // Bad syntax error - notify about flight plan completion (load new).
        Log.msgPrintf ("%s  -> Bad altitude format [%s]", SUBS_PREFIX, "error - skipping");
        fpCmdCompleted ();
        return false;
    }

    Log.msgPrintf ("%s  -> Set default altitude to: %.1fm (from: %.1fm)", SUBS_PREFIX, alt, _conf.dfAltitude);
    _conf.dfAltitude = alt;
    _state.fprd.fRef.altitude = _conf.dfAltitude;

    return true;
}

void FlightPlanRealizer::setTuneControllers (void)
{
	setTuneLatControllers ();
	setTuneLonControllers ();
	setTuneSpdControllers ();
}

void FlightPlanRealizer::setTuneLatControllers (void)
{
	switch (_ramStorage.tuningLat)
	{
	case LAT_TUNING_UNUSED:
		setLatControllers (LAT_UNUSED_ID);
		break;
	case LAT_TUNING_HOLD_THETA:
		setLatControllers (LAT_HOLD_THETA_ID);
		break;
	case LAT_TUNING_THETA_ALT:
		setLatControllers (LAT_THETA_ALT_ID);
		break;
	case LAT_TUNING_HOLD_VZ:
		setLatControllers (LAT_HOLD_VZ_ID);
		break;
	case LAT_TUNING_VZ_ALT:
		setLatControllers (LAT_VZ_ALT_ID);
		break;
	case LAT_TUNING_HOLD_SPD:
		setLatControllers (LAT_HOLD_SPD_ID);
		break;
	default:
		break;
	}

	if (_ramStorage.tuningLat == LAT_TUNING_UNCHANGED)
		_ramStorage.tuningLat = _ramStorage.preTuningLat;
	else
		_ramStorage.preTuningLat = _ramStorage.tuningLat;
}

void FlightPlanRealizer::setTuneLonControllers (void)
{
	switch (_ramStorage.tuningLon)
	{
	case LON_TUNING_UNUSED:
		setLonControllers (LON_UNUSED_ALR_ID);
		break;
	case LON_TUNING_HOLD_PHI:
		setLonControllers (LON_HOLD_PHI_ID);
		break;
	case LON_TUNING_TRACK_WPT:
		setLonControllers (LON_TRACK_WPT_ID);
		break;
	case LON_TUNING_TRACK_PATH:
		setLonControllers (LON_TRACK_PATH_ID);
		break;
	default:
		break;
	}

	if (_ramStorage.tuningLon == LON_TUNING_UNCHANGED)
		_ramStorage.tuningLon = _ramStorage.preTuningLon;
	else
		_ramStorage.preTuningLon = _ramStorage.tuningLon;
}

void FlightPlanRealizer::setTuneSpdControllers (void)
{
	switch (_ramStorage.tuningSpd)
	{
	case SPD_TUNING_UNUSED:
		setSpdControllers (SPD_UNUSED_THR_ID);
		break;
	case SPD_TUNING_HOLD_SPD:
		setSpdControllers (SPD_THR_SPD_ID);
		break;
	case SPD_TUNING_HOLD_ALT:
		setSpdControllers (SPD_THR_ALT_ID);
		break;
	default:
		break;
	}

	if (_ramStorage.tuningSpd == SPD_TUNING_UNCHANGED)
		_ramStorage.tuningSpd = _ramStorage.preTuningSpd;
	else
		_ramStorage.preTuningSpd = _ramStorage.tuningSpd;
}
