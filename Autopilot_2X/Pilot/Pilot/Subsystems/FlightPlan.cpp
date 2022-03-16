/**
*                                                                   
* @class FlightPlan                                                        
* Class to control flight plan in logic layer. (data are proceed by the FPlanContainer class)
* It allows to load, modify and save flight plan commands. It cooperates with the FPlanRealizer class.
* 
* @brief Class storing and controlling flight plan.
* Marcin Pczycki (c) Flytronic 2008                                 
*/

#include <PilotIncludes.h>


/// Constructor, NIOS take notice to the order of the initializators to be the same order as declarations.
FlightPlan::FlightPlan(void):
ODTObserver(), ODTSubject(), SubsystemBase(),
_cmdq(Log),
_fpContainer (*(new FPlanContainer)),
_activeCmdGroupPtr(&_returnBLinkCmdGroup),
_err(FPlanConf::FPErrUnknown),	///< Last system error code.
_forceValidate(false),		///< Force flight plan validation before entering the running mode.
_missionMode(FPlanConf::MissionMode::AUTO),
_systemPanic(false),     	///< Systems critical error flag. Takeoff is suspended.
_isFPLoaded(false),         ///< Flight plan loaded flag.
_internalGotoLineId (ITEM_NOT_SET)
{
    // Tags initialization to prevent tags to has random values in case of registerSubj didn't work.
	SysMonTag = FPRealTag = FPRealBaseChangedTag = - 1;
    SysMonLandedTag = SysMonLinkTag = SysMonLowEnTag = SysMonNoGpsTag = InternalGotoTag = -1;
#if EMRG_LOSS_ENGINE == 1
	SysMonFocusLandTag = -1;
	SysMonLossEngineTag = -1;
#endif
	FControlClimbTag = FControlCruiseTag = FControlLevelFlightTag = -1;
    ExtCmdTag = -1;

	_flightPhase = 0;

	// Create semaphor controlling access to the getPlanData() method.
	if (!_vSem.create("FlightPlan"))
	{
		Log.abort ("Critical Error: FligthPlan_1");
		return;
	}

    //  Recovery mode instruction.
	_recoveryFPD.currentStatus = FPlanConf::Running;
	_recoveryFPD.currentItem = ITEM_NOT_SET;
	MEMCCPY(&_recoveryFPD.command, FPlanConf::RECOVERY_PLAN_COMMAND, 0, FPlanConf::FPLAN_LINE_LENGTH);
	MEMCCPY(&_recoveryFPD.fplanName, "RECOVERY", 0, FPlanConf::FPLAN_NAME_LENGTH);

    // Group of return mode (after lost of radio link) instruction.
	_returnBLinkCmdGroup.setCommand (0, 1, "climb by circle", "fpreturn", FPlanData::ITEM_TYPE_RETURN);	
    _returnBLinkCmdGroup.setCommand (1, 2, "fly base", "fpreturn", FPlanData::ITEM_TYPE_RETURN);
    _returnBLinkCmdGroup.setCommand (2, 3, "circle base", "fpreturn", FPlanData::ITEM_TYPE_RETURN);

	// Group of return mode instructions in case of low level energy.
	_returnLowEnCmdGroup.setCommand (0, 1, "climb by circle", "fpretLE", FPlanData::ITEM_TYPE_RETURN);
	_returnLowEnCmdGroup.setCommand (1, 2, "fly base", "fpretLE", FPlanData::ITEM_TYPE_RETURN);
	_returnLowEnCmdGroup.setCommand (2, 3, "circle base", "fpretLE", FPlanData::ITEM_TYPE_RETURN);

    // Group of return mode isntructions in case of GPS malfunction or errors in GPS. Gegraphical position is computed by alternative methods.
    _returnNoGpsCmdGroup.setCommand (0, 1, "climb by circle", "fpretNGPS", FPlanData::ITEM_TYPE_RETURN);
	_returnNoGpsCmdGroup.setCommand (1, 2, "fly base", "fpretNGPS", FPlanData::ITEM_TYPE_RETURN);
    _returnNoGpsCmdGroup.setCommand (2, 3, "circle base", "fpretNGPS", FPlanData::ITEM_TYPE_RETURN);

	// Group of parachute landing mode instructions in parachute landing phase
	_parachuteLandingCmdGroup.setCommand(0, 1, "fly base land", "fpparaLand", FPlanData::ITEM_TYPE_NORMAL);
	_parachuteLandingCmdGroup.setCommand(1, 2, "prepare circle base land", "fpparaLand", FPlanData::ITEM_TYPE_NORMAL);
	_parachuteLandingCmdGroup.setCommand(2, 3, "circle base parachute", "fpparaLand", FPlanData::ITEM_TYPE_NORMAL);
	_parachuteLandingCmdGroup.setCommand(3, 4, "parachuting", "fpparaLand", FPlanData::ITEM_TYPE_NORMAL);

	// Group of camera guide mission instructions
	_camGuideCmdGroup.setCommand(0, 1, "search", "fpcamguide", FPlanData::ITEM_TYPE_NORMAL);
	//_camGuideCmdGroup.setCommand(1, 2, "homing", "fpcamguide", FPlanData::ITEM_TYPE_NORMAL);
#if EMRG_LOSS_ENGINE == 1
	_lossEngineCmdGroup.setCommand (0, 1, "fly base", "fpretLossEngine", FPlanData::ITEM_TYPE_RETURN);
	_lossEngineCmdGroup.setCommand (1, 2, "circle base", "fpretLossEngine", FPlanData::ITEM_TYPE_RETURN);

	_focusLandCmdGroup.setCommand (0, 1, "descent by circle", "fpland", FPlanData::ITEM_TYPE_FOCUS_LAND);
	_focusLandCmdGroup.setCommand (1, 2, "focus land", "fpland", FPlanData::ITEM_TYPE_FOCUS_LAND);
#endif

	// Default status. In case of system is airborn initialization instruction will change status to the running.
	_status = FPlanConf::Stopped;

	Log.bootPrint("OK" CRLF);
}

/**
* Return name of state as text.
*/
const char* FlightPlan::statusAsString(FPlanConf::FPStatus stat) const
{
	switch(stat)
	{
	case FPlanConf::Running:		return "Running";
	case FPlanConf::OnHold:			return "OnHold";
	case FPlanConf::Stopped:		return "Stopped";
	default:						return "UNDEFINED";
	}
}

/**
* Fills given object with current flight plan registration.
*/
bool FlightPlan::getFPlanData (FPlanData &fpd)
{
	// Must be semahor lock because function will be executed parallely from diferent tasks.
	// In getFPlanData container is no need to use semaphor lock.
	if(!_vSem.lock())
	{
		return false;
	}
	switch (_missionMode)
	{
	case FPlanConf::MissionMode::RECOVERY:
        fpd = _recoveryFPD;
		break;
	case FPlanConf::MissionMode::RTL:
	case FPlanConf::MissionMode::PARACHUTE:
	case FPlanConf::MissionMode::CAMGUIDE:
#if EMRG_LOSS_ENGINE == 1
	case FPlanConf::SpecialMission::FOCUSLAND:
#endif
		fpd = *(_activeCmdGroupPtr->getCurrentFpd());
		break;
	default:
		if (!_fpContainer.getFPlanData(fpd, _status))
	{
        Log.msgPrintf("%sRECOVERY plan used due to: unable to get FPlanData", FPlanConf::SUBS_PREFIX);
        fpd = _recoveryFPD;
	}
		break;
	}
	_vSem.unlock();
	return true;
}

/**
* Registration objects observed
*/
void FlightPlan::linkObserver()
{
    // Possible errors in registerSubj breaks the program.
    ExtCmdTag				= registerSubj (this, EXT_CMD);
    SysMonTag				= registerSubj (SysMon, SYSMON_MSG);
    FPRealTag				= registerSubj (FPReal, FPEL_COMPLETED);
	FPRealBaseChangedTag	= registerSubj(FPReal, FPR_BASE_CHANGED);
    SysMonLandedTag			= registerSubj (SysMon, SYSMON_LANDED);
    InternalGotoTag			= registerSubj (this, FP_INTERNAL_GOTO);
	SysMonLinkTag			= registerSubj (SysMon, SYSMON_LINK);
    SysMonLowEnTag			= registerSubj (SysMon, SYSMON_LOW_ENERGY);
    SysMonNoGpsTag			= registerSubj (SysMon, SYSMON_NO_GPS);

	FControlClimbTag = registerSubj (FControl, CONTROLS_CLIMB);
	FControlCruiseTag = registerSubj (FControl, CONTROLS_CRUISE);
	FControlLevelFlightTag = registerSubj (FControl, CONTROLS_LEVELFLIGHT);

#if EMRG_LOSS_ENGINE == 1
	SysMonFocusLandTag = registerSubj (SysMon, SYSMON_FOCUS_LAND);
	SysMonLossEngineTag = registerSubj (SysMon, SYSMON_LOSS_ENGINE);
#endif
}

/**
* Go to line with given ID.
*/
void FlightPlan::gotoLine(int lineId)
{
    // Store line ID that will be goto instruction. Semaphors are no needed because it is general type.
    _internalGotoLineId = lineId;

    //  Self notify to realize flight plan line i FPlan subsystem.
    notify (FP_INTERNAL_GOTO);
}


/**
* Return TRUE if system is ready for takeoff. (fully functional).
* Sends error messages to the cl (if cl != NULL)
* Function is called in the context of other subsystem - note on synchronnous
*/
bool FlightPlan::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    bool bok = true;

    //  Loading configuration from flash checking.
    if (!_isFPLoaded)
    {
        _err = FPlanConf::FPErrSystemPanic;
        Log.msgPrintf (cl, fLogComm, FPlanConf::getErrorMsg(_err));
        bok = false;
    }

    return bok;
}

/**
* Function writes to command queue new line and sends internal notice.
* Returns TRUE when ok, FALSE when queue i full  (or the line was not written from other reason).
*/
bool FlightPlan::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("FlightPlan_putLine_1");
        cl.answer (FPlanConf::getErrorMsg(FPlanConf::FPErrCmdBufFull));
    }

	/* Sending notification about writting new line to queue.
    *  Attention: putLine funtion has being call from other subsystem, but current subsytem funtion is being executed.
    *  Observer treats this notify as being send from FlightPlan.
	*/
    notify (EXT_CMD);

    return ret;
}


/**
* Task of the MicroC/OS-II operating system. 
*/
void FlightPlan::task(const void* pdata)
{
	SysMonData		smd;
	FPRealData		fprd;
	bool			result = false;

	// Susbsystem initialization depends on where it was started.
	// There are two cases: start on the ground or emergency start in the air (autopilot restart)
	// 1. On the ground default flight plan is loaded an set on the first line. System remains in Stopped mode.
	// 2. In the air (restart) system restore last flight plan from the systems memory and switches to running mode.
	if(SystemStartedOnGround)
	{
		Log.msgPrintf("%sSubsystem initialized on the ground", FPlanConf::SUBS_PREFIX);
		
		// Load default flight plan.
		if(!load())
		{
			_err = _fpContainer.getLastError();
			Log.errorPrint(FPlanConf::getErrorMsg(_err));
			if(_err == FPlanConf::FPErrCantSaveToStateMem)
			{
				// Takeoff will be blocked when flight plan save to the status memory fails.
				// The only way to unlock takeoff is to restart the subsystem.
				_systemPanic = true;
				_err = FPlanConf::FPErrSystemPanic;
				Log.errorPrint(FPlanConf::getErrorMsg(_err));
			}
		}
		else
		{
			_forceValidate = true;
		}
		
	}
	else	// system started in the air.
	{
		Log.msgPrintf("%sSubsystem initialized in the air", FPlanConf::SUBS_PREFIX);

		if(true)
		{
			Log.errorPrint(FPlanConf::getErrorMsg(FPlanConf::FPErrCantLoadLastFPlan));
			
			Log.msgPrintf("%sRECOVERY MODE due to: Cannot load plan from state memory", FPlanConf::SUBS_PREFIX);
			// _recoveryMode = true;
			_missionMode = FPlanConf::MissionMode::RECOVERY;
		}
		run();	
	}

	// Main task loop.
	for (;;)
	{
        OSBase::EvtMask f = waitForAnyAspect ();				//  Wait for notification.

        // Command to read is in the queue.
        if (checkAspect (f, ExtCmdTag))
        {
        	ClassifiedLine	cl;

            while (_cmdq.cmdGet(cl))
            {
                useCmdLine(cl);
            }
        }

		if (checkAspect (f, SysMonTag))
		{
			//  Notify from the SysMon subsystem.
			if (SysMon->getSysMonData (smd))
			{
				result = true;
				if(smd.errFlag != smd.ERR_OK)
				{		
					if(!SystemNowOnGround && (smd.errFlag == smd.ERR_LOW_BATTERY_LEVEL))
					{
						result = load("lowbat");
						if(!result)
						{
							// _recoveryMode = true;
							_missionMode = FPlanConf::MissionMode::RECOVERY;
							Log.msgPrintf("%sRECOVERY MODE due to: Cannot load plan low battery flight plan", FPlanConf::SUBS_PREFIX);
							notify(FP_CHANGED);
						}				
					}
				}
				if(!result)
				{
					Log.errorPrint(FPlanConf::getErrorMsg(_err));
				}
			}
		}

		if (checkAspect(f, FPRealTag))
        {
			switch (_missionMode)
            {
			case FPlanConf::MissionMode::RECOVERY:
                Log.msgPrintf("%sFPReal notification ignored in recovery mode", FPlanConf::SUBS_PREFIX);
				break;
			case FPlanConf::MissionMode::RTL:
				if (_status == FPlanConf::Running || _status == FPlanConf::OnHold)
            {
                if (!_activeCmdGroupPtr->nextFpd())
                {
						//  When switch to next command in group fails then stop the current mode.
						Log.msgPrintf("%sSwitch mission mode failed!", FPlanConf::SUBS_PREFIX);
						_missionMode = FPlanConf::MissionMode::AUTO;
                }
                notify(FP_CHANGED);
            }
				break;
#if EMRG_LOSS_ENGINE == 1
			case FPlanConf::MissionMode::FOCUSLAND:
				if (_status == FPlanConf::Running || _status == FPlanConf::OnHold)
            {
                if (!_activeCmdGroupPtr->nextFpd())
                {
						//  When switch to next command in group fails then stop the current mode.
						Log.msgPrintf("%sSwitch mission mode failed!", FPlanConf::SUBS_PREFIX);
						_missionMode = FPlanConf::MissionMode::AUTO;
                }
                notify(FP_CHANGED);
            }
				break;
#endif
			case FPlanConf::MissionMode::PARACHUTE:
				if (_status == FPlanConf::Running || _status == FPlanConf::OnHold)
            {
                if (!_activeCmdGroupPtr->nextFpd())
                {
						//  When switch to next command in group fails then stop the current mode.
						Log.msgPrintf("%sSwitch mission mode failed!", FPlanConf::SUBS_PREFIX);
						_missionMode = FPlanConf::MissionMode::AUTO;
                }
                notify(FP_CHANGED);
            }
				break;
			case FPlanConf::MissionMode::CAMGUIDE:
				if (_status == FPlanConf::Running || _status == FPlanConf::OnHold)
            {
                if (!_activeCmdGroupPtr->nextFpd())
                {
						//  When switch to next command in group fails then stop the current mode.
						Log.msgPrintf("%sSwitch mission mode failed!", FPlanConf::SUBS_PREFIX);
						_missionMode = FPlanConf::MissionMode::AUTO;
                }
                notify(FP_CHANGED);
            }
				break;
			case FPlanConf::MissionMode::AUTO:
                //  Notify from the FPReal subsystem.
				if (FPReal->getFPRealData(fprd))
                {
					if (_status == FPlanConf::Running)
                    {
                        // In normal working mode flight plan switch to the next point.
						if (!_fpContainer.moveToNextItem())
                        {
							if(_missionMode != FPlanConf::MissionMode::RECOVERY)
							{
								Log.msgPrintf("%sRECOVERY MODE due to: Cannot load next fplan item", FPlanConf::SUBS_PREFIX);
								_missionMode = FPlanConf::MissionMode::RECOVERY;
							}
                        } 
                        // For the ground control station event about change of the current flight plan line is generated.
                        char buff[4];
                        SNPRINTF(buff, sizeof(buff), "%d", _fpContainer.getCurrentItem()); 
                        Log.eventPrint(EVT_FP_LINECHANGE, buff);
                        notify(FP_CHANGED);
                    }
					else if (_status == FPlanConf::OnHold)
                    { 
                        // If the flight plan achieve end it will be switched to the recovery mode.
						if (!run())
                            Log.errorPrint(FPlanConf::getErrorMsg(_err));
                    }
                }
                else
                    Log.errorPrint(FPlanConf::getErrorMsg(FPlanConf::FPErrCantReadFPRealData));
				break;
			default:
				break;
            }
        }
        // Notify from SystemMonitor subsystem that landing had finished.
		if (checkAspect(f, SysMonLandedTag))
        {
			// Turn off Return mode, because instruction did not send FPR_COMPLETED notify and FPlan will stay in that mode.
			if (_missionMode == FPlanConf::MissionMode::RTL)
            {
                Log.msgPrintf("%sRETURN mode cancelled (after landing)", FPlanConf::SUBS_PREFIX);
				// _returnMode = false;
				_missionMode = FPlanConf::MissionMode::AUTO;
            }
            //  Force Stop mode. (Because it is not known if SystemNowOnGround is set.)
			stop(true);
        }

        //  Internal notification about forcing to jump to specified flight plan line.
		if (checkAspect(f, InternalGotoTag))
        {
			if (!setCurrentItem(_internalGotoLineId))
            {
                //  If go to specified line failed go to next one.
				Log.msgPrintf("%sCannot go to specified line - ignored", FPlanConf::SUBS_PREFIX);
                if (!_fpContainer.moveToNextItem())
    				Log.msgPrintf("%sCannot go to next line", FPlanConf::SUBS_PREFIX);
                notify(FP_CHANGED);   
            }

			// for the base station it is generated an event (current flight plan line has changed).
			char buff[LINESIZE];
			SNPRINTF(buff, sizeof(buff), "%d", _fpContainer.getCurrentItem()); 
		    Log.eventPrint(EVT_FP_LINECHANGE, buff);
        }

        // Notify from SystemMonitor that radio has lost link.
		if (checkAspect(f, SysMonLinkTag))
        {
			//  getFPlanData() will return next instruction from _activeCmdGroup till _missionMode flag would be AUTO.
			if (SysMon->isLinkBroken() && !SystemNowOnGround && 
				_missionMode != FPlanConf::MissionMode::RTL && _status != FPlanConf::Stopped)
            {
				FPReal->clearNetLandFlag();
				FPReal->clearLandParachuteFlag();
                Log.msgPrintf("%sRETURN mode enabled (Broken Link)", FPlanConf::SUBS_PREFIX);
                // Choose group of the instruction used after radio lost link.
                _activeCmdGroupPtr = &_returnBLinkCmdGroup;
                // Set first command in the group to be executed.
                _activeCmdGroupPtr->reset();
				// _returnMode = true;
				_missionMode = FPlanConf::MissionMode::RTL;
                notify(FP_CHANGED);
            }
        }

		// Notify from SystemMonitor that radio has lost link.
		if (checkAspect(f, FPRealBaseChangedTag))
		{
			if (!SystemNowOnGround && 
				_missionMode == FPlanConf::MissionMode::PARACHUTE && 
				_status != FPlanConf::Stopped && FPReal->inPreLandPhase())
			{
				Log.msgPrintf("%sPARALANDING mode redo", FPlanConf::SUBS_PREFIX);
				// Choose group of the instruction used in parachute landing phase.
				_activeCmdGroupPtr = &_parachuteLandingCmdGroup;
				// Set first command in the group to be executed.
				_activeCmdGroupPtr->reset();
				_missionMode = FPlanConf::MissionMode::PARACHUTE;
				notify(FP_CHANGED);
			}
		}

        // Notify from System Monitor about low energy level.
		if (checkAspect(f, SysMonLowEnTag))
        {
			//  getFPlanData() will return next instruction from _activeCmdGroup till _missionMode flag would be AUTO.
			if (!SystemNowOnGround && 
				_missionMode != FPlanConf::MissionMode::RTL && _status != FPlanConf::Stopped)
            {
				FPReal->clearNetLandFlag();
				FPReal->clearLandParachuteFlag();
                Log.msgPrintf("%sRETURN mode enabled (Low Energy)", FPlanConf::SUBS_PREFIX);
                // Choose group of the instruction used after Low energy detection.
                _activeCmdGroupPtr = &_returnLowEnCmdGroup;
                // Set first command in the group to be executed.
                _activeCmdGroupPtr->reset();
				// _returnMode = true;
				_missionMode = FPlanConf::MissionMode::RTL;
                notify(FP_CHANGED);
            }
        }

        // Notify from System Monitor about GPS mulfunction.
		if (checkAspect(f, SysMonNoGpsTag))
        {
			//  getFPlanData() will return next instruction from _activeCmdGroup till _missionMode flag would be AUTO.
			if (!SystemNowOnGround && 
				_missionMode != FPlanConf::MissionMode::RTL && _status != FPlanConf::Stopped)
            {
				FPReal->clearNetLandFlag();
				FPReal->clearLandParachuteFlag();
                Log.msgPrintf("%sRETURN mode enabled (No GPS)", FPlanConf::SUBS_PREFIX);
                // Choose group of the instruction used after GPS malfunction.
                _activeCmdGroupPtr = &_returnNoGpsCmdGroup;
				// Set first command in the group to be executed.
                _activeCmdGroupPtr->reset();
				_missionMode = FPlanConf::MissionMode::RTL;
                notify(FP_CHANGED);
            }
        }

		if (checkAspect(f, FControlClimbTag))
        {
			_flightPhase = 1;

            notify(FP_CHANGED);
        }

		if (checkAspect(f, FControlCruiseTag))
        {
			_flightPhase = 2;

            notify(FP_CHANGED);
        }

		if (checkAspect(f, FControlLevelFlightTag))
        {
			_flightPhase = 3;

            notify(FP_CHANGED);
        }

#if EMRG_LOSS_ENGINE == 1
		if (checkAspect (f, SysMonLossEngineTag))
        {
			if (!SystemNowOnGround && 
				_missionMode != FPlanConf::MissionMode::RTL && _status != FPlanConf::Stopped)
			{
				Log.msgPrintf("%sRETURN mode enabled (Loss Engine)", FPlanConf::SUBS_PREFIX);
				// Choose group of the instruction used after loss engine.
				_activeCmdGroupPtr = &_lossEngineCmdGroup;
				// Set first command in the group to be executed.
				_activeCmdGroupPtr->reset();
				// _returnMode = true;
				_missionMode = FPlanConf::MissionMode::RTL;
				notify(FP_CHANGED);
			}
        }

		if (checkAspect (f, SysMonFocusLandTag))
        {
			if (!SystemNowOnGround && !_focusLandMode && _status != FPlanConf::Stopped)
			{
				Log.msgPrintf("%sFORCUS LAND mode enabled (Loss Engine)", FPlanConf::SUBS_PREFIX);
				// Choose group of the instruction used after loss engine.
				_activeCmdGroupPtr = &_focusLandCmdGroup;
				// Set first command in the group to be executed.
				_activeCmdGroupPtr->reset();
				_focusLandMode = true;
				notify(FP_CHANGED);
			}
        }
#endif

	}
}
/**
* Inserts flight plan command after line pointed by afterId param or at the beggining if afterId =0.
* If command had been but before current line, pointer would be moved to the newly inserted line.
* In any case FP_CHANGED notification is send.
*/
bool FlightPlan::insertItem(int newId, int afterId, const char* item)			
{

	bool result = _fpContainer.insert(newId, afterId, item);
	if(!result)
	{
		_err = _fpContainer.getLastError();
		return false;
	}

	bool sResult = true;

    Log.msgPrintf("%sInsert line %d after %d: %s", FPlanConf::SUBS_PREFIX, newId, afterId, item);

	if(!sResult && SystemNowOnGround)
		_systemPanic = true;

    if(!_systemPanic && false)
    {
		notify(FP_CHANGED);	
    }
	
	return sResult;
}

/** 
* Sends current flight plan to the communication chanel. 
*In case of an problems with sending function waits desired time and repeat sending.
*/
bool FlightPlan::listItems(ClassifiedLine& cl)
{
	if(!_fpContainer.printItems(cl))
	{
		_err = _fpContainer.getLastError();
		return false;
	}
	
	return true;
}

/**
* Change specified element in the flight plan.
*/
bool FlightPlan::editItem(int id, const char* item)
{
	// Edit flight plan line.
	// Saves flight plan to status memory.
	// Sends notify about changes.
	
	bool result = _fpContainer.edit(id, item);
	if(!result)
	{
		_err = _fpContainer.getLastError();
		return false;
	}
	result = true;
    Log.msgPrintf("%sEdit line %d: %s", FPlanConf::SUBS_PREFIX, id, item);
	
    // Only when editting current flight plan notify is send.
	if (!_systemPanic && 
		_missionMode != FPlanConf::MissionMode::RTL && 
		_missionMode != FPlanConf::MissionMode::PARACHUTE && 
		_fpContainer.getCurrentItem() == id)
    {
        notify(FP_CHANGED); 
    }
	return result;
}

/**
* Sends status to communication chanel.
*/
void FlightPlan::status(ClassifiedLine& cl)
{
    FPlanData f;
    char buff[LINESIZE];
    
    if (!getFPlanData (f))
    {
        Log.errorPrintf("FlightPlan_status_1");
        return;
    }

    SNPRINTF(buff, sizeof(buff), "fp: %s %s %d %s", statusAsString(f.currentStatus), f.fplanName, f.currentItem, f.command);
    cl.answer(buff);
}

/**
* Sets 'pointer' on the current flight plan element.
*/
bool FlightPlan::setCurrentItem(int id)
{
	switch (_missionMode)
	{
	case FPlanConf::MissionMode::RTL:
	case FPlanConf::MissionMode::PARACHUTE:
	case FPlanConf::MissionMode::CAMGUIDE:
		_missionMode = FPlanConf::MissionMode::AUTO;
		break;
	default:
		break;
	}
	// Clear preland/approach/descent/land/postland/abort flag (landing net)
    FPReal->clearNetLandFlag();

	FPReal->clearLandParachuteFlag();

    //  Check if FPReal subsystem allowe to break the current instruction.
    if (FPReal->isNotBreakable())
    {
        _err = FPlanConf::FPErrCantBreak;
        return false;
    }

	bool result = _fpContainer.setCurrentItem(id);
	if(!result)
	{
		_err = _fpContainer.getLastError();
		return false;
	}

    // state memory is not serviced.
	result = true;
	if(!result && SystemNowOnGround)
	{
		_systemPanic = true;
		return false;
	}
	switch (_missionMode)
	{
	case FPlanConf::MissionMode::RECOVERY:
		_missionMode = FPlanConf::MissionMode::AUTO;
		break;
	default:
		break;
	}
	if (!_systemPanic)
    {
        notify(FP_CHANGED); 
    }
	return result;
}

bool FlightPlan::runFromFirst()
{
	// Clear preland/approach/descent/land/postland/abort flag (landing net)
    FPReal->clearNetLandFlag();
	FPReal->clearLandParachuteFlag();
	
	if(_systemPanic)
	{
		_err = FPlanConf::FPErrSystemPanic;
		return false;
	}
	
	if(!_fpContainer.setAtFirstItem())
	{
		_err = FPlanConf::FPErrPlanIsEmpty;
		return false;
	}

	_status = FPlanConf::Running;
	_flightPhase = 1;
	notify(FP_CHANGED);	

	return true;
}

/**
* Switch system to the running mode.
*/
bool FlightPlan::run()
{
	// Clear preland/approach/descent/land/postland/abort flag (landing net)
    FPReal->clearNetLandFlag();
	FPReal->clearLandParachuteFlag();

	if(_systemPanic)
	{
		_err = FPlanConf::FPErrSystemPanic;
		return false;
	}

	if(_fpContainer.getCurrentItem() == ITEM_NOT_SET)
	{
		_err = FPlanConf::FPErrPlanIsEmpty;
		return false;
	}

	
	_status = FPlanConf::Running;
	notify(FP_CHANGED);	

	return true;
}

/**
* Switch system to the OnHold mode (or Stopped when UAV is on the ground)
*/
bool FlightPlan::hold()
{
	if(SystemNowOnGround)
	{
		_err = FPlanConf::FPErrCantHoldOnGround;
		return false;
	}
	else
		_status = FPlanConf::OnHold;

	notify(FP_CHANGED);
    return true;
}

/**
* Switch system to the Stopped
*/
bool FlightPlan::stop(bool forceStop)
{
// Additional parameter allwo to force stop mode in the air. (By default false)
	if(!SystemNowOnGround && !forceStop)
	{
		_err = FPlanConf::FPErrCantStopInAir;
		return false;
	}
	_status = FPlanConf::Stopped;
    // _returnMode = false;
	// _paraLandingMode = false;
	// _camGuideMode = false;
	_missionMode = FPlanConf::MissionMode::AUTO;
    if(!_systemPanic)
    {
        notify(FP_CHANGED); 
    }

	// set at first element
	if(!_fpContainer.setAtFirstItem())
	{
		_err = _fpContainer.getLastError();
		return false;
	}

    return true;
}

/**
* Delete indicated flight plan element and defragment the arrat.
* If current line was deleted pointer is being set to next line.
*/
bool FlightPlan::deleteItem(int itemId)
{
    int deletedLine = _fpContainer.getCurrentItem();
	bool result = _fpContainer.remove(itemId);
	if(!result)
	{
		_err = _fpContainer.getLastError();
		return false;
	}
	if(_fpContainer.getCurrentItem() == ITEM_NOT_SET)
	{
		if(!SystemNowOnGround)
			_status = FPlanConf::OnHold; 
		else
			_status = FPlanConf::Stopped;
	}

    result = true;
    Log.msgPrintf("%sDelete line %d", FPlanConf::SUBS_PREFIX, itemId);

    //  Sends notify only when deleting current line or lack of set line.
	if (!_systemPanic &&
			(_missionMode == FPlanConf::MissionMode::AUTO && 
			deletedLine == itemId) ||
        _fpContainer.getCurrentItem() == ITEM_NOT_SET)
    {
        notify(FP_CHANGED); 
    }
	return result;
}

/**
* Delete all flight plan elements.
*/
bool FlightPlan::deleteAllItems()
{
	if(!_fpContainer.removeAll())
	{
		_err = _fpContainer.getLastError();
		return false;
	}
	
	if(!SystemNowOnGround)
	{
		_status = FPlanConf::OnHold;
	}
	else
	{
		_status = FPlanConf::Stopped;
	}

    Log.msgPrintf("%sDelete all lines", FPlanConf::SUBS_PREFIX);

    if(!_systemPanic)
    {
        notify(FP_CHANGED); 
    }
	return true;
}

void FlightPlan::getCamGuidePoint(GpsPosition& camGuidePoint)
{
	camGuidePoint = _camGuidePoint;
}

bool FlightPlan::camGuidePointCompute(const char* command)
{
	FParser _parser;
	if (!_parser.loadLine(command))
	{
		Log.errorPrintf("fp: camGuidePointCompute");
		return false;
	}
	// "lat long" command interpretation
	if (_parser.count() != 2)
		return false;

	// Get postion of the camera guide point
	if (!TypeParser::toGpsPosition(_parser.getToken(0), _parser.getToken(1), _camGuidePoint))
	{
		Log.msgPrintf("fp_camGuidePointCompute: Unrecognized camera guide point position");
		return false;
	}
	return true;
}

/**
* Call function from the FPlan depending on received command.
* In case of an error set variable _err and return false.
*/
void FlightPlan::useCmdLine(ClassifiedLine& cl)
{
    FParser			fp;
    bool			result = true;
    bool			skipOK = false;             // needed to the status command. When set fp:ok is not been send.
    bool            answerWithSummary = true;  // Reply has to include summary (number of send lines)
	
	fp.reset();
	fp.loadLine(cl.getLine());

	if(fp.count() < 2)									// after' fp' has to be some command.
	{
		_err = FPlanConf::FPErrCmdNotRecognized;	
		result = false;
	}
	
	if(result)
	{
		char* buff = fp.getToken(1);

		if(STRICMP (buff, "ECHO") == 0)					// for test purposes
		{
			int ntok = fp.count();							//number of tokens
			if(ntok >= 3)
			{
				result = cl.answer(&cl.getLine()[fp.getTokenIndex(2)], false, answerWithSummary);
				Log.msgPrintf("%secho [%s]", FPlanConf::SUBS_PREFIX, fp.getToken(2));
				if(!result)
					_err = FPlanConf::FPErrSendFailure;
			}
			else
			{
				result = false;
				_err = FPlanConf::FPErrMissingParams;		// here are set call errors (lack of parameters etc)
			}
		}
		else if(STRICMP (buff, "LIST") == 0)			// display all flight plan elements
		{
			result = listItems(cl);                     //_err is set by the function in case of an error.
            answerWithSummary = true;
		}
		else if(STRICMP (buff, "INSERT") == 0)			// inserts new flight plan element
		{
			int ntok = fp.count();	
			if(ntok >= 5)
			{
				result = insertItem(fp.getTokenAsInt(2), fp.getTokenAsInt(3), &cl.getLine()[fp.getTokenIndex(4)]);
			}
			else
			{
				result = false;
				_err = FPlanConf::FPErrMissingParams;
			}
		}
		else if (STRICMP (buff, "look") == 0)
		{
			if (!SystemNowOnGround && 
				camGuidePointCompute(&cl.getLine()[fp.getTokenIndex(3)]) &&
				_missionMode != FPlanConf::MissionMode::PARACHUTE && 
				_status != FPlanConf::Stopped)
			{
				result = true;
				Log.msgPrintf("%sCAMERA GUIDE mode enabled", FPlanConf::SUBS_PREFIX);
				// Choose group of the instruction used in parachute landing phase.
				_activeCmdGroupPtr = &_camGuideCmdGroup;
				// Set first command in the group to be executed.
				_activeCmdGroupPtr->reset();
				_missionMode = FPlanConf::MissionMode::CAMGUIDE;
				notify(FP_CHANGED);
			}
		}
		else if(STRICMP (buff, "EDIT") == 0)			//  change indicated flight plan element.
		{
			int ntok = fp.count();	
			if(ntok >= 4)
			{
				result = editItem(fp.getTokenAsInt(2), &cl.getLine()[fp.getTokenIndex(3)]);
			}
			else
			{
				result = false;
				_err = FPlanConf::FPErrMissingParams;
			}
		}

		else if(STRICMP (buff, "START") == 0)			// Sets flight plan on the first elemnt and switch to running mode.
		{
			if (ServMan->isArm())
			{
				result = runFromFirst();
			}else
			{
				result = false;
				_err = FPlanConf::FPErrCantTakeoff;
			}
		}
		else if(STRICMP (buff, "RUN") == 0)				// switch to running mode.
		{
			result = run();		
		}
		else if(STRICMP (buff, "HOLD") == 0)			// switch to hold mode.
		{
			result = hold();		
		}
		else if(STRICMP (buff, "STOP") == 0)			// Switch to stopped mode.
		{
			result = stop();		
		}
		else if(STRICMP (buff, "DELETE") == 0)			// Delete all or indicated element.
		{
			if(fp.count() >= 3)
			{
				if(STRICMP(fp.getToken(2), "ALL") == 0)
					result = deleteAllItems();
				else
					result = deleteItem(fp.getTokenAsInt(2));		
			}
			else
			{
				result = false;
				_err = FPlanConf::FPErrMissingParams;
			}

		}
		else if(STRICMP (buff, "GOTO") == 0)		// set current flight plan element.
		{
			if(fp.count() >= 3)
			{
				if (FPReal->inParachutePhase())
				{
					result = false;
					_err = FPlanConf::FPErrCantBreak;
				}
				else
				{
					result = setCurrentItem(fp.getTokenAsInt(2));
				}				
			}
			else
			{
				result = false;			
				_err = FPlanConf::FPErrMissingParams;
			}
		}
#if LAND_MODE == SEMI_LAND || LAND_MODE == AUTO_LAND
        else if(STRICMP (buff, "LAND") == 0)     // set current flight plan element.
        {
			if (!SystemNowOnGround && 
				_missionMode != FPlanConf::MissionMode::PARACHUTE && 
				_status != FPlanConf::Stopped)
			{
				result = true;
				Log.msgPrintf("%sPARALANDING mode enabled", FPlanConf::SUBS_PREFIX);
				// Choose group of the instruction used in parachute landing phase.
				_activeCmdGroupPtr = &_parachuteLandingCmdGroup;
				// Set first command in the group to be executed.
				_activeCmdGroupPtr->reset();
				_missionMode = FPlanConf::MissionMode::PARACHUTE;
				notify(FP_CHANGED);
			}
        }
#endif
        else if(STRICMP (buff, "STATUS") == 0)          // sends current status
		{

			status(cl);
			skipOK = true;
			result = true;
		}

		else if(STRICMP (buff, "SAVE") == 0)			// Saves flight pan to external memory.
		{
			if(fp.count() >= 2)
			{
				if(fp.count() == 2)
					result = _fpContainer.save();
				else
					result = _fpContainer.save(fp.getToken(2));
			}
			else
			{
				result = false;
				_err = FPlanConf::FPErrMissingParams;
			}
		}

		// for diagnostic purposes - dispaly current data from fpd object.
		// diag update - extra actualization of the fpd object
		else if(STRICMP (buff, "DIAG") == 0)
		{
			FPlanData xfpd;
			result = getFPlanData(xfpd);
			if(result)
			{
    			char dbuff[LINESIZE];
				SNPRINTF(dbuff, sizeof(dbuff), "Current FPData:\n Status:\t\t%s\n CurrentItem:\t\t%d\n FPlanName:\t\t%s\n Command:\t\t%s\n", statusAsString(_status), xfpd.currentItem, xfpd.fplanName, xfpd.command);
				cl.answer(dbuff);
			}
		}

		else	// command not recognized
		{
			_err = FPlanConf::FPErrCmdNotRecognized;
			result = false;
		}
	}

	if(result)
	{	
		if(!skipOK)
			cl.answer(FPlanConf::getErrorMsg(FPlanConf::FPErrNoError), false, answerWithSummary);
	}
	else
	{
		// send last error to the communication chanel.
		cl.answer(FPlanConf::getErrorMsg(_err), false, answerWithSummary);
	}

}

/**
* Load default flight plan from mass storage.
*/
bool FlightPlan::load()
{
    return load(FPlanConf::DEFAULT_PLAN_NAME);
}

/**
* Load desire flight plan from mass storage.
*/
bool FlightPlan::load(const char* name)
{
    // added memory cleaning in case of an error in the function load()

    _isFPLoaded = false;

    if(!_fpContainer.load(name))
    {
        _err = _fpContainer.getLastError();

		// flight plan did not load so container has error flag set and getFPlanData would return recovery bject.
        if(SystemNowOnGround)
        {
            _err = FPlanConf::FPErrSystemPanic;
            _systemPanic = true;
            Log.errorPrintf("%sSYSTEM PANIC due to: unable to load flight plan", FPlanConf::SUBS_PREFIX);
        }
        else
        {
            Log.msgPrintf("%sRECOVERY MODE due to: unable to load flight plan", FPlanConf::SUBS_PREFIX);
            // _recoveryMode = true;
			_missionMode = FPlanConf::MissionMode::RECOVERY;
            notify(FP_CHANGED);
        }
        return false;
    }

    if(!_fpContainer.setAtFirstItem())
    {
        _err = _fpContainer.getLastError();
        return false;
    }

    if(_missionMode == FPlanConf::MissionMode::RECOVERY)
    {
        Log.msgPrintf("%sRecovery mode disabled due to: new plan loaded successfully", FPlanConf::SUBS_PREFIX);
        // _recoveryMode = false;
		_missionMode = FPlanConf::MissionMode::AUTO;
    }

    _isFPLoaded = true;
    notify(FP_CHANGED);
    return true;
}


/**  
* CommandGroup class
*/

/**  
* Constructor
*/
FlightPlan::CommandGroup::CommandGroup (void)
{
    for (int i=0; i<MAX_GROUP_ITEMS; i++)
    {
        items[i].command[0] = 0;
        items[i].currentItem = ITEM_NOT_SET;
        items[i].currentStatus = FPlanConf::Running;
        items[i].fplanName[0] = 0;
    }
    currentIndex = 0;
    maxIndex = -1;
}


/**  
*  Sets index at the beggining.
*/
void FlightPlan::CommandGroup::reset (void)
{
    currentIndex = 0;
}

/**
* Return indicator to the current FPD object in the group.
* When there is no current object returns indicator to first object.
*/
FPlanData* FlightPlan::CommandGroup::getCurrentFpd (void)
{
    if (currentIndex <= maxIndex)
        return &items[currentIndex];

    return &items[0];
}

/**
* Moves index to the next command (if currently is not on the last one).
* Return true when ok.
*/
bool FlightPlan::CommandGroup::nextFpd (void)
{
    if (currentIndex < maxIndex)
    {
        currentIndex++;
        return true;
    }

    return false;
}

/**
* Set command text
*	\param index - position (counted from 0) of the command in the array
*	\param id - identyfication number of the command
*	\param pCmd - command text
*	\param pFPName - flight plan name (fpdefault, return ... )
*	\param itemType - type of the instruction (normal, return mode, ...)
* Commands has to be set in order according to the index.
*/
bool FlightPlan::CommandGroup::setCommand (int index, int id, const char* pCmd, const char* pFPName, FPlanData::ITEM_TYPE itemType)
{
    if (index >= 0 && index < MAX_GROUP_ITEMS)
    {
        items[index].currentItem = id;
        MEMCCPY(items[index].command, pCmd, 0, FPlanConf::FPLAN_LINE_LENGTH);
	    MEMCCPY(items[index].fplanName, pFPName, 0, FPlanConf::FPLAN_NAME_LENGTH);
        items[index].itemType = itemType;
        maxIndex = index;
        return true;
    }

    return false;
}

int FlightPlan::getFlightPhase (void)
{
	return _flightPhase;
}
