/*********************************************************************/
/*                                                                   */
/* FlightController                                                  */
/*                                                                   */
/* 2008 Roman Filipowski @ Flytronic								 */
/*********************************************************************/

#include <PilotIncludes.h>

// There is no file system under NIOS - file name is predefined.
const char FlightController::CONF_FILE_NAME[]   = "fcontrolcfg1";
const char FlightController::SUBS_PREFIX[]      = "control: ";

const char FlightController::CMD_OK[]           = "control: ok";
const char FlightController::ERR_CLOAD[]        = "control: CT02 Cannot load config data";
const char FlightController::ERR_CSAVE[]        = "control: CT03 Cannot save config data";
const char FlightController::ERR_UCOMMAND[]     = "control: CT04 Unrecognized command or wrong number of parameters";
const char FlightController::ERR_BADPAR[]       = "control: CT05 Bad parameter name or value";
const char FlightController::ERR_CMDEXEC[]      = "control: CT06 Command Execute Error";
const char FlightController::MSG_NO_CONFIG[]    = "control: CT07 Config data not loaded from flash";
const char FlightController::ERR_CMD_BUF_FULL[] = "control: CT08 Command buffer full";

const float FlightController::V_REF_DEFAULT       = 70.0f;
const float FlightController::FLAPS_V_REF_DEFAULT = 80.0f;

//----------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------

FlightController::FlightController(void):
ODTObserver(), ODTSubject(), SubsystemBase(),
// initialization in case linObserver was not executed.
_fPRealTag(-1), _pStateTag(-1), _servoTag(-1), _fPRealLaunchTrigTag(-1), _sysMonTag(-1), _sysMonEngCtrlTag(-1), _extCmdTag(-1),
#if LAND_MODE == SEMI_LAND
_servManTriggerElvTag (-1), _servManTriggerAlrTag (-1), _servManTriggerElvAlrTag (-1),
#endif
#if EMRG_LOSS_ENGINE == 1
_bTestThrAlert (false),
#endif
_fPRealAttentionTag(-1), _fpRealNoAttentionTag(-1), _fPRealTkfP2Tag(-1), _fPRealParachuteTag(-1),
_cmdq(Log), _fprd(), _ss(), _conf(),
// Controllers initialization.
Alr_P               (ControllerID::ALR_P,             &_fprd.ctrlProps.AlrCProp.Alr_P,               &_ss.Alr_P,              &_conf.pp.Alr_P,              Pid::NORMAL_PID, &_modAlr),
Elv_Q               (ControllerID::ELV_Q,             &_fprd.ctrlProps.ElvCProp.Elv_Q,               &_ss.Elv_Q,              &_conf.pp.Elv_Q,              Pid::NORMAL_PID),
Rdr_R               (ControllerID::RDR_R,             &_fprd.ctrlProps.RudderCProp.Rdr_R,            &_ss.Rdr_R,              &_conf.pp.Rdr_R,              Pid::NORMAL_PID),
Rdr_Yacc            (ControllerID::RDR_YACC,          &_fprd.ctrlProps.RudderCProp.Rdr_Yacc,         &_ss.Rdr_Yacc,           &_conf.pp.Rdr_Yacc,           Pid::NORMAL_PID),
Thr_Speed           (ControllerID::THR_SPEED,         &_fprd.ctrlProps.ThrCProp.Thr_Speed,           &_ss.Thr_Speed,          &_conf.pp.Thr_Speed,          Pid::NORMAL_PID, &_modThrottle),
Thr_Alt             (ControllerID::THR_ALT,           &_fprd.ctrlProps.ThrCProp.Thr_Alt,             &_ss.Thr_Alt,            &_conf.pp.Thr_Alt,            Pid::NORMAL_PID, &_modThrottle),
Thr_Alt_2State      (ControllerID::THR_ALT_2STATE,    &_fprd.ctrlProps.Thr_Alt_2State,               &_ss.Thr_Alt_2State,     &_conf.pp.Thr_Alt_2State,     Pid::STATE_2H),
Btfly_Alt_2State    (ControllerID::BTFLY_ALT_2STATE,  &_fprd.ctrlProps.Btfly_Alt_2State,             &_ss.Btfly_Alt_2State,   &_conf.pp.Btfly_Alt_2State,   Pid::STATE_2H),
Flp_Speed           (ControllerID::FLP_SPEED,         &_fprd.ctrlProps.FlpCProp.Flp_Speed,           &_ss.Flp_Speed,          &_conf.pp.Flp_Speed,          Pid::NORMAL_PID),
Abr_GPErr           (ControllerID::ABR_GPERR,         &_fprd.ctrlProps.Abr_GPErr,                    &_ss.Abr_GPErr,          &_conf.pp.Abr_GPErr,          Pid::NORMAL_PID),
FAlr_Alr            (ControllerID::FALR_ALR,          &_fprd.ctrlProps.FAlr_Alr,                     &_ss.FAlr_Alr,           &_conf.pp.FAlr_Alr,           Pid::NORMAL_PID),
Btfly_GPath_2State  (ControllerID::BTFLY_GPATH_2STATE,&_fprd.ctrlProps.Btfly_GPath_2State,           &_ss.Btfly_GPath_2State, &_conf.pp.Btfly_GPath_2State, Pid::STATE_2H),
P_Phi               (ControllerID::P_PHI,             &_fprd.ctrlProps.PCProp.P_Phi,                 &_ss.P_Phi,              &_conf.pp.P_Phi,              Pid::NORMAL_PID),
Q_Theta             (ControllerID::Q_THETA,           &_fprd.ctrlProps.QCProp.Q_Theta,               &_ss.Q_Theta,            &_conf.pp.Q_Theta,            Pid::NORMAL_PID, NULL, &_modRefTheta),
R_Psi               (ControllerID::R_PSI,             &_fprd.ctrlProps.RCProp.R_Psi,                 &_ss.R_Psi,              &_conf.pp.R_Psi,              Pid::NORMAL_PID),
R_CoordExp          (ControllerID::R_COORDEXP,        &_fprd.ctrlProps.RCProp.R_CoordExp,            &_ss.R_CoordExp,         &_conf.pp.R_CoordExp,         Pid::NORMAL_PID),
R_Track             (ControllerID::R_TRACK,           &_fprd.ctrlProps.RCProp.R_Track,               &_ss.R_Track,            &_conf.pp.R_Track,            Pid::NORMAL_PID),
Theta_Alt           (ControllerID::THETA_ALT,         &_fprd.ctrlProps.ThetaCProp.Theta_Alt,         &_ss.Theta_Alt,          &_conf.pp.Theta_Alt,          Pid::NORMAL_PID),
Theta_Speed         (ControllerID::THETA_SPEED,       &_fprd.ctrlProps.ThetaCProp.Theta_Speed,       &_ss.Theta_Speed,        &_conf.pp.Theta_Speed,        Pid::NORMAL_PID ),
Phi_Track           (ControllerID::PHI_TRACK,         &_fprd.ctrlProps.PhiCProp.Phi_Track,           &_ss.Phi_Track,          &_conf.pp.Phi_Track,          Pid::NORMAL_PID, &_modPhi),
Phi_CTrack          (ControllerID::PHI_CTRACK,        &_fprd.ctrlProps.PhiCProp.Phi_CTrack,          &_ss.Phi_CTrack,         &_conf.pp.Phi_CTrack,         Pid::NORMAL_PID, &_modPhi),
Phi_Psi             (ControllerID::PHI_PSI,           &_fprd.ctrlProps.PhiCProp.Phi_Psi,             &_ss.Phi_Psi,            &_conf.pp.Phi_Psi,            Pid::NORMAL_PID),
Theta_GPath_2State  (ControllerID::THETA_GPATH_2STATE,&_fprd.ctrlProps.Theta_GPath_2State,           &_ss.Theta_GPath_2State, &_conf.pp.Theta_GPath_2State, Pid::STATE_2H),
Theta_VertSpeed     (ControllerID::THETA_VERTSPEED,   &_fprd.ctrlProps.ThetaCProp.Theta_VertSpeed,   &_ss.Theta_VertSpeed,    &_conf.pp.Theta_VertSpeed,    Pid::NORMAL_PID),
VertSpeed_Alt       (ControllerID::VERTSPEED_ALT,     &_fprd.ctrlProps.VertSpeedCProp.VertSpeed_Alt, &_ss.VertSpeed_Alt,      &_conf.pp.VertSpeed_Alt,      Pid::NORMAL_PID),
Track_TrackCorr     (ControllerID::TRACK_TRACKCORR,   &_fprd.ctrlProps.TrackCProp.Track_TrackCorr,   &_ss.Track_TrackCorr,    &_conf.pp.Track_TrackCorr,    Pid::NORMAL_PID),
Track_Wpt           (ControllerID::TRACK_WPT,         &_fprd.ctrlProps.TrackCProp.Track_Wpt,         &_ss.Track_Wpt,          &_conf.pp.Track_Wpt,          Pid::NORMAL_PID),
TrackCorr_Cte       (ControllerID::TRACKCORR_CTE,     &_fprd.ctrlProps.TrackCorrCProp.TrackCorr_Cte, &_ss.TrackCorr_Cte,      &_conf.pp.TrackCorr_Cte,      Pid::NORMAL_PID),
Phi_L1       		(ControllerID::PHI_L1,     	  	  &_fprd.ctrlProps.L1CProp,			 			 &_conf.l1Conf),
TECS_controller		(ControllerID::TECS, 			  &_fprd.ctrlProps.TECSCProp, 					 &_conf.tecsConf),

_modPhi      (_psd, _fprd, _conf.zeroPhiThrottle, _throttle),
_modRefTheta (_psd, _fprd, _conf.trimThetaFun, _conf.trimThetaFunNoSpd, _throttle),
_modAlr      (_psd, _fprd, _conf.alrDeadZone, _conf.alrRedMin, _conf.alrRed1Phi),
_modThrottle (_fprd.fRef.airspeed, _conf.trimThrV1, _conf.trimThrV2, _conf.trimThrT1, _conf.trimThrT2)
{
    _isConfigLoaded = false;
	_flightPhase = 0;

    // Counters
    _commCounter = _logCounter = 0;
	_thrZeroForceCounter = 0;

	// Flag support to notify phase takeoff catapult phase 2
	_bAutoZerosThrOut = false;

	_lastTrackError = 0.0f;
    _previousP = _previousQ = _previousR = 0.0f;
	_previousLogDivisor = 0;


    if (!_vSem.create("FlightController"))
    {
        Log.abort("Critical Error: FlightController_1.");
        return;
    }

	// Configuration data memory.
    _confMem = StorageFactory::createConfigMemory();
    if( _confMem == NULL)
    {
        Log.abort ("Critical Error: FlightController_2.");
        return;
    }

	_parachuteChanged = false;
	_aerodynamic_load_factor = 1.0f;
    _throttle = 0.0f;
	_measThrottle = 0.0f;
	_calcThrottle = 0.0f;

	// Variable used in the engine malfunction detection algorith.
	_forceReadFromRealFlag = false;
#if LAND_MODE == SEMI_LAND
	_bUpdateAltRef          = false;
	_bUpdateNetPosRef       = false;
	_distFromOriginalNetPos = 0.0f;
#endif
	_PicCicMode = CIC_MODE; 
	_thrAlert = false;
	_alertAltitude = 0.0f;
	_alertTime100 = 0;
	_waitingForClimb = false;
	_bSendMsgEngineFail = false;

	// Flag used in the flight with forced optimal speed.
    _optAirspeedFlag = false;

    _pars = new ParameterNames("control: ", 83);

    if (_pars == NULL)
    {
        Log.abort ("Critical Error: FlightController_1.");
        return;
    }

	// Variables names initialization for commands like: "control set/get..."
	/* 1*/_pars->insert("Alr_P.",              &_conf.pp.Alr_P);
    /* 2*/_pars->insert("Elv_Q.",              &_conf.pp.Elv_Q);
    /* 3*/_pars->insert("Rdr_R.",              &_conf.pp.Rdr_R);
    /* 4*/_pars->insert("Rdr_Yacc.",           &_conf.pp.Rdr_Yacc);
    /* 5*/_pars->insert("Thr_Speed.",          &_conf.pp.Thr_Speed);
    /* 6*/_pars->insert("Thr_Alt.",            &_conf.pp.Thr_Alt);
    /* 7*/_pars->insert("Thr_Alt_2State.",     &_conf.pp.Thr_Alt_2State);
    /* 8*/_pars->insert("Btfly_Alt_2State.",   &_conf.pp.Btfly_Alt_2State);
    /* 9*/_pars->insert("Flp_Speed.",          &_conf.pp.Flp_Speed);
    /*10*/_pars->insert("Abr_GPErr.",          &_conf.pp.Abr_GPErr);
    /*11*/_pars->insert("FAlr_Alr.",           &_conf.pp.FAlr_Alr);
    /*12*/_pars->insert("P_Phi.",              &_conf.pp.P_Phi);
    /*13*/_pars->insert("Q_Theta.",            &_conf.pp.Q_Theta);
    /*14*/_pars->insert("R_Psi.",              &_conf.pp.R_Psi);
    /*15*/_pars->insert("R_CoordExp.",         &_conf.pp.R_CoordExp);
    /*16*/_pars->insert("R_Track.",            &_conf.pp.R_Track);
    /*17*/_pars->insert("Theta_Alt.",          &_conf.pp.Theta_Alt);
    /*18*/_pars->insert("Theta_Speed.",        &_conf.pp.Theta_Speed);
    /*19*/_pars->insert("Phi_Track.",          &_conf.pp.Phi_Track);
    /*20*/_pars->insert("Phi_CTrack.",         &_conf.pp.Phi_CTrack);
	/*21*/_pars->insert("Phi_Psi.",            &_conf.pp.Phi_Psi);
    /*22*/_pars->insert("Track_TrackCorr.",    &_conf.pp.Track_TrackCorr);
    /*23*/_pars->insert("Track_Wpt.",          &_conf.pp.Track_Wpt);
    /*24*/_pars->insert("TrackCorr_Cte.",      &_conf.pp.TrackCorr_Cte);
	/*25*/_pars->insert("Btfly_GPath_2State.", &_conf.pp.Btfly_GPath_2State);
	/*26*/_pars->insert("Theta_GPath_2State.", &_conf.pp.Theta_GPath_2State);
	/*27*/_pars->insert("Theta_VertSpeed.",    &_conf.pp.Theta_VertSpeed);
	/*28*/_pars->insert("VertSpeed_Alt.",      &_conf.pp.VertSpeed_Alt);
	/*29*/_pars->insert("L1Control.",          &_conf.l1Conf);
	/*29*/_pars->insert("TECSControl.",        &_conf.tecsConf);

	/*32*/_pars->insert("commDivisor",         &_conf.commDivisor, 0, 1000);
    /*33*/_pars->insert("logDivisor",          &_conf.logDivisor,  0, 1000);
	/*34*/_pars->insert("commFormat",          &_conf.commFormat, TLM_CONTROLLER_SHORT, TLM_CONTROLLER_LONG);
	/*35*/_pars->insert("logFormat",           &_conf.logFormat,  TLM_CONTROLLER_SHORT, TLM_CONTROLLER_LONG);
	/*36*/_pars->insert("Vref",                &_conf.Vref, 0.0f, 200.0f);
    /*37*/_pars->insert("trimThetaFun",        &_conf.trimThetaFun, 0.0f, 2.0f);
    /*38*/_pars->insert("trimThetaFunNoSpd",   &_conf.trimThetaFunNoSpd, 0.0f, 2.0f);
    /*39*/_pars->insert("alrDeadZone",         &_conf.alrDeadZone, 0.0f, 1.0f);
    /*40*/_pars->insert("zeroPhiThrottle",     &_conf.zeroPhiThrottle, 0.0f, 1.0f);
	/*41*/_pars->insert("pqrFilter",		   &_conf.pqrFilter);
	/*42*/_pars->insert("trackErrFilter",      &_conf.trackErrFilter);
	/*43*/_pars->insert("flapsVref",           &_conf.flapsVref, 0.0f, 200.0f);
    /*44*/_pars->insert("minQCompV",           &_conf.minQCompV, 0.0f, 200.0f);
    /*45*/_pars->insert("maxQCompPhi",         &_conf.maxQCompPhi, 0.0f, 2.0f, 3);
    /*46*/_pars->insert("alrRedMin",           &_conf.alrRedMin, 0.0f, 1.0f, 3);
    /*47*/_pars->insert("alrRed1Phi",          &_conf.alrRed1Phi, 0.0f, 2.0f, 3);
    /*48*/_pars->insert("configAutosave",      &_conf.configAutosave);
    /*49*/_pars->insert("simIdCoeff",          &_conf.simIdCoeff, 0.0f, 300.0f);
    /*50*/_pars->insert("simIdZero",           &_conf.simIdZero, 0.0f, 10.0f);
    /*51*/_pars->insert("thrZeroForceN",       &_conf.thrZeroForceN, 0, 120);
    /*52*/_pars->insert("idFullThrottle",      &_conf.idFullThrottle, 0.0f, 300.0f);
    /*53*/_pars->insert("measThrottleMode",    &_conf.measThrottleMode);
    /*54*/_pars->insert("thrModeSwitchRef",    &_conf.thrModeSwitchRef, 0.0f, 1.0f, 3);
	/*55*/_pars->insert("trimThrV1",           &_conf.trimThrV1, 0.0f, 300.0f);
	/*56*/_pars->insert("trimThrV2",           &_conf.trimThrV2, 0.0f, 300.0f);
	/*57*/_pars->insert("trimThrT1",           &_conf.trimThrT1, 0.0f, 1.0f);
	/*58*/_pars->insert("trimThrT2",           &_conf.trimThrT2, 0.0f, 1.0f);
#if LAND_MODE == SEMI_LAND
	/*59*/_pars->insert("altStep",             &_conf.altStep,          0.0f,   10.0f);
	/*60*/_pars->insert("netPosStep",          &_conf.netPosStep,       0.0f,  100.0f);
	/*61*/_pars->insert("altRefMin",           &_conf.altRefMin,    -1000.0f, 1000.0f);
	/*62*/_pars->insert("netPosRefMax",        &_conf.netPosRefMax,     0.0f, 1000.0f);
	/*63*/_pars->insert("invDirAlr",           &_conf.invDirAlr);
	/*64*/_pars->insert("invDirElv",           &_conf.invDirElv);
	/*65*/_pars->insert("virtualDistNet",      &_conf.virtualDistNet, 0.0f, 10000.0f);
#endif
	/*66*/_pars->insert("thrAlertTime",        &_conf.thrAlertTime, 0.0f, 60.0f);
	/*67*/_pars->insert("chkAlertThr",         &_conf.chkAlertThr, 0.0f, 1.0f);
    /*68*/_pars->insert("optAirspeed",         &_conf.optAirspeed, 70.0f, 150.0f);
    /*69*/_pars->insert("optAltMrgLow",        &_conf.optAltMrgLow, -50.0f, 200.0f);
    /*70*/_pars->insert("optAltMrgHigh",       &_conf.optAltMrgHigh, -50.0f, 200.0f);
	/*71*/_pars->insert("optVg",               &_conf.bOptVg);
    /*72*/_pars->insert("optFlightPhase",      &_conf.bOptFlightPhase);

	Log.bootPrint ("OK" CRLF);                                                                  
}                                                                                               

//-----------------------------------------------------------------------
// getControlData() - share data.
//-----------------------------------------------------------------------

bool FlightController::getOutputData(OutputControls &oc)
{
    if (!_vSem.lock ())
        return false;

    //  Object copy.
	oc = _fprd.outCtrl;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

//-----------------------------------------------------------------------
// getFlightRefData() - share data.
//-----------------------------------------------------------------------

bool FlightController::getFlightRefLowLevelData(FPRealData::FlightReference::FlightReferenceLowLevel &refLowLevel)
{
    if (!_vSem.lock ())
        return false;

    //  Object copy.
	refLowLevel = _fprd.fRef.fRefLowLevel;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

/**
* If subsystem is ready for takeoff function return TRUE.
* Sends messages about errors to the cl (if cl != NULL)
* NOTE: Function is called in context of other subsystem - note on concurrency.  
*/
bool FlightController::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    bool bok = true;

    //  Check if configuration is loaded from flash.
    if (!_isConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_CONFIG);
        bok = false;
    }

    return bok;
}

/**
* Function writtes command to the queue new line and sends internal notification.
* Return true when ok, false when queue is full ( or line was not written because of other error)
*/
bool FlightController::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("FlightController_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }
    
    //  Send notify - new line was written to the queue
    //  NOTE: putLine function is beeing called by other subsytem but notify is being called from the current subsystem.
    notify (EXT_CMD);

    return ret;
}

/**
* useAttentionTrigs() - Service of a notice of an interesting maneuver
*/
void FlightController::useAttentionTrigs(bool flagOn)
{
	if (flagOn) // start of the interesting maneuver to log.
	{
		//  Force to log telemetry 
		_previousLogDivisor = _conf.logDivisor;
		if (_conf.logDivisor == 0 || _conf.logDivisor > FORCED_LOG_DIVISOR)
		{		
			_conf.logDivisor = FORCED_LOG_DIVISOR;
			Log.msgPrintf ("%sForce telemetry logging (%d).", SUBS_PREFIX, _conf.logDivisor);
		}
	}
	else // End of interesting thing.
	{
	    //  Restore telemetry.
		_conf.logDivisor = _previousLogDivisor;
	    Log.msgPrintf ("%sRestore telemetry logging (%d).", SUBS_PREFIX, _conf.logDivisor);
	}
}


/**
* GetSimId() - Function that calculate simulated current consumption.
*/
float FlightController::GetSimId(void) const
{
    return _conf.simIdCoeff * _fprd.outCtrl.fCtrl.throttle + _conf.simIdZero;
}

/**
* linkObserver() - Register object that are observed.
*/
void FlightController::linkObserver()
{
    //  Self registration. Save commands to the queue of the putLine function.
    //  putLine function is called by the other subsystem.
    _extCmdTag = registerSubj (this, EXT_CMD);

    // FPReal subsystem registration that notify about stat changes and interestin maneuver.
	_fPRealTag = registerSubj(FPReal, FPR_CHANGED);
	_fPRealAttentionTag = registerSubj(FPReal, FPR_ATTENTION);
	_fpRealNoAttentionTag = registerSubj(FPReal, FPR_NO_ATTENTION);
    // PState subsystem registration that notify about aktualization of the phisical state of UAV.
	_pStateTag = registerSubj(PState, PSTATE_CHANGED);
    // ServMan subsystem registration that notify PIC / CIC changes.
	_servoTag = registerSubj(ServMan, PIC_N_CIC_CHANGED);
	// FPReal subsytem registration that sends notify about need of reset the controllers integrals during the take off. 
	_fPRealLaunchTrigTag = registerSubj(FPReal, FPR_LAUNCHED);
	// SysMon subsytem registration that send notify about landing had finished.
    _sysMonTag = registerSubj (SysMon, SYSMON_LANDED);
	// SysMon subsytem registration that send notify about motor controller reset necessity.
	_sysMonEngCtrlTag = registerSubj (SysMon, SYSMON_ENG_CTRL);
	// FPReal subsytem registration that sends notify in Takeoff Catapult phase 2
	_fPRealTkfP2Tag = registerSubj(FPReal, FPR_TKF_P2);
	//FPReal subsytem registration that sends notify in Parachuting. only SIL
	_fPRealParachuteTag = registerSubj(FPReal, FPR_PARACHUTE);
#if LAND_MODE == SEMI_LAND
	_servManTriggerElvTag    = registerSubj (ServMan, SERVMAN_TRIGGER_ELV);
	_servManTriggerAlrTag    = registerSubj (ServMan, SERVMAN_TRIGGER_ALR);
	_servManTriggerElvAlrTag = registerSubj (ServMan, SERVMAN_TRIGGER_ELV_ALR);
#endif
}

/**
* task() - MicroC/OS-II operating system task.
*/
void FlightController::task(const void* pdata)
{
    //  Load subsystems configuration data. (Any errors will be written by the confLoad function)
	confLoad();

	for (;;)  
	{
        // Wait for notify.
        OSBase::EvtMask f = waitForAnyAspect ();

        if (checkAspect(f, _extCmdTag)) 
		{
            ClassifiedLine cmdCl;

            // Command are waiting for being read.
            while (_cmdq.cmdGet(cmdCl)) 
			{
                useCmdLine (cmdCl);
            }
        }

		if (checkAspect(f, _fPRealLaunchTrigTag)) 
		{
			// Notification from the FPReal subsystem to reset the controllers integrals during the take off. 
			ClearControllers();
		}
		
		if (checkAspect(f, _fPRealTag) || _forceReadFromRealFlag) 
		{
            _forceReadFromRealFlag = false;
			//  Notify from the FPReal subsystem.
            if (FPReal->getFPRealData (_fprd)) 
			{
				if (!setupControllers()) 
				{
					Log.errorPrint("Error: FlightController_4 (setupControllers).");
				};
			}
			else 
			{
				Log.errorPrint("Error: FlightController_3 (getFPReadlData).");
            }
        }

		if (checkAspect(f, _servoTag)) 
		{
			// Notify from the PState subsystem.	
			if (ServMan->getServoData(_servoData))
			{
				_PicCicMode = _servoData.PicnCic;
				if (_servoData.PicnCic == CIC_MODE) 
				{
					ClearControllers();
				}
			}
			else 
			{
				Log.errorPrint("Error: FlightController_7 (getServoData).");
			}
		}

		if (checkAspect(f, _pStateTag)) 
		{
			//  Notify from the PState subsystem
			if (PState->getPStateData(_psd))  
			{
				if (!computeAll()) 
				{
					Log.errorPrint("Error: FlightController_6 (computeAll).");
				};
			}
			else 
			{
				Log.errorPrint("Error: FlightController_5 (getPStateData).");
			}
		}

        //  Notice of initiation of interesting maneuver
        if (checkAspect (f, _fPRealAttentionTag))
        {
            useAttentionTrigs(true);
        }
		
        //  Notice that interesting maneuver has finished.
        if (checkAspect (f, _fpRealNoAttentionTag))
        {
            useAttentionTrigs(false);
        }

		//  Notice that parachuting, only SIL.
		if (checkAspect(f, _fPRealParachuteTag))
		{
			_parachuteChanged = true;
		}

		// Notice that in Takeoff Catapult Phase 2
		if (checkAspect(f, _fPRealTkfP2Tag))
		{
			if (!_bAutoZerosThrOut)
			{
				_bAutoZerosThrOut = true;
				Log.msgPrintf("%sZeros throttle output", SUBS_PREFIX);
			}
			else
			{
				_bAutoZerosThrOut = false;
				Log.msgPrintf("%sRelease throttle output", SUBS_PREFIX);
			}
		}

		// SysMon subsytem send notify about landing had finished.
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

		// SysMon subsytem send notify about motor controller reset necessity.
		if (checkAspect (f, _sysMonEngCtrlTag))
        {
			if (_thrZeroForceCounter == 0)
			{
				_thrZeroForceCounter = _conf.thrZeroForceN;  //  reset motor time.
				if (_thrZeroForceCounter > 0)  // set thrZeroForceN on 0 lock resetting ennging controller.
					Log.msgPrintf ("%sEngine Controller Reset", SUBS_PREFIX);
			}
        }
#if LAND_MODE == SEMI_LAND
		if (checkAspect (f, _servManTriggerElvTag))
        {
			_bUpdateAltRef = true;
        }

		if (checkAspect (f, _servManTriggerAlrTag))
        {
			_bUpdateNetPosRef = true;
        }

		if (checkAspect (f, _servManTriggerElvAlrTag))
        {
			_bUpdateAltRef    = true;
			_bUpdateNetPosRef = true;
        }
#endif
	}
}


/**
* ClearCOntrollers() -	Reset 1-st part of the controllers.
*						Call during the switch between PIC and CIC
*						Call during first phase of take off after detecting the acceleration in the X
*/

void FlightController::ClearControllers()
{
    Alr_P.state->I = 0.0f;
    Elv_Q.state->I = 0.0f;
    Rdr_R.state->I = 0.0f;
    Rdr_Yacc.state->I = 0.0f;
    Thr_Speed.state->I = 0.0f;
    Thr_Alt.state->I = 0.0f;
    Flp_Speed.state->I = 0.0f;
    Abr_GPErr.state->I = 0.0f;
    FAlr_Alr.state->I = 0.0f;
    P_Phi.state->I = 0.0f;
    Q_Theta.state->I = 0.0f;
    R_Psi.state->I = 0.0f;
    R_CoordExp.state->I = 0.0f;
    R_Track.state->I = 0.0f;
    Theta_Alt.state->I = 0.0f;
    Theta_Speed.state->I = 0.0f;
    Phi_Track.state->I = 0.0f;
    Phi_CTrack.state->I = 0.0f;
    Phi_Psi.state->I = 0.0f;
    Track_TrackCorr.state->I = 0.0f;
    Track_Wpt.state->I = 0.0f;
    TrackCorr_Cte.state->I = 0.0f;
	Theta_VertSpeed.state->I = 0.0f;
	VertSpeed_Alt.state->I = 0.0f;
}

/**
* setupControllers() - Controllers setup (reaction for the notification from the FPREal subsystem)
*/
bool FlightController::setupControllers()
{
	// Turn off controllers that are using airspeed  when there are errors during data reading.
	// We assume that the values to be controlled are being set by the FlightPlanRealizer class.
	// Turned off are valid to the next data reading.
    if (_fprd.other.doNotUseAirspeed)
    {
        _fprd.ctrlProps.ThetaCProp.Theta_Speed.enable = false;
        _fprd.ctrlProps.FlpCProp.Flp_Speed.enable = false;
        _fprd.ctrlProps.ThrCProp.Thr_Speed.enable = false;
    }
	return true;
}

/**
* computeAll() - compute all controllers adjustment and send notify CONTROLS_COMPUTED.
*/
bool FlightController::computeAll()
{
	// int [*100us]
	int t100 = _psd.time100;
	
	// auxiliary variables
	FPRealData::FlightReference* ref = &_fprd.fRef;
	FPRealData::FlightReferenceAut* refa = &_fprd.fRefAut;
	OutputControls::FlightControl* out = &_fprd.outCtrl.fCtrl;

	if (t100 < 0) 
		return false;  // FlightGear sometimes sends negative time (at the begining).

	// The generalized version of the track that give path angle to the tangent to the circle around the point
    refa->trackToWpt = GpsPosition::track(_psd.position, ref->wptTo, ref->circleRadius,
        ref->circleLeft, ref->circleMode, ref->circleModePar);
    
    // Calculate trackFromTo
#if LAND_MODE == SEMI_LAND
	if (!FPReal->inLandPhase())
    {
        _distFromOriginalNetPos = 0.0f;
    }
	if (_bUpdateAltRef)
	{
		_bUpdateAltRef = false;

		adjustAltRef(ref->altitude);
	}

	if (_bUpdateNetPosRef)
	{
		_bUpdateNetPosRef = false;

		GpsPosition newNetPos;
		adjustNetPosRef (ref->wptTo, ref->trackFromTo, newNetPos);
		ref->wptTo = newNetPos;
	}
#endif

	ref->trackFromTo = GpsPosition::track(ref->wptFrom, ref->wptTo);

#if LAND_MODE == SEMI_LAND
	if (FPReal->inLandMode())
	{
		GpsPosition::movePosition (ref->wptTo, ref->trackFromTo, _conf.virtualDistNet, ref->virtualWptTo);
		ref->trackFromTo = GpsPosition::track(ref->wptFrom, ref->virtualWptTo);
	}
#endif

    //  Error deviation from track - It is assumed that ref->wptTo point is set and lies on the route.
	if (FPReal->inAfterPreApproachPhase ())
	{
#if LAND_MODE == AUTO_LAND
		refa->crossTrackError = _psd.dgpsLonRange * sinf((ref->trackFromTo - _psd.dgpsTrack) * DEG_2_RAD);
#elif LAND_MODE == SEMI_LAND
		refa->crossTrackError = GpsPosition::crossTrackError(ref->virtualWptTo, _psd.position, ref->trackFromTo);
#endif
	}
	else
	{
		refa->crossTrackError = GpsPosition::crossTrackError(ref->wptTo, _psd.position, ref->trackFromTo);
	}    
	// Binh edit
	nav_l1->update(_psd, ref->wptFrom, ref->wptTo, 0.0f, ref->circleRadius, ref->circleLeft, static_cast<INT32>(_psd.track * 100));
	spd_hgt_ctr->update_50hz(_psd);
	// Binh end
    //  Angle Psi error.
	refa->psiError = CalcPsiError(ref->fRefLowLevel.psi, _psd.psi);

    // Factor depending on the speed
	float Ka = (_psd.airspeed > 0.1f && !_fprd.other.doNotUseAirspeed) ? 
			(_conf.Vref / _psd.airspeed) : 1.0f;

	refa->speedFactor = Ka * Ka;
	float Kg = _psd.groundspeed > 10.0f ? abs(_psd.groundspeed / _conf.Vref) : 1.0f;
	if(Kg > 1.4f)
		Kg = 1.4f;
	else if(Kg < 0.7f)
		Kg = 0.7f;
	//----------------------------
	// regulators - level 4
	//----------------------------

	TrackCorr_Cte.compute(t100, 0.0f, refa->crossTrackError, ref->crossTrackCorr, 1.0f);

	//----------------------------
    // regulators - level 3
	//----------------------------

	Track_Wpt.compute(t100, 0.0f, refa->trackToWpt, ref->track,	1.0f);
    Track_TrackCorr.compute(t100, 0.0f, GpsPosition::addTrack(ref->trackFromTo, -ref->crossTrackCorr),
        ref->track, 1.0f);
    // end
	refa->trackError = CalcTrackError(ref->track, _psd.track);

	refa->psiError = GpsPosition::subTrack(refa->psiError, ref->crossTrackCorr * DEG_2_RAD, true); // suma refa->psiError i -ref->crossTrackCorr w Radianach

    //  approach path - nie może być wcześniej, bo wymaga obliczonego ref->track
    refa->glidePath = CalcGPath (_psd.altitude, GpsPosition::distance (_psd.position, ref->wptTo), ref->track, refa->trackToWpt);

	// Filtr na TrackError
	if (_conf.trackErrFilter) 
		 refa->trackError = TrackErrorFilter.iirProcessSample(refa->trackError);

    // calculate earlier throtlle value (need in PidModifierPhi)
	_calcThrottle = out->throttle;

	Thr_Speed.compute(t100, _psd.airspeed, ref->airspeed, _calcThrottle, 1.0f);
	Thr_Alt.compute  (t100, _psd.altitude, ref->altitude, _calcThrottle, 1.0f);
	Thr_Alt_2State.compute  (t100, _psd.altitude, ref->altitude, _calcThrottle, 1.0f);

	// calculate throtlle value based on measured current
	// It is always calculated (not only in measTrhottleMode mode), because measTrhottle is always send to the FG  (to be able to simulate engine malfunction - need to set simIdCoeff = 0)
	float iUp = 0.0f;
	float iDown = 0.0f;
	float vUp = 0.0f;
	float vDown = 0.0f;
	int rsStatus = 0;
	SysMon->getBattery(iUp, vUp, iDown, vDown, rsStatus); 
	_measThrottle = (rsStatus == 0 && _conf.idFullThrottle > 0.0f) ? _measThrottle = iDown / _conf.idFullThrottle : _calcThrottle; 
	if (_measThrottle > 1.0f) _measThrottle = 1.0f;
    if (_measThrottle < 0.0f) _measThrottle = 0.0f;

	// set variable for the Theta & Psi controllers modificators.
    if (_conf.measThrottleMode && (_calcThrottle >= _conf.thrModeSwitchRef))
    {
        _throttle = _measThrottle;
    }
    else
    {
        _throttle = _calcThrottle;  
    }

	//----------------------------
    // regulators - level 2
	//----------------------------

	// If the altitude drops under the optIasLowMrg speed is switched to the optimal one.
	// Return to the desire speed is when altitude is over optIasHighMrg.
    float refAirspeed = ref->airspeed;
    if (ref->airspeed > _conf.optAirspeed)
	{
		if (FPlan->getFlightPhase() == 1)
		{
			refAirspeed = _conf.optAirspeed; // force flight with optimal speed.
		}
	}
    //--------------------------------------------------------
	Theta_Speed.compute(t100,_psd.airspeed,	refAirspeed, ref->fRefLowLevel.theta, 1.0f); 
    Theta_Alt.compute(t100, _psd.altitude, ref->altitude, ref->fRefLowLevel.theta, Ka);
#if LAND_MODE == AUTO_LAND
	if (FPReal->inAfterPreApproachPhase() && PState->getUseDGpsData())
		VertSpeed_Alt.compute(t100, _psd.dgpsAltitude, ref->altitude, ref->fRefLowLevel.f32VertSpeed, 1.0f);
	else
#endif
		VertSpeed_Alt.compute(t100, _psd.altitude, ref->altitude, ref->fRefLowLevel.f32VertSpeed, 1.0f);

	Theta_VertSpeed.compute(t100, _psd.f32VertSpeed, ref->fRefLowLevel.f32VertSpeed, ref->fRefLowLevel.theta, Ka);  
	Theta_GPath_2State.compute(t100,    refa->glidePath, ref->glidePath,ref->fRefLowLevel.theta, 1.0f);
	if (FPReal->inAfterPreApproachPhase())
	{
		Phi_Track.compute(t100, refa->trackError, 0.0f, ref->fRefLowLevel.phi, Kg);
	}
	else
	{
		Phi_Track.compute(t100, refa->trackError, 0.0f, ref->fRefLowLevel.phi, FPReal->getUseCorrTrack() ? Kg : 1.0f);
	}
    Phi_CTrack.compute(t100, refa->trackError, 0.0f, ref->fRefLowLevel.phi, 1.0f);
	Phi_Psi.compute(t100, refa->psiError, 0.0f, ref->fRefLowLevel.phi, 1.0f); 
	// Binh edit
	bool inTkfMode = FPReal->inTkfCatapultPhase();
	if (!inTkfMode)
	{
		ref->fRefLowLevel.phi = nav_l1->nav_roll_rad(_psd);
	}
	float demanded_roll = fabsf(Numbers::degrees(ref->fRefLowLevel.phi));
	if (demanded_roll > 85) {
		// limit to 85 degrees to prevent numerical errors
		demanded_roll = 85;
	}
	_aerodynamic_load_factor = 1.0f / Numbers::safe_sqrt(cosf(Numbers::radians(demanded_roll)));
	spd_hgt_ctr->update_pitch_throttle(static_cast<INT32U>(ref->altitude * 100.0f), 
										static_cast<INT32U>(ref->airspeed * 100.0f * KPH_2_MS), 
										_aerodynamic_load_factor, _psd);
	ref->fRefLowLevel.theta = Numbers::radians(static_cast<float>(spd_hgt_ctr->get_pitch_demand()) / 100.0f);
	_calcThrottle = static_cast<float>(spd_hgt_ctr->get_throttle_demand()) / 100.0f;
	// Binh end
	//  Angular speed R (dependence of R calculated from the reference values​​, instead of the current )
    refa->coordR = CalcR(_fprd.other.doNotUseAirspeed ? _conf.Vref : _psd.airspeed, ref->fRefLowLevel.phi, ref->fRefLowLevel.theta);

	if (FPReal->getTuneEnable())
	{
		int tuningLat = 0;
		int tuningLon = 0;
		int tuningSpd = 0;
		if (FPReal->getTuningControllers(tuningLat, tuningLon, tuningSpd))
		{
			if (((tuningLat == LAT_TUNING_UNUSED) || (tuningLat == LAT_TUNING_HOLD_THETA)) &&
				 (tuningLon == LON_TUNING_HOLD_PHI) &&
				 (tuningSpd == SPD_TUNING_UNUSED))
			{
				if (_psd.phi >  abs(ref->fRefLowLevel.phi)) ref->fRefLowLevel.P = -abs(ref->fRefLowLevel.P);
				if (_psd.phi < -abs(ref->fRefLowLevel.phi)) ref->fRefLowLevel.P =  abs(ref->fRefLowLevel.P);

				if (_psd.theta >  abs(ref->fRefLowLevel.theta)) ref->fRefLowLevel.Q = - abs(ref->fRefLowLevel.Q);
				if (_psd.theta < -abs(ref->fRefLowLevel.theta)) ref->fRefLowLevel.Q =   abs(ref->fRefLowLevel.Q);

				// ref->psi okre�la odchylenie (+/-)
				// ref->track okre�la k�t, wok� kt�rego oscyluje
				float xx = _psd.psi - ((ref->track > 180.0f)? (ref->track - 360.0f) : ref->track) * DEG_2_RAD;
				if (xx >  PI) xx -= 2.0f * PI;
				if (xx < -PI) xx += 2.0f * PI;
				// xx jest w przedziale <-PI..+PI>
				if (xx > abs(ref->fRefLowLevel.psi))  ref->fRefLowLevel.R = -abs(ref->fRefLowLevel.R);
				if (xx < -abs(ref->fRefLowLevel.psi)) ref->fRefLowLevel.R = abs(ref->fRefLowLevel.R);
			}
		}
	}

	//----------------------------
	// regulators - level 1
	//----------------------------

	Q_Theta.compute(t100, _psd.theta, ref->fRefLowLevel.theta, ref->fRefLowLevel.Q, 1.0f);
    
	// Elevator compensation in a curve
    // Added minimum speed limit.
    float tmpA = (_fprd.other.doNotUseAirspeed ? _conf.Vref : _psd.airspeed);
    if (tmpA < _conf.minQCompV)
    {
        tmpA = _conf.minQCompV;
    }
	if (Q_Theta.state->enabled)
    {
        //  Compensation limit to defined roll value - Phi. Powyżej tego przechylenia
        //  plane will do the slide (associated with the increase of speed. It is safe.)
        float tPhi = _psd.phi;
        if (tPhi > _conf.maxQCompPhi)
            tPhi = _conf.maxQCompPhi;
        else if (tPhi < -_conf.maxQCompPhi)
            tPhi = -_conf.maxQCompPhi;

        float tQ = tanf(tPhi) * sinf(tPhi) * G / (tmpA * KPH_2_MS);
        // Additional limitations tQ  - phi=1, v=70kph
        if (tQ > 0.66f)
            tQ = 0.66f;
        else if (tQ < 0.0f)
            tQ = 0.0f;

        Numbers::assure (tQ, 0.0f);
		ref->fRefLowLevel.Q += tQ;
	}
    // end
	
	P_Phi.compute(t100, _psd.phi, ref->fRefLowLevel.phi, ref->fRefLowLevel.P, 1.0f);
	R_Psi.compute(t100, 0.0f, refa->psiError, ref->fRefLowLevel.R, 1.0f);
    R_CoordExp.compute(t100, 0.0f, refa->coordR, ref->fRefLowLevel.R, 1.0f);
	R_Track.compute(t100, refa->trackError, 0.0f, ref->fRefLowLevel.R, 1.0f); 

	//----------------------------
	// regulators - level 1
	//----------------------------

	float filteredP = _psd.P;
	float filteredQ = _psd.Q;
	float filteredR = _psd.R;

	if (_conf.pqrFilter) {
		filteredP = NLFilter(_psd.P, &_previousP);
		filteredQ = NLFilter(_psd.Q, &_previousQ);
		filteredR = NLFilter(_psd.R, &_previousR);
	}

	// Factor is limited to 1.0 no mather on maxKas when engine is on and speed less than reference. (Elevator is in propeller stream)
	float elvSpeedFactor = refa->speedFactor;	
	if (_throttle > 0.1f) { // jeśli silnik jest włączony
		if (elvSpeedFactor > 1.0f) { // jeśli prędkość jest mniejsza
			elvSpeedFactor = 1.0f;  // ograniczenie współczynnika bez względu na maxKas
		}
	}

	if (!_vSem.lock ()) {
        return false;
	}

	// regulators that sets data for ServMan subsystem
	Elv_Q.compute(t100,			        filteredQ,		 ref->fRefLowLevel.Q,			out->elevator,			elvSpeedFactor);
	Alr_P.compute(t100,					filteredP,		 ref->fRefLowLevel.P,			out->ailerons,			refa->speedFactor);
	Rdr_R.compute(t100,					filteredR,		 ref->fRefLowLevel.R,			out->rudder,			refa->speedFactor);
	Rdr_Yacc.compute(t100,				_psd.accX,		 0.0f,				out->rudder,			refa->speedFactor);
	Btfly_Alt_2State.compute(t100,		_psd.altitude,	 ref->altitude,		out->butterfly,			1.0f);
	Btfly_GPath_2State.compute(t100,    refa->glidePath, ref->glidePath,	out->butterfly,			1.0f);
	Flp_Speed.compute(t100,				_psd.airspeed,	 _conf.flapsVref,	out->flaps,				1.0f);
	FAlr_Alr.compute(t100,				out->ailerons,	 0.0f,				out->flapsAsAilerons,	1.0f );

	if (PState->getSimLevel() > 0)
	{
		float _leftElevon = 0, _rightElevon = 0;
		elevonMixer(out->ailerons, out->elevator, _leftElevon, _rightElevon);
		out->leftElevon = _leftElevon;
		out->rightElevon = _rightElevon;
	}
	out->throttle = _calcThrottle; // it has been calculated earlier.

	if (_bAutoZerosThrOut)
	{
		//Log.msgPrintf("%s  -> Auto catapult takeoff phase 2 throtlle: %.3f (throtlle)", SUBS_PREFIX, out->throttle);
		// Zeros thottle output
		out->throttle = 0.0f;
	}

	// check if reset of engine controller is needed
	if (_thrZeroForceCounter > 0)
	{
		out->throttle = 0;
		if (--_thrZeroForceCounter == 0)
		{
			_forceReadFromRealFlag = true; // force to read data from realizer.
		}
	}

	if (!_vSem.unlock ()) {
        return false;
	}

	// Send notify to ServoManager
	out->time100 = t100;   // set time in OutputControl for ServoManager
	notify(CONTROLS_COMPUTED);
    if (_conf.bOptFlightPhase || (_flightPhase != 1))
    {
	   checkAltOpt();
    }
		
	//---------------------------------------------------------------------
	// Detect enegine malfunction
    //---------------------------------------------------------------------
	if (_conf.thrAlertTime > 0.0f) // set configuration variable thrAlertTime = 0 , turns off detection procedure.
	{
		// _PicCicMode _ThrAlert _alertAltitude _alertTime _waitingForClimb
		if (_PicCicMode == CIC_MODE)
		{
			if (_thrAlert == false) // if alert is not been set jet.
			{
				if (_throttle >= _conf.chkAlertThr)  // If engine is set to highlu values at witch plane should rise.
				{
					if (_waitingForClimb == false)
					{
						// save time and current altitude and wait for plane rise.
						_waitingForClimb = true;
						_alertAltitude = _psd.altitude; 
						_alertTime100 = _psd.time100;
					}
					else
					{
						// if waiting for plane rise and elapsed time defined in the configuration
						if ((_psd.time100 - _alertTime100) > _conf.thrAlertTime * 10000) // [*100us]
						{
							if (_psd.altitude <= _alertAltitude)
							{
								if (FPReal->isAllowEngineSignal())
								{
									//// if plane did not rised signalize alarm
									//if(!_bSendMsgEngineFail)
									//{
									//	Log.eventPrint(EVT_CTRL_THRALERT, "");
									//	Log.msgPrintf ("%sENGINE FAILURE", SUBS_PREFIX);
									//	_bSendMsgEngineFail = true;
									//}
									_thrAlert = true;
								}
							}
							else
							{
								// it is important not to lock engine failure detection in simulation.
								// Without that (in simulation) during the take off (in CIC) will be set 0.0m altitude (engine max throtlle) and in case of engine failure condition altitude < 0.0m would not be fulfilled.
								_waitingForClimb = false;
							}
						}
					}
				}
                else
                {
					// algorithm resets when there is deuction in engine drive.
					_waitingForClimb = false;
                }
			}
			else
			{
				// if there was an alarm
				if (_throttle < _conf.chkAlertThr)
				{
					// if engine has repaired -  rather for simulation purposes 
					_thrAlert = false;
					_waitingForClimb = false;
				}
			}
		}
		else
		{
			// if there is change to PIC, begining state will be set - unconditional
			_thrAlert = false;
			_waitingForClimb = false;
			_bSendMsgEngineFail = false;
		}

	}
#if EMRG_LOSS_ENGINE == 1
	if(SysMon->getLossEngine() && !_bSendMsgEngineFail)
	{
		// if plane did not rised signalize alarm
		_bSendMsgEngineFail = true;
		Log.eventPrint(EVT_CTRL_THRALERT, "");
		Log.msgPrintf ("%sENGINE FAILURE", SUBS_PREFIX);
	}
#endif
	// sends command to FG
	if (PState->getSimLevel() > 0) 
	{
		// for visualisation values under 0.1 are cutted to 0.
		if (_PicCicMode == PIC_MODE)
        {
            ServMan->setRcControlToFG (out->ailerons, true, out->rudder, true, out->elevator, true, out->throttle, true, out->flaps, true);
            _throttle = out->throttle;
        }
#if EMRG_LOSS_ENGINE == 1
		if(PState->getTestLostEngine())
		{
			_throttle = 0.0f;
			_fprd.outCtrl.fCtrl.throttle = 0.0f;
		}

		// for simulation to test loss engine by throttle alert
		if(_bTestThrAlert)
		{
			_fprd.outCtrl.fCtrl.sendToFg(SimTty, true, true, true, true, true, _parachuteChanged, 0.1f);
		}
		else
#endif
		{
			// for visualisation values under 0.1 are cutted to 0.
			_fprd.outCtrl.fCtrl.sendToFg(SimTty, true, true, true, true, true, _parachuteChanged, (_throttle > 0.1f) ? _throttle : 0.0f);
			_parachuteChanged = false;
		}
	}
	// send telemetry and save to log file.
	sendLogAndTelemetryData();

	return true;
}

void FlightController::elevonMixer(const float aileron, const float elevator, float& leftElevon, float& rightElevon)
{
	leftElevon = aileron * 0.5f + elevator * 0.5f;
	rightElevon = -aileron * 0.5f + elevator * 0.5f;
}

/**
* Function sets many parameters. In case of an error preserves unchanged values.  
*        nameValueItems - pair <name> <value> (as a text)
*/
ParameterNames::ERRCODE FlightController::setParameters (const char* nameValueItems)
{
    return _pars->setParams (nameValueItems);
}


/**
* GetParameter - gets value of the parameter.
*/
bool FlightController::getParameter (const char* pName, ClassifiedLine& cl) const
{
    bool ret = _pars->getParam(pName, cl);
    return ret;
}

/**
*  Saves subsystems configuration data.
*/
bool FlightController::confSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _confMem->saveFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!bok)
        Log.errorPrintf("FlightController_confSave_1");

    return bok;
}

/**
* Read configuration data.
*/
bool FlightController::confLoad(void)
{
     _isConfigLoaded = _confMem->loadFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!_isConfigLoaded)
    {
		// Restore default configuration in case of an error.
        _conf.setDefault ();
        Log.errorPrintf("FlightController_confLoad_1");
    }

	_previousLogDivisor = _conf.logDivisor;

    return _isConfigLoaded; 
}


/**
* useCmdLine - Command from comunication chanel interpretation.
*/
void FlightController::useCmdLine(ClassifiedLine &cl)
{										

	if (!_parser.loadLine(cl.getLine())) 
	{
		Log.errorPrint("Error: FlightController_useCmdLine_1");
		return;
	}
	// "control set <name> <value>" command interpretation
	if ((_parser.count() >= 4) && (STRICMP(_parser.getToken(1), "set") == 0)) 
	{
        ParameterNames::ERRCODE errc = setParameters (cl.getLine() + _parser.getTokenIndex(2));
        if (errc == ParameterNames::ERR_OK) 
		{
            cl.answer(CMD_OK);
		}
		else 
		{
            cl.answer(ERR_BADPAR);
		}

		return;
	}
	// "control get <name>" command interpretation
	else if ((_parser.count() == 3) && (STRICMP(_parser.getToken(1), "get") == 0)) 
	{
		if (getParameter(_parser.getToken(2), cl)) 
		{
			// sens line with summary
            cl.answer(CMD_OK, false, true);
		}
		else 
		{
			cl.answer(ERR_BADPAR);
		}
		return;
		
	}
	// "control save" command interpretation
	else if ((_parser.count() == 3) && 
		(STRICMP(_parser.getToken(1), "save") == 0) &&
		(STRICMP(_parser.getToken(2), "config") == 0)) {

		if (confSave())
            cl.answer(CMD_OK);
        else
            cl.answer(ERR_CSAVE);
        return;
		
	}
	else if ((_parser.count() == 2) && (STRICMP(_parser.getToken(1), "servo") == 0)) {
		if (computeAll())
            cl.answer(CMD_OK);
        else
            cl.answer(ERR_CLOAD);
        return;
	}

#if EMRG_LOSS_ENGINE == 1
	// Parsing the 'servo test Throttle alert' command
    else if (_parser.count() == 4 &&
        STRICMP (_parser.getToken(1), "test") == 0 &&
        STRICMP (_parser.getToken(2), "throttle") == 0 &&
		STRICMP (_parser.getToken(3), "alert") == 0)
    {
		_bTestThrAlert = !_bTestThrAlert;
		Log.errorPrintf("Test Engine Fail: %d", _bTestThrAlert);
		cl.answer(CMD_OK);
		return;
	}
#endif

	cl.answer(ERR_UCOMMAND);
}

/**
* Sends telemetry data and saves in file log.
*/
void FlightController::sendLogAndTelemetryData()
{
    // prepare data
	bool fComm  = (_conf.commDivisor != 0) && (++_commCounter >= _conf.commDivisor);
	bool fLog   = (_conf.logDivisor  != 0) && (++_logCounter  >= _conf.logDivisor);
	int id1 = _conf.logFormat;
	int id2 = _conf.commFormat;

    // sends controllers data
	if (fComm || fLog) {
		Alr_P.sendTlmData(id1, id2, fLog, fComm);
		Elv_Q.sendTlmData(id1, id2, fLog, fComm);
		Rdr_R.sendTlmData(id1, id2, fLog, fComm);
		Rdr_Yacc.sendTlmData(id1, id2, fLog, fComm);
		Thr_Speed.sendTlmData(id1, id2, fLog, fComm);
		Thr_Alt.sendTlmData(id1, id2, fLog, fComm);
		Thr_Alt_2State.sendTlmData(id1, id2, fLog, fComm);
		Btfly_Alt_2State.sendTlmData(id1, id2, fLog, fComm);
		Flp_Speed.sendTlmData(id1, id2, fLog, fComm);
		Abr_GPErr.sendTlmData(id1, id2, fLog, fComm);
		FAlr_Alr.sendTlmData(id1, id2, fLog, fComm);
		P_Phi.sendTlmData(id1, id2, fLog, fComm);
		Q_Theta.sendTlmData(id1, id2, fLog, fComm);
		R_Psi.sendTlmData(id1, id2, fLog, fComm);
		R_CoordExp.sendTlmData(id1, id2, fLog, fComm);
		R_Track.sendTlmData(id1, id2, fLog, fComm);
		Theta_Alt.sendTlmData(id1, id2, fLog, fComm);
		Theta_Speed.sendTlmData(id1, id2, fLog, fComm);
		Phi_Track.sendTlmData(id1, id2, fLog, fComm);
		Phi_CTrack.sendTlmData(id1, id2, fLog, fComm);
		Phi_Psi.sendTlmData(id1, id2, fLog, fComm);
		Track_TrackCorr.sendTlmData(id1, id2, fLog, fComm);
		Track_Wpt.sendTlmData(id1, id2, fLog, fComm);
		TrackCorr_Cte.sendTlmData(id1, id2, fLog, fComm);
		Btfly_GPath_2State.sendTlmData(id1, id2, fLog, fComm);
		Theta_GPath_2State.sendTlmData(id1, id2, fLog, fComm);
		Theta_VertSpeed.sendTlmData(id1, id2, fLog, fComm);
		VertSpeed_Alt.sendTlmData(id1, id2, fLog, fComm);
		Phi_L1.sendTlmData(id1, id2, fLog, fComm);
	}
	if (fComm) _commCounter = 0;
	if (fLog)  _logCounter  = 0;
}

//=====================================================================
// ConfigData class
//=====================================================================

FlightController::ConfigData::ConfigData(void)
{
	setDefault();
}

/**
* Sets the default values
*/
void FlightController::ConfigData::setDefault(void)
{
    commDivisor = COMM_DIVISOR_DEFAULT;
    logDivisor = LOG_DIVISOR_DEFAULT;
	logFormat = PID_FORMAT_DEFAULT;
	commFormat = PID_FORMAT_DEFAULT;
	Vref = V_REF_DEFAULT;
    trimThetaFun = 0.2f;
    trimThetaFunNoSpd = 0.4f;
	alrDeadZone = 0.0f;
    zeroPhiThrottle = 0.6f;
	flapsVref = FLAPS_V_REF_DEFAULT;
    minQCompV = 50.0f;
	pqrFilter = false;
	// turn of angular speed filter.
	trackErrFilter	= false;	// turn off Track Error filter
    configAutosave  = false;    // turn off auto save configuration after landing
    maxQCompPhi = 0.7f;
    alrRedMin = 1.0f;
    alrRed1Phi = 0.7f;
    simIdCoeff = 100.0f;
    simIdZero = 0.680f;
	thrZeroForceN = 48;
	idFullThrottle = 100.0f;
	measThrottleMode = false;
    thrModeSwitchRef = 0.1f;

    trimThrV1 = 80.0f;
	trimThrV2 = 120.0f;
	trimThrT1 = 0.5f;
	trimThrT2 = 0.8f;
#if LAND_MODE == SEMI_LAND
	altStep        =    1.0f;
	netPosStep     =    1.0f;
	altRefMin      =   50.0f;
	netPosRefMax   =   20.0f;
	invDirAlr      =   false;
	invDirElv      =   false;
	virtualDistNet = 1000.0f;
#endif

	thrAlertTime = 5.0f;
	chkAlertThr = 0.99f;
    optAirspeed = 80.0f;
    optAltMrgLow = 50.0f;
    optAltMrgHigh = 5.0f;
	bOptVg = false;
    bOptFlightPhase = false;
    
    pp.SetDefault();
	l1Conf.setDefault();
	tecsConf.setDefault();
}

/**
* Calculate TrackError
*/
float FlightController::CalcTrackError(float refTrack, float curTrack)
{
    static const float TRACK_MIN = -160.0f;
    static const float TRACK_MAX =  160.0f;

	// calculating the difference in angles <-180;+180>
    float delta = GpsPosition::subTrack (refTrack, curTrack);

	// Recognize passing by the border <-180;+180> and elimination "indecision" of the UAV during the reversing + 180 degree. 
	// "Indecision" efect results from aileron torque direction.
	// When the plane wants to reverse for example left by 180 degrees, tilting to the left ailerons causes that plane tracks turns right. (as a result  a shorter path to a given angle by turning to the right)
    if ((delta < TRACK_MIN && _lastTrackError > TRACK_MAX) ||
        (delta > TRACK_MAX && _lastTrackError < TRACK_MIN))
        delta = _lastTrackError;

    _lastTrackError = delta;
    return delta;
}

/**
* Calculate PsiError
*/
float FlightController::CalcPsiError(float refPsi, float curPsi) const
{
    // - psi is in radians. Add aditionl argument in subtrack.
    float delta = GpsPosition::subTrack (refPsi, curPsi, true);

	return delta;
}


/**
* Calculate R (turn coordination)
*/
float FlightController::CalcR(float v, float curPhi, float curTheta) const
{
	float cosTheta = cosf(curTheta);
	v = v * KPH_2_MS;  // change km/h -> m/s
	float R = (v < 0.1f)? 0.0f: sinf(curPhi) * cosTheta * cosTheta * G / v;

	return R;
}


/**
* Calculate Glide Path
*/
float FlightController::CalcGPath (float altitude, float distance, float curTrack, float trackToWpt) const
{
    //  Moving from point condition - return 0, which prevent from reducing the butterfly.
    if (fabsf (GpsPosition::subTrack (curTrack, trackToWpt)) > 90.0f)
        return 0.0f;

    float gPath = altitude > 0.0f ? distance / altitude : 0.0f;
    return gPath;
}


/**
* NL() - non linear LP filter
*/
float FlightController::NLFilter(float input, float *state) const
{
	static const float FC_FILTER_S = 0.3f;
	static const float FC_FILTER_L = 2.0f;
	
	float output = input;
	float delta = input - *state;
    float filterG = FC_FILTER_S * (1.0f + (delta / FC_FILTER_L) * (delta / FC_FILTER_L));
    float inputF = *state + delta * filterG;
    if(fabsf(inputF-input)<20.0f)
    {
 	  output = inputF;  
    }
    *state = output;
	
	return output;
}

void FlightController::checkAltOpt (void)
{
	bool inLandMode = FPReal->inLandMode();
	bool inTkfMode = FPReal->inTkfCatapultPhase();
	float optAirSpdMrgLow = _conf.optAirspeed >= _fprd.fRef.airspeed ? _fprd.fRef.airspeed : _conf.optAirspeed;

	if (_conf.optAltMrgLow < _conf.optAltMrgHigh)
	{
		float temp = _conf.optAltMrgLow;
		_conf.optAltMrgLow = _conf.optAltMrgHigh;
		_conf.optAltMrgHigh = temp;
	}

	if (!SystemNowOnGround && !inLandMode && !inTkfMode)
	{
		if(_flightPhase == 1)
		{
			// Check if plane changes to hold Altitude
			if (_psd.altitude > _fprd.fRef.altitude - _conf.optAltMrgHigh)
			{
				_flightPhase = 2;
				notify (CONTROLS_CRUISE);
				Log.msgPrintf ("%s from climb phase to cruise phase", SUBS_PREFIX);
			}
		}
		else
		{
			// Check if plane changes to climb to higher Altitude
			if (_psd.altitude < _fprd.fRef.altitude - _conf.optAltMrgLow)
			{
				_flightPhase = 1;
				notify (CONTROLS_CLIMB);
				Log.msgPrintf ("%s from cruise phase to climb phase", SUBS_PREFIX);
			}

			// Check if airspeed decreses too low
			if ((_psd.airspeed < optAirSpdMrgLow - 5.0f) && _flightPhase == 3)
			{
				_flightPhase = 2;
				notify (CONTROLS_CRUISE);
				Log.msgPrintf ("%s Cruise phase: airspeed Holding", SUBS_PREFIX);
			}
			// Check if plane changes to level flight
			else if((_psd.airspeed > _fprd.fRef.airspeed -5.0f) && _flightPhase == 2)
			{
				_flightPhase = 3;
				notify (CONTROLS_LEVELFLIGHT);
				Log.msgPrintf ("%s Cruise phase: Level Flight", SUBS_PREFIX);
			}
		}
	}
}

#if LAND_MODE == SEMI_LAND
void FlightController::adjustAltRef (float& altRef)
{
	int valueTriggerElv = ServMan->getValueTriggerElv();
	float deltaAlt = 0.0f;

	if (_conf.invDirElv)
	{
		deltaAlt =  (- (float)(valueTriggerElv) * _conf.altStep);
	}
	else
	{
		deltaAlt =  (float)(valueTriggerElv) * _conf.altStep;
	}

	altRef += deltaAlt;

	if (altRef < _conf.altRefMin)
	{
		altRef = _conf.altRefMin;
	}
}

void FlightController::adjustNetPosRef (GpsPosition& oldNetPos, float track, GpsPosition& newNetPos)
{
	int valueTriggerAlr = ServMan->getValueTriggerAlr();
	float deltaTrack = 0.0f;
	float distance = 0.0f;

	if (_conf.invDirAlr)
	{
		deltaTrack = (float)(valueTriggerAlr) * (90.0f);
	}
	else
	{
		deltaTrack = (float)(valueTriggerAlr) * (-90.0f);
	}

	distance = fabsf ((float)(valueTriggerAlr) * _conf.netPosStep);

	if (distance > _conf.netPosRefMax)
	{
		distance = _conf.netPosRefMax;
	}
	
    _distFromOriginalNetPos += distance;
    
    if (_distFromOriginalNetPos >= _conf.netPosRefMax)
    {
        _distFromOriginalNetPos = _conf.netPosRefMax;
        
        newNetPos = oldNetPos;
    }
    else
    {
        GpsPosition::movePosition (oldNetPos, GpsPosition::addTrack (track, deltaTrack), distance, newNetPos);
    }
}
#endif

/**
* Check engine fail by throttle alert
*/

bool FlightController::getThrAlert (void)
{
	return _thrAlert;
}
