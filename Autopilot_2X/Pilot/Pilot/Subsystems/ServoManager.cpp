#include <PilotIncludes.h>

const char ServoManager::CONF_FILE_NAME[] = "servmancfg";
const char ServoManager::SUBS_PREFIX[]    = "servo: ";

const char ServoManager::CMD_OK[]           = "servo: ok";
const char ServoManager::ERR_SYS[]          = "servo: SV01 System error";
const char ServoManager::ERR_CLOAD[]        = "servo: SV02 Cannot load config data";
const char ServoManager::ERR_CSAVE[]        = "servo: SV03 Cannot save config data";
const char ServoManager::ERR_UCOMMAND[]     = "servo: SV04 Unrecognized command or wrong number of parameters";
const char ServoManager::ERR_BADPAR[]       = "servo: SV05 Bad parameter name or value";
const char ServoManager::ERR_CMDEXEC[]      = "servo: SV06 Command Execute Error";
const char ServoManager::MSG_NO_CONFIG[]    = "servo: SV07 Config data not loaded from flash";
const char ServoManager::ERR_READONLY[]     = "servo: SV08 Cannot set ReadOnly parameter";
const char ServoManager::ERR_CMD_BUF_FULL[] = "servo: SV09 Command buffer full";
const char ServoManager::ERR_ALREADY_ARM[]  = "servo: SV10 System already armed";
const char ServoManager::ERR_ARM[] 		    = "servo: SV11 Mode not armable";
const char ServoManager::ERR_DISARM[] 	    = "servo: SV12 System not armed or not disarmable";
const char ServoManager::ERR_ARM_NOT_ALLOW[] = "servo: SV13 System in parachute phase, cannot arm";

const float ServoManager::SERVO_CONST = 500.0f;

ServoManager::ServoManager(void) :
    ODTObserver(), ODTSubject(), SubsystemBase(),
	// Initialization in case of no execution the 'linkObserver' method
    _fctrlTag(-1), _extCmdTag(-1),
    _cmdq(Log),
    // Counters initialization
    _commCounter(0), _logCounter(0), _commCounterRC(0), _logCounterRC(0),
#if LAND_MODE == SEMI_LAND
	_triggerCounter(0),
#endif
    _waitingForAckID(0),
    _eventAckID(55),
	_currentThrPWM(0),
	_throttleRotate(false)
#if LAND_MODE == SEMI_LAND
	, _valueTriggerElv (0)
	, _valueTriggerAlr (0)
#endif
	, _iArmDisArmTime(0)
	, _bPreArmDisArm(false)
	, _bArmSaveTime(false)
	, _bDisarmSaveTime(false)
	, _bArmed(false)
	, _spinWhenArmed(false)
	, _parachuted(false)
{
    // Formal initialization
    for (int i=0; i<SERVO_COUNT; i++)
        _servoPWM[i] = 0;

    _isConfigLoaded = false;

	// Any value, except 0 or 1. At the beginning system should send an event with information about who controls
	_prevPICnCIC = 9;

	if (!_vSem.create("servo"))
    {
        Log.abort("Critical Error: ServMan_1.");
        return;
    }

	// Configuration memory of a servomechanisms
  _confMem = StorageFactory::createConfigMemory();
  if( _confMem == NULL)
  {
      Log.abort ("Critical Error: ServMan_2.");
      return;
  }

  _pars = new ParameterNames(SUBS_PREFIX, 37);
  if (_pars == NULL)
  {
      Log.abort ("Critical Error: ServMan_3.");
      return;
  }
	
	// Initialization of a variables used by 'servo set/get..' commands
	/* 1*/_pars->insert("commDivisor",    &_conf.subsConf.commDivisor, 0, 1000);
	/* 2*/_pars->insert("logDivisor",     &_conf.subsConf.logDivisor, 0, 1000);
	/* 3*/_pars->insert("commDivisorRC",  &_conf.subsConf.commDivisorRC, 0, 1000);
	/* 4*/_pars->insert("logDivisorRC",   &_conf.subsConf.logDivisorRC, 0, 1000);
#if LAND_MODE == SEMI_LAND
	/* 5*/_pars->insert("triggerDivisor", &_conf.subsConf.triggerDivisor, 0, 1000);
	/* 6*/_pars->insert("triggerZone",    &_conf.subsConf.triggerZone, 0, 1000);
#endif
	// RF: VT1 - Variables used to calculation of an instantaneous fuel consumption
	/* 5*/_pars->insert("minFuelCons", &_conf.subsConf.minFuelCons, 0.0f, 100.0f);
	/* 6*/_pars->insert("maxFuelCons", &_conf.subsConf.maxFuelCons, 0.0f, 100.0f);
	/* 7*/_pars->insert("fuelConsT1", &_conf.subsConf.fuelConsT1, 0.0f, 1.0f);
	/* 8*/_pars->insert("fuelConsT2", &_conf.subsConf.fuelConsT2, 0.0f, 1.0f);
	/* 9*/_pars->insert("fuelConsF1", &_conf.subsConf.fuelConsF1, 0.0f, 100.0f);
	/*10*/_pars->insert("fuelConsF2", &_conf.subsConf.fuelConsF2, 0.0f, 100.0f);
	/*11*/_pars->insert("SpinWhenArmed", &_conf.subsConf.bSpinWhenArmed);
	/*12*/_pars->insert("armProcessTimeout",    &_conf.subsConf.armProcessTimeout, 0, 15000);
	/* 26*/_pars->insert("enginePwmWhenArmed",   &_conf.subsConf.enginePwmWhenArmed, 0, 1250);
	/* 27*/_pars->insert("parachutePwmTurnon", &_conf.subsConf.parachutePwmTurnon, 0, 2000);
	/* 27*/_pars->insert("parachutePwmTurnoff", &_conf.subsConf.parachutePwmTurnoff, 0, 2000);

	/*11*/_pars->insert("s1.", &_conf.servo[0]);
	/*12*/_pars->insert("s2.", &_conf.servo[1]);
	/*13*/_pars->insert("s3.", &_conf.servo[2]);
	/*14*/_pars->insert("s4.", &_conf.servo[3]);
	/*15*/_pars->insert("s5.", &_conf.servo[4]);
	/*16*/_pars->insert("s6.", &_conf.servo[5]);
	/*17*/_pars->insert("s7.", &_conf.servo[6]);
	/*18*/_pars->insert("s8.", &_conf.servo[7]);
	/*19*/_pars->insert("s9.", &_conf.servo[8]);
	/*20*/_pars->insert("s10.", &_conf.servo[9]);
	/*21*/_pars->insert("s11.", &_conf.servo[10]);
	/*22*/_pars->insert("s12.", &_conf.servo[11]);
	/*27*/_pars->insert("s13.", &_conf.servo[12]);

	Log.bootPrint ("OK" CRLF);
}

/** \name Method gets ServoManager subsystem data by copying a content of a ServManData to 'smd'
*/
bool ServoManager::getServoData(ServManData &smd)
{
    if (!_vSem.lock ())
        return false;

    // Copying of a subsystem data
    smd = _servoData;

	if (!_vSem.unlock ())
        return false;

    return true;   
}

/** \name Method returns instantaneous fuel consumption. The fuel consumtion is calculated linearly based on the characteristics
* defined by the points (T1,F1) and (T2,F2) with upper limit (maxThrCons).
* \return '0' if engine is not spinning, else value of a instantaneous fuel consumption.
*/
float ServoManager::getFuelConsumption()
{
	if (_currentThrPWM == 0)
		return 0.0f;

    float throttle = static_cast<float>(_currentThrPWM - _conf.servo[2].minValue) / static_cast<float>(_conf.servo[2].maxValue - _conf.servo[2].minValue);  // Engine throttle claculation

	if (_throttleRotate)
	{
		float f1 = _conf.subsConf.fuelConsF1;
		float f2 = _conf.subsConf.fuelConsF2;
		float t1 = _conf.subsConf.fuelConsT1;
		float t2 = _conf.subsConf.fuelConsT2;
		float fmin =  _conf.subsConf.minFuelCons;
		float fmax =  _conf.subsConf.maxFuelCons;

		if (t1 == t2)
		{
			return (f1 + f2) / 2.0f; // Calculation of an average (its better than 0 or 1)
		}
		
		// Calculation of a coefficients
		float a = (f2 - f1)/(t2 - t1);
		float b = f1 - a * t1;

		// fuel = ax + b
		float fuel = a * throttle + b;
		fuel = (fuel < fmin)? fmin : fuel;
		fuel = (fuel > fmax)? fmax : fuel;
		return fuel;
	}
	else 
	{
		if (throttle >= 0.2f)
		{
			_throttleRotate = true;
		}
		return 0.0f;
	}
}
/** \name Method to calculate the control signal from RC Receiver to Flight Gear for HIL simulation
*/
float ServoManager::calcServoValue (ServoConf servoConf, INT32U servo, float coeffs_p, float coeffs_n)
{
	float result = 0.0f;

	result = (float)((int)servo - servoConf.trim)/SERVO_CONST/servoConf.gain;
	
	if (result > 0.0f)
		result = result/coeffs_p;
	else
		result = result/coeffs_n;

	return result;
}

void ServoManager::parachuteAction(void)
{
	if (_bArmed)
	{
		//disarm engine procedure
		_bArmed = false;
		//notify to IP Core
		armEngine(_bArmed);
		notify(SERVMAN_DISARM);
		// Notify to base station
		Log.eventPrint(EVT_ENGINE_DISARM, "");
		// Reset necessary flags in disarm event
		resetParams();
	}
	_parachuted = true;
	//Send to parachute servo
	//VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO_PARACHUTE_a, _conf.subsConf.parachutePwmTurnon);
	Log.msgPrintf("%sParachute on", SUBS_PREFIX);
}

void ServoManager::setRcControlToFG (float& alr, bool bAlr, float& rdr, bool bRdr,
									float& elv, bool bElv, float& thr, bool bThr,
									float& flp, bool bFlp)
{
	if (bAlr)
		alr = ((float)_tlmRCIN.rcinData[0] - 1500) / 1000;
	if (bRdr)
		rdr = ((float)_tlmRCIN.rcinData[3] - 1500) / 500;
	if (bElv)
		elv = ((float)_tlmRCIN.rcinData[1] - 1500) / 1000;
	if (bThr)
		thr = ((float)_tlmRCIN.rcinData[2] - 1000) / 1000;
	//if (bFlp)
		//flp = calcServoValue (_conf.servo[6], _tlmRCIN.rcinData[6] & MASK_PWM_VALUE, _conf.servo[6].coeffs_p[InFlaps]   , _conf.servo[6].coeffs_n[InFlaps]);
}

/** \name Method checks readiness of a subsystem for a take-off (whether it is fully functional).
* Method sends error messages to 'cl' if it is not NULL.
* NOTE: Method is executed by other subsystem (note on concurrency)
*/
bool ServoManager::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    bool bok = true;

	// Check configuration loading from flash memory
    if (!_isConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_CONFIG);
        bok = false;
    }

    return bok;
}

/** \name Method puts new line to the commands queue and sends the notification.
* \return 'true' if putting of a new line was successful, 'false' if command queue was full or new line wasn't put due to some error.
*/
bool ServoManager::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("ServoManager_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }

	// Notification of entering the line into the queue.
	// NOTE: the 'putLine' function is executed by other subsystem but notification comes from current subsystem.
	// Observer treats it as ServoManager notification.
    notify (EXT_CMD);

    return ret;
}

/** \name Method makes specific subsystem component observable by ServoManager class (the observer)
*/
void ServoManager::linkObserver(void)
{
    // Possible errors in 'registerSubj' terminates tha application

	// Self registration which allows to insert command lines (putLine(..)) into the command queue.
	// Method 'putLine' is executed by other subsystem, but its treated by an observer like it was a ServoManagers function.
    _extCmdTag = registerSubj (this, EXT_CMD);

	// Registration of a FlightControl subsystem provides notifications of ready to use servomechanism adjustments.
	_fctrlTag = registerSubj (FControl, CONTROLS_COMPUTED);

	// FPReal subsytem registration that sends notify in Parachuting
	_fPRealParachuteTag = registerSubj(FPReal, FPR_PARACHUTE);
}

/** \name Method processes main task of a subsystem.
*/
void ServoManager::task(void* pdata)
{
    OutputControls oc;

    loadConf();
		
	for (;;)
    {
		// Waiting for notification
        OSBase::EvtMask f = waitForAnyAspect ();

        if (checkAspect (f, _fctrlTag))
        {
			// Notificaion from FlightController
			if (FControl->getOutputData(oc))
            {
				computeAll(oc);
			}            
        }

		// Command has been entered to the queue
        if (checkAspect (f, _extCmdTag))
        {
            ClassifiedLine cmdl;

            while (_cmdq.cmdGet (cmdl))
            {
                useCmdLine (cmdl);
            }            
        }

		// Notification from FlightPlanRealizer to require parachuting
		if (checkAspect(f, _fPRealParachuteTag))
		{
			if (!SystemNowOnGround)
			{
				parachuteAction();
			}
		}
		
    }
}

/** \name Method sets value of parameters specified by 'nameValueItems' in a textual representation as a pairs of <name> and corresponding <value> for a particular parameter.
* If the parameter is specified incorrectly method doesn't change its value.
*/
ParameterNames::ERRCODE ServoManager::setParameters (const char* nameValueItems)
{
    return _pars->setParams (nameValueItems);
}

/** \name Method gets value for parameter specified by 'pName'.
*/
bool ServoManager::getParameter (const char* pName, ClassifiedLine& cl) const
{
    bool ret = _pars->getParam(pName, cl);
    return ret;
}

/** \name Method parses the commands from the communication channel.
*/
void ServoManager::useCmdLine(ClassifiedLine &cl)
{										

	if (!_parser.loadLine(cl.getLine())) {
		Log.errorPrint("Error: ServMan_useCmdLine_1");
        cl.answer(ERR_SYS);
		return;
	}
	// Parsing the 'servo set <parameter_name> <value>' command
    if ((_parser.count() >= 4) && (STRICMP(_parser.getToken(1), "set") == 0)) {
    
        ParameterNames::ERRCODE errc = setParameters (cl.getLine() + _parser.getTokenIndex(2));
        if (errc == ParameterNames::ERR_OK)
            cl.answer(CMD_OK);
        else if (errc == ParameterNames::ERR_READONLY)
            cl.answer(ERR_READONLY);
        else
            cl.answer(ERR_BADPAR);

        return;
    }
	// Parsing the 'servo get <parameter_name>' command
	else if ((_parser.count() == 3) && (STRICMP(_parser.getToken(1), "get") == 0)) {
		if (getParameter(_parser.getToken(2), cl)) {
			// Send a line with summary
			cl.answer(CMD_OK, false, true);
		}
		else {
			cl.answer(ERR_BADPAR);
		}
		return;
		
	}
	// Parsing the 'servo save' command
	else if (_parser.count() == 3 &&
		STRICMP(_parser.getToken(1), "save") == 0  &&
        STRICMP (_parser.getToken(2), "config") == 0) {

		if (saveConf())
            cl.answer(CMD_OK);
        else
            cl.answer(ERR_CSAVE);
        return;
		
	}
	// Parsing the 'servo parachute on' command
	else if (_parser.count() == 3 &&
		STRICMP(_parser.getToken(1), "parachute") == 0 &&
		STRICMP(_parser.getToken(2), "on") == 0) {

		parachuteAction();
		cl.answer(CMD_OK);
		notify(SERVMAN_ERM_PARACHUTE);
		return;

	}
	
	else if (_parser.count() == 2 && 
		STRICMP(_parser.getToken(1), "mode") == 0)
	{
		char buff[LINESIZE];
		SNPRINTF(buff, sizeof(buff), "%spicncic = %d", SUBS_PREFIX, _prevPICnCIC);
		cl.answer(buff);
		return;
	}
    // Parsing the 'servo arm' command
    else if (_parser.count() == 2 &&
        STRICMP(_parser.getToken(1), "arm") == 0)
    {
      if (_bArmed)
      {
        Log.msgPrintf("%sSystem already armed", SUBS_PREFIX);
        cl.answer(ERR_ALREADY_ARM);
        return;
      }
      else
      {
		  if (FPReal->inParachutePhase())
		  {
			  cl.answer(ERR_ARM_NOT_ALLOW);
		  }
		  else
		  {
			  _bArmed = true;
			  //notify to IP Core
			  armEngine(_bArmed);
			  // Spin from this moment for 1.5 sec
			  if (_conf.subsConf.bSpinWhenArmed)
			  {
				  _spinWhenArmed = true;
				  _iArmDisArmTime = _tlmServo.time;
			  }
			  else
				  _spinWhenArmed = false;

			  // Notify to base station
			  Log.eventPrint(EVT_ENGINE_ARM, "");

			  cl.answer(CMD_OK);
		  }
      }
      return;
    }

    // Parsing the 'servo disarm' command
    else if (_parser.count() == 2 &&
        STRICMP(_parser.getToken(1), "disarm") == 0) {
        if (_bArmed && allowDisarmEngine)
        {
            _bArmed = false;
            //notify to IP Core
            armEngine(_bArmed);

            notify(SERVMAN_DISARM);

            // Notify to base station
            Log.eventPrint(EVT_ENGINE_DISARM, "");
            // Reset necessary flags in disarm event
            resetParams();

            cl.answer(CMD_OK);
        }
        else
        {
            Log.msgPrintf("%sSystem not armed or not disarmable", SUBS_PREFIX);
            cl.answer(ERR_DISARM);
        }
        return;
    }
    // Parsing the 'servo armStatus' command
	else if (_parser.count() == 2 &&
		STRICMP(_parser.getToken(1), "armStatus") == 0)
	{
		char buff[LINESIZE];
		SNPRINTF(buff, sizeof(buff), "%sArm = %d", SUBS_PREFIX, _bArmed);
		cl.answer(buff);
		return;
	}
	
	// Parsing the '"servo &T23: aileron elevator throttle rudder PSwitch QSwitch"
	else if (_parser.count() == 8 &&
        STRICMP(_parser.getToken(1), "&T23:") == 0)
    {
		_fSimRefRoll  = _fSimAil = (static_cast<float>(_parser.getTokenAsInt(2)) - 32767.0f) / 32767.0f;
		_fSimRefPitch = _fSimElv = (static_cast<float>(_parser.getTokenAsInt(3)) - 32767.0f) / 32767.0f;
		_fSimRefVz    = _fSimThr = static_cast<float>(_parser.getTokenAsInt(4)) / 65535.0f;
		_fSimRefYaw   = _fSimRdr = (static_cast<float>(_parser.getTokenAsInt(5)) - 32767.0f) / 32767.0f;

		_leftSwitch = (INT16U)_parser.getTokenAsInt(6);
		_rightSwitch = (INT16U)_parser.getTokenAsInt(7);
        return;
    }

	cl.answer(ERR_UCOMMAND);
}

/** \name Method saves the configuration.
*/
bool ServoManager::saveConf()
{
     if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _confMem->saveFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!bok)
        Log.errorPrintf("ServoManager_confSave_1");

    return bok;
}
/** \name Method reads the configuration.
*/
bool ServoManager::loadConf()
{
     _isConfigLoaded = _confMem->loadFile (CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!_isConfigLoaded)
    {
		// Default configuration restoring after an error
        _conf.setDefaults ();
        Log.errorPrintf("ServoManager_confLoad_1");
    }

    return _isConfigLoaded;
}

/** \name Method calculates the setings of each servos.
*/
void ServoManager::computeAll(const OutputControls& oc)
{
	int     sat = 0;    // PWM value for a servo within defined boundaries

	// Time for the telemetry is taken from the output controls object
	_tlmServo.time = oc.fCtrl.time100 / 10;    // Rescaling from 100 microseconds to 1 millisecond

    for(int i = 0; i < SERVO_COUNT; i++)
    {
		// 1. Values calculation based on gains for particular inputs.
		// (the_sum_of_all (input * input_gain)) * internal_gain_of_a_servo

        float sum =  (oc.fCtrl.ailerons        > 0.0f) ? oc.fCtrl.ailerons        * _conf.servo[i].coeffs_p[InAilerons]        : oc.fCtrl.ailerons        * _conf.servo[i].coeffs_n[InAilerons];
        sum += (oc.fCtrl.elevator        > 0.0f) ? oc.fCtrl.elevator        * _conf.servo[i].coeffs_p[InElevator]        : oc.fCtrl.elevator        * _conf.servo[i].coeffs_n[InElevator];
        sum += (oc.fCtrl.rudder          > 0.0f) ? oc.fCtrl.rudder          * _conf.servo[i].coeffs_p[InRudder]          : oc.fCtrl.rudder          * _conf.servo[i].coeffs_n[InRudder];
        sum += (oc.fCtrl.throttle        > 0.0f) ? oc.fCtrl.throttle        * _conf.servo[i].coeffs_p[InThrottle]        : oc.fCtrl.throttle        * _conf.servo[i].coeffs_n[InThrottle];
        sum += (oc.fCtrl.flaps           > 0.0f) ? oc.fCtrl.flaps           * _conf.servo[i].coeffs_p[InFlaps]           : oc.fCtrl.flaps           * _conf.servo[i].coeffs_n[InFlaps];
        sum += (oc.fCtrl.airbrakes       > 0.0f) ? oc.fCtrl.airbrakes       * _conf.servo[i].coeffs_p[InAirbreakes]	     : oc.fCtrl.airbrakes       * _conf.servo[i].coeffs_n[InAirbreakes];
        sum += (oc.fCtrl.containerDrop   > 0.0f) ? oc.fCtrl.containerDrop   * _conf.servo[i].coeffs_p[InContainerDrop]   : oc.fCtrl.containerDrop   * _conf.servo[i].coeffs_n[InContainerDrop];
        sum += (oc.fCtrl.butterfly       > 0.0f) ? oc.fCtrl.butterfly       * _conf.servo[i].coeffs_p[InButterfly]       : oc.fCtrl.butterfly       * _conf.servo[i].coeffs_n[InButterfly];
        sum += (oc.fCtrl.flapsAsAilerons > 0.0f) ? oc.fCtrl.flapsAsAilerons * _conf.servo[i].coeffs_p[InFlapsAsAilerons] : oc.fCtrl.flapsAsAilerons * _conf.servo[i].coeffs_n[InFlapsAsAilerons]; 
		sum += (oc.fCtrl.parachute       > 0.0f) ? oc.fCtrl.parachute       * _conf.servo[i].coeffs_p[InParachute]		 : oc.fCtrl.parachute       * _conf.servo[i].coeffs_n[InParachute];
        sum += (oc.fCtrl.customOutput[0] > 0.0f) ? oc.fCtrl.customOutput[0] * _conf.servo[i].coeffs_p[InCustom1]         : oc.fCtrl.customOutput[0] * _conf.servo[i].coeffs_n[InCustom1];
        sum += (oc.fCtrl.customOutput[1] > 0.0f) ? oc.fCtrl.customOutput[1] * _conf.servo[i].coeffs_p[InCustom2]         : oc.fCtrl.customOutput[1] * _conf.servo[i].coeffs_n[InCustom2];
        sum += (oc.fCtrl.customOutput[2] > 0.0f) ? oc.fCtrl.customOutput[2] * _conf.servo[i].coeffs_p[InCustom3]         : oc.fCtrl.customOutput[2] * _conf.servo[i].coeffs_n[InCustom3];
        sum += (oc.fCtrl.customOutput[3] > 0.0f) ? oc.fCtrl.customOutput[3] * _conf.servo[i].coeffs_p[InCustom4]         : oc.fCtrl.customOutput[3] * _conf.servo[i].coeffs_n[InCustom4];
		
		// 2. Internal gain
		sum = sum * _conf.servo[i].gain;

		// Data for telemetry
		_tlmServo.servoData[i] = static_cast<INT16S>(sum * 1000.0f);

		// 3. Trimming and conversion to the PWM values of a timer (float to int)
		int trimmed = _conf.servo[i].trim + static_cast<int>(SERVO_CONST * sum);
		
		// 4. Cut range
		if(trimmed > _conf.servo[i].maxValue)
			sat = _conf.servo[i].maxValue;
		else if(trimmed < _conf.servo[i].minValue)
			sat = _conf.servo[i].minValue;
		else
			sat = trimmed;

		// The final value
		_servoPWM[i] = sat;
	}

	// When switching form PIC to CIC executed is only a 'notify()' function, while settings the servos are skipped.
	readRCIN(_tlmRCIN, _tRcStatus);

#if LAND_MODE == SEMI_LAND
	/**************************************************************************************************************/
	if (FPReal->inLandPhase())
	{
		if (_conf.subsConf.triggerDivisor != 0 && (++_triggerCounter % _conf.subsConf.triggerDivisor == 0))
		{
			_triggerCounter = 0;

			int triggerZone = _conf.subsConf.triggerZone;

			_valueTriggerElv = calcValueTriggerElv(triggerZone);
			_valueTriggerAlr = calcValueTriggerAlr(triggerZone);

			if (_valueTriggerElv != 0)
			{
				if (_valueTriggerAlr != 0)
				{
					notify (SERVMAN_TRIGGER_ELV_ALR);
				}
				else
				{
					notify (SERVMAN_TRIGGER_ELV);
				}
			}
			else
			{
				if (_valueTriggerAlr != 0)
				{
					notify (SERVMAN_TRIGGER_ALR);
				}
			}
		}
	}
	/**************************************************************************************************************/
#endif
    // Check ARM
#if PILOT_TARGET == PT_HARDWARE
    bool rdrCondition = (_tlmRCIN.rcinData[3] > 1800);											//RC_A
    bool thrCondition = (_tlmRCIN.rcinData[4] < 1200) && (_tlmRCIN.rcinData[1] > 1000);			//RC_B
    bool ailCondition = (_tlmRCIN.rcinData[6] > 1800);											//RC_C
    bool elvCondition = (_tlmRCIN.rcinData[7] < 1200) && (_tlmRCIN.rcinData[7] > 1000);			//RC_D
    if(elvCondition && ailCondition && rdrCondition && thrCondition && !_bArmed)
#else
    if((_fSimAil < -0.75f) && (_fSimElv > 0.75) && (_fSimThr < 0.2f) && (_fSimRdr > 0.75f) && !_bArmed && !FPReal->inParachutePhase())
#endif
    {
        if(!_bArmSaveTime)
        {
			_bArmSaveTime = true;
			_iArmDisArmTime = _tlmServo.time;
        }
        if(_tlmServo.time - _iArmDisArmTime > 2000 && !_bPreArmDisArm && _bArmSaveTime)
        {
             _bPreArmDisArm = true;
             _iArmDisArmTime = _tlmServo.time;
        }
    }
    else
    {
        _bArmSaveTime = false;
    }

    if(_bPreArmDisArm && !_bArmed)
    {
        if((_tlmServo.time - _iArmDisArmTime) > (unsigned)_conf.subsConf.armProcessTimeout)
        {
            _bArmSaveTime = false;
            _bArmed = true;
            //notify to IP Core
            armEngine(_bArmed);
            _bPreArmDisArm = false;

            // Send event to base station
            Log.eventPrint(EVT_ENGINE_ARM, "");
            Log.msgPrintf ("%Armed by RC", SUBS_PREFIX);

            if (_conf.subsConf.bSpinWhenArmed)
            {
                // Spin for 1.5 sec from this moment
                _iArmDisArmTime = _tlmServo.time;
                _spinWhenArmed = true;
            }
            else
                _spinWhenArmed = false;
         }
    }

    // Check Disarm (Only check in CIC mode)
#if PILOT_TARGET == PT_HARDWARE
    rdrCondition = (_tlmRCIN.rcinData[3] < 1200) && (_tlmRCIN.rcinData[3] > 1000);			//RC_A
    thrCondition = (_tlmRCIN.rcinData[4] < 1200) && (_tlmRCIN.rcinData[1] > 1000);			//RC_B
    ailCondition = (_tlmRCIN.rcinData[6] < 1200) && (_tlmRCIN.rcinData[6] > 1000);			//RC_C
    elvCondition = (_tlmRCIN.rcinData[7] < 1200) && (_tlmRCIN.rcinData[7] > 1000);			//RC_D
    if(elvCondition && ailCondition && rdrCondition && thrCondition && _bArmed)
#else
	if((_fSimAil > 0.75f) && (_fSimElv > 0.75f) && (_fSimThr < 0.2f) && (_fSimRdr < -0.75f) && _bArmed)
#endif
    {
		if (!_bDisarmSaveTime)
		{
			_bDisarmSaveTime = true;
			_iArmDisArmTime = _tlmServo.time;
		}

		else
		{
			if ((_tlmServo.time - _iArmDisArmTime) > 1000)
			{
				_bDisarmSaveTime = false;
				_iArmDisArmTime = 0;
				if(allowDisarmEngine)
				{
					_bArmed = false;
			        //notify to IP Core
			        armEngine(_bArmed);
					Log.msgPrintf ("%sDisarmed by RC", SUBS_PREFIX);
					// Send event to base station
					Log.eventPrint(EVT_ENGINE_DISARM, "");

					// Notify to other system
					notify(SERVMAN_DISARM);
				}else
		        {
		            Log.msgPrintf("%sSystem not disarmable by RC", SUBS_PREFIX);
		        }
				resetParams();
			}
		}
    }
    else
    {
        _bDisarmSaveTime = false;
    }
	// RF: VT1
	_currentThrPWM = (_tlmRCIN.rcinPicnCic == 1)? (_tlmRCIN.rcinData[2] & MASK_PWM_VALUE)  : _servoPWM[2];

 	if(_tlmRCIN.rcinPicnCic != _prevPICnCIC)
	{
		_eventPicnCicChange.time = oc.fCtrl.time100 / 10;
		_eventPicnCicChange.PicnCic = _tlmRCIN.rcinPicnCic;
		fireEventPicnCicChange(_eventPicnCicChange);

		if(_vSem.lock())
		{
			_servoData.time = _eventPicnCicChange.time;
			_servoData.PicnCic = _eventPicnCicChange.PicnCic;
			_vSem.unlock();
			notify(PIC_N_CIC_CHANGED);  // The semaphore must be released before the notification (to prevent an deadlock)
		}
	}

    else
    {
		// Servos are set when the control wasn't change from PIC to CIC.
        setServos();
    }

    _tlmRCIN.time = oc.fCtrl.time100 / 10;  // Rescaling to the milliseconds
    sendRCINTlmData(true, true);
	
	_prevPICnCIC = _tlmRCIN.rcinPicnCic;

	sendServoTlmData(true, true);
}

void ServoManager::fireEventPicnCicChange(const EventPicCicChange& e)
{
	char buf[LINESIZE - 10];	
	if (!Base64::encode(&e, sizeof(EventPicCicChange), buf, sizeof(buf))) 
		{
			Log.errorPrint("Error: ServMan_fireEventPicnCicChange_1");
			return;
		}
		else
		{
			char buf2[LINESIZE];
			SNPRINTF(buf2, sizeof(buf2), "%d %s", ++_eventAckID, buf);
			
			Log.eventPrint(EVT_PIC_CIC, buf2);    
			_waitingForAckID = _eventAckID;
		}
}

/** \name Method sends data to the communication channel.
*/
void ServoManager::sendServoTlmData(bool fLog, bool fComm)
{
	bool log = fLog && _conf.subsConf.logDivisor != 0 && ++_logCounter / _conf.subsConf.logDivisor != 0;
	bool comm = fComm && _conf.subsConf.commDivisor != 0 && ++_commCounter / _conf.subsConf.commDivisor != 0;

	// Sending data to the log
    if (log || comm)
    {
        char buf[LINESIZE];

		if (!Base64::encode(&_tlmServo, sizeof(_tlmServo), buf, sizeof(buf))) 
		{
			Log.errorPrint("Error: ServMan_sendServoTlmData_1");
			return;
		}
		else
		{
			Log.tlmPrint(TLM_SERVO, buf, log, comm);            
			if(log)
				_logCounter = 0;
			if(comm)
				_commCounter = 0;
		}
    }
}

/** \name Method sends the data to the communication channel.
*/
void ServoManager::sendRCINTlmData(bool fLog, bool fComm)
{
	bool log = fLog && _conf.subsConf.logDivisorRC != 0 && ++_logCounterRC / _conf.subsConf.logDivisorRC != 0;
	bool comm = fComm && _conf.subsConf.commDivisorRC != 0 && ++_commCounterRC / _conf.subsConf.commDivisorRC != 0;
	
	// Sending data to the log
    if (log || comm)
    {
        char buf[LINESIZE];

		if (!Base64::encode(&_tlmRCIN, sizeof(_tlmRCIN), buf, sizeof(buf))) 
		{
			Log.errorPrint("Error: ServMan_sendRCINTlmData_1");
			return;
		}
		else
		{
			Log.tlmPrint(TLM_RC_IN, buf, log, comm);            
			if(log)
				_logCounterRC = 0;
			if(comm)
				_commCounterRC = 0;
		}
    }
}

/** \name Method reads current values of a servos / RC channels.
*/
void ServoManager::readRCIN(TlmDataRCIN& t, RCStatus& s) const
{
	if (PState->getSimLevel() > 0)
	{
		t.rcinData[0] = (INT16U)(_fSimRefRoll * 500 + 1500);
		t.rcinData[1] = (INT16U)(_fSimRefPitch * 500 + 1500);
		t.rcinData[2] = (INT16U)(_fSimRefVz * 1000 + 1000);
		t.rcinData[3] = (INT16U)(_fSimRefYaw * 500 + 1500);
		t.rcinPicnCic = _leftSwitch;
		t.rcArmStatus = _bArmed;
		return;
	}
	
	t.rcinControl = 0;
	t.rcinData[0] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC1_a);
	t.rcinData[1] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC2_a);
	t.rcinData[2] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC3_a);
	t.rcinData[3] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC4_a);
	t.rcinData[4] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC5_a);
	t.rcinData[5] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC6_a);
	t.rcinData[6] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC7_a);
	t.rcinData[7] = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC8_a);
	t.rcinPicnCic = (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCmode_a) & MASK_PIC_N_CIC;
	// if (PState->getSimLevel() > 0)
	// {
	// 	t.rcinPicnCic = 0;
	// }
	t.rcArmStatus = _bArmed;
	s.DCH1 = (INT8U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCDCH1_a) & 0x1;
	s.DCH2 = (INT8U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCDCH2_a) & 0x1;
	s.frameLost = (INT8U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCFrLost_a) & 0x1;
	s.totalFrameError = (INT32U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCFrErr_a);
	s.failsafeActivated = (INT8U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCFSActv_a) & 0x1;
	s.lastGoodFrame = (INT32U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RClastGoodFr_a);
	s.NoRcInput = (INT8U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RCNoRCInput_a) & 0x1;
}


/** \name Method sets for each servos a value of the last calculation.
*/
void ServoManager::setServos()
{
	unsigned short outPWM[SERVO_COUNT];

	for (int i = 0; i < SERVO_COUNT; i++)
	{
		outPWM[i] = 0;
	}
	// Values calculated by the regulators
	if (FPReal->getTuneEnable())
	{
		int tuningLat = 0;
		int tuningLon = 0;
		int tuningSpd = 0;
		if (FPReal->getTuningControllers(tuningLat, tuningLon, tuningSpd))
		{
			if ((tuningLat == LAT_TUNING_UNUSED) && (tuningLon == LON_TUNING_HOLD_PHI) && (tuningSpd == SPD_TUNING_UNUSED))
			{
				// AP only controls Roll channels
				_servoIO.servoRegs->SERVO1_c = _tlmRCIN.rcinData[0] & MASK_PWM_VALUE;           // Elevator
				_servoIO.servoRegs->SERVO2_c = _tlmRCIN.rcinData[1] & MASK_PWM_VALUE;           // Rudder
				_servoIO.servoRegs->SERVO3_c = _tlmRCIN.rcinData[2] & MASK_PWM_VALUE;           // Throttle
				_servoIO.servoRegs->SERVO4_c = static_cast<unsigned short>(_servoPWM[3]);       // Aileron
				_servoIO.servoRegs->SERVO5_c = static_cast<unsigned short>(_servoPWM[4]);       // Aileron  
				_servoIO.servoRegs->SERVO6_c = _tlmRCIN.rcinData[5] & MASK_PWM_VALUE;           // Elevator2
				_servoIO.servoRegs->SERVO7_c = _tlmRCIN.rcinData[6] & MASK_PWM_VALUE;           // Flap (Butterfly)
				_servoIO.servoRegs->SERVO8_c = _tlmRCIN.rcinData[7] & MASK_PWM_VALUE;           // Flap (Butterfly)
				_servoIO.servoRegs->SERVO9_c = _tlmRCIN.rcinData[8] & MASK_PWM_VALUE;           // Pantograph

			}
			else if ((tuningLat == LAT_TUNING_HOLD_THETA) && (tuningLon == LON_TUNING_HOLD_PHI) && (tuningSpd == SPD_TUNING_UNUSED))
			{
				// AP only controls Pitch and Roll channels
				_servoIO.servoRegs->SERVO1_c = static_cast<unsigned short>(_servoPWM[0]);       // Elevator
				_servoIO.servoRegs->SERVO2_c = _tlmRCIN.rcinData[1] & MASK_PWM_VALUE;           // Rudder
				_servoIO.servoRegs->SERVO3_c = _tlmRCIN.rcinData[2] & MASK_PWM_VALUE;           // Throttle
				_servoIO.servoRegs->SERVO4_c = static_cast<unsigned short>(_servoPWM[3]);       // Aileron
				_servoIO.servoRegs->SERVO5_c = static_cast<unsigned short>(_servoPWM[4]);       // Aileron  
				_servoIO.servoRegs->SERVO6_c = static_cast<unsigned short>(_servoPWM[5]);       // Elevator2
				_servoIO.servoRegs->SERVO7_c = _tlmRCIN.rcinData[6] & MASK_PWM_VALUE;           // Flap (Butterfly)
				_servoIO.servoRegs->SERVO8_c = _tlmRCIN.rcinData[7] & MASK_PWM_VALUE;           // Flap (Butterfly)
				_servoIO.servoRegs->SERVO9_c = _tlmRCIN.rcinData[8] & MASK_PWM_VALUE;           // Pantograph
			}
			else if ((tuningLat == LAT_TUNING_UNUSED) && (tuningLon == LON_TUNING_HOLD_PHI) && (tuningSpd == SPD_TUNING_HOLD_SPD))
			{
				// pilot controls the elevator to hold altitude
				_servoIO.servoRegs->SERVO1_c = _tlmRCIN.rcinData[0] & MASK_PWM_VALUE;           // Elevator
				_servoIO.servoRegs->SERVO2_c = static_cast<unsigned short>(_servoPWM[1]);       // Rudder
				_servoIO.servoRegs->SERVO3_c = static_cast<unsigned short>(_servoPWM[2]);       // Throttle
				_servoIO.servoRegs->SERVO4_c = static_cast<unsigned short>(_servoPWM[3]);       // Aileron
				_servoIO.servoRegs->SERVO5_c = static_cast<unsigned short>(_servoPWM[4]);       // Aileron  
				_servoIO.servoRegs->SERVO6_c = _tlmRCIN.rcinData[5] & MASK_PWM_VALUE;           // Elevator2
				_servoIO.servoRegs->SERVO7_c = static_cast<unsigned short>(_servoPWM[6]);       // Flap (Butterfly)
				_servoIO.servoRegs->SERVO8_c = static_cast<unsigned short>(_servoPWM[7]);       // Flap (Butterfly)
				_servoIO.servoRegs->SERVO9_c = static_cast<unsigned short>(_servoPWM[8]);       // Pantograph
			}
			else
			{
				// Plane control surfaces
				outPWM[0] = static_cast<unsigned short>(_servoPWM[0]);       // Left Elevon
				outPWM[5] = static_cast<unsigned short>(_servoPWM[5]);       // Right Elevon

				if(_bArmed)
				{
		            // The system is just armed, spin for 1.5 sec
		            if (_spinWhenArmed)
		            {
		                if((_tlmServo.time - _iArmDisArmTime) > 1500)
		                {
		                    _spinWhenArmed = false;
		                    _iArmDisArmTime = 0;
		                }
		                else
		                {
		                	outPWM[2] = static_cast<unsigned short>(_conf.subsConf.enginePwmWhenArmed);
		                }
		            }
		            // Output normal PWM as in armed
		            else
		            {
		            	outPWM[2] = static_cast<unsigned short>(_servoPWM[2]);       // Throttle
		            }
				}else
				{
					outPWM[2] = 0;											 //
				}
				VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO1_a, outPWM[0]);		//Left Elevon
				VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO3_a, outPWM[2]);		//Throttle
				VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO6_a, outPWM[5]);		//Right Elevon
			}
		}
	}
	else
	{
		// Plane control surfaces
		outPWM[0] = static_cast<unsigned short>(_servoPWM[0]);       // Left Elevon
		outPWM[5] = static_cast<unsigned short>(_servoPWM[5]);       // Right Elevon

		if(_bArmed)
		{
            // The system is just armed, spin for 1.5 sec
            if (_spinWhenArmed)
            {
                if((_tlmServo.time - _iArmDisArmTime) > 1500)
                {
                    _spinWhenArmed = false;
                    _iArmDisArmTime = 0;
                }
                else
                {
                	outPWM[2] = static_cast<unsigned short>(_conf.subsConf.enginePwmWhenArmed);
                }
            }
            // Output normal PWM as in armed
            else
            {
            	outPWM[2] = static_cast<unsigned short>(_servoPWM[2]);       // Throttle
            }
		}else
		{
			outPWM[2] = 0;											 //
		}

		VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO1_a, outPWM[0]);		//Left Elevon
		VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO3_a, outPWM[2]);		//Throttle
		VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO6_a, outPWM[5]);		//Right Elevon

		if(_parachuted)
		{
			//Send to parachute servo
			VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO_PARACHUTE_a, _conf.subsConf.parachutePwmTurnon);
		}else
		{
			//Send to parachute servo
			VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO_PARACHUTE_a, _conf.subsConf.parachutePwmTurnoff);
		}
		//INT16U parachute_rc =  (INT16U)VT_SBUS_CONTROLLER_mReadReg(FLYEYE_RC_BASE, RC10_a);
		//VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO_PARACHUTE_a, parachute_rc);
	}

	_servoIO.servoRegs->SERVO_CONTROL_c = MASK_TRIG_MAN;
}

ServoManager::ConfigData::ConfigData(void)
{
    setDefault ();
}

/** \name Method sets default values of configuration data.
*/
void ServoManager::ConfigData::setDefault(void)
{
    commDivisor   = COMM_DIVISOR_DEFAULT;
    logDivisor    = LOG_DIVISOR_DEFAULT;
    commDivisorRC = COMM_DIVISOR_RCIN_DEFAULT;
    logDivisorRC  = LOG_DIVISOR_RCIN_DEFAULT;

#if LAND_MODE == SEMI_LAND
	triggerDivisor = 24;	// default 1Hz
	triggerZone    = 50;
#endif

	// RF: VT1
	minFuelCons = 0.99f;
	maxFuelCons = 4.11f;
	fuelConsT1  = 0.0f;
	fuelConsT2  = 0.75f;
	fuelConsF1  = 0.99f;
	fuelConsF2  = 3.85f;
	bSpinWhenArmed      = false;
	armProcessTimeout = 5000;		//5s
	enginePwmWhenArmed = 1100;
	parachutePwmTurnon = 1850;
	parachutePwmTurnoff = 1150;
}

/** \name Method sets default values of configuration parameters.
*/
void ServoManager::ServManConfig::setDefaults(void)
{
	int c = sizeof(servo[0].coeffs_p) / sizeof(float);

	for(int i =0; i < SERVO_COUNT; i++)
	{
		for(int j = 0; j < c ; j++)
		{
			servo[i].coeffs_n[j] = 0.0f;
			servo[i].coeffs_p[j] = 0.0f;
			servo[i].address = 2000;
			servo[i].gain = 1.0f;
			servo[i].minValue = 1100;
			servo[i].maxValue = 1900;
			servo[i].trim = 1500;
		}
	}

	subsConf.setDefault();

}
/** \name ServoIO class constructor which maps physical adresses to the defined access structures.
*/
ServoManager::ServoIO::ServoIO()
{
    servoRegs = PlatformLayer::getServoRegs();
    rcinRegs = PlatformLayer::getRCinRegs();
}

#if LAND_MODE == SEMI_LAND
int ServoManager::getValueTriggerElv (void)
{
	return _valueTriggerElv;
}

int ServoManager::getValueTriggerAlr (void)
{
	return _valueTriggerAlr;
}

int ServoManager::calcValueTriggerElv (int triggerZone)
{
	int deltaRcElv = (_tlmRCIN.rcinData[0] & MASK_PWM_VALUE) - _conf.servo[0].trim;

	_valueTriggerElv = 0;
	if (deltaRcElv > triggerZone)
	{
		_valueTriggerElv = 1;
	}
	else if (deltaRcElv < (-triggerZone))
	{
		_valueTriggerElv = -1;
	}

	return _valueTriggerElv;
}

int ServoManager::calcValueTriggerAlr (int triggerZone)
{
	int deltaRcAlr = (_tlmRCIN.rcinData[3] & MASK_PWM_VALUE) - _conf.servo[3].trim;

	_valueTriggerAlr = 0;
	if (deltaRcAlr > triggerZone)
	{
		_valueTriggerAlr = 1;
	}
	else if (deltaRcAlr < (-triggerZone))
	{
		_valueTriggerAlr = -1;
	}

	return _valueTriggerAlr;
}
#endif

bool ServoManager::isArm(void)
{
    return _bArmed;
}

void ServoManager::resetParams(void)
{
    _iArmDisArmTime = 0;
    _bPreArmDisArm = false;
    _bArmSaveTime = false;
    _bDisarmSaveTime = false;
    _spinWhenArmed = false;
}

void ServoManager::armEngine(bool isArm)
{
	if(isArm)
	{
		VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO_ARM_a, 1);
	}else
	{
		VT_SBUS_CONTROLLER_mWriteReg(FLYEYE_SERVO_BASE, SERVO_ARM_a, 0);
	}
}

