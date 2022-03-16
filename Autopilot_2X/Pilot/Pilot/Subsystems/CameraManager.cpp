#include <PilotIncludes.h>

const char CameraManager::SUBS_PREFIX[]          = "camera: ";
const char CameraManager::CAMERA_CONF_FILE_NAME[] = "cameracfg";

// Static members initialization
const char CameraManager::ERR_OK[]            = "camera: ok";
const char CameraManager::ERR_CLOAD[]		  = "camera: CM02 Cannot load config data";
const char CameraManager::ERR_CSAVE[]         = "camera: CM03 Cannot save config data";
const char CameraManager::ERR_UCOMMAND[]      = "camera: CM04 Unrecognized command or wrong number of parameters";
const char CameraManager::MSG_NO_CONFIG[]     = "camera: CM07 Config data not loaded from flash";
const char CameraManager::ERR_CMD_BUF_FULL[]  = "camera: CM08 Command buffer full";

CameraManager::CameraManager(void):
    ODTObserver(), ODTSubject(), SubsystemBase(),
    _cmdq(Log)
{
    // Initialization in case of no execution the 'linkObserver' method
    _sysMonLinkTag = -1;
    _extCmdTag = -1;

    _isConfigLoaded = false;

    // Creation of a memory configuration data
    _confMem = StorageFactory::createConfigMemory();
    if( _confMem == NULL)
    {
        Log.abort ("Critical Error: CameraManager_1.");
        return;
    }

	// Initialization of a mapping of parameter names to objects. If initialization failed error message is logged.
    _confNames = new ParameterNames(SUBS_PREFIX, 15);
    if (_confNames == NULL)
    {
        Log.abort ("Critical Error: CameraManager_2.");
        return;
    }

	_confNames->insert("commDivisor", &_conf.commCamDivisor, 0, 1000);
    _confNames->insert("logDivisor", &_conf.logCamDivisor, 0, 1000);
	_confNames->insert("commTrgDivisor", &_conf.commTrgDivisor, 0, 1000);
    _confNames->insert("logTrgDivisor", &_conf.logTrgDivisor, 0, 1000);
    _confNames->insert("ledState", &_conf.ledState, 0, 2, false, UAF_LED);
    
//    Log.bootPrint ("OK" CRLF);
}

/** \name Method registers objects for observation.
*/
void CameraManager::linkObserver()
{
	// Self registration which allows to insert command lines (putLine(..)) into the command queue.
	// Method 'putLine' is executed with the context of different subsystem but it is assumed by the observer as it is a SysMons method. 
    _extCmdTag = registerSubj (this, EXT_CMD);

	// Registration of a system monitor (SysMon) subsystem (notification about loosing or establishing the connection)
    _sysMonLinkTag   = registerSubj (SysMon, SYSMON_LINK);
}

/** \name Method returns 'true' if subsystem is ready for take-off (system is fully functional).
* Method sends error messages to the communication channel 'cl' if it is not null.
* \note Method is executed in the context of different subsystem, pay attention while concurency programming.
*/
bool CameraManager::isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm)
{
    bool bok = true;

    // Check the configuration is loaded from flash
    if (!_isConfigLoaded)
    {
        Log.msgPrintf (cl, fLogComm, MSG_NO_CONFIG);
        bok = false;
    }

    return bok;
}

/** \name Method adds new command line to the commands queue and sends internal notification.
* \return 'true' if adding of a new command was successful, 'false' if queue was full or due to some errors
*/
bool CameraManager::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("CameraManager_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }

	// Notification of entering the line into the queue.
	// NOTE: the 'putLine' function is executed by other subsystem but notification comes from current subsystem.
	// Observer treats it as CameraManager¹s notification.
    notify (EXT_CMD);

    return ret;
}

/** \name Method performs specific MicroC/OS-II operating system task.
*/
void CameraManager::task(const void* pdata)
{
    while(true)
    {
//    	OSTimeDlyHMSM(0, 0, 2, 0);
#if PILOT_TARGET == PT_HARDWARE
    	usleep(1000000);
#endif
    	Log.msgPrintf("Camera Task\r\n");
    }
}

/** \name Method parses and executes 'cl' commands from communication channel
*/
void CameraManager::useCmdLine(ClassifiedLine &cl)
{
    
	if (!_parser.loadLine(cl.getLine()))
    {
        Log.errorPrintf("CameraManager_useCmdLine_1");
        return;
    }   

	// Parsing the 'camera save config' command
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

	// Parsing the 'camera heater <val>' command (0=Off, 1=On)
    else if (_parser.count() == 3 && STRICMP (_parser.getToken(1), "heater") == 0) 
    {
        if(_parser.getTokenAsInt(2)==1)
		{
			_camera.heaterOn();
			cl.answer(ERR_OK);        
		}
        else if(_parser.getTokenAsInt(2)==0)
		{
			_camera.heaterOff();
		    cl.answer(ERR_OK);        
		}
        else{
			cl.answer(ERR_UCOMMAND);
		}
			
        return;
    }
	
	// Parsing the 'camera preheater <val>' command (0=Off, 1=On)
    else if (_parser.count() == 3 && STRICMP (_parser.getToken(1), "preheater") == 0) 
    {
        if(_parser.getTokenAsInt(2)==1)
		{
			_camera.preHeaterOn();
			cl.answer(ERR_OK);        
		}
        else if(_parser.getTokenAsInt(2)==0)
		{
			_camera.preHeaterOff();
		    cl.answer(ERR_OK);        
		}
        else{
			cl.answer(ERR_UCOMMAND);
		}
			
        return;
    }

    // Parsing the 'camera on' command
    else if (_parser.count() == 2 && STRICMP (_parser.getToken(1), "on") == 0) 
    {
//        IOWR_ALTERA_AVALON_PIO_DATA(CAMERA_ON_BASE, 1);
		cl.answer(ERR_OK);
        return;
    }

    // Parsing the 'camera off' command
    else if (_parser.count() == 2 && STRICMP (_parser.getToken(1), "off") == 0) 
    {
//        IOWR_ALTERA_AVALON_PIO_DATA(CAMERA_ON_BASE, 0);
		cl.answer(ERR_OK);
        return;
    }

    // Parsing the 'camera set <name> <value>' command
    else if ((_parser.count() >= 4) && (STRICMP(_parser.getToken(1), "set") == 0)) 
    {
        ParameterNames::ERRCODE errc = setParameters (cl.getLine() + _parser.getTokenIndex(2));

        if (errc == ParameterNames::ERR_OK)
        {
            cl.answer(ERR_OK);
        }
        else {
            cl.answer(ERR_UCOMMAND);
        }

        return;
    }
    
	// Parsing the 'camera get <name>' command
    else if ((_parser.count() == 3) && (STRICMP(_parser.getToken(1), "get") == 0)) {
        if (getParameter(_parser.getToken(2), cl)) {
            // WK: Remove the summary line
            cl.answer(ERR_OK, false, true);
        }
        else {
            cl.answer(ERR_UCOMMAND);
        }
        return;
        
    }

    cl.answer(ERR_UCOMMAND);
}

CameraManager::ConfigData::ConfigData(void)
{
    setDefault ();
}

/** \name Method sets default data configuration
*/
void CameraManager::ConfigData::setDefault(void)
{
    commCamDivisor      =  0;
    logCamDivisor       =  0;
    commTrgDivisor      =  0;
    logTrgDivisor       =  0;
    ledState            = 0;
}

/** \name Method loads subsystem configuration data
* On error method sets default values (an error could disturb current values)
*/
bool CameraManager::confLoad(void)
{
    _isConfigLoaded = _confMem->loadFile (CAMERA_CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!_isConfigLoaded)
    {
		// Restoring the default configuration in case of error
        _conf.setDefault ();
        Log.errorPrintf("CameraManager_confLoad_1");
    }

    return _isConfigLoaded;
}

/** \name Method saves subsystem configuration
*/
bool CameraManager::confSave(void)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save config in flight", SUBS_PREFIX);
        return false;
    }

    bool bok = _confMem->saveFile (CAMERA_CONF_FILE_NAME, &_conf, sizeof(_conf));
    if (!bok)
        Log.errorPrintf("CameraManager_confSave_1");

    return bok;
}

/** \name Method sets values for many parameters. In case of error it keeps unchanged values.
* \param 'nameValueItems' - pairs of name and corresponding value (in textual representation).
*/
ParameterNames::ERRCODE CameraManager::setParameters (const char* nameValueItems)
{
    unsigned int flags=0;
    ParameterNames::ERRCODE err = _confNames->setParams (nameValueItems, &flags);
    if (err == ParameterNames::ERR_OK)
        setParamUserAction (flags);

    return err;
}

/** \name Method gets value of specified by 'pName' parameter
*/
bool CameraManager::getParameter (const char* pName, ClassifiedLine& cl) const
{
    bool ret = _confNames->getParam(pName, cl);
    return ret;
}

/** \name Method performs action after changing the value of parameter
* \param 'flags' - bit flags
*/
void CameraManager::setParamUserAction (unsigned int flags)
{
	// 'ledState' parameter has been changed
    if ((flags & UAF_LED) != 0)
    {
        bool linkBroken = SysMon->isLinkBroken();
        if (_conf.ledState == 0)
            _camera.ledOff();
        else if (_conf.ledState == 1)
            _camera.ledOn();
        else if (_conf.ledState == 2)
            if (linkBroken)
                _camera.ledOn();
            else
                _camera.ledOff();
    }
}
