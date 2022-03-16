#include <PilotIncludes.h>

/** \name Static members initialization
* \{
*/
const char LogManager::ERR_OK[]            = "log: ok";
const char LogManager::ERR_UCOMMAND[]      = "log: LG04 Unrecognized command or wrong number of parameters";
const char LogManager::ERR_SES_NOT_FOUND[] = "log: LG20 Log session not found";
const char LogManager::ERR_CLEAR_LOG[]     = "log: LG21 Cannot clear log";
const char LogManager::ERR_CMD_BUF_FULL[]  = "log: LG22 Command buffer full";
const char LogManager::ERR_LOG_GET[]  	   = "log: LG23 Cannot get log in flight";
const char LogManager::ERR_LOG_REMOVE[]    = "log: LG24 Cannot remove log in flight";
const char LogManager::ERR_LOG_SPACE[]     = "log: LG25 Cannot get space log in flight";
const char LogManager::SUBS_PREFIX[]       = "log";
///\}

LogManager::LogManager(void):
    ODTObserver(), ODTSubject(), SubsystemBase(),
    _cmdq(Log),
    // Initialization in case of no execution the 'linkObserver' method
    _extCmdTag(-1)
{
    Log.bootPrint ("OK" CRLF);
}

/** \name Method puts new line to the commands queue and sends the internal notification.
* \return 'true' if puts was successful, 'false' if queue was full or line wasn't put by the reason of some orther error.
*/
bool LogManager::putLine (ClassifiedLine &cl)
{
    bool ret = _cmdq.cmdPut(cl);
    if (!ret)
    {
        Log.errorPrintf ("LogManager_putLine_1");
        cl.answer (ERR_CMD_BUF_FULL);
    }
    
	// Notification of putting the line to the commands queue
	// NOTE: This method exetutes routines from different subsystem (_cmdq.cmdPut(cl)) but notification (EXT_CMD) is sending from SysMon subsystem.
	// Observer treats it as SysMon notification.
    notify (EXT_CMD);

    return ret;
}

/** \name Method makes specific subsystem component observable by LogManager class (the observer)
*/
void LogManager::linkObserver()
{
	// Self registration which allows to insert command lines (putLine(..)) into the command queue.
	// Method 'putLine' is executed by other subsystem, but its treated by an observer like it was a LogManager function.
    _extCmdTag = registerSubj (this, EXT_CMD);
}

/** \name Method performs specific MicroC/OS-II operating system task.
*/
void LogManager::task(void* pdata)
{
    for (;;)
    {
		// Waiting for a notification
         OSBase::EvtMask f = waitForAnyAspect ();

        if (checkAspect (f, _extCmdTag))
        {
            ClassifiedLine cl;

			// Getting items from the commands queue
            while (_cmdq.cmdGet(cl))
            {
                useCmdLine (cl);
            }
        }

    }
}

/** \name Method parses the commands from the communication channel.
*/
void LogManager::useCmdLine(ClassifiedLine &cl)
{
    if (!_parser.loadLine(cl.getLine()))
    {
        Log.errorPrintf("LogManager_useCmdLine_1");
        return;
    }
    
	// Parsing the 'log list' command
    if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "list") == 0)
    {
#if PILOT_TARGET == PT_WIN32
		Log.logList(cl, SUBS_PREFIX);
#else
		Log.logList(cl);
#endif
        cl.answer(ERR_OK);
        return;
    }

	// Parsing the 'log get boot' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "get") == 0 &&
        STRICMP (_parser.getToken(2), "boot") == 0)
    {
//        if (Log.logGet(cl, SUBS_PREFIX, 0, true))
//        if (Log.logGet(cl))
//            cl.answer(ERR_OK);
//        else
//            cl.answer(ERR_SES_NOT_FOUND);
    	cl.answer(ERR_OK);
        return;
    }

	// Parsing the 'log get n' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "get") == 0)
    {
		if (!SystemNowOnGround)
		{
			cl.answer(ERR_LOG_GET);
		   return;
		}

		char* ptr = 0;
		int session = static_cast<int>(strtol (_parser.getToken(2), &ptr, 10));
		if (*ptr == 0 && session >= 0)
		{
		//            if (Log.logGet(cl, SUBS_PREFIX, session))
#if PILOT_TARGET == PT_WIN32
			if (Log.logGet(cl, SUBS_PREFIX, session))
#else
			if (Log.logGet(cl, session, false))
#endif
				cl.answer(ERR_OK);
			else
				cl.answer(ERR_SES_NOT_FOUND);
			return;
		}
    }

	// Parsing the 'log remove n' command
    else if (_parser.count() == 3 &&
        STRICMP (_parser.getToken(1), "remove") == 0)
    {
		if (!SystemNowOnGround)
		{
			cl.answer(ERR_LOG_REMOVE);
		   return;
		}
        char* ptr = 0;
        int session = static_cast<int>(strtol (_parser.getToken(2), &ptr, 10));
        if (*ptr == 0 && session >= 0)
        {
            if (Log.logRemoveSession(cl, session))
                cl.answer(ERR_OK);
            else
                cl.answer(ERR_SES_NOT_FOUND);
            return;
        }
    }

	// Parsing the 'log clear' command
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "clear") == 0)
    {

    	if (!LogStor->clear(true) || !SystemNowOnGround)
        {
            cl.answer(ERR_CLEAR_LOG);
            return;
        }

        cl.answer(ERR_OK);
        return;
    }

	// Parsing the 'log clear noskip' command
    else if (_parser.count() == 3 && 
        STRICMP (_parser.getToken(1), "clear") == 0 &&
        STRICMP (_parser.getToken(2), "noskip") == 0)
    {
        if (!LogStor->clear(false) || !SystemNowOnGround)
        {
            cl.answer(ERR_CLEAR_LOG);
            return;
        }

        cl.answer(ERR_OK);
        return;
    }

	// Parsing the 'log space' command
    else if (_parser.count() == 2 && 
        STRICMP (_parser.getToken(1), "space") == 0)
    {
		if (!SystemNowOnGround)
		{
			cl.answer(ERR_LOG_SPACE);
		   return;
		}
        int i = LogStor->getFree();
        char s[LINESIZE];
        Log.secureSnprintf (s, LINESIZE, "%s: %.2f MB free", SUBS_PREFIX, (i == -1)? i : i/(1024.0f*1024.0f));
        cl.answer(s, false, false);
        cl.answer(ERR_OK);
        return;
    }

    cl.answer(ERR_UCOMMAND);
}
