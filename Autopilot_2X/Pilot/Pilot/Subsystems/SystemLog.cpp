#include <PilotIncludes.h>

//  Static constants definition
const char SystemLog::BEG_SES_TAG[] = CRLF "*Start:";   //  CRLF must be at the beginning
const char SystemLog::DTIME_SES_TAG[] = CRLF "*Utc:";   //  CRLF must be at the beginning

// Class constructor
SystemLog::SystemLog (void) :
bootPrintEnabled(true), commDevice(NULL), logDevice(NULL), logEnabled(true), timeUpdated(false)
{
}

/** \name Method performs the initialization which couldn't be done in constructor because the object is created
* before OSInit has been executed.
*/
void SystemLog::Init (void)
{
	//  Semaphore controlling access to the standard IO functions
	if (!_ioSem.create("SystemLog"))
    {
        Log.abort ("Critical Error: SystemLog_1.");
        return;
    }

}

/** \name Method sets log file name and device on which the log will be stored. Because the NIOS-II platform does not have
* file system, the file name must be in accordance with a predefined.
* \param 'stor' - device (an object) on which log will be stored.
* \param 'fname' - log file name.
*/
bool SystemLog::setLogFile (StorageBase* stor, const char* fname)
{
//	// Function works only after starting multitasking (because of unprotected object variables)
//    if (Os->isStarted())
//        return false;

    bool b = stor->open(fname);
    if (b)
        logDevice = stor;

	SNPRINTF(fileName, sizeof(fileName), fname);
    return b;

}

/** \name Method sets the communication device (output) to which system messages, that aren't commands
* ansewrs from communication channel, will be send.
* \param 'ttyNo' - comunication device id (platform depended)
*/
bool SystemLog::setCommDeviceByNo (int ttyNo)
{
    if (ttyNo == 2)
        commDevice = Tty2;
    else if (ttyNo == 1)
        commDevice = Tty1;
    else
        commDevice = Tty0;

    return true;
}

/** \name Method sends a text to a defined output device (one or many). Performed after starting multitasking to log information about errors.
* Execution of this method do not suspends the application (data are send in another process).
* \param 'noFlash' - if 'true' text don't be stored in the flash memory.
*/
void SystemLog::errorPrint (const char* msg, bool noFlash)
{
    // Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];

	// Execution of a function without error messages
    int len = secureSnprintf (buf, LINESIZE, "*Err:%9.3f: ", static_cast<float>(PState->getTime100()) / 10000.0f);
    if (len < 0)
        return;

    MEMCCPY (buf+len, msg, 0, LINESIZE-len);
    buf[LINESIZE-1] = 0;

    // To communication channel
    if (commDevice != NULL)
        commDevice->sendLine (buf, true, false, GPar.commMsgCrc);

    // To flash
#if PILOT_TARGET == PT_WIN32 
	if (logDevice != NULL && logEnabled)
		logDevice->append(buf);
#else
    if (logDevice != NULL && !noFlash && logEnabled)
    	logDevice->appendNew(buf, timeUpdated? fileNameNew:fileName, LINESIZE, true);
#endif
}


void SystemLog::errorPrint (const char* msg, int val, const char* msg2, bool noFlash)
{
    // Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[2*LINESIZE];   // Buffer is enlarged in case the result is greater than LINESIZE size
    if (secureSnprintf (buf, LINESIZE, "%s%i%s", msg, val, msg2) >= 0)
    {
        buf[LINESIZE-1] = 0;    // Ensuring that the buffer ends with 0
        errorPrint (buf, noFlash);
    }
}


void SystemLog::errorPrintf (const char* format, ...)
{
    // Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];
    va_list argptr;

	// Semaphores messages are disabled (error messages of the semaphore are send by other function)
    if (!_ioSem.lock ())
        return;

    va_start (argptr, format);

    vsnprintf (buf, sizeof(buf), format, argptr);
    buf[sizeof(buf)-1] = 0;

    va_end (argptr);

    _ioSem.unlock ();

    errorPrint (buf);
}

/** \name Method sends the system message
*/
void SystemLog::msgPrintf (const char* format, ...)
{
	// Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];
    va_list argptr;

    int len = secureSnprintf (buf, LINESIZE, "*Msg:%9.3f: ", static_cast<float>(PState->getTime100()) / 10000.0f);
    if (len < 0)
        return;

    if (!_ioSem.lock ())
        return;

    va_start (argptr, format);

    vsnprintf (buf+len, sizeof(buf)-static_cast<size_t>(len), format, argptr);
    buf[sizeof(buf)-1] = 0;

    va_end (argptr);

    _ioSem.unlock ();

	// To communication channel
    if (commDevice != NULL)
        commDevice->sendLine (buf, false, false, GPar.commMsgCrc);

    // To flash
#if PILOT_TARGET == PT_WIN32 
	if (logDevice != NULL && logEnabled)
		logDevice->append(buf);
#else
    if (logDevice != NULL && logEnabled)
    	logDevice->appendNew(buf, timeUpdated? fileNameNew:fileName, LINESIZE, false);
#endif
}

/** \name Method sends the system message and response to the pcl communication channel.
* Note: Function 'answer' is executed with 'false' as 'summary' argument. Function 'answer' must be re-executed
* at the end with 'true' as 'summary' argument to send the number of rows.
* \param 'pcl' - line to which to send an answer (if NULL no answer will be send)
* \param 'fLogComm' - if flag is set to 'true' the message will be send to the communication channel and log
*/
void SystemLog::msgPrintf (ClassifiedLine* pcl, bool fLogComm, const char* format, ...)
{
    char buf[LINESIZE];
    va_list argptr;

    if (!_ioSem.lock ())
        return;

    va_start (argptr, format);

    vsnprintf (buf, sizeof(buf), format, argptr);
    buf[sizeof(buf)-1] = 0;

    va_end (argptr);

    _ioSem.unlock ();

    if (pcl != NULL)
        pcl->answer (buf, false, false);

    if (fLogComm)
        msgPrintf (buf);
}

/** \name Method sends the system message and response to the pcl communication channel.
* Note: Function 'answer' is executed with 'false' as 'summary' argument. Function 'answer' must be re-executed
* at the end with 'true' as 'summary' argument to send the number of rows.
*/
void SystemLog::msgPrint (ClassifiedLine* pcl, const char* msg)
{
    if (pcl != NULL)
        pcl->answer (msg, false, false);
    else
        msgPrintf (msg);
}

/** \name Method sends the telemetry data.
* \param 'id' - data format identifier
* \param 'data' - textual data (it could be an base64 encoding)
* \param 'fLog' - flag to write to the log
* \param 'fComm' - flag to write to the communication channel
*/
void SystemLog::tlmPrint (enum TLM_FORMAT id, const char* data, bool fLog, bool fComm) const
{
    // Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];
    int j = id % 100;

    MEMCCPY (buf, "*T??: ", 0, LINESIZE);
    // j/10 - integer division
    buf[2] = static_cast<char>((j / 10) + '0');
    buf[3] = static_cast<char>((j % 10) + '0');
    MEMCCPY (buf+6, data, 0, LINESIZE-6);

    // To communication channel
    if (fComm && commDevice != NULL)
        commDevice->sendLine (buf, false, false, GPar.commCrc);

    // To flash
#if PILOT_TARGET == PT_WIN32 
	if (fLog && logDevice != NULL && logEnabled)
		logDevice->append(buf);
#else
    if (fLog && logDevice != NULL && logEnabled)
    	logDevice->appendNew(buf, timeUpdated? fileNameNew:fileName, LINESIZE, false);
#endif
}

void SystemLog::eventPrint (enum EVT id, const char* data) const
{
    // Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];
    int j = id % 100;

    MEMCCPY (buf, "*X??: ", 0, LINESIZE);
    // j/10 - integer division
    buf[2] = static_cast<char>((j / 10) + '0');
    buf[3] = static_cast<char>((j % 10) + '0');
    MEMCCPY (buf+6, data, 0, LINESIZE-6);

    // To communication channel
    if (commDevice != NULL)
        commDevice->sendLine (buf, false, false, GPar.commCrc);

    // To flash
#if PILOT_TARGET == PT_WIN32 
	if (logDevice != NULL && logEnabled)
		logDevice->append(buf);
#else
    if (logDevice != NULL && logEnabled)
    	logDevice->appendNew(buf, timeUpdated? fileNameNew:fileName, LINESIZE, false);
#endif
}


void SystemLog::controlPrint (int id, const char* data) const
{
	// Function works only after starting multitasking
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];
    int j = id % 100;

    MEMCCPY (buf, "*C??: ", 0, LINESIZE);
	// j/10 - integer division
    buf[2] = static_cast<char>((j / 10) + '0');
    buf[3] = static_cast<char>((j % 10) + '0');
    MEMCCPY (buf+6, data, 0, LINESIZE-6);

	// To communication channel
    if (commDevice != NULL)
        commDevice->sendLine (buf, false, false, false);
}

/** \name Method sends message about starting the system (to log only)
*/
void SystemLog::startPrint (void)
{
	// Function works only after startig multitasking
    if (!Os->isStarted())
        return;

    //  To flash
    if (logDevice != NULL  && logEnabled)
    {
		// Empty row at the beginning because previous session may not ends correctly
        char buf[LINESIZE];
        if (secureSnprintf (buf, sizeof(buf),"%s *************** System started ***************", BEG_SES_TAG) >= 0)
        {
#if PILOT_TARGET == PT_WIN32 
			logDevice->append(buf);
#else
			logDevice->appendNew(buf, timeUpdated ? fileNameNew : fileName, LINESIZE, false);
#endif
        }
    }
}

/** \name Method sends date and UTC (from GPS) time information message.
*/
void SystemLog::dateTimePrint (int uyear, int umonth, int uday, int uhour, int umin, int usec)
{
	// Function works only after starting multitasking
    if (!Os->isStarted())
        return;

	// 'DTIME_SES_TAG' has a CRLF marks at the beginning (skipping it)
#if EXTERNAL_MEMORY_TYPE == USE_MMC
    if (secureSnprintf (fileNameNew, sizeof(fileNameNew),"1:/log_%04i_%02i_%02i_%02i_%02i_%02i",
         uyear+2000, umonth, uday, uhour, umin, usec) >= 0)
#elif EXTERNAL_MEMORY_TYPE == USE_SDCARD
        if (secureSnprintf (fileNameNew, sizeof(fileNameNew),"log_%04i_%02i_%02i_%02i_%02i_%02i",
             uyear+2000, umonth, uday, uhour, umin, usec) >= 0)
#endif
    {
    	logDevice->close();
    	logDevice->renameFile(fileName, fileNameNew);
    	logDevice->open(fileNameNew);

    	timeUpdated = true;

		// To communication channel
		if (commDevice != NULL)
			commDevice->sendLine (fileNameNew, false, false, GPar.commMsgCrc);
    }
}

/** \name Method sends to log an echo of the input line.
*/
void SystemLog::inputPrint (const char* msg)
{
	// Function works only after starting multitasking.
    if (!Os->isStarted())
        return;

    char buf[LINESIZE];

	// Execution of a function without error messages
    int len = secureSnprintf (buf, LINESIZE, "*Inp:%9.3f: ", static_cast<float>(PState->getTime100()) / 10000.0f);
    if (len < 0)
        return;

    MEMCCPY (buf+len, msg, 0, LINESIZE-len);
    buf[LINESIZE-1] = 0;

	// To flash
#if PILOT_TARGET == PT_WIN32 
	if (logDevice != NULL && logEnabled)
		logDevice->append(buf);
#else
    if (logDevice != NULL && logEnabled)
    	logDevice->appendNew(buf, timeUpdated? fileNameNew:fileName, LINESIZE, false);
#endif
}

/** \name Method sends text to default system-specified console. Executed at boot before the multitasking starts (method may hold execution of application ).
* If 'bootPrintEnabled' is set to 'false' method does nothing (to not to hold after restart in the air).
*/
void SystemLog::bootPrint (const char* msg) const
{
#if PILOT_TARGET == PT_HARDWARE
	if (bootPrintEnabled && !Os->isStarted()) { ; };
#elif PILOT_TARGET == PT_WIN32
	if (bootPrintEnabled && !Os->isStarted())
		printf("%s", msg);
#endif
}


void SystemLog::bootPrintf (const char* format, ...) const
{
    if (bootPrintEnabled && !Os->isStarted())
    {
        va_list argptr;
        va_start (argptr, format);

        vprintf (format, argptr);

        va_end (argptr);
    }
}

/** \name Method tries to abort the program if 'BootPrintEnable' flag is set to 'true'. Executed at boot after an error.
* When 'bootPrinEnabled' flag is set to 'false' method does nothig ( to allow ignoring errors after reset in the air ).
*/
void SystemLog::tryAbort (const char* msg)
{
    if (Os->isStarted())
    {
        errorPrintf ("%s ABORT." CRLF, msg);
        Os->sleepMs(ABORT_WAIT_MS);
    }
    else
        bootPrintf ("%s ABORT." CRLF, msg);

    if (bootPrintEnabled)
    {
        PlatformLayer::reboot ();
    }
}

// Unconditional program interruption
void SystemLog::abort (const char* msg)
{
    if (Os->isStarted())
    {
        errorPrintf ("%s ABORT." CRLF, msg);
        Os->sleepMs(ABORT_WAIT_MS);
    }
    else
        bootPrintf ("%s ABORT." CRLF, msg);

    PlatformLayer::reboot ();
}


void SystemLog::abort (const char* msg, int val, const char* msg2)
{
    if (Os->isStarted())
    {
        errorPrintf ("%s%i%s ABORT." CRLF, msg, val, msg2);
        Os->sleepMs(ABORT_WAIT_MS);
    }
    else
        bootPrintf ("%s%i%s ABORT." CRLF, msg, val, msg2);

    PlatformLayer::reboot ();
}

/** \name Method performs printing functionality protected by semaphore and ensures that string ends with "\0".
* \return Number of printed characters or -1 when error occured.
*/
int SystemLog::secureSnprintf (char* buffer, size_t count, const char* format, ...)
{
    va_list argptr;

    if (count < 1u)
        return -1;

	// Initialization of an empty string in case of exit with an error
    buffer[0] = 0;

    if (Os->isStarted())
		// Semaphores messages are disabled in order to avoid recursion
        if (!_ioSem.lock (true))
            return -1;

    va_start (argptr, format);

    int ret = vsnprintf (buffer, count, format, argptr);
    // Ensuring that the buffer ends with 0
    buffer[count-1u] = 0;

    va_end (argptr);

    if (Os->isStarted())
        if (!_ioSem.unlock (true))
            return -1;

    return ret;
}

/** \name Method lists all the session data saved in the log. Session starts with marks of the new line
* and '*Start:' statement. Except for the first session which can begin with EOL(end of line) marks.
* Printed offset determines the position of the first character AFTER OMITTING the EOL marks.
* Method is executed by LogManager class (fiend relationship).
*/
void SystemLog::logList (ClassifiedLine& cl, const char* prefix)
{
    int offset = 0;
    int lastOffset = 0;
    int count = 0;
    char buf[LINESIZE];
    char dtBuf[2][LINESIZE] = {{""}, {""}};
    int dtBufInd = 0;
    bool firstCycle = true;
    const int dTimeSesTagLen = static_cast<int>(strlen (DTIME_SES_TAG));

	// Searching for the line endings (because the beginning could be broken)
     const char* sesTag = CRLF;
	 // Offset updates after each call
     while (logDevice->scan(sesTag, offset))
    {
		// Searching for the full start of session
        sesTag = BEG_SES_TAG;
        
		// Searching for a line with date and time of the session (move to the next character to skip text causes interruption)
        int dtOffset = offset + 2;
        dtBuf[dtBufInd % 2][0] = 0;
        if (logDevice->scan(DTIME_SES_TAG, dtOffset, 0, BEG_SES_TAG))
        {
			// A text with CRLF marks at the beginning has been found (so it was in the pattern)
			// Reading the line without CRLF marks and content of the pattern
            dtOffset += dTimeSesTagLen;
            if (logDevice->readLine(dtBuf[dtBufInd % 2], LINESIZE, dtOffset))
            {
				// 'dtOffset' points to the beginning of a new line.
				// Go back about two characters (CRLF) to be able to detect the beginning of a new line (including the CRLF mark)
                dtOffset -= 2;
            }
        }
        
        if (!firstCycle)
        {
            int size1 = offset - lastOffset;            
			// Printing 'dtBuf' from the previous object of the loop
            if (secureSnprintf (buf, sizeof(buf), "%s: ses:%3d   off:%8d   size:%7d   date:%s",
                prefix, count++, lastOffset+2, size1, dtBuf[(dtBufInd+1) % 2]) >= 0)
            {
                buf[LINESIZE-1] = 0;    // Ensuring that the buffer ends with 0
                // Printing without line numbers
                cl.answer(buf, true, false);
            }
        }
        else
        {
            firstCycle = false;
        }

        lastOffset = offset;
		// 'dtOffset contains offset+2 or beginning of the line which is the next line before the line with '*Utc' and starts with CRLF.
        offset = dtOffset;
        dtBufInd++;
    }
	// Last row
    int size2 = offset - lastOffset - 2;
    if (secureSnprintf (buf, sizeof(buf), "%s: ses:%3d   off:%8d   size:%7d   date:%s",
        prefix, count++, lastOffset+2, size2, dtBuf[(dtBufInd+1) % 2]) >= 0)
    {
        buf[LINESIZE-1] = 0;    // Ensuring that the buffer ends with 0
        // Printing without line numbers
        cl.answer(buf, true, false);
    }
}

void SystemLog::logList (ClassifiedLine& cl)
{
	std::vector<std::string> listFile = logDevice->listFile();

	int n = listFile.size();
	std::sort(listFile.begin(), listFile.end());
	for(int i=0; i < n; i++)
	{
        cl.answer(listFile[i].c_str(), true, false);
	}
	listFile_SD = listFile;
}

/** \name Method sends log with the specified id to the specified device. Method holds execution of application.
* \param 'cl' - the output device to which the content of a log is send.
* \param 'prefix' - line prefix (usually it is a subsystem name).
* \param 'session' - id of the sending session.
* \param 'boot' - flag indicates the last session with enabled telemetry filtration ('session' isn't take into account)
* \return 'True' if session with given id has been found, 'false' otherwise.
*/
bool SystemLog::logGet (ClassifiedLine& cl, const char* prefix, int session, bool boot)
{
    int offset=0;
    int count = 0;
    bool firstCycle = true;
    int tokenLength = static_cast<int>(strlen(BEG_SES_TAG));
    char buf[2*LINESIZE];   // Size during processing may be larger then LINESIZE.
    char content[LINESIZE];

	// Searching for the session
    if (!boot)
    {
		// Session with the specified id
        if (!findSessionOffset (session, offset))
            return false;
    }
    else
    {
        // Last session
        if (findSessionOffset (INT_MAX, offset))
            return false;
    }

    while (logDevice->readLine(content, sizeof(content), offset))
    {
		// When new session mark is found, the loop is escaped
        if (!firstCycle && STRNICMP (content, BEG_SES_TAG + 2, tokenLength-2) == 0)
            return true;
        else
            firstCycle = false;

		// Telemetry skipping in boot mode
        if (boot && content[0] == '*' && content[1] == 'T')
            continue;

        if (secureSnprintf (buf, sizeof(buf), "%s: line:%5d  %s", prefix, count++, content) >= 0)
        {
			// Length reduction in the case when the line is too long to pass by communication channel
            buf[LINESIZE-1] = 0;
			// Printing without line numbers
            cl.answer(buf, true, false);
        }
    }

    return true;
}

bool SystemLog::logGet (ClassifiedLine& cl)
{
	//List File
//	logList(cl);
	std::vector<std::string> listFile = logDevice->listFile();

	int n = listFile.size();
	std::sort(listFile.begin(), listFile.end());
	if(n == 0)
	{
		return false;
	}

	for(int i=0; i < n-1; i++)
	{
		std::string rootfile = listFile[i];

		if(!logDevice->UploadFile(cl, rootfile))
		{
			return false;
		}else
		{
				logDevice->remove(rootfile);
		}
		OSTimeDlyHMSM(0, 0, 2, 0);  //delay 2s

	}

	return true;
}

bool SystemLog::logGet (ClassifiedLine& cl, int session, bool boot)
{
	std::vector<std::string> listFile = logDevice->listFile();

	int n = listFile.size();
	std::sort(listFile.begin(), listFile.end());
	if(n == 0)
	{
		return false;
	}

	if(session >= n )
	{
		return false;
	}
	std::string rootfile = listFile[session];
	if(!logDevice->UploadFile(cl, rootfile))
	{
		return false;
	}
	return true;
}

bool SystemLog::logRemoveSession(ClassifiedLine& cl, int session)
{
	std::vector<std::string> listFile = logDevice->listFile();

	int n = listFile.size();
	std::sort(listFile.begin(), listFile.end());
	if(n == 0)
	{
		return false;
	}

	for(int i=0; i < n-1; i++)
	{
		std::string rootfile = listFile[i];

		if(i == session)
		{
				logDevice->remove(rootfile);
		}
		OSTimeDlyHMSM(0, 0, 2, 0);  //delay 2s

	}

	return true;
}

/** \name Method seeks the offset to the session with the given id (id starts from 0). Session starts with the new line sign and the '*Start' string.
* Except for the first session which can begin with EOL(end of line) marks. The returned offset determines the position of the first character
* AFTER OMITTING the EOL initial marks.
* \return 'True' if session was found, 'false' otherwise (in this case the offset holds possition of last found session).
*/
bool SystemLog::findSessionOffset (int session, int &offset) const
{
    int lastOffset = 0;
    offset = 0;

	// Searching for the line endings (because the beginning could be broken)
    bool ret = logDevice->scan(CRLF, offset);
	// End of line markers are omitted
    offset+=2;

	// Searching for the entire beginning of session
    for (int i = 1; ret && i <= session; i++)
    {
        lastOffset = offset;
        ret = logDevice->scan(BEG_SES_TAG, offset);
        offset+=2;
    }
    
	// If offset to current session cannot be found, an offset to the last session is returned
    if (!ret)
        offset = lastOffset;

    return ret;
}

/** \name Method disables logging. Method is executed by SystemMonitor class for the purpose of memory testing.
*/
void SystemLog::logDisable (void)
{
    logEnabled = false;
}

/** \name Method enables logging
*/
void SystemLog::logEnable (void)
{
    logEnabled = true;
}
