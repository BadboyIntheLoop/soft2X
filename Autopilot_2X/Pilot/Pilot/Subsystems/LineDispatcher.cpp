#include <PilotIncludes.h>


LineDispatcher::LineDispatcher(void):
    ODTObserver(), ODTSubject(),
	// Initialization of a pointers to classified line objects
    nmeaLine(&clArr[0]),
    cmdLine(&clArr[1]),
    simLine(&clArr[2]),
    tmpLine(&clArr[4]),
    auxNmeaLine(&clArr[5]),
	hmrLine(&clArr[6]),
	// Tags are initialized with '-1' because their indices start with '0'
#if MAGNETOMETER_TYPE == USE_HMR
    GpsTag(-1), Tty0Tag(-1), Tty0ChTag(-1), HmrTag(-1), Tty1Tag(-1), Tty2Tag(-1)
#else
GpsTag(-1), Tty0Tag(-1), Tty0ChTag(-1),  Tty1Tag(-1), Tty2Tag(-1)
#endif
{
	// Creation of a semaphore controling access to the subsystem data
    if (!_vSem.create("LineDispatcher"))
    {
        Log.abort ("Critical Error: LineDispatcher_1.");
        return;
    }
	Log.bootPrint("OK" CRLF);
}

/** \name Method performs specific MicroC/OS-II operating system task.
*/
void LineDispatcher::task(const void* p_arg)
{
    while(true)
    {
        // Waiting for notification of receiving a line from some device(s)
        OSBase::EvtMask f = waitForAnyAspect ();

		// Line is waiting for reading (reading to 'tmpLine')
		// Now only the NMEA line is detected due to the source of origin (starting with '$')
        if (checkAspect (f, GpsTag))
        {
            tmpLine->reset();
			// Direct access to the '_line' field because 'getLine()' returns const pointer
            Gps->getLastLine(tmpLine->_line);
            tmpLine->_sender = Gps;
            classify ();
        }

		// Use of an 'else' statement is prohibited here because there is possibility of comming several notifications at the same time
		if (checkAspect(f, Tty0Tag))
		{
			tmpLine->reset();
			Tty0->getLastLine(tmpLine->_line);
			tmpLine->_sender = Tty0;
			classify();
		}

		// Reading from 'CommChannel' object (which derives from the 'SerialDeviceBase' but do not handle the 'getLastLine')
		if (checkAspect(f, Tty0ChTag))
		{
			ChannelData rcvData;
			tmpLine->reset();
			while (Tty0->getData(rcvData))
			{
				tmpLine->_sender = Tty0;
				MEMCCPY(tmpLine->_line, rcvData.getPayload().getData(), 0, LINESIZE);
				classify();
			}
		}

        if (checkAspect (f, Tty1Tag))
        {
            tmpLine->reset();
            Tty1->getLastLine(tmpLine->_line);
            tmpLine->_sender = Tty1;
            classify ();
        }

        if (checkAspect (f, Tty2Tag))
        {
            tmpLine->reset();
            Tty2->getLastLine(tmpLine->_line);
            tmpLine->_sender = Tty2;
            classify ();
        }
    }
}

/** \name Method registers objects for observation
*/
void LineDispatcher::linkObserver(void)
{
    if (Gps != NULL)
        GpsTag = registerSubj (Gps, LINE_RECEIVED);
    if (Tty0 != NULL)
    {
		// Temporary object Tty0 may be of SerialDeviceBase class (old conception) or of CommChannel class (new conception)
		// If this object is of CommChannel class then it sends CH_DATA_RECEIVED instead of LINE_RECEIVED notificaion and the data must be read from it in different way
        Tty0Tag = registerSubj (Tty0, LINE_RECEIVED);
        Tty0ChTag = registerSubj (Tty0, CH_DATA_RECEIVED);
    }
#if MAGNETOMETER_TYPE == USE_HMR
    if(Hmr != NULL)
    {
    	HmrTag = registerSubj (Hmr, LINE_RECEIVED);
    }
#endif

    if (Tty1 != NULL)
    {
        Tty1Tag = registerSubj (Tty1, LINE_RECEIVED);
    }

	if (Tty2 != NULL)
	{
        Tty2Tag = registerSubj (Tty2, LINE_RECEIVED);
	}
}

/** \name Method classifies received line
*/
void LineDispatcher::classify (void)
{
	// Ensure the string ends correctly
    tmpLine->_line[LINESIZE-1] = 0;

	// NMEA sequence from GPS
    if (tmpLine->getLine()[0]  == '$')
    {
        if (tmpLine->_sender == Gps)
        {
			// Primary GPS on the dedicated channel
            if (swapBuffers (&nmeaLine))
                notify (NMEA_LINE);
        }
        else if (tmpLine->_sender == Tty2)
        {
			// Secondary GPS on the Tty2 channel
            if (swapBuffers (&auxNmeaLine))
                notify (AUX_NMEA_LINE);
        }

        return;
    }

	// The sequence of virtual measurements from simulator
    if (tmpLine->getLine()[0] == '#')
    {
        if (swapBuffers (&simLine))
            notify (SIM_LINE);

		return;
    }

#if MAGNETOMETER_TYPE == USE_HMR

	// Writting the command (or other received text) to the log
	// NOTE: Some commands and characters are omitted by function 'logInput'
    if ((tmpLine->_sender != Gps) && (tmpLine->_sender != Tty2) && (tmpLine->_sender != Hmr))
    	logInput (tmpLine);
#else
	// Writting the command (or other received text) to the log
	// NOTE: Some commands and characters are omitted by function 'logInput'
    if ((tmpLine->_sender != Gps) && (tmpLine->_sender != Tty2))
    	logInput (tmpLine);
#endif

    // Ignored line (stored to the log)
    if (tmpLine->getLine()[0] == '[')
    {
        return;
    }

    // Control of an optional CRC sum
    if (!tmpLine->extractCrc())
    {
        tmpLine->answer ("E01 Bad CRC");
        return;
    }

	// Remaining input data (input telemetry)
    if (tmpLine->getLine()[0] == '&')
    {
        inputAuxData();
        return;
    }

	// Other command - determining the commands identifier (if available)
    if (!tmpLine->extractCmdId())
    {
        tmpLine->answer ("E02 Bad command id");
        return;
    }

	// Defining of a destination subsystem
	// 'subsystemCmd()' internally sends notification and executes 'swapBuffers()'
    if (subsystemCmd ())
        return;

	// Attempt to interpret the command in this subsystem
    if (!localCmd ())
        tmpLine->answer ("E03 Bad subsystem name or direct command");
}

/** \name Method assigns a temporary object with the received line to the variable describing the classified line.
* At the same time the previous assignment of the object as a temporary.
* \param 'cl' - pointer to the classified line object (mutable)
* \return 'true' on success, 'false' on lock by another task (rather not possible)
*/
bool LineDispatcher::swapBuffers (ClassifiedLine** cl)
{
	// Access control to variable by semaphore
    if (!_vSem.lock ())
    {
        Log.errorPrint("LineDispatcher_swapBuffers_1");
        return false;
    }

    // Buffers switching
    ClassifiedLine* a = tmpLine;
    tmpLine = *cl;
    *cl = a;

    // Releasing of a semaphore
	if (!_vSem.unlock ())
        return false;

    return true;
}

/** \name Method interprets commands for present subsystem.
* \return 'true' if command was parsed correctly and executed.
*/
bool LineDispatcher::localCmd (void)
{
	// Parsing the 'echo' command
	if (STRNICMP (tmpLine->getLine(), "echo", 4) == 0 && (tmpLine->getLine()[4] == 0 || isspace (tmpLine->getLine()[4]) != 0)) {
        if (isspace (tmpLine->getLine()[4]) != 0)
            tmpLine->answer (tmpLine->getLine() + 5);
        return true;

    }
	// Parsing the 'cpu' command
	else if (STRICMP (tmpLine->getLine(), "cpu") == 0) {
		// CPU load [%]
        char b[LINESIZE];

		// If OS_TASK_STAT_EN == 0 then OSCPUUsage variable does not exists
#if OS_TASK_STAT_EN == 1
        SNPRINTF (b, sizeof(b), "cpu: %u%%", OSCPUUsage);
#else
        SNPRINTF (b, sizeof(b), "cpu: %u%%", 0);
#endif

        tmpLine->answer (b);
        return true;

    }
	// Parsing the 'ver' command
	else if (STRICMP (tmpLine->getLine(), "ver") == 0) 
	{
        // Software version
        char b2[LINESIZE];
        SNPRINTF (b2, sizeof(b2), "ver: %i.%i", SW_VER_MAJOR, SW_VER_MINOR);
        tmpLine->answer (b2);
        return true;

    } 
	else if (STRICMP (tmpLine->getLine(), "sn") == 0) 
	{
        // Serial number
        char b2[LINESIZE];
        SNPRINTF (b2, sizeof(b2), "sn: %u", SerialNo);
        tmpLine->answer (b2);
        return true;
    }
	else if (STRICMP (tmpLine->getLine(), "software detail") == 0)
	{
		// Detail software version
		char b2[LINESIZE];
		SNPRINTF (b2, sizeof(b2), "Model: UAV2-X, thu nghiem bay tu dong ");
		tmpLine->answer (b2);
		return true;
	}
	else if (STRICMP (tmpLine->getLine(), "author") == 0)
	{
		// Software deverlop Author
		char b2[LINESIZE];
		SNPRINTF (b2, sizeof(b2), "Developed by Nguyen Tien Truong");
		tmpLine->answer (b2);
		return true;
	}

    return false;
}

/** \name Method interprets commands designed to other subsystem.
* Method sends notification to the other subsystems and sets the content for a proper buffer.
* \return 'true' if command was parsed correctly
*/
bool LineDispatcher::subsystemCmd (void)
{
    const char *line = tmpLine->getLine();
    
	// A new concept of caching (each subsystem has its own queue)
	// Order depends on frequency of sending the commands (approximately)
    if (STRNICMP (line, "fp", 2) == 0 && (line[2] == 0 || isspace (line[2]) != 0))
    {
        SysMon->putLine(*tmpLine);    // 'SysMon' interprets also an 'FPlan' commands
        FPlan->putLine(*tmpLine);
        return true;
    }
    else if (STRNICMP (line, "real", 4) == 0 && (line[4] == 0 || isspace (line[4]) != 0))
    {
        FPReal->putLine(*tmpLine);
        return true;
    }
    else if (STRNICMP (line, "ps", 2) == 0 && (line[2] == 0 || isspace (line[2]) != 0))
    {
        PState->putLine(*tmpLine);
        return true;
    }
    else if (STRNICMP (line, "smon", 4) == 0 && (line[4] == 0 || isspace (line[4]) != 0))
    {
        SysMon->putLine(*tmpLine);
        return true;
    }
    else if (STRNICMP (line, "control", 7) == 0 && (line[7] == 0 || isspace (line[7]) != 0))
    {
        FControl->putLine(*tmpLine);
        return true;
    }
    else if (STRNICMP (line, "servo", 5) == 0 && (line[5] == 0 || isspace (line[5]) != 0))
    {
        ServMan->putLine(*tmpLine);
        return true;
    }
    else if (STRNICMP (line, "log", 3) == 0 && (line[3] == 0 || isspace (line[3]) != 0))
    {
        LogMan->putLine(*tmpLine);
        return true;
    }

    return false;
}

/** \name Method interprets external data (input telemetry)
*/
void LineDispatcher::inputAuxData (void)
{
    const char *line = tmpLine->getLine();

    if (STRNICMP (line, "&T18:", 5) == 0 && isspace (line[5]) != 0)
    {
        FPReal->putLine(*tmpLine);
        return;
    }
}

/** \name Method sets an empty object with data line from simulator (by copy the object)
* \return 'true' on success, 'false' on lock by another task (rather not possible)
*/
bool LineDispatcher::getSimLine(ClassifiedLine &cl)
{
    return getXLine (cl, simLine);
}

/** \name Method sets an empty object with data line from primary GPS (by copy the object)
* \return 'true' on success, 'false' on lock by another task (rather not possible)
*/
bool LineDispatcher::getNmeaLine(ClassifiedLine &cl)
{
    return getXLine (cl, nmeaLine);
}

#if MAGNETOMETER_TYPE == USE_HMR
/** \name Method sets an empty object with data line from primary HMR2300 (by copy the object)
* \return 'true' on success, 'false' on lock by another task (rather not possible)
*/
bool LineDispatcher::getHmrLine(ClassifiedLine &cl)
{
    return getXLine (cl, hmrLine);
}
#endif

/** \name Method sets an empty object with data line from secondary GPS (by copy the object)
* \return 'true' on success, 'false' on lock by another task (rather not possible)
*/
bool LineDispatcher::getAuxNmeaLine(ClassifiedLine &cl)
{
    return getXLine (cl, auxNmeaLine);
}

/** \name Method sets an empty object with data line with external command (by copy the object)
* \return 'true' on success, 'false' on lock by another task (rather not possible)
*/
bool LineDispatcher::getCmdLine(ClassifiedLine &cl)
{
    return getXLine (cl, cmdLine);
}

/** \name Auxiliary method copies the object taking into account the multi-tasking.
* \param 'clTarget' - an empty object that will be filled with data
* \param 'clSource' - pointer to a source object
*/
bool LineDispatcher::getXLine(ClassifiedLine &clTarget, const ClassifiedLine* clSource)
{
    // Access control to variable by semaphore
    if (!_vSem.lock ())
    {
        Log.errorPrint("LineDispatcher_getXLine_1");
        return false;
    }

    clTarget = *clSource;

	// Releasing of a semaphore
	if (!_vSem.unlock ())
        return false;

    return true;
}

/** \name Method writes content of a line to the log (flash) from current position (without CRC and comands identifier)
*/
void LineDispatcher::logInput (const ClassifiedLine* inp) const
{
    const char* txt = inp->getLine();
	// Skipping some commands (temporarily)
    if ((txt[0] == '&') ||
        (STRNICMP (txt, "camera ptz ", 11) == 0) ||
        (STRNICMP (txt, "real turn ",  10) == 0) ||
        (STRNICMP (txt, "ps rtk ", 7) == 0))  // RF
    {
		return;
	}

    Log.inputPrint (txt);
}

