#ifndef SYSTEMLOG_H
#define SYSTEMLOG_H

class StorageBase;
class SerialDeviceBase;
class ClassifiedLine;

/** \file
* \brief Declaration of a system and error messages handling class
*/

/** Class is designed to storing system events information to the system log. Class contains a set of dedicated methods handling
* system messages and system error messages. The system messages are handled by "bootPrint()" and "bootPrintf()" methods, in the
* case of entering the system in the startig stage since the multitasking routine starts to work, and "msgPrint()" or "msgPrintf()"
* methods are in use when system is in running stage. System error messages are handled by "errorPrint()" and "errorPrintf()" methods,
* while critical system errors are handled by "abort()" and "tryAbort()" methods. The object of the class (not a pointer) SystemLog is
* created automatically, as a global variable, since "main()" function gets executed (before initialization of uC/OS-II system).
* Constructor of this class cannot contains any system functions. It also cannot inherit form any of class having constructor containing
* system functions.
*/
/// Class implements system and error messages handling
class SystemLog
{
public:
    SystemLog(void);
    
    void Init(void);  ///< Method initilizes the 'SystemLog' subsystem. This routine couldn't be done during 'SystemLog' construction because 'Log' object is created before 'OSInit' has been executed.

	friend class LogManager;	///< Friendship declaration with LogManager, LogManager has access to the private members
    friend class SystemMonitor;	///< Friendship declaration with SystemMonitor, SystemMonitor has access to the private members.

	/** \name Methods defining the log output devices.
	* \{
	*/
    bool setLogFile (StorageBase* stor, const char* fname);
    bool setCommDeviceByNo (int ttyNo);
	///\}

	/** \name Methods of sending text messages to the default system output device (usually console) on the particular platform.
	* These methods can hold an execution of an application. These methods are intended for use only before starting multitasking.
	* If flag 'bootPrintEnable' is set to 'false' these methods do nothing.
	* \{
	*/
    void bootPrint (const char* msg) const;
    void bootPrintf (const char* format, ...) const;
	///\}

    /** \name Methods of sending error messages to the specified output device (one or many devices).
	* Output devices are specified by 'commDevice' and 'logDevice' variables. Methods never holds execution of an application at most ignores the text sent.
	* Methods can be executed only after the multitasking starts.
	* \{
	*/    
	void errorPrint (const char* msg, bool noFlash = false);
    void errorPrint (const char* msg, int val, const char* msg2, bool noFlash = false);
    void errorPrintf (const char* format, ...);
	///\}

	/** \name Methods of sending system messages.
	* \{
	*/
    void msgPrintf (const char* format, ...);
    void msgPrintf (ClassifiedLine* pcl, bool fLogComm, const char* format, ...);
    void msgPrint (ClassifiedLine* pcl, const char* msg);
	///\}
	    
    void tlmPrint (enum TLM_FORMAT id, const char* data, bool fLog, bool fComm) const;	///< Method sends the telemetry data.
    void eventPrint (enum EVT id, const char* data) const;	///< Method sends a notification to the base station
    void controlPrint (int id, const char* data) const;		///< Method sends control data to the external devices.
	void startPrint (void);				///< Method sends the system start messge (log only).
	void dateTimePrint (int uyear, int umonth, int uday, int uhour, int umin, int usec);	///< Method sends the message of data and UTC time (from GPS)
    void inputPrint (const char* msg);	///< Method sends to log echo of the input line. 
	void tryAbort (const char* msg);	///< Method tries to abort the program if 'BootPrintEnable' is set to 'true'.
	void abort (const char* msg);		///< Unconditional program termination.
    void abort (const char* msg, int val, const char* msg2);
	int secureSnprintf (char* buffer, size_t count, const char* format, ...);	///< Method performs the 'snprintf' function protected by semaphore. Method ensures proper string ending. 
    bool bootPrintEnabled;	///< Flag controls 'bootPrint' and 'tryAbort' methods execution. If flag is set to 'false' 'bootPrint' and 'tryAbort' are skipped. By default flag is set to 'true'.

private:
    static const char BEG_SES_TAG[];	///< Beginning section tag of the log.
    static const char DTIME_SES_TAG[];	///< Date and time section tag of the log.
    static const INT16U ABORT_WAIT_MS = 20;	///< System restart delay in milliseconds, allowing to write error message to the log.
//    static const char fileName[];
    bool findSessionOffset (int session, int &offset) const;

	/** \name Member functions executed by LogManager class.
	* \{
	*/
    void logList (ClassifiedLine& cl, const char* prefix);
    void logList (ClassifiedLine& cl);
    bool logGet (ClassifiedLine& cl, const char* prefix, int session, bool boot=false);
    bool logGet (ClassifiedLine& cl);
    bool logGet (ClassifiedLine& cl, int session, bool boot);
    bool logRemoveSession(ClassifiedLine& cl, int session);
	///\}

	/** name Member functions executed by SystemMonitor class
	* \{
	*/
    void logDisable (void);
    void logEnable (void);
	///\}

	/** \name Devices on which error messages will be sent when multitasking mode has been started.
	* By default 'commDevice' and 'logDevice' are set to 'NULL' (no sending). This pointers should be set on after I/O devices has been initialized.
	* \{
	*/
    SerialDeviceBase* commDevice;
    StorageBase* logDevice;
	///\}

    bool logEnabled;
	Semaphore _ioSem;	///< Semaphore controlling access to the standart I/O functions.
	bool timeUpdated;
	char fileNameNew[LINESIZE];
	char fileName[LINESIZE];
	std::vector<std::string> listFile_SD;
};

#endif //SYSTEMLOG_H
