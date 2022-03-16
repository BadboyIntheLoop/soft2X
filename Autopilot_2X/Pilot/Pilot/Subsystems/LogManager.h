#ifndef LOGMANAGER_H
#define LOGMANAGER_H

/** \file
* \brief Declaration of system log handler class
*/

/** Class is designed to managing the system log. Class allows to representation of a system log session directory, getting of a content for specified session,
* erasing entire log content by commands.
*/
/// Class implements system log handling
class LogManager: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:
    LogManager(void);
    void task(void* pdata);                     ///< System tasks handler method
    void linkObserver();                        ///< Observed objects linking method
    virtual bool putLine (ClassifiedLine &cl);  ///< Method puts new line to the command queue

protected:
	/// Destructor is disabled (object should never be destroyed)
    virtual ~LogManager(void){};

private:
	/** \name Error messages
	*\{
	*/
    static const char ERR_OK[];
    static const char ERR_UCOMMAND[];
    static const char ERR_SES_NOT_FOUND[];
    static const char ERR_CLEAR_LOG[];
    static const char SUBS_PREFIX[];
    static const char ERR_CMD_BUF_FULL[];
    static const char ERR_LOG_GET[];
    static const char ERR_LOG_REMOVE[];
    static const char ERR_LOG_SPACE[];
	///\}

    CmdQueue<2> _cmdq;      ///< Queue of a subsystem input lines (commands) (two lines)
    int         _extCmdTag; ///< Tag received during registration of a observed object
    FParser     _parser;    ///< Parser of a textual commands from communication channel

    /// Copy constructor is disabled
    LogManager(LogManager&);
	/// Copy operator is disabled
    LogManager& operator=(const LogManager&);

    void useCmdLine (ClassifiedLine &cl);   ///< Method interprets and executes command
};

#endif  // LOGMANAGER_H
