#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

/** \file
* \brief Declaration of a camera handler interface
*/

/** Class is a specialized interface to handle the camera (gimbal). Class implements the observer pattern
* by inheriting base classes ODTSubject and ODTObserver representing the objects of observation and observer respectively.
* Thanks to this CameraManager is both the observer and object of observation, in particular, it may watch itself.
* The registration of the observed objects takes place in "linkObserver()" method, and the registered and observed system
* components are physical state (PState), flight plan realizer (FPReal), system monitor (SysMon), and the camera manager component itself.
*/
/// Class implements handler interface for a camera
class CameraManager: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:
    CameraManager(void);
    void task(const void* pdata);               ///< System tasks handler method
    void linkObserver(void);                    ///< Observed objects linker method
    virtual bool isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm); ///< Subsystem readiness for takeoff checker method
    virtual bool putLine (ClassifiedLine &cl);                         ///< New command line to command queue inserter method

protected:
    // Destructor is disabled - object should not be destroyed
    virtual ~CameraManager(void){};

private:
	/** \name Error messages
	* \{
	*/
    static const char ERR_OK[];
    static const char ERR_UCOMMAND[];
	///\}

	/// Sensors configuration file name
    static const char CAMERA_CONF_FILE_NAME[];
	/// Subsystem name used as a prefix
    static const char SUBS_PREFIX[];

	static const char ERR_CLOAD[];
	static const char ERR_CSAVE[];
	static const char MSG_NO_CONFIG[];
    static const char ERR_CMD_BUF_FULL[];

	/** \name User Action Flag
	* Bit flags passed when definig the parameter (insertion) and returned after the particular parameter has been set.
    * Flags identify the function wchich must be executed after setting the parameter.
	* \{
	*/
    static const unsigned int UAF_LED       =  1; ///< Switch on/off the LED indicator
	///\}

    /** \name Tags obtained when registering of the observed object
	* \{
	*/
    int _sysMonLinkTag;
    int _extCmdTag;
	///\}

    CmdQueue<2> _cmdq;              ///< Subsystems input command line queue (2 lines)
    FParser _parser;                ///< Parser of a textual commands form communication channel
	Camera  _camera;

    bool _isConfigLoaded;           ///< Flag indicates that configuration has been loaded from flash

    void useCmdLine (ClassifiedLine &cl);   ///< Method interprets and executes 'cl' commands

	/// CameraManager subsystem configuration data class
    class ConfigData
    {
    public:
        int   commCamDivisor;		///< Number of samples after which data from camera will be send to the communication channel (0 to n, 0 disables sending)
        int   logCamDivisor;		///< Number of samples after which data from camera will be send to log (0 to n, 0 disables sending)
        int   commTrgDivisor;		///< Number of samples after which data of target will be send to the communication channel (0 to n, 0 disables sending)
        int   logTrgDivisor;		///< Number of samples after which data of target will be send to log (0 to n, 0 disables sending)
        int   ledState;            ///< State of a LED indicator (0-off, 1-on, 2-automatically activated when connection is lost)

		ConfigData (void);
        void setDefault (void);    ///< Default parameter setter method
    };

    // Copy constructor is disabled
    CameraManager(CameraManager&);
	// Copy operator is disabled
    CameraManager& operator=(const CameraManager&);

	ConfigData   _conf;            ///< Subsystem configuration data
	StorageBase* _confMem;         ///< Subsystem configuration memory
    ParameterNames* _confNames;    ///< Object associates names with the configuration parameters
	bool confLoad (void);          ///< Method loads subsystem configuration
    bool confSave (void);          ///< Method saves subsystem configuration
    
    ParameterNames::ERRCODE setParameters (const char* nameValueItems);///< Parameters setter method
    bool getParameter(const char* pName, ClassifiedLine& cl) const;
    void setParamUserAction (unsigned int flags);                 ///< Method performs action when parameter value has been changed
};

#endif  // CAMERAMANAGER_H
