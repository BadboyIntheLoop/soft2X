/**
* @class FlightPlan                                                        
*                                                                   
* @brief Class storing and controlling flight plan.
*                                                                   
*                                                                   
* Marcin Pczycki (c) Flytronic 2008                                
*/

#ifndef FLIGHTPLAN_H
#define FLIGHTPLAN_H

/** \file
* \brief Subsystem class containing and handling flight plan data
*
* 
*/
class FlightPlan: public ODTObserver, public ODTSubject, public SubsystemBase
{
public:
    FlightPlan(void);

    void	task(const void* pdata);                ///< Operating system task.
    void	linkObserver();                         ///< Link observer objects.
    bool	getFPlanData(FPlanData &fpd);
    void    gotoLine(int lineId);                   ///< Go to given line.
    virtual bool isReadyForTakeoff(ClassifiedLine *cl, bool fLogComm);   ///< True, when uav is ready for takeoff (any errors are send to cl).
    virtual bool putLine (ClassifiedLine &cl);                           ///< Write new line to command queue.

    //consts
    static const int    ITEM_NOT_SET = -1;          ///< currentItem not set.
    static const int    ITEM_NOT_FOUND = -1;        ///< id not changed.
#if LAND_MODE == SEMI_LAND || LAND_MODE == AUTO_LAND
    static const int    PRELAND_ID = 801;           ///< special id for preland sequence
#endif
    static const int    CAMGUIDE_ID = 901;           ///< special id for preland sequence
protected:
    ///  Destructor lock - object should not be destroyed.
    virtual ~FlightPlan(void){};

private:
    //  Group of instructions used in return mode.
    class CommandGroup
    {
    public:
        //  copy constructor 
        CommandGroup (void);
        void reset (void);                     ///<  Set index at begining.
        FPlanData* getCurrentFpd (void);       ///<  Return pointer to the current FPD object.
        bool nextFpd (void);                   ///<  Move index to next command.
        bool setCommand (int index, int id, const char* pCmd, const char* pFPName,
            FPlanData::ITEM_TYPE itemType);    ///<  Set instruction data (must be by order of appearance)
    private:
        static const int MAX_GROUP_ITEMS = 5;
        FPlanData items[MAX_GROUP_ITEMS];     ///<  Array of objects that describe command.
        int  currentIndex;
        int  maxIndex;
    };

    CmdQueue<2>         _cmdq;                          ///< Subsystems input lines (commands) queue. (3 lines)
    FPlanContainer&     _fpContainer;                   ///< Container to store flight plan. It contains controlling functions.
    FPlanData           _recoveryFPD;                   ///< Flight plan instruction shared in recovery mode.
    CommandGroup        _returnBLinkCmdGroup;           ///< Group of commands executed after lost of radio link.
    CommandGroup        _returnLowEnCmdGroup;           ///< Group of commands executed after recognised that is low energy.
    CommandGroup        _returnNoGpsCmdGroup;           ///< Group of commands executed after GPS malfunction.
	CommandGroup		_parachuteLandingCmdGroup;		///< Group of commands executed in parachute landing process 
    CommandGroup		_camGuideCmdGroup;		        ///< Group of commands executed in camera guide process 
#if EMRG_LOSS_ENGINE == 1
	CommandGroup        _lossEngineCmdGroup;   ///< Group of commands executed after loss engine.
	CommandGroup        _focusLandCmdGroup;    ///< Group of commands executed after below emergency altitude.
#endif
    CommandGroup*       _activeCmdGroupPtr;

    FPlanConf::FPStatus	_status;

    // internal variables
	FPlanConf::FPError _err;			///< Store last error code.
	bool            _forceValidate;		///< Setting this flag force to validate after switching to the running mode.
    FPlanConf::MissionMode _missionMode;

#if EMRG_LOSS_ENGINE == 1
	// bool            _focusLandMode;
#endif
	bool            _systemPanic;		///< Critical system malfunction flag. Takeoff is forbiden when the flag i set to true.
	bool            _isFPLoaded;		///< Flight plan loaded from flash flag.
	int             _internalGotoLineId;///< ID of line to go to.
    int             SysMonTag;
    int             FPRealTag;
	int             FPRealBaseChangedTag;
    int             SysMonLandedTag;
	int             SysMonLinkTag;
    int             SysMonLowEnTag;
    int             SysMonNoGpsTag;
#if EMRG_LOSS_ENGINE == 1
	int             SysMonFocusLandTag;
	int             SysMonLossEngineTag;
#endif
    int             InternalGotoTag;
	int             FControlClimbTag;
	int             FControlCruiseTag;
	int				FControlLevelFlightTag;
    int             ExtCmdTag;

    Semaphore       _vSem;  ///< Controll access to the method getFPlanData()


    /// Constructor lock and assignment operator.
    FlightPlan(FlightPlan&);
    FlightPlan& operator=(const FlightPlan&);

    // private methods

    void	useCmdLine(ClassifiedLine& cl);     ///< Execute Flight plan functions according to the received command.

    // Operations on the flight plan elements.
    bool    insertItem(int newId, int afterId, const char* item);
    bool    editItem(int id, const char* item);
    bool    deleteItem(int itemId);
    bool    deleteAllItems();
    bool    listItems(ClassifiedLine& cl);      ///< Sends list of the lines to the communication channel.
    bool    setCurrentItem(int id);             ///< Sets current flight plan line.


    // status
    void    status(ClassifiedLine& cl);         ///< Sends current status to the communication channel.
    const char* statusAsString(FPlanConf::FPStatus stat) const;   ///< Returns current status as a text.
    bool    run();                              ///< Switch flight plan to running mode.
    bool    hold();                             ///< Switch flight plan to onhold mode.
    bool    stop(bool forceStop = false);       ///< Switch flight plan to stopped mode.

    // notifications
    void    sendOK(const ClassifiedLine& cl);               ///< sends fp:ok to the communication channel.
    void    sendError(const ClassifiedLine& cl, int errorCode, const char* errorMessage); ///< Sends error information to the communication channel.


    bool    load();                             ///< Load default flight plan from memory.
    bool    load(const char* name);


	bool	runFromFirst();						///< Sets flight plan to the first point and switch to the running mode.

	int     _flightPhase;  /// 0: UNKNOWN
						   /// 1: CLIMB
						   /// 2: CRUISE
    bool	camGuidePointCompute(const char* command);
    GpsPosition _camGuidePoint;
public:
	int getFlightPhase (void);
    void    getCamGuidePoint(GpsPosition& camGuidePoint);
};

#endif  // FLIGHTPLAN_H

