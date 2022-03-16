/**                                                                
* @class FPlanConf                                                     
*                                                                   
* @brief Set of variables and errors used by the FPlan, FPlanContainer and FPlanData.
*                                                                   
* Marcin Pczycki @ Flytronic 2008                                   
*/

#ifndef FPLANCONF_H
#define FPLANCONF_H

/** \file
* \brief Set of constants and enums used in the FPlan, FPlanContainer and FPlanData classes
*
*
*/
class FPlanConf
{
public:
	
	static const int	FPLAN_LIST_SIZE = 130;			///< Number of elements in flight plan.
	static const int	FPLAN_MAX_ID_VALUE = 999999;	///< Max value of ID.
    static const int	FPLAN_LINE_LENGTH = 120;		///< Leght of the single line of the flight plan.
	static const int	FPLAN_SENDLINE_DELAY = 4;		///< At which line the system has to wait before sending the next one.
	static const int	FPLAN_SENDLINE_DELAY_VALUE = 2; ///< Time to wait in miliseconds to wait before sending a line.
	static const int	FPLAN_MAX_ERRORS = 26;			///< Leght of the array storing structure with error code and messages.
	static const int	FPLAN_NAME_LENGTH = 20;			///< Lenght of the flight plan name.

    static const char   SUBS_PREFIX[];
	static const char	STATE_MEM_PLAN_NAME[FPlanConf::FPLAN_NAME_LENGTH];
	static const char	STATE_MEM_PLAN_NAME2[FPlanConf::FPLAN_NAME_LENGTH];
	static const char	RECOVERY_PLAN_COMMAND[FPlanConf::FPLAN_LINE_LENGTH];
	static const char	DEFAULT_PLAN_NAME[FPlanConf::FPLAN_NAME_LENGTH];
	static const int	RECOVERY_PLAN_ID = 1;

	/// FlightPlan status
	enum FPStatus													
	{
		UNDEFINED = 0,
		Running = 1,
		OnHold = 2,
		Stopped = 3
	};

	/// Mission mode
	enum class MissionMode
	{
		RECOVERY = 0,
		RTL = 1,
		PARACHUTE = 2,
		CAMGUIDE = 3,
		FOCUSLAND = 4,
		AUTO = 5
	};

	struct ErrorMsg
	{
		int			eCode;
		char		eMsg[FPlanConf::FPLAN_LINE_LENGTH];	
	};


	/// Enum values indicates subsequent messages in error array.
	enum FPError
	{
		FPErrNoError = -1,
		FPErrUnknown = 0,
		FPErrCmdNotRecognized = 1,
		FPErrMissingParams = 2,
		FPErrLineListLimitExceeded = 3,
		FPErrInsertFailure = 4,
		FPErrIdExists = 5,
		FPErrIdDoesntExist = 6,
		FPErrSendFailure = 7,
		FPErrWrongIdValue = 8,
		FPErrMissingFPlanFile = 9,
		FPErrInitFailedSysMonFlag = 10,
		FPErrCantStopInAir = 11,
		FPErrCantLoadLastFPlan = 12,
		FPErrCantSaveToStateMem = 13,
		FPErrFailedToLoadFPlan = 14,
		FPErrSystemPanic = 15,
		FPErrCantAccessFPlanData = 16,
		FPErrPlanIsEmpty = 18,
		FPErrCantReadFPRealData = 19,
		FPErrCantHoldOnGround = 20,
        FPErrNotReady = 21,
        FPErrCantBreak = 22,
        FPErrNoFPlan = 23,
        FPErrCmdBufFull = 24,
		FPErrCantTakeoff = 25
	};

    // error handling.
    static const		ErrorMsg errors[FPlanConf::FPLAN_MAX_ERRORS];   ///< Container with errors.
    static int			getErrorPos(int eCode);							///< Find error by its code.
    static const char*	getErrorMsg(FPlanConf::FPError eCode);			///< Returns pointer to the message error in the array - errors[]

};



#endif  
