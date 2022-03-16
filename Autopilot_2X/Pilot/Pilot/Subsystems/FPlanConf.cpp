/**                                                                
* @class FPlanConf                                                     
*                                                                   
* @brief Set of variables and errors used by the FPlan, FPlanContainer and FPlanData.
*                                                                   
* Marcin Pczycki @ Flytronic 2008                                   
*/


#include <PilotIncludes.h>

const char  FPlanConf::SUBS_PREFIX[]                = "fp: ";
const char	FPlanConf::RECOVERY_PLAN_COMMAND[]		= "land";
const char	FPlanConf::DEFAULT_PLAN_NAME[]			= "fpdefault";
const char	FPlanConf::STATE_MEM_PLAN_NAME[]		= "fpram";
const char	FPlanConf::STATE_MEM_PLAN_NAME2[]		= "fpram2";



/**
* Returns index error in the array.
*/
int FPlanConf::getErrorPos(int eCode)
{
	for(int i = 0; i < FPlanConf::FPLAN_MAX_ERRORS; i++)
	{
		if(errors[i].eCode == eCode)
			return eCode;
	}
	
	// 0 return is safe because it is set to "Unknown error".
	return 0;
}

/**
* Returns pointer to the message error in the errors[] array.
*/
const char* FPlanConf::getErrorMsg(FPlanConf::FPError eCode)
{
	for(int i = 0; i < FPlanConf::FPLAN_MAX_ERRORS; i++)
	{
		if(errors[i].eCode == eCode)
			return errors[i].eMsg;
	}

	return (errors[FPlanConf::FPErrUnknown].eMsg);
}

/**
* Array of the codes and error messages.
*/
const FPlanConf::ErrorMsg FPlanConf::errors[] = {
	{FPErrUnknown,				"fp: FP20 Unknown error"},	///< This error must be first in the array!
	{FPErrCmdNotRecognized,		"fp: FP21 Command not recognized"},
	{FPErrMissingParams,		"fp: FP22 Syntax error - missing parameters"},
	{FPErrLineListLimitExceeded,"fp: FP23 List is full or line is too long"},
	{FPErrInsertFailure,		"fp: FP24 Failure during inserting command"},
	{FPErrIdExists,				"fp: FP25 Item with specified ID already exists"},
	{FPErrIdDoesntExist,		"fp: FP26 Item with specified ID does not exist"},
	{FPErrSendFailure,			"fp: FP27 Failed to send message"},
	{FPErrWrongIdValue,			"fp: FP28 Wrong itemID value"},
	{FPErrMissingFPlanFile,		"fp: FP29 Missing flight plan file"},
	{FPErrInitFailedSysMonFlag, "fp: FP30 SysMon error flag detected during fplan init"},
	{FPErrCantStopInAir,		"fp: FP31 System cannot be stopped when the plane is airborne"},
	{FPErrCantLoadLastFPlan,	"fp: FP32 Cannot load recently used flight plan"},
	{FPErrCantSaveToStateMem,	"fp: FP33 Failed to save plan modifications in state memory."},
	{FPErrFailedToLoadFPlan,	"fp: FP34 Failed to load flight plan from external memory."},
	{FPErrSystemPanic,			"fp: FP35 Critical system error, run mode is disabled until restart"},
	{FPErrCantAccessFPlanData,	"fp: FP36 Failed to access FlightPlan Data object"},
	{FPErrPlanIsEmpty,			"fp: FP38 Plan is empty"},
	{FPErrCantReadFPRealData,	"fp: FP39 Unable to read FPRealData"},
	{FPErrCantHoldOnGround,		"fp: FP40 OnHold mode is not allowed on the ground"},
	{FPErrNotReady,      		"fp: FP41 System not ready  - cannot switch to <Running> mode"},
    {FPErrCantBreak,            "fp: FP42 Canot break current instruction"},
    {FPErrNoFPlan,              "fp: FP43 Flight Plan not loaded from flash"},
    {FPErrCmdBufFull,           "fp: FP44 Command buffer full"},
	{FPErrCantTakeoff,          "fp: FP45 System is disarm - cannot takeoff"},
	{FPErrNoError,				"fp: ok"}
};
