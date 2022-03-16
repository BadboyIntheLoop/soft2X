/**
* @class FPlanData                                                         
*                                                                   
* @brief Class representing the data made available by the subsystem based on the FlightPlan class.                                   
* This class is an interface between subsystems.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef FPLANDATA_H
#define FPLANDATA_H


class FPlanData
{
public:
    enum ITEM_TYPE
    {
        ITEM_TYPE_NORMAL,   ///< An enum constant representing the item type normal option
#if EMRG_LOSS_ENGINE == 1
		ITEM_TYPE_FOCUS_LAND,
#endif
        ITEM_TYPE_RETURN	///< An enum constant representing the item type return option
    };

    FPlanData(void);

	FPlanConf::FPStatus		currentStatus;  ///< Current status of the subsystem.
	int						currentItem;	///< id of the current element.
	char					fplanName[FPlanConf::FPLAN_NAME_LENGTH];	///<Current flight plan name.
	char					command[FPlanConf::FPLAN_LINE_LENGTH];  ///< Current instruction.
    ITEM_TYPE               itemType;   ///< Type of the instruction.
};

#endif  //  FPLANDATA_H
