/**
* @class FPlanData                                                         
*                                                                   
* @brief Class representing the data made available by the subsystem based on the FlightPlan class.                                   
* This class is an interface between subsystems.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include "PilotIncludes.h"

FPlanData::FPlanData(void):
currentStatus(FPlanConf::Stopped),
currentItem(FlightPlan::ITEM_NOT_SET),
itemType(ITEM_TYPE_NORMAL)
{
    fplanName[0] = 0;
    command[0] = 0;
}

