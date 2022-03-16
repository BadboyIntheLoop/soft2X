#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

StorageFactory::StorageFactory(void)
{
}

StorageFactory::~StorageFactory(void)
{
}

#if PILOT_TARGET == PT_WIN32

/** \name Method creates an object used to store subsytem's configuration parameters (non volatile).
* Each subsystem creates it's own instance of this object.
*/
StorageBase* StorageFactory::createConfigMemory (bool singleThread)
{
     return new StorageW32 ("..\\StorageW32", false, singleThread);
}

/** \name Method creates an object used to store a flight plan.
* This object is used only by a FPlan subsystem.
*/
StorageBase* StorageFactory::createFPlanMemory (void)
{
    return new StorageW32 ("..\\StorageW32");
}

/** \name Method creates an object used to store a log.
* This object is used only by a Log subsystem.
*/
StorageBase* StorageFactory::createLogMemory (void)
{
	// 'true' parameters enables asynchronous handling of an 'append' function (it requires to run additional task)
    return new StorageW32 ("..\\StorageW32", true);
}

#elif PILOT_TARGET == PT_HARDWARE
StorageBase* StorageFactory::createConfigMemory (bool singleThread)
{
    return new StorageNIOSFlash (false, singleThread);
}

StorageBase* StorageFactory::createFPlanMemory (void)
{
    return new StorageNIOSFlash ();
}

StorageBase* StorageFactory::createLogMemory (void)
{
    // 'true' parameters enables asynchronous handling of an 'append' function (it requires to run additional task)
//    return new StorageNIOSFlash (true, false);
	return new StorageSD (true, true);
}

#endif  // PILOT_TARGET
