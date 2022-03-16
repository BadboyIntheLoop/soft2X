#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#ifndef PILOT_OS
    #error "Define PILOT_OS in pilot_cfg.h"
#endif

#if (PILOT_TARGET != PT_WIN32) || (PILOT_OS == OS_UCOSII)

OSUcosII::OSUcosII(void):
    OSBase(), _nSem(0), _nEvtg(0)
{
    // Checking the conformity of the number of events in the group with the uC/OS-II configuration
    if (EVT_IN_GROUP != OS_FLAGS_NBITS)
        Log.abort ("Critical Error: OSUcosII_1.");

	// Initialization of arrays of semaphores and events
    for (int i=0; i<MAX_SEMAPHORES; i++)
        _semTab[i] = (OS_EVENT*)0;

    for (int i=0; i<MAX_EVT_GROUPS; i++)
        _evtgTab[i] = (OS_FLAG_GRP*)0;
};

/** \name Method initializes operating system structures and creates working threads.
* \note 'OSInit()' function cannot be executed at the beginning of the constructor. For unknown reasons it demages the heap!
* Thats why separate function 'init()' was implemented which should be executed after creation of this object.
*/
void OSUcosII::init (void)
{
#if PILOT_TARGET == PT_WIN32
	// uC/OS-II initialization (NIOS2IDE makes it automatically)
    OSInit();
#else 
#if (APP_OSIII_ENABLED == DEF_ENABLED)
    OS_ERR  os_err;
#endif

    UCOS_LowLevelInit();

    CPU_Init();
    Mem_Init();

#if (APP_OSIII_ENABLED == DEF_ENABLED)
    OSInit(&os_err);
#else
    OSInit();
#endif

#endif
}

/** \name Method starts the multitasking.
*/
void OSUcosII::start (void)
{
    OSStart();
}

/** \name Method checks whether 'start' function was executed.
*/
bool OSUcosII::isStarted (void) const
{
    return (OSRunning == OS_TRUE);
}

/** \name Method initializes system load counters. It applies only to a NIOS-II platform.
*/
void OSUcosII::statInit (void)
{
#if PILOT_TARGET != PT_WIN32
    OSStatInit ();
#endif  // PILOT_TARGET
}

/** \name Method creates new thread.
* Detailed description of an interface is in a OSBase class.
*/
bool OSUcosII::taskCreate (TaskProc task, void *arg, void *stackBtm, int stackElements, int taskId, int* const errCode)
{
    OS_STK* bos = static_cast<OS_STK*>(stackBtm);   // Pointer to the beginning of a stack (element 0)
    OS_STK* tos = bos + stackElements - 1;          // Pointer to the last element on a stack

    INT8U tcErr = OSTaskCreateExt (task, arg, tos, static_cast<INT8U>(taskId), static_cast<INT16U>(taskId),
        bos, static_cast<INT32U>(stackElements), (void *)0,OS_TASK_OPT_STK_CHK);

    if (errCode != NULL)
        *errCode = static_cast<int>(tcErr);
    return (tcErr == OS_ERR_NONE);
}

/** \name Method suspends processing of a current thread.
*/
void OSUcosII::taskSuspend (void)
{
    OSTaskSuspend(OS_PRIO_SELF);
}

/** \name Method creates new semaphore.
*/
OSBase::SemHandle OSUcosII::semCreate (bool locked)
{
    if (_nSem > MAX_SEMAPHORES-2)
        return 0;

    OS_EVENT* sem = OSSemCreate(static_cast<INT16U>(locked ? 0 : 1));
    if (sem != (OS_EVENT*)0)
    {
        // Omitting the '_evtgTab[0]' because "handle==0" means an error
        _semTab[++_nSem] = sem;
        return _nSem;
    }

    return 0;
}

/** \name Method performs locking of a semaphore
*/
OSBase::SEMCODE OSUcosII::semLock (SemHandle sh, int mSec, int* const errCode)
{
    if (sh < 1 || sh > _nSem || _semTab[sh] == 0)
    {
        if (errCode != NULL)
            *errCode = 0;
        return OSBase::SEM_OTHER;
    }

	// Calculation of a ticks from milliseconds
    INT16U ticks = static_cast<INT16U>((OS_TICKS_PER_SEC * mSec) / 1000);

	// Protection against disabling the timeout (ticks==0)
    if (mSec > 0 && ticks == 0)
        ticks = 1;

    INT8U err = 0;
    OSSemPend (_semTab[sh], ticks, &err);
    if (errCode != NULL)  
        *errCode = err;

	if (err == OS_ERR_NONE)
	{
        return OSBase::SEM_OK;
	}
    else if (err == OS_ERR_TIMEOUT)
    {
        return OSBase::SEM_TIMEOUT;
    }

    return OSBase::SEM_OTHER;
}

/** \name Method performs releasing of a semaphore.
*/
OSBase::SEMCODE OSUcosII::semUnlock (SemHandle sh, int* const errCode)
{
    if (sh < 1 || sh > _nSem || _semTab[sh] == 0)
    {
        if (errCode != NULL)
            *errCode = 0;
        return OSBase::SEM_OTHER;
    }

    // Releasing of the semaphore
    INT8U osErr = OSSemPost (_semTab[sh]);
    if (osErr != OS_ERR_NONE)
    {
        if (errCode != NULL)
            *errCode = osErr;
        return OSBase::SEM_OTHER;
    }

	// Checking whether the semaphore was not too much times unlocked (post)
    OS_SEM_DATA data;
    osErr = OSSemQuery (_semTab[sh], &data);
    if (errCode != NULL)
        *errCode = osErr;

    if (osErr != OS_ERR_NONE)
        return OSBase::SEM_OTHER;

    if (data.OSCnt > 1)
    {
		// Semaphore was too many times enable - initialization by 1
        OSSemSet (_semTab[sh], 1, &osErr);
        if (errCode != NULL)
            *errCode = osErr;

        if (osErr != OS_ERR_NONE)
            return OSBase::SEM_OTHER;

        return OSBase::SEM_FIXED;
    }

    return OSBase::SEM_OK;
}

/** \name Method creates an event groups.
*/
OSBase::EvtHandle OSUcosII::evtCreate (int* const errCode)
{
    if (_nEvtg > MAX_EVT_GROUPS-2)
    {
        if (errCode != NULL)
            *errCode = 0;
        return 0;
    }
    
    INT8U perr = 0;
	// Creating of the new group of a system uC/OS-II flags
    OS_FLAG_GRP* evtg = OSFlagCreate (0, &perr);
    if (errCode != NULL)
        *errCode = perr;

    if (perr == OS_ERR_NONE)
    {
        // Omitting the '_evtgTab[0]' because "handle==0" means an error
        _evtgTab[++_nEvtg] = evtg;
        return _nEvtg;
    }

    return 0;
}

/** \name Method sets or resets the event flags.
*/
OSBase::EVTCODE OSUcosII::evtPost (EvtHandle eh, EvtMask flags, bool setFlags, int* const errCode)
{
    if (eh < 1 || eh > _nEvtg || _evtgTab[eh] == 0)
    {
        if (errCode != NULL)
            *errCode = 0;
        return OSBase::EVT_OTHER;
    }

    INT8U perr = 0;
    OSFlagPost (_evtgTab[eh], flags, static_cast<INT8U>(setFlags ? OS_FLAG_SET : OS_FLAG_CLR), &perr);

    // Setting of an operating system specific error code
    if (errCode != NULL)
        *errCode = perr;

    if (perr == OS_ERR_NONE)
        return OSBase::EVT_OK;

    return OSBase::EVT_OTHER;
}

/** \name Method performs waiting for any event from the group.
*/
OSBase::EVTCODE OSUcosII::evtWaitAny (EvtHandle eh, EvtMask &flags, int* const errCode, bool bConsume)
{
    if (eh < 1 || eh > _nEvtg || _evtgTab[eh] == 0)
    {
        if (errCode != NULL)
            *errCode = 0;
        return OSBase::EVT_OTHER;
    }

    INT8U perr = 0;
    INT8U cons = (bConsume ? OS_FLAG_CONSUME : 0);
    flags = static_cast<EvtMask>(OSFlagPend (_evtgTab[eh], ~(OS_FLAGS)0, OS_FLAG_WAIT_SET_ANY + cons, 0, &perr));

	// Setting of an operating system specific error code
    if (errCode != NULL)
        *errCode = perr;

    if (perr == OS_ERR_NONE)
        return OSBase::EVT_OK;

    return OSBase::EVT_OTHER;
}

/** \name Method suspends processing of a thread for a specified number of milliseconds.
*/
void OSUcosII::sleepMs (int mSec)
{
	// Calculation of a ticks from milliseconds
    INT16U ticks = static_cast<INT16U>((OS_TICKS_PER_SEC * mSec) / 1000);

    // Waiting
    OSTimeDly (ticks);
}

/** \name Method returns number of system ticks since system start.
*/
int OSUcosII::ticks (void)
{
    return static_cast<int>(OSTimeGet());
}

/** \name Methods returns minimum amount of free space on the thread's stack.
*/
int OSUcosII::stackMinFree (int taskId)
{
    OS_STK_DATA stkData;

    INT8U err = OSTaskStkChk (static_cast<INT8U>(taskId), &stkData);
    if (err == OS_ERR_NONE)
        return static_cast<int>(stkData.OSFree);

    return -1;
}

/** \name Method returns the operating system name.
*/
const char* OSUcosII::osName (void)
{
#if (PILOT_TARGET == PT_WIN32)
    return "Windows hosted RTOS";
#else
    return "RTOS native";
#endif  //  (PILOT_TARGET == PT_WIN32)
}

#endif // (PILOT_TARGET != PT_WIN32) || (PILOT_OS == OS_UCOSII)
