#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#ifndef PILOT_OS
    #error "Define PILOT_OS in pilot_cfg.h"
#endif

#if (PILOT_TARGET == PT_WIN32) && (PILOT_OS == OS_WIN)

OSNativeW32::OSNativeW32(void):
    OSBase(), _isStarted(false), _nSem(0), _nEvtg(0)
{
	// Initialization of a semaphores and events
    for (int i=0; i<MAX_SEMAPHORES; i++)
        _semTab[i] = 0;

    for (int i=0; i<MAX_EVT_GROUPS; i++)
        for (int j=0; j<EVT_IN_GROUP; j++)
            _evtTab[i][j] = 0;

	// Setting of a higher priority for entire process
    SetPriorityClass (GetCurrentProcess(), HIGH_PRIORITY_CLASS);
};

/** \name Method initializes operating system structures and creates working threads
*/
void OSNativeW32::init (void)
{
	// Dummy method
}

/** \name Method starts multitasking. On Windows OS it stops current thread
*/
void OSNativeW32::start (void)
{
    _isStarted = true;
    SuspendThread (GetCurrentThread());
}

/** \name Method checks whether 'start' function was executed
*/
bool OSNativeW32::isStarted (void) const
{
    return _isStarted;
}

/** \name Method initializes system load counters
*/
void OSNativeW32::statInit (void)
{
	// Dummy method
}

/** \name Method creates new thread
* Detailed description of an interface is in OSBase class.
*/
bool OSNativeW32::taskCreate (TaskProc task, void *arg, void *stackBtm, int stackElements, int threadId, int* const errCode)
{
    if (errCode != NULL)
        *errCode = 0;

    // Creating of a thread
    HANDLE thr = CreateThread(NULL, 0, task , arg, 0, NULL);
    if (thr == NULL) 
    {
        if (errCode != NULL)
            *errCode = static_cast<int>(GetLastError());
        return false;
    }

    return true;
}

/** \name Method suspends current thread
*/
void OSNativeW32::taskSuspend (void)
{
    SuspendThread (GetCurrentThread());
}

/** \name Method creates new smaphore
*/
OSBase::SemHandle OSNativeW32::semCreate (bool locked)
{
    if (_nSem > MAX_SEMAPHORES-2)
        return 0;

    HANDLE sem = CreateSemaphore (NULL, locked ? 0 : 1, 1, NULL);
    if (sem != NULL)
    {
		// Omitting the '_evtgTab[0]' because "handle==0" means an error
        _semTab[++_nSem] = sem;
        return _nSem;
    }

    return 0;
}

/** \name Method performs locking of a semaphore
*/
OSBase::SEMCODE OSNativeW32::semLock (SemHandle sh, int mSec, int* const errCode)
{
    if (sh < 1 || sh > _nSem || _semTab[sh] == 0)
    {
        if (errCode != NULL)
            *errCode = 0;
        return OSBase::SEM_OTHER;
    }

	// Infinite waiting when mSec == 0
    if (mSec == 0)
        mSec = INFINITE;

    DWORD err = WaitForSingleObject (_semTab[sh], static_cast<DWORD>(mSec));
    if (errCode != NULL)
        *errCode = static_cast<int>(err);

    if (err == WAIT_OBJECT_0)
        // Ok
        return OSBase::SEM_OK;
    else if (err == WAIT_TIMEOUT)
        // Timeout
        return OSBase::SEM_TIMEOUT;

    return OSBase::SEM_OTHER;
}

/** \name Method performs releasing of a semaphore
*/
OSBase::SEMCODE OSNativeW32::semUnlock (SemHandle sh, int* const errCode)
{
    if (sh < 1 || sh > _nSem || _semTab[sh] == 0)
    {
        if (errCode != NULL)
            *errCode = 0;
        return OSBase::SEM_OTHER;
    }

    BOOL berr = ReleaseSemaphore (_semTab[sh], 1, NULL);
    int le = static_cast<int>(GetLastError());
    if (errCode != NULL)
        *errCode = le;
    if (berr != 0)
        //  ok
        return OSBase::SEM_OK;
    else
    {
        if (le == ERROR_TOO_MANY_POSTS)
			// Semaphore was not reset (too many releases)
            return OSBase::SEM_FIXED;
    }

    return OSBase::SEM_OTHER;
}

/** \name Method creates an event groups
* Groups of events are not implemented in Windows OS (only single events)
*/
OSBase::EvtHandle OSNativeW32::evtCreate (int* const errCode)
{
    if (errCode != NULL)
        *errCode = 0;

    if (_nEvtg > MAX_EVT_GROUPS-2)
        return 0;

	// Creation of a new event group
    for (int i=0; i<EVT_IN_GROUP; i++)
    {
        HANDLE h = CreateEvent (NULL, TRUE, FALSE, NULL);

		// Handle validation
        if (h == NULL)
        {
            if (errCode != NULL)
                *errCode = static_cast<int>(GetLastError());
            return 0;
        }

		// Omitting the '_nEvtg[0]' because index '0' means an error
        _evtTab[_nEvtg+1][i] = h;
    }

    _nEvtg++;

    return _nEvtg;
}

/** \name Method sets or resets the event flags
*/
OSBase::EVTCODE OSNativeW32::evtPost (EvtHandle eh, EvtMask flags, bool setFlags, int* const errCode)
{
    if (errCode != NULL)
        *errCode = 0;

    if (eh < 1 || eh > _nEvtg)
        return OSBase::EVT_OTHER;

	// Iterating through flags and setting or resetting appropriate event related to those flags
    OSBase::EvtMask m = 1;
    for (int i=0; i<EVT_IN_GROUP; i++)
    {
        if ((m & flags) != 0)
        {
            BOOL b = 0;
            if (setFlags)
                b = SetEvent (_evtTab[eh][i]);
            else
                b = ResetEvent (_evtTab[eh][i]);

            if (b == 0)
            {
                // Error
                if (errCode != NULL)
                    *errCode = static_cast<int>(GetLastError());
                return OSBase::EVT_OTHER;
            }
        }
        m*=2;
    }

    return OSBase::EVT_OK;
}

/** \name Method performs waiting for any event from the group
*/
OSBase::EVTCODE OSNativeW32::evtWaitAny (EvtHandle eh, EvtMask &flags, int* const errCode, bool bConsume)
{
    if (errCode != NULL)
        *errCode = 0;

    if (eh < 1 || eh > _nEvtg)
        return OSBase::EVT_OTHER;

	// Waiting for an occurrence for at least one event
	// 'e' contains an index to the first beginning of the array event causing the return
	// To 'e' a constant value 'WAIT_OBJECT_0', 'WAIT_ABANDONED_0' is added. Method can return WAIT_TIMEOUT or WAIT_FAILED
    DWORD m = WaitForMultipleObjects (EVT_IN_GROUP, _evtTab[eh], FALSE, INFINITE);

    if (m >= WAIT_OBJECT_0 && m - WAIT_OBJECT_0 < EVT_IN_GROUP) // parasoft-suppress  BD-PB-CC "We do not need to see that WAIT_OBJECT == 0"
    {
		// Ok - reset the returned event
        m -= WAIT_OBJECT_0;
        if (bConsume && (ResetEvent (_evtTab[eh][m]) == 0))
        {
            // Error
            if (errCode != NULL)
                *errCode = static_cast<int>(GetLastError());
            return OSBase::EVT_OTHER;
        }

		// Set appropriate bit in the mask and return
        flags = (static_cast<EvtMask>(1) << m);
        
        return OSBase::EVT_OK;
    }

    // Error
    if (errCode != NULL)
        *errCode = static_cast<int>(GetLastError());

    return OSBase::EVT_OTHER;
}

/** \name Method suspends processing of a thread for a specified number of milliseconds
*/
void OSNativeW32::sleepMs (int mSec)
{
    Sleep (mSec);
}

/** \name Method returns number of system ticks since system start
*/
int OSNativeW32::ticks (void)
{
    return static_cast<int>(GetTickCount());
}

/** \name Methods returns minimum amount of free space on the thread's stack
*/
int OSNativeW32::stackMinFree (int taskId)
{
	// Dummy method
    return 0;
}

/** \name Method returns operating system name
*/
const char* OSNativeW32::osName (void)
{
    return "Windows native";
}

#endif // (PILOT_TARGET == PT_WIN32) && (PILOT_OS == OS_WIN)
