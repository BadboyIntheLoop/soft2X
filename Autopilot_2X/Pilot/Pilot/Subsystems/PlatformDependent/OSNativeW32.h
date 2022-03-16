#ifndef OSNATIVEW32_H
#define OSNATIVEW32_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#ifndef PILOT_OS
    #error "Define PILOT_OS in pilot_cfg.h"
#endif

#if (PILOT_TARGET == PT_WIN32) && (PILOT_OS == OS_WIN)

/** \file
* \brief Declaration of a class handling specific functions of a Windows operation system
*/

/** Class handles specific functions of the Windows operating system.
* Class dedicated only for Windows OS.
*/
/// Class implements functionality for handling Windows operating system specific functions
class OSNativeW32: public OSBase
{
public:
    OSNativeW32 (void);
    virtual void init (void);	///< Method initializes operating system structures and creates working threads
    virtual void start (void);	///< Method starts multitasking. On Windows OS it stops current thread
    virtual bool isStarted (void) const;	///< Method checks whether 'start' function was executed
    virtual void statInit (void);	///< Method initializes system load counters
    virtual bool taskCreate (TaskProc task, void *arg, void *stackBtm, int stackElements, int taskId, int* const errCode=NULL);	///< Method creates new thread
    virtual void taskSuspend (void); ///< Method suspends current thread
    virtual SemHandle semCreate (bool locked=false);	///< Method creates new smaphore
    virtual SEMCODE semLock (SemHandle sh, int mSec, int* const errCode=NULL);	///< Method performs locking of a semaphore
    virtual SEMCODE semUnlock (SemHandle sh, int* const errCode=NULL);	///< Method performs releasing of a semaphore
    virtual EvtHandle evtCreate (int* const errCode=NULL);	///< Method creates an event groups
    virtual EVTCODE evtPost (EvtHandle eh, EvtMask flags, bool setFlags, int* const errCode=NULL);	///< Method sets or resets the event flags
    virtual EVTCODE evtWaitAny (EvtHandle eh, EvtMask &flags, int* const errCode=NULL, bool bConsume=true);	///< Method performs waiting for any event from the group
    virtual void sleepMs (int mSec); ///< Method suspends processing of a thread for a specified number of milliseconds
    virtual int ticks (void);	///< Method returns number of system ticks since system start
    virtual int stackMinFree (int taskId);	///< Methods returns minimum amount of free space on the thread's stack
    virtual const char* osName (void); ///< Method returns operating system name

protected:
    // Destructor is disabled - object should never be destroyed
    virtual ~OSNativeW32(void){};

private:
    // Copy constructor is disabled
    OSNativeW32(OSNativeW32&);
	// Copy operator is disabled
    OSNativeW32& operator=(const OSNativeW32&);
    
    bool    _isStarted;                             ///< Flag indicates execution of a 'start' function
    HANDLE  _semTab[MAX_SEMAPHORES];                ///< Array with pointers to the structures of Windows OS semaphores
	int     _nSem;                                  ///< Number of created semaphores
    HANDLE  _evtTab[MAX_EVT_GROUPS][EVT_IN_GROUP];  ///< Array of pointers to the Windows event handles
    int     _nEvtg;                                 ///< Number of created event groups
};

#endif // (PILOT_TARGET == PT_WIN32) && (PILOT_OS == OS_WIN)

#endif  // OSNATIVEW32_H
