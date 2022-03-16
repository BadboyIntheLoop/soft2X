#ifndef OSUCOSII_H
#define OSUCOSII_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#ifndef PILOT_OS
    #error "Define PILOT_OS in pilot_cfg.h"
#endif

#if (PILOT_TARGET != PT_WIN32) || (PILOT_OS == OS_UCOSII)

/** \file
* \brief Declaration of a class handling specific functions of uC/OS-II operating system
*/

/** Class handles specific functions of the uC/OS-II operating system. Universal class for win32 and Nios-II.
*/
/// Class implements functionality for handling uC/OS-II operating system specific functions
class OSUcosII: public OSBase
{
public:
    OSUcosII (void);
    virtual void init (void);	///< Method initializes operating system structures and creates working threads
    virtual void start (void);	///< Method starts the multitasking
    virtual bool isStarted (void) const;	///< Method checks whether 'start' function was executed
    virtual void statInit (void);	///< Method initializes system load counters
    virtual bool taskCreate (TaskProc task, void *arg, void *stackBtm, int stackElements, int taskId, int* const errCode=NULL);	///< Method creates new thread
    virtual void taskSuspend (void);	///< Method suspends processing of a current thread
    virtual SemHandle semCreate (bool locked=false);	///< Method creates new semaphore
    virtual SEMCODE semLock (SemHandle sh, int mSec, int* const errCode=NULL);	///< Method performs locking of the semaphore
    virtual SEMCODE semUnlock (SemHandle sh, int* const errCode=NULL);	///< Method performs releasing of the semaphore
    virtual EvtHandle evtCreate (int* const errCode=NULL);	///< Method creates an event groups
    virtual EVTCODE evtPost (EvtHandle eh, EvtMask flags, bool setFlags, int* const errCode=NULL);	///< Method sets or resets the event flags
    virtual EVTCODE evtWaitAny (EvtHandle eh, EvtMask &flags, int* const errCode=NULL, bool bConsume=true);	///< Method performs waiting for any event from the group
    virtual void sleepMs (int mSec);	///< Method suspends processing of a thread for a specified number of milliseconds
    virtual int ticks (void);	///< Method returns number of system ticks since system start
    virtual int stackMinFree (int taskId);	///< Methods returns minimum amount of free space on the thread's stack
    virtual const char* osName (void);	///< Method returns the operating system name

protected:
    // Destructor is disabled - object should never be destroyed
    virtual ~OSUcosII(void){};

private:
    // Copy constructor is disabled
    OSUcosII(OSUcosII&);
	// Copy operator is disabled
    OSUcosII& operator=(const OSUcosII&);

    OS_EVENT* _semTab[MAX_SEMAPHORES];      ///< Array with pointers to the structures of uC/OS-II semaphores
    int _nSem;                              ///< Number of created semaphores
    OS_FLAG_GRP* _evtgTab[MAX_EVT_GROUPS];  ///< Array of pointers to the structures of uC/OS-II event groups
    int _nEvtg;                             ///< Number of created event groups
};

#endif // (PILOT_TARGET != PT_WIN32) || (PILOT_OS == OS_UCOSII)

#endif  // OSUCOSII_H
