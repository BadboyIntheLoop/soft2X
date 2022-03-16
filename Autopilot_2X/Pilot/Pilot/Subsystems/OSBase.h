#ifndef OSBASE_H
#define OSBASE_H

/** \file
* \brief Base class for classes handles operating systems specific functoinality
*/

/** Base class (an interface) for classes handles operating systems specific functoinality. Class is Win32 and NIOS-II compatible.
*/
///
class OSBase
{
public:
    typedef int SemHandle;
    typedef int EvtHandle;
    typedef unsigned int EvtMask;

    static const int MAX_SEMAPHORES = 40;   ///< Maximum amount of system semaphores
    static const int MAX_EVT_GROUPS = 30;   ///< Maximum amount of bit events group
    static const int EVT_IN_GROUP   = 32;   ///< Number of event flags in single group

	/** \name Error codes returned by handler methods of semaphores
	* \{
	*/
    enum SEMCODE
    {
        SEM_OK = 0,     ///< All correct
        SEM_FIXED,      ///< Ok, but setting the amount of semaphores was required (too many unlocks)
        SEM_TIMEOUT,    ///< Semaphore unlocking timeout
        SEM_OTHER       ///< Other errors
    };
	///\}

	/** \name Error codes returned by event handler methods
	* \{
	*/
    enum EVTCODE
    {
        EVT_OK = 0,     ///< Ok
        EVT_TIMEOUT,    ///< Timeout waiting for event
        EVT_OTHER       ///< Other errors
    };
	///\}

    /// Pure virtual method. Method initializes the structures of the operating system and creates working threads.
    virtual void init (void) = 0;

	/// Pure virtual method. Method explicitly starts multitasking in the operating system. Method stops current thread.
    virtual void start (void) = 0;

	/// Pure virtual method. Method returns bool value whether multitasking has been started
    virtual bool isStarted (void) const = 0;

	/// Pure virtual method. Method initializes the system load counters
    virtual void statInit (void) = 0;

	/** \name Pure virtual method. Method creates new thread of the operating system. Thread is created as a immediately running.
	* \param 'taskProc' - pointer to a handler function
	* \param 'arg' - argument of a handler function
	* \param 'stackBtm' - pointer to the beginning of the stack
	* \param 'stackElements' - amount of a stack items
	* \param 'taskId' - task uid, it is also an relative task priority (5 to 250 - the lowest number the highest priority)
	* \param 'errCode' - operating system specific error code. If NULL no error is returned. NOTE: this argument is used only for statistic purposes, do not use it to control the programs flow.
	* \return 'true' if task was created successfully, 'false' otherwise.
	*/
    virtual bool taskCreate (TaskProc task, void *arg, void *stackBtm, int stackElements, int taskId, int* const errCode=NULL) = 0;

    /// Pure virtual method. Method stops current thread.
    virtual void taskSuspend (void) = 0;

	/** \name Pure virtual method. Method creates new binary semaphore (with value of 0 or 1).
	* Semaphore is initialized with value of 1 which means that protected resource is accessible.
	* Method returns handle to the semaphore or '0' is any error occured.
	*/
    virtual SemHandle semCreate (bool locked=false) = 0;

	/** \name Pure virtual method. Method blocks access to the resource.
	* \param 'sh' - handle to a semaphore
	* \param 'mSec' - number of milliseconds to wait for the release of semaphore in use
	* \param 'errCode' - operation system specific error code. If NULL error code is not returned. NOTE: this argument is used only for statistic purposes, do not use it to control the programs flow.
	* \return 'SEM_OK' - on success, 'SEM_TIMEOUT' - semaphore unlocking timeout, 'SEM_OTHER' - other error
	*/
    virtual SEMCODE semLock (SemHandle sh, int mSec, int* const errCode=NULL) = 0;

	/** \name Pure virtual method. Method unlocks the semaphore.
	* \param 'sh' - handle to a semaphore
	* \param 'errCode' - operating system specific error code. If NULL no error is returned. NOTE: this argument is used only for statistic purposes, do not use it to control the programs flow.
	* \return 'SEM_OK' - on success, 'SEM_FIXED' - ok, but setting the amount of semaphores was required (too many unlocks), 'SEM_OTHER' - other error
	*/
    virtual SEMCODE semUnlock (SemHandle sh, int* const errCode=NULL) = 0;

	/** \name Pure virtual method. Method creates a group of binary events. The event flags creating this group are resetted.
	* \param 'errCode' - operating system specific error code. If NULL no error is returned. NOTE: this argument is used only for statistic purposes, do not use it to control the programs flow.
	* \return A handle to a group of binary events or '0' if error occured.
	*/
    virtual EvtHandle evtCreate (int* const errCode=NULL) = 0;

	/** \name Pure virtual method. Method sets/resets an event occurrence flag.
	* \param 'eh' - a handle to the group of binary events
	* \param 'flags' - binary flags correcponding to event states
	* \param 'setFlags' - flags modificaion mode: 'true' - set flag mode, 'false' - reset flag mode
	* \param 'errCode' - operating system specific error code. If NULL no error is returned. NOTE: this argument is used only for statistic purposes, do not use it to control the programs flow.
	* \return 'EVT_OK' - ok, 'EVT_OTHER' - other error
	*/
    virtual EVTCODE evtPost (EvtHandle eh, EvtMask flags, bool setFlags, int* const errCode=NULL) = 0;

    /** \name Pure virtual method. Method waits (without timeouting) on an occurrence of any event from specified events group. Method resets flags of the events that have occurred.
	* \note There is possibility of simulaneously occurrence of a several events. In such a case, the effect is dependent on the operating system. In some operating systems
	* all flags are set instantly and reseting the apropriate events flags, in the others only single bit is set (and reseting appropriate events flag) and the event is handled by
	* the next call of a function.
	* \param 'eh' - a handle to the events group
	* \param 'flags' - returned flags corresponding to the occurred events
	* \param 'errCode' - operating system specific error code. If NULL no error is returned. NOTE: this argument is used only for statistic purposes, do not use it to control the programs flow.
	* \param 'bConsume' - 'true' - the set event flag is resetted, 'false' - flag remains unchanged
	* \return 'EVT_OK' - ok, 'EVT_OTHER' - other error
	*/
    virtual EVTCODE evtWaitAny (EvtHandle eh, EvtMask &flags, int* const errCode=NULL, bool bConsume=true) = 0;

	/** \name Pure virtual method. Method sleeps the thread execution for a specified time (in milliseconds).
	* Specified time is rounded down or up to the nearest multiple of a system ticks.
	*/
    virtual void sleepMs (int mSec) = 0;

	/** \name Pure virtual method. Method returns number of a system ticks since start of a system.
	*/
    virtual int ticks (void) = 0;

	/** \name Pure virtual method. Method returns minimum amount of stack free space required to run a system for a specified thread or '-1' if error occured.
	* \param 'taskId' - a thread identifier returned by 'taskCreate' method
	*/
    virtual int stackMinFree (int taskId) = 0;

	/// Pure virtual method. Method returns an operation system name.
    virtual const char* osName (void) = 0;

protected:
    /// Default constructor is disabled
    OSBase(void){};
    /// Destructor is disabled (releasing the object by a pointer to the base class)
    virtual ~OSBase(void){}; /* parasoft-suppress  OOP-31 "The NIOS-II environment reports warning when it's not virtual" */

private:
    // Copy constructor is disabled
    OSBase(OSBase&);
	// Copy operator is disabled
    OSBase& operator=(const OSBase&);
};

#endif  // OSBASE_H
