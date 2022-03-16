#include <PilotIncludes.h>

#if OS_FLAGS_NBITS != 32
    #error "Define OS_FLAGS_NBITS 32 in os_cfg.h"
#endif

/// Array of mappings the index of a bit to a number
const OSBase::EvtMask ODTObserver::FLAGMASK[] = {
    1u<<0, 1u<<1, 1u<<2, 1u<<3, 1u<<4, 1u<<5, 1u<<6, 1u<<7,
    1u<<8, 1u<<9, 1u<<10, 1u<<11, 1u<<12, 1u<<13, 1u<<14, 1u<<15,
    1u<<16, 1u<<17, 1u<<18, 1u<<19, 1u<<20, 1u<<21, 1u<<22, 1u<<23,
    1u<<24, 1u<<25, 1u<<26, 1u<<27, 1u<<28, 1u<<29, 1u<<30, 1u<<31
};


ODTObserver::ODTObserver(void):
    nRegistered(0)
{
	// Creation of a new group of MicroC/OS-II system flags
    int perr = 0;
    _flagGrp = Os->evtCreate (&perr);
    if (_flagGrp == 0)
        // 'abort' terminates application
        Log.abort ("Critical Error: ODTObserver_ODTObserver_1 [", perr, "].");
}

/** \name Method notifies of occurrence of an aspect (executed by an observed object)
* \param 'obsData' - parameter passed to the observed object during it's attaching to the observer. Parameter is used to identify sender and an aspect.
*/
void ODTObserver::update (int obsData) const
{
    int perr = 0;
    OSBase::EVTCODE err = Os->evtPost (_flagGrp, FLAGMASK[obsData], true, &perr);
    if (err != OSBase::EVT_OK)
        Log.errorPrint("ODTObserver_update_1 [", perr, "]");
}

/** \name Method registers an observed object.
* \param 'sender' - object of observation
* \param 'asp' - an aspect (notification) for a observer
* \return An identifier of a pair <sender, aspect>
* \note Method must be executed before start of a multitasking.
*/
int ODTObserver::registerSubj (ODTSubject* sender, const ASPECT asp)
{
    if (nRegistered > OSBase::EVT_IN_GROUP-1)
    {
        // 'abort' terminates the application
        Log.abort ("Critical Error: ODTObserver_registerSubj_1 [", nRegistered, "].");
        return -1;  // dla formalnoœci - nigdy tu nie dojdzie
    }
    
    if (sender == (ODTSubject*)0)
    {
        Log.abort ("Critical Error: ODTObserver_registerSubj_2.");
        return -1;
    }
    
    sender->attach(this, asp, nRegistered);
    return nRegistered++;
}

/** \name Method waits for occurrence of an aspect.
* \return A set of bits corresponding to the returned pairs <sender,aspect> to be checked out by 'checkAspect' method
*/
OSBase::EvtMask ODTObserver::waitForAnyAspect (void)
{
    int perr = 0;
    OSBase::EvtMask flags = 0;
    OSBase::EVTCODE err = Os->evtWaitAny (_flagGrp, flags, &perr);
    if (err != OSBase::EVT_OK)
        Log.errorPrint("ODTObserver_waitForAnyAspect_1 [", perr, "]");

    return flags;
}

/** \name Method resets all waiting aspects
*/
void ODTObserver::resetAllAspects (void) const
{
    int perr = 0;
    OSBase::EVTCODE err = Os->evtPost (_flagGrp, ~(OSBase::EvtMask)0, false, &perr);
    if (err != OSBase::EVT_OK)
        Log.errorPrint("ODTObserver_update_1 [", perr, "]");
}
