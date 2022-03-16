#ifndef ODTOBSERVER_H
#define ODTOBSERVER_H

class ODTSubject;

/** \file
* \brief Class implements an Observer Design Template Observer
*/

/** Base class serves as an observing object (observer) of a Observer Design Template.
*/
///  Class implements an observer of an Observer Design Template
class ODTObserver
{
public:
    static const OSBase::EvtMask FLAGMASK[];

	/// Notification of the occurence of an aspect (notified by observerd object)
    void update (int obsData) const;

protected:
	/// Default constructor is disabled
    ODTObserver(void);
    /// Destructor is disabled (releaseing the object by pointer to its base class)
    virtual ~ODTObserver(void){};

	/// Waiting for a notification of an aspect
    OSBase::EvtMask waitForAnyAspect (void);

	/** (Inline method) Ckecking the result of 'waitForAnyAspect' method if registered pair <sender,aspect> has occurred.
	*/
    bool checkAspect (OSBase::EvtMask evtFlags, int index) const
    {
        if (index >=0 && index <=31)
            return ((evtFlags & FLAGMASK[index]) != 0);
        return false;
    }

	/// Registration of an object to observe
	/// NOTE: method must be executed before start of a multitasking
    int registerSubj (ODTSubject* sender, ASPECT asp);

	/// Resetting the waiting aspects
    void resetAllAspects (void) const;

private:
	/// Copy constructor is disabled
    ODTObserver(ODTObserver&);
	/// Copy operator is disabled
    ODTObserver& operator=(const ODTObserver&);

    OSBase::EvtHandle _flagGrp;
    int nRegistered;
};

#endif  //ODTOBSERVER_H
