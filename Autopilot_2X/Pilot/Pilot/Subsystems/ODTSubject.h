#ifndef ODTSUBJECT_H
#define ODTSUBJECT_H

class ODTObserver;

/** \file Declaration of a base class acts as a subiect of an Observer Design Temlate
* \brief 
*/

/** Base class serves as an object of observation (subiect) an Observer Design Template.
*/
/// Class implements a subiect of an Observer Design Template
class ODTSubject
{
public:
	/// Maximum amount of stored pairs<observer,aspect>
    static const int MAXATTACH = 30;

	/// Observer attaching method
	void attach (ODTObserver* obs, ASPECT asp, int obsData);

	/// Method notifies observers of the occurence of aspect
    void notify (ASPECT asp) const;

protected:
	/// Default constructor is disabled
    ODTSubject(void);
	/// Destructor is disabled (releaseing the object by pointer to its base class)
    virtual ~ODTSubject(void){};

private:
	/// Copy constructor is disabled
    ODTSubject(ODTSubject&);
	/// Copy operator is disabled
    ODTSubject& operator=(const ODTSubject&);

    ODTObserver* observers[MAXATTACH];
    ASPECT aspects[MAXATTACH];
    int observerData[MAXATTACH];
    int nAttached;
};

#endif  //  ODTSUBJECT_H
