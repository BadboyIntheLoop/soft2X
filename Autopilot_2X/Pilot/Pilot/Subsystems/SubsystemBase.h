/** \file
* \brief Declaration of a base class for specialized subsystem classes
*/

/** Base class for all specialized subsystem classses. Class implements common, base functionality by defining virtual member methods for ready for
* takeoff checking (isReadyForTakeoff(ClassifiedLine*, bool)), and putting command line to subsystems queue (putLine(ClassifiedLine&)). Both virtual
* methods returns "true" while they are not overriten in derived classes. All subsystems should derives straight from this class (not from a superclass).
*/
/// Base class for specialized subsystem classes
#ifndef SUBSYSTEMBASE_H
#define SUBSYSTEMBASE_H

class ClassifiedLine;

class SubsystemBase
{
public:
	/** Method checks system readiness for take-off, and returns 'true' by default.
	* \param 'cl' - communication device to which to possible error messages will be send (if 'cl' is not NULL).
	* \param 'toLog' - if 'true' then error messages will be logged.
	*/
    virtual bool isReadyForTakeoff (ClassifiedLine *cl, bool toLog) {return true;};

	/// Send commands line to subsystems queue method
    virtual bool putLine (ClassifiedLine &cl) {return true;};
  
protected:
	/// Class constructor is disabled
    SubsystemBase(void) {};
	/// Class destructor is disabled - Parasoft-suppress OOP-31 "NIOS-2 environment reports a warning while destructor is not virtual"
    virtual ~SubsystemBase(void){};
	
private:
	/// Copy contructor is disabled
    SubsystemBase(SubsystemBase&);
	/// Assignment operator is disabled
	SubsystemBase& operator=(const SubsystemBase&);
};

#endif //SUBSYSTEMBASE_H
