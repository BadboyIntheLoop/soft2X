#include <PilotIncludes.h>


ODTSubject::ODTSubject(void):
    nAttached(0)
{
    for (int i=0; i<MAXATTACH; i++)
    {
        observers[i] = 0;
        aspects[i] = UNKNOWN_ASPECT;
        observerData[i] = 0;
    }
}

/** \name Method attaches observer specified by an 'obs' argument to the list of notificated observers
* \param 'obs' - a pointer to an observer object
* \param 'asp' - aspect for the observer
* \param 'obsData' - any data from the observer passed back during notification 
* \note method 'attach' must be executed before starting multitasking
*/
void ODTSubject::attach (ODTObserver* obs, ASPECT asp, int obsData)
{
    if (nAttached < MAXATTACH)
    {
        observers[nAttached] = obs;
        aspects[nAttached] = asp;
        observerData[nAttached++] = obsData;
    }
    else
		/// Too small list size - terminating the program
        Log.abort ("Critical Error: ODTSubject_attach_1.");
}

/** \name Method notifies registered observers of occurence of an aspect specified by 'asp' argument
*/
void ODTSubject::notify (ASPECT asp) const
{
    for (int i = 0; i < nAttached; i++)
        if (aspects[i] == asp)
            observers[i]->update (observerData[i]);
}
