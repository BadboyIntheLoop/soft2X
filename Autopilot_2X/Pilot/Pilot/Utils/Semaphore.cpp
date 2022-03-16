/**
*                                                                   
* @class Semaphore                                                         
* @brief Class to generate and controll single semaphor.
*                                                                 
* Marcin Pczycki (c) Flytronic 2008                                 
* Witold Kruczek @ Flytronic 2012                                   
*/

#include <PilotIncludes.h>

/**
* Constructor, Single class instance controls single semaphor.
*/
Semaphore::Semaphore(void):
    _sem(0)
{
    _name[0] = 0;
}

Semaphore::~Semaphore(void)
{
}

/**
* Create semaphore
*/
bool Semaphore::create(const char* name)
{
    _sem = Os->semCreate();
    if (_sem == 0)
        return false;

    int len = strlen(name);
	if(len > _NAME_SIZE - 1)
		return false;

    MEMCCPY(&_name, name, 0, len + 1);
	return true;
}

/**
* Lock access
*/
bool Semaphore::lock(bool suppressErrMsg, int timeout)
{
    int osErr = 0;  // Error code depending on operating system (for diagnostic purposes)

    // Access controll to the varaibles by the semaphore
    OSBase::SEMCODE err = Os->semLock (_sem, timeout, &osErr);
    if (err != OSBase::SEM_OK)
    {
        if (!suppressErrMsg)
        {
            // Posible of recurention use only msg function
            msg ("Semaphore_lock: ", _name, osErr);
        }
        return false;
    }

    return true;
}

/**
* Lock access with log message
*/
bool Semaphore::lock(const char* pLogMsg, const char* pPrefix, bool suppressErrMsg, int timeout)
{
    int osErr = 0;  // Error code depending on operating system (for diagnostic purposes)

    // Access controll to the varaibles by the semaphore
    OSBase::SEMCODE err = Os->semLock (_sem, timeout, &osErr);
    if (err != OSBase::SEM_OK)
    {
        if (!suppressErrMsg)
        {
            // Posible of recurention use only msg function
            msg ("Semaphore_lock: ", _name, osErr);
        }

		Log.msgPrintf (pLogMsg, pPrefix);

        return false;
    }

    return true;
}

/**
* Release lock
*/
bool Semaphore::unlock(bool suppressErrMsg)
{
    int osErr = 0;  // Error code depending on operating system (for diagnostic purposes)

    //  Unlock semaphore
    OSBase::SEMCODE err = Os->semUnlock (_sem, &osErr);
    if (err == OSBase::SEM_OK)
        // semaphore released without error
        return true;
    else if (err == OSBase::SEM_FIXED)
    {
        // Semaphore value need to be correct
        if (!suppressErrMsg)
        {
            // Posible of recurention use only msg function
            msg ("Semaphore_unlock_1: ", _name, osErr);
        }
        return true;
    }
    else
    {
        //  Error semaphore could not been unlocked.
        if (!suppressErrMsg)
        {
            // Posible of recurention use only msg function
            msg ("Semaphore_unlock_2: ", _name, osErr);
        }
    }

    return false;
}

/**
* Auxiliary function formating and sending message with an error.
*/
void Semaphore::msg (const char* text, const char* name, int errCode) const
{
    char buf[LINESIZE*2];

    MEMCCPY (buf, text, 0, 100);
    strncat (buf, name, 100);
    strncat (buf, ", err: ", 100);
 
	if (TypeParser::toStr (errCode, buf + strlen(buf), LINESIZE))
    {
		// Posible of recurention use only one argument version of function errorPrint
        Log.errorPrint (buf);
    }
}
