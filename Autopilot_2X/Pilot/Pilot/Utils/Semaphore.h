/**
*                                                                   
* @class Semaphore                                                         
* @brief Class to generate and controll single semaphor.
*                                                                 
* Marcin Pczycki (c) Flytronic 2008                                 
* Witold Kruczek @ Flytronic 2012                                   
*/

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

class Semaphore
{
private:
    static const int _NAME_SIZE = 25;
    static const int _TIMEOUT_MS = 200;     // timeout in ms

    OSBase::SemHandle   _sem;

    char                _name[_NAME_SIZE];

    void msg (const char* text, const char* name, int errCode) const;

public:
    Semaphore(void);
    ~Semaphore(void);

    bool create(const char* name);
    bool lock(bool suppressErrMsg = false, int timeout = _TIMEOUT_MS);
	bool lock(const char* pLogMsg, const char* pPrefix, bool suppressErrMsg = false, int timeout = _TIMEOUT_MS);
    bool unlock(bool suppressErrMsg = false);
};

#endif //SEMAPHORE_H

