/**                                                               
* @class CmdQueue                                                         
*                                                                  
* Class template controling/handling subsystems input commands queue.                                                    
* Method are called in different thread context.
* 2012 Witold Kruczek @ Flytronic                                  
*/

#ifndef CMDQUEUE_H
#define CMDQUEUE_H


template <int N2>
class CmdQueue
{
public:
    //  copy constructor (automatic generated)
    CmdQueue (SystemLog &pLog);                 //  constructor
    bool cmdPut (const ClassifiedLine &cl);     //  Write commnad to the queue
    bool cmdGet (ClassifiedLine &cl);           //  Read command from the queue

private:
    StatQueue <ClassifiedLine, N2> _sq;     //  Next line
    Semaphore                      _sem;    //  Semaphor controlling access to the queue
};



/*
* Constructor
*/
template <int N2> CmdQueue <N2>::CmdQueue (SystemLog &pLog)
{
    //  Create semaphore 
    if (!_sem.create("CmdQueue"))
    {
        pLog.abort ("Critical Error: CmdQueue_1.");
        return;
    }
};

/*
* Write new commnad to the queue. Access is protected by the semaphore.
*/
template <int N2> bool CmdQueue <N2>::cmdPut (const ClassifiedLine &cl)
{
    if (!_sem.lock ())
        return false;

    bool ret = _sq.put (cl);

    //  Error in unlocking the semaphore. It is written by the Semaphore class to the log file.
    //  Here is ignored.
    _sem.unlock ();

    return ret;   
}



/**
* Get new command from queue. Access is protected by the semaphore.
*/
template <int N2> bool CmdQueue <N2>::cmdGet (ClassifiedLine &cl)
{
    if (!_sem.lock ())
        return false;

    bool ret = _sq.get (cl);

    //  Error in unlocking the semaphore. It is written by the Semaphore class to the log file.
    //  Here is ignored.
    _sem.unlock ();

    return ret;   
}

#endif //CMDQUEUE_H
