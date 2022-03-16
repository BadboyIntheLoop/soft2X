/**
*                                                                   
* @class StatQueue                                                         
*                                                                   
* @brief Universal template for one direction queue class.
* The queue stores copy of written elements (not a pointers).
* Memory is allocated only once for whole queue, during creatong object of the class.             
*                                                                   
* Stored elements must have copy constructor (could be default)                                                   
*                                                                   
* Class is not multi threaded so its demanding external semaphores to synchronisation.                                                   
*                                                                   
* 2012 Witold Kruczek @ Flytronic                                   
*/

#ifndef STATQUEUE_H
#define STATQUEUE_H


template <class T, int N>
class StatQueue
{
public:
    //  copy constructor standard
    StatQueue (void);       ///< Constructor
    bool put (const T &el); ///< Add element to the buffer
    bool get (T &el);       ///< Get element from the buffer

private:
    T buf[N];       ///< Buffer stroing elements
    int putPos;     ///< Index at which new element will be written to the buffer
    int getPos;     ///< Index from which element will be read.
    int cnt;        ///< Number of elements in queue
};

/**
* Constructor 
* /param T - name of the class 
* /param N - maximum number of elements in queue.
*/
template <class T, int N>
StatQueue <T, N>::StatQueue (void):
    putPos(0), getPos(0), cnt(0)
{};


/**
* Add element "el" to the queue.
* Returns false when there was an error (queue is full)
*/
template <class T, int N>
bool StatQueue <T, N>::put (const T &el)
{
    // Check if queue is full
    if (cnt >= N)
        return false;

    buf[putPos++] = el;       // Copy element to the queue
    putPos %= N;            // Index of next writting
    cnt++;

    return true;
};


/**
* Get element "el" from the queue.
* Returns false when the queue is empty (Passed element "el" will not been changed)
*/
template <class T, int N>
bool StatQueue <T, N>::get (T &el)
{
    //  Check if queue is empty
    if (cnt <= 0)
        return false;

    el = buf[getPos++];       // Copy element from the queue
    getPos %= N;            // Index of next reading
    cnt--;

    return true;
};

#endif //STATQUEUE_H
