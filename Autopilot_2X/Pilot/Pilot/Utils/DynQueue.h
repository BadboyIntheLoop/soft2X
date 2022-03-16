/**
*                                                                   
* @ class DynQueue                                                          
*                                                                   
* @brief Universal template class that supports the functionality a queue one way.
* Queue stores copy of the written elements (not a pointer). Memory is allocated at the start only once during object of class creation.                                                                          
* Number of elements is specified as a parameter to the constructor.
*                                                                   
* Stored elements must have copy constructor (could be default)                                                       
* Class is multi thread. Class is not multi threaded - requires external semaphorse to the synchronization                                                 
*                                                                   
* 2013 Witold Kruczek @ Flytronic                                   
*/

#ifndef DYNQUEUE_H
#define DYNQUEUE_H


template <class T, class TI = T>
class DynQueue
{
public:
    explicit DynQueue (int maxElems);        ///< Constructor
    bool put (const TI* el);                 ///< Write element at the end of queue.
    bool putWork (void);                     ///< Write working element at the end of queue.
    bool get (TI& el);                       ///< Get element from beginning of queue.
    bool get (void);                         ///< Skip element 
    const TI* first (void);                  ///< Pointer to the first element of queue.
    TI* work (void);                         ///< Pointer to working element (used by putWork to preparing data)

protected:
    ~DynQueue(void){}; ///< Destructor lock (release object by pointer to the base class). Destructor is protected and class do not has virtual functions so it could be not virtual.

private:
    DynQueue (const DynQueue&); ///< Lock copy constructor
    DynQueue& operator=(const DynQueue&); ///< Lock assigment operator

    T* const m_buf;       ///< Pointer to array of elements
    int putPos;           ///< Position in elements buffer (index) at which new element will be saved.
    int getPos;           ///< Position in elements buffer (index) from which element will be read.
    int cnt;              ///< Current number of elements in queue.
    const int m_maxElems; ///< Maximum number of elements in queue.
};


/**
* Constructor
* /param T - name of class which elements will be store.
* /param TI - base class T (interface) used when T class is an template class
*/
template <class T, class TI>
DynQueue<T, TI>::DynQueue (int maxElems):
    m_buf(new T[maxElems]), putPos(0), getPos(0), cnt(0), m_maxElems(maxElems)
{};


/**
* Write "el" element to the queue.
* Return false when error occurs or queue is full.
* NOTE: "el" element is a pointer not a reference in order to avoid creation a temporary object in parent function
*/
template <class T, class TI>
bool DynQueue <T, TI>::put (const TI* el)
{
    // Check if queue is full
    if (cnt >= m_maxElems)
        return false;

    m_buf[putPos++] = *el;       // Copy element to queue
    putPos %= m_maxElems;        // Positions of next write
    cnt++;

    return true;
};


/**
* Writes working element (work) to queue.
* Return false when error occurs or queue is full.
*/
template <class T, class TI>
bool DynQueue <T, TI>::putWork (void)
{
    // Check if queue is full
    if (cnt >= m_maxElems)
        return false;

    putPos++;                    //  Element "work" is in queue so pointer need to be moved.
    putPos %= m_maxElems;        // Positions of next write
    cnt++;

    return true;
};


/** 
* Gets elemebt "el" from queue
* Return false when queue is empty.
* In case of an error specified element "el" will not been changed.
*/
template <class T, class TI>
bool DynQueue <T, TI>::get (TI& el)
{
    // Check if queue is full
    if (cnt <= 0)
        return false;

    el = m_buf[getPos++];       // Copy element from queue.
    getPos %= m_maxElems;       // Positions of next read.
    cnt--;

    return true;
};


/**
* Skip current (first) element in the queue.
* Return false when queue is empty.
*/
template <class T, class TI>
bool DynQueue <T, TI>::get (void)
{
    // Check if queue is empty.
    if (cnt <= 0)
        return false;

    getPos++;
    getPos %= m_maxElems;       // Positions of next read.
    cnt--;

    return true;
}


/*)*
* Returns pointer to the first element of the queue (current for get)
* Do not delete element and do not move position. Nie usuwa elementu i nie przesuwa pozycji.
* Return NULL when queue is empty.
*/
template <class T, class TI>
const TI* DynQueue <T, TI>::first (void)
{
    //  Check if queue is empty.
    if (cnt <= 0)
        return NULL;

    return &m_buf[getPos];
}


/**
* Returns pointer to the woriking element used to prepare data by user.
* Currently element is not in queue, it has been put after calling putWork.
* Do not delete element and do not move position. Nie usuwa elementu i nie przesuwa pozycji.
* Return NULL when queue is full.
*/
template <class T, class TI>
TI* DynQueue <T, TI>::work (void)
{
    //  Check if queue is full.
    if (cnt >= m_maxElems)
        return NULL;

    return &m_buf[putPos];
}

#endif //DYNQUEUE_H
