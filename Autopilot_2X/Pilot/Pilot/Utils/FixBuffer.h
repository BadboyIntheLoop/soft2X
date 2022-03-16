/**
*                                                                           
* @class FixBuffer                                                                 
*                                                                           
* @brief Base class for classes having buffer functionality.
* Class could have different alloacated memmory.
*                                                                           
* Memory with constant size is allocated in private object's variable.
* It is relese automaticaly.
*                                                                           
* Data could be copied between specializations this class that has different sizes (by using base class functions)                           
*                                                                           
* 2013 Witold Kruczek @ Flytronic                                           
*/

#ifndef FIXBUFFER_H
#define FIXBUFFER_H


template <int N>
class FixBuffer: public BufferBase
{
public:
    FixBuffer (void);                               ///<  Konstruktor domyœlny (pusty bufor)
    explicit FixBuffer (const char* cString);       ///< Constructor based on string c
    explicit FixBuffer (const BufferBase& src);     ///< Copy constructor
    explicit FixBuffer (const FixBuffer<N>& src);   ///< Copy constructor

    FixBuffer& operator= (const BufferBase& src);   ///< Assigment operator

private:
    unsigned char data[N];                          ///< Buffer storing data
};


/**
* Default constructor (empty buffer)
* /param N - buffer size
* First buffer byte is 0.
*/
template <int N>
FixBuffer<N>::FixBuffer (void): BufferBase(data, N)
{
    data[0] = 0;
}


/**
* Constructor based on string c
* /param N - buffer size
* /param cString - string  that initialize buffer ended with 0
*/
template <int N>
FixBuffer<N>::FixBuffer (const char* cString): BufferBase(data, N)
{
    data[0] = 0;

    // Add specified string to empty buffer
    concatS (cString);
}


/**
* Copy constructor
* /param N - buffer size
*/
template <int N>
FixBuffer<N>::FixBuffer (const BufferBase& src): BufferBase(src, data, N)
{}


/**
* Copy constructor
* /param N - buffer size
*/
template <int N>
FixBuffer<N>::FixBuffer (const FixBuffer<N>& src): BufferBase(src, data, N)
{}


/** 
* Assigment operator 
* Any object inherited from NufferBase could be assign (especiall FixBuffer with different size of internal buffer (different N param of template)
* Assigment to the buffer with smaller size will cut off the data.
*/
template <int N>
FixBuffer<N>& FixBuffer<N>::operator= (const BufferBase& src) 
{
    this->BufferBase::operator= (src);
    return *this;
}


#endif  // FIXBUFFER_H
