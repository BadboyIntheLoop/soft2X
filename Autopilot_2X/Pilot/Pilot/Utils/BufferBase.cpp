/**
*                                                                      
* @class BufferBase                                                            
*                                                                       
* @brief Base class for classes having buffer functionality
* Used to be inherit by the template class FixBuffer<N>.    
*                                                                       
* Memory allocation is made in derivative class (provides address and buffer size to the BufferBase constructor)                  
*                                                                       
* 2013 Witold Kruczek @ Flytronic                                       
*/

#include <PilotIncludes.h>


/**
* Constructor
* /param pData - pointer to the allocated buffer
* /param size  - size of buffer in bytes
*/
BufferBase::BufferBase (unsigned char* pData, int size):
    m_nUsed(0), m_nSize(size), m_pData(pData)
{}

/**
* Copy constructor with an extra parameters.
* /param src - reference to source object
* /param pData - pointer to the allocated buffer
* /param size  - size of buffer in bytes
*/
BufferBase::BufferBase (const BufferBase& src, unsigned char* pData, int size):
    m_nUsed(0), m_nSize(size), m_pData(pData)
{
    copy (src);        
}

/** 
* Assigment operator
*/
BufferBase& BufferBase::operator= (const BufferBase& src)
{
    copy (src);
    return *this;
}

/**
* Return pointer to data.
* Data are ended with byte 0
*/
const unsigned char* BufferBase::getData (void) const
{
    return m_pData;
}


/**
* Returns number of used bytes in buffer.
* Not including byte 0 at the end of data.
*/
int BufferBase::getBufferUsed (void) const
{
    return m_nUsed;
}

/**
* Returns buffer size in bytes.
*/
int BufferBase::getBufferSize (void) const
{
    return m_nSize;
}

/**
* Add defined string C to the end of data
* /param cStringToConcat - string C to be added
* Returns false when there is an error or data where  cutoff.
* Buffer could include binary data or other string. It is guaranteed that at the end of buffer will be 0.
*/
bool BufferBase::concatS (const char* cStringToConcat)
{
    if (cStringToConcat == NULL)
        return false;

    int i = 0;
    while ((m_nUsed < (m_nSize-1)) && (cStringToConcat[i] != '\0'))
    {
        m_pData[m_nUsed] = static_cast<unsigned char>(cStringToConcat[i]);
        m_nUsed++;
        i++;
    }


    // Add 0 outside the critical data
    m_pData[m_nUsed] = 0;

    // cutoff detected
    if (cStringToConcat[i] != '\0')
        return false;

    return true;
}

/**
* Add single byte to the existing data.
* /param c - byte to add
* Returns false when there is an error or data where cutoff. 
* Buffer could include binary data or other string. It is guaranteed that at the end of buffer will be 0.
*/
bool BufferBase::concatB (unsigned char c)
{
    if (m_nUsed < (m_nSize-1))
    {
        m_pData[m_nUsed++] = c;
        m_pData[m_nUsed] = 0;
        return true;
    }

    return false;
}

/**
* Add string of bytes to existing data.
* /param  data - pointer to the buffer with data
* /param  nBytes - number of bytes to add
* Returns false when there is an error or data where cutoff. 
* Buffer could include binary data or other string. It is guaranteed that at the end of buffer will be 0.
* NOTE: Specified buffer could not overlap on internat object buffer.
*/
bool BufferBase::concatBytes (const unsigned char* data, int nBytes)
{
    if (m_nUsed + nBytes < m_nSize)
    {
        memcpy (m_pData, data, nBytes);
        m_nUsed += nBytes;
        m_pData[m_nUsed] = 0;
        return true;
    }

    return false;
}

/**
* Add class's BufferBAse object data to existing data.
* /param bb - object to add
* Returns false when there is an error or data where cutoff. 
*/
bool BufferBase::concat (const BufferBase& bb)
{
    if (m_nUsed + bb.m_nUsed < m_nSize)
    {
        memcpy (&m_pData[m_nUsed], bb.m_pData, bb.m_nUsed);
        m_nUsed += bb.m_nUsed;
        m_pData[m_nUsed] = 0;
        return true;
    }

    return false;
}

/**
* Reset buffer
*/
void BufferBase::reset (void)
{
    m_nUsed = 0;
    m_pData[m_nUsed] = 0;
}

/**
* Set manually number of valid bytes in buffer.
* USE ONLY in manually manipulation of the buffer.
* First byte beyond range must be set to 0.
*/
void BufferBase::rawSetBufferUsed (int used)
{
    m_nUsed = used;
}

/**
* Private function copying data from other object to current object.
* Set correct number of bytes in buffer.
* /param  src - reference to source object
* Return false when:
*	- source and destination object is the same.
*	- data were cutoff
* NOTE: when calling from constructor object variables must be initialized.
*/
bool BufferBase::copy (const BufferBase& src)
{
    // IF it is the same object return.
    if (this == &src)
        return false;

    // corectness flag
    bool bOk = true;

    // Number of bytes to copy - last byte must be 0
    m_nUsed = src.getBufferUsed ();
    if (m_nUsed >= m_nSize)
    {
        m_nUsed = m_nSize-1;
        // data cutoff
        bOk = false;
    }

    // Data copy
    memcpy (m_pData, src.getData (), m_nUsed);

    // Add 0 beyond valid data
    m_pData[m_nUsed] = 0;

    return bOk;
}
