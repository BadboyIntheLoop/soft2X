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

#ifndef BUFFERBASE_H
#define BUFFERBASE_H


class BufferBase
{
public:
    BufferBase& operator= (const BufferBase& src);              ///< Assigment operator

    const unsigned char* getData (void) const;                  ///< Pointer to the beginning of the data in buffer.
    int getBufferUsed (void) const;                             ///< Number of valid bytes in buffer.
    int getBufferSize (void) const;                             ///< Size of buffer in bytes.
    bool concatS (const char* cStringToConcat);                 ///< Add string C to existing data.
    bool concatB (unsigned char c);                             ///< Add single byte to existing data.
    bool concatBytes (const unsigned char* data, int nBytes);   ///< Add string bytes to existing data.
    bool concat (const BufferBase& bb);                         ///< Add BufferBase class object.
    void reset (void);                                          ///< Reset buffer.

    void rawSetBufferUsed (int used);                   ///< Manually number of valid bytes in buffer setting.

protected:
    BufferBase (unsigned char* pData, int size); ///< Constructor
    BufferBase (const BufferBase& src, unsigned char* pData, int size); ///< Copying constructor with additional parameters.
    ~BufferBase(void){}; ///< Destructor lock (release object by pointer to the base class). Destructor is protected and class do not has virtual functions so it could be not virtual.

private:
    BufferBase (const BufferBase& src); ///< Lock default copying constructor.
    bool copy (const BufferBase& src); ///< Copy data from other buffer.

    int       m_nUsed;              ///< Number of valid bytes in buffer.
    const int m_nSize;              ///< Size of buffer in bytes.
    unsigned char* const m_pData;   ///< Pointer to buffer.
};


#endif  // BUFFERBASE_H
