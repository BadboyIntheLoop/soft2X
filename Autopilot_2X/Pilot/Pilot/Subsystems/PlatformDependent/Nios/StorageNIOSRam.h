#ifndef STORAGENIOSRAM_H
#define STORAGENIOSRAM_H

// Includes declaration of a PILOT_TARGET platform

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_HARDWARE

/** \file
* \brief Declaration of a class supporting non volatile RAM memory in the NIOSII environment
*/

/** Class supports non volatile RAM memory in the NIOSII environment. There are predefined areas, in the RAM memory, which simulates the files.
* Each file has two areas written alternately in order to improve reliability. Data are written only at the begining of a file and they are
* complemented by signature the beginning and end, checked when reading. Functions 'append', 'clear', 'getSize' and 'task' are not supported.
* There can be many objects of this class but each must handle only one file object. Access to the RAM is synchronized by a static semaphore because of the
* mechanism of write locking.
* Write locking mechanism (in the destination device) involves of writing to the appropriate registers a reserved word of opening a write and
* the number of bytes to be written. After writing of a specified number of bytes, write is locked again.
*/
/// Implementation of a class supporting non volatile RAM memory in the NIOSII environment
class StorageNIOSRam: public StorageBase
{
public:    
    StorageNIOSRam(void);
    
    bool open (const char* fname);					///< Openign file method
    bool close (void);							    ///< Closing file method
    bool save (const void* buf, int size);			///< Writing of specified buffer to file method
    bool load (void* buf, int size);				//< Reading of a specified buffer from file method with validation
    bool append (const char* lineToWrite, bool suppressErrMsg=false); ///< Not implemented
    bool read (void* buf, int size, int offset);	///< Not implemented
    bool clear (bool fastClear=true);				///< Not implemented
    int getSize (void);								///< Not implemented
    bool scan (const char* text, int &offset, int maxCount=0, const char* stopText=NULL);	///< Not implemented
    bool readLine (char* buf, int bufSize, int &offset);				///< Not implemented
    bool writeFlash(const void* buf, unsigned int offset, int size);	///< Not implemented
    bool calcFlashCRC(unsigned int offset, int size, INT16U& crc);		///< Not implemented
	
	void task(void* pdata);							///< Not implemented

private:
    static const int FNAME_LENGTH = 20;         ///< File name buffer size
    static const int MAX_FILES = 10;            ///< Files array size
    static const int CHUNK_SIZE = 0x400;        ///< Size of the primary file element

	/** \name Signatures of the beginning and end of the data (initialized in '.cpp' because they must have a physical addresses)
	* \{
	*/
    static const unsigned int BEG_SIGNATURE;
    static const unsigned int END_SIGNATURE;
    static const unsigned int ZERO_SIGNATURE;
	///\}

	/** \name Maximum time of waiting for an access to the flash memory by the save and load functions (in ticks).
	* It should be shorter then the calculation for a control cycle (1/12 sec).
	* \note Block erasing operation (e.g. before writing) takes about 1,5 seconds!
	*/
    static const int RAM_SEM_WAIT_TICKS = 5;

#if PILOT_TARGET == PT_HARDWARE
    static const unsigned int FILES_SIZE = 65536;	 // Size reserved for files
    static char temp[FILES_SIZE];
    static const unsigned int NVRAM_BASE;
    static const unsigned int FILES_OFFSET = 0;	///< Offset of an file array from the beginning of the RAM memory (should be a multiple of 4)

#else
    #error "FILES_OFFSET, etc. undefined (StorageNIOSRam.h)"
#endif

    struct FileItem                 ///< Structure of a simulated file in the flash memory
    {
        const char fileName[FNAME_LENGTH];          ///< File name
        const unsigned int fileSize;                ///< File size in bytes
        const unsigned int fileOffset1;             ///< Offset of a first copy from the beginning of a file array
        const unsigned int fileOffset2;             ///< Offset of a second copy from the beginning of a file array
    };
    
    int fIdent;                               ///< File identifier
    bool fileOpened;                          ///< Flag indicates if file is open
    unsigned int saveCounter;                 ///< Save counter (it works like a timestamps)

    static OS_EVENT* rSem;                    ///< Semaphore controlling access to the non volatile RAM memory
    static FileItem files[MAX_FILES];         ///< Array of descriptors of predefined file

    // Destructor is disabled - object should never be destroyed
    ~StorageNIOSRam(void){};

    bool lockRam (void);
    bool unlockRam (void);
    void writeEnable (int count);
    bool writeChunk (char* dest, const void* source, int size);    
};

#endif  //  PILOT_TARGET
#endif  //   STORAGENIOSRAM_H
