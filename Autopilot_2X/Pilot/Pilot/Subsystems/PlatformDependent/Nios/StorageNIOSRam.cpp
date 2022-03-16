#include "PilotIncludes.h"

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_HARDWARE

// Initialization of a class static members
OS_EVENT* StorageNIOSRam::rSem = static_cast<OS_EVENT*> (0);
const unsigned int StorageNIOSRam::BEG_SIGNATURE = 0xf1c0409a;
const unsigned int StorageNIOSRam::END_SIGNATURE = 0x20ae0cf5;
const unsigned int StorageNIOSRam::ZERO_SIGNATURE = 0;

#if PILOT_TARGET == PT_HARDWARE
// Simulation of a non volatile RAM memory in a conventional array
char StorageNIOSRam::temp[FILES_SIZE];
const unsigned int StorageNIOSRam::NVRAM_BASE = reinterpret_cast<unsigned int>(StorageNIOSRam::temp);

#else   //PILOT_TARGET
    #error "PILOT_TARGET undefined (StorageNIOSRam.cpp)"

#endif  //PILOT_TARGET

/** \name Predefined files.
* If declared array is greater than number of stored elements unused positions shoud be initialized with 0 (C++ guarantees that).
* Unused fields in structure are also initialized with 0 by default. Files must be of size equal to multiple of 4 bytes.
*/
StorageNIOSRam::FileItem StorageNIOSRam::files[] = {
    {"fprealram",   CHUNK_SIZE,  0*CHUNK_SIZE,  1*CHUNK_SIZE},
    {"pstateram",   CHUNK_SIZE,  2*CHUNK_SIZE,  3*CHUNK_SIZE},
    {"fcontrolram", CHUNK_SIZE,  4*CHUNK_SIZE,  5*CHUNK_SIZE},
    {"servmanram",  CHUNK_SIZE,  6*CHUNK_SIZE,  7*CHUNK_SIZE},
    {"test",        CHUNK_SIZE,  8*CHUNK_SIZE,  9*CHUNK_SIZE},
    {"fpram2",      CHUNK_SIZE, 10*CHUNK_SIZE, 11*CHUNK_SIZE},
    {"fpram",       20*CHUNK_SIZE, 12*CHUNK_SIZE, 32*CHUNK_SIZE}
};

StorageNIOSRam::StorageNIOSRam()
{
    fileOpened = false;
    saveCounter = 0;

	// Creation of a semaphore common to all objects of this class (if it's has not been created already)
    if (rSem == static_cast<OS_EVENT*> (0))
    {
        rSem = OSSemCreate (1);      // Access is unlocked
        if (rSem == static_cast<OS_EVENT*> (0)) 
        {
            Log.abort ("Critical Error: StorageNIOSRam_1.");
            return;
        }

		// File array validation
        unsigned int a = 0;
        for (int i=0; (i<MAX_FILES) && (files[i].fileName[0] != 0); i++)
        {
            if (a > files[i].fileOffset1)
            {
                // Overlapped ranges
                Log.abort ("Critical Error: StorageNIOSRam_2.");
                return;
            }
            a = files[i].fileOffset1 + files[i].fileSize;
    
            if (a > files[i].fileOffset2)
            {
                // Overlapped ranges
                Log.abort ("Critical Error: StorageNIOSRam_3.");
                return;
            }
    
            a = files[i].fileOffset2 + files[i].fileSize;        
        }
        
        if (a > FILES_SIZE)
        {
			// Sum of files size is larger than the available space
            Log.abort ("Critical Error: StorageNIOSRam_4.");
            return;
        }
    }
}

/** \name Method opens a file
* In single object only one file may be opened. If some file was opened before, it is going to be closed.
* A file cannot be opened by a multiple objects simultaneously. A file name must be one of the predefined file names.
* \param 'fname' - file name
* \return 'true' on success, 'false' otherwise
*/
bool StorageNIOSRam::open (const char* fName)
{
    // Closing of an opened file
    if (fileOpened)
        close ();
    
    // Searching for a file name in a file name array
    for (int i=0; i<MAX_FILES; i++)
        if (STRNICMP (fName, files[i].fileName, FNAME_LENGTH) == 0)
        {
            fileOpened = true;
            fIdent = i;

            return true;
        }
            
    Log.errorPrintf("StorageNIOSRam_open_1 [%s]", fName);
    return false;
}

/** \name Methods closes file
* Dummy method which changes only an file status of open
*/
bool StorageNIOSRam::close (void)
{
    fileOpened = false;
    return true;
}

/** \name Method writes any data to a file
* \param 'buf' - pointer to a data to be written
* \param 'size' - size of data in bytes
* \return 'true' on success, 'flase' otherwise
*/
bool StorageNIOSRam::save (const void* buf, int size)
{   
    if (!fileOpened)
    {
        Log.errorPrintf("StorageNIOSRam_save_1");
        return false;
    }
    
	// Check if data size is not greater than file size
    if (static_cast<unsigned int>(size) > files[fIdent].fileSize
        -sizeof(BEG_SIGNATURE)-sizeof(END_SIGNATURE)-sizeof(saveCounter))
    {
        Log.errorPrintf("StorageNIOSRam_save_2");
        return false;
    }
          
	// Calculation of a pointer to the beginning of a data
    char* chunkBegPtr1 = reinterpret_cast<char*>(NVRAM_BASE + FILES_OFFSET + files[fIdent].fileOffset1);
    char* chunkBegPtr2 = reinterpret_cast<char*>(NVRAM_BASE + FILES_OFFSET + files[fIdent].fileOffset2);
       
	// Waiting for an access to RAM - 'lock' function prints optional error
    if (!lockRam ())
        return false;

	// Copy of a file 1
    if (!writeChunk (chunkBegPtr1, buf, size))
    {
        unlockRam ();
        return false;
    }

    // Copy of a file 2
    if (!writeChunk (chunkBegPtr2, buf, size))
    {
        unlockRam ();
        return false;
    }
     
    if (!unlockRam ())
        return false;
               
    return true;
}

/** \name Method reads from file any data with signatures validation
* \param 'buf' - pointer to the buffer to which data will be written
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageNIOSRam::load (void* buf, int size)
{
    if (!fileOpened)
    {
        Log.errorPrintf("StorageNIOSRam_load_1");
        return false;
    }
    
	// Calculation of a pointer to the beginning of data
    char* chunkBegPtr1 = reinterpret_cast<char*>(NVRAM_BASE + FILES_OFFSET + files[fIdent].fileOffset1);
    char* chunkBegPtr2 = reinterpret_cast<char*>(NVRAM_BASE + FILES_OFFSET + files[fIdent].fileOffset2);

	// For reading semaphores aren't needed because locking mechanism isn't used
	// Reading of a counters and signatures from copy 1
    unsigned int beg1, count1, end1;
    memcpy (&beg1, chunkBegPtr1, sizeof(beg1));    
    memcpy (&count1, chunkBegPtr1 + sizeof(BEG_SIGNATURE), sizeof(count1));    
    memcpy (&end1, chunkBegPtr1 + sizeof(BEG_SIGNATURE) + sizeof(saveCounter) + size, sizeof(end1));    

    // Reading of a counters and signatures from copy 2
    unsigned int beg2, count2, end2;
    memcpy (&beg2, chunkBegPtr2, sizeof(beg2));    
    memcpy (&count2, chunkBegPtr2 + sizeof(BEG_SIGNATURE), sizeof(count2));    
    memcpy (&end2, chunkBegPtr2 + sizeof(BEG_SIGNATURE) + sizeof(saveCounter) + size, sizeof(end2));
    
	// Selecting the last, correct line
    char* ptr = NULL;
    if (beg2==BEG_SIGNATURE && end2==END_SIGNATURE)
        ptr = chunkBegPtr2;
    if (beg1==BEG_SIGNATURE && end1==END_SIGNATURE && count1>count2)
        ptr = chunkBegPtr1;
        
    if (ptr == NULL)
    {
        Log.errorPrintf("StorageNIOSRam_load_2");
        return false;
    }
    
    // Reading of a data
    memcpy (buf, ptr + sizeof(BEG_SIGNATURE) + sizeof(saveCounter), size);
    
    return true;
}

bool StorageNIOSRam::read (void* buf, int size, int offset)
{
    Log.errorPrintf("StorageNIOSRam_read_1: Not implemented");
    return false;
}

bool StorageNIOSRam::clear (bool fastClear)
{
    Log.errorPrintf("StorageNIOSRam_clear_1: Not implemented");
    return false;
}

int StorageNIOSRam::getSize ()
{
    Log.errorPrintf("StorageNIOSRam_getSize_1: Not implemented");
    return false;
}

/** \name Method handles uC/OS-II system tasks associated with writing line to Flash
*/
void StorageNIOSRam::task(void* pdata)
{
    Log.errorPrintf("StorageNIOSRam_task_0");
    OSTaskSuspend (OS_PRIO_SELF);        
}

bool StorageNIOSRam::append (const char* lineToWrite, bool suppressErrMsg)
{
    if (!suppressErrMsg)
        Log.errorPrintf("StorageNIOSRam_append_1: Not implemented");
    return false;
}

bool StorageNIOSRam::scan (const char* text, int &offset, int maxCount, const char* stopText)
{
    Log.errorPrintf("StorageNIOSRam_scan_1: Not implemented");
    return false;
}

bool StorageNIOSRam::readLine (char* buf, int bufSize, int &offset)
{
    Log.errorPrintf("StorageNIOSRam_readLine_1: Not implemented");
    return false;
}

bool StorageNIOSRam::writeFlash(const void* buf, unsigned int offset, int size)
{
    Log.errorPrintf("StorageNIOSRam_writeFlash_1: Not implemented");
    return false;
}

bool StorageNIOSRam::calcFlashCRC(unsigned int offset, int size, INT16U& crc)
{
    Log.errorPrintf("StorageNIOSRam_calcFlashCRC_1: Not implemented");
    return false;
}

/** \name Method locks access to a non volatile RAM memory
*/
bool StorageNIOSRam::lockRam ()
{
    INT8U osErr;
 
    OSSemPend (rSem, RAM_SEM_WAIT_TICKS, &osErr);
    if (osErr != OS_NO_ERR)
    {
		// Error opening the semaphore
        Log.errorPrint("StorageNIOSRam_lockRam_1 [", osErr, "]");
        return false;
    }
    return true;
}

/** \name Method unlocks the semaphore controlling acces to the flash memory
*/
bool StorageNIOSRam::unlockRam (void)
{
    INT8U osErr;
 
    osErr = OSSemPost (rSem);
    if (osErr != OS_NO_ERR)
    {
       Log.errorPrint("StorageNIOSRam_unlockRam_1 [", osErr, "]");
       return false;
    }
    return true;
}

void StorageNIOSRam::writeEnable (int count)
{
#if PILOT_TARGET == PT_HARDWARE
#endif  //  PILOT_TARGET
}

bool StorageNIOSRam::writeChunk (char* dest, const void* source, int size)
{
	// Opening the RAM (use of a write lock for specified number of bytes)
    writeEnable (sizeof(BEG_SIGNATURE) + sizeof(END_SIGNATURE));

	// Clearing sygnatures
    memcpy (dest, &ZERO_SIGNATURE, sizeof(ZERO_SIGNATURE));
    memcpy (dest + sizeof(BEG_SIGNATURE) + sizeof(saveCounter) + size, &ZERO_SIGNATURE, sizeof(ZERO_SIGNATURE));
    
	// Checking the signatures are cleared (it could be locked)
    if (memcmp (dest, &ZERO_SIGNATURE, sizeof(ZERO_SIGNATURE)) != 0 || 
        memcmp (dest + sizeof(BEG_SIGNATURE) + sizeof(saveCounter) + size, &ZERO_SIGNATURE, sizeof(ZERO_SIGNATURE)) != 0)
    {
        Log.errorPrintf("StorageNIOSRam_writeChunk_1");
        return false;
    }
    
	// Opening the RAM
    writeEnable (sizeof(BEG_SIGNATURE) + sizeof (saveCounter) + size + sizeof(END_SIGNATURE));

    // Writing
    memcpy (dest, &BEG_SIGNATURE, sizeof(BEG_SIGNATURE));
    memcpy (dest + sizeof(BEG_SIGNATURE), &saveCounter, sizeof(saveCounter));
    memcpy (dest + sizeof(BEG_SIGNATURE)+ sizeof(saveCounter), source, size);
    memcpy (dest + sizeof(BEG_SIGNATURE)+ sizeof(saveCounter) + size, &END_SIGNATURE, sizeof(END_SIGNATURE));
    saveCounter++;
    
    // Closing RAM (theoretically unnecessary because the counter is back to zero)
    writeEnable (0);
    
    return true;
}

#endif  //  PILOT_TARGET
