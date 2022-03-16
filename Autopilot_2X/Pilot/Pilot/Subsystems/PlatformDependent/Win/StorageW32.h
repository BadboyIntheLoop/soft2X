#ifndef STORAGEW32_H
#define STORAGEW32_H

// Includes declaration of a PILOT_TARGET platform

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

/** \file
* \brief Declaration of a class supporting mass storage as a files in Windows OS
*/

/** Class supports mass storage as a files in Windows OS
*/
/// Implementation of a class supporting mass storage as a files in Windows OS
class StorageW32: public StorageBase
{
public:
    explicit StorageW32(const char* path, bool async=false, bool singleThread=false);
    
    bool open (const char* fname);             ///< Method opens the file
    bool close (void);                         ///< Method closes the file
    bool save (const void* buf, int size);     ///< Method saves specified buffer to the file
    bool load (void* buf, int size);           ///< Method reads with validation specified buffer from file
    bool append (const char* buf, bool suppressErrMsg=false);	///< Method appends specified text line to the file
    bool read (void* buf, int size, int offset);	///< Method reads specified buffer from file
    bool clear (bool fastClear=true);				///< Method erases file content
    int getSize (void);								///< Method returns size of a file
    /// Method finds first occurence of a text pattern
    bool scan (const char* text, int &offset, int maxCount=0, const char* stopText=NULL);
    bool readLine (char* buf, int bufSize, int &offset);	///< Method reads text line from file
    bool writeFlash(const void* buf, unsigned int offset, int size);
	bool readFlash(void* buf, unsigned int offset, int size);
    bool calcFlashCRC(unsigned int offset, int size, INT16U& crc);
	bool eraseFlash(unsigned int offset, int size);

	bool mkdir(std::string path);
	bool  isDirectory(std::string path);
	std::vector<std::string> pathSplit(std::string path);
	std::vector<std::string> listFile(void);
	bool renameFile(const char* oldname, const char* newname);
	bool renameFile(std::string oldname, std::string newname);
	bool check_file(const char *fname);
	bool remove(const char *fname);
	bool remove(std::string path);
	bool hasSuffix(const std::string& s, const std::string& suffix);
	bool contain(const std::string& s, const std::string& subs);
	bool copy(std::string strFrom, std::string strTo);
	bool formatSD(void);
	bool UploadFile(ClassifiedLine& cl, std::string nameFile);

	bool appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg = false);

    void task (void* pdata);	///< Method handless operating system task

private:
    static const unsigned int BEG_SIGNATURE;	///< Begin data signature
    static const unsigned int END_SIGNATURE;	///< End data signature

    int fd;                     ///< File descriptor
    FILE* _f;                   ///< Pointer to file with stream access
    FILE* _fro;                 ///< Poniter to file with stream access for reading by scan and readLine only
    bool fileOpened;            ///< Flag indicates if file is open
    bool _async;                ///< Flag enables asynchronous usage of an 'append' function (on Win32 it is dummy)
	Semaphore _vSem;            ///< File access semaphore (functions 'append', 'scan' and 'clear')
    bool _singleThread;         ///< Flag indicates if object is created in a single threaded environment - most of the functions not available

    // Destructor is disabled - object should never be destroyed
    ~StorageW32(void){};
};

#endif  //  PILOT_TARGET
#endif  //  STORAGEW32_H
