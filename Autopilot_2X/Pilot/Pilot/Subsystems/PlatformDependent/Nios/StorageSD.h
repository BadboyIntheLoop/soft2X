
#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

//extern "C" {
//  #include "sdcard.h"
//  #include "ff.h"
//}
#include <string>
#define TYPE_LOG 0
#define TYPE_DAT 1

//const char LOG_FILE_NAME[] = "log";

// Compilation on Windows only for static code analysis
#if (PILOT_TARGET == PT_HARDWARE) 

/** \file
* \brief Declaration of a class supporting Flash mass memory in the NIOSII environment
*/

/** Class supports Flash mass memory in the NIOSII environment. In the flash memory are predefined areas simulating files of two types:
*	- with constant size, deleted automatically during write, writing synchronously by 'save' function, data are updated at the beginning and at the end with
*	signatures checked during reading.
*
*	- with variable size, deleted manually (log), writing asynchronously by dedicated task ('append' function).
*	
* Erasing a flash memory block of size 128KB takes about 1.5 seconds and freezes the cpu (active loop).
* Writing takes similar amount of time, reading is fast.
* There can be many objects of this class - synchronization access is via a static semaphore.
* \note Objects of this class would not work under Win32 (compilation only for static code analysis)
*/
/// Implementation of a class supporting Flash mass memory in the NIOSII environment
class StorageSD: public StorageBase
{
public:    
    explicit StorageSD(bool async=false, bool appendMode = false);
    
    bool open (const char* fname);              ///< Method opens file
    bool close (void);                          ///< Method closes file
	bool save (const void* buf, int size);      ///< Method writes specified buffer to the file
    bool load (void* buf, int size);            ///< Method reads specified buffer from file with validation
    bool append (const char* lineToWrite, bool suppressErrMsg=false);	///< Method appends specified string to the file
    bool appendBinary(const char* fileName, const void* buf, int size);
    bool isFileExist(const char* prefix, const int index);
    bool read (void* buf, int size, int offset);  ///< Method reads specified buffer from file
    bool clear (bool fastClear=true);             ///< Method erases content of a file
    int getSize (void);                           ///< Method returns file size
    int getFree (void);                           ///< Method returns free space in bytes (refers to the logu)
    bool scan (const char* text, int &offset, int maxCount=0, const char* stopText=NULL);	///< Method finds first occurrence of a text pattern
    bool readLine (char* buf, int bufSize, int &offset);	///< Method reads text line from a file
	bool writeFlash(const void* buf, unsigned int offset, int size);	///< Method writes to the flash memory (used by FlashProgrammer)
    bool calcFlashCRC(unsigned int offset, int size, INT16U& crc);	///< Method calculates CRC sum for flash (used by FlashProgrammer)
    bool readFlash(void* buf, unsigned int offset, int size);
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

    bool appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg=false);

    void task(void* pdata);                     ///< Method handles uC/OS-II system tasks

private:
    static const int FNAME_LENGTH = 255;         ///< File name buffer size
    //static const int MAX_FILES = 15;            ///< Number of elements in the file array
	/** \name Signatures of beginning and end of a data (initialized in '.cpp' because they must have physical adresses)
	* \{
	*/
    //static const unsigned int BEG_SIGNATURE;
    //static const unsigned int END_SIGNATURE;
	///\}
    
	/** \name Maximum time of waiting for an access to the flash memory by the save and load functions (in ticks).
	* It should be shorter then the calculation for a control cycle (1/24 sec).
	* \note Block erasing operation (e.g. before writing) takes about 1,5 seconds!
	*/
    static const INT16U FLASH_SEM_WAIT_MS = 50;       
    static const INT16U APPD_BUF_SEM_WAIT_MS = 30;	 // Maximum time of waiting for an access to the buffer for 'append' function (ticks)
        
	/** \name Maximum time of waiting for an access to the flash in 'append' function (in ticks)
	* It should be longer then erasing and writing of the block.
	*/
    static const INT16U APPD_FLASH_SEM_WAIT_MS = 1000; //10000
	static const int BUF_LINES = 100; //100	///< Number of lines in cyclic buffer storing lines before writing to the flash
    static const INT8U ERRWAIT = 10;	///< Waiting time in seconds for the occurrence of an error (protection against clogging the log)
    //static const char PILOT_FLASH_NAME[];	///< Flash name
    static const int BUF_LINE_NUM = 200;				///< Number of lines in cyclic buffer storing lines before writing to the SD file.
    static const int BUF_LINE_SIZE = 240;
    static const int FILE_TYPE = 2;				///< Number of file type

#if PILOT_TARGET == PT_WIN32
    static const unsigned int BLOCK_SIZE = 0x20000;	///< Size of the flash memory block (128KB)
    static const unsigned int FILES_OFFSET = 0x0;	///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
    static const unsigned int FILES_SIZE = BLOCK_SIZE*10;	///< Size reserved for files

	/* parasoft-suppress  INIT-09 "Need for static analysis" */
    static const char dummyFlash[FILES_SIZE]; ///< Flash start address
    static const unsigned int PILOT_FLASH_BASE = 0;

#elif PILOT_TARGET == PT_HARDWARE
    //static const unsigned int BLOCK_SIZE = 0x20000;	///< Size of the flash memory block (128KB)
    //static const unsigned int FILES_OFFSET = 0x400000;	///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
    //static const unsigned int FILES_SIZE = 0x1FFFFFF - FILES_OFFSET + 1;	///< Size reserved for files
    //static const unsigned int PILOT_FLASH_BASE = 0;	///< Flash start address
    
#else
    #error "PILOT_TARGET unknown (StorageSD.h)"
#endif

    // Destructor is disabled - object should never be destroyed
        ~StorageSD(void){};

    bool _async;                          ///< Flag enables asynchronous handling of an 'append' function
    OS_EVENT* _mboxSigSD;
    static bool _staticInitialized;

    typedef char TLine[BUF_LINE_SIZE];

//    uint8_t _fileType;									///< FileType = 0 : log file
    													///<          = 1 : data file

    ///< Structure describe file simulated in the SD memory
    struct FileItem2
    {
    	int fileType;							///< file type: 0 - log / 1 - dat
    	int size;								///< Size of line written to SD file from cyclic buffer
    	bool isFileOpened;
    	int countappend;
    	char fileName[FNAME_LENGTH];			///< file name for appendNewInternal
    	TLine appendCycBuf[BUF_LINE_NUM];       ///< Array of cyclic buffers for an 'append' function
    	int cbUserPos;                       	///< Array of user fields in the array of  cyclic buffer
    	int cbSystemPos;                     	///< Array of task position stored in the array of cyclic buffer
    	FIL fileSDObj;							///< File object structure
    private:
		/// Copy operator is disbaled (structure holds some constants - content cannot be change after creation)
        FileItem2& operator=(const FileItem2&);
    };

    static FileItem2 fileSD[FILE_TYPE];						///< Description of a SD file
	char _fileSDName[FNAME_LENGTH];			///< file name for appendNewInternal
	int cntToLog;
	FRESULT res;
	FATFS fatfs;
	TCHAR *_Path;
    bool _appendMode;					///< Not really use --- à voir !!!!

    //Variable for upload File
    FIL fileUpload;
    unsigned int numRead;
    typedef struct line_infor
    {
    	uint8_t endline_pos;
    	uint8_t line_num;
    }line_infor_t;

    Semaphore& _flashSem (void) const;    	///< Semaphore controlling access to the flash memory (wrapped by function)
    Semaphore& _cirBufSem(uint8_t index) const;

	bool appendNewInternal(char* buf, FileItem2* fileItem, int size = BUF_LINE_SIZE);
	bool getFileSize(const char * fname,unsigned int *len);
    static uint8_t error_status;

public:
    static uint8_t getStorageErr(void);
};

#endif  // PILOT_TARGET

