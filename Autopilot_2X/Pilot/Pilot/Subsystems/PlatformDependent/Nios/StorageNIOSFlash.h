#ifndef STORAGENIOSFLASH_H
#define STORAGENIOSFLASH_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if (PILOT_TARGET == PT_HARDWARE)
/************************** Constant Definitions *****************************/

/*
 * The following constants define the commands which may be sent to the Flash
 * device.
 */
#define WRITE_STATUS_CMD	0x01
#define WRITE_CMD			0x02
#define READ_CMD			0x03
#define WRITE_DISABLE_CMD	0x04
#define READ_STATUS_CMD		0x05
#define WRITE_ENABLE_CMD	0x06
#define FAST_READ_CMD		0x0B
#define DUAL_READ_CMD		0x3B
#define QUAD_READ_CMD		0x6B
#define BULK_ERASE_CMD		0xC7
#define	SEC_ERASE_CMD		0xD8
#define READ_ID				0x9F
#define READ_CONFIG_CMD		0x35
#define WRITE_CONFIG_CMD	0x01
#define BANK_REG_RD			0x16
#define BANK_REG_WR			0x17
/* Bank register is called Extended Address Register in Micron */
#define EXTADD_REG_RD		0xC8
#define EXTADD_REG_WR		0xC5
#define	DIE_ERASE_CMD		0xC4
#define READ_FLAG_STATUS_CMD	0x70

/*
 * The following constants define the offsets within a FlashBuffer data
 * type for each kind of data.  Note that the read data offset is not the
 * same as the write data because the QSPI driver is designed to allow full
 * duplex transfers such that the number of bytes received is the number
 * sent and received.
 */
#define COMMAND_OFFSET		0 /* Flash instruction */
#define ADDRESS_1_OFFSET	1 /* MSB byte of address to read or write */
#define ADDRESS_2_OFFSET	2 /* Middle byte of address to read or write */
#define ADDRESS_3_OFFSET	3 /* LSB byte of address to read or write */
#define DATA_OFFSET		4 /* Start of Data for Read/Write */
#define DUMMY_OFFSET		4 /* Dummy byte offset for fast, dual and quad
				     reads */
#define DUMMY_SIZE		1 /* Number of dummy bytes for fast, dual and
				     quad reads */
#define RD_ID_SIZE		4 /* Read ID command + 3 bytes ID response */
#define BULK_ERASE_SIZE		1 /* Bulk Erase command size */
#define SEC_ERASE_SIZE		4 /* Sector Erase command + Sector address */
#define BANK_SEL_SIZE	2 /* BRWR or EARWR command + 1 byte bank value */
#define RD_CFG_SIZE		2 /* 1 byte Configuration register + RD CFG command*/
#define WR_CFG_SIZE		3 /* WRR command + 1 byte each Status and Config Reg*/
#define DIE_ERASE_SIZE	4	/* Die Erase command + Die address */

/*
 * The following constants specify the extra bytes which are sent to the
 * Flash on the QSPI interface, that are not data, but control information
 * which includes the command and address
 */
#define OVERHEAD_SIZE		4

/*
 * Base address of Flash1
 */
#define FLASH1BASE 0x0000000

/*
 * Sixteen MB
 */
#define SIXTEENMB 0x1000000


/*
 * Mask for quad enable bit in Flash configuration register
 */
#define FLASH_QUAD_EN_MASK 0x02

#define FLASH_SRWD_MASK 0x80

/*
 * Bank mask
 */
#define BANKMASK 0xF000000

/*
 * Identification of Flash
 * Micron:
 * Byte 0 is Manufacturer ID;
 * Byte 1 is first byte of Device ID - 0xBB or 0xBA
 * Byte 2 is second byte of Device ID describes flash size:
 * 128Mbit : 0x18; 256Mbit : 0x19; 512Mbit : 0x20
 * Spansion:
 * Byte 0 is Manufacturer ID;
 * Byte 1 is Device ID - Memory Interface type - 0x20 or 0x02
 * Byte 2 is second byte of Device ID describes flash size:
 * 128Mbit : 0x18; 256Mbit : 0x19; 512Mbit : 0x20
 */
#define MICRON_ID_BYTE0		0x20
#define MICRON_ID_BYTE2_128	0x18
#define MICRON_ID_BYTE2_256	0x19
#define MICRON_ID_BYTE2_512	0x20
#define MICRON_ID_BYTE2_1G	0x21

#define SPANSION_ID_BYTE0		0x01
#define SPANSION_ID_BYTE2_128	0x18
#define SPANSION_ID_BYTE2_256	0x19
#define SPANSION_ID_BYTE2_512	0x20

#define WINBOND_ID_BYTE0		0xEF
#define WINBOND_ID_BYTE2_128	0x18

#define MACRONIX_ID_BYTE0		0xC2
#define MACRONIX_ID_BYTE2_256	0x19
#define MACRONIX_ID_BYTE2_512	0x1A
#define MACRONIX_ID_BYTE2_1G	0x1B

#define ISSI_ID_BYTE0			0x9D
#define ISSI_ID_BYTE2_256		0x19
/*
 * The index for Flash config table
 */
/* Spansion*/
#define SPANSION_INDEX_START			0
#define FLASH_CFG_TBL_SINGLE_128_SP		SPANSION_INDEX_START
#define FLASH_CFG_TBL_STACKED_128_SP	(SPANSION_INDEX_START + 1)
#define FLASH_CFG_TBL_PARALLEL_128_SP	(SPANSION_INDEX_START + 2)
#define FLASH_CFG_TBL_SINGLE_256_SP		(SPANSION_INDEX_START + 3)
#define FLASH_CFG_TBL_STACKED_256_SP	(SPANSION_INDEX_START + 4)
#define FLASH_CFG_TBL_PARALLEL_256_SP	(SPANSION_INDEX_START + 5)
#define FLASH_CFG_TBL_SINGLE_512_SP		(SPANSION_INDEX_START + 6)
#define FLASH_CFG_TBL_STACKED_512_SP	(SPANSION_INDEX_START + 7)
#define FLASH_CFG_TBL_PARALLEL_512_SP	(SPANSION_INDEX_START + 8)

/* Micron */
#define MICRON_INDEX_START				(FLASH_CFG_TBL_PARALLEL_512_SP + 1)
#define FLASH_CFG_TBL_SINGLE_128_MC		MICRON_INDEX_START
#define FLASH_CFG_TBL_STACKED_128_MC	(MICRON_INDEX_START + 1)
#define FLASH_CFG_TBL_PARALLEL_128_MC	(MICRON_INDEX_START + 2)
#define FLASH_CFG_TBL_SINGLE_256_MC		(MICRON_INDEX_START + 3)
#define FLASH_CFG_TBL_STACKED_256_MC	(MICRON_INDEX_START + 4)
#define FLASH_CFG_TBL_PARALLEL_256_MC	(MICRON_INDEX_START + 5)
#define FLASH_CFG_TBL_SINGLE_512_MC		(MICRON_INDEX_START + 6)
#define FLASH_CFG_TBL_STACKED_512_MC	(MICRON_INDEX_START + 7)
#define FLASH_CFG_TBL_PARALLEL_512_MC	(MICRON_INDEX_START + 8)
#define FLASH_CFG_TBL_SINGLE_1GB_MC		(MICRON_INDEX_START + 9)
#define FLASH_CFG_TBL_STACKED_1GB_MC	(MICRON_INDEX_START + 10)
#define FLASH_CFG_TBL_PARALLEL_1GB_MC	(MICRON_INDEX_START + 11)

/* Winbond */
#define WINBOND_INDEX_START				(FLASH_CFG_TBL_PARALLEL_1GB_MC + 1)
#define FLASH_CFG_TBL_SINGLE_128_WB		WINBOND_INDEX_START
#define FLASH_CFG_TBL_STACKED_128_WB	(WINBOND_INDEX_START + 1)
#define FLASH_CFG_TBL_PARALLEL_128_WB	(WINBOND_INDEX_START + 2)

/* Macronix */
#define MACRONIX_INDEX_START			(FLASH_CFG_TBL_PARALLEL_128_WB + 1 - 3)
#define FLASH_CFG_TBL_SINGLE_256_MX		MACRONIX_INDEX_START
#define FLASH_CFG_TBL_STACKED_256_MX	(MACRONIX_INDEX_START + 1)
#define FLASH_CFG_TBL_PARALLEL_256_MX	(MACRONIX_INDEX_START + 2)
#define FLASH_CFG_TBL_SINGLE_512_MX		(MACRONIX_INDEX_START + 3)
#define FLASH_CFG_TBL_STACKED_512_MX	(MACRONIX_INDEX_START + 4)
#define FLASH_CFG_TBL_PARALLEL_512_MX	(MACRONIX_INDEX_START + 5)
#define FLASH_CFG_TBL_SINGLE_1G_MX		(MACRONIX_INDEX_START + 6)
#define FLASH_CFG_TBL_STACKED_1G_MX		(MACRONIX_INDEX_START + 7)
#define FLASH_CFG_TBL_PARALLEL_1G_MX	(MACRONIX_INDEX_START + 8)
/* ISSI */
#define ISSI_INDEX_START				(FLASH_CFG_TBL_PARALLEL_1G_MX + 1)
#define FLASH_CFG_TBL_SINGLE_256_ISSI	ISSI_INDEX_START
#define FLASH_CFG_TBL_STACKED_256_ISSI	(ISSI_INDEX_START + 1)
#define FLASH_CFG_TBL_PARALLEL_256_ISSI	(ISSI_INDEX_START + 2)
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define QSPI_DEVICE_ID		XPAR_XQSPIPS_0_DEVICE_ID
/*
 * The following defines are for dual flash stacked mode interface.
 */
#define LQSPI_CR_FAST_QUAD_READ		0x0000006B /* Fast Quad Read output */
#define LQSPI_CR_1_DUMMY_BYTE		0x00000100 /* 1 Dummy Byte between
						     address and return data */

#define DUAL_STACK_CONFIG_WRITE		(XQSPIPS_LQSPI_CR_TWO_MEM_MASK | \
					 LQSPI_CR_1_DUMMY_BYTE | \
					 LQSPI_CR_FAST_QUAD_READ)

#define DUAL_QSPI_CONFIG_WRITE		(XQSPIPS_LQSPI_CR_TWO_MEM_MASK | \
					 XQSPIPS_LQSPI_CR_SEP_BUS_MASK | \
					 LQSPI_CR_1_DUMMY_BYTE | \
					 LQSPI_CR_FAST_QUAD_READ)

/*
 * Number of flash pages to be written.
 */
#define PAGE_COUNT		32

/*
 * Max page size to initialize write and read buffer
 */
#define MAX_PAGE_SIZE 1024

/*
 * Flash address to which data is to be written.
 */
#define TEST_ADDRESS		0x000000


#define UNIQUE_VALUE		0x06

static XQspiPs QspiInstance;
// Compilation on Windows only for static code analysis


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
class StorageNIOSFlash: public StorageBase
{
public:    
    explicit StorageNIOSFlash(bool async=false, bool singleThread=false);
    
    bool open (const char* fname);              ///< Method opens file
    bool close (void);                          ///< Method closes file
	bool save (const void* buf, int size);      ///< Method writes specified buffer to the file
    bool load (void* buf, int size);            ///< Method reads specified buffer from file with validation
    bool append (const char* lineToWrite, bool suppressErrMsg=false);	///< Method appends specified string to the file
    bool appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg=false);
    bool read (void* buf, int size, int offset);  ///< Method reads specified buffer from file
    bool clear (bool fastClear=true);             ///< Method erases content of a file
    int getSize (void);                           ///< Method returns file size
    int getFree (void);                           ///< Method returns free space in bytes (refers to the logu)
    bool scan (const char* text, int &offset, int maxCount=0, const char* stopText=NULL);	///< Method finds first occurrence of a text pattern
    bool readLine (char* buf, int bufSize, int &offset);	///< Method reads text line from a file
	bool writeFlash(const void* buf, unsigned int offset, int size);	///< Method writes to the flash memory (used by FlashProgrammer)
    bool readFlash(void* buf, unsigned int offset, int size);    ///< Method writes to the flash memory (used by FlashProgrammer)
    bool eraseFlash(unsigned int offset, int size);
    bool calcFlashCRC(unsigned int offset, int size, INT16U& crc);	///< Method calculates CRC sum for flash (used by FlashProgrammer)
    int QspiFlashInit(XQspiPs *QspiInstancePtr, u16 QspiDeviceId);
    //
    void FlashErase(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 *WriteBfrPtr);
    void FlashWrite(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command, u8 *WriteBfrPtr);
    int FlashReadID(XQspiPs *QspiPtr, u8 *WriteBfrPtr, u8 *ReadBfrPtr);
    void FlashRead(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command,
    				u8 *WriteBfrPtr, u8 *ReadBfrPtr);
    int SendBankSelect(XQspiPs *QspiPtr, u8 *WriteBfrPtr, u32 BankSel);
    void BulkErase(XQspiPs *QspiPtr, u8 *WriteBfrPtr);
    void DieErase(XQspiPs *QspiPtr, u8 *WriteBfrPtr);
    u32 GetRealAddr(XQspiPs *QspiPtr, u32 Address);
    unsigned int buffToUint(u8* buf);
    void copyArray(u8* from, u8* to, unsigned int length, unsigned int offset);

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

    void task(void* pdata);                     ///< Method handles uC/OS-II system tasks

private:
    static const int FNAME_LENGTH = 20;         ///< File name buffer size
    static const int MAX_FILES = 16;            ///< Number of elements in the file array
    static const int PAGESIZE = 256;
	/** \name Signatures of beginning and end of a data (initialized in '.cpp' because they must have physical adresses)
	* \{
	*/
    static const unsigned int BEG_SIGNATURE;
    static const unsigned int END_SIGNATURE;
	///\}
    
	/** \name Maximum time of waiting for an access to the flash memory by the save and load functions (in ticks).
	* It should be shorter then the calculation for a control cycle (1/24 sec).
	* \note Block erasing operation (e.g. before writing) takes about 1,5 seconds!
	*/
    static const INT16U FLASH_SEM_WAIT_MS = 10;
    static const INT16U APPD_BUF_SEM_WAIT_MS = 30;	 // Maximum time of waiting for an access to the buffer for 'append' function (ticks)
        
	/** \name Maximum time of waiting for an access to the flash in 'append' function (in ticks)
	* It should be longer then erasing and writing of the block.
	*/
    static const INT16U APPD_FLASH_SEM_WAIT_MS = 10000;
	static const int BUF_LINES = 100;	///< Number of lines in cyclic buffer storing lines before writing to the flash
    static const INT8U ERRWAIT = 10;	///< Waiting time in seconds for the occurrence of an error (protection against clogging the log)
    static const char PILOT_FLASH_NAME[];	///< Flash name

#if PILOT_TARGET == PT_WIN32
    static const unsigned int BLOCK_SIZE = 0x20000;	///< Size of the flash memory block (128KB)
    static const unsigned int FILES_OFFSET = 0x0;	///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
    static const unsigned int FILES_SIZE = BLOCK_SIZE*10;	///< Size reserved for files

	/* parasoft-suppress  INIT-09 "Need for static analysis" */
    static const char dummyFlash[FILES_SIZE]; ///< Flash start address
    static const unsigned int PILOT_FLASH_BASE = 0;

#elif PILOT_TARGET == PT_HARDWARE
    static const unsigned int BLOCK_SIZE = 0x20000; ///< Size of the flash memory block (128KB)
    static const unsigned int FILES_OFFSET = 0x1600000;  ///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
//    static const unsigned int FILES_OFFSET = 0x00000;  ///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
    static const unsigned int ELF_FILE_OFFSET = 0x200000;   ///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
    static const unsigned int SOF_FILE_OFFSET = 0x20000;    ///< Offset from the beginning of the flash memory to the array of files (it must be a multiple of four bytes)
    static const unsigned int MAX_FILE_LEN    = 0x200000;   ///< Maximum data size to programm (2MB)
    static const unsigned int FILES_SIZE = 0x1FFFFFF - FILES_OFFSET + 1;    ///< Size reserved for files
    static const unsigned int PILOT_FLASH_BASE = CFI_FLASH_BASE;    ///< Flash start address
    
#else
    #error "PILOT_TARGET unknown (StorageNIOSFlash.h)"
#endif

    typedef char TLine[LINESIZE];
    typedef char TPage[PAGESIZE];

    struct FileItem                 ///< Structure describe file silumated in the flash memory
    {
        const char fileName[FNAME_LENGTH];          ///< File name
        const unsigned int fileSize;                ///< Size in bytes
        const unsigned int fileOffset;              ///< Offset from the beginning of memory to the array of files
        const bool appendOnly;                      ///< Flag enables writing only with the 'append' function
               unsigned int appendNextFilePos;      ///< Next empty position in file              
			  /** \name Fields to support asynchronous
			  * \{
			  */
              TLine* appendCycBuf;                  ///< Pointer to a cyclic buffer for an 'append' function
              int cbUserPos;                        ///< User fields in a cyclic buffer
              int cbSystemPos;                      ///< Task position stored in a cyclic buffer
              TPage* appendFlashBuf;
              unsigned int posInTPage;
              unsigned int posPage;
			  ///\}

    private:
		/// Copy operator is disbaled (structure holds some constants - content cannot be change after creation)
        FileItem& operator=(const FileItem&);
    };
    
    /**************************** Type Definitions *******************************/

    struct FlashInfo{
    	u32 SectSize;		/* Individual sector size or
    						 * combined sector size in case of parallel config*/
    	u32 NumSect;		/* Total no. of sectors in one/two flash devices */
    	u32 PageSize;		/* Individual page size or
    						 * combined page size in case of parallel config*/
    	u32 NumPage;		/* Total no. of pages in one/two flash devices */
    	u32 FlashDeviceSize;	/* This is the size of one flash device
    						 * NOT the combination of both devices, if present
    						 */
    	u8 ManufacturerID;	/* Manufacturer ID - used to identify make */
    	u8 DeviceIDMemSize;	/* Byte of device ID indicating the memory size */
    	u32 SectMask;		/* Mask to get sector start address */
    	u8 NumDie;			/* No. of die forming a single flash */
    private:
		/// Copy operator is disbaled (structure holds some constants - content cannot be change after creation)
    	FlashInfo& operator=(const FlashInfo&);
    };

    int _fIdent;                          ///< File identifier
    bool _fileOpened;                     ///< Flag indicates if file is open
    bool _async;                          ///< Flag enables asynchronous handling of an 'append' function
    // mailbox do synchronizacji zadania zapisuj¹cego.
    // Mo¿e byæ kilka zadañ, ka¿de dla innego pliku, dlatego nie jest statyczne.
    OS_EVENT* _mboxSig;
    static bool _staticInitialized;
    bool _singleThread;                   ///< Flag indicates if object is created in a single threaded environment - most of the functions not available

    static bool _initFlash;
    static u32 FlashMake;
    static u32 FCTIndex;	/* Flash configuration table index */
    static FlashInfo Flash_Config_Table[34];  ///<Flash Config Table
    static FileItem files[MAX_FILES];     ///< Array of descriptions of a predefied file
    static TLine linBuf1[BUF_LINES];      ///< Cyclic buffer of lines to write for a single file
    static TPage lineBufPage[1];

    Semaphore& _bufSem (void) const;	  ///< Semaphore controlling access to the buffer of an 'append' function (wrapped by function)
    Semaphore& _flashSem (void) const;    ///< Semaphore controlling access to the flash memory (wrapped by function)

    // Destructor is disabled - object should never be destroyed
    ~StorageNIOSFlash(void){};

    int findEndOfFile (const char* buf, int size);
    bool append2 (FileItem* fi, const char* buf, int size);
};

#endif  // PILOT_TARGET
#endif  // STORAGENIOSFLASH_H
