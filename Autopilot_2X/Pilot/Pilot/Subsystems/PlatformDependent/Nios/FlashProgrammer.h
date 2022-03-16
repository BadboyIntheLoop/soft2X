#ifndef FLASHPROGRAMMER_H
#define FLASHPROGRAMMER_H

#ifndef PILOT_TARGET
	#error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Compilation on Windows only for a static analysis purpose
#if (PILOT_TARGET == PT_HARDWARE) || (PILOT_TARGET == PT_WIN32)

/** \name FlashProgrammer result values
* \{
*/
enum FLASH_PROG_RESULT
{
    FLASH_PROG_OK,				///< OK
	FLASH_PROG_UCOMMAND_ERROR,	///< invalid syntax of an 'smon flash' command
	FLASH_PROG_BADPAR_ERROR,	///< invalid parameter
	FLASH_PROG_NOT_PREP_ERROR,	///< no 'prepare' command  
	FLASH_PROG_LEN_ERROR,		///< invalid data size
	FLASH_PROG_ADDR_ERROR,		///< invalid address
	FLASH_PROG_SDRAM_CRC_ERROR,	///< SDRAM memory CRC error (data buffer)
	FLASH_PROG_WRITE_ERROR,		///< FLASH memory programming error (fatal error)
	FLASH_PROG_CRC_ERROR,		///< FLASH memory CRC error (fatal error)
    FLASH_PROG_INTERNAL_ERROR,	///< internal error (e.g an SNPRINTF instruction error) 
	FLASH_PROG_SILENT			///< no response
};
///\}

/** \file
* \brief Declaration of a class supporting programming of a flash memory by a parallel connection
*/

/** Class supports programming of a flash memory by a parallel connection e.g. bluetooth, radio link or fastRS.
* Flash memory may be programmed by an execution of a command sequence from SystemMonitor in two modes:
* - Normal mode
* In the normal mode command sequence is as follow
*  - smon flash prepare <start-address> <length> <crc>
*  - smon flash <srec-data>
*  - ...
*  - smon flash <srec-data>
*  - smon flash program
*  .
*  where:
*  - <start_address> is an address of a beginning of a memory to programm
*  - <length> is a size of a memory to programm in bytes
*  - <srec-data> is a data to be programmed in a s-record format
*  - <crc> is a control sum for data (CRC16)
*  .
* .
*
* - Fast mode
* In FAST mode programming is done without pilot's answer on a commands with <srec-data>.
* After sending a package of records, pilot is asked to address that should be sent as the next.
* Fast mode is initialized by a command:
* - smon flash prepare <start-address> <length> <crc> FAST
* .
*
* Request of an address is realized with a command
* - smon flash get address
* .
*
* \note
* - It is assumed that addresses are continuous in successive 'smon flash <srec-data>' instructions.
* The exception is repetition of the last command (due to the possibility of losing a response)
* .
*
* - For the 'smon flash <srec-data>' command only a records of type S0 (header) and S3 (data with 32-bits address) are permitted.
* Record of type S0 is ignored. The other types are treated as incorrect.
* .
*
* - Value of <length> must be greater then 0
* .
*
* - CRC sum is check out before the programming (checking the SDRAM buffer)
* and after the programming (checking the flash memory)
* .
*/

/// Implementation of a class supporting programming of a flash memory by a parallel connection
class FlashProgrammer
{
private: 
    // Copy constructor is disabled
    FlashProgrammer(FlashProgrammer&);
	// Copy operator is disabled
    FlashProgrammer& operator=(const FlashProgrammer&);

    static const unsigned int FLASH_MAX_LEN  = 0x1800000;	///< Maximum data size to programm (2MB)   //default = 0x1500000;
	static const unsigned int FLASH_ADDR_PTF = 0x200000;	///< Address of a beginning of a flash memory for programm (PTF)
	static const unsigned int FLASH_ADDR_SOF = 0x20000;		///< Address of a beginning of a flash memory for altera configuration (SOF)
    static const unsigned int MIN_SREC_LEN   = 10;			///< Minimun correct length of an s-record
    static const unsigned int MAX_SREC_DATA  = 0xFF;		///< Maximum amount of data in a record (address+data+crc)
    static const unsigned int FLASH_BLOCK_SIZE = 0x20000;	///< Size of a flash memory block
    static const unsigned int FLASH_PAGE_LEN = 256;
    static const unsigned int MCS_EXTENDED_LINEAR_ADDR = 0x10000;

	unsigned int _address;		///< Address of a beginning of a memory to be written
	unsigned int _length;		///< Size of an area in bytes
	unsigned int _cur_address; 	///< Address in a memory for a next byte to write
	unsigned int _cur_length;	///< Size of an area currently prepared for writing
	unsigned int _cmd_address;	///< Address of a data in a recently executed command
	unsigned int _cmd_length;	///< Size of a data in a recently executed command
	unsigned int _cur_page;     ///< Extended Linear Address Record
	INT16U _crc;				///< Control sum CRC16 

	unsigned char *_sdram;		///< Pointer to an SDRAM memory
    char const *_prefix;		///< Pointer to a (constant) prefix of a subsystem
    StorageBase* _flashMem;		///< Flash memory

	bool _prepared;
	bool _fastMode;				///< Working mode flag (normal/fast)

	FLASH_PROG_RESULT cmdProgram(ClassifiedLine* pcl);
    FLASH_PROG_RESULT cmdPrepare(int addr, int len, int crc, bool fast);
    FLASH_PROG_RESULT cmdSrecord(char* srec);
    FLASH_PROG_RESULT cmdGetAddress(ClassifiedLine* pcl);

	bool hh2byte(char* text, unsigned char& val);

public:

	FlashProgrammer(void);
    void setPrefix(const char *prefix);  
	FLASH_PROG_RESULT flashCmd(ClassifiedLine* pcl, FParser* parser);
};

#endif  //  PILOT_TARGET
#endif  //  FLASHPROGRAMMER_H
