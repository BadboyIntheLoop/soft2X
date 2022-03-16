#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Compilation on Windows only for static analysis purpose
#if (PILOT_TARGET == PT_HARDWARE) 

#if PILOT_TARGET == PT_WIN32
const char StorageNIOSFlash::PILOT_FLASH_NAME[] = CFI_FLASH_NAME;

#elif PILOT_TARGET == PT_HARDWARE

u8 ReadBuffer[(PAGE_COUNT * MAX_PAGE_SIZE) + (DATA_OFFSET + DUMMY_SIZE)*8];
u8 WriteBuffer[(PAGE_COUNT * MAX_PAGE_SIZE) + DATA_OFFSET];
u8 write_buf[5000];
u8 read_buf[PAGE_COUNT * MAX_PAGE_SIZE];

#else
    #error "PILOT_TARGET undefined (StorageNIOSFlash.cpp)"
#endif

// Initialization of a static class members
bool StorageNIOSFlash::_staticInitialized = false;
StorageNIOSFlash::TLine StorageNIOSFlash::linBuf1[BUF_LINES] = {0};
StorageNIOSFlash::TPage StorageNIOSFlash::lineBufPage[1] = {0};
const unsigned int StorageNIOSFlash::BEG_SIGNATURE = 0xf1c0409b;
const unsigned int StorageNIOSFlash::END_SIGNATURE = 0x20ae0cf6;
bool StorageNIOSFlash::_initFlash = false;
u32 StorageNIOSFlash::FlashMake = 0;
u32 StorageNIOSFlash::FCTIndex = 0;


/** \name Predefined files.
* If declared array is greater than number of stored elements unused positions shoud be initialized with 0 (C++ guarantees that).
* Unused fields in structure are also initialized with 0 by default. Currently files location should starts at the beginning of a flash block.
* Incremental file (log) may have a several block in length. 
*/
StorageNIOSFlash::FileItem StorageNIOSFlash::files[] = {
    {"fpdefault",    20000, 0, false, 0},
    {"fplowbat",     20000, 1*BLOCK_SIZE, false, 0},
    {"fpreserved1",  20000, 2*BLOCK_SIZE, false, 0},
	{"ahrs2cfg",      2000, 3*BLOCK_SIZE, false, 0},
	{"pstatecfg",     2000, 5*BLOCK_SIZE, false, 0},
	{"gaugecfg",      2000, 7*BLOCK_SIZE, false, 0},
    {"smoncfg",       2000, 9*BLOCK_SIZE, false, 0},
    {"fprealcfg",     2000, 11*BLOCK_SIZE, false, 0},
    {"fcontrolcfg1",  4800, 13*BLOCK_SIZE, false, 0},
    {"servmancfg",    2000, 15*BLOCK_SIZE, false, 0},
    {"test",      	  2000, 17*BLOCK_SIZE, false, 0},
    {"cameracfg",     2000, 19*BLOCK_SIZE, false, 0},
    {"globalparams",  2000, 21*BLOCK_SIZE, false, 0},
    {"serialno",      2000, 23*BLOCK_SIZE, false, 0},
	{"resetcnt",      2000, 25*BLOCK_SIZE, false, 0},// never change localization in flash!
    {"log",          FILES_SIZE-26*BLOCK_SIZE, 26*BLOCK_SIZE, true, 0, linBuf1, 0, 0, lineBufPage, 0, 0}
};

StorageNIOSFlash::FlashInfo StorageNIOSFlash::Flash_Config_Table[34] = {
		/* Spansion */
		{0x10000, 0x100, 256, 0x10000, 0x1000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_128, 0xFFFF0000, 1},
		{0x10000, 0x200, 256, 0x20000, 0x1000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_128, 0xFFFF0000, 1},
		{0x20000, 0x100, 512, 0x10000, 0x1000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_128, 0xFFFE0000, 1},
		{0x10000, 0x200, 256, 0x20000, 0x2000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_256, 0xFFFF0000, 1},
		{0x10000, 0x400, 256, 0x40000, 0x2000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_256, 0xFFFF0000, 1},
		{0x20000, 0x200, 512, 0x20000, 0x2000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_256, 0xFFFE0000, 1},
		{0x40000, 0x100, 512, 0x20000, 0x4000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_512, 0xFFFC0000, 1},
		{0x40000, 0x200, 512, 0x40000, 0x4000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_512, 0xFFFC0000, 1},
		{0x80000, 0x100, 1024, 0x20000, 0x4000000,
				SPANSION_ID_BYTE0, SPANSION_ID_BYTE2_512, 0xFFF80000, 1},
		/* Spansion 1Gbit is handled as 512Mbit stacked */
		/* Micron */
		{0x10000, 0x100, 256, 0x10000, 0x1000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_128, 0xFFFF0000, 1},
		{0x10000, 0x200, 256, 0x20000, 0x1000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_128, 0xFFFF0000, 1},
		{0x20000, 0x100, 512, 0x10000, 0x1000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_128, 0xFFFE0000, 1},
		{0x10000, 0x200, 256, 0x20000, 0x2000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_256, 0xFFFF0000, 1},
		{0x10000, 0x400, 256, 0x40000, 0x2000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_256, 0xFFFF0000, 1},
		{0x20000, 0x200, 512, 0x20000, 0x2000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_256, 0xFFFE0000, 1},
		{0x10000, 0x400, 256, 0x40000, 0x4000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_512, 0xFFFF0000, 2},
		{0x10000, 0x800, 256, 0x80000, 0x4000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_512, 0xFFFF0000, 2},
		{0x20000, 0x400, 512, 0x40000, 0x4000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_512, 0xFFFE0000, 2},
		{0x10000, 0x800, 256, 0x80000, 0x8000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_1G, 0xFFFF0000, 4},
		{0x10000, 0x1000, 256, 0x100000, 0x8000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_1G, 0xFFFF0000, 4},
		{0x20000, 0x800, 512, 0x80000, 0x8000000,
				MICRON_ID_BYTE0, MICRON_ID_BYTE2_1G, 0xFFFE0000, 4},
		/* Winbond */
		{0x10000, 0x100, 256, 0x10000, 0x1000000,
				WINBOND_ID_BYTE0, WINBOND_ID_BYTE2_128, 0xFFFF0000, 1},
		{0x10000, 0x200, 256, 0x20000, 0x1000000,
				WINBOND_ID_BYTE0, WINBOND_ID_BYTE2_128, 0xFFFF0000, 1},
		{0x20000, 0x100, 512, 0x10000, 0x1000000,
				WINBOND_ID_BYTE0, WINBOND_ID_BYTE2_128, 0xFFFE0000, 1},
		/* Macronix */
		{0x10000, 0x200, 256, 0x20000, 0x2000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_256, 0xFFFF0000, 1},
		{0x10000, 0x400, 256, 0x40000, 0x2000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_256, 0xFFFF0000, 1},
		{0x20000, 0x200, 512, 0x20000, 0x2000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_256, 0xFFFE0000, 1},
		{0x10000, 0x400, 256, 0x40000, 0x4000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_512, 0xFFFF0000, 1},
		{0x10000, 0x800, 256, 0x80000, 0x4000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_512, 0xFFFF0000, 1},
		{0x20000, 0x400, 512, 0x40000, 0x4000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_512, 0xFFFE0000, 1},
		{0x2000, 0x4000, 256, 0x80000, 0x8000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_1G, 0xFFFF0000, 1},
		{0x2000, 0x8000, 256, 0x100000, 0x8000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_1G, 0xFFFF0000, 1},
		{0x4000, 0x4000, 512, 0x80000, 0x8000000,
				MACRONIX_ID_BYTE0, MACRONIX_ID_BYTE2_1G, 0xFFFE0000, 1},
		/* ISSI */
		{0x10000, 0x200, 256, 0x20000, 0x2000000,
				ISSI_ID_BYTE0, ISSI_ID_BYTE2_256, 0xFFFF0000, 1}
};

/** \name Method wraps a static semaphore. Method is needed to guarantee the static initialization order.
*/
Semaphore& StorageNIOSFlash::_bufSem (void) const
{
    static Semaphore xBufSem;
    return xBufSem; 
}

/** \name Method wraps a static semaphore. Method is needed to guarantee the static initialization order.
*/
Semaphore& StorageNIOSFlash::_flashSem (void) const
{
    static Semaphore xFlashSem;
    return xFlashSem;
}

/** \name Class constructor
* \param 'async' - flag enables asynchronous handling of an 'append' function (through dedicated task)
*/
StorageNIOSFlash::StorageNIOSFlash(bool async, bool singleThread):
    _fIdent(0),
    _fileOpened(false),
    _async(async),
    _mboxSig(static_cast<OS_EVENT*>(0)),
    _singleThread(singleThread)
//	FlashMake(SPANSION_ID_BYTE0),
//	FCTIndex(0)
{
	// Creation of a mailbox of the uC/OS-II system to synchronize with writing thread (for each object of this class)
    if (async)
    {
        _mboxSig = OSMboxCreate (static_cast<void*>(0));
        if (_mboxSig == static_cast<OS_EVENT*>(0)) 
        {
            Log.abort ("Critical Error: StorageNIOSFlash_1.");
            return;
        }
    }

	// Creation of a semaphores common to all objects of this class (if they have not been created already)
	// Initialized at the beginning as a static members
    if (!_staticInitialized)
    {
		// Semaphore controlling access to flash memory
        if (!_flashSem().create("StorageNIOSFlash_flash"))
        {
            Log.abort ("Critical Error: StorageNIOSFlash_2.");
            return;
        }

		// Semaphore controlling access to the buffer for 'append' function
        if (!_bufSem().create("StorageNIOSFlash_buf"))
        {
            Log.abort ("Critical Error: StorageNIOSFlash_3.");
            return;
        }

		// Validation of an array of files
        unsigned int a = 0;
        for (int i=0; (i<MAX_FILES) && (files[i].fileName[0] != 0); i++)
        {
            if (a > files[i].fileOffset)
            {
				// Ranges overlapped
                Log.abort ("Critical Error: StorageNIOSFlash_4.");
                return;
            }
            a = files[i].fileOffset + files[i].fileSize;
            
            if (files[i].fileSize > BLOCK_SIZE && !files[i].appendOnly)
            {
				// File length greater then size of block
                Log.abort ("Critical Error: StorageNIOSFlash_5.");
                return;
            }
        }
    }
   
   _staticInitialized = true;
}

/** \name Method creates new file or opens it if it's created already.
* In single object only one file may be opened. If some file was opened before, it is going to be closed.
* A file cannot be opened by a multiple objects simultaneously. A file name must be one of the predefined file names.
* \param 'fname' - file name
* \return 'true' on success, 'false' otherwise
*/
bool StorageNIOSFlash::open (const char* fName)
{
    // Closing of an opened file
    if (_fileOpened)
        close ();
    if(!_initFlash && Os->isStarted())
    {
    	int Status = QspiFlashInit(&QspiInstance, QSPI_DEVICE_ID);
    	if(Status != XST_SUCCESS)
    	{
    		Log.errorPrintf("StorageNIOSFlash_open_0 [%s]", fName);
    		return false;
    	}
    	else
        	_initFlash = true;

    }
    
    // Searching for a file name in a file name array
    for (int i=0; i<MAX_FILES; i++)
    {
        if (STRNICMP (fName, files[i].fileName, FNAME_LENGTH) == 0)
        {
            _fileOpened = true;
            _fIdent = i;

            if (files[_fIdent].appendOnly)
            {
				// Reseting of a position in a cyclic buffer
                files[_fIdent].cbSystemPos = 0;
                files[_fIdent].cbUserPos = 0;
                if(_initFlash)
                {
                	unsigned int num_page = files[_fIdent].fileSize/(PAGE_COUNT*MAX_PAGE_SIZE);
                	for(unsigned int page=0; page < num_page; page++)
                	{
                		FlashRead(&QspiInstance, FILES_OFFSET + files[_fIdent].fileOffset + page*(PAGE_COUNT*MAX_PAGE_SIZE), (PAGE_COUNT*MAX_PAGE_SIZE), QUAD_READ_CMD, WriteBuffer, read_buf);
//                		for(int i=0; i<PAGE_COUNT*MAX_PAGE_SIZE; i++)
//                		{
//                			if(read_buf[i] != 0xFF)
//                			{
//                				u8 c = read_buf[i];
//                			}
////                				files[_fIdent].appendNextFilePos = page*(PAGE_COUNT*MAX_PAGE_SIZE) + i;
//                		}
                		unsigned int appendNextFilePosTmp = findEndOfFile ((const char*)read_buf, (PAGE_COUNT*MAX_PAGE_SIZE));
                		if(appendNextFilePosTmp != (PAGE_COUNT*MAX_PAGE_SIZE) && appendNextFilePosTmp != 0x0)
                		{
                			files[_fIdent].appendNextFilePos = page*(PAGE_COUNT*MAX_PAGE_SIZE) + appendNextFilePosTmp;
                			return true;
                		}
                		else
                		{
                			if(page == 0 && appendNextFilePosTmp == 0)
                			{
                				files[_fIdent].appendNextFilePos = 0;
                				return true;
                			}
                			if(page == num_page - 1 && appendNextFilePosTmp == PAGE_COUNT*MAX_PAGE_SIZE)
                			{
                				files[_fIdent].appendNextFilePos = page*(PAGE_COUNT*MAX_PAGE_SIZE) + appendNextFilePosTmp;;
                				return true;
                			}
                		}
                	}
                }

            }
            return true;
        }
    }

    Log.errorPrintf("StorageNIOSFlash_open_1 [%s]", fName);
    return false;
}

/** \name Method closes file.
* Dummy method which changes only an file status of open
*/
bool StorageNIOSFlash::close (void)
{
    _fileOpened = false;
    return true;
}


/*****************************************************************************/
/**
*
*
* This function writes to the  serial Flash connected to the QSPI interface.
* All the data put into the buffer must be in the same page of the device with
* page boundaries being on 256 byte boundaries.
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	Address contains the address to write data to in the Flash.
* @param	ByteCount contains the number of bytes to write.
* @param	Command is the command used to write data to the flash. QSPI
*		device supports only Page Program command to write data to the
*		flash.
* @param	Pointer to the write buffer (which is to be transmitted)
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void StorageNIOSFlash::FlashWrite (XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command, u8 *WriteBfrPtr)
{
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
	u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  /* Must send 2 bytes */
	u8 FlashStatus[2];
	u32 RealAddr;
	u32 BankSel;
	u8 ReadFlagSRCmd[] = {READ_FLAG_STATUS_CMD, 0};
	u8 FlagStatus[2];

	/*
	 * Translate address based on type of connection
	 * If stacked assert the slave select based on address
	 */
	RealAddr = GetRealAddr(QspiPtr, Address);
	/*
	 * Bank Select
	 */
	if(Flash_Config_Table[FCTIndex].FlashDeviceSize > SIXTEENMB) {
		/*
		 * Calculate bank
		 */
		BankSel = RealAddr/SIXTEENMB;
		/*
		 * Select bank
		 */
		SendBankSelect(QspiPtr, WriteBfrPtr, BankSel);
	}

	/*
	 * Send the write enable command to the Flash so that it can be
	 * written to, this needs to be sent as a separate transfer before
	 * the write
	 */
	XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,
				sizeof(WriteEnableCmd));


	/*
	 * Setup the write command with the specified address and data for the
	 * Flash
	 */
	/*
	 * This will ensure a 3B address is transferred even when address
	 * is greater than 128Mb.
	 */
	WriteBfrPtr[COMMAND_OFFSET]   = Command;
	WriteBfrPtr[ADDRESS_1_OFFSET] = (u8)((RealAddr & 0xFF0000) >> 16);
	WriteBfrPtr[ADDRESS_2_OFFSET] = (u8)((RealAddr & 0xFF00) >> 8);
	WriteBfrPtr[ADDRESS_3_OFFSET] = (u8)(RealAddr & 0xFF);

	/*
	 * Send the write command, address, and data to the Flash to be
	 * written, no receive buffer is specified since there is nothing to
	 * receive
	 */
	XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, NULL,
				ByteCount + OVERHEAD_SIZE);

	if((Flash_Config_Table[FCTIndex].NumDie > 1) &&
			(FlashMake == MICRON_ID_BYTE0)) {
		XQspiPs_PolledTransfer(QspiPtr, ReadFlagSRCmd, FlagStatus,
					sizeof(ReadFlagSRCmd));
	}
	/*
	 * Wait for the write command to the Flash to be completed, it takes
	 * some time for the data to be written
	 */
	while (1) {
		/*
		 * Poll the status register of the Flash to determine when it
		 * completes, by sending a read status command and receiving the
		 * status byte
		 */
		XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd, FlashStatus,
					sizeof(ReadStatusCmd));

		/*
		 * If the status indicates the write is done, then stop waiting,
		 * if a value of 0xFF in the status byte is read from the
		 * device and this loop never exits, the device slave select is
		 * possibly incorrect such that the device status is not being
		 * read
		 */
		if ((FlashStatus[1] & 0x01) == 0) {
			break;
		}
	}

		if((Flash_Config_Table[FCTIndex].NumDie > 1) &&
			(FlashMake == MICRON_ID_BYTE0)) {
		XQspiPs_PolledTransfer(QspiPtr, ReadFlagSRCmd, FlagStatus,
					sizeof(ReadFlagSRCmd));
	}

}

/*****************************************************************************/
/**
*
*
* This function erases the sectors in the  serial Flash connected to the
* QSPI interface.
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	Address contains the address of the first sector which needs to
*		be erased.
* @param	ByteCount contains the total size to be erased.
* @param	Pointer to the write buffer (which is to be transmitted)
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void StorageNIOSFlash::FlashErase(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 *WriteBfrPtr)
{
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
	u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  /* Must send 2 bytes */
	u8 FlashStatus[2];
	int Sector;
	u32 RealAddr;
	u32 LqspiCr;
	u32 NumSect;
	u32 BankSel;
	u8 BankInitFlag = 1;
	u8 ReadFlagSRCmd[] = { READ_FLAG_STATUS_CMD, 0 };
	u8 FlagStatus[2];

	/*
	 * If erase size is same as the total size of the flash, use bulk erase
	 * command or die erase command multiple times as required
	 */
	if (ByteCount == ((Flash_Config_Table[FCTIndex]).NumSect *
			(Flash_Config_Table[FCTIndex]).SectSize) ) {

		if(QspiPtr->Config.ConnectionMode == XQSPIPS_CONNECTION_MODE_STACKED){

			/*
			 * Get the current LQSPI configuration register value
			 */
			LqspiCr = XQspiPs_GetLqspiConfigReg(QspiPtr);
			/*
			 * Set selection to L_PAGE
			 */
			XQspiPs_SetLqspiConfigReg(QspiPtr,
					LqspiCr & (~XQSPIPS_LQSPI_CR_U_PAGE_MASK));

			/*
			 * Assert the Flash chip select.
			 */
			XQspiPs_SetSlaveSelect(QspiPtr);
		}

		if(Flash_Config_Table[FCTIndex].NumDie == 1) {
			/*
			 * Call Bulk erase
			 */
			BulkErase(QspiPtr, WriteBfrPtr);
		}

		if(Flash_Config_Table[FCTIndex].NumDie > 1) {
			/*
			 * Call Die erase
			 */
			DieErase(QspiPtr, WriteBfrPtr);
		}
		/*
		 * If stacked mode, bulk erase second flash
		 */
		if(QspiPtr->Config.ConnectionMode == XQSPIPS_CONNECTION_MODE_STACKED){

			/*
			 * Get the current LQSPI configuration register value
			 */
			LqspiCr = XQspiPs_GetLqspiConfigReg(QspiPtr);
			/*
			 * Set selection to U_PAGE
			 */
			XQspiPs_SetLqspiConfigReg(QspiPtr,
					LqspiCr | XQSPIPS_LQSPI_CR_U_PAGE_MASK);

			/*
			 * Assert the Flash chip select.
			 */
			XQspiPs_SetSlaveSelect(QspiPtr);

			if(Flash_Config_Table[FCTIndex].NumDie == 1) {
				/*
				 * Call Bulk erase
				 */
				BulkErase(QspiPtr, WriteBfrPtr);
			}

			if(Flash_Config_Table[FCTIndex].NumDie > 1) {
				/*
				 * Call Die erase
				 */
				DieErase(QspiPtr, WriteBfrPtr);
			}
		}

		return;
	}

	/*
	 * If the erase size is less than the total size of the flash, use
	 * sector erase command
	 */

	/*
	 * Calculate no. of sectors to erase based on byte count
	 */
//	NumSect = ByteCount/(Flash_Config_Table[FCTIndex].PageSize) + 1;
	NumSect = ByteCount/(Flash_Config_Table[FCTIndex].SectSize) + 1;

	/*
	 * If ByteCount to k sectors,
	 * but the address range spans from N to N+k+1 sectors, then
	 * increment no. of sectors to be erased
	 */

	if( ((Address + ByteCount) & Flash_Config_Table[FCTIndex].SectMask) ==
			((Address + (NumSect * Flash_Config_Table[FCTIndex].SectSize)) &
					Flash_Config_Table[FCTIndex].SectMask) ) {
		NumSect++;
	}

	for (Sector = 0; Sector < (int)NumSect; Sector++) {

		/*
		 * Translate address based on type of connection
		 * If stacked assert the slave select based on address
		 */
		RealAddr = GetRealAddr(QspiPtr, Address);

		/*
		 * Initial bank selection
		 */
		if((BankInitFlag) &&
				(Flash_Config_Table[FCTIndex].FlashDeviceSize > SIXTEENMB)) {
			/*
			 * Reset initial bank select flag
			 */
			BankInitFlag = 0;
			/*
			 * Calculate initial bank
			 */
			BankSel = RealAddr/SIXTEENMB;
			/*
			 * Select bank
			 */
			SendBankSelect(QspiPtr, WriteBfrPtr, BankSel);
		}
		/*
		 * Check bank and send bank select if new bank
		 */
		if((BankSel != RealAddr/SIXTEENMB) &&
				(Flash_Config_Table[FCTIndex].FlashDeviceSize > SIXTEENMB)) {
			/*
			 * Calculate initial bank
			 */
			BankSel = RealAddr/SIXTEENMB;
			/*
			 * Select bank
			 */
			SendBankSelect(QspiPtr, WriteBfrPtr, BankSel);
		}

		/*
		 * Send the write enable command to the SEEPOM so that it can be
		 * written to, this needs to be sent as a separate transfer
		 * before the write
		 */
		XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,
				  sizeof(WriteEnableCmd));

		/*
		 * Setup the write command with the specified address and data
		 * for the Flash
		 */
		/*
		 * This ensures 3B address is sent to flash even with address
		 * greater than 128Mb.
		 */
		WriteBfrPtr[COMMAND_OFFSET]   = SEC_ERASE_CMD;
		WriteBfrPtr[ADDRESS_1_OFFSET] = (u8)(RealAddr >> 16);
		WriteBfrPtr[ADDRESS_2_OFFSET] = (u8)(RealAddr >> 8);
		WriteBfrPtr[ADDRESS_3_OFFSET] = (u8)(RealAddr & 0xFF);

		/*
		 * Send the sector erase command and address; no receive buffer
		 * is specified since there is nothing to receive
		 */
		XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, NULL,
					SEC_ERASE_SIZE);

		if((Flash_Config_Table[FCTIndex].NumDie > 1) &&
				(FlashMake == MICRON_ID_BYTE0)) {
			XQspiPs_PolledTransfer(QspiPtr, ReadFlagSRCmd, FlagStatus,
						sizeof(ReadFlagSRCmd));
		}
		/*
		 * Wait for the sector erase command to the Flash to be completed
		 */
		while (1) {
			/*
			 * Poll the status register of the device to determine
			 * when it completes, by sending a read status command
			 * and receiving the status byte
			 */
			XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd,
						FlashStatus,
						sizeof(ReadStatusCmd));

			/*
			 * If the status indicates the write is done, then stop
			 * waiting, if a value of 0xFF in the status byte is
			 * read from the device and this loop never exits, the
			 * device slave select is possibly incorrect such that
			 * the device status is not being read
			 */
			if ((FlashStatus[1] & 0x01) == 0) {
				break;
			}
		}

		if((Flash_Config_Table[FCTIndex].NumDie > 1) &&
				(FlashMake == MICRON_ID_BYTE0)) {
			XQspiPs_PolledTransfer(QspiPtr, ReadFlagSRCmd, FlagStatus,
						sizeof(ReadFlagSRCmd));
		}

		Address += Flash_Config_Table[FCTIndex].SectSize;
//		Address += Flash_Config_Table[FCTIndex].PageSize;

	}
}


/*****************************************************************************/
/**
*
* This function reads serial Flash ID connected to the SPI interface.
* It then deduces the make and size of the flash and obtains the
* connection mode to point to corresponding parameters in the flash
* configuration table. The flash driver will function based on this and
* it presently supports Micron and Spansion - 128, 256 and 512Mbit and
* Winbond 128Mbit
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	Pointer to the write buffer (which is to be transmitted)
* @param	Pointer to the read buffer to which valid received data should be
* 			written
*
* @return	XST_SUCCESS if read id, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int StorageNIOSFlash::FlashReadID(XQspiPs *QspiPtr, u8 *WriteBfrPtr, u8 *ReadBfrPtr)
{
	int Status;
	int StartIndex;
	/*
	 * Read ID in Auto mode.
	 */
	WriteBfrPtr[COMMAND_OFFSET]   = READ_ID;
	WriteBfrPtr[ADDRESS_1_OFFSET] = 0x23;		/* 3 dummy bytes */
	WriteBfrPtr[ADDRESS_2_OFFSET] = 0x08;
	WriteBfrPtr[ADDRESS_3_OFFSET] = 0x09;

	Status = XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, ReadBfrPtr,
				RD_ID_SIZE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Deduce flash make
	 */
	if(ReadBfrPtr[1] == MICRON_ID_BYTE0) {
		FlashMake = MICRON_ID_BYTE0;
		StartIndex = MICRON_INDEX_START;
	}else if(ReadBfrPtr[1] == SPANSION_ID_BYTE0) {
		FlashMake = SPANSION_ID_BYTE0;
		StartIndex = SPANSION_INDEX_START;
	}else if(ReadBfrPtr[1] == WINBOND_ID_BYTE0) {
		FlashMake = WINBOND_ID_BYTE0;
		StartIndex = WINBOND_INDEX_START;
	} else if(ReadBfrPtr[1] == MACRONIX_ID_BYTE0) {
		FlashMake = MACRONIX_ID_BYTE0;
		StartIndex = MACRONIX_INDEX_START;
	} else if(ReadBfrPtr[0] == ISSI_ID_BYTE0) {
		FlashMake = ISSI_ID_BYTE0;
		StartIndex = ISSI_INDEX_START;
	}


	/*
	 * If valid flash ID, then check connection mode & size and
	 * assign corresponding index in the Flash configuration table
	 */
	if(((FlashMake == MICRON_ID_BYTE0) || (FlashMake == SPANSION_ID_BYTE0)||
			(FlashMake == WINBOND_ID_BYTE0)) &&
			(ReadBfrPtr[3] == MICRON_ID_BYTE2_128)) {

		switch(QspiPtr->Config.ConnectionMode)
		{
			case XQSPIPS_CONNECTION_MODE_SINGLE:
				FCTIndex = FLASH_CFG_TBL_SINGLE_128_SP + StartIndex;
				break;
			case XQSPIPS_CONNECTION_MODE_PARALLEL:
				FCTIndex = FLASH_CFG_TBL_PARALLEL_128_SP + StartIndex;
				break;
			case XQSPIPS_CONNECTION_MODE_STACKED:
				FCTIndex = FLASH_CFG_TBL_STACKED_128_SP + StartIndex;
				break;
			default:
				FCTIndex = 0;
				break;
		}
	}
	/* 256 and 512Mbit supported only for Micron and Spansion, not Winbond */
	if(((FlashMake == MICRON_ID_BYTE0) || (FlashMake == SPANSION_ID_BYTE0)
			|| (FlashMake == MACRONIX_ID_BYTE0)) &&
			(ReadBfrPtr[3] == MICRON_ID_BYTE2_256)) {

		switch(QspiPtr->Config.ConnectionMode)
		{
			case XQSPIPS_CONNECTION_MODE_SINGLE:
				FCTIndex = FLASH_CFG_TBL_SINGLE_256_SP + StartIndex;
				break;
			case XQSPIPS_CONNECTION_MODE_PARALLEL:
				FCTIndex = FLASH_CFG_TBL_PARALLEL_256_SP + StartIndex;
				break;
			case XQSPIPS_CONNECTION_MODE_STACKED:
				FCTIndex = FLASH_CFG_TBL_STACKED_256_SP + StartIndex;
				break;
			default:
				FCTIndex = 0;
				break;
		}
	}
	if((FlashMake == ISSI_ID_BYTE0) &&
			(ReadBfrPtr[2] == MICRON_ID_BYTE2_256)) {
		switch(QspiPtr->Config.ConnectionMode)
		{
			case XQSPIPS_CONNECTION_MODE_SINGLE:
				FCTIndex = FLASH_CFG_TBL_SINGLE_256_ISSI;
				break;
			case XQSPIPS_CONNECTION_MODE_PARALLEL:
				FCTIndex = FLASH_CFG_TBL_PARALLEL_256_ISSI;
				break;
			case XQSPIPS_CONNECTION_MODE_STACKED:
				FCTIndex = FLASH_CFG_TBL_STACKED_256_ISSI;
				break;
			default:
				FCTIndex = 0;
				break;
		}
	}
	if ((((FlashMake == MICRON_ID_BYTE0) || (FlashMake == SPANSION_ID_BYTE0)) &&
			(ReadBfrPtr[3] == MICRON_ID_BYTE2_512)) || ((FlashMake ==
			MACRONIX_ID_BYTE0) && (ReadBfrPtr[3] == MACRONIX_ID_BYTE2_512))) {

		switch(QspiPtr->Config.ConnectionMode)
		{
			case XQSPIPS_CONNECTION_MODE_SINGLE:
				FCTIndex = FLASH_CFG_TBL_SINGLE_512_SP + StartIndex;
				break;
			case XQSPIPS_CONNECTION_MODE_PARALLEL:
				FCTIndex = FLASH_CFG_TBL_PARALLEL_512_SP + StartIndex;
				break;
			case XQSPIPS_CONNECTION_MODE_STACKED:
				FCTIndex = FLASH_CFG_TBL_STACKED_512_SP + StartIndex;
				break;
			default:
				FCTIndex = 0;
				break;
		}
	}
	/*
	 * 1Gbit Single connection supported for Spansion.
	 * The ConnectionMode will indicate stacked as this part has 2 SS
	 * The device ID will indicate 512Mbit.
	 * This configuration is handled as the above 512Mbit stacked configuration
	 */
	/* 1Gbit single, parallel and stacked supported for Micron */
	if(((FlashMake == MICRON_ID_BYTE0) &&
		(ReadBfrPtr[3] == MICRON_ID_BYTE2_1G)) ||
		((FlashMake == MACRONIX_ID_BYTE0) &&
		 (ReadBfrPtr[3] == MACRONIX_ID_BYTE2_1G))) {

		switch(QspiPtr->Config.ConnectionMode)
		{
			case XQSPIPS_CONNECTION_MODE_SINGLE:
				FCTIndex = FLASH_CFG_TBL_SINGLE_1GB_MC;
				break;
			case XQSPIPS_CONNECTION_MODE_PARALLEL:
				FCTIndex = FLASH_CFG_TBL_PARALLEL_1GB_MC;
				break;
			case XQSPIPS_CONNECTION_MODE_STACKED:
				FCTIndex = FLASH_CFG_TBL_STACKED_1GB_MC;
				break;
			default:
				FCTIndex = 0;
				break;
		}
	}

	xil_printf("FlashID=0x%x 0x%x 0x%x\n\r", ReadBfrPtr[1], ReadBfrPtr[2],
		   ReadBfrPtr[3]);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function performs an I/O read.
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	Address contains the address of the first sector which needs to
*			be erased.
* @param	ByteCount contains the total size to be erased.
* @param	Command is the command used to read data from the flash. Supports
* 			normal, fast, dual and quad read commands.
* @param	Pointer to the write buffer which contains data to be transmitted
* @param	Pointer to the read buffer to which valid received data should be
* 			written
*
* @return	none.
*
* @note		None.
*
******************************************************************************/
void StorageNIOSFlash::FlashRead(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command,
				u8 *WriteBfrPtr, u8 *ReadBfrPtr)
{
	u32 RealAddr;
	u32 RealByteCnt;
	u32 BankSel;
	u32 BufferIndex;
	u32 TotalByteCnt;
	u8 ShiftSize;

	/*
	 * Retain the actual byte count
	 */
	TotalByteCnt = ByteCount;

	while(((signed long)(ByteCount)) > 0) {

		/*
		 * Translate address based on type of connection
		 * If stacked assert the slave select based on address
		 */
		RealAddr = GetRealAddr(QspiPtr, Address);

		/*
		 * Select bank
		 */
		if(Flash_Config_Table[FCTIndex].FlashDeviceSize > SIXTEENMB) {
			BankSel = RealAddr/SIXTEENMB;
			SendBankSelect(QspiPtr, WriteBfrPtr, BankSel);
		}

		/*
		 * If data to be read spans beyond the current bank, then
		 * calculate RealByteCnt in current bank. Else
		 * RealByteCnt is the same as ByteCount
		 */
		if((Address & BANKMASK) != ((Address+ByteCount) & BANKMASK)) {
			RealByteCnt = (Address & BANKMASK) + SIXTEENMB - Address;
		}else {
			RealByteCnt = ByteCount;
		}


		/*
		 * Setup the write command with the specified address and data for the
		 * Flash
		 */
		WriteBfrPtr[COMMAND_OFFSET]   = Command;
		WriteBfrPtr[ADDRESS_1_OFFSET] = (u8)((RealAddr & 0xFF0000) >> 16);
		WriteBfrPtr[ADDRESS_2_OFFSET] = (u8)((RealAddr & 0xFF00) >> 8);
		WriteBfrPtr[ADDRESS_3_OFFSET] = (u8)(RealAddr & 0xFF);

		if ((Command == FAST_READ_CMD) || (Command == DUAL_READ_CMD) ||
		    (Command == QUAD_READ_CMD)) {
			RealByteCnt += DUMMY_SIZE;
		}
		/*
		 * Send the read command to the Flash to read the specified number
		 * of bytes from the Flash, send the read command and address and
		 * receive the specified number of bytes of data in the data buffer
		 */
		XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr,
				&(ReadBfrPtr[TotalByteCnt - ByteCount]),
				RealByteCnt + OVERHEAD_SIZE);

		/*
		 * To discard the first 5 dummy bytes, shift the data in read buffer
		 */
		if((Command == FAST_READ_CMD) || (Command == DUAL_READ_CMD) ||
			    (Command == QUAD_READ_CMD)){
			ShiftSize = OVERHEAD_SIZE + DUMMY_SIZE;
		}else{
			ShiftSize =  OVERHEAD_SIZE;
		}

		for(BufferIndex = (TotalByteCnt - ByteCount);
				BufferIndex < (TotalByteCnt - ByteCount) + RealByteCnt;
				BufferIndex++) {
			ReadBfrPtr[BufferIndex] = ReadBfrPtr[BufferIndex + ShiftSize];
		}

		/*
		 * Increase address to next bank
		 */
		Address = (Address & BANKMASK) + SIXTEENMB;
		/*
		 * Decrease byte count by bytes already read.
		 */
		if ((Command == FAST_READ_CMD) || (Command == DUAL_READ_CMD) ||
		    (Command == QUAD_READ_CMD)) {
			ByteCount = ByteCount - (RealByteCnt - DUMMY_SIZE);
		}else {
			ByteCount = ByteCount - RealByteCnt;
		}

	}

}

/*****************************************************************************/
/**
*
* This functions selects the current bank
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	Pointer to the write buffer which contains data to be transmitted
* @param	BankSel is the bank to be selected in the flash device(s).
*
* @return	XST_SUCCESS if bank selected, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int StorageNIOSFlash::SendBankSelect(XQspiPs *QspiPtr, u8 *WriteBfrPtr, u32 BankSel)
{
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };

	/*
	 * Bank select commands for Micron and Spansion are different
	 */
	if(FlashMake == MICRON_ID_BYTE0) {
		/*
		 * For Micron command WREN should be sent first
		 * except for some specific feature set
		 */
		XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,
					sizeof(WriteEnableCmd));

		WriteBfrPtr[COMMAND_OFFSET]   = EXTADD_REG_WR;
		WriteBfrPtr[ADDRESS_1_OFFSET] = BankSel;

		/*
		 * Send the Extended address register write command
		 * written, no receive buffer required
		 */
		XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, NULL,
				BANK_SEL_SIZE);

	}
	if(FlashMake == SPANSION_ID_BYTE0) {
		WriteBfrPtr[COMMAND_OFFSET]   = BANK_REG_WR;
		WriteBfrPtr[ADDRESS_1_OFFSET] = BankSel;

		/*
		 * Send the Extended address register write command
		 * written, no receive buffer required
		 */
		XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, NULL,
				BANK_SEL_SIZE);
	}

	/* Winbond can be added here */

	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* This functions performs a bulk erase operation when the
* flash device has a single die. Works for both Spansion and Micron
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	WritBfrPtr is the pointer to command+address to be sent
*
* @return	None
*
* @note		None.
*
******************************************************************************/
void StorageNIOSFlash::BulkErase(XQspiPs *QspiPtr, u8 *WriteBfrPtr)
{
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
	u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  /* Must send 2 bytes */
	u8 FlashStatus[2];

	/*
	 * Send the write enable command to the Flash so that it can be
	 * written to, this needs to be sent as a separate transfer
	 * before the erase
	 */
	XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,
			  sizeof(WriteEnableCmd));

	/*
	 * Setup the bulk erase command
	 */
	WriteBfrPtr[COMMAND_OFFSET]   = BULK_ERASE_CMD;

	/*
	 * Send the bulk erase command; no receive buffer is specified
	 * since there is nothing to receive
	 */
	XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, NULL,
				BULK_ERASE_SIZE);

	/*
	 * Wait for the erase command to the Flash to be completed
	 */
	while (1) {
		/*
		 * Poll the status register of the device to determine
		 * when it completes, by sending a read status command
		 * and receiving the status byte
		 */
		XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd,
					FlashStatus,
					sizeof(ReadStatusCmd));

		/*
		 * If the status indicates the write is done, then stop
		 * waiting; if a value of 0xFF in the status byte is
		 * read from the device and this loop never exits, the
		 * device slave select is possibly incorrect such that
		 * the device status is not being read
		 */
		if ((FlashStatus[1] & 0x01) == 0) {
			break;
		}
	}
}

/*****************************************************************************/
/**
*
* This functions performs a die erase operation on all the die in
* the flash device. This function uses the die erase command for
* Micron 512Mbit and 1Gbit
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	WritBfrPtr is the pointer to command+address to be sent
*
* @return	None
*
* @note		None.
*
******************************************************************************/
void StorageNIOSFlash::DieErase(XQspiPs *QspiPtr, u8 *WriteBfrPtr)
{
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
	u8 DieCnt;
	u8 ReadFlagSRCmd[] = { READ_FLAG_STATUS_CMD, 0 };
	u8 FlagStatus[2];

	for(DieCnt = 0; DieCnt < Flash_Config_Table[FCTIndex].NumDie; DieCnt++) {
		/*
		 * Select bank - the lower of the 2 banks in each die
		 * This is specific to Micron flash
		 */
		SendBankSelect(QspiPtr, WriteBfrPtr, DieCnt*2);

		/*
		 * Send the write enable command to the SEEPOM so that it can be
		 * written to, this needs to be sent as a separate transfer
		 * before the write
		 */
		XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,
				  sizeof(WriteEnableCmd));

		/*
		 * Setup the write command with the specified address and data
		 * for the Flash
		 */
		/*
		 * This ensures 3B address is sent to flash even with address
		 * greater than 128Mb.
		 * The address is the start address of die - MSB bits will be
		 * derived from bank select by the flash
		 */
		WriteBfrPtr[COMMAND_OFFSET]   = DIE_ERASE_CMD;
		WriteBfrPtr[ADDRESS_1_OFFSET] = 0x00;
		WriteBfrPtr[ADDRESS_2_OFFSET] = 0x00;
		WriteBfrPtr[ADDRESS_3_OFFSET] = 0x00;

		/*
		 * Send the sector erase command and address; no receive buffer
		 * is specified since there is nothing to receive
		 */
		XQspiPs_PolledTransfer(QspiPtr, WriteBfrPtr, NULL,
				DIE_ERASE_SIZE);

		/*
		 * Wait for the sector erase command to the Flash to be completed
		 */
		while (1) {
			/*
			 * Poll the status register of the device to determine
			 * when it completes, by sending a read status command
			 * and receiving the status byte
			 */
			XQspiPs_PolledTransfer(QspiPtr, ReadFlagSRCmd, FlagStatus,
					sizeof(ReadFlagSRCmd));

			/*
			 * If the status indicates the write is done, then stop
			 * waiting, if a value of 0xFF in the status byte is
			 * read from the device and this loop never exits, the
			 * device slave select is possibly incorrect such that
			 * the device status is not being read
			 */
			if ((FlagStatus[1] & 0x80) == 0x80) {
				break;
			}
		}

	}
}

/*****************************************************************************/
/**
*
* This functions translates the address based on the type of interconnection.
* In case of stacked, this function asserts the corresponding slave select.
*
* @param	QspiPtr is a pointer to the QSPI driver component to use.
* @param	Address which is to be accessed (for erase, write or read)
*
* @return	RealAddr is the translated address - for single it is unchanged;
* 			for stacked, the lower flash size is subtracted;
* 			for parallel the address is divided by 2.
*
* @note		None.
*
******************************************************************************/
u32 StorageNIOSFlash::GetRealAddr(XQspiPs *QspiPtr, u32 Address)
{
	u32 LqspiCr;
	u32 RealAddr;

	switch(QspiPtr->Config.ConnectionMode) {
	case XQSPIPS_CONNECTION_MODE_SINGLE:
		RealAddr = Address;
		break;
	case XQSPIPS_CONNECTION_MODE_STACKED:
		/*
		 * Get the current LQSPI Config reg value
		 */
		LqspiCr = XQspiPs_GetLqspiConfigReg(QspiPtr);

		/* Select lower or upper Flash based on sector address */
		if(Address & Flash_Config_Table[FCTIndex].FlashDeviceSize) {
			/*
			 * Set selection to U_PAGE
			 */
			XQspiPs_SetLqspiConfigReg(QspiPtr,
					LqspiCr | XQSPIPS_LQSPI_CR_U_PAGE_MASK);

			/*
			 * Subtract first flash size when accessing second flash
			 */
			RealAddr = Address &
					(~Flash_Config_Table[FCTIndex].FlashDeviceSize);

		}else{

			/*
			 * Set selection to L_PAGE
			 */
			XQspiPs_SetLqspiConfigReg(QspiPtr,
					LqspiCr & (~XQSPIPS_LQSPI_CR_U_PAGE_MASK));

			RealAddr = Address;

		}

		/*
		 * Assert the Flash chip select.
		 */
		XQspiPs_SetSlaveSelect(QspiPtr);
		break;
	case XQSPIPS_CONNECTION_MODE_PARALLEL:
		/*
		 * The effective address in each flash is the actual
		 * address / 2
		 */
		RealAddr = Address / 2;
		break;
	default:
		/* RealAddr wont be assigned in this case; */
	break;

	}

	return(RealAddr);

}

int StorageNIOSFlash::QspiFlashInit(XQspiPs *QspiInstancePtr, u16 QspiDeviceId)
{
	int Status;
	XQspiPs_Config *QspiConfig;
	/*
	 * Initialize the QSPI driver so that it's ready to use
	 */
	QspiConfig = XQspiPs_LookupConfig(QspiDeviceId);
	if (NULL == QspiConfig) {
		return XST_FAILURE;
	}

	Status = XQspiPs_CfgInitialize(QspiInstancePtr, QspiConfig,
					QspiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build
	 */
	Status = XQspiPs_SelfTest(QspiInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}



	/*
	 * Set the pre-scaler for QSPI clock
	 */
	XQspiPs_SetClkPrescaler(QspiInstancePtr, XQSPIPS_CLK_PRESCALE_8);

	/*
	 * Set Manual Start and Manual Chip select options and drive the
	 * HOLD_B high.
	 */
//	XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_FORCE_SSELECT_OPTION |
//					     XQSPIPS_MANUAL_START_OPTION |
//					     XQSPIPS_HOLD_B_DRIVE_OPTION);
	XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_FORCE_SSELECT_OPTION |
					     XQSPIPS_HOLD_B_DRIVE_OPTION);
	if(QspiConfig->ConnectionMode == XQSPIPS_CONNECTION_MODE_STACKED) {
		/*
		 * Enable two flash memories, Shared bus (NOT separate bus),
		 * L_PAGE selected by default
		 */
		XQspiPs_SetLqspiConfigReg(QspiInstancePtr, DUAL_STACK_CONFIG_WRITE);
	}

	if(QspiConfig->ConnectionMode == XQSPIPS_CONNECTION_MODE_PARALLEL) {
		/*
		 * Enable two flash memories on separate buses
		 */
		XQspiPs_SetLqspiConfigReg(QspiInstancePtr, DUAL_QSPI_CONFIG_WRITE);
	}


	/*
	 * Assert the Flash chip select.
	 */
	XQspiPs_SetSlaveSelect(QspiInstancePtr);

	/*
	 * Read flash ID and obtain all flash related information
	 * It is important to call the read id function before
	 * performing proceeding to any operation, including
	 * preparing the WriteBuffer
	 */
	FlashReadID(QspiInstancePtr, WriteBuffer, ReadBuffer);

	return XST_SUCCESS;
}



/** \name Method saves to file any data with attached signatures of the beginning (4 bytes), ending (4 bytes) and CRC sum (2 bytes)
* \param 'buf' - pointer to a data to be written
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
* \note Wrtting can be performed only at the beginning and in the range of a single block of flash memory. Before writting all memory block is erased,
* previous content isn't secured.
*/
bool StorageNIOSFlash::save (const void* buf, int size)
{   
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_save_0");
        return false;
    }

    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_save_1");
        return false;
    }
    
    INT16U ecrc=0;
	// Check if size of data isn't greater then file size
    if (size+sizeof(BEG_SIGNATURE)+sizeof(END_SIGNATURE)+sizeof(ecrc) > files[_fIdent].fileSize)
    {
        Log.errorPrintf("StorageNIOSFlash_save_2");
        return false;
    }
    
	// Check if file can be written only by an 'append' function
    if (files[_fIdent].appendOnly)
    {
        Log.errorPrintf("StorageNIOSFlash_save_3");
        return false;
    }
          
	// Calculation of the offset of the block relative to the beginning of the flash
    int blockBeg = FILES_OFFSET + files[_fIdent].fileOffset;
       
	// Waiting for an access to flash - function 'lockFlash' prints out optional error
    if (!_flashSem().lock (false, FLASH_SEM_WAIT_MS))
        return false;

    int size_sum = sizeof(BEG_SIGNATURE) + size + sizeof(END_SIGNATURE) + sizeof(ecrc);
    int size_factor =  size_sum/Flash_Config_Table[FCTIndex].PageSize;
    int size_modul = size_sum%Flash_Config_Table[FCTIndex].PageSize;

//    FlashErase(&QspiInstance, blockBeg, BLOCK_SIZE, WriteBuffer);
    FlashErase(&QspiInstance, blockBeg, size_sum, WriteBuffer);

    u8 beg_sig[4];
    memcpy(beg_sig, (u8*)&BEG_SIGNATURE, 4);
    copyArray(beg_sig,write_buf, sizeof(BEG_SIGNATURE), 0);

    u8 dat[size];
    memcpy(dat, (const void*)buf,size);
    copyArray(dat,write_buf, (unsigned int)size, sizeof(BEG_SIGNATURE));

    u8 end_sig[4];
    memcpy(end_sig, (u8*)&END_SIGNATURE, 4);
    copyArray(end_sig,write_buf, sizeof(END_SIGNATURE), sizeof(BEG_SIGNATURE)+size);

	// Calculation of an CRC and appending the result to the end
    ecrc = Crc::computeBin(static_cast<const unsigned char*>(buf), size);

    u8 crc_sig[2];
    memcpy(crc_sig, (u8*)&ecrc, 2);
    copyArray(crc_sig,write_buf, 2, sizeof(BEG_SIGNATURE)+size+sizeof(END_SIGNATURE));

    for(int page = 0; page <= size_factor; page++)
    {
    	if(page <= size_factor - 1)
    	{
    		for(unsigned int j=0; j<Flash_Config_Table[FCTIndex].PageSize; j++)
    			WriteBuffer[OVERHEAD_SIZE + j] = write_buf[page*Flash_Config_Table[FCTIndex].PageSize + j];
    		FlashWrite(&QspiInstance, blockBeg + page*Flash_Config_Table[FCTIndex].PageSize,Flash_Config_Table[FCTIndex].PageSize,WRITE_CMD,  WriteBuffer);
    		if(blockBeg + page*Flash_Config_Table[FCTIndex].PageSize > FILES_OFFSET + 24*BLOCK_SIZE)
    			Log.errorPrintf("StorageNIOSFlash_load_10");
    	}
    	else
    	{
    		for(int j=0; j<size_modul; j++)
    			WriteBuffer[OVERHEAD_SIZE + j] = write_buf[page*Flash_Config_Table[FCTIndex].PageSize + j];
    		FlashWrite(&QspiInstance,blockBeg + page*Flash_Config_Table[FCTIndex].PageSize ,size_modul,WRITE_CMD,  WriteBuffer);
    		if(blockBeg + page*Flash_Config_Table[FCTIndex].PageSize > FILES_OFFSET + 24*BLOCK_SIZE)
    			Log.errorPrintf("StorageNIOSFlash_load_11");
    	}
    }

    // Unlocking of an access - function 'unlockFlash' prints out optional error
    if (!_flashSem().unlock ())
        return false;           
        
    return true;   
}

/** \name Method reads from file any data with validation (signatures of the beginning, ending and CRC sum)
* \param 'buf' - pointer to the buffer to which data will be read
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageNIOSFlash::load (void* buf, int size)
{
    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_load_1");
        return false;
    }

	// Calculation of the offset of the block relative to the beginning of the flash
    int blockBeg = FILES_OFFSET + files[_fIdent].fileOffset;
    
    if (!_singleThread)
		// Waiting for access to the flash - function 'lockFlash' prints out optional error
        if (!_flashSem().lock (false, FLASH_SEM_WAIT_MS))
            return false;

    // Reading of a signatures
    unsigned int begSig = 0;
    unsigned int endSig = 0;

    //int size_factor =  size/Flash_Config_Table[FCTIndex].PageSize;

    u8 beg_sig[4];
    u8 end_sig[4];
    FlashRead(&QspiInstance, blockBeg, sizeof(begSig), QUAD_READ_CMD, WriteBuffer, beg_sig);
    begSig = buffToUint(beg_sig);

    FlashRead(&QspiInstance, blockBeg + sizeof(begSig) + size, sizeof(endSig), QUAD_READ_CMD, WriteBuffer, end_sig);
    endSig = buffToUint(end_sig);

    if (begSig != BEG_SIGNATURE || endSig != END_SIGNATURE)
    {
        // Signatures error
        if (!_singleThread)
            _flashSem().unlock();
        Log.errorPrintf("StorageNIOSFlash_load_3");
        return false;
    }

    // Reading of a CRC
    INT16U readCrc=0;
    u8 crc_sig[2];
    FlashRead(&QspiInstance, blockBeg + sizeof(begSig) + size + sizeof(endSig), sizeof(readCrc), QUAD_READ_CMD, WriteBuffer, crc_sig);
    readCrc = static_cast<INT16U>(crc_sig[1] << 8 | crc_sig[0]);

	// Reading of a content (without result validation - function always returns 0)
    FlashRead(&QspiInstance, blockBeg + sizeof(begSig), size + sizeof(readCrc), QUAD_READ_CMD, WriteBuffer, ReadBuffer);
    memcpy(buf, (const void*)ReadBuffer,size);

    // Calculation of a CRC and comparison with readed one
    INT16U computedCrc = Crc::computeBin(static_cast<const unsigned char*>(buf), size);
    if (readCrc != computedCrc)
    {
        // CRC error
        if (!_singleThread)
            _flashSem().unlock();
        Log.errorPrintf("StorageNIOSFlash_load_4");
        return false;
    }
            
    // Unlocking of an access
    if (!_singleThread)
        if (!_flashSem().unlock ())
            return false;           
        
    return true;   
}

/** \name Method reads any data from a file. Method do not validates the read data. Method used only for files written by an 'append' function.
* \param 'buf' - pointer to the buffer to which data will be read
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageNIOSFlash::read (void* buf, int size, int offset)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_read_0");
        return false;
    }

    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_read_1");
        return false;
    }

	// Check if the file can be write only by an 'append' finction
    if (!files[_fIdent].appendOnly)
    {
        Log.errorPrintf("StorageNIOSFlash_read_2");
        return false;
    }

    // Calculation of the offset of the block relative to the beginning of the flash
    int blockBeg = FILES_OFFSET + files[_fIdent].fileOffset;
    
	// Waiting for an access to flash - function 'lockFlash' prints out optional error
    if (!_flashSem().lock (false, FLASH_SEM_WAIT_MS))
        return false;

    FlashRead(&QspiInstance, blockBeg+offset, size, QUAD_READ_CMD, WriteBuffer, ReadBuffer);
    memcpy(buf, (const void*)ReadBuffer,size);
            
    // Unlocking of an access
    if (!_flashSem().unlock ())
        return false;           
        
    return true;   
}

/** \name Method reduces file size to 0. Method may be use only for a files written by an 'append' function.
* \param 'fastClear' - when 'true' only used blocks are cleared, when 'false' entire file is cleared.
* \return 'true' on success, 'fasle' otherwise
*/
bool StorageNIOSFlash::clear (bool fastClear)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_clear_0");
        return false;
    }

    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_clear_1");
        return false;
    }
    
    // Check if the file can be write only by an 'append' finction
    if (!files[_fIdent].appendOnly)
    {
        Log.errorPrintf("StorageNIOSFlash_clear_2");
        return false;
    }
          
    // Calculation of the offset of the block relative to the beginning of the flash
    int blockBeg = FILES_OFFSET + files[_fIdent].fileOffset;

    // Number of blocks to clear
    int blocks;
    if (fastClear)
    {
        // Erasing of a used blocks only
        blocks = ((files[_fIdent].appendNextFilePos-1) / BLOCK_SIZE) + 1;
//        blocks = ((files[_fIdent].appendNextFilePos-1) / PAGESIZE) + 1;
    }
    else
    {
        // Erasing of entire file
        blocks = ((files[_fIdent].fileSize-1) / BLOCK_SIZE) + 1;
//        blocks = ((files[_fIdent].fileSize-1) / PAGESIZE) + 1;
    }

	// Waiting for an access to flash - function 'lockFlash' prints out optional error
    if (!_flashSem().lock (false, 100))
        return false;

    for (int i=0; i<blocks; i++)
    {
    	FlashErase(&QspiInstance, blockBeg + i*BLOCK_SIZE, BLOCK_SIZE, WriteBuffer);
    }
//    FlashErase(&QspiInstance, blockBeg, files[_fIdent].fileSize, WriteBuffer);

	// Reseting of a last position pointer in a buffer and in a file
    files[_fIdent].cbSystemPos = 0;
    files[_fIdent].cbUserPos = 0;
    files[_fIdent].appendNextFilePos = 0;
            
	// Unlocking of an access - function '_flashSem().unlock' prints optioanl error
    if (!_flashSem().unlock ())
        return false;           
        
	// Adding an empty line at the beginning
    if (!append (CRLF))
    {
        Log.errorPrintf("StorageNIOSFlash_clear_5");
        return false;
    }
    
    return true;   
}

/** \name Method returns size of a file in bytes or '-1' if an error occured. Method works only for files written by an 'append' function.
*/
int StorageNIOSFlash::getSize ()
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_getSize_0");
        return -1;
    }

    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_getSize_1");
        return -1;
    }
    
    if (!files[_fIdent].appendOnly)
    {
        Log.errorPrintf("StorageNIOSFlash_getSize_2");
        return -1;
    }
    
    // Semaphore is not needed here
    return files[_fIdent].appendNextFilePos;
}

/** \name Method returns size of free space in file (in bytes) or '-1' if an error occured.
* Method works only for files written by an 'append' function.
*/
int StorageNIOSFlash::getFree ()
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_getFree_0");
        return -1;
    }

    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_getFree_1");
        return -1;
    }
    
    if (!files[_fIdent].appendOnly)
    {
        Log.errorPrintf("StorageNIOSFlash_getFree_2");
        return -1;
    }

    // Semaphore is not needed here
    return files[_fIdent].fileSize - files[_fIdent].appendNextFilePos;
}

/** \name Method handles uC/OS-II system tasks of writing the line to the flash
*/
void StorageNIOSFlash::task(void* pdata)
{
    INT8U err = 0;
    char buf[LINESIZE];

    if (!_async)
    {
        // Object has not enabled handling of an asynchronous
        Log.errorPrint("StorageNIOSFlash_task_0", true);
        OSTaskSuspend (OS_PRIO_SELF);
    }
    if(!_initFlash)
    {
    	int Status = QspiFlashInit(&QspiInstance, QSPI_DEVICE_ID);
    	if(Status != XST_SUCCESS)
    	{
    		Log.errorPrint("StorageNIOSFlash_task_0", true);
    	}
    	else
    		_initFlash = true;
    }
    while(true)
    {
		// Waiting for a signal that something has been added to the cyclic buffer
		// 'fi' specifies the file to be handled
        FileItem* fi = static_cast <FileItem*>(OSMboxPend (_mboxSig, 0, &err));
        if (err != OS_NO_ERR || fi == static_cast<void*>(0))
        {
            Log.errorPrint("StorageNIOSFlash_task_1 [", err, "]", true);
            // Wait 10s to not clog up the error log
            OSTimeDlyHMSM (0, 0, ERRWAIT, 0);
            continue;
        }

		// Waiting for an access to structures (rather there should be no blocking)
		// Function 'lockSem' prints out optional error
        if (!_bufSem().lock ())
			// If the semaphore cannot be locked then method waits for the next signal
            continue;

		// Check if there is any line waiting in the cyclic buffer
        if (fi->cbUserPos == fi->cbSystemPos)
        {
            _bufSem().unlock ();
            continue;
        }

		// New position in buffer for a writing task
        int newSystemPos = (fi->cbSystemPos + 1) % BUF_LINES;
		// Copying of a line to the local buffer to quickly release the semaphore
		// Line in a cyclic buffer has a guaranteed '/0' character at the end
        MEMCCPY (buf, fi->appendCycBuf[newSystemPos], 0, LINESIZE);
		// Updating of the new position
        fi->cbSystemPos = newSystemPos;

		// Check if there is any line waiting for a write
        if (fi->cbUserPos != fi->cbSystemPos)
        {
			// Awakening of the writing task for the next loop iteration
			// 'fi' parameters specifies currently handled file
            err = OSMboxPost (_mboxSig, fi);
            if (err != OS_NO_ERR && err != OS_MBOX_FULL)
            {
                Log.errorPrint("StorageNIOSFlash_task_2 [", err, "]", true);
            }
        }

		// Unlocking of an access to the buffer - prints out optional error
        _bufSem().unlock ();

        // Writing to the flash
        append2 (fi, buf, strlen(buf));
    }

}

/** \name Method appends a text line to the end of file.
* \param 'lineToWrite' - pointer to a string to be appended
* \return 'true' on success, 'false' otherwise
* \note There cannot be '0xFF' in the string content. '0xFF' represents a free space in a file. Method appends automatically end line characters (CRLF).
*/
bool StorageNIOSFlash::append (const char* lineToWrite, bool suppressErrMsg)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_append_0");
        return false;
    }

    if (!_async)
    {
        // Object has not enabled handling of an asynchronous
        if (!suppressErrMsg)
            Log.errorPrint("StorageNIOSFlash_append_1", true);
        return false;
    }

    if (!_fileOpened)
    {
		// Second parameter (true) disables logging in flash (recursion!)
        if (!suppressErrMsg)
            Log.errorPrint("StorageNIOSFlash_append_2", true);
        return false;
    }

	// Waiting for an access to the structures (rather there should be no blocking)
	// Prints out an optional error
    if (!_bufSem().lock (suppressErrMsg))
        return false;

    FileItem* fi = &files[_fIdent];

	// Check if file can be written only by an 'append' function
    if (!fi->appendOnly)
    {
        _bufSem().unlock (suppressErrMsg);
        if (!suppressErrMsg)
            Log.errorPrint("StorageNIOSFlash_append_3", true);
        return false;
    }

    // New position in buffer related with the user
    int newUserPos = (fi->cbUserPos + 1) % BUF_LINES;

	// Check if buffer is full
    if (newUserPos == fi->cbSystemPos)
    {
		// Buffer is full, written line is ignored. This is not an error.
        _bufSem().unlock (suppressErrMsg);
        return false;
    }

	// Copying of a line to the buffer
    char* t = static_cast <char*> (MEMCCPY (fi->appendCycBuf[newUserPos], lineToWrite, 0, LINESIZE-3));
    if (t == NULL)
		// Line to long - '/0' has not been copied
        t = fi->appendCycBuf[newUserPos] + LINESIZE - 3;
    else
        t--;

	// Copying of an end line character
    MEMCCPY (t, CRLF, 0, 3);

	// Protection if the line was too long and was truncated
    fi->appendCycBuf[newUserPos][LINESIZE-1] = '\0';

    fi->cbUserPos = newUserPos;

	// Unlocking access to the buffer
    if (!_bufSem().unlock (suppressErrMsg))
        return false;

    // Obudzenie zadania zapisuj¹cego, parametr fi okreœla który plik obs³u¿yæ w zadaniu
    // Poprzednie powiadomienie mog³o nie zostaæ obs³u¿one, ale to nie jest b³¹d

	// Awakening of the writing task, 'fi' parameter indicates the file to be handled by a task
	// Previous notification may not be handled but this is not an error
    INT8U err = OSMboxPost (_mboxSig, fi);
    if (err != OS_NO_ERR && err != OS_MBOX_FULL)
    {
        if (!suppressErrMsg)
            Log.errorPrint("StorageNIOSFlash_append_4 [", err, "]", true);
        return false;
    }

    return true;
}

bool StorageNIOSFlash::appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg)
{
	return true;
}

/** \name Method finds first occurrence of a text pattern
* \param 'text' - text pattern
* \param 'offset' - offset for searching in a file. Returning is offset of the found string.
* \param 'maxCount' - searches only for the specified number of bytes ('0' means no limits)
* \param 'stopText' - stops searching after finding the specified text (method then returns 'false')
* \return 'true' if pattern has been found and offset of the pattern has been set. 'false' if returned offset is equal to a file size (only if 'maxCount' == 0)
* or place directly after the termination of the search
* \note The function can not be interrupted by a higher priority task writing to flash
*/
  bool StorageNIOSFlash::scan (const char* text, int &offset, int maxCount, const char* stopText)
  {    
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_scan_0");
        return false;
    }

    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_scan_1");
        return false;
    }

	// Waiting for an access to the structure (rather there should be no blocking)
    //  Function 'lockStruct' prints out an optional error
    if (!_bufSem().lock ())
        return false;

    FileItem* fi = &files[_fIdent];

	// Check if file can written only by an 'append' function
    if (!fi->appendOnly)
    {
        _bufSem().unlock ();
        Log.errorPrintf("StorageNIOSFlash_scan_2");
        return false;
    }

	// Flash is accessible for reading directly in a address space of a system
//    char* fileBeg = reinterpret_cast<char*>(PILOT_FLASH_BASE
//                    + FILES_OFFSET + fi->fileOffset);

    int fileSize = fi->appendNextFilePos;    

    if (!_bufSem().unlock ())
        return false;

    int len = strlen (text);
    int stopLen = 0;
    if (stopText != NULL)
        stopLen = strlen (stopText);

	// An upper boundary must be less than the size of the text to search, to not to go out the range of an array in 'memcmp' function
    int uBound = fileSize - len;
    int maxOffset = uBound;
	// Limiting the search area
    if (maxCount > 0 && offset + maxCount < uBound)
        maxOffset = offset + maxCount;
    bool ret = false;
    char c = text[0];   // Time optimization
    u8 fileBeg[maxOffset];
//    FlashRead(&QspiInstance, FILES_OFFSET + fi->fileOffset, maxOffset, QUAD_READ_CMD, WriteBuffer, fileBeg);
//    memcpy(fileBeg, (const void*)ReadBuffer,maxOffset);

    while (offset < maxOffset)
    {
        if (fileBeg[offset] == c)
        {
            // Searching text
            if (memcmp (fileBeg + offset, text, len) == 0)
            {
                ret = true;
                break;
            }
            // Breaking text
            if (stopText != NULL && memcmp (fileBeg + offset, stopText, stopLen) == 0)
                break;
        }
        offset++;
    }

    // When the end of file is encountered, the offset contains the file size
    if (offset == uBound)
        offset = fileSize;

    return ret;
  }

/** \name Method reads a line of text from the file. Method doesn't copy an end line characters.
* \param 'buf' - buffer for the read line
* \param 'bufSize' - size of the buffer
* \param 'offset' - starting offset of reading, returns the offset to the beginning of a new line
* \return 'true' when line has been read successfully, 'false' when nothing has been read (end of file)
*/
bool StorageNIOSFlash::readLine (char* buf, int bufSize, int &offset)
  {    
    if (_singleThread)
    {
        Log.errorPrintf("StorageNIOSFlash_readLine_0");
        return false;
    }

    if (bufSize < 1)
    {
        Log.errorPrintf("StorageNIOSFlash_readLine_1");
        return false;
    }

    if (offset < 0)
    {
        Log.errorPrintf("StorageNIOSFlash_readLine_1a");
        return false;
    }
    
    buf[0] = '\0';
    
    if (!_fileOpened)
    {
        Log.errorPrintf("StorageNIOSFlash_readLine_2");
        return false;
    }

	// Waiting for an access to the structures (rather there should be no blocking)
	// Function 'lockStruct' prints out an optional error
    if (!_bufSem().lock ())
        return false;

    FileItem* fi = &files[_fIdent];

	//Check if file can be written by an 'append' function
    if (!fi->appendOnly)
    {
        _bufSem().unlock ();
        Log.errorPrintf("StorageNIOSFlash_readLine_3");
        return false;
    }

	// Flash is accessible for reading directly in a address space of a system
    char* fileBeg = reinterpret_cast<char*>(PILOT_FLASH_BASE
                    + FILES_OFFSET + fi->fileOffset);
    int fileSize = fi->appendNextFilePos;    

    if (!_bufSem().unlock ())
        return false;

    // Check if this is an end of a file
    if (offset >= fileSize)
    {
        offset = fileSize;
        return false;
    }
            
    int maxBytesToCopy  = fileSize - offset;
    if (bufSize < maxBytesToCopy)
        maxBytesToCopy = bufSize;
    maxBytesToCopy--;

	// Waiting for an access to flash - function 'lockFlash' prints out optional error
    if (!_flashSem().lock (false, FLASH_SEM_WAIT_MS))
        return false;
    
//    FlashRead(&QspiInstance, FILES_OFFSET + fi->fileOffset + offset, maxBytesToCopy, QUAD_READ_CMD, WriteBuffer, ReadBuffer);
    //memcpy(buf, (const void*)ReadBuffer,size);
    MEMCCPY (buf, ReadBuffer, '\r', maxBytesToCopy);

//    MEMCCPY (buf, fileBeg + offset, '\r', maxBytesToCopy);
    buf[bufSize-1] = '\0'; // for safety

    // Finding the location of the 'CR' character
    char* crPos = static_cast<char*>(memchr (buf, '\r', maxBytesToCopy));
    if (crPos != NULL)
    {
        *crPos = '\0';
        // Setting of a new offset (Line ends with CRLF, but it may be truncated)
        offset += (crPos - buf + 2);
    }
    else
    {
        //  There is no 'CR' character in a buffer (last line in a file)
        buf[maxBytesToCopy] = '\0';
        offset += (maxBytesToCopy + 1);
    }
                
    // Unlocking of an access
    if (!_flashSem().unlock ())
        return false;           
        
    return true;   
  }

/** \name Method finds a byte with size of 255 in a specified scope of memory. It is assumed that bytes 255 are located in a one group at the end of a buffer.
* The result is of type 'signed' because it specifies the position in a file (not in a address space)
* \param 'buf' - buffer to be searched
* \param 'size' - size of a buffer (>0)
* \return Returned is an index of a found byte <0 to size-1> or a value of size if the byte was not found or an error occured
*/
int StorageNIOSFlash::findEndOfFile (const char* buf, int size)
{
    int p1 = 0;
    int p2 = size;
    int ret = size;
    
    // Waiting for an access to flash - function 'lockFlash' prints out an optional errors
    if (!_flashSem().lock (false, FLASH_SEM_WAIT_MS))
        return size;
    
    // Binary search
    do
    {
        // Calculation of the middle of the range
        int m = (p2-p1)/2 + p1;
        if (buf[m] == static_cast<char>(255))
        {
            if(m == 0 || buf[m-1] != static_cast<char>(255))
            {
                // Byte has been found
                ret = m;
                break;
            }
            else
				// Shifting of the right boundary to the center
                p2 = m;    
        }
        else
        {
			// Shifting of the left boundary to the center
            p1 = m + 1;
        }
        
    } while (p1<p2);

    // Unlocking an access
    if (!_flashSem().unlock ())
        return size;           
    
    return ret;    
}

/** \name Method appends data to the end of a file
* \param 'fi' - pointer to the structure describing the file to be written
* \param 'buf' - pointer to data to be appended
* \param 'size' - size of data in bytes
* \return 'true' on success, 'false' otherwise
* \note Method operates only on a strings. Within a string cannot be used a '0xFF' character,
* those characters indicates an empty places in a file.
*/
bool StorageNIOSFlash::append2 (FileItem* fi, const char* buf, int size)
{        
    // Try to lock the flash (without error reporting)
    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
    {
		// Flash is busy - message is printed by a 'lockFlash' function
        return false;
    }        

	// Check if there is enough space in a flash
    if (fi->appendNextFilePos >= fi->fileSize)
    {
		// Flash is full - this is not an error
        _flashSem().unlock ();
        return false;
    }

//    // Number of a block in a file
//    int blockInFile = fi->appendNextFilePos/BLOCK_SIZE;
//	// Offset of the beginning of a block relative to the beginning of the flash
//    int blockBeg = FILES_OFFSET + fi->fileOffset + blockInFile*BLOCK_SIZE;
//	// Offset of the beginning of a data relative to the beginning of the flash
//    int dataBeg = FILES_OFFSET + fi->fileOffset + fi->appendNextFilePos;
//	// Pointer to the beginning of data to be written
//    const char* sourcePtr = buf;
//	// Size to the end of the block
//    int dsize = BLOCK_SIZE - (dataBeg - blockBeg);
//    if (dsize > size)
//        dsize = size;
//
//    if((fi->posInTPage + dsize) <= PAGESIZE)
//    {
//    	copyArray((u8*)sourcePtr,(u8*)fi->appendFlashBuf, (unsigned int)dsize, fi->posInTPage);
//    	fi->posInTPage += dsize;
//    	for(unsigned int i=0; i<fi->posInTPage; i++)
//    		WriteBuffer[OVERHEAD_SIZE + i] = fi->appendFlashBuf[0][i];
//    	FlashWrite(&QspiInstance, blockBeg + fi->posPage*Flash_Config_Table[FCTIndex].PageSize,fi->posInTPage,WRITE_CMD,  WriteBuffer);
//
//    }else
//    {
//    	copyArray((u8*)sourcePtr,(u8*)fi->appendFlashBuf, (unsigned int)(PAGESIZE - fi->posInTPage), fi->posInTPage);
//    	int tmp = 0;
//    	tmp = PAGESIZE - fi->posInTPage;
//    	fi->posInTPage = fi->posInTPage + dsize - PAGESIZE;
//    	for(unsigned int i=0; i<Flash_Config_Table[FCTIndex].PageSize; i++)
//    		WriteBuffer[OVERHEAD_SIZE + i] = fi->appendFlashBuf[0][i];
//    	FlashWrite(&QspiInstance, blockBeg + fi->posPage*Flash_Config_Table[FCTIndex].PageSize,Flash_Config_Table[FCTIndex].PageSize,WRITE_CMD,  WriteBuffer);
//    	fi->posPage += 1;
//    	for(unsigned int i=0; i<fi->posInTPage; i++)
//    	{
//    		fi->appendFlashBuf[0][i] = sourcePtr[tmp+i];
////    		WriteBuffer[OVERHEAD_SIZE + i] = 100;
//    		WriteBuffer[OVERHEAD_SIZE + i] = fi->appendFlashBuf[0][i];
//    	}
//    	FlashWrite(&QspiInstance, blockBeg + fi->posPage*Flash_Config_Table[FCTIndex].PageSize,fi->posInTPage,WRITE_CMD,  WriteBuffer);
//    }
//
//	// Writing to the flash without crossing scope of a block
////    int wcode = alt_write_flash_block(fd, blockBeg, dataBeg, sourcePtr, dsize);
//
//    // Updating position in a file
//    fi->appendNextFilePos += dsize;
//	// Writing to the next block in flash
//	// Check if there is some data to move to next block and if so check if th;ere is enough space in a flash
//    if (size > dsize &&
//        fi->appendNextFilePos < fi->fileSize)
//    {
//        blockBeg += BLOCK_SIZE;
//        dataBeg = blockBeg;
//        sourcePtr = buf + dsize;
//        dsize = size - dsize;
//        fi->posPage = 0;
//
//        if((fi->posInTPage + dsize) <= PAGESIZE)
//        {
//        	copyArray((u8*)sourcePtr,(u8*)fi->appendFlashBuf, (unsigned int)dsize, fi->posInTPage);
//        	fi->posInTPage += dsize;
//        	for(unsigned int i=0; i<fi->posInTPage; i++)
//        		WriteBuffer[OVERHEAD_SIZE + i] = fi->appendFlashBuf[0][i];
//        	FlashWrite(&QspiInstance, blockBeg + fi->posPage*Flash_Config_Table[FCTIndex].PageSize,fi->posInTPage,WRITE_CMD,  WriteBuffer);
//
//        }else
//        {
//        	copyArray((u8*)sourcePtr,(u8*)fi->appendFlashBuf, (unsigned int)(PAGESIZE - fi->posInTPage), fi->posInTPage);
//        	int tmp = 0;
//        	tmp = PAGESIZE - fi->posInTPage;
//        	fi->posInTPage = fi->posInTPage + dsize - PAGESIZE;
//        	for(unsigned int i=0; i<Flash_Config_Table[FCTIndex].PageSize; i++)
//        		WriteBuffer[OVERHEAD_SIZE + i] = fi->appendFlashBuf[0][i];
//        	FlashWrite(&QspiInstance, blockBeg + fi->posPage*Flash_Config_Table[FCTIndex].PageSize,Flash_Config_Table[FCTIndex].PageSize,WRITE_CMD,  WriteBuffer);
//        	fi->posPage += 1;
//        	for(unsigned int i=0; i<fi->posInTPage; i++)
//        	{
//        		fi->appendFlashBuf[0][i] = sourcePtr[tmp+i];
//        		WriteBuffer[OVERHEAD_SIZE + i] = fi->appendFlashBuf[0][i];
//        	}
//        	FlashWrite(&QspiInstance, blockBeg + fi->posPage*Flash_Config_Table[FCTIndex].PageSize,fi->posInTPage,WRITE_CMD,  WriteBuffer);
//        }
//
//		// Updating position in a file
//        fi->appendNextFilePos += dsize;
//    }
           
    // Unlocking flash - function '_flashSem().unlock' prints out an optional error
    if (!_flashSem().unlock ())
        return false;
    
    return true;
}

/** \name Method writes data block to the flash memory. Method is used by a FlashProgrammer.
*/
bool StorageNIOSFlash::writeFlash(const void* buf, unsigned int offset, int size)
{
	// Try to lock the flash (without error reporting)
    if (!_flashSem().lock (false, FLASH_SEM_WAIT_MS))
    {
		// Flash is busy - message is prints out by 'lockFlash' function
        return false;
    }        

    u8 dat[size];
    memcpy(dat, (const void*)buf,size);

	for(int j=0; j<size; j++)
		WriteBuffer[OVERHEAD_SIZE + j] = dat[j];
	FlashWrite(&QspiInstance, offset,size,WRITE_CMD,  WriteBuffer);

	// Unlocking the flash - function '_flashSem().unlock' ptints out an optional error
    if (!_flashSem().unlock ())
        return false;

    return true;
}

bool StorageNIOSFlash::eraseFlash(unsigned int offset, int size)
{
	// Try to lock the flash (without error reporting)
    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
    {
		// Flash is busy - message is prints out by 'lockFlash' function
        return false;
    }
    FlashErase(&QspiInstance, offset, size, WriteBuffer);
	// Unlocking the flash - function '_flashSem().unlock' ptints out an optional error
    if (!_flashSem().unlock ())
        return false;

    return true;
}

/** \name Method writes data block to the flash memory. Method is used by a FlashProgrammer.
*/
bool StorageNIOSFlash::readFlash(void* buf, unsigned int offset, int size)
{
    // Try to lock the flash (without error reporting)
    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
    {
        // Flash is busy - message is prints out by 'lockFlash' function
        return false;
    }        

    FlashRead(&QspiInstance, offset, size, QUAD_READ_CMD, WriteBuffer, ReadBuffer);
    memcpy(buf, (const void*)ReadBuffer,size);

    // Unlocking the flash - function '_flashSem().unlock' ptints out an optional error
    if (!_flashSem().unlock ())
        return false;

    return true;
}

/** \name Method claculates CRC sum for a flash memory. Method is protected by a semaphore and it is used by a FlashProgrammer.
*/
bool StorageNIOSFlash::calcFlashCRC(unsigned int offset, int size, INT16U& crc)
{
	// Try to lock the flash (without error reporting)
    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
    {
        // Flash is busy - message is prints out by 'lockFlash' function
        return false;
    }        
    
    crc = 0;  // for certainty
    crc = Crc::computeBin(reinterpret_cast<unsigned char*>(PILOT_FLASH_BASE + offset), size);

	// Unlocking of a flash - function '_flashSem().unlock' prints an optional error
    if (!_flashSem().unlock ())
        return false;

    return true;
}

unsigned int StorageNIOSFlash::buffToUint(u8* buf)
{
	unsigned ret = static_cast<unsigned int>(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
	return ret;
}

void StorageNIOSFlash::copyArray(u8*from, u8* to, unsigned int length, unsigned int offset)
{
	for(unsigned int i=0; i<length; i++)
		to[offset+i] = from[i];
}

bool StorageNIOSFlash::mkdir(std::string path)
{
	return true;
}

bool StorageNIOSFlash::isDirectory(std::string path) {
	return true;
} // isDirectory

std::vector<std::string> StorageNIOSFlash::pathSplit(std::string path) {

} // pathSplit

std::vector<std::string> StorageNIOSFlash::listFile(void)
{

}

bool StorageNIOSFlash::renameFile(const char* oldname, const char* newname){
	return true;
}

bool StorageNIOSFlash::renameFile(std::string oldname, std::string newname){
	return true;
}

bool StorageNIOSFlash::check_file(const char *fname)
{
	return true;
}

bool StorageNIOSFlash::remove(const char *fname)
{
		return true;
}

bool StorageNIOSFlash::remove(std::string path)
{
	return true;
}

bool StorageNIOSFlash::hasSuffix(const std::string& s, const std::string& suffix)
{
	return true;
}

bool StorageNIOSFlash::contain(const std::string& s, const std::string& subs)
{
		return true;
}

bool StorageNIOSFlash::copy(std::string strFrom, std::string strTo)
{
	return true;
}

bool StorageNIOSFlash::formatSD()
{
	return true;
}

bool StorageNIOSFlash::UploadFile(ClassifiedLine& cl, std::string nameFile)
{
	return true;
}


#endif  // PILOT_TARGET
