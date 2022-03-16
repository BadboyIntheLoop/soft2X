#include <PilotIncludes.h>
#include <stdio.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in vConfig.h"
#endif

#if (PILOT_TARGET == PT_HARDWARE)
#include <unistd.h>
// Initialization of a static class members
bool StorageSD::_staticInitialized = false;
StorageSD::FileItem2 StorageSD::fileSD[] =
{
		{TYPE_LOG, BUF_LINE_SIZE, false, 0},
		{TYPE_DAT, BUF_LINE_SIZE, false, 0}
//		{{0}, 0, 0, BUF_LINE_SIZE, false,0,"",0},
//		{{0}, 0, 0, BUF_LINE_SIZE, false,0,"",0}
};

uint8_t StorageSD::error_status = 0;


/** \name Method wraps a static semaphore. Method is needed to guarantee the static initialization order.
*/
Semaphore& StorageSD::_flashSem (void) const
{
    static Semaphore xFlashSem;
    return xFlashSem;
}

// bienht
Semaphore& StorageSD::_cirBufSem(uint8_t index) const
{
//	switch (index)
//	{
//		case TYPE_LOG:
//			static Semaphore _xcirBufSemLog; // Khoi tao 1 mang gom 2 semaphore cho 2 circular buffer
//			return _xcirBufSemLog;		 // Return lai Semaphore lien quan
//			break;
//		case TYPE_DAT:
//			static Semaphore _xcirBufSemDat; // Khoi tao 1 mang gom 2 semaphore cho 2 circular buffer
//			return _xcirBufSemDat;		 // Return lai Semaphore lien quan
//			break;
//		default:
//			return _xcirBufSemLog;
//			break;
//	}
	static Semaphore _xcirBufSemLog; // Initiate semaphore for circular log buffer
	static Semaphore _xcirBufSemDat; // Initiate semaphore for circular dat buffer

	if (index == TYPE_LOG)
	{
		return _xcirBufSemLog;
	}
	else if (index == TYPE_DAT)
	{
		return _xcirBufSemDat;
	}
	else
	{
		return _xcirBufSemLog;
	}
}


/** \name Class constructor
* \param 'async' - flag enables asynchronous handling of an 'append' function (through dedicated task)
*/
StorageSD::StorageSD( bool async, bool appendMode):
	_async(async),
	_mboxSigSD(static_cast<OS_EVENT*>(0)),
//	_Path((TCHAR *)"/sdcard/"),
	_appendMode(appendMode)
{
#if EXTERNAL_MEMORY_TYPE == USE_MMC
	_Path = (TCHAR *)"1:/";
#elif EXTERNAL_MEMORY_TYPE == USE_SDCARD
	_Path = (TCHAR *)"0:/";
#endif
    res = FR_DISK_ERR;
	cntToLog =0;
	numRead = 0;

	// Creation of a mailbox of the uC/OS-II system to synchronize with writing thread (for each object of this class)
    if (async)
    {
        _mboxSigSD = OSMboxCreate (static_cast<void*>(0));
        if (_mboxSigSD == static_cast<OS_EVENT*>(0)) 
        {
            Log.abort ("Critical Error: StorageSD_1.");
            return;
        }
    }

	// Creation of a semaphores common to all objects of this class (if they have not been created already)
	// Initialized at the beginning as a static members
    if (!_staticInitialized)
    {
		// Semaphore controlling access to flash memory
        if (!_flashSem().create("StorageSD_SDcard"))
        {
            Log.abort ("Critical Error: StorageSD_2.");
            return;
        }

		if (!_cirBufSem(TYPE_LOG).create("StorageSD_data"))
		{
			Log.abort ("Critical Error: StorageSD_5.");
			return;
		}

		if (!_cirBufSem(TYPE_DAT).create("StorageSD_data"))
		{
			Log.abort ("Critical Error: StorageSD_6.");
			return;
		}
//FIXME
//    	if (disk_module_init() == 0)
//    	{
//    		Log.abort ("Critical Error: StorageSD_2a.");
//    		return;		// Hardware error
//    	}
		/*
		 * Register volume work area, initialize device
		 */
		res = f_mount(&fatfs, _Path, 0);

		if (res != FR_OK) {
			Log.errorPrint("StorageSD_init_1", true);
		}

//		/*
//		 * Path - Path to logical driver, 0 - FDISK format.
//		 * 0 - Cluster size is automatically determined based on Vol size.
//		 */
//		res = f_mkfs(_Path, 0, 0);
//		if (res != FR_OK) {
//			Log.errorPrint("StorageSD_init_2", true);
//		}
    }
   
   _staticInitialized = true;
}

/** \name Method creates new file or opens it if it's created already.
* In single object only one file may be opened. If some file was opened before, it is going to be closed.
* A file cannot be opened by a multiple objects simultaneously. A file name must be one of the predefined file names.
* \param 'fname' - file name
* \return 'true' on success, 'false' otherwise
*/
bool StorageSD::open (const char* fName)
{
	MEMCCPY(_fileSDName, fName, 0, FNAME_LENGTH); 		// Open
    return true;
}

/** \name Method closes file.
* Dummy method which changes only an file status of open
*/
bool StorageSD::close (void)
{
    return true;
}


bool StorageSD::appendBinary(const char* fileName, const void* buf, int size)
{
	    return true;
}
/** \name Method saves to file any data with attached signatures of the beginning (4 bytes), ending (4 bytes) and CRC sum (2 bytes)
* \param 'buf' - pointer to a data to be written
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
* \note Wrtting can be performed only at the beginning and in the range of a single block of flash memory. Before writting all memory block is erased,
* previous content isn't secured.
*/
bool StorageSD::save (const void* buf, int size)
{
	FIL file;
	unsigned int bw;
	FRESULT res;

    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
	{
		// Flash is busy - message is printed by a 'lockFlash' function
		return false;
	}

	res = f_open(&file, _fileSDName, FA_WRITE | FA_OPEN_ALWAYS);// Nen su dung FA_OPEN_EXISTING 	// save
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_save_1", true);
		_flashSem().unlock ();
		return false;
	}

	res = f_write(&file, buf, size, &bw);	/* Write data to the file */
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_save_2", true);
		_flashSem().unlock ();
		return false;
	}

	if ((int)bw != size)
	{
		Log.errorPrint("StorageSD_save_3", true);
		_flashSem().unlock ();
		return false;
	}

	INT16U crc = Crc::computeBin(static_cast<const unsigned char*>(buf), size);

	res = f_write(&file, &crc, sizeof(crc), &bw);	/* Write data to the file */
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_save_4", true);
		_flashSem().unlock ();
		return false;
	}

	if ((int)bw != sizeof(crc))
	{
		Log.errorPrint("StorageSD_save_5", true);
		_flashSem().unlock ();
		return false;
	}

	res = f_close(&file);			/* Close the file */
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_save_6", true);
		_flashSem().unlock ();
		return false;
	}

	if (!_flashSem().unlock ())
	        return false;


	return true;
}

/** \name Method reads from file any data with validation (signatures of the beginning, ending and CRC sum)
* \param 'buf' - pointer to the buffer to which data will be read
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageSD::load (void* buf, int size)
{
	FIL file;
	unsigned int br;
	FRESULT res;

    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
	{
		// Flash is busy - message is printed by a 'lockFlash' function
		return false;
	}

	res = f_open(&file, _fileSDName, FA_READ | FA_OPEN_EXISTING);		// Load
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_load_1", true);
		_flashSem().unlock ();
		return false;
	}

	res = f_read(&file, buf, size, &br);
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_load_2", true);
		_flashSem().unlock ();
		return false;
	}
	if ((int)br != size)
	{
		Log.errorPrint("StorageSD_load_3", true);
		_flashSem().unlock ();
		return false;
	}

	INT16U readedCrc;
	res = f_read(&file, &readedCrc, sizeof(readedCrc), &br);
	if ((int)br != sizeof(readedCrc))
	{
		Log.errorPrint("StorageSD_load_4", true);
		_flashSem().unlock ();
		return false;
	}

	INT16U computedCrc = Crc::computeBin(static_cast<unsigned char*>(buf),size);
	if (readedCrc != computedCrc)
	{
		Log.errorPrint("StorageSD_load_5", true);
		_flashSem().unlock ();
		return false;
	}

	res = f_close(&file);			/* Close the file */
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_load_5", true);
	    return false;
	}
	if (!_flashSem().unlock ())
		        return false;

//	if(error_status==0)
//	{
//		error_status &= (~SMON_ERR_LOADCONF);  // enter here only if there is no error of loading both the two config files
//	}
    return true;
}

/** \name Method handles uC/OS-II system tasks of writing the line to the flash
*/
void StorageSD::task(void* pdata)
{
    INT8U err = 0;
    char bufSD[BUF_LINE_SIZE];
    unsigned int bufSDLen;
    bool ret_val;
    
    if (!_async)
    {
        // Object has not enabled handling of an asynchronous
        Log.errorPrint("StorageSD_task_0", true);
        OSTaskSuspend (OS_PRIO_SELF);        
    }
    
    while (true)
    {

		// Waiting for a signal that something has been added to the cyclic buffer
		// 'fiSD' specifies the file to be handled
    	void* item = OSMboxPend(_mboxSigSD, 0, &err);

		if (err != OS_NO_ERR || item == static_cast<void*>(0))
		{
			Log.errorPrint("StorageSD_task_1 [", err, "]", true);
//			printf("\n Loi task");
			// Wait 10s to not clog up the error log
			OSTimeDlyHMSM (0, 0, ERRWAIT, 0);
			continue;
		}

//		__PERFORMANCE_START(MOD_STORAGE_SD)

		FileItem2* fiSD = static_cast<FileItem2*>(item);
		int filetype = fiSD->fileType;

		if (!_cirBufSem(filetype).lock ())
			// If the semaphore cannot be locked then method waits for the next signal
			continue;

		// Check if there is any line waiting in the cyclic buffer
		if (fiSD->cbUserPos == fiSD->cbSystemPos)
		{
			_cirBufSem(filetype).unlock ();
			continue;
		}

		// New position in buffer for a writing task
		int newSystemPos = (fiSD->cbSystemPos + 1) % BUF_LINE_NUM;
		// Copying of a line to the local buffer to quickly release the semaphore
		switch (filetype)
		{
			case TYPE_LOG:
			{
				// Line in a cyclic buffer has a guaranteed '/0' character at the end
				MEMCCPY (bufSD, fiSD->appendCycBuf[newSystemPos], 0, BUF_LINE_SIZE);
				bufSDLen = strlen(bufSD);
				break;
			}
			case TYPE_DAT:
			{
				memcpy(bufSD, fiSD->appendCycBuf[newSystemPos], fiSD->size);
				bufSDLen = fiSD->size;
				break;
			}
			default:
				break;
		}

		// Check if there is any line waiting for a write
		if (fiSD->cbUserPos != newSystemPos)
		{
			// Awakening of the writing task for the next loop iteration
			// 'fi' parameters specifies currently handled file
			err = OSMboxPost (_mboxSigSD, fiSD);
			if (err != OS_NO_ERR && err != OS_MBOX_FULL)
			{
				Log.errorPrint("StorageSD_task_9 [", err, "]", true);
			}
		}


		// Unlocking of an access to the buffer - prints out optional error
		_cirBufSem(filetype).unlock ();

		// Writing to the flash
		ret_val = appendNewInternal(bufSD, fiSD, bufSDLen);	// task
		// Updating of the new position only if writing to the SD card is successful
		if (ret_val)
		{
			fiSD->cbSystemPos = newSystemPos;
		}
		else
		{
			;
		}


		// Post
		cntToLog++;
		if (cntToLog >= 250)
		{
//			printf("interval %d ms\n",Os->ticks());
//			Log.msgPrintf("interval %d ms\n",Os->ticks());
			cntToLog = 0;
			// Awakening of the writing task for the next loop iteration
			// 'log file' parameters specifies currently handled file
			err = OSMboxPost (_mboxSigSD, &fileSD[TYPE_LOG]);
			if (err != OS_NO_ERR && err != OS_MBOX_FULL)
			{
				Log.errorPrint("StorageSD_task_10 [", err, "]", true);
			}
		}

//		__PERFORMANCE_INFO(MOD_STORAGE_SD)
    }    
}

/** \name Method appends a text line to the end of file.
* \param 'lineToWrite' - pointer to a string to be appended
* \return 'true' on success, 'false' otherwise
* \note There cannot be '0xFF' in the string content. '0xFF' represents a free space in a file. Method appends automatically end line characters (CRLF).
*/
bool StorageSD::append (const char* lineToWrite, bool suppressErrMsg)
{
    return true;
}


///** \name Method appends a line of text ('buf') to the end of file.
//* \param 'buf' - pointer to data which will be appended (text line)
//* \param 'suppressErrMsg' - if 'true' no error messages are displayed (to avoid recurrence)
//* \return 'true' on success, 'false' otherwise
//* \note There cannot be '0xFF' in the string content. '0xFF' represents a free space in a file. Method appends automatically end line characters (CRLF).
//*/
bool StorageSD::appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg)
{
	uint8_t file_type;

	if (!_async)
    {
        // Object has not enabled handling of an asynchronous
        if (!suppressErrMsg)
            Log.errorPrint("StorageSD_append_1", true);
        return false;
    }

	// Check file name de phan biet ghi vao file type log (1) hay data (2)
	//FIXME
	/*
	int namecpy = strcmp(LOG_FILE_NAME, fileName);
	if (namecpy == 0)
	{
		file_type = TYPE_LOG;		// log file
	}
	else
	{
		file_type = TYPE_DAT;		// dat file
	}
	*/
	//default file_type = TYPE_LOG
	file_type = TYPE_LOG;

	// Waiting for an access to the structures (rather there should be no blocking)
	// Prints out an optional error
	if (!_cirBufSem(file_type).lock (suppressErrMsg))
	{
		return false;
	}

    FileItem2* fiSD1 = &fileSD[file_type];
    MEMCCPY(fiSD1->fileName, fileName, 0, FNAME_LENGTH);
    fiSD1->fileType = file_type;

    // New position in buffer related with the user
	int newUserPos = (fiSD1->cbUserPos + 1) % BUF_LINE_NUM;

	// Check if buffer is full.
	// Buffer is full if " (fiSD->cbUserPos[_fileType] +1) =  fiSD->cbSystemPos[_fileType] "
	// or in other word "tail + 1 (buffer write pointer) = head (buffer read pointer)"
	if (newUserPos == fiSD1->cbSystemPos)
	{
		// increase buffer reading pointer 1 position for writing new data to the buffer
		fiSD1->cbSystemPos = fiSD1->cbSystemPos + 1;
	}

	// coppy new data (buf) to the cyclic buffer
	switch (file_type)
	{
		case TYPE_LOG:
		{
			// 1. Copying of a line to the buffer
			char* t = static_cast <char*> (MEMCCPY (fiSD1->appendCycBuf[newUserPos], buf, 0, BUF_LINE_SIZE-3));
			// 2. Get buf size
			if (t == NULL)
			{
				// Line too long - '/0' has not been copied
				t = fiSD1->appendCycBuf[newUserPos] + BUF_LINE_SIZE - 3;
			}
			else
			{
				t--; // the size of the buffer = result of MEMCCPY - 1
			}
			// 3. Copying of an end line character
			MEMCCPY (t, CRLF, 0, 3);
			// 4. Protection if the line was too long and was truncated
			fiSD1->appendCycBuf[newUserPos][BUF_LINE_SIZE-1] = '\0';
			break;
		}
		case TYPE_DAT:
		{
			memcpy(fiSD1->appendCycBuf[newUserPos], buf, size);
			fiSD1->size = size;
			break;
		}
		default:
			break;
	}

	// Assign new value to cbUserPos pointer
	fiSD1->cbUserPos = newUserPos;


	// Awakening of the writing task, 'fiSD' parameter indicates the file to be handled by a task
	// Previous notification may not be handled but this is not an error

	INT8U err = OSMboxPost (_mboxSigSD, &fileSD[file_type]);
	if (err != OS_NO_ERR && err != OS_MBOX_FULL)
	{
		Log.errorPrint("StorageSD_appendBinary_4 [", err, "]", true);
		// Unlocking access to the buffer
		_cirBufSem(file_type).unlock(suppressErrMsg);
		return false;
	}

	// Unlocking access to the buffer
	if (!_cirBufSem(file_type).unlock(suppressErrMsg))
	{
		return false;
	}

	return true;
}



bool StorageSD::appendNewInternal(char* buf, FileItem2* fileItem, int size)
{
	unsigned int numbw;

	if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
	{
		// Flash is busy - message is printed by a 'lockFlash' function
		return false;
	}

	if (fileItem->fileType == TYPE_DAT)
	{
		fileItem->isFileOpened = false;
	}

	if ( !(fileItem->isFileOpened) )
	{
		res = f_open(&(fileItem->fileSDObj), fileItem->fileName , FA_WRITE | FA_OPEN_ALWAYS );// Nen su dung FA_OPEN_EXISTING
		if (res != FR_OK)
		{
			Log.errorPrint("StorageSD_appendBinary2_1", true);
			_flashSem().unlock ();

			return false;
		}

		res = f_lseek(&(fileItem->fileSDObj), file_size(&(fileItem->fileSDObj)));
		if (res != FR_OK)
		{
			Log.errorPrint("StorageSD_appendBinary2_2", true);
			_flashSem().unlock ();

			return false;
		}

		fileItem->isFileOpened = true;	// false if log. true if dat
	}
	res = f_write(&(fileItem->fileSDObj), buf, size, &numbw);	/* Write data to the file */
	if (res != FR_OK)
	{
		fileItem->isFileOpened = false;
		Log.errorPrint("StorageSD_appendBinary2_3", true);
		_flashSem().unlock ();

		return false;
	}

	if (fileItem->fileType == TYPE_LOG)
	{
		fileItem->countappend++;
	}
	else
	{
		fileItem->countappend = 222;
	}

	if (fileItem->countappend > 200)
	{
		res = f_close(&(fileItem->fileSDObj));
		if (res != FR_OK)
		{
			Log.errorPrint("StorageSD_appendBinary2_4", true);
			_flashSem().unlock ();

			return false;
		}
		fileItem->countappend = 0;
		fileItem->isFileOpened = false;
	}
	else
	{
		res = f_sync(&(fileItem->fileSDObj));
		if (res != FR_OK)
		{
			Log.errorPrint("StorageSD_appendBinary2_5", true);
			_flashSem().unlock ();

			return false;
		}
	}

	if (!_flashSem().unlock ())
	{
		return false;
	}



	return true;
}

bool StorageSD::getFileSize(const char * fname,unsigned int * len)
{
	if (!_flashSem().lock(false, APPD_FLASH_SEM_WAIT_MS))
	{
		return false;
	}
	FILINFO sb;
	FRESULT ret = f_stat(fname, &sb);
	if(ret == FR_OK && sb.fsize > 0)
	{
		*len = sb.fsize;
		_flashSem().unlock();
		return true;
	}
	_flashSem().unlock();
	return false;
}


/** \name Method claculates CRC sum for a flash memory. Method is protected by a semaphore and it is used by a FlashProgrammer.
*/
//bool StorageSD::calcCRC(const void * buf, int size, INT16U& crc)
//{
//    crc = 0;  // for certainty
//    crc = Crc::computeBin(reinterpret_cast<unsigned char*>(buf), size);
//    return true;
//}

bool StorageSD::isFileExist(const char* prefix, const int index)
{
	FIL file;
	char tempBuf[8];	// check buf

	// Check
	SNPRINTF(tempBuf, sizeof(tempBuf), "%s%d", prefix, index);

	// Try open card
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}

 	res = f_open(&file, tempBuf, FA_READ | FA_OPEN_EXISTING);		// thu mo tempBuf
    if (res != FR_OK)
    {
    	Log.errorPrint("StorageSD_isExist_1", true);
    	_flashSem().unlock();
    	return false;
    }

    // close the file
	res = f_close(&file);
    if (res != FR_OK)
    {
    	Log.errorPrint("StorageSD_isExist_1", true);
    	_flashSem().unlock();
    	return false;
    }

    if (!_flashSem().unlock ())
            return false;
    return true;
}

bool StorageSD::clear (bool fastClear)
{
	// Try open card
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}

//	res = f_mkfs("", 0 , 0);
	res = f_mkfs(_Path, 0 , 0);
	if(res == FR_MKFS_ABORTED)
	{
    	Log.errorPrint("Format failed", true);
    	_flashSem().unlock();
		return 0;
	}

    if (!_flashSem().unlock ())
    {
		return false;
    }

	Log.msgPrintf("StorageSD:Disk formated and Mount OK.");
	return 1;
}

uint8_t StorageSD::getStorageErr(void)
{
	return error_status;
}

bool StorageSD::mkdir(std::string path)
{
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}

	FRESULT rc = f_mkdir(path.c_str());
	if (rc != FR_OK) {
    	Log.errorPrint("StorageSD: Mkdir failed", true);
    	_flashSem().unlock();
		return  false;
	}
    if (!_flashSem().unlock ())
    {
		return false;
    }

	return true;
}

bool StorageSD::isDirectory(std::string path) {
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}

	FILINFO statBuf;
	FRESULT rc = f_stat(path.c_str(), &statBuf);
	if (rc != FR_OK)
	{
		_flashSem().unlock();
		return false;
	}
    if (!_flashSem().unlock ())
    {
		return false;
    }
	return true;
} // isDirectory

std::vector<std::string> StorageSD::pathSplit(std::string path) {
	std::istringstream stream(path);
	std::vector<std::string> ret;
	std::string pathPart;
	while (std::getline(stream, pathPart, '/')) {
		ret.push_back(pathPart);
	}
	return ret;
} // pathSplit

std::vector<std::string> StorageSD::listFile(void)
{
	std::vector<std::string> ret;
	FILINFO finfo;
//	FILINFO *finfo = new FILINFO;
	FRESULT res;
	DIR dirs;
	char lfname[LINESIZE];
	finfo.lfname = lfname;
//	finfo.lfname[0] = ' ';
	finfo.lfsize = LINESIZE;

	_flashSem().lock(false, FLASH_SEM_WAIT_MS);

	res = f_opendir(&dirs, _Path);
	if(res == FR_OK)
	{
		while((f_readdir(&dirs, &finfo) == FR_OK) && finfo.lfname[0])
		{
			if(finfo.fattrib & AM_ARC)
			{
				char name[LINESIZE];
				MEMCCPY(name, lfname, 0, finfo.lfsize); 		// Open
				std::string tmp = name;
				ret.push_back(tmp);
			}else
			{
				continue;
			}
		}
	}else
	{
		Log.errorPrint("StorageSD: failed to open root directory", true);
	}
	_flashSem().unlock();
	return ret;
}

bool StorageSD::renameFile(const char* oldname, const char* newname){
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}
	FRESULT result;
	result= f_rename(oldname, newname);
	if(result != FR_OK)
	{
		_flashSem().unlock();
		return false;
	}
    if (!_flashSem().unlock ())
    {
		return false;
    }
	return true;
}

bool StorageSD::renameFile(std::string oldname, std::string newname){
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}
	FRESULT result;
	result= f_rename(oldname.c_str(),newname.c_str());
	if(result != FR_OK)
	{
		_flashSem().unlock();
		return false;
	}
    if (!_flashSem().unlock ())
    {
        return false;
    }
	return true;
}

bool StorageSD::check_file(const char *fname)
{
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}
	FILINFO sb;
	FRESULT ret = f_stat(fname, &sb);
	if(ret == FR_OK && sb.fsize > 0)
	{
		_flashSem().unlock();
		return true;
	}
	_flashSem().unlock();
	return false;
}

bool StorageSD::remove(const char *fname)
{
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}
	FRESULT ret;
	ret = f_unlink(fname);
	if(ret != FR_OK)
	{
		_flashSem().unlock();
		return false;
	}

    if (!_flashSem().unlock ())
    {
    	return false;
    }
	return true;
}

bool StorageSD::remove(std::string path)
{
#if EXTERNAL_MEMORY_TYPE == USE_MMC
	std::string rootName = "1:/" + path;
#elif EXTERNAL_MEMORY_TYPE == USE_SDCARD
	std::string rootName = path;
#endif
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}
	FRESULT ret;
	ret = f_unlink(rootName.c_str());
	if(ret != FR_OK)
	{
		_flashSem().unlock();
		return false;
	}

    if (!_flashSem().unlock ())
    {
    	return false;
    }
	return true;
}

bool StorageSD::hasSuffix(const std::string& s, const std::string& suffix)
{
	return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

bool StorageSD::contain(const std::string& s, const std::string& subs)
{
	if (s.find(subs) != std::string::npos) {
		return true;
	}
	return false;
}

bool StorageSD::copy(std::string strFrom, std::string strTo)
{
	std::ifstream srce(strFrom.c_str(), std::ios::binary);
	std::ofstream dest(strTo.c_str(), std::ios::binary);
	dest << srce.rdbuf();

	return true;
}

bool StorageSD::formatSD()
{
	if (!_flashSem().lock(false, APPD_BUF_SEM_WAIT_MS))
	{
		return false;
	}
	FRESULT ret;
	ret = f_mkfs("", 0, 0);
	if (ret != FR_OK) {
		Log.errorPrint("StorageSD_formatSD", true);
		_flashSem().unlock();
		return false;
	}
    if (!_flashSem().unlock ())
    {
    	return false;
    }
	return true;
}

bool StorageSD::UploadFile(ClassifiedLine& cl, std::string nameFile)
{
	unsigned int fileSize = 0;
	FRESULT res;
	char buf;
	int size = LINESIZE;
	char buf_line[LINESIZE];
	int count = 0;
	char buf_send[2*LINESIZE];   // Size during processing may be larger then LINESIZE.

	numRead = 0;
#if EXTERNAL_MEMORY_TYPE == USE_MMC
	std::string rootName = "1:/" + nameFile;
#elif EXTERNAL_MEMORY_TYPE == USE_SDCARD
	std::string rootName = nameFile;
#endif
//	//Get File Length
//	if(!getFileSize(rootName.c_str(), &fileSize))
//	{
//		return false;
//	}
	//Open File
    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
	{
		// Flash is busy - message is printed by a 'lockFlash' function
		return false;
	}
	res = f_open(&fileUpload, rootName.c_str(), FA_READ);		// Load
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_load_1", true);
		_flashSem().unlock ();
		return false;
	}
	if (!_flashSem().unlock ())
		return false;

	bool endoffile = false;
	while(!f_eof(&fileUpload))
	{
		int i = 0;
	//ReadLine and Send
		while(true)
		{
			if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
			{
				// Flash is busy - message is printed by a 'lockFlash' function
				return false;
			}
			res = f_read(&fileUpload, &buf, 1, &numRead);
			if (res != FR_OK)
			{
				Log.errorPrint("StorageSD_load_2", true);
				_flashSem().unlock ();
				return false;
			}
			if (!_flashSem().unlock ())
				return false;
			//
			if(f_eof(&fileUpload))
			{
				endoffile = true;
				break;
			}

			// Skipping lines starting with any end-line characters
			if ((i == 0) && (buf == '\r' || buf == '\n'))
			{
				continue;
			}
			if (i < LINESIZE)
				buf_line[i++] = buf;

			if('\n' == buf || '\r' == buf)
			{
				break;
			}

		}
		buf_line[i-1] = '\0';
		count++;
		if (SNPRINTF (buf_send, sizeof(buf_send), "log: line:%5d  %s", count, buf_line) >= 0)
		{
			if(!endoffile)
				cl.answer(buf_send, true, false);
		}
	}

	//Close File
    if (!_flashSem().lock (false, APPD_FLASH_SEM_WAIT_MS))
	{
		// Flash is busy - message is printed by a 'lockFlash' function
		return false;
	}
	res = f_close(&fileUpload);			/* Close the file */
	if (res != FR_OK)
	{
		Log.errorPrint("StorageSD_load_5", true);
	    return false;
	}
	if (!_flashSem().unlock ())
		return false;

    return true;
}

bool StorageSD::read (void* buf, int size, int offset)
{

	return true;
}

int StorageSD::getSize (void)
{
	return 0;
}

int StorageSD::getFree (void)
{
	return 0;
}

bool StorageSD::scan (const char* text, int &offset, int maxCount, const char* stopText)
{

	return true;
}

bool StorageSD::readLine (char* buf, int bufSize, int &offset)
{
	return true;
}

bool StorageSD::writeFlash(const void* buf, unsigned int offset, int size)
{
	return true;
}

bool StorageSD::calcFlashCRC(unsigned int offset, int size, INT16U& crc)
{
	return true;
}

bool StorageSD::readFlash(void* buf, unsigned int offset, int size)
{
	return true;
}

bool StorageSD::eraseFlash(unsigned int offset, int size)
{
	return true;
}

#endif  // PILOT_TARGET
