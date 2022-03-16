#include "PilotIncludes.h"

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

// Class static members initialization
const unsigned int StorageW32::BEG_SIGNATURE = 0x40a1cc0d;
const unsigned int StorageW32::END_SIGNATURE = 0x3ba109e4;

/** \name Class constructor. If directory doesn't exists it will be created if only one directory must be created, not the entire path.
* \param 'path' - path to the directory where the files will be
* \param 'async' - flag enables asynchronous working of an 'append' function (on Win32 it is not used but flag set to 'false'
* disables 'append' function compatibility with version for NIOS)
*/
StorageW32::StorageW32(const char* path, bool async, bool singleThread)
{
    fd = -1;
    fileOpened = false;
    _async = async;
    _f = NULL;  // File is not opened, it must be NULL in case of try to close
    _fro = NULL;
    _singleThread = singleThread;
   
    // Try to create directory
    int err2 = _mkdir(path);
    if (err2 == 0 || (err2 != 0 && errno == EEXIST))
    {
        // The directory was created or already existed
        int err = _chdir (path);
        if (err != 0)
        {
            Log.abort("Error: StorageW32_1: [", errno, "]");
            return;
        }
    }
    else
    {
        Log.abort("Error: StorageW32_2: [", errno, "]");
        return;
    }

	// File access semaphore (functions 'append', 'scan' and 'clear')
	if (!_vSem.create("StorageW32"))
    {
        Log.abort ("Critical Error: StorageW32_3.");
        return;
    }
}

/** \name Method creates file if not exists and opens it. Only one file can be opened at a time. If some file is opened, method coses it,
* and opens file specified by 'fname'.
* \param 'fname' - file name 
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::open (const char* fname)
{
	// Closing of other opened file
    if (fileOpened)
        close ();

    fd = _open (fname, _O_BINARY | _O_CREAT | _O_RDWR, _S_IWRITE);
    if (fd == -1)
    {
        Log.errorPrint("StorageW32_open_1 [", errno, "]");
        return false;
    }

	// Stream interface (with buffering) - it significantly accelerates the 'scan' function
    _f = _fdopen(fd, "r+b");    // Binary mode for reading and writting
    if (_f == NULL)
    {
        Log.errorPrint("StorageW32_open_2 [", errno, "]");
        return false;
    }

    _fro = fopen (fname, "rb"); // Binary mode for reading
    if (_fro == NULL)
    {
        Log.errorPrint("StorageW32_open_3 [", errno, "]");
        return false;
    }

    fileOpened = true;

    return true;
}
    
/** \name Method closes the file.
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::close (void)
{
    if (fileOpened)
    {
        if (_f != NULL)
        {
            int err = fclose (_f);
            if  (err == -1)
            {
                Log.errorPrint("StorageW32_close_1 [", errno, "]");
                return false;
            }
        }

        if (_fro != NULL)
        {
            int err = fclose (_fro);
            if  (err == -1)
            {
                Log.errorPrint("StorageW32_close_2 [", errno, "]");
                return false;
            }
        }

        fileOpened = false;
    }
    return true;
}

/** \name Method writes to the file any data with appended signature of the begin (4 bytes), signature of the end (4 bytes)
* and CRC sum (2 bytes)
* \param 'buf' - pointer to data to be written
* \param 'size' - size of data in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::save (const void* buf, int size)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_save_4");
        return false;
    }

    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_save_3");
        return false;
    }

	// Setting position in a file
    if (_lseek (fd, 0, SEEK_SET) == -1)
    {
        Log.errorPrintf("StorageW32_save_1 [%d]", errno);
        return false;
    }

	// Writting signature of the beginning
    int cnt;
    cnt=_write (fd, &BEG_SIGNATURE, sizeof(BEG_SIGNATURE));
    if (cnt != sizeof(BEG_SIGNATURE))
    {
        Log.errorPrintf("StorageW32_save_5 [%d]", errno);
        return false;
    }

    // Writting to the file
    cnt=_write (fd, buf, size);
    if (cnt != size)
    {
        Log.errorPrintf("StorageW32_save_6 [%d]", errno);
        return false;
    }

    // Writting signature of the ending
    cnt=_write (fd, &END_SIGNATURE, sizeof(END_SIGNATURE));
    if (cnt != sizeof(END_SIGNATURE))
    {
        Log.errorPrintf("StorageW32_save_7 [%d]", errno);
        return false;
    }

    // Writting CRC sum
    INT16U ecrc = Crc::computeBin (static_cast<const unsigned char*>(buf), size);
    cnt=_write (fd, &ecrc, sizeof(ecrc));
    if (cnt != sizeof(ecrc))
    {
        Log.errorPrintf("StorageW32_save_8 [%d]", errno);
        return false;
    }

    return true;
}

/** \name Methods reads from file any data with validation (signatures of the beginning and ending and CRC sum)
* \param 'buf' - pointer to buffer to which data will be written
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::load (void* buf, int size)
{
    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_load_0");
        return false;
    }

    // Setting position in file
    if (_lseek (fd, 0, SEEK_SET) == -1)
    {
        Log.errorPrint("StorageW32_load_1 [", errno, "]");
        return false;
    }

    // Reading of the beginning signature
    unsigned int bSig=0, eSig=0;
    int cnt = _read (fd, &bSig, sizeof(bSig));
    if (bSig != BEG_SIGNATURE)
    {
        Log.errorPrintf("StorageW32_load_2 [%d]", errno);
        return false;
    }

    // Reading from file
    cnt = _read (fd, buf, size);
    if (cnt != size)
    {
        Log.errorPrintf("StorageW32_load_3 [%d]", errno);
        return false;
    }

    // Reading signature of the end
    cnt = _read (fd, &eSig, sizeof(eSig));
    if (eSig != END_SIGNATURE)
    {
        Log.errorPrintf("StorageW32_load_4 [%d]", errno);
        return false;
    }

    // Reading a CRC sum
    INT16U readCrc=0;
    cnt = _read (fd, &readCrc, sizeof(readCrc));
    if (cnt != sizeof(readCrc))
    {
        Log.errorPrintf("StorageW32_load_5 [%d]", errno);
        return false;
    }

	// Calculation of a CRC and comparison with readed one
    INT16U computedCrc = Crc::computeBin (static_cast<const unsigned char*>(buf), size);
    if (readCrc != computedCrc)
    {
        Log.errorPrintf("StorageW32_load_3 [rd=%d, comp=%d]", readCrc, computedCrc);
        return false;
    }

    return true;
}

/** \name Method reads any data
* \param 'buf' - pointer to buffer to which data will be written
* \param 'size' - data size in bytes
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::read (void* buf, int size, int offset)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_read_1");
        return false;
    }

    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_read_2");
        return false;
    }

    // Setting of a position in file
    if (_lseek (fd, offset, SEEK_SET) == -1)
    {
        Log.errorPrint("StorageW32_read_3 [", errno, "]");
        return false;
    }

    // Reading from a file
    int cnt = _read (fd, buf, size);
    if (cnt == -1)
    {
        Log.errorPrint("StorageW32_read_4 [", errno, "]");
        return false;
    }

    return true;
}

/** \name Method appends a line of text to the file. CRLF is automatically appended.
* \param 'buf' - pointer to data which will be appended (text line)
* \param 'suppressErrMsg' - if 'true' no error messages are displayed (to avoid recurrence)
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::append (const char* buf, bool suppressErrMsg)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_append_0");
        return false;
    }

    if (!_async)
    {
        if (!suppressErrMsg)
            Log.errorPrint("StorageW32_append_1", true);
        return false;
    }

    if (!fileOpened)
    {
        if (!suppressErrMsg)
            Log.errorPrint("StorageW32_append_2", true);
        return false;
    }

    if (!_vSem.lock (suppressErrMsg))
        return false;

    // Setting a position in a file
    if (fseek (_f, 0, SEEK_END) != 0)
    {
        if (!suppressErrMsg)
            Log.errorPrint("StorageW32_append_3 [", errno, "]", true);
        _vSem.unlock (suppressErrMsg);
        return false;
    }

    // Writting to the file
    if (fwrite (buf, 1, strlen(buf), _f) <= 0)
    {
        if (!suppressErrMsg)
            Log.errorPrint("StorageW32_append_4 [", errno, "]", true);
        _vSem.unlock (suppressErrMsg);
        return false;
    }
    if (fwrite (CRLF, 1, strlen(CRLF), _f) <= 0)
    {
        if (!suppressErrMsg)
            Log.errorPrint("StorageW32_append_5 [", errno, "]", true);
        _vSem.unlock (suppressErrMsg);
        return false;
    }

	// Clearing the buffer - it is needed because the program can terminate unexpectedly
    fflush (_f);

    if (!_vSem.unlock (suppressErrMsg))
        return false;

    return true;
}

/** \name Method finds first occurrence of a text pattern
* \param 'text' - text pattern
* \param 'offset' - offset for searching in a file. Offset of found pattern is returned (by a reference)
* \param 'maxCount', 'stopText' - not used
* \return 'true' if pattern has been found and offset of the pattern has been set. 'false' if returned offset is equal to a file size.
*/
bool StorageW32::scan (const char* text, int &offset, int maxCount, const char* stopText)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_scan_1");
        return false;
    }

	// Function is ignored when 'stopText' is set
    if (stopText != NULL)
        return false;

    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_scan_2");
        return false;
    }

    int buf;
    int len = strlen(text);
    int i = 0;
    int nBytes = 0;
    bool ret = false;

   // Setting position in file
    if (fseek (_fro, offset, SEEK_SET) != 0)
    {
        Log.errorPrint("StorageW32_scan_3 [", errno, "]");
        return false;
    }

	// Searching the file char by char
    while ((buf = fgetc(_fro)) != EOF)
    {
		// Searching is a long operation. Other tasks of the uC/OS-II system has higher priority but whole process holds other
		// Windows proicesses e.g. FlighGear. Here we give control of every certain number of bytes.
        if (++nBytes % 10000 == 0)
            Os->sleepMs(10);

        if (i < len && static_cast<char>(buf) == text[i])
        {
            // In the pattern
            i++;

            if (i == len)
            {
                // Pattern has been found
                offset = ftell(_fro) - i;
                ret = true;
                break;
            }

            continue;
        }

        if (i > 0)
        {
			// Char doesn't fit to pattern (fall back position in the file)
            fseek(_fro, -i, SEEK_CUR);
            i = 0;
        }
    }

	// When end of file is reached then offset value is equal to the file size
    if (!ret)
        offset = ftell(_fro);

    return ret;
}

/** \name Method reads a text line from file. Method doesn't copy end line characters.
* \param 'buf' - buffer for read data
* \param 'bufSize' - buffer size
*\ \param 'offset' - offset for searching in a file. Returning is offset from the beginning of a new line.
* \return 'true' if line has been read. 'fasle' - if no data has been read (end of file)
*/
bool StorageW32::readLine (char* buf, int bufSize, int &offset)
{
    bool ret = false;

    if (bufSize < 1)
    {
        Log.errorPrintf("StorageW32_readLine_0");
        return false;
    }

    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_readLine_1");
        return false;
    }

    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_readLine_2");
        return false;
    }

    // Setting position in file
    if (fseek (_fro, offset, SEEK_SET) == -1)
    {
        Log.errorPrint("StorageW32_readLine_3 [", errno, "]");
        return false;
    }
    
    // Fill the entire buffer
    int count = fread(buf, 1, bufSize-1, _fro);
    if (count > bufSize-1)  //  na wszelki wypadek (wskazane przez BugDetective)
        count = bufSize-1;
    //  Setting '0' at the end (because there wasn't '0' in the file)
    buf[bufSize-1] = 0;

    if (count > 0)
    {
		// Finding the position of CR character
        char* crPos = static_cast<char*>(memchr (buf, '\r', count));
        if (crPos != NULL)
        {
			// Writting a '0'
            *crPos = 0;
			// Setting of a new offset (lines ends with CRLF but may be subtracted)
            offset += (crPos - buf + 2);
        }
        else
        {
            // End of file
            buf[count] = 0;
            offset += count;
        }
        ret = true;
    }

	return ret;
}

/** \name Method reduces file size to '0' (clears the file)
* \return 'true' on success, 'false' otherwise
*/
bool StorageW32::clear (bool fastClear)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_clear_1");
        return false;
    }

    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_clear_2");
        return false;
    }

    if (!_vSem.lock ())
        return false;

    if (fflush (_f) != 0)
    {
        Log.errorPrint("StorageW32_clear_3 [", errno, "]");
        _vSem.unlock ();
        return false;
    }

    rewind(_f);

    if (_chsize (fd, 0) == -1)
    {
        Log.errorPrint("StorageW32_clear_4 [", errno, "]");
        _vSem.unlock ();
        return false;
    }

	// Writting of a end line character, to make this session recognizable
    fwrite(CRLF, 1, 2, _f);

    rewind (_fro);

    if (!_vSem.unlock ())
        return false;

    return true;
}

/** \name Method returns size of the file
* \return file size in bytes or '-1' when an error occured
*/
int StorageW32::getSize (void)
{
    if (_singleThread)
    {
        Log.errorPrintf("StorageW32_getSize_1");
        return -1;
    }

    if (!fileOpened)
    {
        Log.errorPrintf("StorageW32_getSize_2");
        return -1;
    }

    long l = _filelength (fd);
    if (l == -1)
        Log.errorPrint("StorageW32_getSize_3 [", errno, "]");

    return static_cast<int>(l);
}

/** \name Method handles an uC/OS-II operating system tasks related to an 'update' function
*/
void StorageW32::task(void* pdata)
{
    // Dummy method - processed synchronously in W32
	Os->taskSuspend();
}

/** \name Method is used temporarily to simulate programming an autopilot by fastRS
*/
bool StorageW32::writeFlash(const void* buf, unsigned int offset, int size)
{
    return true;
}

bool StorageW32::readFlash(void* buf, unsigned int offset, int size)
{
	return true;
}

/** \name Dummy method used by the flash programmer
*/
bool StorageW32::calcFlashCRC(unsigned int offset, int size, INT16U& crc) 
{
    // CRC value has been already calculated so returned value is always correct ('true')
	return true;
}

bool StorageW32::mkdir(std::string path)
{
	return true;
}

bool StorageW32::isDirectory(std::string path) {
	return true;
} // isDirectory

std::vector<std::string> StorageW32::pathSplit(std::string path) {
	std::vector<std::string> ret;
	return ret;
} // pathSplit

std::vector<std::string> StorageW32::listFile(void)
{
	std::vector<std::string> ret;
	return ret;
}

bool StorageW32::renameFile(const char* oldname, const char* newname){
	return true;
}

bool StorageW32::renameFile(std::string oldname, std::string newname){
	return true;
}

bool StorageW32::check_file(const char *fname)
{
	return true;
}

bool StorageW32::remove(const char *fname)
{
	return true;
}

bool StorageW32::remove(std::string path)
{
	return true;
}

bool StorageW32::hasSuffix(const std::string& s, const std::string& suffix)
{
	return true;
}

bool StorageW32::contain(const std::string& s, const std::string& subs)
{
	return true;
}

bool StorageW32::copy(std::string strFrom, std::string strTo)
{
	return true;
}

bool StorageW32::formatSD()
{
	return true;
}

bool StorageW32::UploadFile(ClassifiedLine& cl, std::string nameFile)
{
	return true;
}

bool StorageW32::eraseFlash(unsigned int offset, int size)
{
	return true;
}

bool StorageW32::appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg)
{
	return true;
}

#endif  //  PILOT_TARGET
