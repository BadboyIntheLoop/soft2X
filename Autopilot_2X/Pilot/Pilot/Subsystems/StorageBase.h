#ifndef STORAGEBASE_H
#define STORAGEBASE_H

/** \file
* \brief Declaration of a base class for storage specialized classes
*/

/** Virtual, base class for all specialized classes implementing the storage (files, flash, SDCard). Class declares virtual and pure virtual
* methods providing basic editing functionality like opening, closing, saveing, appending, reading. Class provides functionality supported
* by Win32 and NIOS-II.
*/
/// Base class for storage specialized classes
class StorageBase
{
public:
    virtual bool open (const char* fname) = 0;              ///< Opening the file method
    virtual bool close (void) = 0;                          ///< Closing the file method
    virtual bool save (const void* buf, int size) = 0;      ///< Writting input buffer 'buf' to file method
    virtual bool saveFile (const char* fileName, void* buf, int size);  ///< Opening, writting input buffer 'buf' and closing the file method
    virtual bool load (void* buf, int size) = 0;            ///< Reading buffer 'buf' from file with correctness checking method
    virtual bool loadFile (const char* fileName, void* buf, int size);  ///< Opening, writting input buffer 'buf' and closing file method
    /// Appending a line of text ('buf') to the log method
    virtual bool append (const char* buf, bool suppressErrMsg=false) = 0;
    /// Reading text with offset from log to buffer of size 'size' method
    virtual bool read (void* buf, int size, int offset) = 0;
    virtual bool clear (bool fastClear=true) = 0;           ///< Erasing file content method
    virtual int getSize (void) = 0;                         ///< Returning file size method
    virtual int getFree (void);                             ///< Method returns free log space in bytes
	/// Method searches for the first occurrence of the text pattern
    virtual bool scan (const char* text, int &offset, int maxCount=0, const char* stopText=NULL) = 0;
    /// Reading a single line of text method
    virtual bool readLine (char* buf, int bufSize, int &offset) = 0;
    /// Flash writting method (used by FlashProgrammer)
    virtual bool writeFlash(const void* buf, unsigned int offset, int size) = 0;
    virtual bool readFlash(void* buf, unsigned int offset, int size) = 0;
    virtual bool eraseFlash(unsigned int offset, int size) = 0;
    /// CRC calculation of a flash memory method (used by FlashProgrammer)
    virtual bool calcFlashCRC(unsigned int offset, int size, INT16U& crc) = 0;

    virtual bool mkdir(std::string path) = 0;
    virtual bool  isDirectory(std::string path) = 0;
    virtual std::vector<std::string> pathSplit(std::string path) = 0;
    virtual std::vector<std::string> listFile(void) = 0;
    virtual bool renameFile(const char* oldname, const char* newname) = 0;
    virtual bool renameFile(std::string oldname, std::string newname) = 0;
    virtual bool check_file(const char *fname) = 0;
    virtual bool remove(const char *fname) = 0;
    virtual bool remove(std::string path) = 0;
    virtual bool hasSuffix(const std::string& s, const std::string& suffix) = 0;
    virtual bool contain(const std::string& s, const std::string& subs) = 0;
    virtual bool copy(std::string strFrom, std::string strTo) = 0;
    virtual bool formatSD(void) = 0;
    virtual bool UploadFile(ClassifiedLine& cl, std::string nameFile) = 0;

    virtual bool appendNew(const void* buf, const char* fileName, int size, bool suppressErrMsg=false) = 0;

	/// System tasks handler method
    virtual void task (void* pdata) = 0;
    
    /** \name Test methods
	* \{
	*/
    virtual bool saveTest(const char* fname);
    virtual bool appendTest(const char* fname);
	///\}

protected:
	/// Constructor is disabled.
    StorageBase(void);
	/// Class destructor is disabled - Parasoft-suppress OOP-31 "NIOS-2 environment reports a warning while destructor is not virtual"
    virtual ~StorageBase(void){};

private:
    /// Copy contructor is disabled
    StorageBase(StorageBase&);
	/// Assignment operator is disabled
    StorageBase& operator=(const StorageBase&);
};

#endif  // STORAGEBASE_H
