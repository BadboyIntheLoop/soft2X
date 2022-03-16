#ifndef CONSOLEADWIN_H
#define CONSOLEADWIN_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif


#if PILOT_TARGET == PT_WIN32

/** \file
* \brief Declatarion of a class supporting low level functions of the CRT console in Windows operating system
*/

/** Class supports low level functions of the CRT console in Windows operating system.
* Class is designed to cooperation with CommChannel class. Detailed description of an interface is in IAsyncDevice class.
*/
/// Implementation of a class supporting low level functions of the CRT console in Windows operating system
class ConsoleADWin: public IAsyncDevice
{
public:
    ConsoleADWin (bool diag);
    virtual bool sendBufferWait (const unsigned char* buf, int nBytes);
    virtual bool recvBufferWait (unsigned char* buf, int bufSize, int& nBytes);
    bool isUsable (void) const;

protected:
    // Destructor is disabled - object should never be destroyed
    virtual ~ConsoleADWin(void) {};

private:
    // Copy constructor is disabled
    ConsoleADWin(const ConsoleADWin&);

    static DWORD WINAPI threadFunction( LPVOID pParam );    ///< Main function of Win32 thread reading line from console
    static void pseudoIsr1(void);                           ///< Handling method of an imitating uC/OS-II interruption

    // Copy operator is disabled
    ConsoleADWin& operator=(const ConsoleADWin&);

    static const char SUBS_PREFIX[];            ///< Subsystem prefix
    static const int ERRWAIT_MS = 10000;        ///< Waiting time after occurrence of an error [ms] (protection against clogging of the error log)
    static const INT8U PSEUDO_INT_VECTOR = 1u;  ///< Identifier of simulated vector of interrupts for uC/OS-II
    static const char PSEUDO_INT_NAME[];        ///< Name of the simulated vector of interrupts for uC/OS-II
    static const int CON_BUFSIZE = 256;         ///< Maximum number of characters read from console

    static OSBase::SemHandle _mbsem;            ///< Semaphore controlling waiting in 'recvBufferWait' function (used atypically)
    static int instancesCount;                  ///< Class instances counter (to control, because it can be only one)

    char   m_buf1[CON_BUFSIZE]; ///< Data buffer read from console (1)
    char   m_buf2[CON_BUFSIZE]; ///< Data buffer read from console (2)
    char*  m_work;              ///< Pointer to buffer used currently to read data from console
	char*  m_lastLine;          ///< Pointer to buffer holds previously read data
    HANDLE m_hMutex;            ///< Windows mutex controlling access to the buffers and pointers
    HANDLE m_thr;               ///< Handle to the Windows thread reading console data
    bool   m_isUsable;          ///< Flag indicates the correctness of object creation
    bool   m_diag;              ///< Flag indicates use of diagnostic
};

#endif // PILOT_TARGET
#endif // CONSOLEADWIN_H
