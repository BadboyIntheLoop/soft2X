#ifndef CONSOLEW32_H
#define CONSOLEW32_H

// Includes declaration of a PILOT_TARGET platform

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

/** \file
* \brief Declaration of a class supporting CRT console in a Windows operating system
*/

/** Class supports a CRT console in a Windows operating system
*/
/// Implementation of a class supporting CRT console in Windows operating system
class ConsoleW32: public SerialDeviceBase
{
public:
    ConsoleW32(void);
    bool getLastLine(char* lineReceived);
    bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false,
        bool withCrc = false);
    void taskIn(void* pdata);
    void taskOut(void* pdata);

private:
    static const char SUBS_PREFIX[];			///< Subsystem prefix
    static const int ERRWAIT_MS = 10000;		///< Waiting time after occurrence of an error [ms] (protection against clogging of the error log)
    static const INT8U PSEUDO_INT_VECTOR = 1;	///< Identifier of simulated vector of interrupts for uC/OS-II
    static const char PSEUDO_INT_NAME[];		///< Name of the simulated vector of interrupts for uC/OS-II

    void open(void);

    char buf1[LINESIZE];
    char buf2[LINESIZE];
    char* work;
    char* lastLine;
    HANDLE hMutex;
    HANDLE thr;
    Semaphore _vsem;

    // Destructor is disabled - object should never be destroyed
    ~ConsoleW32(void){};

    static DWORD WINAPI threadFunction( LPVOID pParam );	///< Main function of the Win32 thread which performs reading a line from console
    static void PseudoIsr1(void);	///< Method simulates handling of an uC/OS-II interrupt
    static OSBase::SemHandle _mbsem;
    static int instancesCount;	///< Class instances counter (to control, because it can be only one)
};

#endif // PILOT_TARGET
#endif // CONSOLEW32_H
