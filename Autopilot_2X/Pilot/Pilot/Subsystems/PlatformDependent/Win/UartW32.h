#ifndef UARTW32_H
#define UARTW32_H

// Includes declaration of a PILOT_TARGET platform

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

/** \file
* \brief Declaration of a class supporting serial port in Windows OS
*/

/** Class supports serial port in Windows OS
*/
/// Implementation of a class supporting serial post in Windows OS
class UartW32: public SerialDeviceBase
{
public:
    static const int ERRWAIT_MS = 10000;    ///< Waiting time after occurrence of an error [ms] (protection against clogging of the error log)

    UartW32(LPCSTR portName, DWORD baudRate);
    bool getLastLine(char* lineReceived);
    bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false,
        bool withCrc = false);
    void taskIn(void* pdata);
    void taskOut(void* pdata);

private:
    static const DWORD INBUFSIZE = 64000;
    static const DWORD OUTBUFSIZE = 64000;

    // Destructor is disabled - object should never be destroyed
    ~UartW32(void){};

    void open(LPCSTR portName, DWORD baudRate); ///< Method performs opening of the serial port

    char buf1[LINESIZE];
    char buf2[LINESIZE];
    char* work;
    char* lastLine;
    HANDLE h;
    DCB dcb;
    static DWORD WINAPI threadFunction( LPVOID pParam ) ;
    COMMTIMEOUTS ct;
    OSBase::SemHandle _mbsem;
    HANDLE hMutex;
    HANDLE thr;
};

#endif // PILOT_TARGET == PT_WIN32
#endif // UARTW32_H
