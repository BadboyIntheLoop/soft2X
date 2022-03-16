#ifndef UDPW32_H
#define UDPW32_H

// Includes declaration of a PILOT_TARGET platform

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

/** \file
* \brief Declaration of a class supporting UDP port in Windows OS
*/

/** Class supports UDP port in Windows OS
*/
/// Implementation of a class supporting UDP post in Windows OS
class UdpW32: public SerialDeviceBase
{
public:
    UdpW32(u_short inPort, u_short outPort, const char* outIP);
    bool getLastLine(char* lineReceived);	///< Method reads recently received line
    bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false,
        bool withCrc = false);	///< Method sends a line of text with line termination character
    void taskIn(void* pdata);	///< Method handles an uC/OS-II operating system task
    void taskOut(void* pdata);	///< Method handles uC/OS-II operating system task sending line of a text

private:
    static const int ERRWAIT_MS = 10000;		///< Waiting time after occurrence of an error [ms] (protection against clogging of the error log)
    static const INT8U PSEUDO_INT_VECTOR = 2;	///< Identifier of simulated vector of interrupts for uC/OS-II
    static const char PSEUDO_INT_NAME[];		///< Name of simulated vector of interrupts for uC/OS-II
    static const int UDP_BUFSIZE = LINESIZE;	///< Buffer size for UDP datagram

    void open(u_short inPort, u_short outPort, const char* outIP);

    char buf1[LINESIZE];
    char buf2[LINESIZE];
    char* work;
    char* lastLine;
    SOCKET recvSocket, sendSocket;
    sockaddr_in sendAddr;
    HANDLE hMutex;
    HANDLE thr;
	Semaphore _vsem;

    // Destructor is disabled - object shouldn't be destroyed
    ~UdpW32(void){};

    static DWORD WINAPI threadFunction( LPVOID pParam ) ; ///< Main function of the W32 thread which reads UDP data
    static void PseudoIsr2(void);	///< Method simulates handling of an uC/OS-II interrupt
    static OSBase::SemHandle _mbsem;
    static int instancesCount; ///< Class instances counter (to control, because it can be only one)
};

#endif // PILOT_TARGET
#endif // UDPW32_H
