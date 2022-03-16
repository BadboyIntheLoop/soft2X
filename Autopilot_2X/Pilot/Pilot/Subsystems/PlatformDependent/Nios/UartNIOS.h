#ifndef UARTNIOS_H
#define UARTNIOS_H


#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_HARDWARE

/** \file
* \brief Declaration of a class supporting UART port in NIOS-II system
*/

/** Class supports UART and JTAG_UART ports in NIOS-II system
* Sent text lines are written to the circular buffer, from where dedicated uC/OS-II system task
* gets them and sends to the serial port. Received lines are stored in the working buffer. After
* reading the end line character working buffer is switched to output buffer and notification
* is send.
*/
/// Implementation of a class supporting UART port in the NIOS-II system
class UartNIOS: public SerialDeviceBase
{
public:
    UartNIOS(CPU_INT32U portID, int baudRate = 0, int outBufLines = 10, bool enable = true);
    bool getLastLine(char* lineReceived);
    bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false,
        bool withCrc = false);
    bool sendBinary (const unsigned char* buf, int size);
    void clearInput (void);
    void taskIn(void* pdata);	///< Method handles an uC/OS-II operating system task
    void taskOut(void* pdata);	///< Method handles uC/OS-II operating system task sending line of a text
    
private:
    static const int ERRWAIT = 10;	///< Waiting time after occurrence of an error [ms] (protection against clogging of the error log)
    static const int SEMWAITTICKS = 5;	///< Waiting time (in cycles) for a semaphore (protection against deadlock)
    static const int MAX_WAIT_QUEUE = 30;	///< Queue length beyond which activate waiting for a send (if 'wait' == true)
    static const int MAX_SEND_TRIALS = 8;	///< Number of tries of sending a line (if 'wait' == true)
    static const int SEND_WAIT_TICKS = 5;	///< Waiting time between attempts to send a line (in system ticks)

    typedef char TLine[LINESIZE];

    // Destructor is disabled - object should never be destroyed
    ~UartNIOS(void){};

    void open(CPU_INT32U portID, int baudRate, int outBufLines);

    TLine buf1;             ///< Working or output buffer (switchable)
    TLine buf2;             ///< Working or output buffer (switchable)
    char* work;             ///< Pointer to a working buffer for received characters
    char* lastLine;         ///< Pointer to an output buffer with received line
    OS_EVENT* mboxSig;      ///< Mailbox to synchronize the sending task
    Semaphore _outBufSem;   ///< Semaphore controlling access to the cyclic buffer (for sending)
    Semaphore _inBufSem;    ///< Semaphore controlling access to the output buffer (for receiving)
    bool _clearInputFlag;   ///< The requests flag to clear the input buffer
    FILE* fp;               ///< File associated with UART
    int fd;                 ///< UART file descriptor
    int nLines;             ///< Number of lines of a cyclic buffer
    TLine* outLinesBuffer;  ///< Cyclic buffer with lines to send
    int lUser, lSystem;     ///< Positions in a cyclic buffer
    int _maxWaitQueue;      ///< Queue length beyond which activates waiting for a send (if 'wait' == true)
    AXIUARTLITE_HANDLE file;
    bool isConfigUartHandler;
    CPU_INT32U portIDUART;
     
};

#endif // PILOT_TARGET
#endif // UARTNIOS_H
