/*
 * UartCAM.h
 *
 *  Created on: Oct 12, 2020
 *      Author: truongnt
 */

#ifndef UARTCAM_H
#define UARTCAM_H

#define CAM_INT_ID_OFFSET		5
#define CAM_INT_ID				XPAR_FABRIC_XLCONCAT_0_DOUT_INTR + CAM_INT_ID_OFFSET

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
class UartCAM: public ODTObserver, public SerialDeviceBase
{
public:
    UartCAM(CPU_INT32U portID, int baudRate = 0, int outBufLines = 10);
    bool getLastLine(char* lineReceived);
    bool sendLine (const char* lineToSend, bool suppressErrMsg = false, bool withWaiting = false,
        bool withCrc = false);
    bool sendBinary (const unsigned char* buf, int size);
    void clearInput (void);
    void taskIn(void* pdata);	///< Method handles an uC/OS-II operating system task
    void taskOut(void* pdata);	///< Method handles uC/OS-II operating system task sending line of a text
    void linkObserver();		///< Method links observed objects
    bool getObjectTrackInfor(cam_msg_response_t &msg);
#if USE_WSM == WSM_ENABLE
    bool getWsmData(wsm_msg_res_t &msg);
#endif

private:

#if USE_WSM == WSM_ENABLE
    static const uint8_t WSM_SYNC1 = 0x24;
    static const uint8_t WSM_SYNC2 = 0x40;
    static const uint8_t WSM_SYNC3 = 0x03;
#endif
    static const uint8_t CAM_SYNC1 = 0x24;
    static const uint8_t CAM_SYNC2 = 0x40;
    static const uint8_t CAM_SYNC3 = 0x04;

    typedef struct camMsgBuffer {
    	uint8_t mode;
    	int32_t px;
    	int32_t py;
    	int32_t width;
    	int32_t height;
    	float hfov;
    	float gimbalPan;
    	float gimbalTilt;
    	float gimbalRoll;
    	uint8_t reserved[31];
    }camMsgBuffer_t;
    typedef union camMsgData {
    	camMsgBuffer_t camMsg;
    	uint8_t bytes[64];
    }camMsgData_t;
    typedef struct cam_msg_parse
    {
        enum
        {
            SYNC1 = 0,
			SYNC2,
            SYNC3,
            DATALENGTH,
            DATA,
            CRC,
        } cam_state;
        camMsgData_t data;
        uint8_t crc;
        uint8_t dataLength;
        uint16_t read;
    }cam_msg_parse_t;

#if USE_WSM == WSM_ENABLE
    typedef struct wsmMsgBuffer {
    	int16_t		magX;
    	int16_t		magY;
    	int16_t		magZ;
    }wsmMsgBuffer_t;
    typedef union wsmMsgData {
    	wsmMsgBuffer_t wsmMsg;
    	uint8_t bytes[6];
    }wsmMsgData_t;
    typedef struct wsm_msg_parse
    {
        enum
        {
            SYNC1 = 0,
			SYNC2,
            SYNC3,
            DATALENGTH,
            DATA,
            CRC,
        } wsm_state;
    	wsmMsgData_t	data;
        uint8_t 		crc;
        uint8_t 		dataLength;
    	uint16_t 		read;
    }wsm_msg_parse_t;
#endif
    // Destructor is disabled - object should never be destroyed
    ~UartCAM(void){};
    static const int ERRWAIT = 10;			///< Waiting time in seconds after the occurrence of an error (protection against clogging the log)
    static const int MAX_WAIT_QUEUE = 115;	///< Value of a queue lenght above which waiting for a send is activated (when wait==true)
    static const int MAX_SEND_TRIALS = 8;	///< Number of tries to send a line (when wait==true)
    static const int SEND_WAIT_TICKS = 5;	///< Waiting time between the tries to send a line (ticks)
    static const int CAM_BUFFER_SIZE = 1024;	///< Length of an output buffer

    //
    static const int  CAM_DIVISOR_OFFSET 		= 0*4;
    static const int  CAM_WR_FIFO_OFFSET 		= 1*4;
    static const int  CAM_RESET_IRQ_OFFSET 		= 2*4;
    static const int  CAM_SET_IRQ_CHARACTER 	= 3*4;
    static const int  CAM_RD_CONTROL_OFFSET 	= 0*4;
    static const int  CAM_RD_FIFO_OFFSET 		= 1*4;

	/** \name Structure of an FastRS control register (0x00)
	* \{
	*/
    typedef union camControl_t{
        struct bits {
            u32 baud_div     :  7; ///< Baudrate divisor (0=460800, 1=230400, 3=115200, 7=57600, 11=38400, 23=19200, 47=9600)
            u32 tx_level     : 10; ///< Number of bytes in TX FIFO (max. 1023)
            u32 rx_level     : 10; ///< Number of bytes in RX FIFO (max. 1023)
            u32 dummy        :  3; ///< dummy
            u32 rxe          :  1; ///< RX empty flag
            u32 txe          :  1; ///< TX empty flag
        } bits;
        u32 word;
    }camControl_t;

    typedef char TLine[LINESIZE];

    TLine buf1;             ///< Place holder for a working buffer or an output buffer (switchable)
    TLine buf2;             ///< Place holder for a working buffer or an output buffer (switchable)
    char* work;             ///< Pointer to the working buffer which receive the characters
    char* lastLine;         ///< Pointer to the output buffer with received line
    OS_EVENT* mboxSig;      ///< Mailbox to synchronize the sending task
    Semaphore _outBufSem;   ///< Semaphore controlling access to the cyclic buffer (at transmitting)
    Semaphore _inBufSem;    ///< Semaphore controlling access to the output buffer (at receiving)
    Semaphore _vSem;        ///< Semaphore controlling access to the data buffer
    bool _clearInputFlag;   ///< Flag of an request to clear an output buffer
    int nLines;             /// Number of lines in a cyclic buffer
    TLine* outLinesBuffer;  ///< Cyclic buffer with lines to be send
    int lUser, lSystem;     ///< Positions in a cyclic buffer
    int _maxWaitQueue;      ///< Value of a queue lenght above which waiting for a send is activated (when wait==true)
    int _irqCamTag;      	///< Tag generated by a FastRS

    void open(const char* portName, int baudRate, int outBufLines);
    bool parse_cam(uint8_t temp);
    bool process_message_cam();
    uint8_t calcCRC(uint8_t * data, int length);

#if USE_WSM == WSM_ENABLE
    bool parse_wsm(uint8_t temp);
#endif


    cam_msg_parse_t	cam_msg;
    cam_msg_response_t cam_msg_res;
#if USE_WSM == WSM_ENABLE
    wsm_msg_parse_t		wsm_msg;
    wsm_msg_res_t		wsm_msg_res;
#endif
    uint32_t crc_counter;
    uint32_t crc_error_counter;

};

#endif // PILOT_TARGET
#endif /* UARTCAM_H */
