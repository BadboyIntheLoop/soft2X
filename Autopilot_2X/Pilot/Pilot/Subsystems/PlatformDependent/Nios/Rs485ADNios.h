#ifndef RS485ADNIOS_H
#define RS485ADNIOS_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Compilation on Windows only for a static analysis purpose
#if (PILOT_TARGET == PT_HARDWARE) || (PILOT_TARGET == PT_WIN32)

#if OS_FLAGS_NBITS != 32
    #error "Define OS_FLAGS_NBITS 32 in os_cfg.h"
#endif

/** \file
* \brief Declaration of a class supporting an RS485 interface on a Altera NIOS platform
*/

/** Class supports an RS485 interface on a Altera NIOS platform. Class provides functionality to
* for sending and receiving with waiting a binary data. Class is arranged to cooperate with the CommChannel class.
* Class uses mechanisms of a uC/OS-II system.
*/
/// Implementation of a class supporting an RS485 interface on a Altera NIOS platform
class Rs485ADNios: public IAsyncDevice
{
public:
    Rs485ADNios (u32 baseAddr, u32 irqNo, u32 bdRate);
    virtual bool sendBufferWait (const unsigned char* buf, int nBytes);
    virtual bool recvBufferWait (unsigned char* buf, int bufSize, int& nBytes);
    bool isUsable (void) const;

    static void intHandler (void* context, u32 id);

protected:
    // Destructor is disabled - object should never be destroyed
    virtual ~Rs485ADNios(void){};

private:
    typedef union RS485Control_t
    {
        struct bits
        {
            u32 baud_div     :  5; // Baudrate divisor (0=460800, 1=230400, 3=115200, 7=57600, 11=38400, 23=19200, 47=9600)
            u32 module_enable:  1; // default: "0" = dzia?a starty modu? 485
            u32 echo_disable :  1; // default: "0" = dzia?a echo
            u32 tx_level     : 10; // Number of bytes in TX FIFO (max. 1023)
            u32 rx_level     : 10; // Number of bytes in RX FIFO (max. 1023)
            u32 dummy        :  3; // dummy
            u32 rxe          :  1; // RX empty flag
            u32 txe          :  1; // TX empty flag
        } bits;
        u32 word;
    }RS485Control_t;

    // Copy constructor is disabled
    Rs485ADNios (const Rs485ADNios&);

    // Copy operator is disabled
    Rs485ADNios& operator=(const Rs485ADNios&);

	/** Device registers relative to a base address
	* \{
	*/
    static const u32 REG_CTRL = 0u;
    static const u32 REG_CHAR = 4u;
    static const u32 REG_IRQ  = 8u;
	///\}

    static const OS_FLAGS EV_INT_RCV   = 1u;          ///< Event Flags bit mask indicating reading of a FIFO characters
    static const unsigned int MAX_INSTANCES =    8u;  ///< Maximum number of objects of this class
    static const unsigned int FL_BITS_INST  =    4u;  ///< Number of bits in a events flag reserved for a single instance
	static const u32 SEND_FIFO_SIZE     = 1024u;  ///< Size of a transmitter FIFO queue

    static unsigned int nInstances;                 ///< Number of class instances
    static OS_FLAG_GRP* evtFlagGrp;                 ///< Event flags for all devices of this class

    const u32 m_baseAddr;                       ///< The base address of the device instance
    const u32 m_irqNo;                          ///< Identifier of an interruption generated by a device
    const u32 m_bdRate;                         ///< Transmission speed [bit/s]
    u32 bdRateDivisor;                          ///< Value of a transmission speed stored in a registry
    unsigned int m_instanceNo;						///< Current instance identifier
    bool m_usable;                                  ///< flag determines whether the object is correctly created
};

#endif  // PILOT_TARGET
#endif  // RS485ADNIOS_H
