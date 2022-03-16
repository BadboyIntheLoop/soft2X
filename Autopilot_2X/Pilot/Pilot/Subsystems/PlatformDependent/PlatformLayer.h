#ifndef PLATFORMLAYER_H
#define PLATFORMLAYER_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#ifndef PILOT_OS
    #error "Define PILOT_OS in pilot_cfg.h"
#endif

#define DATA_OFFSET		4 /* Start of Data for Read/Write */
/*
 * Number of flash pages to be written.
 */
#define PAGE_COUNT		32

/*
 * Max page size to initialize write and read buffer
 */
#define MAX_PAGE_SIZE 1024

// Forward declaration
class CommChannel;

// Declaration of types and function specific for a NIOS-II platform
// which are treated as dummies in Win32 environment it is used for the purpose of static analysis
#if (PILOT_TARGET == PT_WIN32)

/* parasoft off */
// Temporary disable static analysis

/* parasoft on */
// Enable static analysis

#endif  //PILOT_TARGET

// Declaration of types for autopilot operating system
#if (PILOT_TARGET == PT_WIN32) && (PILOT_OS == OS_WIN)
    typedef LPTHREAD_START_ROUTINE TaskProc;
    #define TSKPROC DWORD WINAPI
    #define TSKRET return 0
#else
    typedef void (*TaskProc)(void*);
    #define TSKPROC void
    #define TSKRET
#endif  //PILOT_TARGET, PILOT_OS

class OSBase;   // Forward declaration (class OSBase and PlatformLayer use each other)

/** \file
* \brief Declaration of a class with static functions associated with the hardware platform
*/

/** Class implements static member functions performing some actions related to the hardware platform
* such as e.g. read from, and write to the ports.
*/
/// Class implements a set of static member functions associated with the hardware platform
class PlatformLayer
{
public:
    static void setMotorBrake (bool enable);
    static void setBlueTooth (bool enable);
    static void reboot (void);	///< Method performs system reboot
    static ServoRegs* getServoRegs (void);	///< Method returns start address of a servo control registers
    static RCinRegs* getRCinRegs (void);	///< Method returns start address of a registers used to reading state of the RC radio
    static void SCPDelay (int time);	///< Method performs active waiting. Time in milliseconds
    static OSBase* OSPrepare (void);	///< Method creates objects handling operating system functions
    static void initSensors1 (void);	///< Method initializes sensors whose readings are needed to determine whether the plane is in the air before multitasking starts
    static void initTty (SerialDeviceBase*& pGps, SerialDeviceBase*& pTty0, SerialDeviceBase*& pTty1,
        SerialDeviceBase*& pTty2, CommChannel*& pCTty485);  ///< Method creates communication objects Gps, Tty0, Tty1, Tty2, Ctty485 for specified hardware platform
    static void initTty (SerialDeviceBase*& pGps,SerialDeviceBase*& pHmr, SerialDeviceBase*& pTty0, SerialDeviceBase*& pTty1,
        SerialDeviceBase*& pTty2, CommChannel*& pCTty485);	///< Method creates communication objects Gps, Tty0, Tty1, Tty2, Ctty485 for specified hardware platform
    static void initPlatform (void);	///< Method initializes hardware platform

#if PILOT_TARGET == PT_HARDWARE
    static int QSPI_reset();
#endif
private:
    PlatformLayer(void);

    static const INT32U MASK_NIOS_DISABLE_CACHE = 0x80000000;

#if (PILOT_TARGET == PT_WIN32)
	// Simulation of a registers related to the control servos and reading the RC radio
    static ServoRegs _srDummy;
    static RCinRegs  _rcirDummy;
#endif  //PILOT_TARGET
};

#endif  // PLATFORMLAYER_H
