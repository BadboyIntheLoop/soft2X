#include <PilotIncludes.h>

#if PILOT_TARGET == PT_HARDWARE
/************************************************************************************/
/*                                                                                  */
/*                                  NIOS-II (hardware)                              */
/*                                                                                  */
/************************************************************************************/


void PlatformLayer::setMotorBrake (bool enable)
{
	return;
}

void PlatformLayer::setBlueTooth (bool enable)
{
}

/** \name Method performs system reboot (for NIOS the processor and devices are reset).
* Note: In NIOS the 'exit()' function has infinite loop!
*/
void PlatformLayer::reboot (void)
{
    fflush (stdout);

	QSPI_reset();
	//Unlock SLCR Register
	Xil_Out32(XPS_SYS_CTRL_BASEADDR + 0x008, 0xDF0D);
	//Perform PSS Reset (System Software Reset)
	Xil_Out32(XPS_SYS_CTRL_BASEADDR + 0x200, 0x1);

}

/** \name Method returns start address of a servo control registers.
*/
ServoRegs* PlatformLayer::getServoRegs (void)
{
    return (ServoRegs*)(FLYEYE_SERVO_BASE | MASK_NIOS_DISABLE_CACHE);
}

/** \name Method returns start address of a registers used to reading state of the RC radio.
*/
RCinRegs* PlatformLayer::getRCinRegs (void)
{
    return (RCinRegs*)(0u | MASK_NIOS_DISABLE_CACHE);
}

/** \name Method performs active waiting. Time in milliseconds.
*/
void PlatformLayer::SCPDelay(int time)
{
    long int i;

    for(i=0;i<time*10000;i++)
    {
        asm("nop;");     
        asm("nop;");
        asm("nop;");
        asm("nop;");
        asm("nop;");
        asm("nop;");
        asm("nop;");
        asm("nop;");
        asm("nop;");
        asm("nop;");       
    }
}

/** \name Method creates objects handling operating system functions.
*/
OSBase* PlatformLayer::OSPrepare(void)
{
    return new OSUcosII();
}

/** \name Method initializes sensors whose readings are needed to determine whether the plane is in the air before multitasking starts.
*/
void PlatformLayer::initSensors1 (void)
{
    Gauge::initGauge();
}

/** \name Method creates communication objects Gps, Tty0, Tty1, Tty2, Ctty485 for specified hardware platform.
*/
void PlatformLayer::initTty (SerialDeviceBase*& pGps, SerialDeviceBase*& pTty0, SerialDeviceBase*& pTty1,
    SerialDeviceBase*& pTty2, CommChannel*& pCTty485)
{
	//    Log.bootPrint ("  - Gps \t ");
		// Gps channel is disabled by default - enabled after its restart
	    pGps = new UartNIOS (XPAR_UART_GPS_DEVICE_ID, 0, 5, false);

	//    Log.bootPrint ("  - Tty1\t ");
	    pTty1 = new UartNIOS (XPAR_UART_TELEMETRY_DEVICE_ID, 0, 295);

	//    Log.bootPrint ("  - Tty2\t ");

	    pTty2 = new FastRS(XPAR_VIETTEL_FASTRS_0_DEVICE_ID, 921600, 295);
}



/** \name Method creates communication objects Gps, Tty0, Tty1, Tty2, Ctty485 for specified hardware platform.
*/
void PlatformLayer::initTty (SerialDeviceBase*& pGps,SerialDeviceBase*& pHmr, SerialDeviceBase*& pTty0, SerialDeviceBase*& pTty1,
    SerialDeviceBase*& pTty2, CommChannel*& pCTty485)
{
//    Log.bootPrint ("  - Gps \t ");
	// Gps channel is disabled by default - enabled after its restart
    pGps = new UartNIOS (XPAR_UART_GPS_DEVICE_ID, 0, 5, false);

#if MAGNETOMETER_TYPE == USE_HMR
	// Hmr
    pHmr = new Rs485HMR (0, 115200, 295);
#endif

//    Log.bootPrint ("  - Tty1\t ");
    pTty1 = new UartNIOS (XPAR_UART_TELEMETRY_DEVICE_ID, 0, 295);

//    Log.bootPrint ("  - Tty2\t ");
    pTty2 = new FastRS(XPAR_VIETTEL_FASTRS_0_DEVICE_ID, 921600, 295);

//    pTtyCAM = new UartCAM(XPAR_UART_CAM_DEVICE_ID, 0, 295);

}

int PlatformLayer::QSPI_reset()
{
	XQspiPs_Config *QspiConfig;
	int Status;
	XQspiPs QspiInstance;

	u8 WriteBfrPtr_tmp[(PAGE_COUNT * MAX_PAGE_SIZE) + DATA_OFFSET];

	QspiConfig = XQspiPs_LookupConfig(XPAR_XQSPIPS_0_DEVICE_ID);
		if (NULL == QspiConfig) {
			xil_printf("Cannot find QSPI Flash Device\n\r");
			return XST_FAILURE;
		}

		Status = XQspiPs_CfgInitialize(&QspiInstance, QspiConfig,
						QspiConfig->BaseAddress);
		if (Status != XST_SUCCESS) {
			xil_printf("Error : Config Initialize QSPI\n\r");
			return XST_FAILURE;
		}

	/*
		* Set the pre-scaler for QSPI clock
	 */
		XQspiPs_SetClkPrescaler(&QspiInstance, XQSPIPS_CLK_PRESCALE_8);

	/*
		* Set Manual Start and Manual Chip select options and drive the
		* HOLD_B high.
	*/
		XQspiPs_SetOptions(&QspiInstance, XQSPIPS_FORCE_SSELECT_OPTION |
							     XQSPIPS_MANUAL_START_OPTION |
							     XQSPIPS_HOLD_B_DRIVE_OPTION);
		if(QspiConfig->ConnectionMode == XQSPIPS_CONNECTION_MODE_STACKED) {
		/*
		 	 * Enable two flash memories, Shared bus (NOT separate bus),
			 * L_PAGE selected by default
		*/
			XQspiPs_SetLqspiConfigReg(&QspiInstance, DUAL_STACK_CONFIG_WRITE);
		}

		if(QspiConfig->ConnectionMode == XQSPIPS_CONNECTION_MODE_PARALLEL) {
		/*
			* Enable two flash memories on separate buses
		*/
			XQspiPs_SetLqspiConfigReg(&QspiInstance, DUAL_QSPI_CONFIG_WRITE);
		}

		/*
			* Assert the Flash chip select.
		*/
		XQspiPs_SetSlaveSelect(&QspiInstance);

		WriteBfrPtr_tmp[COMMAND_OFFSET]   = BANK_REG_WR;
		WriteBfrPtr_tmp[ADDRESS_1_OFFSET] = 0x00;
	/*
		* Send the Extended address register write command
		* written, no receive buffer required
	*/
		XQspiPs_PolledTransfer(&QspiInstance, WriteBfrPtr_tmp, NULL,BANK_SEL_SIZE);

		return XST_SUCCESS;
}


/** \name Method initializes hardware platform - called at the beginning
*/
void PlatformLayer::initPlatform (void)
{
	// Method is not used on NIOS platform - environment initializes hardware before call of 'main()' function
}

#elif PILOT_TARGET == PT_WIN32
/************************************************************************************/
/*                                                                                  */
/*                                  Windows                                         */
/*                                                                                  */
/************************************************************************************/

/* parasoft on */
// Enable static analysis

// Simulation of a registers related to the control servos and reading the RC radio
ServoRegs PlatformLayer::_srDummy = {0};
RCinRegs  PlatformLayer::_rcirDummy = {0};


void PlatformLayer::setMotorBrake (bool enable)
{
    // Dummy method
}

void PlatformLayer::setBlueTooth (bool enable)
{
    // Dummy method
}

/** Method performs system reboot (in Windows it terminates application).
*/
void PlatformLayer::reboot (void)
{
    fflush (stdout);
	// 'abort' do not terminate the console immediately so there's a chance to read the error message
    abort ();
}

/** \name Method returns start address of a servo control registers.
*/
ServoRegs* PlatformLayer::getServoRegs (void)
{
    return &PlatformLayer::_srDummy;
}

/** \name Method returns start address of a registers used to reading state of the RC radio.
*/
RCinRegs* PlatformLayer::getRCinRegs (void)
{
    return &_rcirDummy;
}

/** \name Method performs active waiting. Time in milliseconds.
*/
void PlatformLayer::SCPDelay(int time)
{
    // Dummy method
}

/** \name Method creates objects handling operating system functions.
*/
OSBase* PlatformLayer::OSPrepare(void)
{
#if PILOT_OS == OS_WIN
    OSBase* b = new OSNativeW32();
#elif PILOT_OS == OS_UCOSII
    OSBase* b = new OSUcosII();
#else
    #error PILOT_OS not defined!
#endif

    return b;
}

/** \name Method initializes sensors whose readings are needed to determine whether the plane is in the air before multitasking starts.
*/
void PlatformLayer::initSensors1 (void)
{
    // Dummy method
}

/** \name Method creates communication objects Gps, Tty0, Tty1, Tty2, Ctty485 for specified hardware platform.
*/
void PlatformLayer::initTty (SerialDeviceBase*& pGps, SerialDeviceBase*& pTty0, SerialDeviceBase*& pTty1,
    SerialDeviceBase*& pTty2, CommChannel*& pCTty485)
{
    pGps = NULL;
	// Test of RS485 involves the turn off the normal operation console on and connected it as a RS485 device.
	// Console prints out values of a bytes in hex and and takes it also as a hex.
#ifdef WK_TEST_485
        Log.bootPrint ("  - Tty485 (C)\t ");
        ConsoleADWin* devConsole = new ConsoleADWin (true);
        pCTty485 = new CommChannel(*devConsole, 2, 2, true, "Tty485");

#else   // WK_TEST_485
    #ifdef WK_TEST_CONSOLE
        Log.bootPrint ("  - Tty0 (C)\t ");
        ConsoleADWin* devConsole = new ConsoleADWin (false);
        pTty0 = new CommChannel(*devConsole, 295, 2, true, "Tty0");
    #else
        Log.bootPrint ("  - Tty0\t ");
        pTty0 = new ConsoleW32();
    #endif
#endif  // WK_TEST_485

    Log.bootPrint ("  - Tty1\t ");
    pTty1 = new UdpW32(5512, 5513, "127.0.0.1");

    pTty2 = NULL;

}
/** \name Method creates communication objects Gps, Tty0, Tty1, Tty2, Ctty485 for specified hardware platform.
*/
void PlatformLayer::initTty(SerialDeviceBase*& pGps, SerialDeviceBase*& pHmr, SerialDeviceBase*& pTty0, SerialDeviceBase*& pTty1,
	SerialDeviceBase*& pTty2, CommChannel*& pCTty485)
{
    pGps = NULL;
    // Test of RS485 involves the turn off the normal operation console on and connected it as a RS485 device.
    // Console prints out values of a bytes in hex and and takes it also as a hex.
#ifdef WK_TEST_485
        Log.bootPrint ("  - Tty485 (C)\t ");
        ConsoleADWin* devConsole = new ConsoleADWin (true);
        pCTty485 = new CommChannel(*devConsole, 2, 2, true, "Tty485");

#else   // WK_TEST_485
    #ifdef WK_TEST_CONSOLE
        Log.bootPrint ("  - Tty0 (C)\t ");
        ConsoleADWin* devConsole = new ConsoleADWin (false);
        pTty0 = new CommChannel(*devConsole, 295, 2, true, "Tty0");
    #else
        Log.bootPrint ("  - Tty0\t ");
        pTty0 = new ConsoleW32();
    #endif
#endif  // WK_TEST_485

    Log.bootPrint ("  - Tty1\t ");
    pTty1 = new UdpW32(5512, 5513, "127.0.0.1");

    pTty2 = NULL;

}


/** \name Method initializes hardware platform - called at the beginning
*/
void PlatformLayer::initPlatform (void)
{
	// On Win32 platform method is not used
}

#else  // PILOT_TARGET
/************************************************************************************/
/*                                                                                  */
/*                                  Error - undefined platform                      */
/*                                                                                  */
/************************************************************************************/
    #error "PILOT_TARGET unknown (PlatformLayer.cpp)"

#endif //PILOT_TARGET == PT_DEVKIT
