/**
*                                                                   
* @class IRQ                                                        
*                                                                   
* @brief Interrupt handler procedures and registering functions.                
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>

#if PILOT_TARGET == PT_HARDWARE
/************************************************************************************/
/*                                                                                  */
/*                                  NIOS-II (hardware)                              */
/*                                                                                  */
/************************************************************************************/

/**
* Interrupt handler from AD converter procedure (accelerometers, gyroscopes)
* To be visible outside this file procedure is static.
*/
static void handle_adc_interrupts(void* context, u32 id)
{
    // PState must be initialized and multitasking must be turned on
    if (PState != NULL && OSRunning == OS_TRUE)
    { 
        PState->notify (IRQ_ADC);
        return;
    }
}

static void handle_fastrs_interrupts(void* context, u32 id)
{
    // IOWR_16DIRECT (FLYEYE_FASTRS_BASE, 0x04, '*');  // DEBUG

    if (Tty2 != NULL && OSRunning == OS_TRUE)
    { 
        Tty2->notify (IRQ_FAST_RS);
        return;
    }
}

void imu_handle_int()
{
    // PState must be initialized and multitasking must be turned on
    if (PState != NULL && OSRunning == OS_TRUE)
    {
    	s16 imu_buf[7];
    	VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[0] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[1] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[2] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[3] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[4] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[5] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
    	imu_buf[6] = VIETTEL_IMU_CONTROLLER_mReadReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);


        PState->notify (IRQ_ADC);
        VIETTEL_IMU_CONTROLLER_mWriteReg(IMU_SPI_0_BASE, VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG3_OFFSET, 0);
        return;
    }

}

void fastrs_handle_int()
{
	//reset interrupt
	VIETTEL_FASTRS_mWriteReg(FLYEYE_FASTRS_BASE, 0x08, 0);
    if (Tty2 != NULL && OSRunning == OS_TRUE)
    {
        Tty2->notify (IRQ_FAST_RS);
        return;
    }
}
#if USE_CAM == CAM_ENABLE
void cam_handle_int()
{
	//reset interrupt
	VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, 0x08, 0);
    if (TtyCAM != NULL && OSRunning == OS_TRUE)
    {
    	TtyCAM->notify (IRQ_CAM);
        return;
    }
}
#endif

#if MAGNETOMETER_TYPE == USE_HMR
void hmr_handle_int()
{
	//reset interrupt
	HMR_mWriteReg(FLYEYE_HMR_BASE, 0x08, 0);
    if (Hmr != NULL && OSRunning == OS_TRUE)
    {
    	Hmr->notify (IRQ_HMR_RS);
        return;
    }
}
#endif


/**
* Interrupt handling initialization
* Call in main() function.
*/
void initIrq()
{
//	UCOS_IntSrcEn(ADIS_INT_ID);
//	UCOS_IntTypeSet(ADIS_INT_ID, UCOS_INT_TYPE_LEVEL);
//	UCOS_IntVectSet(ADIS_INT_ID, 0xa0, 0, (UCOS_INT_FNCT_PTR)imu_handle_int , 0);
    // Register interrupt handler from FAST_RS procedure (is line to be read)
    VIETTEL_FASTRS_mWriteReg(FLYEYE_FASTRS_BASE, 0x08, 0);
	UCOS_IntSrcEn(FASTRS_INT_ID);
	UCOS_IntTypeSet(FASTRS_INT_ID, UCOS_INT_TYPE_LEVEL);
	UCOS_IntVectSet(FASTRS_INT_ID, 0xa4, 0, (UCOS_INT_FNCT_PTR)fastrs_handle_int , 0);

#if USE_CAM == CAM_ENABLE
    VIETTEL_FASTRS_mWriteReg(FLYEYE_CAM_BASE, 0x08, 0);
	UCOS_IntSrcEn(CAM_INT_ID);
	UCOS_IntTypeSet(CAM_INT_ID, UCOS_INT_TYPE_LEVEL);
	UCOS_IntVectSet(CAM_INT_ID, 0xa5, 0, (UCOS_INT_FNCT_PTR)cam_handle_int , 0);
#endif

#if MAGNETOMETER_TYPE == USE_HMR
	HMR_mWriteReg(FLYEYE_HMR_BASE, 0x08, 0);
	UCOS_IntSrcEn(RS485HMR_INT_ID);
	UCOS_IntTypeSet(RS485HMR_INT_ID, UCOS_INT_TYPE_LEVEL);
	UCOS_IntVectSet(RS485HMR_INT_ID, 0xa7, 0, (UCOS_INT_FNCT_PTR)hmr_handle_int , 0);
#endif

}


#elif PILOT_TARGET == PT_WIN32
/************************************************************************************/
/*                                                                                  */
/*                                  Windows                                         */
/*                                                                                  */
/************************************************************************************/


//  fake plug
void initIrq() {};

#else  // PILOT_TARGET
/************************************************************************************/
/*                                                                                  */
/*                                  Error - undefined platformtforma                */
/*                                                                                  */
/************************************************************************************/
    #error "PILOT_TARGET unknown (IRQ.cpp)"

#endif //PILOT_TARGET == PT_DEVKIT
