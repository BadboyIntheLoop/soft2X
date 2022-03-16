// Declarations required to compile NIOS sources under Windows
// Included as "include" to all files

#ifndef NIOSDUMMYDECL_H
#define NIOSDUMMYDECL_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

#if PILOT_TARGET == PT_WIN32

/* parasoft off */
// Temporarily disabling static analysis

#define FLYEYE_RESET_STATUS_BASE 0
#define FLYEYE_ADC_BASE 0
#define FLYEYE_CAMERA_BASE 0
#define FLYEYE_SCP_BASE 0
#define FLYEYE_MAGNET_BASE 0
#define FLYEYE_AGL_BASE 0
#define GYRO_TEST_BASE 0
#define SPI_SCP1_BASE 0
#define SPI_SCP2_BASE 0
#define SPI_ACCU_BASE 0
#define CAMERA_ON_BASE 0
#define PIC_NCIC_MASK_BASE 0
#define FLYEYE_RC_IN_BASE 0
#define FLYEYE_RC_BASE 0
#define ALT_CPU_FREQ 96000000
#define IMU_SPI_0_BASE 0
#define FLYEYE_HMR_BASE 0
#define FLYEYE_MMC_BASE 0
#define FLYEYE_SERVO_BASE 0
#define FLYEYE_LW20_BASE 0
#define FLYEYE_ADS_BASE 0
#define XPAR_VIETTEL_SPI_BARO_CONTROLLER_0_S_AXI_BASEADDR 0
#define XPAR_SPI_AD7980_ENVI_0_S_AXI_BASEADDR 0
#define XPAR_SPI_AD7980_POWER_0_S_AXI_BASEADDR 0
#define XPAR_SSC_I2C_PRESSURE_SENSOR_0_S00_AXI_BASEADDR 0
#define VIETTEL_IMU_CONTROLLER_S00_AXI_SLV_REG3_OFFSET 0
#define XPAR_FABRIC_XLCONCAT_0_DOUT_INTR 0
#define XPAR_UART_CAM_S_AXI_BASEADDR 0

#define ALT_AVALON_SPI_COMMAND_MERGE 0

typedef signed char     alt_8;
typedef unsigned char   alt_u8;
typedef signed short    alt_16;
typedef unsigned short  alt_u16;
typedef signed long     alt_32;
typedef unsigned long   alt_u32;

int alt_avalon_spi_command(alt_u32 base, alt_u32 slave,
                           alt_u32 write_length, const alt_u8 * write_data,
                           alt_u32 read_length, alt_u8 * read_data,
                           alt_u32 flags);

int alt_irq_register (alt_u32 id, 
                             void*   context, 
                             void (*irq_handler)(void*, alt_u32));

int __builtin_ldbio (volatile const void *);
int __builtin_ldbuio (volatile const void *);
int __builtin_ldhio (volatile const void *);
int __builtin_ldhuio (volatile const void *);
int __builtin_ldwio (volatile const void *);
void __builtin_stbio (volatile void *, int);
void __builtin_sthio (volatile void *, int);
void __builtin_stwio (volatile void *, int);
void __builtin_sync (void);
int __builtin_rdctl (int);
void __builtin_wrctl (int, int);

#define __IO_CALC_ADDRESS_DYNAMIC(BASE, OFFSET) \
  ((void *)(((alt_u8*)BASE) + (OFFSET)))

#define VT_SBUS_CONTROLLER_mReadReg(BASE, OFFSET) \
  __builtin_ldwio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)))

#define VT_SBUS_CONTROLLER_mWriteReg(BASE, OFFSET, DATA) \
  __builtin_stwio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)), (DATA))

#define VIETTEL_IMU_CONTROLLER_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define VIETTEL_IMU_CONTROLLER_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define VIETTEL_MAGNET_WSM_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define VIETTEL_MAGNET_WSM_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define HMR_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define HMR_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define VIETTEL_I2C_LW20_CONTROLLER_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define VIETTEL_I2C_LW20_CONTROLLER_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define VIETTEL_SPI_BARO_CONTROLLER_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define VIETTEL_SPI_BARO_CONTROLLER_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define SPI_AD7980_ENVI_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define SPI_AD7980_ENVI_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define SPI_AD7980_POWER_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define SPI_AD7980_POWER_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define SSC_I2C_PRESSURE_SENSOR_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define SSC_I2C_PRESSURE_SENSOR_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define ADS1015_mReadReg(BASE, OFFSET) \
	__builtin_ldwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)))

#define ADS1015_mWriteReg(BASE, OFFSET, DATA) \
	__builtin_stwio(__IO_CALC_ADDRESS_DYNAMIC((BASE), (OFFSET)), (DATA))

#define IORD_32DIRECT(BASE, OFFSET) \
  __builtin_ldwio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)))
#define IORD_16DIRECT(BASE, OFFSET) \
  __builtin_ldhuio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)))
#define IORD_8DIRECT(BASE, OFFSET) \
  __builtin_ldbuio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)))

#define IOWR_32DIRECT(BASE, OFFSET, DATA) \
  __builtin_stwio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)), (DATA))
#define IOWR_16DIRECT(BASE, OFFSET, DATA) \
  __builtin_sthio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)), (DATA))
#define IOWR_8DIRECT(BASE, OFFSET, DATA) \
  __builtin_stbio (__IO_CALC_ADDRESS_DYNAMIC ((BASE), (OFFSET)), (DATA))

#define SYSTEM_BUS_WIDTH 32
#define __IO_CALC_ADDRESS_NATIVE(BASE, REGNUM) \
  ((void *)(((alt_u8*)BASE) + ((REGNUM) * (SYSTEM_BUS_WIDTH/8))))
#define IORD(BASE, REGNUM) \
  __builtin_ldwio (__IO_CALC_ADDRESS_NATIVE ((BASE), (REGNUM)))
#define IOWR(BASE, REGNUM, DATA) \
  __builtin_stwio (__IO_CALC_ADDRESS_NATIVE ((BASE), (REGNUM)), (DATA))

#define IOADDR_ALTERA_AVALON_PIO_DATA(base)           __IO_CALC_ADDRESS_NATIVE(base, 0)
#define IORD_ALTERA_AVALON_PIO_DATA(base)             IORD(base, 0) 
#define IOWR_ALTERA_AVALON_PIO_DATA(base, data)       IOWR(base, 0, data)

/* parasoft on */
// Enable static analysis

#endif  // PILOT_TARGET
#endif  // NIOSDUMMYDECL_H
