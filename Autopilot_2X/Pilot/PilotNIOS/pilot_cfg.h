/*********************************************************************/
/*                                                                   */
/* PilotCfg                                                          */
/*                                                                   */
/* Definicje zale¿ne od platformy sprzêtowej.                        */
/*                                                                   */
/* 2008 Witold Kruczek @ Flytronic                                   */
/*********************************************************************/

#ifndef PILOTCFG_H
#define PILOTCFG_H

#define PT_WIN32    1
#define PT_DEVKIT   2
#define PT_HARDWARE 3

#define ENABLED 1
#define DISABLED 0

#define PILOT_TARGET PT_HARDWARE 

#define VUA_SC_3TH   1          // small
#define VUA_SM       2          // medium
#define VUA_SL       3          // long
#define VUA_SC       4          // 3G
#define VUA_SC_6G    5          // 6G
#define PILOT_TYPE   VUA_SC_6G

#define VUA_G        1          // gas
#define VUA_E        2          // electric
#define ENGINE_TYPE  VUA_G

#define UBLOX_8 1
#define UBLOX_6 2
#define GPS_TYPE UBLOX_8

#define EMRG_LOSS_ENGINE DISABLED
#define USE_DGPS DISABLED
#define USE_LEAFLETS DISABLED


#define AUTO_LAND DISABLED
#define SEMI_LAND ENABLED
#define LAND_MODE SEMI_LAND

#define USE_SDCARD		0
#define USE_MMC			1
#define EXTERNAL_MEMORY_TYPE	USE_MMC

#define USE_HMR			0
#define USE_MMC			1
#define MAGNETOMETER_TYPE	USE_HMR

#define CAM_ENABLE		1
#define CAM_DISABLE	0
#define USE_CAM		CAM_ENABLE

#define WSM_ENABLE		1
#define WSM_DISABLE		0
#define USE_WSM			WSM_ENABLE

#define BARO_SPI	0
#define BARO_I2C	1
#define BARO_TYPE	BARO_SPI

#define FLIGHTGEAR 1
#define XPLANE 2

#if PILOT_TYPE == VUA_SC_6G
    #define SIMULATOR XPLANE
#else 
    #define SIMULATOR FLIGHTGEAR   
#endif     

#endif  //  PILOTCFG_H
