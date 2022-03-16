
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cctype>
#include <cmath>
#include <cfloat>
#include <climits>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>


using namespace std;

#include <pilot_cfg.h>

#if PILOT_TARGET == PT_HARDWARE
//#include <termios.h>
//#include <sys/ioctl.h>
#include <cstdbool>
#include <unistd.h>
#include <errno.h>
#include "xparameters.h"
#include "xil_types.h"
#include <includes.h>   // Declarations of the operatin system uC/OS-II
#include "xil_io.h"
#include "xqspips.h"
#include "xadcps.h"
#include "xil_misc_psreset_api.h"
#include "Viettel_FastRS.h"
#include "Viettel_IMU_Controller.h"
#include "VT_SBUS_Controller.h"
#include "Viettel_Spi_Baro_Controller.h"
#include "SSC_I2C_PRESSURE_SENSOR.h"
#if MAGNETOMETER_TYPE == USE_HMR
#include "HMR.h"
#endif
#include "Viettel_I2C_LW20_Controller.h"
#include "AD5622.h"
#include "ADS1015.h"
#include "VT_MMC5883_MA.h"

/* Third-party source code declarations. */
//extern "C" {
//	#include "libfatfs/ocsdc/sdcard.h"
#include "ff.h"
#include <user_types.h>
//}
    
#elif PILOT_TARGET == PT_WIN32
	#include <includes.h>   // Declarations of the operatin system uC/OS-II
	#include <xil_types.h>
    #include <direct.h>
    #include <errno.h>
    #include <io.h>
    #include <fcntl.h>
    #include <sys/stat.h>
    #include <NiosDummyDecl.h>

#else
    #error "Define PILOT_TARGET in pilot_cfg.h"

#endif  //PILOT_TARGET

#include <Defs.h>
#include <Numbers.h>
#include <StatQueue.h>
#include <DynQueue.h>
#include <rotation.h>
#include <Vector2.h>
#include <Matrix3.h>
#include <quaternion.h>
#include <Vector3.h>
#include <Filter.h>

#include <ODTSubject.h>
#include <SubsystemBase.h>
#include <StorageBase.h>
#include <PidModifierBase.h>
#include <GpsPosition.h>
#include <Base64.h>
#include <FParser.h>
#include <FPlanConf.h>
#include <PStateHealth.h>
#include <Iir.h>
#include <Crc.h>
#include <ServManData.h>
#include <ServoConf.h>
#include <TypeParser.h>
#include <ClassifiedLine.h>
#include <SysMonData.h>
#include <FPRealTrigger.h>
#include <ublox.h>
#if MAGNETOMETER_TYPE == USE_HMR
#include <Hmr2300.h>
#endif
//#include <ADIS16467.h>
#include <FlightTimeEstimator.h>
#include <IRQ.h>
#include <WindData.h>
#include <AhrsCF.h>
#include <IAsyncDevice.h>
#include <BufferBase.h>
#include <Rs485ADNiosHandlerWrapper.h>

#include <FPlanData.h>          ///< ref: FPlanConf
#include <FPRealData.h>         ///< ref: GpsPosition
#include <SerialDeviceBase.h>   ///< ref: ODTSubject
#include <GpsData.h>            ///< ref: GpsPosition
#include <DGpsData.h>           ///< ref: DGps Position
#include <PlatformLayer.h>      ///< ref: ServManData
#include <Camera.h>             ///< ref: GpsPosition
#include <FPRealCondition.h>    ///< ref: GpsPosition
#include <PidModifierPhi.h>     ///< ref: PidModifierBase
#include <PidModifierTheta.h>   ///< ref: PidModifierBase
#include <PidModifierAlr.h>     ///< ref: PidModifierBase
#include <PidModifierThrottle.h>///< ref: PidModifierBase
#include <StorageFactory.h>     ///< ref: StorageBase
#include <WindResolver.h>       ///< ref: GpsPosition
#include <FixBuffer.h>          ///< ref: BufferBase
#include <Rs485ADNios.h>          ///< ref: IAsyncDevice

#include <Pid.h>                ///< ref: FPRealData
#include <Navigation.h>
#include <SpdHgtControl.h>
#include <L1_Control.h>
#include <TECS.h>

#include <Gauge.h>              ///< ref: PlatformLayer (alt_*)
#include <ParameterNames.h>     ///< ref: GpsPosition, FPRealData
#include <OSBase.h>             ///< ref: PlatformLayer
#include <ChannelData.h>        ///< ref: FixedBuffer

#include <Ahrs.h>               ///< ref: Gauge
#include <OSUcosII.h>           ///< ref: OSBase, PlatformLayer
#include <OSNativeW32.h>        ///< ref: OSBase, PlatformLayer
#include <Semaphore.h>          ///< ref: OSBase
#include <ODTObserver.h>        ///< ref: OSBase
#include <ConsoleADWin.h>       ///< ref: OSBase, IAsyncDevice

#include <PStateData.h>         ///< ref: GpsPosition, WindData, Gauge, Ahrs
#include <StorageW32.h>         ///< ref: StorageBase, Semaphore
#include <StorageNIOSFlash.h>   ///< ref: StorageBase, Semaphore
#include <GeneralUtils.h>
#include <File.h>
#include <StorageSD.h>			///< ref: StorageBase, Semaphore
#include <StorageNIOSRam.h>     ///< ref: StorageBase, OSBase
#include <SystemLog.h>          ///< ref: Semaphore
#include <LineDispatcher.h>     ///< ref: ODTObserver, ODTSubject, ClassifiedLine, Semaphore, FParser
#include <UdpW32.h>             ///< ref: SerialDeviceBase, Semaphore
#include <UartW32.h>            ///< ref: SerialDeviceBase
#include <ConsoleW32.h>         ///< ref: SerialDeviceBase, Semaphore, OSBase
#include <UartNIOS.h>           ///< ref: SerialDeviceBase, Semaphore
#include <UartCAM.h>
#include <FastRS.h>             ///< ref: SerialDeviceBase, ODTObserver, Semaphore
#if MAGNETOMETER_TYPE == USE_HMR
#include <Rs485HMR.h>
#endif
#include <FplanContainer.h>     ///< ref: FPlanConf, Semaphore
#include <CmdQueue.h>           ///< ref: StatQueue, ClassifiedLine, Semaphore, SystemLog

#include <FlightPlan.h>         
#include <FlightPlanRealizer.h>
#include <FlightController.h>
#include <FlashProgrammer.h>
#include <SystemMonitor.h>
#include <PhysicalState.h>
#include <ServoManager.h>
#include <LogManager.h>
#include <CameraManager.h>
#include <CommChannel.h>


#include <GlobVars.h>           //ref: All subsystems in one place


