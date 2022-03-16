/**
*                                                                  
* @class PStateData                                                                                                                                
* Class representing data shared by the subsystem based on the PhysicalState class.                                   
* It is used as the interface beetwen susbsystems.                                                      
*                                                                   
* Class including data for communication and log.
*                                                                   
* 2008-2010 Witold Kruczek @ Flytronic                              
*/

#include <PilotIncludes.h>


void PStateTlmShort::fillFrom (const PStateData &psd, float pAmslCorr, float pAalCorr)
{
    	time = psd.time100 / 10;         // conversion from 100us to 1ms
		altitude = static_cast<INT32S>((psd.altitude + pAalCorr) * 100.0f);
		psd.position.getPosTlm(posLat, posLon);
		psi = static_cast<INT16S>(psd.psi * 1000.0f);
		theta = static_cast<INT16S>(psd.theta * 1000.0f);
		phi = static_cast<INT16S>(psd.phi * 1000.0f);
		windFrom = static_cast<INT16S>(psd.wind.from * 10.0f);
		windSpeed = static_cast<INT16S>(psd.wind.speed * 10.0f);
		airspeed = static_cast<INT16S>(psd.airspeed * 10.0f);
		groundspeed = static_cast<INT16S>(psd.groundspeed * 10.0f);
		track = static_cast<INT16S>(psd.track * 10.0f);
		amslCorr = static_cast<INT16S>(pAmslCorr);
		gpsAmsl = static_cast<INT32S>(psd.gpsAmsl * 100.0f);
}


void PStateTlmLong::fillFrom (const PStateData &psd, float pAmslCorr, float pAalCorr)
{
    	time = psd.time100 / 10;         // conversion from 100us to 1ms
		altitude = static_cast<INT32S>((psd.altitude + pAalCorr) * 100.0f);
		psd.position.getPosTlm(posLat, posLon);
		psi = static_cast<INT16S>(psd.psi * 1000.0f);
		theta = static_cast<INT16S>(psd.theta * 1000.0f);
		phi = static_cast<INT16S>(psd.phi * 1000.0f);
		R = static_cast<INT16S>(psd.R * 10000.0f);
		Q = static_cast<INT16S>(psd.Q * 10000.0f);
		P = static_cast<INT16S>(psd.P * 10000.0f);
		accZ = static_cast<INT16S>(psd.accZ * 1000.0f);
		accY = static_cast<INT16S>(psd.accY * 1000.0f);
		accX = static_cast<INT16S>(psd.accX * 1000.0f);
		windFrom = static_cast<INT16S>(psd.wind.from * 10.0f);
		windSpeed = static_cast<INT16S>(psd.wind.speed * 10.0f);
		airspeed = static_cast<INT16S>(psd.airspeed * 10.0f);
		groundspeed = static_cast<INT16S>(psd.groundspeed * 10.0f);
		track = static_cast<INT16S>(psd.track * 10.0f);
		amslCorr = static_cast<INT16S>(pAmslCorr);
		TAS = static_cast<INT16S>(psd.tas * 10.0f);
		OAT = static_cast<INT8S>(psd.outerAirTemp);
		gpsAmsl = static_cast<INT32S>(psd.gpsAmsl * 100.0f);
}

void PStateRawGaugeTlm::fillFrom (int time100us, const Gauge::rawMeasurementT &rm,const Gauge::rawDiffPressureMeasurementT &dp,const Gauge::rawBaroMeasurementT &ba, const Gauge::rawMagnetMeasurementT &mm, const Ahrs::ahrs_dataT &ahrs, const PStateData &psd, const AuxSensorData &auxSensorData)
{
	time    = time100us / 10;  // conversion from 100us to 1ms

    accZL   = static_cast<INT16S>(rm.accZL & 0xffff);
    accYL   = static_cast<INT16S>(rm.accYL & 0xffff);
    accXL   = static_cast<INT16S>(rm.accXL & 0xffff);
    accZH   = static_cast<INT16S>(rm.accZH & 0xffff);
    accYH   = static_cast<INT16S>(rm.accYH & 0xffff);
	accXH   = static_cast<INT16S>(rm.accXH & 0xffff);
	gyroZL  = static_cast<INT16S>(rm.gyroZL & 0xffff);
	gyroYL  = static_cast<INT16S>(rm.gyroYL & 0xffff);
	gyroXL  = static_cast<INT16S>(rm.gyroXL & 0xffff);


	gyroZH  = static_cast<INT16S>(rm.gyroZH & 0xffff);
	gyroYH  = static_cast<INT16S>(rm.gyroYH & 0xffff);
	gyroXH  = static_cast<INT16S>(rm.gyroXH & 0xffff);

	press1  = static_cast<INT32U>(dp.Pressure*100.0f);   //psi
	press2  = static_cast<INT32U>(ba.Pressure);
    temp1   = static_cast<INT16U>(mm.magTemperature);
    temp2   = static_cast<INT16U>(ba.Temperature*10.0f);

	accOverflow  = static_cast<INT8U>(rm.accOverflow & 0xff);
	gyroOverflow = static_cast<INT8U>(rm.gyroOverflow & 0xff);

#if USE_WSM == WSM_ENABLE
	magX  = static_cast<INT32S>(mm.magX * 1000.0f);
	magY  = static_cast<INT32S>(mm.magY * 1000.0f);
	magZ  = static_cast<INT32S>(mm.magZ * 1000.0f);
#else

#if MAGNETOMETER_TYPE == USE_HMR
	magX  = static_cast<INT32S>(mm.magX * 10000.0f);
	magY  = static_cast<INT32S>(mm.magY * 10000.0f);
	magZ  = static_cast<INT32S>(mm.magZ * 10000.0f);
#else
	magX  = static_cast<INT32S>(mm.magX * 1000.0f);
	magY  = static_cast<INT32S>(mm.magY * 1000.0f);
	magZ  = static_cast<INT32S>(mm.magZ * 1000.0f);
#endif
#endif
    
	diffPress  = static_cast<INT32S>(dp.Pressure *100.0f);   //psi
    staticPress = static_cast<INT32U>(ba.Pressure);
	gyroZTemperature  = static_cast<INT16U>(rm.gyroZTemperature & 0xffff);
	gyroYTemperature  = static_cast<INT16U>(rm.gyroYTemperature & 0xffff);
	gyroXTemperature  = static_cast<INT16U>(rm.gyroXTemperature & 0xffff);

	ahrsUpdateFlag = static_cast<INT8U>(ahrs.ahrsUpdateFlag & 0xff);
	inclError = static_cast<INT16S>(ahrs.incdError);
}

void PStateAuxSensorTlm::fillFrom (int time100us, const AuxSensorData &auxSensorData)
{
	u32_time             = time100us / 10;  // conversion from 100us to 1ms
	u16_engineSpeed      = static_cast<INT16U>(auxSensorData.i_engineSpeed & 0xffff);
	u8_fuelLevel         = static_cast<INT8U>(auxSensorData.i_fuelLevel & 0xff);
	u8_env_rh            = static_cast<INT8U>(auxSensorData.i_env_rh & 0xff);
	u8_env_temp          = static_cast<INT8U>(auxSensorData.i_env_temp & 0xff);
	u16_voltage_1        = static_cast<INT16U>(auxSensorData.f_voltage_1 * 100.0f);
    u16_voltage_2        = static_cast<INT16U>(auxSensorData.f_voltage_2 * 100.0f);
	u8_current           = static_cast<INT8U>(auxSensorData.f_current * 100.0f);
	u32_pressure         = static_cast<INT32U>(auxSensorData.i_pressure);
	u8_engine_temp       = static_cast<INT8U>(auxSensorData.i_engine_temp & 0xff);
	u16_hubStatus        = static_cast<INT16U>(auxSensorData.t_hubStatus.u16_status & 0xffff);
#if PILOT_TYPE == VUA_SL
	u16_coolingOilTemp   = static_cast<INT16U>(auxSensorData.i_coolingOilTemp & 0xffff);
	u16_coolingWaterTemp = static_cast<INT16U>(auxSensorData.i_coolingWaterTemp & 0xffff);
	u16_cylinderTemp     = static_cast<INT16U>(auxSensorData.i_cylinderTemp & 0xffff);
	u16_oilPressure      = static_cast<INT16U>(auxSensorData.i_oilPressure & 0xffff);
	u8_orangeLed         = static_cast<INT8U>(auxSensorData.i_orangeLed & 0xff);
	u8_redLed            = static_cast<INT8U>(auxSensorData.i_redLed & 0xff);
	u16_engineSpeedTCU   = static_cast<INT16U>(auxSensorData.i_engineSpeedTCU & 0xffff);
	u8_yellowLed         = static_cast<INT8U>(auxSensorData.i_yellowLed & 0xff);
#endif
}
