/**                                                                 
* @class FPRealData                                                                   
*     
* @brief Class representing the data made available by the subsystem based on the FlightPlanRealizer class.                                   
* This class is an interface between subsystems.
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>


/**  
*  Constructor
*/
FPRealData::ControllerProperties::ControllerProperties()
{
    reset();
}


/**  
*  Reset all controllers values.
*/
void FPRealData::ControllerProperties::reset(void)
{
    enable = false;
    minValue = 0.0f;
    maxValue = 0.0f;
    marginLow = 0.0f;
    marginHigh = 0.0f;
    invMargins = false;
    bank = 0;
}

/**
*  Set up properties for controllers
*/
void FPRealData::ControllerProperties::
	setControllerParams (INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	enable     = true;
	minValue   = minVal;
	maxValue   = maxVal;
	bank       = bankVal;
}

void FPRealData::ThrottleControllerProperties::disableControllers (void)
{
	Thr_Speed.enable = false;
	Thr_Alt.enable   = false;
}
void FPRealData::ThrottleControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Thr_Alt.enable, Thr_Speed.enable);
}
void FPRealData::ThrottleControllerProperties::triggerEnableFlag (void)
{
	bool thr_alt_flag   = enableFlagGrp.bits.bit0;
	bool thr_speed_flag = enableFlagGrp.bits.bit1;

	if (updateEnableBit (Thr_Alt.enable, thr_alt_flag))     return;
	if (updateEnableBit (Thr_Speed.enable, thr_speed_flag)) return;

	updateEnableFlag ();
}
void FPRealData::ThrottleControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case SPD_THR_SPD_ID:
		Thr_Speed.setControllerParams (minVal, maxVal, bankVal);
		break;
	case SPD_THR_ALT_ID:
		Thr_Alt.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::FlapControllerProperties::disableControllers (void)
{
	Flp_Speed.enable = false;
}
void FPRealData::FlapControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Flp_Speed.enable);
}
void FPRealData::FlapControllerProperties::triggerEnableFlag (void)
{
	bool flp_spd_flag   = enableFlagGrp.bits.bit0;

	if (updateEnableBit (Flp_Speed.enable, flp_spd_flag))     return;

	updateEnableFlag ();
}
void FPRealData::FlapControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case SPD_FLP_SPD_ID:
		Flp_Speed.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::ElvControllerProperties::disableControllers (void)
{
	Elv_Q.enable = false;
}
void FPRealData::ElvControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Elv_Q.enable);
}
void FPRealData::ElvControllerProperties::triggerEnableFlag (void)
{
	bool elv_q_flag   = enableFlagGrp.bits.bit0;

	if (updateEnableBit (Elv_Q.enable, elv_q_flag))     return;

	updateEnableFlag ();
}
void FPRealData::ElvControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LAT_HOLD_Q_ID:
		Elv_Q.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::QControllerProperties::disableControllers (void)
{
	Q_Theta.enable = false;
}
void FPRealData::QControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Q_Theta.enable);
}
void FPRealData::QControllerProperties::triggerEnableFlag (void)
{
	bool q_theta_flag   = enableFlagGrp.bits.bit0;

	if (updateEnableBit (Q_Theta.enable, q_theta_flag))     return;

	updateEnableFlag ();
}
void FPRealData::QControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LAT_HOLD_THETA_ID:
		Q_Theta.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::ThetaControllerProperties::disableControllers (void)
{
	Theta_Alt.enable = false;
	Theta_Speed.enable = false;
	Theta_VertSpeed.enable = false;
}
void FPRealData::ThetaControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Theta_Alt.enable, Theta_Speed.enable, Theta_VertSpeed.enable);
}
void FPRealData::ThetaControllerProperties::triggerEnableFlag (void)
{
	bool theta_alt_flag       = enableFlagGrp.bits.bit0;
	bool theta_speed_flag     = enableFlagGrp.bits.bit1;
	bool theta_vertspeed_flag = enableFlagGrp.bits.bit2;

	if (updateEnableBit (Theta_Alt.enable, theta_alt_flag))             return;
	if (updateEnableBit (Theta_Speed.enable, theta_speed_flag))         return;
	if (updateEnableBit (Theta_VertSpeed.enable, theta_vertspeed_flag)) return;

	updateEnableFlag ();
}
void FPRealData::ThetaControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LAT_HOLD_SPD_ID:
		Theta_Speed.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LAT_THETA_ALT_ID:
		Theta_Alt.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LAT_HOLD_VZ_ID:
		Theta_VertSpeed.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::VertSpeedControllerProperties::disableControllers (void)
{
	VertSpeed_Alt.enable = false;
}
void FPRealData::VertSpeedControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(VertSpeed_Alt.enable);
}
void FPRealData::VertSpeedControllerProperties::triggerEnableFlag (void)
{
	bool vertspeed_alt_flag = enableFlagGrp.bits.bit0;

	if (updateEnableBit (VertSpeed_Alt.enable, vertspeed_alt_flag)) return;
	updateEnableFlag ();
}
void FPRealData::VertSpeedControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LAT_VZ_ALT_ID:
		VertSpeed_Alt.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::AlrControllerProperties::disableControllers (void)
{
	Alr_P.enable = false;
}
void FPRealData::AlrControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Alr_P.enable);
}
void FPRealData::AlrControllerProperties::triggerEnableFlag (void)
{
	bool alr_p_flag   = enableFlagGrp.bits.bit0;

	if (updateEnableBit (Alr_P.enable, alr_p_flag))     return;

	updateEnableFlag ();
}
void FPRealData::AlrControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_HOLD_P_ID:
		Alr_P.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::PControllerProperties::disableControllers (void)
{
	P_Phi.enable = false;
}
void FPRealData::PControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(P_Phi.enable);
}
void FPRealData::PControllerProperties::triggerEnableFlag (void)
{
	bool p_phi_flag   = enableFlagGrp.bits.bit0;

	if (updateEnableBit (P_Phi.enable, p_phi_flag))     return;

	updateEnableFlag ();
}
void FPRealData::PControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_HOLD_PHI_ID:
		P_Phi.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::PhiControllerProperties::disableControllers (void)
{
	Phi_Track.enable = false;
	Phi_CTrack.enable = false;
	Phi_Psi.enable = false;
}
void FPRealData::PhiControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Phi_Track.enable, Phi_CTrack.enable, Phi_Psi.enable);
}
void FPRealData::PhiControllerProperties::triggerEnableFlag (void)
{
	bool phi_track_flag  = enableFlagGrp.bits.bit0;
	bool phi_ctrack_flag = enableFlagGrp.bits.bit1;
	bool phi_psi_flag    = enableFlagGrp.bits.bit2;

	if (updateEnableBit (Phi_Track.enable, phi_track_flag))   return;
	if (updateEnableBit (Phi_CTrack.enable, phi_ctrack_flag)) return;
	if (updateEnableBit (Phi_Psi.enable, phi_psi_flag))       return;

	updateEnableFlag ();
}
void FPRealData::PhiControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_HOLD_TRACK_ID:
		Phi_Track.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LON_HOLD_CTRACK_ID:
		Phi_CTrack.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LON_HOLD_PSI_ID:
		Phi_Psi.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::TrackControllerProperties::disableControllers (void)
{
	Track_Wpt.enable = false;
	Track_TrackCorr.enable = false;
}
void FPRealData::TrackControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Track_Wpt.enable, Track_TrackCorr.enable);
}
void FPRealData::TrackControllerProperties::triggerEnableFlag (void)
{
	bool track_wpt_flag       = enableFlagGrp.bits.bit0;
	bool track_trackCorr_flag = enableFlagGrp.bits.bit1;

	if (updateEnableBit (Track_Wpt.enable, track_wpt_flag))             return;
	if (updateEnableBit (Track_TrackCorr.enable, track_trackCorr_flag)) return;

	updateEnableFlag ();
}
void FPRealData::TrackControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_TRACK_WPT_ID:	case LON_CRC_WPT_ID:
		Track_Wpt.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LON_TRACK_PATH_ID:
		Track_TrackCorr.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::TrackCorrControllerProperties::disableControllers (void)
{
	TrackCorr_Cte.enable = false;
}
void FPRealData::TrackCorrControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(TrackCorr_Cte.enable);
}
void FPRealData::TrackCorrControllerProperties::triggerEnableFlag (void)
{
	bool trackCorr_cte_flag   = enableFlagGrp.bits.bit0;

	if (updateEnableBit (TrackCorr_Cte.enable, trackCorr_cte_flag))     return;

	updateEnableFlag ();
}
void FPRealData::TrackCorrControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_TRACK_PATH_ID:
		TrackCorr_Cte.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::RudderControllerProperties::disableControllers (void)
{
	Rdr_R.enable = false;
	Rdr_Yacc.enable = false;
}
void FPRealData::RudderControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(Rdr_R.enable, Rdr_Yacc.enable);
}
void FPRealData::RudderControllerProperties::triggerEnableFlag (void)
{
	bool rudder_r_flag    = enableFlagGrp.bits.bit0;
	bool rudder_yacc_flag = enableFlagGrp.bits.bit1;

	if (updateEnableBit (Rdr_R.enable, rudder_r_flag))       return;
	if (updateEnableBit (Rdr_Yacc.enable, rudder_yacc_flag)) return;

	updateEnableFlag ();
}
void FPRealData::RudderControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_HOLD_R_ID:
		Rdr_R.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LON_HOLD_YACC_ID:
		Rdr_Yacc.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

void FPRealData::RControllerProperties::disableControllers (void)
{
	R_Psi.enable = false;
	R_CoordExp.enable = false;
	R_Track.enable = false;
}
void FPRealData::RControllerProperties::updateEnableFlag (void)
{
	enableFlagGrp.byte = Bits::setByte(R_Psi.enable, R_CoordExp.enable, R_Track.enable);
}
void FPRealData::RControllerProperties::triggerEnableFlag (void)
{
	bool r_psi_flag      = enableFlagGrp.bits.bit0;
	bool r_coordexp_flag = enableFlagGrp.bits.bit1;
	bool r_track_flag    = enableFlagGrp.bits.bit2;

	if (updateEnableBit (R_Psi.enable, r_psi_flag))           return;
	if (updateEnableBit (R_CoordExp.enable, r_coordexp_flag)) return;
	if (updateEnableBit (R_Track.enable, r_track_flag))       return;

	updateEnableFlag ();
}
void FPRealData::RControllerProperties::
	setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case LON_R_PSI_ID:
		R_Psi.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LON_R_COORDEXP_ID:
		R_CoordExp.setControllerParams (minVal, maxVal, bankVal);
		break;
	case LON_R_TRACK_ID:
		R_Track.setControllerParams (minVal, maxVal, bankVal);
		break;
	default: break;
	}
	triggerEnableFlag ();
}

/**
* L1 Controller
*/
void FPRealData::L1ControllerProperties::disableControllers(void)
{
	L1Control.enable = false;
}
void FPRealData::L1ControllerProperties::updateEnableFlag(void)
{
	enableFlagGrp.byte = Bits::setByte(L1Control.enable);
}
void FPRealData::L1ControllerProperties::triggerEnableFlag(void)
{
	bool L1Control_flag = enableFlagGrp.bits.bit0;

	if (updateEnableBit(L1Control.enable, L1Control_flag))     return;

	updateEnableFlag();
}
void FPRealData::L1ControllerProperties::
setControllerParams(INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	switch (ID)
	{
	case L1_HEADING_HOLD_ID:
		L1Control.setControllerParams(minVal, maxVal, bankVal);
		nav_mode = NAV_MODE::HEADING_HOLD;
		break;
	case L1_CIRCLE_ID:
		L1Control.setControllerParams(minVal, maxVal, bankVal);
		nav_mode = NAV_MODE::CIRCLE;
		break;
	case L1_HOMING_ID:
		L1Control.setControllerParams(minVal, maxVal, bankVal);
		nav_mode = NAV_MODE::HOMING;
		break;
	default: break;
	}
	triggerEnableFlag();
}

/**
* TECSThr Controller
*/
void FPRealData::TECSControllerProperties::disableControllers(void)
{
	ThrTheta_TECS.enable = false;
}
void FPRealData::TECSControllerProperties::updateEnableFlag(void)
{
	enableFlagGrp.byte = Bits::setByte(ThrTheta_TECS.enable);
}
void FPRealData::TECSControllerProperties::triggerEnableFlag(void)
{
	bool ThrTheta_TECS_flag = enableFlagGrp.bits.bit0;

	if (updateEnableBit(ThrTheta_TECS.enable, ThrTheta_TECS_flag))
		return;

	updateEnableFlag();
}
void FPRealData::TECSControllerProperties::
setControllerParams(INPUT CONTROLLER_CHANNEL_ID ID, INPUT float minVal, INPUT float maxVal, INPUT int bankVal)
{
	ThrTheta_TECS.setControllerParams(minVal, maxVal, bankVal);
	triggerEnableFlag();
}

/**  
*  Function that turns off all controllers.
*/
void FPRealData::ControllerPropertiesSet::disableControllers(void)
{
    ThrCProp.disableControllers();
	FlpCProp.disableControllers();

	ElvCProp.disableControllers();
	QCProp.disableControllers();
	ThetaCProp.disableControllers();
	VertSpeedCProp.disableControllers();

	AlrCProp.disableControllers();
	PCProp.disableControllers();
    PhiCProp.disableControllers();    
	TrackCProp.disableControllers ();
	TrackCorrCProp.disableControllers();
	
	RudderCProp.disableControllers();
	RCProp.disableControllers ();

    Thr_Alt_2State.enable = false;
    Btfly_Alt_2State.enable = false;
    Abr_GPErr.enable = false;
    FAlr_Alr.enable = false;
    Btfly_GPath_2State.enable = false;
	Theta_GPath_2State.enable = false;
}

/**  
*  Function that turns off all latitude controllers.
*/
void FPRealData::ControllerPropertiesSet::disableLatControllers(void)
{
	ElvCProp.disableControllers();
	QCProp.disableControllers();
	ThetaCProp.disableControllers();
	VertSpeedCProp.disableControllers();
}

/**  
*  Function that turns off all longitude controllers.
*/
void FPRealData::ControllerPropertiesSet::disableLonAlrControllers(void)
{
	AlrCProp.disableControllers();
	PCProp.disableControllers();;
	TrackCorrCProp.disableControllers();
	PhiCProp.disableControllers();
	TrackCProp.disableControllers ();
}

/**  
*  Function that turns off all longitude controllers.
*/
void FPRealData::ControllerPropertiesSet::disableLonRdrControllers(void)
{
	RudderCProp.disableControllers ();
	RCProp.disableControllers ();
}

/**  
*  Function that turns off all speed controllers.
*/
void FPRealData::ControllerPropertiesSet::disableSpdControllers(void)
{
	ThrCProp.disableControllers();
	FlpCProp.disableControllers();
}


/**  
*  Constructor
*/
OutputControls::FlightControl::FlightControl (void)
{
    reset();
    time100 = 0;
    for (int i=0; i<CUSTOM_OUTPUT_COUNT; i++)
        customOutput[i] = 0.0f;
}


/**  
* Sends to the specified device steering control isntruction and engine steering in "Props" FlightGear format.
*  /param device - device to which send instruction.
*  /param b*** - turn on cotrolling for specified element.
*/
void OutputControls::FlightControl::sendToFg (SerialDeviceBase* device, bool bAilerons, bool bElevator,
	bool bRudder, bool bThrottle, bool bFlaps, bool _parachuteChanged, float measThrottle) const
{
    char buf[LINESIZE];

    if (bAilerons)
    {
        SNPRINTF (buf, sizeof(buf), "set controls/flight/aileron %f ", ailerons);
        device->sendLine(buf);
    }

    if (bElevator)
    {
        SNPRINTF (buf, sizeof(buf), "set controls/flight/elevator %f ", elevator);
        device->sendLine(buf);
    }

	SNPRINTF(buf, sizeof(buf), "set controls/flight/l_elevon %f ", leftElevon);
	device->sendLine(buf);

	SNPRINTF(buf, sizeof(buf), "set controls/flight/r_elevon %f ", rightElevon);
	device->sendLine(buf);

    if (bRudder)
    {
        SNPRINTF (buf, sizeof(buf), "set controls/flight/rudder %f ", rudder);
        device->sendLine(buf);
    }

    if (bThrottle)
    {
		SNPRINTF(buf, sizeof(buf), "set rpm %f ", (double)measThrottle * 16000);
		device->sendLine(buf);
    }

	if (bFlaps)
	{
        // setting inly for the butterfly (airbrake in the model)
        SNPRINTF (buf, sizeof(buf), "set controls/flight/speedbrake %f ", butterfly);
        device->sendLine(buf);
	}
	if (_parachuteChanged)
	{
		SNPRINTF(buf, sizeof(buf), "set parachute on");
		device->sendLine(buf);
	}
}


/**  
*  Reset all variables.
*  Executed at the beggining every flight plan instruction to clean the state.
*/
void OutputControls::FlightControl::reset (void)
{
    ailerons        = 0.0f;
    elevator        = 0.0f;
	leftElevon		= 0.0f;
	rightElevon		= 0.0f;
    rudder          = 0.0f;
    throttle        = 0.0f;
    flaps           = 0.0f;
    airbrakes       = 0.0f;
    containerDrop   = 0.0f;
    butterfly       = 0.0f;
    flapsAsAilerons = 0.0f;
    // from version 0.489 instead of antenna is lock heating.
    parachute       = 0.0f;
}


/**  
* Fills telemetry structure.
*/
void FPRealTlm::fillFrom (int time100us, bool doNotBreak, bool doNotManualTurn, bool manualTurn,
						  bool preland, bool approach, bool descent, bool land, bool abortLnd,
                          bool doNotCameraGuide, bool cameraGuide, bool observation, bool returnMode, int xProcId,
                          int xFpStatus, float xRefAirspeed, float xRefAltitude, int xFpItemId)
{
    //  Integer division  !
    time    = time100us / 10;   // time must be in ms but is in 100us
    flags   = (doNotBreak       ?    1 : 0) +
              (doNotManualTurn  ?    2 : 0) +
              (manualTurn       ?    4 : 0) +
              (doNotCameraGuide ?    8 : 0) +
              (cameraGuide      ?   16 : 0) +
              (observation      ?   32 : 0) +
			  (returnMode		?	64 : 0) +
			  (preland			?	128 : 0) +
			  (approach			?	256 : 0) +
			  (descent			?	512 : 0) +
			  (land				?	1024 : 0) +
			  (abortLnd			?	2048 : 0);

    procId   = static_cast<INT8U>(xProcId);
    fpStatus = static_cast<INT8U>(xFpStatus);
    refAirspeed  = static_cast<INT8U>(xRefAirspeed);
    refAltitude  = static_cast<INT16S>(xRefAltitude);
    fpItemId     = static_cast<INT16S>(xFpItemId);
}

void FPRealTlm::fillFrom(int time100us, bool doNotBreak, bool doNotManualTurn, bool manualTurn,
	bool lnd_preland, bool lnd_estime_heading, bool lnd_parachuting, bool lnd_parachuted, bool tkf_p1, bool tkf_p2, bool tkf_p3, 
	bool doNotCameraGuide, bool cameraGuide, bool observation, bool returnMode, int xProcId,
	int xFpStatus, float xRefAirspeed, float xRefAltitude, int xFpItemId)
{
	//  Integer division  !
	time = time100us / 10;   // time must be in ms but is in 100us
	flags = (doNotBreak ? 1 : 0) +
		(doNotManualTurn ? 2 : 0) +
		(manualTurn ? 4 : 0) +
		(doNotCameraGuide ? 8 : 0) +
		(cameraGuide ? 16 : 0) +
		(observation ? 32 : 0) +
		(returnMode ? 64 : 0) +
		(lnd_preland ? 128 : 0) +
		(lnd_estime_heading ? 256 : 0) +
		(lnd_parachuting ? 512 : 0) +
		(lnd_parachuted ? 1024 : 0) +
		(tkf_p1 ? 2048 : 0) +
		(tkf_p2 ? 4096 : 0) +
		(tkf_p3 ? 8192 : 0);

	procId = static_cast<INT8U>(xProcId);
	fpStatus = static_cast<INT8U>(xFpStatus);
	refAirspeed = static_cast<INT8U>(xRefAirspeed);
	refAltitude = static_cast<INT16S>(xRefAltitude);
	fpItemId = static_cast<INT16S>(xFpItemId);
}
