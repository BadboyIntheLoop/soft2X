#include "PilotIncludes.h"

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle

L1_Control::L1_Control(ControllerID id, FPRealData::L1ControllerProperties *cp0, L1Params *param0) : _latAccDem(0.0f),
                               _L1_dist(0.0f),
                               _WPcircle(false),
                               _nav_bearing(0.0f),
                               _bearing_error(0.0f),
                               _crosstrack_error(0.0f),
                               _target_bearing_cd(0),
                               _last_Nu(0.0f),
                               _last_update_waypoint_us(0)
{
    init(id, cp0, param0);
};

void L1_Control::init(ControllerID id, FPRealData::L1ControllerProperties *cp0, L1Params *param0)
{
	tlmData.tlm.id = static_cast<INT8U>(id);
	params = param0;
	cp = cp0;
}

L1Params::L1Params()
{
	setDefault();
}

void L1Params::setDefault()
{
    L1_period = 20.0f;
    L1_damping = 0.25f;  
    L1_xtrack_i_gain = 0.01f; 
    loiter_bank_limit = 60.0f;
	logEnable = false;
	tlmEnable = false;
}

/*
  Wrap AHRS yaw if in reverse - Numbers::radians
 */
float L1_Control::get_yaw(const PStateData& psd)
{
    if (_reverse) {
        return Numbers::wrap_PI(M_PI + psd.psi);
    }
    return psd.psi;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t L1_Control::get_yaw_sensor(float psi) const
{
    int32_t yaw_sensor = static_cast<int32_t>(Numbers::degrees(psi) * 100.0f);
    if (_reverse) {
        return Numbers::wrap_180_cd(18000 + yaw_sensor);
    }
    return yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t L1_Control::nav_roll_cd(const PStateData& psd) const
{
    float ret;
	ret = cosf(psd.theta) * Numbers::degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
	ret = Numbers::constrain_float1(ret, -9000, 9000);
    return static_cast<int32_t>(ret);
}

float L1_Control::nav_roll_rad(const PStateData& psd) const
{
    float ret;
	ret = cosf(psd.theta) * atanf(_latAccDem * 0.101972f); // 0.101972 = 1/9.81
	return Numbers::constrain_float1(ret, -PI2, PI2);
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t L1_Control::nav_bearing_cd(void) const
{
    return Numbers::wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t L1_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t L1_Control::target_bearing_cd(void) const
{
    return Numbers::wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float L1_Control::turn_distance(float wp_radius) const
{
    //wp_radius *= Numbers::safe_sqrt(_ahrs.get_EAS2TAS());
    wp_radius *= 1;
    return Numbers::minF(wp_radius, _L1_dist);
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float L1_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float L1_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = Numbers::constrain_float1(params->loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(Numbers::radians(sanitized_bank_limit)) * G;

    float nominal_velocity_sea_level = 0.0f;
    // if(_spdHgtControl == nullptr) {
    //     nominal_velocity_sea_level = 0.0f;
    // } else {
    //     nominal_velocity_sea_level =  _spdHgtControl->get_target_airspeed();
    // }

    //float eas2tas_sq = Numbers::safe_sqrt(_ahrs.get_EAS2TAS());
    float eas2tas_sq = 1;

    if (Numbers::is_zero(sanitized_bank_limit) || Numbers::is_zero(nominal_velocity_sea_level) ||
        Numbers::is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = Numbers::safe_sqrt(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return Numbers::maxF(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void L1_Control::_prevent_indecision(const PStateData& psd, float &Nu)
{
    const float Nu_limit = 0.9f*M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
		labs(Numbers::wrap_180_cd(_target_bearing_cd - get_yaw_sensor(psd.psi))) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        Nu = _last_Nu;
    }
}

// update L1 control for waypoint navigation
void L1_Control::update_waypoint(const PStateData& psd, const GpsPosition &prev_WP, const GpsPosition &next_WP, float dist_min)
{
    // Log.msgPrintf ("GpsPosition: %.6f", psd.position.getLat());
    PStateHealth psh;
    if (!PState->getPStateHealth(psh))
    {
        return;
    }
    GpsPosition _current_loc = psd.position;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    //uint32_t now = AP_HAL::micros();
    uint32_t now = psd.time100 * 100;
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1f) {
        dt = 0.1f;
        _L1_xtrack_i = 0.0f;
    }
    _last_update_waypoint_us = now;

    // Calculate L1 gain required for specified damping
    float K_L1 = 4.0f * params->L1_damping * params->L1_damping;

    // Get current position and velocity
    if (psh.gpsError != PStateHealth::gpsErrorCode::ERR_GPS_OK) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    // update _target_bearing_cd
    _target_bearing_cd = GpsPosition::track_cd(_current_loc, next_WP);

    //Calculate groundspeed
    float groundSpeed = psd.groundspeed;
    Vector2f _groundspeed_vector = Vector2f(cosf(Numbers::radians(psd.track)), sinf(Numbers::radians(psd.track))) * groundSpeed;
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw(psd)), sinf(get_yaw(psd))) * groundSpeed;
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/1/pipi
    _L1_dist = Numbers::maxF(0.3183099f * params->L1_damping * params->L1_period * groundSpeed, dist_min);

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = GpsPosition::distanceVector(prev_WP, next_WP);
    float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = GpsPosition::distanceVector(_current_loc, next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw(psd)), sinf(get_yaw(psd)));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = GpsPosition::distanceVector(prev_WP,_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;
    if (WP_A_dist > _L1_dist && alongTrackDist/Numbers::maxF(WP_A_dist, 1.0f) < -0.7071f)
    {
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (Numbers::radians) from AC to L1 point
    } else if (alongTrackDist > AB_length + groundSpeed*3) {
        // we have passed point B by 3 seconds. Head towards B
        // Calc Nu to fly To WP B
        const Vector2f B_air = GpsPosition::distanceVector(_current_loc, next_WP);
        Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft
        xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (Numbers::radians) from AC to L1 point
    } else { //Calc Nu to fly along AB line

        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track
        ltrackVel = _groundspeed_vector * AB; // Velocity along track
        float Nu2 = atan2f(xtrackVel,ltrackVel);
        //Calculate Nu1 angle (Angle to L1 reference point)
        float sine_Nu1 = _crosstrack_error/Numbers::maxF(_L1_dist, 0.1f);
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
        sine_Nu1 = Numbers::constrain_float1(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1 = asinf(sine_Nu1);

        // compute integral error component to converge to a crosstrack of zero when traveling
        // straight but reset it when disabled or if it changes. That allows for much easier
        // tuning by having it re-converge each time it changes.
        if (params->L1_xtrack_i_gain <= 0 || !Numbers::is_equal(params->L1_xtrack_i_gain, _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i = 0;
            _L1_xtrack_i_gain_prev = params->L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < Numbers::radians(5)) {
            _L1_xtrack_i += Nu1 * params->L1_xtrack_i_gain * dt;

            // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
            _L1_xtrack_i = Numbers::constrain_float1(_L1_xtrack_i, -0.1f, 0.1f);
        }

        // to converge to zero we must push Nu1 harder
        Nu1 += _L1_xtrack_i;

        Nu = Nu1 + Nu2;
        _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (Numbers::radians) from AC to L1 point
    }

    _prevent_indecision(psd, Nu);
    _last_Nu = Nu;

    //Limit Nu to +-(pi/2)
	Nu = Numbers::constrain_float1(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

	// Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (Numbers::radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for loitering
void L1_Control::update_loiter(const PStateData& psd, const GpsPosition &center_WP, float radius, int8_t loiter_direction)
{
    GpsPosition _current_loc = psd.position;
    PStateHealth psh;
    if (!PState->getPStateHealth(psh))
    {
        return;
    }
    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius = loiter_radius(fabsf(radius));

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / params->L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * params->L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * params->L1_damping * params->L1_damping;

    //Get current position and velocity
    //if (_ahrs.get_position(_current_loc) == false) {
    if (psh.gpsError != PStateHealth::gpsErrorCode::ERR_GPS_OK) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    //Calculate groundspeed
    float groundSpeed = psd.groundspeed;
    Vector2f _groundspeed_vector = Vector2f(cosf(Numbers::radians(psd.track)), sinf(Numbers::radians(psd.track))) * groundSpeed;
    groundSpeed = Numbers::maxF(_groundspeed_vector.length() , 1.0f);


    // update _target_bearing_cd
    _target_bearing_cd = GpsPosition::track_cd(_current_loc, center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * params->L1_damping * params->L1_period * groundSpeed;

    //Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = GpsPosition::distanceVector(center_WP, _current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(psd.psi), sinf(psd.psi));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(psd, Nu);
    _last_Nu = Nu;

    Nu = Numbers::constrain_float1(Nu, -PI2, PI2); //Limit Nu to +- Pi/2

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    //Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

    // keep crosstrack error for reporting
    _crosstrack_error = xtrackErrCirc;

    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

	int loiterDirection = -1;
	if (loiter_direction)
	{
		loiterDirection = -1;
	}
	else
	{
		loiterDirection = 1;
	}

    //Calculate tangential velocity
	float velTangent = xtrackVelCap * float(loiterDirection);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  Numbers::maxF(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent / Numbers::maxF((0.5f * radius), (radius + xtrackErrCirc));
    latAccDemCircCtr = 0.0f;

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
	float latAccDemCirc = loiterDirection * (latAccDemCircPD + latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
	if (xtrackErrCirc > 0.0f && loiterDirection * latAccDemCap < loiterDirection * latAccDemCirc) {
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (Numbers::radians) from AC to L1 point
    } else {
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (Numbers::radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (Numbers::radians)from AC to L1 point
    }

    tlmData.P = _L1_dist;
    tlmData.I = _nav_bearing;
    tlmData.D = _latAccDem;

    tlmData.tlm.time = psd.time100 / 10;
    tlmData.tlm.input = _bearing_error;
	tlmData.tlm.output = static_cast<FLOAT>(nav_roll_cd(psd));
    tlmData.tlm.reference = 0;
    _data_is_stale = false; // status are correctly updated with current waypoint data
}


// update L1 control for heading hold navigation
void L1_Control::update_heading_hold(const PStateData& psd, int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/params->L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = Numbers::wrap_180_cd(navigation_heading_cd);
    _nav_bearing = Numbers::radians(navigation_heading_cd * 0.01f);

    Nu_cd = _target_bearing_cd - Numbers::wrap_180_cd(get_yaw_sensor(psd.psi));
    Nu_cd = Numbers::wrap_180_cd(Nu_cd);
    Nu = Numbers::radians(Nu_cd * 0.01f);

    //Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    //float groundSpeed = _groundspeed_vector.length();
    float groundSpeed = psd.groundspeed;

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (Numbers::radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = Numbers::constrain_float1(Nu, -PI2, PI2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for level flight on current heading
void L1_Control::update_level_flight(const PStateData& psd)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = get_yaw_sensor(psd.psi);
    _nav_bearing = psd.psi;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

void L1_Control::update(const PStateData& psd, const GpsPosition& prev_WP, const GpsPosition& next_WP,
	float dist_min, float radius, int8_t loiter_direction, int32_t navigation_heading_cd)
{
    if (!cp->L1Control.enable)
		return;

	switch (cp->nav_mode)
	{
	case NAV_MODE::CIRCLE:
		update_loiter(psd, next_WP, radius, loiter_direction);
        break;
	case NAV_MODE::HOMING:
		update_waypoint(psd, prev_WP, next_WP, dist_min);
        break;
	case NAV_MODE::HEADING_HOLD:
		update_heading_hold(psd, navigation_heading_cd);
        break;
	default:
		break;
	}
}

/**
* Send data to communication chanel
*/
void L1_Control::sendTlmData(int formatL, int formatT, bool fLog, bool fComm)
{
	char buf[LINESIZE] = "";
	int bufFormat = -1;  // define which data format are in the buffer

	if (!cp->L1Control.enable) return;
	if (!params->tlmEnable && !params->logEnable) return;
	if (!fLog && !fComm) return;

	// log
	if (fLog && params->logEnable)
	{
		switch (formatL)
		{
		case TLM_CONTROLLER_LONG:
			if (!Base64::encode(&tlmData, sizeof(tlmData), buf, sizeof(buf))) return;
			bufFormat = TLM_CONTROLLER_LONG;
			Log.tlmPrint(TLM_CONTROLLER_LONG, buf, true, false);
			break;
		default: return;
		}
	}
	// tlm
	if (fComm && params->tlmEnable)
	{
		if (formatT == bufFormat)
		{
			// if telemetry format is the same as log format data is alrready prepared
			Log.tlmPrint(static_cast <enum TLM_FORMAT>(formatT), buf, false, true);
			return;
		}
		switch (formatT)
		{
		case TLM_CONTROLLER_LONG:
			if (!Base64::encode(&tlmData, sizeof(tlmData), buf, sizeof(buf))) return;
			Log.tlmPrint(TLM_CONTROLLER_LONG, buf, false, true);
			break;
		default: return;
		}
	}
}