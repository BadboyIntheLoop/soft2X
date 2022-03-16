#pragma once
#include <PilotIncludes.h>

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

TECS::TECS(ControllerID id, FPRealData::TECSControllerProperties *cp0, TECSParams *param0) :
    _integTHR_state(0.0f)
    , _integSEB_state(0.0f)
    , _last_throttle_dem(0.005f)
    , _pitch_dem(0.0f)
    , _throttle_dem(0.0f)
    , _DT(0.1f)
{
    init(id, cp0, param0);
};

void TECS::init(ControllerID id, FPRealData::TECSControllerProperties *cp0, TECSParams *param0)
{
	tlmData.tlm.id = static_cast<INT8U>(id);
	params = param0;
	cp = cp0;
}

TECSParams::TECSParams()
{
	setDefault();
}

void TECSParams::setDefault()
{
    TECS_airspeed_max = 50.0f;
    TECS_airspeed_min = 15.0f;
    throttle_cruise = 50.0f;
    throttle_max = 100.0f;
    throttle_min = 0.0f;
    throttle_slewrate = 0.0f;

    spdCompFiltOmega = 0.5f;   // 0.5 - 2
    maxClimbRate = 5.0f;       // 0.1 - 20
    minSinkRate = 2.0f;        // 0.1 - 10
    maxSinkRate = 5.0f;        // 0 - 20
    timeConst = 7.0f;          // 3 - 10
    ptchDamp = 0.1f;           // 0.1 - 1
    thrDamp = 0.5f;            // 0.1 - 1
    integGain = 0.2f;          // 0 - 0.5
    vertAccLim = 7.0f;         // 1 - 10
    rollComp = 10.0f;          // 5 - 30
    spdWeight = 1.0f;          // 0 - 2
    pitch_max = 15;            // 0 - 45
    pitch_min = -25;           // -45 - 0
    pitch_ff_v0 = 22.0f;       // 5 - 50
    pitch_ff_k = 0.0f;         // -5 - 0
    
	logEnable = false;
	tlmEnable = false;
}

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

void TECS::update_50hz(const PStateData& psd)
{
    // Implement third order complementary filter for height and height rate
    // estimated height rate = _climb_rate
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    /*
      if we have a vertical position estimate from the EKF then use
      it, otherwise use barometric altitude
     */
    if (!cp->ThrTheta_TECS.enable)
		return;

    // _ahrs.get_relative_position_D_home(_height); [float meter]
    _height = psd.altitude;

    // Calculate time in seconds since last update
    uint32_t now = psd.time100 * 100;

    float DT = (now - _update_50hz_last_usec) * 1.0e-6f;
    if (DT > 1.0f) {
        _climb_rate = 0.0f;
        _height_filter.dd_height = 0.0f;
        DT            = 0.02f; // when first starting TECS, use a
        // small time constant
    }
    _update_50hz_last_usec = now;

    // Use inertial nav verical velocity and height if available
    _climb_rate = -psd.velned.z;

    // Update and average speed rate of change
    // Get DCM
    Matrix3f rotMat;
    rotMat.from_euler(psd.phi, psd.theta, psd.psi);
    // Calculate speed rate of change
    float temp = rotMat.c.x * G + psd.accX * G;
    // take 5 point moving average
    _vel_dot = _vdot_filter.apply(temp);

}

void TECS::_update_speed(float load_factor, const PStateData& psd)
{
    // Calculate time in seconds since last update
    uint32_t now = psd.time100 * 100;
    float DT = (now - _update_speed_last_usec) * 1.0e-6f;
    _update_speed_last_usec = now;

    // Convert equivalent airspeeds to true airspeeds

    // float EAS2TAS = _ahrs.get_EAS2TAS();
    float EAS2TAS = 1.0f;
    _TAS_dem = _EAS_dem * EAS2TAS;
    _TASmax   = params->TECS_airspeed_max * EAS2TAS;
    _TASmin   = params->TECS_airspeed_min * EAS2TAS;

    if (_TASmax < _TASmin) {
        _TASmax = _TASmin;
    }
    if (_TASmin > _TAS_dem) {
        _TASmin = _TAS_dem;
    }
    _EAS = psd.airspeed * KPH_2_MS;
    // Reset states of time since last update is too large
    if (DT > 1.0f) {
        _TAS_state = (_EAS * EAS2TAS);
        _integDTAS_state = 0.0f;
        DT            = 0.1f; // when first starting TECS, use a
        // small time constant
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _TAS_state
    float aspdErr = (_EAS * EAS2TAS) - _TAS_state;
    float integDTAS_input = aspdErr * params->spdCompFiltOmega * params->spdCompFiltOmega;
    // Prevent state from winding up
    if (_TAS_state < 3.1f) {
        integDTAS_input = Numbers::maxF(integDTAS_input , 0.0f);
    }
    _integDTAS_state = _integDTAS_state + integDTAS_input * DT;
    float TAS_input = _integDTAS_state + _vel_dot + aspdErr * params->spdCompFiltOmega * 1.4142f;
    _TAS_state = _TAS_state + TAS_input * DT;
    // limit the airspeed to a minimum of 3 m/s
    _TAS_state = Numbers::maxF(_TAS_state, 3.0f);

}

void TECS::_update_speed_demand(void)
{
    // Set the airspeed demand to the minimum value if an underspeed condition exists
    // or a bad descent condition exists
    // This will minimise the rate of descent resulting from an engine failure,
    // enable the maximum climb rate to be achieved and prevent continued full power descent
    // into the ground due to an unachievable airspeed value
    if ((_flags.badDescent) || (_flags.underspeed))
    {
        _TAS_dem     = _TASmin;
    }

    // Constrain speed demand, taking into account the load factor
    _TAS_dem = Numbers::constrain_float1(_TAS_dem, _TASmin, _TASmax);

    // calculate velocity rate limits based on physical performance limits
    // provision to use a different rate limit if bad descent or underspeed condition exists
    // Use 50% of maximum energy rate to allow margin for total energy contgroller
    const float velRateMax = 0.5f * _STEdot_max / _TAS_state;
    const float velRateMin = 0.5f * _STEdot_min / _TAS_state;
    const float TAS_dem_previous = _TAS_dem_adj;

    // assume fixed 10Hz call rate
    const float dt = 0.1f;

    // Apply rate limit
    if ((_TAS_dem - TAS_dem_previous) > (velRateMax * dt))
    {
        _TAS_dem_adj = TAS_dem_previous + velRateMax * dt;
        _TAS_rate_dem = velRateMax;
    }
    else if ((_TAS_dem - TAS_dem_previous) < (velRateMin * dt))
    {
        _TAS_dem_adj = TAS_dem_previous + velRateMin * dt;
        _TAS_rate_dem = velRateMin;
    }
    else
    {
        _TAS_rate_dem = (_TAS_dem - TAS_dem_previous) / dt;
        _TAS_dem_adj = _TAS_dem;
    }
    // Constrain speed demand again to protect against bad values on initialisation.
    _TAS_dem_adj = Numbers::constrain_float1(_TAS_dem_adj, _TASmin, _TASmax);
}

void TECS::_update_height_demand(void)
{
    // Apply 2 point moving average to demanded height
    _hgt_dem = 0.5f * (_hgt_dem + _hgt_dem_in_old);
    _hgt_dem_in_old = _hgt_dem;

    float max_sink_rate = params->maxSinkRate;

    // Limit height rate of change
    if ((_hgt_dem - _hgt_dem_prev) > (params->maxClimbRate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev + params->maxClimbRate * 0.1f;
    }
    else if ((_hgt_dem - _hgt_dem_prev) < (-max_sink_rate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev - max_sink_rate * 0.1f;
    }
    _hgt_dem_prev = _hgt_dem;

    // Apply first order lag to height demand
    _hgt_dem_adj = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;

    _hgt_rate_dem = (_hgt_dem_adj - _hgt_dem_adj_last) / 0.1f;
    _flare_counter = 0;

    // for landing approach we will predict ahead by the time constant
    // plus the lag produced by the first order filter. This avoids a
    // lagged height demand while constantly descending which causes
    // us to consistently be above the desired glide slope. This will
    // be replaced with a better zero-lag filter in the future.
    float new_hgt_dem = _hgt_dem_adj;
    hgt_dem_lag_filter_slew = 0;
    _hgt_dem_adj_last = _hgt_dem_adj;
    _hgt_dem_adj = new_hgt_dem;
}


void TECS::_detect_underspeed(const PStateData& psd)
{
    // see if we can clear a previous underspeed condition. We clear
    // it if we are now more than 15% above min speed, and haven't
    // been below min speed for at least 3 seconds.
    uint32_t now = psd.time100 * 100;
    if (_flags.underspeed &&
        _TAS_state >= _TASmin * 1.15f &&
        now - _underspeed_start_ms > 3000U) {
        _flags.underspeed = false;
    }

    if (((_TAS_state < _TASmin * 0.9f) &&
        (_throttle_dem >= _THRmaxf * 0.95f) &&
        true) ||
        ((_height < _hgt_dem_adj) && _flags.underspeed))
    {
        _flags.underspeed = true;
        if (_TAS_state < _TASmin * 0.9f) {
            // reset start time as we are still underspeed
            _underspeed_start_ms = now;
        }
    }
    else
    {
        // this clears underspeed if we reach our demanded height and
        // we are either below 95% throttle or we above 90% of min
        // airspeed
        _flags.underspeed = false;
    }
}

void TECS::_update_energies(void)
{
    // Calculate specific energy demands
    _SPE_dem = _hgt_dem_adj * G;
    _SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

    // Calculate specific energy rate demands
    _SPEdot_dem = _hgt_rate_dem * G;
    _SKEdot_dem = _TAS_state * _TAS_rate_dem;

    // Calculate specific energy
    _SPE_est = _height * G;
    _SKE_est = 0.5f * _TAS_state * _TAS_state;

    // Calculate specific energy rate
    _SPEdot = _climb_rate * G;
    _SKEdot = _TAS_state * _vel_dot;

}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */
float TECS::timeConstant(void) const
{
    if (params->timeConst < 0.1f) {
        return 0.1f;
    }
    return params->timeConst;
}

/*
  calculate throttle demand - airspeed enabled case
 */
void TECS::_update_throttle_with_airspeed(const PStateData& psd)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    float SPE_err_max = 0.5f * _TASmax * _TASmax - _SKE_dem;
    float SPE_err_min = 0.5f * _TASmin * _TASmin - _SKE_dem;
    
    // Calculate total energy error
    _STE_error = Numbers::constrain_float1((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
    float STEdot_dem = Numbers::constrain_float1((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    STEdot_error = 0.2f*STEdot_error + 0.8f*_STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
    if (_flags.underspeed)
    {
        _throttle_dem = 1.0f;
    }
    else if (_flags.is_gliding)
    {
        _throttle_dem = 0.0f;
    }
    else
    {
        // Calculate gain scaler from specific energy error to throttle
        // (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf) is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        float K_STE2Thr = 1 / (timeConstant() * (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf));

        // Calculate feed-forward throttle
        float ff_throttle = 0;
        float nomThr = params->throttle_cruise * 0.01f;
        Matrix3f rotMat;
        rotMat.from_euler(psd.phi, psd.theta, psd.psi);
        // Use the demanded rate of change of total energy as the feed-forward demand, but add
        // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
        // drag increase during turns.
        float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
        STEdot_dem = STEdot_dem + params->rollComp * (1.0f/Numbers::constrain_float1(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);
        ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

        // Calculate PD + FF throttle
        float throttle_damp = params->thrDamp;

        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        // Constrain throttle demand
        _throttle_dem = Numbers::constrain_float1(_throttle_dem, _THRminf, _THRmaxf);

        float THRminf_clipped_to_zero = Numbers::constrain_float1(_THRminf, 0, _THRmaxf);

        // Rate limit PD + FF throttle
        // Calculate the throttle increment from the specified slew time
        if (params->throttle_slewrate != 0) {
            float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * params->throttle_slewrate * 0.01f;

            _throttle_dem = Numbers::constrain_float1(_throttle_dem,
                                            _last_throttle_dem - thrRateIncr,
                                            _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        // Calculate integrator state upper and lower limits
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        float maxAmp = 0.5f*(_THRmaxf - THRminf_clipped_to_zero);
        float integ_max = Numbers::constrain_float1((_THRmaxf - _throttle_dem + 0.1f),-maxAmp,maxAmp);
        float integ_min = Numbers::constrain_float1((_THRminf - _throttle_dem - 0.1f),-maxAmp,maxAmp);

        // Calculate integrator state, constraining state
        // Set integrator to a max throttle value during climbout
        _integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;
        _integTHR_state = Numbers::constrain_float1(_integTHR_state, integ_min, integ_max);

        // Sum the components.
        _throttle_dem = _throttle_dem + _integTHR_state;
    }

    // Constrain throttle demand
    _throttle_dem = Numbers::constrain_float1(_throttle_dem, _THRminf, _THRmaxf);
}

float TECS::_get_i_gain(void)
{
    float i_gain = params->integGain;
    return i_gain;
}

void TECS::_detect_bad_descent(void)
{
    // Detect a demanded airspeed too high for the aircraft to achieve. This will be
    // evident by the the following conditions:
    // 1) Underspeed protection not active
    // 2) Specific total energy error > 200 (greater than ~20m height error)
    // 3) Specific total energy reducing
    // 4) throttle demand > 90%
    // If these four conditions exist simultaneously, then the protection
    // mode will be activated.
    // Once active, the following condition are required to stay in the mode
    // 1) Underspeed protection not active
    // 2) Specific total energy error > 0
    // This mode will produce an undulating speed and height response as it cuts in and out but will prevent the aircraft from descending into the ground if an unachievable speed demand is set
    float STEdot = _SPEdot + _SKEdot;
    if (((!_flags.underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) && (_throttle_dem >= _THRmaxf * 0.9f)) || (_flags.badDescent && !_flags.underspeed && (_STE_error > 0.0f))) && !_flags.is_gliding)
    {
        _flags.badDescent = true;
    }
    else
    {
        _flags.badDescent = false;
    }
}

void TECS::_update_pitch(void)
{
    // Calculate Speed/Height Control Weighting
    // This is used to determine how the pitch control prioritises speed and height control
    // A weighting of 1 provides equal priority (this is the normal mode of operation)
    // A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
    // A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected. In this instance, if airspeed
    // rises above the demanded value, the pitch angle will be increased by the TECS controller.
    float SKE_weighting = Numbers::constrain_float1(params->spdWeight, 0.0f, 2.0f);
    logging.SKE_weighting = SKE_weighting;
    
    float SPE_weighting = 2.0f - SKE_weighting;

    // Calculate Specific Energy Balance demand, and error
    float SEB_dem      = _SPE_dem * SPE_weighting - _SKE_dem * SKE_weighting;
    float SEBdot_dem   = _SPEdot_dem * SPE_weighting - _SKEdot_dem * SKE_weighting;
    float SEB_error    = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * SKE_weighting);
    float SEBdot_error = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * SKE_weighting);

    logging.SKE_error = _SKE_dem - _SKE_est;
    logging.SPE_error = _SPE_dem - _SPE_est;
    
    // Calculate integrator state, constraining input if pitch limits are exceeded
    float integSEB_input = SEB_error * _get_i_gain();
    if (_pitch_dem > _PITCHmaxf)
    {
        integSEB_input = Numbers::minF(integSEB_input, _PITCHmaxf - _pitch_dem);
    }
    else if (_pitch_dem < _PITCHminf)
    {
        integSEB_input = Numbers::maxF(integSEB_input, _PITCHminf - _pitch_dem);
    }
    float integSEB_delta = integSEB_input * _DT;

    // Apply max and min values for integrator state that will allow for no more than
    // 5deg of saturation. This allows for some pitch variation due to gusts before the
    // integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
    // During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle
    // demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
    // integrator has to catch up before the nose can be raised to reduce speed during climbout.
    // During flare a different damping gain is used
    float gainInv = (_TAS_state * timeConstant() * G);
    float temp = SEB_error + 0.5f*SEBdot_dem * timeConstant();

    float pitch_damp = params->ptchDamp;
    temp += SEBdot_error * pitch_damp;

    // if (_flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || _flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
    //     temp += _PITCHminf * gainInv;
    // }
    float integSEB_min = (gainInv * (_PITCHminf - 0.0783f)) - temp;
    float integSEB_max = (gainInv * (_PITCHmaxf + 0.0783f)) - temp;
    float integSEB_range = integSEB_max - integSEB_min;

    logging.SEB_delta = integSEB_delta;
    
    // don't allow the integrator to rise by more than 20% of its full
    // range in one step. This prevents single value glitches from
    // causing massive integrator changes. See Issue#4066
    integSEB_delta = Numbers::constrain_float1(integSEB_delta, -integSEB_range*0.1f, integSEB_range*0.1f);

    // prevent the constraint on pitch integrator _integSEB_state from
    // itself injecting step changes in the variable. We only want the
    // constraint to prevent large changes due to integSEB_delta, not
    // to cause step changes due to a change in the constrain
    // limits. Large steps in _integSEB_state can cause long term
    // pitch changes
    integSEB_min = Numbers::minF(integSEB_min, _integSEB_state);
    integSEB_max = Numbers::maxF(integSEB_max, _integSEB_state);

    // integrate
    _integSEB_state = Numbers::constrain_float1(_integSEB_state + integSEB_delta, integSEB_min, integSEB_max);

    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (temp + _integSEB_state) / gainInv;


    // Add a feedforward term from demanded airspeed to pitch.
    if (_flags.is_gliding) {
        _pitch_dem_unc += (_TAS_dem_adj - params->pitch_ff_v0) * params->pitch_ff_k;
    }

    // Constrain pitch demand
    _pitch_dem = Numbers::constrain_float1(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit
    float ptchRateIncr = _DT * params->vertAccLim / _TAS_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr)
    {
        _pitch_dem = _last_pitch_dem + ptchRateIncr;
    }
    else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr)
    {
        _pitch_dem = _last_pitch_dem - ptchRateIncr;
    }

    // re-constrain pitch demand
    _pitch_dem = Numbers::constrain_float1(_pitch_dem, _PITCHminf, _PITCHmaxf);

    _last_pitch_dem = _pitch_dem;
}

void TECS::_initialise_states(const PStateData& psd)
{
    // Initialise states and variables if DT > 1 second or in climbout
    if (_DT > 1.0f || _need_reset)
    {
        _integTHR_state      = 0.0f;
        _integSEB_state      = 0.0f;
        _last_throttle_dem = params->throttle_cruise * 0.01f;
        _last_pitch_dem    = psd.theta;
        _hgt_dem_adj_last  = psd.altitude;
        _hgt_dem_adj       = _hgt_dem_adj_last;
        _hgt_dem_prev      = _hgt_dem_adj_last;
        _hgt_dem_in_old    = _hgt_dem_adj_last;
        _TAS_dem_adj       = _TAS_dem;
        _flags.underspeed        = false;
        _flags.badDescent        = false;
        _flags.reached_speed_takeoff = false;
        _DT                = 0.1f; // when first starting TECS, use a
        // small time constant
        _need_reset = false;
    }
}

void TECS::_update_STE_rate_lim(void)
{
    // Calculate Specific Total Energy Rate Limits
    // This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = params->maxClimbRate * G;
    _STEdot_min = - params->minSinkRate * G;
}


void TECS::update_pitch_throttle(int32_t hgt_dem_cm,
                                    int32_t EAS_dem_cm,
                                    float load_factor,
                                    const PStateData& psd)
{
    if (!cp->ThrTheta_TECS.enable)
        return;
    // Calculate time in seconds since last update
    uint32_t now = psd.time100 * 100;
    _DT = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _update_pitch_throttle_last_usec = now;

    // Convert inputs
    _hgt_dem = hgt_dem_cm * 0.01f;
    _EAS_dem = EAS_dem_cm * 0.01f;

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(load_factor, psd);

    _THRmaxf  = params->throttle_max * 0.01f;
    _THRminf  = params->throttle_min * 0.01f;

    // min of 1% throttle range to prevent a numerical error
    _THRmaxf = Numbers::maxF(_THRmaxf, _THRminf+0.01f);

    // work out the maximum and minimum pitch
    // if TECS_PITCH_{MAX,MIN} isn't set then use
    // LIM_PITCH_{MAX,MIN}. Don't allow TECS_PITCH_{MAX,MIN} to be
    // larger than LIM_PITCH_{MAX,MIN}
    _PITCHmaxf = Numbers::maxF(params->pitch_max, 0.0f);
    _PITCHminf = Numbers::minF(params->pitch_min, 0.0f);
    
    // convert to Numbers::radians
    _PITCHmaxf = Numbers::radians(_PITCHmaxf);
    _PITCHminf = Numbers::radians(_PITCHminf);

    // don't allow max pitch to go below min pitch
    _PITCHmaxf = Numbers::maxF(_PITCHmaxf, _PITCHminf);

    // initialise selected states and variables if DT > 1 second or in climbout
    _initialise_states(psd);

    // Calculate Specific Total Energy Rate Limits
    _update_STE_rate_lim();

    // Calculate the speed demand
    _update_speed_demand();

    // Calculate the height demand
    _update_height_demand();

    // Detect underspeed condition
    _detect_underspeed(psd);

    // Calculate specific energy quantitiues
    _update_energies();

    // Calculate throttle demand - use simple pitch to throttle if no
    // airspeed sensor.
    // Note that caller can demand the use of
    // synthetic airspeed for one loop if needed. This is required
    // during QuadPlane transition when pitch is constrained
    _update_throttle_with_airspeed(psd);

    // Detect bad descent due to demanded airspeed being too high
    _detect_bad_descent();

    // Calculate pitch demand
    _update_pitch();
}