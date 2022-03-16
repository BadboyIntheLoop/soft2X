#pragma once
//#include <Navigation.h>
//#include <PilotIncludes.h>
/// @file    L1_Control.h
/// @brief   L1 Control algorithm. This is a instance of an
/// Navigation class

/*
 *  Modified to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */
class L1Params
{	
public:
    L1Params(void);
    float L1_period;         // Attitude control input time constant: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    float L1_damping;        // default maximum acceleration for roll axis in degrees/sec/sec
    float L1_xtrack_i_gain;  // default maximum acceleration for pitch axis in degrees/sec/sec
    float loiter_bank_limit; // default maximum acceleration for yaw axis in degrees/sec/sec
    bool logEnable;
    bool tlmEnable;
    void setDefault();
};
class L1_Control : public Navigation {
public:
    L1Params* params;
	PidTlmData tlmData;
	FPRealData::L1ControllerProperties* cp;

	L1_Control(ControllerID id, FPRealData::L1ControllerProperties *cp0, L1Params *param0);

    /* Do not allow copies */
    L1_Control(const L1_Control &other) = delete;
    L1_Control &operator=(const L1_Control&) = delete;

    /* see Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(const PStateData& psd) const override;
    float nav_roll_rad(const PStateData& psd) const override;
    float lateral_acceleration(void) const override;

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const override;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const override;

    float crosstrack_error(void) const override { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const override { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const override;
    float turn_distance(float wp_radius) const override;
    float turn_distance(float wp_radius, float turn_angle) const override;
    float loiter_radius (const float loiter_radius) const override;
    void update_waypoint(const PStateData& psd, const GpsPosition& prev_WP, const GpsPosition& next_WP, float dist_min = 0.0f) override;
    void update_loiter(const PStateData& psd, const GpsPosition &center_WP, float radius, int8_t loiter_direction) override;
    void update_heading_hold(const PStateData& psd, int32_t navigation_heading_cd) override;
    void update_level_flight(const PStateData& psd) override;
    void update(const PStateData& psd, const GpsPosition& prev_WP, const GpsPosition& next_WP,
        float dist_min = 0.0f, float radius = 200.0f, int8_t loiter_direction = 1, int32_t navigation_heading_cd = 0) override;
    bool reached_loiter_target(void) override;

    // set the default NAVL1_PERIOD
    //void set_default_period(float period) {
    //    _L1_period.set_default(period);
    //}

    void set_data_is_stale(void) override {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const override {
        return _data_is_stale;
    }

    // this supports the NAVl1_* user settable parameters
    //static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) override {
        _reverse = reverse;
    }

    void sendTlmData(int formatL, int formatT, bool fLog, bool fComm);
private:
    // PStateData &_psd;
    // PStateHealth& _psh;
    // reference to the AHRS object
    // AP_AHRS &_ahrs;

    // pointer to the SpdHgtControl object
    // const AP_SpdHgtControl *_spdHgtControl;

    // lateral acceration in m/s required to fly to the
    // L1 reference point (+ve to right)
    float _latAccDem;

    // L1 tracking distance in meters which is dynamically updated
    float _L1_dist;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // bearing angle (radians) to L1 point
    float _nav_bearing;

    // bearing error angle (radians) +ve to left of track
    float _bearing_error;

    // crosstrack error in meters
    float _crosstrack_error;

    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

    // L1 tracking loop period (sec)
    float _L1_period;
    // L1 tracking loop damping ratio
    float _L1_damping;

    // previous value of cross-track velocity
    float _last_Nu;

    void init(ControllerID id, FPRealData::L1ControllerProperties *cp0, L1Params *param0);

    // prevent indecision in waypoint tracking
    void _prevent_indecision(const PStateData& psd, float &Nu);

    // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
    // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
    float _L1_xtrack_i = 0;
    float _L1_xtrack_i_gain;
    float _L1_xtrack_i_gain_prev = 0;
    uint32_t _last_update_waypoint_us;
    bool _data_is_stale = true;

    float _loiter_bank_limit;
    int32_t _count = 0;
    bool _reverse = false;
    float get_yaw(const PStateData& psd);
    int32_t get_yaw_sensor(float psi) const;
};
