/// @file    TECS.h
/// @brief   Combined Total Energy Speed & Height Control. This is a instance of an
/// SpdHgtControl class

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */
#pragma once

// #include <AP_Math/AP_Math.h>
// #include <AP_AHRS/AP_AHRS.h>
// #include <AP_Param/AP_Param.h>
// #include <AP_Vehicle/AP_Vehicle.h>
// #include <SpdHgtControl/SpdHgtControl.h>
// #include <AP_Landing/AP_Landing.h>
class TECSParams
{
public:
    TECSParams(void);
    float TECS_airspeed_max;
    float TECS_airspeed_min;
    float throttle_cruise;
    float throttle_max;
    float throttle_min;
    float throttle_slewrate;
    
    // TECS tuning parameters
    float spdCompFiltOmega;
    float maxClimbRate;
    float minSinkRate;
    float maxSinkRate;
    float timeConst;
    float ptchDamp;
    float thrDamp;
    float integGain;
    float vertAccLim;
    float rollComp;
    float spdWeight;
    float pitch_max;        // deg
    float pitch_min;        // deg
    float pitch_ff_v0;
    float pitch_ff_k;

    bool logEnable;
    bool tlmEnable;
    void setDefault();
};

class TECS : public SpdHgtControl {
public:
    TECSParams* params;
	PidTlmData tlmData;
	FPRealData::TECSControllerProperties* cp;

	TECS(ControllerID id, FPRealData::TECSControllerProperties* cp0, TECSParams *param0);

    /* Do not allow copies */
    TECS(const TECS &other) = delete;
    TECS &operator=(const TECS&) = delete;

    // Update of the estimated height and height rate internal state
    // Update of the inertial speed rate internal state
    // Should be called at 50Hz or greater
    void update_50hz(const PStateData& psd) override;

    // Update the control loop calculations
    void update_pitch_throttle(int32_t hgt_dem_cm,
                               int32_t EAS_dem_cm,
                               float load_factor, const PStateData& psd) override;
    // demanded throttle in percentage
    // should return -100 to 100, usually positive unless reverse thrust is enabled via _THRminf < 0
    int32_t get_throttle_demand(void) override {
        return int32_t(_throttle_dem * 100.0f);
    }

    // demanded pitch angle in centi-degrees
    // should return between -9000 to +9000
    int32_t get_pitch_demand(void) override {
        return int32_t(_pitch_dem * 5729.5781f);
    }

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) override {
        return _vel_dot;
    }

    // return current target airspeed
    float get_target_airspeed(void) const override {
        return _TAS_dem_adj;
    }

    // return maximum climb rate
    float get_max_climbrate(void) const override {
        return params->maxClimbRate;
    }

    // added to let SoaringContoller reset pitch integrator to zero
    void reset_pitch_I(void) override {
        _integSEB_state = 0.0f;
    }

    // return height rate demand, in m/s
    float get_height_rate_demand(void) const {
        return _hgt_rate_dem;
    }

    // set path_proportion
    void set_path_proportion(float path_proportion) override {
        _path_proportion = Numbers::constrain_float1(path_proportion, 0.0f, 1.0f);
    }

    // set soaring flag
    void set_gliding_requested_flag(bool gliding_requested) override {
        _flags.gliding_requested = gliding_requested;
    }

    // set propulsion failed flag
    void set_propulsion_failed_flag(bool propulsion_failed) override {
        _flags.propulsion_failed = propulsion_failed;
    }


    // set pitch max limit in degrees
    // void set_pitch_max_limit(int8_t pitch_limit) {
    //     _pitch_max_limit = pitch_limit;
    // }

    // force use of synthetic airspeed for one loop
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed_once = true;
    }

    // reset on next loop
    void reset(void) override {
        _need_reset = true;
    }

    // this supports the TECS_* user settable parameters
    //static const struct AP_Param::GroupInfo var_info[];

private:
    void init(ControllerID id, FPRealData::TECSControllerProperties* cp0, TECSParams* param0);
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;

    enum {
        OPTION_GLIDER_ONLY=(1<<0),
    };

    // temporary _pitch_max_limit. Cleared on each loop. Clear when >= 90
    // int8_t _pitch_max_limit = 90;
    
    // current height estimate (above field elevation)
    float _height;

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // estimated climb rate (m/s)
    float _climb_rate;

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;           // m/s

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // Integrator state 6 - pitch integrator
    float _integSEB_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;

    // Equivalent airspeed
    float _EAS;         // m/s

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current true airspeed demand
    float _TAS_dem;

    // Equivalent airspeed demand
    float _EAS_dem;

    // height demands
    float _hgt_dem;
    float _hgt_dem_in_old;
    float _hgt_dem_adj;
    float _hgt_dem_adj_last;
    float _hgt_rate_dem;
    float _hgt_dem_prev;
    float _land_hgt_dem;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;

    // Total energy rate filter state
    float _STEdotErrLast;

    struct flags {
        // Underspeed condition
        bool underspeed:1;

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;

        // true if the soaring feature has requested gliding flight
        bool gliding_requested:1;

        // true when we are in gliding flight, in one of three situations;
        //   - THR_MAX=0
        //   - gliding has been requested e.g. by soaring feature
        //   - engine failure detected (detection not implemented currently)
        bool is_gliding:1;

        // true if a propulsion failure is detected.
        bool propulsion_failed:1;
    };
    union {
        struct flags _flags;
        uint8_t _flags_byte;
    };

    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // auto mode flightstage
    //enum AP_Vehicle::FixedWing::FlightStage _flight_stage;

    // pitch demand before limiting
    float _pitch_dem_unc;

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

    // Specific energy error quantities
    float _STE_error;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // counter for demanded sink rate on land final
    uint8_t _flare_counter;

    // slew height demand lag filter value when transition to land
    float hgt_dem_lag_filter_slew;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;

    float _land_pitch_min = -90;

    // need to reset on next loop
    bool _need_reset;

    // internal variables to be logged
    struct {
        float SKE_weighting;
        float SPE_error;
        float SKE_error;
        float SEB_delta;
    } logging;

    INT8 _use_synthetic_airspeed;
    
    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed_once;
    
    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float load_factor, const PStateData& psd);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(const PStateData& psd);

    // Update Specific Energy Quantities
    void _update_energies(void);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(const PStateData& psd);

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states(const PStateData& psd);

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    // current time constant
    float timeConstant(void) const;
};
