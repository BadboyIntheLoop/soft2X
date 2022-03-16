/**
* @class PID
* @brief Define PID controller
* 2008 Roman Filipowski @ Flytronic                                 
*/

#ifndef PID_H
#define PID_H

class PidModifierBase;

/// controllers ID used in the telemetry
enum class ControllerID
{
		
	
		ALR_P				=  0,  ///< Aileron from P (angular velocity in X axis)
		ELV_Q				=  1,  ///< Elevator from Q (angular velocity in Y axis)
		RDR_R				=  2,  ///< Rudder from R (angular velocity in Z axis)
		RDR_YACC			=  3,  ///< Rudder from Yacc
		THR_SPEED			=  4,  ///< Throttle from Airspeed
		THR_ALT				=  5,  ///< Throttle from Altitude
		THR_ALT_2STATE		=  6,  ///< Throttle from Altitude (2 state with histeresis)
		BTFLY_ALT_2STATE	=  7,  ///< Butterfly from Altitude (2 state with histeresis)
		FLP_SPEED			=  8,  ///< Flaps From Airspeed
		ABR_GPERR			=  9,  ///< Airbrakes from Glide Path Error
		FALR_ALR			= 10,  ///< Flaps As Aileron from Ailerons
		P_PHI				= 11,  ///< Angular velocity in X axis from Phi (roll)
		Q_THETA				= 12,  ///< Angular velocity in Y axis from Theta (pitch)
		R_PSI   			= 13,  ///< Angular velocity in Z axis from Psi (yaw)
		R_COORDEXP			= 14,  ///< Angular velocity in Z axis from  calculated value of turn coordination 
		R_TRACK			    = 15,  ///< Angular velocity in Z axis from track deviation.
		THETA_ALT			= 16,  ///< Theta from Altitude
		THETA_SPEED			= 17,  ///< Theta from Speed
		PHI_TRACK			= 18,  ///< Phi from Track - Angle roll from track deviation.
		PHI_CTRACK			= 19,  ///< Phi from CTrack - Angle roll from track deviation in circle.
		PHI_PSI				= 20,  ///< Phi from Psi - Angle roll from  from Psi error
		TRACK_TRACKCORR		= 21,  ///< Track from CrossTrack Error - track angle from track deviation
		TRACK_WPT			= 22,  ///< Track from Waypoint - track angle from angle to waypoint
		TRACKCORR_CTE		= 23,  ///< Track angle correction from desire track.
		BTFLY_GPATH_2STATE  = 24,  ///< The flap setting in Butterfly from GlidePath
		THETA_GPATH_2STATE  = 25,  ///< Theta in Butterfly from GlidePath
        THETA_VERTSPEED     = 26,  ///<
		VERTSPEED_ALT       = 27,
		PHI_L1       		= 28,
		TECS       			= 29,
};

/** 
* Controller parameters for BANK_COUNT banks
*/
class PidBank
{
public:

	float Kp;		///< Proporctional factor
	float Ti;		///< Doubled time
	float Tt;		///< Integrator's reset time for part of anti-windup
	float Td;		///< Lead time
	float N;		///< Limitation factor of part D value
	float wP;		///< "weight" for proportional part error: eP = wp * Vref - wx * V
	float wD;		///< "weight" for differential part error:  eD = wd * Vref - V
	float wX;       ///< possibility of omnit controller in the cascade by direct movement of Vref value to controller output (if: wp=1, wx=0, Kp=1)
	float sRef;		///< "slew rate" on reference value
	float sOut;		///< "slew rate" on output value
	float Imin;		///< Minimum value limitation of integral.
	float Imax;		///< Maximum value limitation of integral.

    PidBank(void);
    void setDefault(void);
};

//----------------------------------------------------------------------
// Classes determining controllers parameteres
//----------------------------------------------------------------------

class PidParams
{
public:
	PidBank bank[BANK_COUNT];	///< parameteres banks
	float maxKas;				///< Maximum correction factor from speed

	bool	tlmEnable;			///< Telemetry status (on/off)
	bool    logEnable;			///< Log write status (on/off)

    PidParams(void);
	void SetDefault(bool fromCtor=false);

private:
    static const float DEFAULT_MAX_KAS; ///< Maximum factor from airspeed 
};

class PidParamsSet
{
public:
	PidParams 
		Alr_P, Elv_Q, Rdr_R, Rdr_Yacc, Thr_Speed, Thr_Alt, Thr_Alt_2State, Btfly_Alt_2State, 
		Flp_Speed, Abr_GPErr, FAlr_Alr, P_Phi, Q_Theta, R_Psi, R_CoordExp, R_Track, Theta_Alt, Theta_Speed, 
		Phi_Track, Phi_CTrack, Phi_Psi, Track_TrackCorr, Track_Wpt, TrackCorr_Cte, ThrTheta_TECS,
		Btfly_GPath_2State, Theta_GPath_2State, Theta_VertSpeed, VertSpeed_Alt; 
	
	void SetDefault(void);
};

//----------------------------------------------------------------------
// Classes determining controllers state
//----------------------------------------------------------------------

class PidState
{
public:
	bool	enabled;		///< on/off
	float   output_1;       ///< previous output value
	float   output_1m;      ///< previous value  poprzednia warto�� output przed uwzgl�dnieniem slew-rate (regulator 2-stanowy)
	float	D_1;			///< previous value of part D
	float   eD_1;			///< previous value of error from part D (eD)
	float   ref_1;			///< previous reference value
	float	I;				///< integrator (part I)
	int     time100_1; 		///< previous sample time [s]
	bool	computed;		///< if controller made calculations
	bool	timeout;		///< if there was timeout in previous sample

    PidState(void)
        :   enabled(false), output_1(0.0f), output_1m(0.0f), D_1(0.0f), eD_1(0.0f), ref_1(0.0f), I(0.0f),
        time100_1(0), computed(false), timeout(false)
    {};
};

class PidStateSet
{
public:
	PidState 
		Alr_P, Elv_Q, Rdr_R, Rdr_Yacc, Thr_Speed, Thr_Alt, Thr_Alt_2State, Btfly_Alt_2State, 
		Flp_Speed, Abr_GPErr, FAlr_Alr, P_Phi, Q_Theta, R_Psi, R_CoordExp, R_Track, Theta_Alt, Theta_Speed, 
		Phi_Track, Phi_CTrack, Phi_Psi, Track_TrackCorr, Track_Wpt, TrackCorr_Cte, ThrTheta_TECS,
		Btfly_GPath_2State, Theta_GPath_2State, Theta_VertSpeed, VertSpeed_Alt; 
};

#pragma pack(1)

class PidTlmDataMin
{
public:
	// general data for T03 format
	INT8U id;
	INT32U time;
	float  input;
	float  output;
	float  reference;

    PidTlmDataMin (void)
        : id(0), time(0), input(0.0f), output(0.0f), reference(0.0f)
    {};
};

class PidTlmData
{
public:
	PidTlmDataMin tlm;
	// additional data for T04 format
	float P;
	float I;
	float D;

    PidTlmData (void)
        : P(0.0f), I(0.0f), D(0.0f)
    {};
};

#pragma pack()

/**
* Describes controller PID
*/
class Pid
{
public:
    enum PidMode
    {
        SIMPLE_PID  = 0,  ///< simplified controller PID
        NORMAL_PID  = 1,  ///< controller PID
        STATE_2H    = 2,  ///< 2-state controller with histeresis
        MULTI_STEP  = 3   ///< multi step controller
    };

    PidMode	mode;							///< controllers working mode

    FPRealData::ControllerProperties* cp;	///< pointer to the realizer parameters
    PidState*  state;						///< pointer to the controlers state variables
    PidParams* param;						///< pointer to controllers parameters

	PidTlmData tlmData;						///< telemetry data from last measurement

    Pid (ControllerID id, FPRealData::ControllerProperties *cp0, PidState *state0, 
		PidParams *param0, PidMode mode0, PidModifierBase *outModifier = NULL, PidModifierBase *refModifier = NULL)
    {
        init (id, cp0, state0, param0, mode0, outModifier, refModifier);
    }

	bool compute(int time100, float input, float reference, float &output, float Kas);
	void clearState();	///< reset controllers values
	void sendTlmData(int formatL, int formatT, bool fLog, bool fComm); ///< send telemetry to communication chanel and file log.

private:
    static const float MIN_TS;          ///< Minimum sampling time value.
    static const float MAX_TS;          ///< Maximum sampling time value.

    PidModifierBase* _outModifier;
    PidModifierBase* _refModifier;

    void init(ControllerID id, FPRealData::ControllerProperties *cp0, PidState *state0, 
        PidParams *param0, PidMode mode0, PidModifierBase *outModifier = NULL, PidModifierBase *refModifier = NULL);
};


#endif // PID_H
