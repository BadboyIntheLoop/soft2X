/**                                                                 
* @class FPRealData                                                                   
*     
* @brief Class representing the data made available by the subsystem based on the FlightPlanRealizer class.                                   
* This class is an interface between subsystems.
* 2008 Witold Kruczek @ Flytronic                                   
*/


#ifndef FPREALDATA_H
#define FPREALDATA_H

class SerialDeviceBase;

enum CONTROLLER_CHANNEL_ID 
{
	LAT_UNUSED_ID = 0,
	LAT_UNUSED_Q_ID,
	LAT_UNUSED_THETA_ID,
	LAT_UNUSED_VZ_ID,
	LAT_UNCHANGED_ID,

	LAT_HOLD_Q_ID,
	LAT_HOLD_THETA_ID,
	LAT_HOLD_VZ_ID,
	LAT_HOLD_SPD_ID,
	LAT_THETA_ALT_ID,
	LAT_VZ_ALT_ID,
	
	LON_UNUSED_ALR_ID,
	LON_UNUSED_P_ID,
	LON_UNUSED_PHI_ID,
	LON_UNUSED_TRACK_ID,
	LON_UNUSED_TRACKCORR_ID,
	LON_UNCHANGED_ALR_ID,

	LON_HOLD_P_ID,
	LON_HOLD_PHI_ID,
	LON_HOLD_TRACK_ID,
	LON_HOLD_CTRACK_ID,
	LON_HOLD_PSI_ID,
	LON_TRACK_WPT_ID,
	LON_CRC_WPT_ID,
	LON_TRACK_PATH_ID,
	
	LON_UNUSED_RDR_ID,
	LON_UNUSED_R_ID,
	LON_UNCHANGED_RDR_ID,

	LON_HOLD_R_ID,
	LON_HOLD_YACC_ID,
	LON_R_PSI_ID,
	LON_R_COORDEXP_ID,
	LON_R_TRACK_ID,
	
	SPD_UNUSED_THR_ID,
	SPD_UNUSED_FLP_ID,
	SPD_UNCHANGED_THR_ID,
	SPD_UNCHANGED_FLP_ID,

	SPD_THR_SPD_ID,
	SPD_THR_ALT_ID,
	SPD_FLP_SPD_ID,

	TECS_ID,

	L1_HEADING_HOLD_ID,
	L1_CIRCLE_ID,
	L1_HOMING_ID
};

enum class NAV_MODE{
	HEADING_HOLD,
	CIRCLE,
	HOMING
};

///  Class stores variables subset of the FlightPlanRealizer subsystem - outputs.
class OutputControls
{
public:
    //  Class stores output variables controlling the flight.
    //  Variables are logical character they are mapped for the defined servos in ServMan subsystem.
    //  All values are in range <-1;1> or <0;1>
    class FlightControl
    {
    public:
        FlightControl(void);

        int    time100; ///< Time from system has started [*100us]
        float  ailerons;	///< steerage in X axis (ailerons)
        float  elevator;	///< steerage in Y axis (elevator)
		float  leftElevon;	///< left elevon 
		float  rightElevon;	///< right elevon 
        float  rudder;  ///< steerage in Z axis (rudder)
        float  throttle;	///< throtle steerage <0;1>
        float  flaps;   ///< flaps steerage (could be mapped to ailerons) <0;1>
        float  airbrakes;   ///< airbreaks steerage <0;1>
        float  containerDrop;   ///< container drop control <0;1>
        float  butterfly;   ///< controlling of the "butterfly" mode <0;1>
        float  flapsAsAilerons; ///< steerage of the sterowanie aileron assisting <0;1>
		float  parachute; ///< antenna pantograph <0;1>
		float	customOutput[CUSTOM_OUTPUT_COUNT];  ///< controling an extra servos.

        // Send controlling data to FlightGear-a
        void sendToFg (SerialDeviceBase* device, bool bAilerons, bool bElevator, bool bRudder,
			bool bThrottle, bool bFlaps, bool _parachuteChanged, float measThrottle) const;
        
        void reset (void); ///<  reset all variables.
    };

    FlightControl           fCtrl;
};

/**  
*  Class stores base station data.
*/
class BaseStationData
{
public:
    BaseStationData()
        :  baseElvOffset(0.0f)
    {};

    GpsPosition basePosition;      
    float baseElvOffset;           ///<  Elevation offset of the base station from the elevation of the takeoff place.
};

/**  
* Class strores set of variables from the FlightPlanRealizer subsystem shared outside.
*/
class FPRealData
{
public:

    //  Class stores reference variables for the flight controllers.
    class FlightReference
    {
    public:
		class FlightReferenceLowLevel
		{
		public:
			float  P;				///< Angular velocity in X axis (roll rate) [rad/s]
			float  Q;				///< Angular velocity in Y axis (pitch rate) [rad/s]
			float  R;				///< Angular velocity in Z axis (yaw rate) [rad/s]
			float  theta;			///< Euler angle corresponding to "pitch" in straight flight.
			float  phi;				///< Euler angle corresponding to "roll" in straight flight.
			float  psi;				///< Euler angle corresponding to "yaw" in straight flight.
			float  f32VertSpeed;    ///< Vertical Speed [kph]

			FlightReferenceLowLevel(void)
				:	P(0.0f), Q(0.0f), R(0.0f), theta(0.0f), phi(0.0f), psi(0.0f), f32VertSpeed(0.0f) 
			{};
		};

        FlightReference(void)
            :	fRefLowLevel(), airspeed(0.0f), altitude(0.0f), track(0.0f), trackFromTo(0.0f)
            ,   crossTrackCorr(0.0f), circleRadius(0.0f), circleLeft(true)
            ,   circleMode(0), circleModePar(20.0f), glidePath(0.0f)
        {};

		FlightReferenceLowLevel fRefLowLevel;
        float  airspeed;		///< Airspeed [kph]
        float  altitude;		///< Altitude GPS [m]
		float  track;			///< Desire track.
        float  trackFromTo;		///< Track angle between two waypoints.
        GpsPosition wptFrom;
        GpsPosition wptTo;
		GpsPosition virtualWptTo;
        float  crossTrackCorr;  ///< Track angle (trackFromToWpt) correction associated with an error deviation from the track.
        float  circleRadius;	///< Circle radius during circle around wptTo waypoint.
        bool   circleLeft;		///< Direction of circle. true = left.
        int    circleMode;		///< Circle algorithm (0-old, 1-new)
        float  circleModePar;   ///< Current circle algorithm parameter (distance from circumference switching mode)
        float  glidePath;		///< Plane axcellence in given conditions.
    };

	/**
	* Class stores reference variables for the flight controllers.
	* Variables are automatic calculated in every  cycle and are not controlled by controllers outputs.
	*/
    class FlightReferenceAut
    {
    public:
        FlightReferenceAut(void)
            :	coordR(0.0f), trackToWpt(0.0f), trackError(0.0f), psiError(0.0f), crossTrackError(0.0f), speedFactor(1.0f), glidePath(0.0f)
        {};

        float  coordR;			///< Angular velocity in Z axis calculated for preserving coordination in the turn.
        float  trackToWpt;		///< Track angle from current position to the wptTo waypoint.
        float  trackError;		///< The difference between a predetermined angle and the current track taking account a discontinuity at the point 0.
        float  psiError;		///< The difference between a predetermined angle and the current psi angle taking account a discontinuity at the point 0.
        float  crossTrackError; ///< Deviation from track measured distance from trackFromToWpt line.
        float  speedFactor;		///< The gain factor depends on speed.
        float  glidePath;		///< Distance to the wptTo point an altitude quotient.
    };


    /**
    *  Class to store single controller properties.
	*/
    class ControllerProperties
    {
    public:
        ControllerProperties(void);

        bool   enable;		///< Enable controller flag.
        float  minValue;	///< Minium output value.
        float  maxValue;	///< Maximum output value.
        float  marginLow;   ///< Lower margin (2 state controller)
        float  marginHigh;  ///< Upper margin (2 state controller)
        bool   invMargins;  ///< Flag that reverse margins (turn on after upper margin has been excessed, turn off after lower margin has been excessed)
        int    bank;

        void   reset (void);

		void setControllerParams (INPUT float minVal, INPUT float maxVal, INPUT int bankVal = 0);
    };

	/**
	* Classes for interface controller propertise
	*/
	class ControllerPropertiesInterface
	{
	public:
		bool updateEnableBit (bool& newBit, bool& oldBit)
		{
			if (Bits::triggerHighEdge (newBit, oldBit))
			{
				disableControllers();
				newBit = true;
				updateEnableFlag ();
				return true;
			}
			return false;
		}
        
        virtual void updateEnableFlag (void) = 0;
		virtual void triggerEnableFlag (void) = 0;
		virtual void disableControllers (void) = 0;

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0) = 0;
		virtual ~ControllerPropertiesInterface() {};
	};

	/**
	* Classes for throtle channel
	* Thr_Alt, Thr_Spd
	*/
	class ThrottleControllerProperties: public ControllerPropertiesInterface
	{
	public:
		ThrottleControllerProperties (void) : Thr_Alt(), Thr_Speed() { enableFlagGrp.byte = 0; }
		ControllerProperties Thr_Alt;
		ControllerProperties Thr_Speed;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	/**
	* Classes for flap channel
	* Flp_Spd
	*/
	class FlapControllerProperties: public ControllerPropertiesInterface
	{
	public:
		FlapControllerProperties (void) : Flp_Speed() { enableFlagGrp.byte = 0; }
		ControllerProperties Flp_Speed;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	/**
	* Classes for elevator channel
	* Elv_Q, Q_Theta, Theta_Alt, Theta_Speed, Theta_VertSpd, VertSpd_Alt
	*/
	class ElvControllerProperties: public ControllerPropertiesInterface
	{
	public:
		ElvControllerProperties (void) : Elv_Q() { enableFlagGrp.byte = 0; }
		ControllerProperties Elv_Q;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class QControllerProperties: public ControllerPropertiesInterface
	{
	public:
		QControllerProperties (void) : Q_Theta() { enableFlagGrp.byte = 0; }
		ControllerProperties Q_Theta;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class ThetaControllerProperties: public ControllerPropertiesInterface
	{
	public:
		ThetaControllerProperties (void) : Theta_Alt(), Theta_Speed(), Theta_VertSpeed() { enableFlagGrp.byte = 0; }
		ControllerProperties Theta_Alt;
		ControllerProperties Theta_Speed;
		ControllerProperties Theta_VertSpeed;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class VertSpeedControllerProperties: public ControllerPropertiesInterface
	{
	public:
		VertSpeedControllerProperties (void) : VertSpeed_Alt() { enableFlagGrp.byte = 0; }
		ControllerProperties VertSpeed_Alt;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	/**
	* Classes for aileron channel
	* Alr_P, P_Phi, Phi_Track, Phi_Psi, Phi_CTrack, Track_Wpt, Track_TrackCorr, TrackCorr_Cte
	*/
	class AlrControllerProperties: public ControllerPropertiesInterface
	{
	public:
		AlrControllerProperties (void) : Alr_P() { enableFlagGrp.byte = 0; }
		ControllerProperties Alr_P;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class PControllerProperties: public ControllerPropertiesInterface
	{
	public:
		PControllerProperties (void) : P_Phi() { enableFlagGrp.byte = 0; }
		ControllerProperties P_Phi;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class PhiControllerProperties: public ControllerPropertiesInterface
	{
	public:
		PhiControllerProperties (void) : Phi_Track(), Phi_CTrack(), Phi_Psi() { enableFlagGrp.byte = 0; }
		ControllerProperties Phi_Track;
		ControllerProperties Phi_CTrack;
		ControllerProperties Phi_Psi;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class TrackControllerProperties: public ControllerPropertiesInterface
	{
	public:
		TrackControllerProperties (void) : Track_Wpt(), Track_TrackCorr() { enableFlagGrp.byte = 0; }
		ControllerProperties Track_Wpt;
		ControllerProperties Track_TrackCorr;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class L1ControllerProperties : public ControllerPropertiesInterface
	{
	public:
		L1ControllerProperties(void) : L1Control() { enableFlagGrp.byte = 0; }
		ControllerProperties L1Control;
		Byte enableFlagGrp;
		NAV_MODE nav_mode = NAV_MODE::CIRCLE;

		virtual void updateEnableFlag(void);
		virtual void triggerEnableFlag(void);
		virtual void disableControllers(void);

		virtual void setControllerParams(INPUT CONTROLLER_CHANNEL_ID ID,
										 INPUT float minVal,
										 INPUT float maxVal,
										 INPUT int bankVal = 0);
	};

	class TECSControllerProperties : public ControllerPropertiesInterface
	{
	public:
		TECSControllerProperties(void) : ThrTheta_TECS() { enableFlagGrp.byte = 0; }
		ControllerProperties ThrTheta_TECS;
		Byte enableFlagGrp;

		virtual void updateEnableFlag(void);
		virtual void triggerEnableFlag(void);
		virtual void disableControllers(void);

		virtual void setControllerParams(INPUT CONTROLLER_CHANNEL_ID ID,
			INPUT float minVal,
			INPUT float maxVal,
			INPUT int bankVal = 0);
	};

	class TrackCorrControllerProperties: public ControllerPropertiesInterface
	{
	public:
		TrackCorrControllerProperties (void) : TrackCorr_Cte() { enableFlagGrp.byte = 0; }
		ControllerProperties TrackCorr_Cte;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	/**
	* Classes for rudder channel
	* Rdr_R, Rdr_Yacc, R_Psi, R_CoordExp, R_Track
	*/
	class RudderControllerProperties: public ControllerPropertiesInterface
	{
	public:
		RudderControllerProperties (void) : Rdr_R(), Rdr_Yacc() { enableFlagGrp.byte = 0; }
		ControllerProperties Rdr_R;
		ControllerProperties Rdr_Yacc;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

	class RControllerProperties: public ControllerPropertiesInterface
	{
	public:
		RControllerProperties (void) : R_Psi(), R_CoordExp(), R_Track() { enableFlagGrp.byte = 0; }
		ControllerProperties R_Psi;
		ControllerProperties R_CoordExp;
		ControllerProperties R_Track;
		Byte enableFlagGrp;

		virtual void updateEnableFlag (void);
		virtual void triggerEnableFlag (void);
		virtual void disableControllers (void);

		virtual void setControllerParams (INPUT CONTROLLER_CHANNEL_ID ID,
										  INPUT float minVal,
										  INPUT float maxVal,
										  INPUT int bankVal = 0);
	};

    /**  
    *  Set of propertis of all controllers.
	*/
    class ControllerPropertiesSet
    {
    public:
		ThrottleControllerProperties  ThrCProp;
		FlapControllerProperties      FlpCProp;

		ElvControllerProperties       ElvCProp;
		QControllerProperties         QCProp;
		ThetaControllerProperties     ThetaCProp;
		VertSpeedControllerProperties VertSpeedCProp;
		
		AlrControllerProperties       AlrCProp;
		PControllerProperties         PCProp;
		PhiControllerProperties       PhiCProp;
		TrackControllerProperties     TrackCProp;
		TrackCorrControllerProperties TrackCorrCProp;
		L1ControllerProperties 		  L1CProp;
		TECSControllerProperties 	  TECSCProp;
		
		RudderControllerProperties    RudderCProp;
		RControllerProperties         RCProp;
        
        ControllerProperties
        Thr_Alt_2State,
        Btfly_Alt_2State,
        Abr_GPErr,
        FAlr_Alr,
		Btfly_GPath_2State,
		Theta_GPath_2State;

        void disableControllers (void);
		void disableLatControllers (void);
		void disableLonAlrControllers (void);
		void disableLonRdrControllers (void);
		void disableSpdControllers (void);
    };

    class OtherData
    {
    public:
        OtherData(void)
            :	doNotUseAirspeed(false)
        {};

        bool    doNotUseAirspeed;   // Flag forbid to use airspeed (flight with constant Theta angle)
    };

    //  Public fields.
    OutputControls          outCtrl;
    FlightReference         fRef;
    FlightReferenceAut      fRefAut;
    ControllerPropertiesSet ctrlProps;
    OtherData               other;
};


//  Structure packing 
#pragma pack(1)

/**  
*  Structure for the telemetric data associated with FlightPlanRealizer subsystem.
*   Must be packed. For portability types uC/OS-II system were used.
*/
class FPRealTlm
{
public:
    INT32U time;         // Relative time [ms]
    INT16U flags;        // Active flags (bits):
                         //     1-doNotBreak
                         //     2-doNotManualTurn
                         //     4-manualTurn
                         //     8-doNotCameraGuide
                         //    16-cameraGuide
                         //    32-observation
                         //    64-returnMode
                         //   128-prelandMode
                         //   256-approachMode
                         //   512-descentMode
                         //  1024-landMode
						 //  2048-abortLndMode
    INT8U  procId;       // ID of the currently executed procedure 
                         //   not implemented always 0
    INT8U  fpStatus;     // FPlan subsystems status.
                         //     0-undefined
                         //     1-Running
                         //     2-OnHold
                         //     3-Stopped
    INT8U  refAirspeed;  // Reference airspeed [kph]
    INT16S refAltitude;  // Reference altitude [m]
    INT16S fpItemId;     // Id of the current flight plan element (-1: item not set)

    FPRealTlm (void)
        :   time(0), flags(0), procId(0), fpStatus(0), refAirspeed(0), refAltitude(0), fpItemId(-1)
    {};

    void fillFrom (int time100us, bool doNotBreak, bool doNotManualTurn, bool manualTurn,
				   bool preland, bool approach, bool descent, bool land, bool abortLnd,
                   bool doNotCameraGuide, bool cameraGuide, bool observation, bool returnMode, int xProcId,
                   int xFpStatus, float xRefAirspeed, float xRefAltitude, int xFpItemId);
	void fillFrom(int time100us, bool doNotBreak, bool doNotManualTurn, bool manualTurn,
		bool lnd_preland, bool lnd_estime_heading, bool lnd_parachuting, bool lnd_abort, bool tkf_p1, bool tkf_p2, bool tkf_p3, 
		bool doNotCameraGuide, bool cameraGuide, bool observation, bool returnMode, int xProcId,
		int xFpStatus, float xRefAirspeed, float xRefAltitude, int xFpItemId);
};


class TlmCamShort
{
public:
    INT32U time;	///< Relative time [ms]
    INT16S pan;		///< Angle of camera set in pan (-180, +180, positive in right, 0 straight) [degrees * 100]
    INT16S tilt;	///< Angle of camera set in tilt (np. -30, +220, positive to down, 0 straight) [degrees * 100]
    INT16U fLen;	///< the focal length of lens (35mm) [mm * 10]
    INT8U  flags;   ///< Camera state flags    1 - Point To on     2 - Stabilization on      4 - IR on
    
    TlmCamShort (void)
        :   time(0), pan(0), tilt(0), fLen(0), flags(0)
    {};
};

// End of structure packing
#pragma pack()

#endif  //  FPREALDATA_H
