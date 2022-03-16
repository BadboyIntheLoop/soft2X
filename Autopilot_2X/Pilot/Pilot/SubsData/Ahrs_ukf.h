//=====================================================================
//                                                                 
// Ahrs
//
//
// 2008 Grzegorz Tyma @ Flytronic
//=====================================================================

#ifndef AHRS_H
#define AHRS_H


class Ahrs
{
public:
    
    explicit Ahrs(int cyclesPerSecond);
	void ahrsMain(const struct Gauge::measurementT* measurement, const struct Gauge::magnetMeasurementT* magnetMeasurement, float airspeed, unsigned int time, int pressureSource, int magEnable, int flightMode, int version, int debugLevel, float groundSpeed, float track_deg, bool gpsGood);
	void ahrsMainFG(const struct Gauge::measurementT* measurement, float airspeed, unsigned int time, float phi, float theta, float psi, int version, float groundSpeed, float track, bool gpsGood);
#if (PILOT_TYPE == VUA_SC_6G)
	void estimateNED(const struct Gauge::measurementT* measurement, GpsPosition currentPosition, float amsl, float northSpeed, float eastSpeed, bool update, bool simulation);
#endif
#if (PILOT_TYPE == VUA_SC_6G) && (USE_LASER == ENABLED)
	void estimateVertLaser(const struct Gauge::measurementT* measurement, float laserAlt);
#endif
	void lockUpdate(int seconds);
	void unlockUpdate(void);
    bool isUpdateLocked(void) const;

//struct contains data calculated by AHRS module
	struct ahrs_dataT{
		//unbiased rates
		float p;
		float q;
		float r;
		//eulers
		float phi;
		float theta;
		float psi;
		//derivatives
		float phiDot;
		float thetaDot;
		float psiDot;	
		INT8U ahrsUpdateFlag;
		//magnetic error
		float incdError;

		float f32EstVertSpeed;
		float f32EstAlt;
		float f32EstDeltaAlt;

#if (PILOT_TYPE == VUA_SC_6G) && (USE_LASER == ENABLED)
		float f32EstVertSpeedLaser;
		float f32EstAltLaser;
		float f32EstDeltaAltLaser;
#endif
#if (PILOT_TYPE == VUA_SC_6G)
		float f32EstEastSpeed;
		float f32EstEastPosition;
		float f32EstNorthSpeed;
		float f32EstNorthPosition;
        float f32RawEastPosition;
        float f32RawNorthPosition;
		float f32RawNorthAcc;
		float f32RawEastAcc;
		GpsPosition origin;
		bool  originUpdate;
		float deltaNorth;
		float deltaEast;
#endif
	};


	struct ahrs_dataT gAhrsData;
	float Declination;					//declination [rad]
	float Pi_2minusInclination;			//pi/2 - inclination [rad]  




private:
    //#define MAG_UPDATE 1 
    static const bool BMAG_UPDATE = true;

    static const int AXIS_X=0;
    static const int AXIS_Y=1;
    static const int AXIS_Z=2; 
    static const int AXIS_NB=3; 

    static const int AXIS_P=0; 
    static const int AXIS_Q=1;
    static const int AXIS_R=2;

    static const int UPDATE_PHI=0;
    static const int UPDATE_THETA=1;
    static const int UPDATE_PSI=2;
    static const int UPDATE_NB=3;

    //  Opis sta³ych typu float na pocz¹tku pliku Ahrs.cpp (podczas inicjalizacji)
    static const float ahrs_Q_gyro;

    //static const float AHRS_R_PHI;
    static const float AHRS_R_PHI;
    static const float AHRS_R_THETA;
    static const float AHRS_R_PSI;
    static const float AHRS_DT;

	static const float KF_CR_ALT;
	static const float KF_CR_CR;
	static const float KF_CR_BIAS;

	static const float KF_GPS_X;
	static const float KF_GPS_VX;
	static const float KF_GPS_BX;

	struct StateKalmanFilter
	{
		float f32Speed;
		float f32Distance;
		float f32BiasAcc;
	};

    struct ahrs_stateT{
        //the quaternion attitude estimate and three gyro bias values 
        float ahrs_q0;
        float ahrs_q1;
        float ahrs_q2;
        float ahrs_q3;
        float ahrs_bias_p;
        float ahrs_bias_q;
        float ahrs_bias_r;
        //unbiased rates
        float ahrs_p;
        float ahrs_q;
        float ahrs_r;
        //eulers
        float ahrs_phi;
        float ahrs_theta;
        float ahrs_psi;
        //time derivative of state
        float ahrs_q0_dot;
        float ahrs_q1_dot;
        float ahrs_q2_dot;
        float ahrs_q3_dot;
        //The Direction Cosine Matrix is used to help rotate measurements
        //to and from the body frame.  We only need five elements from it,
        //so those are computed explicitly rather than the entire matrix
        float ahrs_dcm00;
        float ahrs_dcm01;
        float ahrs_dcm02;
        float ahrs_dcm12;
        float ahrs_dcm22;
        //Covariance matrix and covariance matrix derivative
        float ahrs_P[4][4];
		float ahrs_Phik[4][4];
        //Kalman filter variables
        float ahrs_PHt[4];
        double ahrs_K[4][3];
        float ahrs_E;

		//thanhnt
		double ahrs_Pfin[4][4];
		double chol_lower[4][4];
		double invPyy[3][3];
		double ahrs_Pinit[4][4];



		
        //H represents the Jacobian of the measurements of the attitude
        //with respect to the states of the filter.  We do not allocate the bottom
        //three rows since we know that the attitude measurements have no
        //relationship to gyro bias.
        float ahrs_H[4];

        int updateSelector;

        //FG data interval not constant 
        float dt;
        float dt_1;
        float lastTime;

        //airspeed history, needed for derivative
        float airspeedHistory[3];
        int airspeedDivider;
        float airspeed_1;
        float airspeed_2;

		// Groundspeed history
		float velocityXHistory[3];
		float velocityYHistory[3];
		int   groundspeedDivider;
		float accelX;
		float accelY;

        //derivatives
        float psiDot;
        float thetaDot;
        float phiDot;
        float accXlin;
        float lastAccXlin;

        int ahrsInitialized;

        //eulers plain
        float plain_phi;
        float plain_theta;
        float plain_psi;
		
		//correction flag
		INT8U ahrsUpdateFlag;
		bool ahrsResetUpdatePhiConditionFlag;
		bool ahrsResetUpdateThetaConditionFlag;

		StateKalmanFilter vertSpeedKF;
#if ((PILOT_TYPE == VUA_SC_6G) && (USE_LASER == ENABLED))
		StateKalmanFilter vertSpeedKFLaser;
#endif
#if (PILOT_TYPE == VUA_SC_6G)
		StateKalmanFilter northSpeedKF;
		StateKalmanFilter eastSpeedKF;
#endif
	};

    
    struct ahrs_stateT gAhrsState;

	int lockUpdateCounter;
	int resetUpdatePhiCounter;
	int resetUpdateThetaCounter;

	int magValid; // if data is valid
    int gaugeCyclesPerSecond;


	static const int CAMERA_SEND_PHI_ADDR		= 0x1B*4; // register address where are psi data
														 
	static const int CAMERA_SEND_THETA_ADDR		= 0x1C*4;
	static const int CAMERA_SEND_YAW_ADDR		= 0x1D*4;


    // being set in constructor
	//static const int GAUGE_CYCLES_PER_SECOND = 24;

	void ahrs_predict( struct ahrs_stateT *state, const float* gyro) const;
	void run_kalman( struct ahrs_stateT *state, float Rnoise[3][3], float error[3], int active) const;
	void ahrs_update_phi( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const;
	void ahrs_update_phi_from_mag( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const;
	void ahrs_update_theta_from_mag( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const;
	float ahrs_psi_of_mag( const struct ahrs_stateT *state, const float* mag) const;
	void ahrs_update_theta( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const;
	void ahrs_update_psi( struct ahrs_stateT *state, float psiFromHeading , int active, const float *mag, int magEnable) const;
	void ahrsCompute(struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float psiFromHeading , float airspeed, float *mag, int magEnable, int flightMode, int version, float groundspeed, bool gpsGood);
	void ahrsAccelerometerModify(struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float airspeed) const;
	void ahrsCalculateDots(struct ahrs_stateT* ahrsState) const;
	void ahrsInit(float psiFromHeading , const float* accel, const float* gyro, float airspeed, const float *mag);
	void ahrsInitFG(float psiFromHeading, const float* accel, const float* gyro, float airspeed, float phi, float theta, float agl);
	float ahrs_theta_of_accel( const float* accel) const;
	float ahrs_phi_of_accel( const float* accel) const;
	void eulers_of_quat(float* euler, const float* quat) const;
	void norm_quat(float* quat) const;
	void AHRS_DCM_OF_QUAT(struct ahrs_stateT *state) const;
	void AHRS_PHI_OF_DCM(struct ahrs_stateT *state) const;
	void AHRS_THETA_OF_DCM(struct ahrs_stateT *state) const;
	void AHRS_PSI_OF_DCM(struct ahrs_stateT *state) const;  
	void AHRS_EULER_OF_DCM(struct ahrs_stateT *state) const;
	void AHRS_COMPUTE_H_PSI(struct ahrs_stateT *state) const;
	void AHRS_COMPUTE_H_THETA(struct ahrs_stateT *state) const;
	void AHRS_COMPUTE_H_PHI(struct ahrs_stateT *state) const;
	void AHRS_QUAT_OF_EULER(struct ahrs_stateT *state) const;
	void AHRS_NORM_QUAT(struct ahrs_stateT *state) const;
	void AHRS_WARP(float *x, float b) const;

	//thanhnt
	double det_3(double m[4][4]) const;
	double det_4(double m[4][4]) const;
	bool Cholesky_Decomposition(struct ahrs_stateT *state) const ;
	void invMatrix_3(struct ahrs_stateT *state, double m[3][3]) const;



	void ahrsPlain(struct ahrs_stateT* ahrsState) const;
	void crossProduct(const float *a, const float *b, float *c) const;
	float psi_of_mag(float phi, float theta, const float *mag, float D) const;
	float theta_of_mag(float phi, float theta, const float *mag, float I, float *w) const;
	float phi_of_mag(float phi, float theta, const float *mag, float I, float *w) const;
	int sasbc(const float *A, float epsilon, float *x, float *w) const;
	float incd_of_mag(float phi, float theta, const float *mag, float I) const;
	float checkMagnetometer(struct ahrs_stateT *state, const float *mag, int magEnable) const;

	void estVertSpeed(const struct Gauge::measurementT* measurement, struct ahrs_stateT* ahrs_state);
	void gs84ToNED(struct ahrs_stateT* ahrs_state);
	void accelBody2NED (const struct Gauge::measurementT* measurement, struct ahrs_stateT* ahrs_state, float *accel_NED, bool simulation);
	void estimateSpeed (float measurement, float accel_NED, float dt, struct StateKalmanFilter* position, bool update);
    void estimatePosition (float measurement, float accel_NED, float dt, struct StateKalmanFilter* position, bool update);
	void ahrsQuadAccelModify (struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float velX, float velY) const;
	bool ahrsCheckResetUpdatedCondition (float psdAngle, float updatedAngle, float err_deg, int &resetCounter, int second);
};   

#endif  // AHRS_H
