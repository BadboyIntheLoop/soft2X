/**
*                                                                   
* @class AhrsCF                                                            
*                                                                   
* @brief Class calculates angle position of the aircraft relative to the earth basing on the data from gyroscopes, accelerometers and magnetometer.     
*                                                                   
* NOTE: OPbject of this class is binnary saved in the memory. It could not store any pointer or virtual functions.                        
*                                                                   
* 2012 Witold Kruczek @ Flytronic                                   
*/

#ifndef AHRSCF_H
#define AHRSCF_H


class AhrsCF
{
public:
    typedef float FLT;      ///< float type used in numeric calculation inside AHRS

    //  Status flags
    static const unsigned char FLG_BAD_INTERVAL =  1;      ///< Incorrect interval between samples.
    static const unsigned char FLG_FAST_ALIGN   =  2;      ///< Quick handshaking mode indications (on ground)
    static const unsigned char FLG_NO_UPD_USER  =  4;      ///< Accelerometers and magnetometer not in use (user)
    static const unsigned char FLG_NO_UPD_COND  =  8;      ///< Accelerometers and magnetometer not in use (forced by measurement conditions)
    static const unsigned char FLG_V_NOT_USE    = 16;      ///< Speed not in use (acceleration compensation)
    static const unsigned char FLG_CALIBRATING  = 32;      ///< AHRS during calibration
    static const unsigned char FLG_UNCALIBRATED = 64;      ///< AHRS is not calibrated


    // Structure packing
    #pragma pack(1)

    /**
	* Telemetric data structure (AhrsCF diagnostics)
    * NOTE: Must be packed. For portability types from uC/OS-II system were used.
	*/
    class Tlm
    {
    public:
        INT32U time;         ///< Relative time [ms]
    
        INT16S psiRef;		///< Reference Euler angle - passed from outside to compare results [rad * 1000]
        INT16S thetaRef;	///< Reference Euler angle - passed from outside to compare results [rad * 1000]
        INT16S phiRef;		///< Reference Euler angle - passed from outside to compare results [rad * 1000]

        INT16S thetaG;		///< Euler angle calculated without centripetal and linear acceleration compensation [rad * 1000]
        INT16S phiG;		///< Euler angle calculated without centripetal and linear acceleration compensation [rad * 1000]

        INT16S thetaGA;		///< Euler angle calculated with acceleration compensation [rad * 1000]
        INT16S phiGA;		///< Euler angle calculated with acceleration compensation [rad * 1000]
       
        INT16S psiGAM;		///< Euler angle calculated with acceleration and magnetometer integration compensation [rad * 1000]
        INT16S thetaGAM;	///< Euler angle calculated with acceleration and magnetometer integration compensation [rad * 1000]
        INT16S phiGAM;		///< Euler angle calculated with acceleration and magnetometer integration compensation [rad * 1000]
        
        INT16S psi;			///< Results Euler angle including data from gyroscopes [rad * 1000]
        INT16S theta;		///< Results Euler angle including data from gyroscopes [rad * 1000]
        INT16S phi;			///< Results Euler angle including data from gyroscopes [rad * 1000]
        
        INT16S aux1;		///< Data defined for experimental purposes
        INT16S aux2;		///< Data defined for experimental purposes
        INT16S aux3;		///< Data defined for experimental purposes
        
        INT16S biasP;		///< Bias calculated for gyroscope [rad/s * 1000000]
        INT16S biasQ;		///< Bias calculated for gyroscope [rad/s * 1000000]
        INT16S biasR;		///< Bias calculated for gyroscope [rad/s * 1000000]
   
		INT8U  flags;       ///< Bit flag defining state or errors FLG_*

        Tlm (void):
            time(0), psiRef(0), thetaRef(0), phiRef(0), thetaG(0), phiG(0), thetaGA(0), phiGA(0),
            psiGAM(0), thetaGAM(0), phiGAM(0), psi(0), theta(0), phi(0), aux1(0), aux2(0), aux3(0),
            biasP(0), biasQ(0), biasR(0), flags(0)
        {};
    };

    //  End of structure packing
    #pragma pack()

    class Matrix33;     // Declaration
    class Quaternion;

    /**
	* Class representing 3-elements vector
	*/
    class Vector3
    {
    public:
        Vector3 (FLT pe1, FLT pe2, FLT pe3):
          e1(pe1), e2(pe2), e3(pe3)
        {};

        static Vector3 cross (const Vector3 &a, const Vector3 &b);    ///< Vector product a x b
        static Vector3 sub (const Vector3 &a, const Vector3 &b);      ///< Vector differrence a - b
        static Vector3 add (const Vector3 &a, const Vector3 &b);      ///< Vector sum a + b
        void add (const Vector3 &a);                                  ///< Current vector sum + a
        void normalize (void);                                        ///< Normalize vector
        void multC (FLT c);                                           ///< Multiply vector by constant c
        void rotM (const Matrix33 &m);                                ///< Rotation vector by matrix m
        void rotQ (const Quaternion &q);                              ///< Rotation vector by quaternion q

        FLT e1, e2, e3;
    };


    /**
	* Class representing 3x3 matrix
	*/
    class Matrix33
    {
    public:
        Matrix33 (FLT pe11, FLT pe12, FLT pe13,
                  FLT pe21, FLT pe22, FLT pe23,
                  FLT pe31, FLT pe32, FLT pe33):
          e11(pe11), e12(pe12), e13(pe13),
          e21(pe21), e22(pe22), e23(pe23),
          e31(pe31), e32(pe32), e33(pe33)
        {};

        static Matrix33 multExt (const Vector3 &aC, const Vector3 &bR);  ///< Multiplies column vector a by row vector b
        static Matrix33 dcmZYX (FLT phi, FLT theta, FLT psi);            ///< Creates rotation matrix for Euler angles.
        void add (const Matrix33 &m);                                    ///< Add second matrix elements
        void getAnglesZYX (FLT &phi, FLT &theta, FLT &psi) const;        ///< Calculates Euler angles (sequence  ZYX)
        void transpose (void);                                           ///< Transpose matrix

        FLT e11, e12, e13,
            e21, e22, e23,
            e31, e32, e33;
    };



	/**
	* Quaternion class 
	* q0: scalar part, q1-q3: vector part
	*/
    class Quaternion
    {
    public:
        //  Constructors
        Quaternion (FLT pq0, FLT pq1, FLT pq2, FLT pq3):
          q0(pq0), q1(pq1), q2(pq2), q3(pq3)
        {};
        Quaternion (FLT pq0, const Vector3 &pv):
          q0(pq0), q1(pv.e1), q2(pv.e2), q3(pv.e3)
        {};

        static Quaternion mult (const Quaternion &a, const Quaternion &b);  ///< Quaternion multiplication a and b
        static Quaternion fromDcm (const Matrix33 &c);                      ///< Conversion from rotation matrix
        void multC (FLT c);                                                 ///< Multiply by constant c
        void add (const Quaternion &a);                                     ///< Current quaternion and a sum
        void normalize (void);                                              ///< Quaternion normalize
        void conj (void);                                                   ///< Transformation to the conjugated quaternion. 
        void getAnglesZYX (FLT &phi, FLT &theta, FLT &psi) const;           ///< Calculate Euler angles

        FLT q0, q1, q2, q3;
    };


    /**
	* Class representing configuration data
	*/
    class ConfigData
    {
    public:
        // Copy constructor is compiler generated
        ConfigData(void)
        {
            setDefault();
        }

        float   minTSample;         ///< Minimum sample time in relation to the default time 
        float   maxTSample;         ///< Maximum sample time in relation to the default time
        float   linAccCorr;         ///< Compensation factor of linear acceleration  [no dimension]
        float   cenAccCorr;         ///< Compensation factor of centripetal acceleration  [no dimension] 
        float   minTasMs;           ///< Minimal TAS speed that are allowed to use [m/s]
        float   kp;                 ///< Gain of the P filter
        float   kpFast;             ///< Gain of the P filter in FAST_ALIGN mode
        float   ki;                 ///< Gain of the I filter
        float   kiFast;             ///<  Gain of the I filter in FAST_ALIGN mode
        float   maxAMOffTime;       ///< Maximum time interval when accelerometers and magnetometer is turned off [s]
        bool    useTasFilter;       ///< Use lowpass filter for TAS speed.
        int     calSamples;         ///< Number of samples needed in gyroscopes calibration.
        float   calMaxPQR;          ///< Maximum value of P,Q,R allowed during gyroscopes calibration [rad/s]
        float   tasCorr;            ///< Factor of correction of the input AHRS speed value
        float   vYCoeff;            ///< Y velocity factor in slides depending from ay
        float   vZCoeff;            ///< Z velocity factor depending on TAS in straight flight
        float   updMaxAx;           ///< Acceleration in X axis blocking correction (NOTE: not abs) [m/(s*s)]
        float   biasMaxChng;        ///< Bias changes limitation during flight (relative to value during calibration) [rad/s]
        float   pCorr;              ///< Angular speed P correction (NOTE: Gyroscopes must be calibrated in Gauge !) [no dimension] 
        float   qCorr;              ///< Angular speed Q correction (NOTE: Gyroscopes must be calibrated in Gauge !) [no dimension] 
        float   rCorr;              ///< Angular speed R correction (NOTE: Gyroscopes must be calibrated in Gauge !) [no dimension] 

        void setDefault (void);
    };

    
    /** Butterworth's lowpass filter second row 1-section with structure:
    *  http://www.mathworks.com/help/signal/ref/dfilt.df2sos.html
    *  It is used to TAS speed before acceleration calculations.
	*/
    class FilButt2
    {
    public:
        FilButt2 (FLT ps1, FLT pa1, FLT pa2, FLT pa3, FLT pb1, FLT pb2, FLT pb3):
          s1(ps1), a1(pa1), a2(pa2), a3(pa3), b1(pb1), b2(pb2), b3(pb3), z1(0), z2(0)
        {};

        FLT filter (FLT in);                //  Filter function

    private:
        FLT s1, a1, a2, a3, b1, b2, b3;     //  Factor
        FLT z1, z2;                         //  State variables
    };

    
    explicit AhrsCF (int samplesPerSecond);          // Constructor

    bool  putData (int sysTime100, FLT tas, FLT incl, FLT decl,
                   FLT p,  FLT q,  FLT r,
                   FLT ax, FLT ay, FLT az,
                   FLT mx, FLT my, FLT mz,
                   Tlm &tlm);               //  Transmit new set of data needed to calculations

    void  getAngles (FLT &pPhi, FLT &pTheta, FLT &pPsi) const;
    void  setEarthMode (bool bEarth);
    void  setAccMagOff (bool boff);
    bool  calibrate (void);
    bool  isReady (void) const;
    ConfigData& getConfigRef (void);

private:
    ConfigData      _cfg;            ///< Configuration data
    Quaternion      _quat;           ///< State quaternion current orientation
    Vector3         _inte;           ///< Integral error
    Vector3         _calStoredInte;  ///< Integral error set during calibration (saved gryroscopes Biases)
    FLT             _dfTSample;      ///< Default time between samples [s]
    int             _oldSysTime100;  ///< System time of previous sample [*100us]
    FLT             _oldTas;         ///< TAS speed from previous sample [m/s]
    FLT             _offTimeLeft;    ///< Time left to returned on accelerometers and magnetometer [s]
    FLT             _phi;            ///< Euler angles calculated from _quat (sequence ZYX)
    FLT             _theta;
    FLT             _psi;
    bool            _initQuat;       ///< If initiate state quaternion with current angles.
    bool            _earthMode;      ///< On ground mode
    unsigned char   _flags;          ///< States flag
    FilButt2        _flt;            ///< Lowpass filter for TAS speed.
    int             _calSamples;     ///< Number of samples to beginning of calibration.
    FLT             _calP;           ///< P,Q,R values sume during calibration
    FLT             _calQ;
    FLT             _calR;

    void setStatusFlags (const unsigned char flagBit, bool bEnable);
    void limitRange (FLT& val, FLT base, FLT margin) const;

};

#endif  // AHRSCF_H
