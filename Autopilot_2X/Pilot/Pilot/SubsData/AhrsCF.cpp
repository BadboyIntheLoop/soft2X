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

#include <PilotIncludes.h>

/**
* Constructor 
* /param samplesPerSecond - time between samples [s]
*/
AhrsCF::AhrsCF (int samplesPerSecond):
    _quat(1,0,0,0), _inte(0,0,0), _calStoredInte(0,0,0),
    _dfTSample(static_cast<FLT>(1)/samplesPerSecond), _oldSysTime100(0), _oldTas(0.0f),
    _offTimeLeft(0), _phi(0), _theta(0), _psi(0), _initQuat(true), _earthMode(true),
    _flags(FLG_FAST_ALIGN | FLG_UNCALIBRATED),
    // Filter factor for f cutoff fc=1Hz at fs=24Hz
    _flt(0.014401440346511196f, 1, -1.6329931618554523f, 0.69059892324149719f, 1, 2, 1),
    _calSamples(0), _calP(0), _calQ(0), _calR(0)
{
}


/**
* Function deliver new set of data to used in calculations.
* /param sysTime - system time [*100us]
* /param tas - true airspeed [m/s]
* /param incl - magnetic inclination [rad]
* /param decl - magnetic declination [rad]
* /param p,q,r - angular velocity in x,y,z axis [rad/s]
* /param ax,ay,az - data from accelerometers x,y,z [m/(s*s)]
* /param mx,my,mz - dane z magnetometrów x,y,z [dowolne jednostki]
* return true when passed values caused state update.
*
* Axes of aircraft definition: x: along the fuselage facing forward
*                              y: along the wings facing in right sied
*                              z: perpendicularn to x,y - right-handed system
* Gravity field of the earth is directed along the z axis (positive value)
*      ax,ay,az: positives values when no external gravitational field strength resulting in acceleration of the aircraft is applied in accordance with the return of the axis
*      The general dependence of the aircraft coordinate system:
*          measured acceleration = acceleration resulting from speed changes - gravity field
*      In test in Earth's gravity field should be assumed that signs are correct when:
*          az < 0 when aircraft stands horizontal
*          ax > 0 when aircraft has raised up front
*          ay < 0 when aircraft is tilted to the right
*      This convention is compatible with FlightGear, but NOT with autopilot's accelerometers.
*      AX and ay accelerometeres must be multiplied by -1.
*/
bool  AhrsCF::putData (int sysTime100, FLT tas, FLT incl, FLT decl,
                       FLT p,  FLT q,  FLT r,
                       FLT ax, FLT ay, FLT az,
                       FLT mx, FLT my, FLT mz,
                       AhrsCF::Tlm &tlm)
{
   // update new samples system time.
    FLT tSample = static_cast<FLT>(sysTime100 - _oldSysTime100) * 0.0001f;      // konwersja do sekund
    _oldSysTime100 = sysTime100;

    //  Time control.  Sample is ignored when time difference between samples is to short or too long.
    if (tSample < _cfg.minTSample * _dfTSample || tSample > _cfg.maxTSample * _dfTSample)
    {
        setStatusFlags (FLG_BAD_INTERVAL, true);
        return false;
    }
    setStatusFlags (FLG_BAD_INTERVAL, false);

    //  Linear and angle speed correction.
    tas *= _cfg.tasCorr;
    p *= _cfg.pCorr;
    q *= _cfg.qCorr;
    r *= _cfg.rCorr;

    //  Speed filtering.
    FLT nfTas = tas;                // save not filtred speed for diagnostics.
    FLT fTas = _flt.filter (tas);   // low pass filter
    if (_cfg.useTasFilter)
        tas = fTas;

    FLT dTas = tas - _oldTas;       // time samples difference to calculate accelerations.
    _oldTas = tas;


    //  Gyroscopes calibration.
    //  Calibration is made cycle until flag FLG_CALIBRATING will reset.
    if ((_flags & FLG_CALIBRATING) != 0)
    {
        // Check conditions that allows for precise calibration.
        if (abs(p) < _cfg.calMaxPQR && abs(q) < _cfg.calMaxPQR && abs(r) < _cfg.calMaxPQR)
        {
            _calP += p; _calQ += q; _calR += r;
            _calSamples++;
            if (_calSamples >= _cfg.calSamples)
            {
                setStatusFlags (FLG_UNCALIBRATED, false);
                _inte.e1 = -_calP/_calSamples;
                _inte.e2 = -_calQ/_calSamples;
                _inte.e3 = -_calR/_calSamples;

                // save calibration results.
                _calStoredInte = _inte;

                // Preparations for next calibration cycle.
                _calSamples = 0;
                _calP = _calQ = _calR = 0;
            }
        }
        else
        {
            // There was disruption during calibration - start from beginning.
            _calSamples = 0;
            _calP = _calQ = _calR = 0;
        }
    }

    //==================================================================================
    // Check conditions allowing to use accelerometers and magnetometer to the quaterion's state correction.
    //      - acceleration in X axis is to hight (aircraft is climbing)
    if(!_earthMode && (ax > _cfg.updMaxAx))
        setStatusFlags (FLG_NO_UPD_COND, true);
    else
        setStatusFlags (FLG_NO_UPD_COND, false);

    //==================================================================================
    //  Calculate angles based on accelerometers.

    //  Calculate Phi Theta angles based on gravity vector without acceleration compensation
    //  For diagnostic purposes
    FLT phiG = atan2 (-ay, -az);            // minus minusy,to be in 1 quarter not in 4
    FLT thetaG = atan2 (ax, sqrt (ay*ay + az*az));

    //  Resultant acceleration vector measured by accelerometers.
    Vector3 fB (ax, ay, az);

    //  Angular speed vector
    Vector3 omegaBRaw (p, q, r);
    //  Angular speed vector but from Bias corrections. _inte consists corections
    Vector3 omegaB = Vector3::add (omegaBRaw, _inte);

    //  speed in Y axis during slide (simplification: assume that the acceleration is proportional to Y)
    FLT vairBY = -ay * _cfg.vYCoeff;
    //  speed in Z axis (simplification:  is proportional to speed and acceleration)
    FLT vairBZ = tas * _cfg.vZCoeff * (-az/9.81f);
    //  Linear vector speed 
    Vector3 vairB (tas, vairBY, vairBZ);

    // centripetal acceleration vector
    Vector3 aB = Vector3::cross (omegaB, vairB);
    aB.multC (_cfg.cenAccCorr);             // scaling factor (most offen equal 1)

    // Linear speed compensation (in x plane axis)
    FLT dALin = dTas / tSample;             // momentary accelration between samples (calculated from TAS speed)
    aB.e1 += dALin * _cfg.linAccCorr;       // scaling factor (most offen equal 1)

    // If speed is to low ignore centripetal and linear acceleration correction.
    if (tas < _cfg.minTasMs || _earthMode)
    {
        aB.e1 = aB.e2 = aB.e3 = 0;
        setStatusFlags (FLG_V_NOT_USE, true);      
    }
    else
        //  Turn on including speed, accelerometers and magnetometer (if they were turned off by the user)
        //  set long agreement times
        setStatusFlags (FLG_V_NOT_USE | FLG_NO_UPD_USER, false);

    // gravity vector in aircarft system. gB = aB-fB
    Vector3 gB = Vector3::sub (aB, fB);

    //  Calculate Phi and Theta angles basing on gravity vector in aircraft system compensated with centripetal and linear acceleration
    //  only for diagnostics
    FLT phiGA = atan2 (gB.e2, gB.e3);
    FLT thetaGA = atan2 (-gB.e1, sqrt (gB.e2*gB.e2 + gB.e3*gB.e3));  // - due to Theta has correct sign

    //==================================================================================
    //  Magnetometer integration - TRIAD algorithm
    //  The algorithm finds an approximate rotation matrix between the two coordinate systems (earth and aircraft system)
    //  when having in each two known vectors (gravity vector and magnetic field vector) 
    //  Algorithm sets only Psi angle (not interfers in Phi and Theta angles calculated earlier)

    //  Aircraft's system 

    Vector3 iB = gB;                        // gravity vector in the aircraft system
    iB.normalize();

    Vector3 mB (mx, my, mz);                // magnetic field vector in the aircraft system

    Vector3 jB = Vector3::cross (iB, mB);   // perpendicular to the surface iB, mB vector in the aircraft system
    jB.normalize();

    Vector3 kB = Vector3::cross (iB, jB);   // third vector setting coordinate system of the aircraft
    kB.normalize();

    //  Earth's system

    Vector3 iR (0.0f, 0.0f, 1.0f);          // gravity vector in the Earth system

    FLT t1 = cos (incl);
    Vector3 mR (t1 * cos(decl), t1 * sin(decl), sin(incl));

    Vector3 jR = Vector3::cross (iR, mR);   // magnetic field vector in the Earth system
    jR.normalize();

    Vector3 kR = Vector3::cross (iR, jR);   // third vector setting coordinate system of the Earth

    // Calculate matrix rotation m between Earth and aircraft  systems
    Matrix33 mi = Matrix33::multExt (iB, iR);
    Matrix33 mj = Matrix33::multExt (jB, jR);
    Matrix33 mk = Matrix33::multExt (kB, kR);

    Matrix33 m(mi);     // initiate with mi value
    m.add (mj);         // add mj
    m.add (mk);         // add mk

    //  Calculate Phi, Theta and Psi angles from rotation matrix.
    //  Only for diagnostics.
    FLT phiGAM = 0.0f; FLT thetaGAM = 0.0f; FLT psiGAM = 0.0f;
    m.getAnglesZYX (phiGAM, thetaGAM, psiGAM);
    
    //==================================================================================
    //  Gyroscopes handling
    //  Algorithm operates directly on the quatenion error without auxiliary vectors. [010] [001].
    //  Gives directly (with out iteration) optimal rotation axis between state quaternion (_quat) and quaternion founded from m matrix (calculated with TRIAD algorith from accelerometers and magnetometer)
 
    //  At first running quaternion value state should be initiate that be equal rotation matrix m.
	if (_initQuat)
    {
        _quat = Quaternion::fromDcm (m);
        // turn on gyroscopes calibration.
        calibrate();
        _initQuat = false;
    }

    // Setting a complementary filter gains depending on the mode.  
    // Til takeoff gain should be greater (**Fast) to increase the share of accelerometers and magnetometer. This cause that calibration will be faster.
    float kpt = _cfg.kp;
    float kit = _cfg.ki;
    if ((_flags & (FLG_FAST_ALIGN | FLG_CALIBRATING)) != 0)
    {
        kpt = _cfg.kpFast;
        //  On ground do not integrate errors.
        kit = 0;
    }

    //  Create quaternion from matrix m
    Quaternion quatm = Quaternion::fromDcm (m);
	// The creation of the quaternion conjugated to the state quaternion (give rotation in the opposite direction)
    Quaternion quatc (_quat);
    quatc.conj ();

    //  Compound of 2 rotations: one side with state quaternion, other with correction quternion.
    //  Result is an error quaternion w aircraft system. Describes rotation that is needed to move from state quaternion to correction quaternion.
    //  Rotation need to be scaled due to be not proceed in one step but according to Kp gain.
    Quaternion qe = Quaternion::mult (quatc, quatm);
    qe.normalize ();

    //  Calculate error vector from quaternion error.
    //  Source: R.Mahony, T.Hamel - "Robust Nonlinear Observers for Attitude Estimation of Mini UAVs" in "Advances in Unmanned Aerial Vehicles" Springer 2007
    Vector3 e (qe.q1, qe.q2, qe.q3);
    e.multC (qe.q0);

    // Work on gyroscope only, when measuremnt conditions are not met.
    if ((_flags & (FLG_NO_UPD_COND)) != 0)
    {
        e.e1 = e.e2 = e.e3 = 0;
    }

    // Work on gyroscope only, turned on by the user during takeof, when speed is too low for linear acceleration correction.
    if ((_flags & FLG_NO_UPD_USER) != 0)
    {
        e.e1 = e.e2 = e.e3 = 0;
        //  Automatic exit from this mode after timeout
        _offTimeLeft -= tSample;
        if (_offTimeLeft < 0)
            setStatusFlags (FLG_NO_UPD_USER, false);
    }

    //  Scale error vector. Needed during testing other algorithms that calculates error vector (keeps exists ki and kp factors)
    e.multC (0.5f);

    // Integration of the error vector: inte = inte + kit*e*tSample
    Vector3 tinte (e);
    tinte.multC (kit * tSample);
    _inte.add (tinte);
    
    // Innovation vector calculations: delta = inte + kpt*e
    Vector3 delta (e);
    delta.multC (kpt);
    delta.add (_inte);

    // Add measured angular speed to innovation vector. Speed is not beeing corrected by Bias, because Bias is contained in _inte
    delta.add (omegaBRaw);

    // Create quaternion from innovation vector ("pure quaternion" from 0 in real part)
    Quaternion pk (0, delta);

    // calculate required change of state quaternion (quaternion's derivative)
    Quaternion qdot = Quaternion::mult (_quat, pk);
    qdot.multC (0.5f);

    // Calculate new orientation
    qdot.multC (tSample);
    _quat.add (qdot);

    // State quaternion normalize
    _quat.normalize();

    //  Calculate Euler angles
    _quat.getAnglesZYX (_phi, _theta, _psi);

    // Checking limitation for maximum change of Bias during flight
    limitRange (_inte.e1, _calStoredInte.e1, _cfg.biasMaxChng);
    limitRange (_inte.e2, _calStoredInte.e2, _cfg.biasMaxChng);
    limitRange (_inte.e3, _calStoredInte.e3, _cfg.biasMaxChng);

    // Fill telemetry structure
    tlm.time  = static_cast<INT32U>(sysTime100 / 10);       // conversion from 100us to 1ms
    tlm.phiG     = static_cast<INT16S>(phiG * 1000.0f);
    tlm.thetaG   = static_cast<INT16S>(thetaG * 1000.0f);
    tlm.phiGA    = static_cast<INT16S>(phiGA * 1000.0f);
    tlm.thetaGA  = static_cast<INT16S>(thetaGA * 1000.0f);
    tlm.phiGAM   = static_cast<INT16S>(phiGAM * 1000.0f);
    tlm.thetaGAM = static_cast<INT16S>(thetaGAM * 1000.0f);
    tlm.psiGAM   = static_cast<INT16S>(psiGAM * 1000.0f);
    tlm.phi   = static_cast<INT16S>(_phi * 1000.0f);
    tlm.theta = static_cast<INT16S>(_theta * 1000.0f);
    tlm.psi   = static_cast<INT16S>(_psi * 1000.0f);
    tlm.biasP = static_cast<INT16S>(_inte.e1 * 1000000.0f);
    tlm.biasQ = static_cast<INT16S>(_inte.e2 * 1000000.0f);
    tlm.biasR = static_cast<INT16S>(_inte.e3 * 1000000.0f);
    tlm.aux1  = static_cast<INT16S>(nfTas * 100.0f);
    tlm.aux2  = static_cast<INT16S>(fTas * 100.0f);
    tlm.aux3  = 0;
    tlm.flags = _flags;

    return false;
}


/**
* Return calculated Euler angles (sequence ZYX)
*/
void AhrsCF::getAngles (FLT &pPhi, FLT &pTheta, FLT &pPsi) const
{
    pPhi = _phi;
    pTheta = _theta;
    pPsi = _psi;
}

/**
* Turn on/off ground mode
*/
void AhrsCF::setEarthMode (bool bEarth)
{
    // Set complementary filter gains 
	setStatusFlags (FLG_FAST_ALIGN, bEarth);

    // Break calibration when ground mode is on
    if (!bEarth)
        setStatusFlags (FLG_CALIBRATING, false);

    _earthMode = bEarth;
}

/**
* Turn off accelerometers and magnetometer.
* Used before takeoffwhen low speed do not allowed for linear acceleration compensation.
* Flag will be automatic turned off after timeout (defined in config param)
*/
void AhrsCF::setAccMagOff (bool boff)
{
    setStatusFlags (FLG_NO_UPD_USER, boff);
    if (boff)
        _offTimeLeft = _cfg.maxAMOffTime;
}


/**  
* Starts gyroscopes process (defines Bias).
* During specified time, measured angle speed is being averaged (under stringent conditions to external interference)
* If interference wil be detected process startes from beginning.
* Return false when calibration process was started in the air.
*/
bool  AhrsCF::calibrate (void)
{
    if (_earthMode)
    {
        setStatusFlags (FLG_CALIBRATING | FLG_UNCALIBRATED, true);
        _calSamples = 0;
        // Calculated Bias is stored till end of calibration.
        _calP = _calQ = _calR = 0;

        return true;
    }

    return false;
}


/**
* Returns true when AHRS is ready to takeoff (is calibrated)
*/
bool  AhrsCF::isReady (void) const
{
    return (_flags & FLG_UNCALIBRATED) == 0;
}


/**
* Returns reference to the structure with configuration data.
* This structure can be stored and read externally.
*/
AhrsCF::ConfigData& AhrsCF::getConfigRef (void)
{
    return _cfg;
}


/**
* Set or delete specified bits from status word.
*/
void AhrsCF::setStatusFlags (const unsigned char flagBits, bool bEnable)
{
    // Time controll Kontrola czasu. Sample is ignored when time between samples is to short or too long.
    if (bEnable)
        _flags |= flagBits;
    else
        _flags &= static_cast<unsigned char>(~(flagBits));
}


/**
* Limits value of "val" to the range "base +- margin"
*/
void AhrsCF::limitRange (FLT& val, FLT base, FLT margin) const
{
    if (val > base + margin)
        val = base + margin;
    if (val < base - margin)
        val = base - margin;
 }


/**
* Set default configuration parameters.
*/
void AhrsCF::ConfigData::setDefault (void)
{
    minTSample      =   0.5f;
    maxTSample      =   3.0f;
    linAccCorr      =   1.0f;
    cenAccCorr      =   1.0f;
    minTasMs        =   8.0f;
    kp              =   0.09f;
    kpFast          =   0.8f;
    ki              =   0.0003f;
    kiFast          =   0.025f;
    maxAMOffTime    =  20.0f;
    useTasFilter    = true;
    calSamples      = 360;
    calMaxPQR       =   0.02f;
    tasCorr         =   0.95f;
    vYCoeff         =   1.0f;
    vZCoeff         =   0.08f;     // for FG: 0.03
    updMaxAx        =   2.0f;
    biasMaxChng     =   0.0025f;
    pCorr           =   1.0f;
    qCorr           =   1.0f;
    rCorr           =   1.0f;
}


/**
* Calculates vector product a x b
* Returns result by value.
*/
AhrsCF::Vector3 AhrsCF::Vector3::cross (const Vector3 &a, const Vector3 &b)
{
    Vector3 result (a.e2*b.e3 - a.e3*b.e2,
                    a.e3*b.e1 - a.e1*b.e3,
                    a.e1*b.e2 - a.e2*b.e1);
    return result;
}


/**
* Calculates vector difference a - b
* Returns result by value.
*/
AhrsCF::Vector3 AhrsCF::Vector3::sub (const Vector3 &a, const Vector3 &b)
{
    Vector3 result (a.e1 - b.e1,
                    a.e2 - b.e2,
                    a.e3 - b.e3);
    return result;
}


/**
* Calculates vectos sum a + b
* Returns result by value.
*/
AhrsCF::Vector3 AhrsCF::Vector3::add (const Vector3 &a, const Vector3 &b)
{
    Vector3 result (a.e1 + b.e1,
                    a.e2 + b.e2,
                    a.e3 + b.e3);
    return result;
}


/**
* Calculates current vector + a 
*/
void AhrsCF::Vector3::add (const Vector3 &a)
{
    e1 += a.e1;
    e2 += a.e2;
    e3 += a.e3;
}


/**
* Normalize vector (changes it's lenght to 1)
*/
void AhrsCF::Vector3::normalize (void)
{
    FLT norm = sqrt(e1*e1 + e2*e2 + e3*e3);     // lenght (norm) vector

    if (norm == static_cast<FLT>(0.0f))
        return;

    e1 /= norm;
    e2 /= norm;
    e3 /= norm;
}


/**
* Calculates the product of the components of a vector by a constant c
*/
void AhrsCF::Vector3::multC (FLT c)
{
    e1 *= c;
    e2 *= c;
    e3 *= c;
}


/**
* Rotates vector by matrix m
*/
void AhrsCF::Vector3::rotM (const Matrix33 &m)
{
    FLT te1 = m.e11*e1 + m.e12*e2 + m.e13*e3;
    FLT te2 = m.e21*e1 + m.e22*e2 + m.e23*e3;
    FLT te3 = m.e31*e1 + m.e32*e2 + m.e33*e3;

    e1=te1; e2=te2; e3=te3;
}


/**
* Rotates vector by the quaternion r
* Quaternion must be normalized (length 1)
*/
void AhrsCF::Vector3::rotQ (const Quaternion &r)
{
    static FLT d = static_cast<FLT>(2.0f);
    
    //  create rotation matrix
    Matrix33 m (1 - d*r.q2*r.q2 - d*r.q3*r.q3,  d*(r.q1*r.q2 + r.q0*r.q3),      d*(r.q1*r.q3 - r.q0*r.q2),
                d*(r.q1*r.q2 - r.q0*r.q3),      1 - d*r.q1*r.q1 - d*r.q3*r.q3,  d*(r.q2*r.q3 + r.q0*r.q1),
                d*(r.q1*r.q3 + r.q0*r.q2),      d*(r.q2*r.q3 - r.q0*r.q1),      1 - d*r.q1*r.q1 - d*r.q2*r.q2);

    //  rotation
    rotM (m);
}


/**
* Multiplies column vector a by row vector b
* Returns matrix 3x3 by the value
*/
AhrsCF::Matrix33 AhrsCF::Matrix33::multExt (const Vector3 &a, const Vector3 &b)
{
    Matrix33 result (a.e1*b.e1, a.e1*b.e2, a.e1*b.e3,
                     a.e2*b.e1, a.e2*b.e2, a.e2*b.e3,
                     a.e3*b.e1, a.e3*b.e2, a.e3*b.e3);
    return result;
};


/**
* Creates rotation matrix for Euler angles. (sequence ZYX Earth->Body)
* Matrix allow to convert vectro from Earth coordinates to aircraft coordinates.
*/
AhrsCF::Matrix33 AhrsCF::Matrix33::dcmZYX (FLT phi, FLT theta, FLT psi)
{
    FLT cPhi = cos(phi);     FLT sPhi = sin(phi);
    FLT cTheta = cos(theta); FLT sTheta = sin(theta);
    FLT cPsi = cos(psi);     FLT sPsi = sin(psi);

    Matrix33 result (cTheta*cPsi,                   cTheta*sPsi,                  -sTheta,
                     sPhi*sTheta*cPsi - cPhi*sPsi,  sPhi*sTheta*sPsi + cPhi*cPsi,  sPhi*cTheta,
                     cPhi*sTheta*cPsi + sPhi*sPsi,  cPhi*sTheta*sPsi - sPhi*cPsi,  cPhi*cTheta);
    return result;
}


/**
* Add elements of m matrix to current matrix
*/
void AhrsCF::Matrix33::add (const Matrix33 &m)
{
    e11 += m.e11; e12 += m.e12; e13 += m.e13;
    e21 += m.e21; e22 += m.e22; e23 += m.e23;
    e31 += m.e31; e32 += m.e32; e33 += m.e33;
}


/**
* Calculates Euler angles (sequence  ZYX)
* Returned angles are in range -pi/2; +pi/2
*/
void AhrsCF::Matrix33::getAnglesZYX (FLT &Phi, FLT &Theta, FLT &Psi) const
{
    Phi = atan2 (e23, e33);
    Theta = -asin (e13);
    Psi = atan2 (e12, e11);
}


/**
* Performs matrix transposition
*/
void AhrsCF::Matrix33::transpose (void)
{
    FLT te12 = e12;
    FLT te13 = e13;
    FLT te23 = e23;

    e12 = e21;  e13 = e31;  e23 = e32;
    e21 = te12; e31 = te13; e32 = te23;
}


/**
* Static function to multiply quaternions a and b.
* Returns  result by value.
*/
AhrsCF::Quaternion AhrsCF::Quaternion::mult (const Quaternion &a, const Quaternion &b)
{
    Quaternion result (b.q0*a.q0 - b.q1*a.q1 - b.q2*a.q2 - b.q3*a.q3,
                       b.q0*a.q1 + b.q1*a.q0 - b.q2*a.q3 + b.q3*a.q2,
                       b.q0*a.q2 + b.q1*a.q3 + b.q2*a.q0 - b.q3*a.q1,
                       b.q0*a.q3 - b.q1*a.q2 + b.q2*a.q1 + b.q3*a.q0);
    return result;
}


/**
* Static function converting specified rotation matrix to the quaternion.
* For simplification do not convert all kind of matrix.
*/
AhrsCF::Quaternion AhrsCF::Quaternion::fromDcm (const Matrix33 &m)
{
    FLT a = m.e11 + m.e22 + m.e33 + 1.0f;
    if (a > 0.0f)
    {
        a = 0.5f * sqrt (a);
        Quaternion result (a,
                           (m.e23 - m.e32) / (4.0f*a),
                           (m.e31 - m.e13) / (4.0f*a),
                           (m.e12 - m.e21) / (4.0f*a));
        return result;
    }
    else
    {
        Quaternion result (1,0,0,0);
        return result;
    }
}


/**
* Calculates the product of the quaternion components by a constant c.
*/
void AhrsCF::Quaternion::multC (FLT c)
{
    q0 *= c;
    q1 *= c;
    q2 *= c;
    q3 *= c;
}


/**
* Calculates sum of current quaterniona and a.
*/
void AhrsCF::Quaternion::add (const Quaternion &a)
{
    q0 += a.q0;
    q1 += a.q1;
    q2 += a.q2;
    q3 += a.q3;
}


/**
* Normalize quaternion (change its length to 1)
*/
void AhrsCF::Quaternion::normalize (void)
{
    FLT norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);     // length (norm)

    if (norm == static_cast<FLT>(0.0f))
        return;

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}


/**
* Creates conjugated quaternion.
*/
void AhrsCF::Quaternion::conj (void)
{
    q1 *= -1.0f;
    q2 *= -1.0f;
    q3 *= -1.0f;
}


/**
* Calculates Euler angles (sequence ZYX)
* Returned angles are in range -pi/2; +pi/2
*/
void AhrsCF::Quaternion::getAnglesZYX (FLT &Phi, FLT &Theta, FLT &Psi) const
{
    Phi = atan2 (2.0f*(q2*q3 + q0*q1),      q0*q0 - q1*q1 - q2*q2 + q3*q3);
    Theta = -asin (2.0f*(q1*q3 - q0*q2));
    Psi = atan2 (2.0f*(q1*q2 + q0*q3),      q0*q0 + q1*q1 - q2*q2 - q3*q3);
}


/**
* Filter input signal.
* /param in - input signal sample.
* Returns calculated output sample.
*/
AhrsCF::FLT AhrsCF::FilButt2::filter (FLT in)
{
    FLT t1 = in*s1 - z1*a2;
    FLT t2 = (t1 - z2*a3) / a1;
    FLT t3 = t2*b1 + z1*b2;
    FLT t4 = t3    + z2*b3;

    z2 = z1;
    z1 = t2;

    return t4;
}
