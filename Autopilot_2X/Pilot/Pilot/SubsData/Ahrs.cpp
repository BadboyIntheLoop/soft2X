/**
*                                                                  
* @class AHRS                                                              
*                                                                   
* 2008 Grzegorz Tyma @ Flytronic   								 
*/

#include <PilotIncludes.h>


#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Static variables declarations

/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */

const float Ahrs::ahrs_Q_gyro=1e-04f;

/*
 * R is our measurement noise estimate.  Like Q, it is supposed to be
 * an NxN matrix with elements on the diagonals.  However, since we can
 * not directly measure the gyro bias, we have no estimate for it.
 * We only have an expected noise in the pitch and roll accelerometers
 * and in the compass.
 */

const float Ahrs::AHRS_R_PHI   = 1.7f * 1.7f;
const float Ahrs::AHRS_R_THETA = 1.3f * 1.3f;
const float Ahrs::AHRS_R_PSI   = 2.5f * 2.5f;
const float Ahrs::AHRS_DT      = 1.0f / 100.0f;

const float Ahrs::KF_CR_ALT  = 0.0722f;     //0.2267f;
const float Ahrs::KF_CR_CR   = 0.05627f;    //0.4097f;
const float Ahrs::KF_CR_BIAS =-0.0202f;     //-0.1728f;

const float Ahrs::KF_GPS_X   = -0.2438564f; 
const float Ahrs::KF_GPS_VX  =  0.323019f; 
const float Ahrs::KF_GPS_BX  = -0.28472529f; 

/**
* Constructor
*/
Ahrs::Ahrs(int cyclesPerSecond):
gaugeCyclesPerSecond(cyclesPerSecond)
{
	gAhrsState.ahrsInitialized = 0;

	lockUpdateCounter = 0;

	resetUpdatePhiCounter = 0;
	resetUpdateThetaCounter = 0;

	magValid = 0;

	Declination = 0.0698f;					//declination [rad]
	Pi_2minusInclination = 0.4131f;			//pi/2 - inclination [rad]  
}


/*
 * The rotation matrix to rotate from NED frame to body frame without
 * rotating in the yaw axis is:
 *
 * [ 1      0         0    ] [ cos(Theta)  0  -sin(Theta) ]
 * [ 0  cos(Phi)  sin(Phi) ] [      0      1       0      ]
 * [ 0 -sin(Phi)  cos(Phi) ] [ sin(Theta)  0   cos(Theta) ]
 *
 * This expands to:
 *
 * [  cos(Theta)              0      -sin(Theta)         ]
 * [  sin(Phi)*sin(Theta)  cos(Phi)   sin(Phi)*cos(Theta)]
 * [  cos(Phi)*sin(Theta)  -sin(Phi)  cos(Phi)*cos(Theta)]
 *
 * However, to untilt the compass reading, we need to use the 
 * transpose of this matrix.
 *
 * [  cos(Theta)  sin(Phi)*sin(Theta)  cos(Phi)*sin(Theta) ]
 * [      0       cos(Phi)            -sin(Phi)            ]
 * [ -sin(Theta)  sin(Phi)*cos(Theta)  cos(Phi)*cos(Theta) ]
 *
 * Additionally,
 * since we already have the DCM computed for our current attitude,
 * we can short cut all of the trig. substituting
 * in from the definition of euler2quat and quat2euler, we have:
 *
 * [ cos(Theta)   -dcm12*dcm02   -dcm22*dcm02 ]
 * [    0          dcm22         -dcm12       ]
 * [ dcm02         dcm12          dcm22       ]
 *
 */

/**
 * Compute the five elements of the DCM (Direction Cosine Matrix) that we use for our
 * rotations and Jacobians.  This is used by several other functions
 * to speedup their computations.
 */
void Ahrs::AHRS_DCM_OF_QUAT(struct ahrs_stateT *state) const
{				
    state->ahrs_dcm00 = 1.0f-2.0f*(state->ahrs_q2*state->ahrs_q2 + state->ahrs_q3*state->ahrs_q3);	
    state->ahrs_dcm01 =      2.0f*(state->ahrs_q1*state->ahrs_q2 + state->ahrs_q0*state->ahrs_q3);	
    state->ahrs_dcm02 =      2.0f*(state->ahrs_q1*state->ahrs_q3 - state->ahrs_q0*state->ahrs_q2);	
    state->ahrs_dcm12 =      2.0f*(state->ahrs_q2*state->ahrs_q3 + state->ahrs_q0*state->ahrs_q1);	
    state->ahrs_dcm22 = 1.0f-2.0f*(state->ahrs_q1*state->ahrs_q1 + state->ahrs_q2*state->ahrs_q2);	
}
/**
 * Compute Euler angles from our DCM.
 */
void Ahrs::AHRS_PHI_OF_DCM(struct ahrs_stateT *state) const
{
    if(state->ahrs_dcm22!=0.0f)
        state->ahrs_phi = atan2f( state->ahrs_dcm12, state->ahrs_dcm22 );
    else {
        if(state->ahrs_dcm12!=0.0f)
            state->ahrs_phi = PI;
        else
            state->ahrs_phi=0.0f;
    }            
}

void Ahrs::AHRS_THETA_OF_DCM(struct ahrs_stateT *state) const
{
    if(fabsf(state->ahrs_dcm02) < 1.0f)
        state->ahrs_theta = -asinf( state->ahrs_dcm02 );
    else
    {
         //WK: unikniêcie ew. dzielenia przez 0
        state->ahrs_theta = (state->ahrs_dcm02 > 0.0f) ? -PI2 : PI2;
    }
}

void Ahrs::AHRS_PSI_OF_DCM(struct ahrs_stateT *state) const
{
    if(state->ahrs_dcm00!=0.0f)
        state->ahrs_psi = atan2f( state->ahrs_dcm01, state->ahrs_dcm00 );
    else {
        if(state->ahrs_dcm01!=0.0f)
            state->ahrs_psi = PI;
        else
            state->ahrs_psi=0.0f;
    }
}

void Ahrs::AHRS_EULER_OF_DCM(struct ahrs_stateT *state) const
{
    AHRS_PHI_OF_DCM(state);
    AHRS_THETA_OF_DCM(state);
    AHRS_PSI_OF_DCM(state);
}

/**
 * initialise the quaternion from the set of eulers
 */
void Ahrs::AHRS_QUAT_OF_EULER(struct ahrs_stateT *state) const
{			                             
    const float phi2     = state->ahrs_phi * 0.5f;	                             
    const float theta2   = state->ahrs_theta * 0.5f;	                             
    const float psi2     = state->ahrs_psi * 0.5f;	                             
						                             
    const float sinphi2   = sinf( phi2 );	                            
    const float cosphi2   = cosf( phi2 );	                            
						                             
    const float sintheta2 = sinf( theta2 );	                             
    const float costheta2 = cosf( theta2 );	                             
						                             
    const float sinpsi2   = sinf( psi2 );	                             
    const float cospsi2   = cosf( psi2 );	                             
    									     
    state->ahrs_q0 =  cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2; 
    state->ahrs_q1 = -cosphi2 * sintheta2 * sinpsi2 + sinphi2 * costheta2 * cospsi2; 
    state->ahrs_q2 =  cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2; 
    state->ahrs_q3 =  cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2; 
  }

/**
 * normalize quaternion
 */
void Ahrs::AHRS_NORM_QUAT(struct ahrs_stateT *state) const
{			
    float  mag = state->ahrs_q0*state->ahrs_q0 + state->ahrs_q1*state->ahrs_q1
               + state->ahrs_q2*state->ahrs_q2 + state->ahrs_q3*state->ahrs_q3;
    if (mag > 0.0f)
    {
        float mag2 = 1.0f / sqrtf( mag );

        state->ahrs_q0 *= mag2;				
        state->ahrs_q1 *= mag2;				
        state->ahrs_q2 *= mag2;				
        state->ahrs_q3 *= mag2;
    }
}

/**
 * Compute the Jacobian of the measurements to the system states.
 */
void Ahrs::AHRS_COMPUTE_H_PHI(struct ahrs_stateT *state) const
{          						
    float phi_err = state->ahrs_dcm22*state->ahrs_dcm22 + state->ahrs_dcm12*state->ahrs_dcm12;     
    if(phi_err!=0.0f)
    {	
        phi_err = 2.0f / phi_err; 
        state->ahrs_H[0] = (state->ahrs_q1 * state->ahrs_dcm22) * phi_err;				        
        state->ahrs_H[1] = (state->ahrs_q0 * state->ahrs_dcm22 + 2.0f * state->ahrs_q1 * state->ahrs_dcm12) * phi_err;	        
        state->ahrs_H[2] = (state->ahrs_q3 * state->ahrs_dcm22 + 2.0f * state->ahrs_q2 * state->ahrs_dcm12) * phi_err;	        
        state->ahrs_H[3] = (state->ahrs_q2 * state->ahrs_dcm22) * phi_err;
    }
}

void Ahrs::AHRS_COMPUTE_H_THETA(struct ahrs_stateT *state) const
{						        
    float theta_err  =  1.0f - state->ahrs_dcm02*state->ahrs_dcm02;

    //WK: Zmiana zmniejszaj¹ca ryzyko nieprawid³owych operacji float
    if (theta_err > 0.0f)
    {
        theta_err = sqrtf(theta_err);
        theta_err = -2.0f / theta_err; 

        state->ahrs_H[0] = -state->ahrs_q2 * theta_err;						
        state->ahrs_H[1] =  state->ahrs_q3 * theta_err;						
        state->ahrs_H[2] = -state->ahrs_q0 * theta_err;						
        state->ahrs_H[3] =  state->ahrs_q1 * theta_err;
    }
}

void Ahrs::AHRS_COMPUTE_H_PSI(struct ahrs_stateT *state) const
{					                
    float psi_err = (state->ahrs_dcm00*state->ahrs_dcm00 + state->ahrs_dcm01*state->ahrs_dcm01);
    if(psi_err!=0.0f)
    {
        psi_err = 2.0f / psi_err;
        state->ahrs_H[0] = (state->ahrs_q3 * state->ahrs_dcm00) * psi_err;
        state->ahrs_H[1] = (state->ahrs_q2 * state->ahrs_dcm00) * psi_err;
        state->ahrs_H[2] = (state->ahrs_q1 * state->ahrs_dcm00 + 2 * state->ahrs_q2 * state->ahrs_dcm01) * psi_err;
        state->ahrs_H[3] = (state->ahrs_q0 * state->ahrs_dcm00 + 2 * state->ahrs_q3 * state->ahrs_dcm01) * psi_err;
    }
}


void Ahrs::AHRS_WARP(float *x, float b) const
{
    float xx = *x;

	while( xx < -b )				
      xx += (2.0f * b);	
    while( xx > b )				
      xx -= (2.0f * b);	

	*x = xx;
  }


/*
 * Call ahrs_state_update every dt seconds with the raw body frame angular
 * rates.  It updates the attitude state estimate via this function:
 *
 *      quat_dot = Wxq(pqr) * quat
 *      bias_dot = 0
 *
 * Since F also contains Wxq, we fill it in here and then reuse the computed
 * values.  This avoids the extra floating point math.
 *
 * Wxq is the quaternion omega matrix:
 *
 *              [ 0, -p, -q, -r ]
 *      1/2 *   [ p,  0,  r, -q ]
 *              [ q, -r,  0,  p ]
 *              [ r,  q, -p,  0 ]
 *
 *
 *
 *
 *                 [ 0  -p  -q  -r   q1  q2  q3]
 *   F =   1/2 *   [ p   0   r  -q  -q0  q3 -q2]
 *                 [ q  -r   0   p  -q3  q0  q1]
 *                 [ r   q  -p   0   q2 -q1 -q0]
 *                 [ 0   0   0   0    0   0   0]
 *                 [ 0   0   0   0    0   0   0]
 *                 [ 0   0   0   0    0   0   0]
 *
 */
void Ahrs::ahrs_predict( struct ahrs_stateT *state, const float* gyro) const
{
  float s0,s1,s2,s3;
  float ahrs_FP[7][7];
  float ahrs_Pdot[7][7];
  //F represents the Jacobian of the derivative of the system with respect
  //its states.  We do not allocate the bottom three rows since we know that
  //the derivatives of bias_dot are all zero.
  float ahrs_F[7][7] = {0};
  float ahrs_dt = state->dt;

  //PERF_BEGIN (PERFORMANCE_COUNTER_BASE, SECTION1);//31089*3.33/3  
  state->ahrs_p = gyro[0] -  state->ahrs_bias_p;
  state->ahrs_q = gyro[1] -  state->ahrs_bias_q;
  state->ahrs_r = gyro[2] -  state->ahrs_bias_r;

  // compute F 
  //  F is only needed later on to update the state covariance P.
  //  However, its [0:3][0:3] region is actually the Wxq(pqr) which is needed to
  //  compute the time derivative of the quaternion, so we compute F now 

  ahrs_F[0][0] = ahrs_F[1][1] = ahrs_F[2][2] = ahrs_F[3][3] = 0.0f;
  ahrs_F[1][0] = ahrs_F[2][3] = gyro[0] * 0.5f;
  ahrs_F[2][0] = ahrs_F[3][1] = gyro[1] * 0.5f;
  ahrs_F[3][0] = ahrs_F[1][2] = gyro[2] * 0.5f;

  ahrs_F[0][1] = ahrs_F[3][2] = -ahrs_F[1][0];
  ahrs_F[0][2] = ahrs_F[1][3] = -ahrs_F[2][0];
  ahrs_F[0][3] = ahrs_F[2][1] = -ahrs_F[3][0];
  // Fill in [4:6][0:3] region 
  ahrs_F[0][4] = ahrs_F[2][6] = state->ahrs_q1 * 0.5f;
  ahrs_F[0][5] = ahrs_F[3][4] = state->ahrs_q2 * 0.5f;
  ahrs_F[0][6] = ahrs_F[1][5] = state->ahrs_q3 * 0.5f;

  ahrs_F[1][4] = ahrs_F[2][5] = ahrs_F[3][6] = state->ahrs_q0 * -0.5f;
  ahrs_F[3][5] = -ahrs_F[0][4];
  ahrs_F[1][6] = -ahrs_F[0][5];
  ahrs_F[2][4] = -ahrs_F[0][6];

//   compute Phik
        float ahrs_Phik[7][7] = {0};
        ahrs_Phik[0][0] = ahrs_Phik[1][1] = ahrs_Phik[2][2] = ahrs_Phik[3][3] = 1.0f;
        ahrs_Phik[1][0] = ahrs_Phik[2][3] = gyro[0] * 0.5f * ahrs_dt;
        ahrs_Phik[2][0] = ahrs_Phik[3][1] = gyro[1] * 0.5f * ahrs_dt;
        ahrs_Phik[3][0] = ahrs_Phik[1][2] = gyro[2] * 0.5f * ahrs_dt;

        ahrs_Phik[0][1] = ahrs_Phik[3][2] = -ahrs_Phik[1][0];
        ahrs_Phik[0][2] = ahrs_Phik[1][3] = -ahrs_Phik[2][0];
        ahrs_Phik[0][3] = ahrs_Phik[2][1] = -ahrs_Phik[3][0];
        // Fill in [4:6][0:3] region 
        ahrs_Phik[0][4] = ahrs_Phik[2][6] = state->ahrs_q1 * 0.5f * ahrs_dt;
        ahrs_Phik[0][5] = ahrs_Phik[3][4] = state->ahrs_q2 * 0.5f * ahrs_dt;
        ahrs_Phik[0][6] = ahrs_Phik[1][5] = state->ahrs_q3 * 0.5f * ahrs_dt;

        ahrs_Phik[1][4] = ahrs_Phik[2][5] = ahrs_Phik[3][6] = state->ahrs_q0 * -0.5f;
        ahrs_Phik[3][5] = -ahrs_Phik[0][4];
        ahrs_Phik[1][6] = -ahrs_Phik[0][5];
        ahrs_Phik[2][4] = -ahrs_Phik[0][6];   
        // Fill in [4:6][4:6] region 
        ahrs_Phik[4][4] = -0.01f * ahrs_dt + 1.0f;
        ahrs_Phik[5][5] = -0.01f * ahrs_dt + 1.0f;
        ahrs_Phik[6][6] = -0.01f * ahrs_dt + 1.0f;

  // update state 
  // quat_dot = Wxq(pqr) * quat 
  state->ahrs_q0     = ahrs_Phik[0][0] * state->ahrs_q0 + ahrs_Phik[0][1] * state->ahrs_q1 + ahrs_Phik[0][2] * state->ahrs_q2 + ahrs_Phik[0][3] * state->ahrs_q3 + ahrs_Phik[0][4] * state->ahrs_bias_p + ahrs_Phik[0][5] * state->ahrs_bias_q + ahrs_Phik[0][6] * state->ahrs_bias_r;
  state->ahrs_q1     = ahrs_Phik[1][0] * state->ahrs_q0 + ahrs_Phik[1][1] * state->ahrs_q1 + ahrs_Phik[1][2] * state->ahrs_q2 + ahrs_Phik[1][3] * state->ahrs_q3 + ahrs_Phik[1][4] * state->ahrs_bias_p + ahrs_Phik[1][5] * state->ahrs_bias_q + ahrs_Phik[1][6] * state->ahrs_bias_r;
  state->ahrs_q2     = ahrs_Phik[2][0] * state->ahrs_q0 + ahrs_Phik[2][1] * state->ahrs_q1 + ahrs_Phik[2][2] * state->ahrs_q2 + ahrs_Phik[2][3] * state->ahrs_q3 + ahrs_Phik[2][4] * state->ahrs_bias_p + ahrs_Phik[2][5] * state->ahrs_bias_q + ahrs_Phik[2][6] * state->ahrs_bias_r;
  state->ahrs_q3     = ahrs_Phik[3][0] * state->ahrs_q0 + ahrs_Phik[3][1] * state->ahrs_q1 + ahrs_Phik[3][2] * state->ahrs_q2 + ahrs_Phik[3][3] * state->ahrs_q3 + ahrs_Phik[3][4] * state->ahrs_bias_p + ahrs_Phik[3][5] * state->ahrs_bias_q + ahrs_Phik[3][6] * state->ahrs_bias_r;
  state->ahrs_bias_p = ahrs_Phik[4][0] * state->ahrs_q0 + ahrs_Phik[4][1] * state->ahrs_q1 + ahrs_Phik[4][2] * state->ahrs_q2 + ahrs_Phik[4][3] * state->ahrs_q3 + ahrs_Phik[4][4] * state->ahrs_bias_p + ahrs_Phik[4][5] * state->ahrs_bias_q + ahrs_Phik[4][6] * state->ahrs_bias_r;
  state->ahrs_bias_q = ahrs_Phik[5][0] * state->ahrs_q0 + ahrs_Phik[5][1] * state->ahrs_q1 + ahrs_Phik[5][2] * state->ahrs_q2 + ahrs_Phik[5][3] * state->ahrs_q3 + ahrs_Phik[5][4] * state->ahrs_bias_p + ahrs_Phik[5][5] * state->ahrs_bias_q + ahrs_Phik[5][6] * state->ahrs_bias_r;
  state->ahrs_bias_r = ahrs_Phik[6][0] * state->ahrs_q0 + ahrs_Phik[6][1] * state->ahrs_q1 + ahrs_Phik[6][2] * state->ahrs_q2 + ahrs_Phik[6][3] * state->ahrs_q3 + ahrs_Phik[6][4] * state->ahrs_bias_p + ahrs_Phik[6][5] * state->ahrs_bias_q + ahrs_Phik[6][6] * state->ahrs_bias_r;
  
  
  // normalize quaternion 
  AHRS_NORM_QUAT(state);
  
  AHRS_DCM_OF_QUAT(state);
  AHRS_EULER_OF_DCM(state);

  // update covariance
  // Pdot = F*P*F' + Q
  // P += Pdot * dt



    // state.ahrs_P = state.ahrs_P + ( ahrs_F * state.ahrs_P * ahrs_F' )*state.dt + Qd_k;
    // Q = diag([0 0 0 0 ahrs_Q_gyro ahrs_Q_gyro ahrs_Q_gyro] );
    // Qd_k = 1/2 * ahrs_phik * Q + 1/2 * Q * ahrs_phik';
        // compute the Qdk matrix
        float ahrs_Qdk[7][7] = {0};
        // Fill in [4:6][4:6] region 
        ahrs_Qdk[4][4] = ahrs_Phik[4][4]*ahrs_Q_gyro;
        ahrs_Qdk[5][5] = ahrs_Phik[5][5]*ahrs_Q_gyro;
        ahrs_Qdk[6][6] = ahrs_Phik[6][6]*ahrs_Q_gyro;



  for (int j=0; j<7; j++) {
      s0=s1=s2=s3=0.0f;
      for (int k=0; k<7; k++) {
          float tmp = state->ahrs_P[k][j];
          s0 += ahrs_F[0][k] * tmp;
          s1 += ahrs_F[1][k] * tmp;
          s2 += ahrs_F[2][k] * tmp;
          s3 += ahrs_F[3][k] * tmp;
      }
      ahrs_FP[0][j] = s0;
      ahrs_FP[1][j] = s1;
      ahrs_FP[2][j] = s2;
      ahrs_FP[3][j] = s3;
  }

  for (int j=0; j<4; j++) {
      s0=s1=s2=s3=0.0f;
      for (int k=0; k<7; k++) {
          float tmp = ahrs_F[j][k];
          s0 += ahrs_FP[0][k] * tmp;
          s1 += ahrs_FP[1][k] * tmp;
          s2 += ahrs_FP[2][k] * tmp;        
          s3 += ahrs_FP[3][k] * tmp;
      }
      ahrs_Pdot[0][j] = s0;
      ahrs_Pdot[1][j] = s1;
      ahrs_Pdot[2][j] = s2;
      ahrs_Pdot[3][j] = s3;

  }

  for (int j=0; j<4; j++){
      state->ahrs_P[0][j] += (ahrs_Pdot[0][j] * ahrs_dt);
      state->ahrs_P[1][j] += (ahrs_Pdot[1][j] * ahrs_dt);
      state->ahrs_P[2][j] += (ahrs_Pdot[2][j] * ahrs_dt);
      state->ahrs_P[3][j] += (ahrs_Pdot[3][j] * ahrs_dt);
  }

  state->ahrs_P[4][4] += (ahrs_Qdk[4][4] * ahrs_dt);
  state->ahrs_P[5][5] += (ahrs_Qdk[5][5] * ahrs_dt);
  state->ahrs_P[6][6] += (ahrs_Qdk[6][6] * ahrs_dt);
}


/*
 * Do the Kalman filter on the acceleration and compass readings.
 * This is normally a very simple:
 *
 *      E = H * P * H' + R
 *      K = P * H' * inv(E)
 *      P = P - K * H * P
 *      X = X + K * error
 *
 * We notice that P * H' is used twice, so we can cache the
 * results of it.
 *
 * H represents the Jacobian of measurements to states, which we know
 * to only have the top four rows filled in since the attitude
 * measurement does not relate to the gyro bias.  This allows us to
 * ignore parts of PHt
 *
 * We also only process one axis at a time to avoid having to perform
 * the 3x3 matrix inversion.
 */

void Ahrs::run_kalman( struct ahrs_stateT *state, const float R_axis, const float error, int active) const
{
  // PHt = P * H' 
  float h0 = state->ahrs_H[0];
  float h1 = state->ahrs_H[1];
  float h2 = state->ahrs_H[2];
  float h3 = state->ahrs_H[3];
  
  state->ahrs_PHt[0] = state->ahrs_P[0][0] * h0 + state->ahrs_P[0][1] * h1 + state->ahrs_P[0][2] * h2 + state->ahrs_P[0][3] * h3;
  state->ahrs_PHt[1] = state->ahrs_P[1][0] * h0 + state->ahrs_P[1][1] * h1 + state->ahrs_P[1][2] * h2 + state->ahrs_P[1][3] * h3;
  state->ahrs_PHt[2] = state->ahrs_P[2][0] * h0 + state->ahrs_P[2][1] * h1 + state->ahrs_P[2][2] * h2 + state->ahrs_P[2][3] * h3;
  state->ahrs_PHt[3] = state->ahrs_P[3][0] * h0 + state->ahrs_P[3][1] * h1 + state->ahrs_P[3][2] * h2 + state->ahrs_P[3][3] * h3;
  state->ahrs_PHt[4] = state->ahrs_P[4][0] * h0 + state->ahrs_P[4][1] * h1 + state->ahrs_P[4][2] * h2 + state->ahrs_P[4][3] * h3;
  state->ahrs_PHt[5] = state->ahrs_P[5][0] * h0 + state->ahrs_P[5][1] * h1 + state->ahrs_P[5][2] * h2 + state->ahrs_P[5][3] * h3;
  state->ahrs_PHt[6] = state->ahrs_P[6][0] * h0 + state->ahrs_P[6][1] * h1 + state->ahrs_P[6][2] * h2 + state->ahrs_P[6][3] * h3;

  //  E = H * PHt + R 
  state->ahrs_E = R_axis;
  state->ahrs_E += h0 * state->ahrs_PHt[0];
  state->ahrs_E += h1 * state->ahrs_PHt[1];
  state->ahrs_E += h2 * state->ahrs_PHt[2];
  state->ahrs_E += h3 * state->ahrs_PHt[3];

  // Compute the inverse of E 
  if(state->ahrs_E == 0.0f)
  {
	  state->ahrs_E = 10000.0f;
  }else{
	  state->ahrs_E = 1.0f / state->ahrs_E;
  }
  // Compute K = P * H' * inv(E) 
  state->ahrs_K[0] = state->ahrs_PHt[0] * state->ahrs_E;
  state->ahrs_K[1] = state->ahrs_PHt[1] * state->ahrs_E;
  state->ahrs_K[2] = state->ahrs_PHt[2] * state->ahrs_E;
  state->ahrs_K[3] = state->ahrs_PHt[3] * state->ahrs_E;
  state->ahrs_K[4] = state->ahrs_PHt[4] * state->ahrs_E;
  state->ahrs_K[5] = state->ahrs_PHt[5] * state->ahrs_E;
  state->ahrs_K[6] = state->ahrs_PHt[6] * state->ahrs_E;

  // Update our covariance matrix: P = P - K * H * P 

  // Compute HP = H * P, reusing the PHt array. 
  state->ahrs_PHt[0] = h0 * state->ahrs_P[0][0] + h1 * state->ahrs_P[1][0] + h2 * state->ahrs_P[2][0] + h3 * state->ahrs_P[3][0];
  state->ahrs_PHt[1] = h0 * state->ahrs_P[0][1] + h1 * state->ahrs_P[1][1] + h2 * state->ahrs_P[2][1] + h3 * state->ahrs_P[3][1];
  state->ahrs_PHt[2] = h0 * state->ahrs_P[0][2] + h1 * state->ahrs_P[1][2] + h2 * state->ahrs_P[2][2] + h3 * state->ahrs_P[3][2];
  state->ahrs_PHt[3] = h0 * state->ahrs_P[0][3] + h1 * state->ahrs_P[1][3] + h2 * state->ahrs_P[2][3] + h3 * state->ahrs_P[3][3];
  state->ahrs_PHt[4] = h0 * state->ahrs_P[0][4] + h1 * state->ahrs_P[1][4] + h2 * state->ahrs_P[2][4] + h3 * state->ahrs_P[3][4];
  state->ahrs_PHt[5] = h0 * state->ahrs_P[0][5] + h1 * state->ahrs_P[1][5] + h2 * state->ahrs_P[2][5] + h3 * state->ahrs_P[3][5];
  state->ahrs_PHt[6] = h0 * state->ahrs_P[0][6] + h1 * state->ahrs_P[1][6] + h2 * state->ahrs_P[2][6] + h3 * state->ahrs_P[3][6];

  // Compute P -= K * HP (aliased to PHt) 
    for(int j=0 ; j<7 ; j++ )
    {
      float tmp = state->ahrs_PHt[j];
      state->ahrs_P[0][j] -= state->ahrs_K[0] * tmp;
      state->ahrs_P[1][j] -= state->ahrs_K[1] * tmp;
      state->ahrs_P[2][j] -= state->ahrs_K[2] * tmp;
      state->ahrs_P[3][j] -= state->ahrs_K[3] * tmp;
    }
  // Update our state: X += K * error 
  state->ahrs_q0     += state->ahrs_K[0] * error;
  state->ahrs_q1     += state->ahrs_K[1] * error;
  state->ahrs_q2     += state->ahrs_K[2] * error;
  state->ahrs_q3     += state->ahrs_K[3] * error;
  state->ahrs_bias_p += state->ahrs_K[4] * error;
  state->ahrs_bias_q += state->ahrs_K[5] * error;
  state->ahrs_bias_r += state->ahrs_K[6] * error;

  /// normalize quaternion 
  AHRS_NORM_QUAT(state);
}

/**
* Calculate PHI from accelerometers
*/
float Ahrs::ahrs_phi_of_accel( const float* accel ) const
{
  float accZ = accel[AXIS_Z];
  float accY = accel[AXIS_Y];		
  if(accZ!=0.0f)
  {
	  return atan2f(accY, -accZ);
  }else{
	  if(accel[AXIS_Y]!=0.0f){
		  return PI*accY/accY;
	  }else{
		  return 0.0f;
	  }
  }
}

/**
* Calculate THETA from accelerometers
*/
float Ahrs::ahrs_theta_of_accel( const float* accel) const
{
    float x = accel[AXIS_X] / G;
    if(x>1.0f) x = 1.0f;
    if(x<-1.0f) x = -1.0f;

	return -asinf(x);	
}

/** 
* Calculate PSI from magnetometers
*/
float Ahrs::ahrs_psi_of_mag( const struct ahrs_stateT *state, const float* mag) const
{
	const float ctheta  = cosf( state->ahrs_theta );

	const float stheta  = sinf( state->ahrs_theta );
	const float cphi  = cosf( state->ahrs_phi );
	const float sphi  = sinf( state->ahrs_phi );
	float psi = 0.0f;

	const float x =  
		ctheta*      mag[0]+
		sphi*stheta* mag[1]+
		cphi*stheta* mag[2];
	const float y =		
		0.0f*     mag[0]+
		cphi*  mag[1]+
		-sphi* mag[2];

	if((x==0.0f)&&(y<0.0f)){
		psi = PI2;
	}else if((x==0.0f)&&(y>0.0f)){
		psi = 1.5f*PI;
	}else if(x<0.0f){
		psi = PI - atanf( y/x );
	}else if((x>0.0f)&&(y<0.0f)){
		psi = -atan( y/ x );
	}else if((x>0.0f)&&(y>0.0f)){
		psi = 2.0f*PI-atanf( y/ x );
	};

   //add here declination correction 
   psi += 3.6f/180.0f*PI;

   //+PI..-PI range
   if(psi>PI)
   {
        psi = psi - 2.0f*PI;
   } 
  
  return psi;
}


/**
* PHI update
*/
void Ahrs::ahrs_update_phi( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const
{
  float err_phi=0.0f;

  AHRS_COMPUTE_H_PHI(state);
  float accel_phi = ahrs_phi_of_accel(accel);
  if(active != 0)
	err_phi = accel_phi - state->ahrs_phi;
  AHRS_WARP( &err_phi, PI);
  run_kalman( state,AHRS_R_PHI, err_phi, active );
  AHRS_DCM_OF_QUAT(state);
  AHRS_EULER_OF_DCM(state);
}


/**
* THETA update from magnetometer
*/
void Ahrs::ahrs_update_theta_from_mag( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const
{
  float err_theta = 0.0f;
  float I = Pi_2minusInclination;;
  float w = 0.0f;

  if (BMAG_UPDATE)
  {
      // phi update from magnetometer
      if((active != 0) && (magValid != 0) && (magEnable==1))
      {
          float incd = incd_of_mag(state->ahrs_phi, state->ahrs_theta, mag, I);
          AHRS_COMPUTE_H_THETA(state);
          float theta_mag = theta_of_mag(state->ahrs_phi, state->ahrs_theta, mag, I, &w);

          if(w==0.0f)
          {
              err_theta = theta_mag - state->ahrs_theta;
              AHRS_WARP( &err_theta, PI2);
              if(abs(incd)>0.04f)
              {
                  run_kalman( state, 2.5f*2.5f , err_theta, active );
              }else{
                  run_kalman( state, 5.5f*5.5f , err_theta, active );
              }

              AHRS_DCM_OF_QUAT(state);
              AHRS_EULER_OF_DCM(state);
          }
      }
  }
}



/**
* THETA update
*/
void Ahrs::ahrs_update_theta( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const
{
  float err_theta=0.0f ;
  AHRS_COMPUTE_H_THETA(state);
  float accel_theta = ahrs_theta_of_accel(accel);
  if(active != 0)
	err_theta = accel_theta - state->ahrs_theta;
  AHRS_WARP( &err_theta, PI2);
  run_kalman( state, AHRS_R_THETA, err_theta, active );
  AHRS_DCM_OF_QUAT(state);
  AHRS_EULER_OF_DCM(state);
}


/**
* PHI update from magnetometer
*/
void Ahrs::ahrs_update_phi_from_mag( struct ahrs_stateT *state, const float* accel, int active, const float *mag, int magEnable) const
{
  float err_phi = 0.0f;
  float I = Pi_2minusInclination;
  float w = 0.0f;

  if (BMAG_UPDATE)
  {
      // phi update from magnetometer
      if((active != 0) && (magValid != 0) && ((magEnable == 1)||(magEnable == 2)) )
      {
          float incd = incd_of_mag(state->ahrs_phi, state->ahrs_theta, mag, I);
          AHRS_COMPUTE_H_PHI(state);
          float phi_mag = phi_of_mag(state->ahrs_phi, state->ahrs_theta, mag, I, &w);

          if(w==0.0f)
          {
              err_phi = phi_mag - state->ahrs_phi;
              AHRS_WARP( &err_phi, PI);
              if(abs(incd)>0.04f)
              {
                  run_kalman( state, 2.5f*2.5f, err_phi, active );
              }else{
                  run_kalman( state, 5.5f*5.5f, err_phi, active );
              }
              AHRS_DCM_OF_QUAT(state);
              AHRS_EULER_OF_DCM(state);
          }
      }
  }
}

/**
* PSI update
*/
void Ahrs::ahrs_update_psi( struct ahrs_stateT *state, float psiFromHeading , int active, const float *mag, int magEnable) const
{
  float mag_psi = 0.0f;
  float err_psi = 0.0f ;
  float incd = 0.0f;

  float I = Pi_2minusInclination;
  float D = Declination;

  AHRS_COMPUTE_H_PSI(state);

  if (!BMAG_UPDATE)
  {
      mag_psi = ahrs_psi_of_mag(state, mag);
      if(active != 0)
          err_psi = mag_psi - state->ahrs_psi;

      AHRS_WARP( &err_psi, PI);
      run_kalman( state, AHRS_R_PSI, err_psi, active );
      AHRS_DCM_OF_QUAT(state);
      AHRS_EULER_OF_DCM(state);
  }
  else
  {
      if((active != 0)&&(magEnable!=0))
      {
          incd = incd_of_mag(state->ahrs_phi, state->ahrs_theta, mag, I);
          mag_psi = psi_of_mag(state->ahrs_phi, state->ahrs_theta, mag, D);
          err_psi = mag_psi - state->ahrs_psi;
      }
      AHRS_WARP( &err_psi, PI);
      run_kalman( state, AHRS_R_PSI, err_psi, active );
      AHRS_DCM_OF_QUAT(state);
      AHRS_EULER_OF_DCM(state);
  }
}


/**
* Get Eulers form quaternions
*/
void Ahrs::eulers_of_quat(float* euler, const float* quat) const
{
  float dcm00 = 1.0f-2.0f*(quat[2]*quat[2] + quat[3]*quat[3]);
  float dcm01 =      2.0f*(quat[1]*quat[2] + quat[0]*quat[3]);
  float dcm02 =      2.0f*(quat[1]*quat[3] - quat[0]*quat[2]);
  float dcm12 =      2.0f*(quat[2]*quat[3] + quat[0]*quat[1]);
  float dcm22 = 1.0f-2.0f*(quat[1]*quat[1] + quat[2]*quat[2]);

  euler[0] = atan2f( dcm12, dcm22 );
  if(fabsf(dcm02) < 1.0f){
	  euler[1] = -asinf( dcm02 );
  }else{
      euler[1] = (dcm02 > 0.0f) ? -PI2 : PI2;
  }
  euler[2] = atan2f( dcm01, dcm00 );
}

/**
* Normalize quaternions
*/
void Ahrs::norm_quat(float* quat) const{
  float  mag = quat[0]*quat[0] + quat[1]*quat[1] + 
                quat[2]*quat[2] + quat[3]*quat[3];
  if(mag <= 0.0f)
  {
	  mag = 100000.0f;
  }else{
	  mag = 1.0f / sqrtf (mag);
  }
  quat[0] *= mag;
  quat[1] *= mag;
  quat[2] *= mag;
  quat[3] *= mag;
}


/** 
* Initialize the AHRS state data and covariance matrix.
* /param gyro
* /param accel
* /param psiFromHeading (or mag)
* /param airspeed
* output: ahrsState
*/
void Ahrs::ahrsInit(float psiFromHeading, const float* accel, const float* gyro, float airspeed, const float *mag)
{
	ahrs_stateT* state = &gAhrsState; 
	// P will be updated only on the non zero locations 
	memset (state->ahrs_P, 0, sizeof( state->ahrs_P ));

	for(int i=0 ; i<4 ; i++ )
		state->ahrs_P[i][i] = 1.0f;
	for(int i=4 ; i<7 ; i++ )
		state->ahrs_P[i][i] = 0.5f;

  //assume vehicle is still, so initial bias are gyro readings 
  //AHRS starts during FG flight
	state->ahrs_bias_p = 0.0f;
	state->ahrs_bias_q = 0.0f;
	state->ahrs_bias_r = 0.0f;

	state->psiDot = 0.0f;
	state->thetaDot = 0.0f;
	state->phiDot = 0.0f;
	state->ahrsUpdateFlag = 0;
	state->ahrsResetUpdatePhiConditionFlag = false;
	state->ahrsResetUpdateThetaConditionFlag = false;

    state->dt = AHRS_DT;
	state->dt_1 = AHRS_DT;

	state->airspeed_1 = airspeed;	
	state->airspeed_2 = airspeed;

	state->accXlin = 0.0f;
	state->lastAccXlin = 0.0f;


	state->ahrs_phi = ahrs_phi_of_accel(accel);

    if (Numbers::isValid(accel[AXIS_X]))
        state->ahrs_theta = ahrs_theta_of_accel(accel);
    else
        state->ahrs_theta = 0.0f;

    if(mag[3] == 1.0f)     //mag[3] = 1: magnetometer is present
    {
       state->ahrs_psi = ahrs_psi_of_mag( state, mag );
    }else{
       state->ahrs_psi = 0.0f;
    }

    state->updateSelector = 0;


	AHRS_QUAT_OF_EULER(state);
	AHRS_DCM_OF_QUAT(state);
    
    
	gAhrsData.p = 0.0f;
	gAhrsData.q = 0.0f;
	gAhrsData.r = 0.0f;
	gAhrsData.phi = 0.0f;
	gAhrsData.theta = 0.0f;
	gAhrsData.psi = 0.0f;
	gAhrsData.phiDot = 0.0f;
	gAhrsData.thetaDot = 0.0f;
	gAhrsData.psiDot = 0.0f;
	gAhrsData.ahrsUpdateFlag = 0;
	gAhrsData.incdError = 0;
	gAhrsData.f32EstAlt = 0.0f;
	gAhrsData.f32EstDeltaAlt = 0.0f;
	gAhrsData.f32EstVertSpeed = 0.0f;

	state->groundspeedDivider = 0;
	state->velocityXHistory[0] = 0.0f;
	state->velocityXHistory[1] = 0.0f;
	state->velocityXHistory[2] = 0.0f;

	state->velocityYHistory[0] = 0.0f;
	state->velocityYHistory[1] = 0.0f;
	state->velocityYHistory[2] = 0.0f;

	state->accelX = 0.0f;
	state->accelY = 0.0f;

	state->airspeedDivider = 0;
	state->airspeedHistory[0] = airspeed;
	state->airspeedHistory[1] = airspeed;
	state->airspeedHistory[2] = airspeed;	

	state->vertSpeedKF.f32Distance = 0.0f;
	state->vertSpeedKF.f32BiasAcc  = 0.0f;
	state->vertSpeedKF.f32Speed    = 0.0f;
}

/**
* Extracts gravity part from accelerators
*/
void Ahrs::ahrsAccelerometerModify(struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float airspeed) const
{
	accel[0] += ahrsState->accXlin;
	accel[1] += gyro[2] * airspeed * KPH_2_MS;
	accel[2] += gyro[1] * airspeed * KPH_2_MS;

	ahrsState->airspeed_2 = ahrsState->airspeed_1;
	ahrsState->airspeed_1 = airspeed;
}


/**
* Extracts gravity part from acceleration in Quad mode
*/

void Ahrs::ahrsQuadAccelModify(struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float velX, float velY) const
{
	accel[0] += ahrsState->accelX + gyro[2] * velY * KPH_2_MS;
	accel[1] += ahrsState->accelY + gyro[2] * velX * KPH_2_MS;
	accel[2] += gyro[1] * velX * KPH_2_MS - gyro[0] * velY * KPH_2_MS;
}
/** 
* Calculates Euler angles dots
*/
void Ahrs::ahrsCalculateDots(struct ahrs_stateT* ahrsState) const
{
	float  sinPhi = sinf(ahrsState->ahrs_phi);
	float  cosPhi = cosf(ahrsState->ahrs_phi);
	float  sinTheta = sinf(ahrsState->ahrs_theta);
	float  cosTheta = cosf(ahrsState->ahrs_theta);

	ahrsState->thetaDot = ahrsState->ahrs_q*cosPhi - ahrsState->ahrs_r*sinPhi;
	if(cosTheta == 0.0f)
	{
		ahrsState->psiDot = 0.0f;
	}else{
		ahrsState->psiDot = (ahrsState->ahrs_q*sinPhi + ahrsState->ahrs_r*cosPhi)/cosTheta ;
	}
	ahrsState->phiDot = ahrsState->ahrs_p + ahrsState->psiDot*sinTheta;

}



/** 
* The main AHRS function
* /param ahrsState
* /param gyro
* /param accel
* /param psiFromHeading (or mag)
* /param airspeed
* output: ahrsState
*/
void Ahrs::ahrsCompute(struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float psiFromHeading , float airspeed, float *mag, int magEnable, int flightMode, int version, float groundspeed)
{
    magValid = 0;

    if(mag[3] != 0.0f)
    {
        float norm = sqrtf(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);

        if(fabsf(norm-1.0f)<1.0f)
        {
            magValid = 1;
        }

        if(norm>0.0f)
        {
            mag[0] = mag[0]/norm;
            mag[1] = mag[1]/norm;
            mag[2] = mag[2]/norm;

        }
    }

    if(ahrsState->dt==0.0f)
    {
        ahrsState->dt_1 = ahrsState->dt;
        return;
    }

    //------------------------------------	
    ahrs_predict(ahrsState, gyro);
    //------------------------------------

    //save X acc before correction
    float accX = accel[0];

    float psd_theta = ahrsState->ahrs_theta;
    float psd_phi = ahrsState->ahrs_phi;


	if((airspeed > 20.0f))
	{
		ahrsAccelerometerModify(ahrsState, gyro, accel, airspeed);
	}


    ahrsState->ahrsUpdateFlag = 0;
    //--------------------------------------------------
    // dongnt11 edited
    float updated_phi = ahrs_phi_of_accel(accel);
    float updated_theta = ahrs_theta_of_accel(accel);

    ahrsState->ahrsResetUpdatePhiConditionFlag = ahrsCheckResetUpdatedCondition(psd_phi, updated_phi, 5.0f, resetUpdatePhiCounter, 2);
    ahrsState->ahrsResetUpdateThetaConditionFlag = ahrsCheckResetUpdatedCondition (psd_theta, updated_theta, 5.0f, resetUpdateThetaCounter, 2);
    //--------------------------------------------------

        if((abs(ahrsState->ahrs_p)<0.01f&&((abs(accel[2])<2.0f*9.81f))&&((abs(accel[1])<8.0f))&&((abs(accel[2])>3.0f))
        		&&((abs(updated_phi - psd_phi) < 0.05f) || ahrsState->ahrsResetUpdatePhiConditionFlag)

            &&(abs(ahrsState->ahrs_q)<0.1f)&&(abs(ahrsState->ahrs_r)<0.1f)&& (abs(accX)<2.0f)&&(lockUpdateCounter<=0))||((airspeed<20.0f)&&(lockUpdateCounter<=0)))
        {				
            ahrs_update_phi(ahrsState,accel,1,mag,magEnable);
//stara wersja-------------------
			if(version == 0)
			{
				ahrs_update_phi_from_mag(ahrsState,accel,1,mag,magEnable);
			}
//-------------------------------
			ahrsState->ahrsUpdateFlag = static_cast<INT8U>(ahrsState->ahrsUpdateFlag | 1);
        }
        else
        {
            ahrs_update_phi(ahrsState,accel,0,mag,magEnable);
        }
       
		if(flightMode == 1)
		{
// restricted conditions
			if ( ( 
				   (abs(ahrsState->accXlin)<0.1f) && 
				   (abs(accX)<2.0f) && 
				   ((abs(accel[0]) < abs(G * sinf(psd_theta + 0.5f * DEG_2_RAD))) &&
				   (abs(updated_theta - psd_theta) < 0.05f) || ahrsState->ahrsResetUpdateThetaConditionFlag) &&
				   (abs(gyro[1])<0.05f) &&
				   (lockUpdateCounter<=0)
		 		  )
				|| 
				(
				  (airspeed<20.0f) &&
				  (lockUpdateCounter<=0)
				) /*(airspeed<5.0f)*/
			)	//condtional update, X acceleration
			{	
				ahrs_update_theta(ahrsState,accel,1,mag,magEnable);
//old version---------
				if(version == 0)
				{
					ahrs_update_phi_from_mag(ahrsState,accel,1,mag,magEnable);
				};
//---------------------
				ahrsState->ahrsUpdateFlag = static_cast<INT8U>(ahrsState->ahrsUpdateFlag | 2);
			}
			else
			{
				ahrs_update_theta(ahrsState,accel,0,mag,magEnable);
			}
		}else{
//for landing
			if ( ( 
				   (abs(ahrsState->accXlin)<0.5f) && 
				   (abs(accX)<2.0f) && 
				   (abs(gyro[1])<0.05f) &&
				   (lockUpdateCounter<=0)
		 		 )
				|| 
				(
				  (airspeed<20.0f) &&
				  (lockUpdateCounter<=0)
				) /*(airspeed<5.0f)*/
			)	//condtional update, X acceleration
			{	
				ahrs_update_theta(ahrsState,accel,1,mag,magEnable);
				ahrsState->ahrsUpdateFlag = static_cast<INT8U>(ahrsState->ahrsUpdateFlag | 2);
			}
			else
			{
				ahrs_update_theta(ahrsState,accel,0,mag,magEnable);
			}
	
		}

// new version----------------
		if(version > 0)
		{

			//Phi update from g was done, do theta update from magntetometer	
			if(ahrsState->ahrsUpdateFlag==1)
			{
				ahrs_update_theta_from_mag(ahrsState,accel,1,mag,magEnable);
			}

			//Theta update from g was done, do phi update from magntetometer	
			if(ahrsState->ahrsUpdateFlag==2)
			{
				ahrs_update_phi_from_mag(ahrsState,accel,1,mag,magEnable);
			}
		}
//--------------------------

		//If both theta and phi were updated from g, don't use updates from magnetometers

		//test magnetometer
		gAhrsData.incdError = checkMagnetometer(ahrsState,mag,magEnable);

        //case UPDATE_PSI: {
        // magnetometer is present
        // we can add conditions from phi and theta
        // updates only when magnetometer on horizontal plane

        if ( (
				(magValid == 1) && 
		 		(abs(ahrsState->ahrs_phi)<0.26f) &&
				(abs(ahrsState->ahrs_theta)<0.26f) && 
				(lockUpdateCounter<=0)
			 ) //ograniczenie korekcji, tylko dla malych theta, phi
             ||
			 (
				(magValid == 1)&&	
				(airspeed<20.0f)&&
				(lockUpdateCounter<=0)
			 )
		   )
		{																												 //zwiekszenie phi do +-15deg, korekcje w krazeniu!
            ahrs_update_psi(ahrsState,psiFromHeading,1,mag,magEnable);
            ahrsState->ahrsUpdateFlag = static_cast<INT8U>(ahrsState->ahrsUpdateFlag | 4);
        }
        else
        {
            ahrs_update_psi(ahrsState,psiFromHeading,0,mag,magEnable);
        }

    ahrsState->updateSelector++;
    if(ahrsState->updateSelector>=UPDATE_NB)
        ahrsState->updateSelector=0;

    ahrsCalculateDots(ahrsState);
}


//if magEnable = 1, allows for magnetometer corrections for all Eulers angles
//if magEnable = 2, allows for magnetometer corrections only for psi

void Ahrs::ahrsMain(const struct Gauge::measurementT* measurement, const struct Gauge::magnetMeasurementT* magnetMeasurement, float airspeed, unsigned int time, int pressureSource, int magEnable,int flightMode, int version, int debugMsg, float groundSpeed, float track_deg)
{
    float gyro[3];
    float accel[3];
    float mag[4];

	float velX, velY; // for exclude forward acceleration
    //mag[3] = 1: magnetometer present, mag[3] = 0: magnetometer absent;
    float psiFromHeading = 0.0f;
	struct ahrs_stateT* ahrsState = &gAhrsState;
    float speed = 0.0f;

    mag[3] = 0.0f;
    if((magnetMeasurement->magnetometerPresent == 1)&& (magEnable != 0))
    {
        mag[3] = 1.0f;
        mag[0] = magnetMeasurement->magX;
        mag[1] = magnetMeasurement->magY;
        mag[2] = magnetMeasurement->magZ;
	}

	if(lockUpdateCounter>0)
	{
		lockUpdateCounter--;
	}

    float actTime = time * 0.0001f; //[*100us]
    float deltaTime = actTime - ahrsState->lastTime;
	ahrsState->lastTime = actTime;
	ahrsState->dt_1 = ahrsState->dt;
	if(deltaTime == 0.001f)
    {
        ahrsState->dt = 1.0f/100.01f;
    }else{        
        ahrsState->dt = deltaTime;
    } 

	speed = airspeed;
    
	//decimation by 3
	//linear acceleration evaluation
	
	//GT: sprawdziæ dla danych pomiarowych z czujnika ciœnienia ró¿nicowego:
	//	  1.)czy potrzebna jest decymacja
	//	  2.)czy dla IAS w³¹czona jest filtracja, jeœli tak to wyliczyæ ias tutaj aby pozbyæ siê opóŸnienia 


	(ahrsState->airspeedDivider)++;
	
	if(ahrsState->airspeedDivider == 9)
	{
		ahrsState->airspeedDivider=0;
		ahrsState->airspeedHistory[2] = ahrsState->airspeedHistory[1];
		ahrsState->airspeedHistory[1] = ahrsState->airspeedHistory[0];
		ahrsState->airspeedHistory[0] = speed;
		ahrsState->accXlin = (ahrsState->airspeedHistory[0]-ahrsState->airspeedHistory[2])/2.0f/ahrsState->dt*KPH_2_MS/10.0f;
                                                                    //(/4.0 <- decymacja,odstep w czasie wiekszy!) 

	}
   
	// decimation by 24 for groundspeed
	(ahrsState->groundspeedDivider)++;

	float velNorth = groundSpeed * cosf(track_deg * DEG_2_RAD);
	float velEast  = groundSpeed * sinf(track_deg * DEG_2_RAD);

	velX =  velNorth * cosf(ahrsState->ahrs_psi) + velEast * sinf(ahrsState->ahrs_psi);
	velY = -velNorth * sinf(ahrsState->ahrs_psi) + velEast * cosf(ahrsState->ahrs_psi);

	if(ahrsState->groundspeedDivider == 11)
	{
		// ground speed
		ahrsState->groundspeedDivider = 0;
		ahrsState->velocityXHistory[2] = ahrsState->velocityXHistory[1];
		ahrsState->velocityXHistory[1] = ahrsState->velocityXHistory[0];
		ahrsState->velocityXHistory[0] = velX;

		ahrsState->velocityYHistory[2] = ahrsState->velocityYHistory[1];
		ahrsState->velocityYHistory[1] = ahrsState->velocityYHistory[0];
		ahrsState->velocityYHistory[0] = velY;

		ahrsState->accelX = (ahrsState->velocityXHistory[0]-ahrsState->velocityXHistory[2])/2.0f/ahrsState->dt*KPH_2_MS/12.0f;

		ahrsState->accelY = (ahrsState->velocityYHistory[0]-ahrsState->velocityYHistory[2])/2.0f/ahrsState->dt*KPH_2_MS/12.0f;
                                                                //(/4.0 <- decymacja,odstep w czasie wiekszy!)
	}

    accel[0] = G*measurement->accXL;
	accel[1] = G*measurement->accYL;
	accel[2] = G*measurement->accZH;  // RF: modyfikacja dla VT1 (na accL - dargania nie do wyeliminowania)

	if (version == 1)
	{
		gyro[0] = (fabsf(measurement->gyroXL) > 1.0f) ? measurement->gyroXH : measurement->gyroXL;
		gyro[1] = (fabsf(measurement->gyroYL) > 1.0f) ? measurement->gyroYH : measurement->gyroYL;
		gyro[2] = (fabsf(measurement->gyroZL) > 1.0f) ? measurement->gyroZH : measurement->gyroZL;
	}
	else if (version == 2)
	{
		gyro[0] = measurement->gyroXH;
		gyro[1] = measurement->gyroYH;
		gyro[2] = measurement->gyroZH;
	}

	if(ahrsState->ahrsInitialized == 0)
    {
        ahrsInit(psiFromHeading, accel, gyro, speed, mag);
        ahrsState->ahrsInitialized = 1;  
		ahrsState->lastTime = actTime;
        
    }else{
    
//===============================================================================           
        ahrsCompute(ahrsState, gyro, accel, psiFromHeading, airspeed, mag, magEnable, flightMode, version, groundSpeed);
//===============================================================================
    }

	estVertSpeed (measurement, ahrsState);

	gAhrsData.p = ahrsState->ahrs_p;
	gAhrsData.q = ahrsState->ahrs_q;
	gAhrsData.r = ahrsState->ahrs_r;
	gAhrsData.phi = ahrsState->ahrs_phi;
	gAhrsData.theta = ahrsState->ahrs_theta;
	gAhrsData.psi = ahrsState->ahrs_psi;
	gAhrsData.phiDot = ahrsState->phiDot;
	gAhrsData.thetaDot = ahrsState->thetaDot; 
	gAhrsData.psiDot = ahrsState->psiDot;
	gAhrsData.ahrsUpdateFlag = ahrsState->ahrsUpdateFlag;
	gAhrsData.f32EstAlt = ahrsState->vertSpeedKF.f32Distance;
	gAhrsData.f32EstDeltaAlt = ahrsState->vertSpeedKF.f32Speed * ahrsState->dt;
	gAhrsData.f32EstVertSpeed = ahrsState->vertSpeedKF.f32Speed;
}


void Ahrs::ahrsInitFG(float psiFromHeading, const float* accel, const float* gyro, float airspeed,  float phi, float theta, float agl)
{
	ahrs_stateT* state = &gAhrsState; 
	// P will be updated only on the non zero locations 
	memset( state->ahrs_P, 0, sizeof( state->ahrs_P ) );

	for(int i=0 ; i<4 ; i++ )
		state->ahrs_P[i][i] = 1.0f;
	for(int i=4 ; i<7 ; i++ )
		state->ahrs_P[i][i] = 0.5f;

  //assume vehicle is still, so initial bias are gyro readings 
  //AHRS starts during FG flight
	state->ahrs_bias_p = 0.0f;
	state->ahrs_bias_q = 0.0f;
	state->ahrs_bias_r = 0.0f;

	state->psiDot = 0.0f;
	state->thetaDot = 0.0f;
	state->phiDot = 0.0f;

    state->dt = AHRS_DT;
	state->dt_1 = AHRS_DT;

	state->airspeed_1 = airspeed;	
	state->airspeed_2 = airspeed;

	state->accXlin = 0.0f;
	state->lastAccXlin = 0.0f;


	state->ahrs_phi = phi;//ahrs_phi_of_accel(accel);
	state->ahrs_theta = theta;//ahrs_theta_of_accel(accel);
	state->ahrs_psi = psiFromHeading;
	state->updateSelector = 0;

	AHRS_QUAT_OF_EULER(state);
	AHRS_DCM_OF_QUAT(state);
	state->ahrs_psi = psiFromHeading;//ahrs_psi_of_mag( state, mag );

	gAhrsData.p = 0.0f;
	gAhrsData.q = 0.0f;
	gAhrsData.r = 0.0f;
	gAhrsData.phi = 0.0f;
	gAhrsData.theta = 0.0f;
	gAhrsData.psi = 0.0f;
	gAhrsData.phiDot = 0.0f;
	gAhrsData.thetaDot = 0.0f;
	gAhrsData.psiDot = 0.0f;
	gAhrsData.f32EstVertSpeed = 0.0f;
	gAhrsData.f32EstAlt = 0.0f;
	gAhrsData.f32EstDeltaAlt = 0.0f;

	state->groundspeedDivider = 0;
	state->velocityXHistory[0] = 0.0f;
	state->velocityXHistory[1] = 0.0f;
	state->velocityXHistory[2] = 0.0f;

	state->velocityYHistory[0] = 0.0f;
	state->velocityYHistory[1] = 0.0f;
	state->velocityYHistory[2] = 0.0f;

	state->accelX = 0.0f;
	state->accelY = 0.0f;

	state->airspeedDivider = 0;
	state->airspeedHistory[0] = airspeed;
	state->airspeedHistory[1] = airspeed;
	state->airspeedHistory[2] = airspeed;	

	state->vertSpeedKF.f32Distance = agl;
	state->vertSpeedKF.f32BiasAcc  = 0.0f;
	state->vertSpeedKF.f32Speed    = 0.0f;

	state->ahrsResetUpdatePhiConditionFlag = false;
	state->ahrsResetUpdateThetaConditionFlag = false;

}

/**
* Calculates Euler angles from gyros
*/
void Ahrs::ahrsPlain(struct ahrs_stateT* ahrsState) const
{
	float  sinPhi = sinf(ahrsState->plain_phi);
	float  cosPhi = cosf(ahrsState->plain_phi);
	float  sinTheta = sinf(ahrsState->plain_theta);
	float  cosTheta = cosf(ahrsState->plain_theta);

	ahrsState->thetaDot = ahrsState->ahrs_q*cosPhi - ahrsState->ahrs_r*sinPhi;
	if(cosTheta == 0.0f)
	{
		ahrsState->psiDot = 0.0f;
	}else{
		ahrsState->psiDot = (ahrsState->ahrs_q*sinPhi + ahrsState->ahrs_r*cosPhi)/cosTheta ;
	}
	ahrsState->phiDot = ahrsState->ahrs_p + ahrsState->psiDot*sinTheta;

	//integration
	ahrsState->plain_theta += ahrsState->thetaDot * ahrsState->dt; 
	ahrsState->plain_psi += ahrsState->psiDot * ahrsState->dt;
	ahrsState->plain_phi += ahrsState->phiDot * ahrsState->dt;
}

void Ahrs::ahrsMainFG(const struct Gauge::measurementT* measurement, float airspeed, unsigned int time,  float phi, float theta, float psi, int version, float groundSpeed, float track_deg)
{
    float gyro[3];
    float accel[3];
    float mag[4];

	float velX, velY;
    //mag[3] = 1: magnetometer present, mag[3] = 0: magnetometer absent;
    float psiFromHeading = 0.0f;
	struct ahrs_stateT* ahrsState = &gAhrsState;
	int magEnable = 0;

    mag[3] = 0.0f; //magnetometer not available

	float actTime = time * 0.0001f; //[*100us]
	float deltaTime = actTime - ahrsState->lastTime;
	
	if(lockUpdateCounter>0)
	{
		lockUpdateCounter--;
	}
	
	//There was probably simlevel switch (4->1->4->1)
	if(deltaTime>(10.0f/100.01f))
	{
		ahrsState->ahrsInitialized = 0;
	}

	ahrsState->lastTime = actTime;
	ahrsState->dt_1 = ahrsState->dt;
	if(deltaTime == 0.0f)
    {
        ahrsState->dt = (float)(1.0f/100.01f);
		//ahrsState->dt = 0.0f;
    }else{        
        ahrsState->dt = deltaTime;
    } 


	float speed = airspeed;
    
	//decimation by 12
	//linear acceleration evaluation

	ahrsState->airspeedDivider++;

	ahrsState->airspeedDivider=0;
	ahrsState->airspeedHistory[2] = ahrsState->airspeedHistory[1];
	ahrsState->airspeedHistory[1] = ahrsState->airspeedHistory[0];
	ahrsState->airspeedHistory[0] = speed;//airspeed;
	ahrsState->accXlin = (ahrsState->airspeedHistory[0]-ahrsState->airspeedHistory[2])/(ahrsState->dt+ahrsState->dt_1)*KPH_2_MS;
    Numbers::assure (ahrsState->accXlin, 0.0f);


    accel[0] =  -G*measurement->accXL;
	accel[1] =  -G*measurement->accYL;
	accel[2] =  G*measurement->accZL;

    gyro[0] = measurement->gyroXL;
    gyro[1] = measurement->gyroYL;
    gyro[2] = measurement->gyroZL;
    
	if(psi<PI)
	{
		psiFromHeading = psi;
	}else{
		psiFromHeading = (psi-2.0f*PI);
	}    

    if(ahrsState->ahrsInitialized == 0)
    {
		ahrsInitFG(psiFromHeading, accel, gyro, speed, phi, theta, measurement->agl);
        ahrsState->ahrsInitialized = 1;  
		ahrsState->lastTime = actTime;
		ahrsState->plain_theta = ahrsState->ahrs_theta;
		ahrsState->plain_psi = ahrsState->ahrs_psi;
		ahrsState->plain_phi = ahrsState->ahrs_phi;
        
    }else
	{
			// decimation by 24 for groundspeed
		(ahrsState->groundspeedDivider)++;

		float velNorth = groundSpeed * cosf(track_deg * DEG_2_RAD);
		float velEast  = groundSpeed * sinf(track_deg * DEG_2_RAD);

		velX =  velNorth * cosf(ahrsState->ahrs_psi) + velEast * sinf(ahrsState->ahrs_psi);
		velY = -velNorth * sinf(ahrsState->ahrs_psi) + velEast * cosf(ahrsState->ahrs_psi);
	
		if(ahrsState->groundspeedDivider == 11)
		{
			// ground speed
			ahrsState->groundspeedDivider = 0;
			ahrsState->velocityXHistory[2] = ahrsState->velocityXHistory[1];
			ahrsState->velocityXHistory[1] = ahrsState->velocityXHistory[0];
			ahrsState->velocityXHistory[0] = velX;

			ahrsState->velocityYHistory[2] = ahrsState->velocityYHistory[1];
			ahrsState->velocityYHistory[1] = ahrsState->velocityYHistory[0];
			ahrsState->velocityYHistory[0] = velY;

			ahrsState->accelX = (ahrsState->velocityXHistory[0]-ahrsState->velocityXHistory[2])/2.0f/ahrsState->dt*KPH_2_MS/12.0f; 

			ahrsState->accelY = (ahrsState->velocityYHistory[0]-ahrsState->velocityYHistory[2])/2.0f/ahrsState->dt*KPH_2_MS/12.0f; 
                                                                    //(/4.0 <- decymacja,odstep w czasie wiekszy!) 
		}
//===============================================================================           
        ahrsCompute(ahrsState, gyro, accel, psiFromHeading, airspeed, mag, magEnable, 1, version, groundSpeed);
//===============================================================================
    }

	estVertSpeed (measurement, ahrsState);

	gAhrsData.p = ahrsState->ahrs_p;
	gAhrsData.q = ahrsState->ahrs_q;
	gAhrsData.r = ahrsState->ahrs_r;
	gAhrsData.phi = ahrsState->ahrs_phi;
	gAhrsData.theta = ahrsState->ahrs_theta;
	gAhrsData.psi = ahrsState->ahrs_psi;
	gAhrsData.phiDot = ahrsState->phiDot;
	gAhrsData.thetaDot = ahrsState->thetaDot; 
	gAhrsData.psiDot = ahrsState->psiDot;
	gAhrsData.f32EstAlt = ahrsState->vertSpeedKF.f32Distance;
	gAhrsData.f32EstDeltaAlt = ahrsState->vertSpeedKF.f32Speed * ahrsState->dt;
	gAhrsData.f32EstVertSpeed = ahrsState->vertSpeedKF.f32Speed;
}

//This function locks AHRS updates (based on accelerometers and magnetometers) for time interval defined by seconds
void Ahrs::lockUpdate(int seconds){
	lockUpdateCounter = seconds * gaugeCyclesPerSecond;
}

//This function unlocks AHRS updates
void Ahrs::unlockUpdate(void){
	lockUpdateCounter = 0;
}

//Returns true if update is locked
bool Ahrs::isUpdateLocked(void) const
{
    return (lockUpdateCounter != 0);
}

//cross product calculation for 3-D vectors
// c = a x b
void Ahrs::crossProduct(const float *a, const float *b, float *c) const
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
}


/**
* Correction from magnetometer
*/
float Ahrs::checkMagnetometer(struct ahrs_stateT *state, const float *mag, int magEnable) const
{
	float I = Pi_2minusInclination;
	float incd = 0;  

	if(magEnable!=0)	
		incd = incd_of_mag(state->ahrs_phi, state->ahrs_theta, mag, I);
	
	return incd;
}


float Ahrs::incd_of_mag(float phi, float theta, const float *mag, float I) const
{
	// D = Declination;
	// I = pi/2 - Inclination;

	float sphi = sinf(phi);
	float cphi = cosf(phi);
	float stheta = sinf(theta);
	float ctheta = cosf(theta);

	float z = -stheta*mag[0] + sphi*ctheta*mag[1] + cphi*ctheta*mag[2];
	float incd = asinf(z) - (PI2-I);

    Numbers::assure(incd, 0.0f);

	return incd;
}


int Ahrs::sasbc(const float *A, float epsilon, float *x, float *w) const
{
	*w = 0;

	float tmp = sqrtf(fabsf(A[0]*A[0]+A[1]*A[1]));

	if(fabsf(A[2]) >= ((1-epsilon)*tmp))
	{
		*w = 1;
		x[0] = PI2;
		if(A[2] < 0.0f)
			x[0] = -PI2;
	}
	else
	{
        //WK: ochrona przed tmp == 0
        if (tmp > 0.0f)
        {
            tmp = A[2]/tmp;
            x[0] = asinf(tmp);
        }
	}

	x[1] = (PI - fabsf(x[0]));
	if(x[0] < 0)
	{
		x[1] = -x[1];
	}

	tmp = atan2f(A[1],A[0]);
	x[0] = x[0] - tmp;
	x[1] = x[1] - tmp;

	
	if(x[0] > PI)
	{
		x[0] = x[0] - 2*PI;
	}
	if(x[0] <= -PI)
	{
		x[0] = x[0] + 2*PI;
	}
	if(x[1] > PI)
	{
		x[1] = x[1] - 2*PI;
	}
	if(x[1] <= -PI)
	{
		x[1] = x[1] + 2*PI;
	}


	return 0;
}


float Ahrs::phi_of_mag(float phi, float theta, const float *mag, float I, float *w) const
{
	float A[3];
    float phic[2] = {0.0f, 0.0f};

	// D = Declination;
	// I = pi/2 - Inclination;

	float stheta = sinf(theta);
	float ctheta = cosf(theta);
	float cI = cosf(I);

	A[0] = ctheta*mag[1];
	A[1] = ctheta*mag[2];
	A[2] = cI + stheta*mag[0];

	sasbc(A, 0.03f, phic, w);

	if(fabsf(phic[0] - phi) > fabsf(phic[1] - phi))
	{
		phic[0] = phic[1];
	}

	return phic[0];
}


float Ahrs::theta_of_mag(float phi, float theta, const float *mag, float I, float *w) const
{
	float A[3];
	float thetac[2] = {0.0f, 0.0f};

	// D = Declination;
	// I = pi/2 - Inclination;

	float sphi = sinf(phi);
	float cphi = cosf(phi);
	float cI = cosf(I);

	A[0] = -mag[0];
	A[1] = sphi*mag[1] + cphi*mag[2];
	A[2] = cI;

	sasbc(A, 0.03f, thetac, w);

	if(fabsf(thetac[0] - theta) > fabsf(thetac[1] - theta))
	{
		thetac[0] = thetac[1];
	}

	return thetac[0];
}


float Ahrs::psi_of_mag(float phi, float theta, const float *mag, float D) const
{
	// D = Declination;
	// I = pi/2 - Inclination;

	float sphi = sinf(phi);
	float cphi = cosf(phi);
	float stheta = sinf(theta);
	float ctheta = cosf(theta);

	float x = ctheta*mag[0] + sphi*stheta*mag[1] + cphi*stheta*mag[2];
	float y = cphi*mag[1] - sphi*mag[2];

	float psi = -atan2f(y,x);

	psi = psi + D;
	if(psi > PI)
	{
		psi = psi - 2*PI;
	}
	if(psi <= -PI)
	{
		psi = psi + 2*PI;
	}

	return psi;
}

void Ahrs::estVertSpeed(const struct Gauge::measurementT* measurement, ahrs_stateT* ahrs_state)
{
	//< dongnt11 for Estimating the climb rate by integrating the AccZ in NED
	AHRS_DCM_OF_QUAT(ahrs_state);
	float fDcm1 = -ahrs_state->ahrs_dcm02 * ahrs_state->ahrs_dcm22;
	float fDcm2 = -ahrs_state->ahrs_dcm12;
	float fDcm3 =  ahrs_state->ahrs_dcm22;

	// Modified Acceleration in Quad land mode if it crashed to the ground
	float accX = measurement->accXL;
	float accY = measurement->accYL;
	float accZ = measurement->accZL;

	float fAccZ_NED_G = fDcm1 * accX + fDcm2 * accY + fDcm3 * accZ;
	float fAccZ_NED = -G * fAccZ_NED_G - G;

	// integratation
	float fTsample_2 = ahrs_state->dt * ahrs_state->dt/2;
	float fAlt = ahrs_state->vertSpeedKF.f32Distance + ahrs_state->dt * ahrs_state->vertSpeedKF.f32Speed - fTsample_2 * ahrs_state->vertSpeedKF.f32BiasAcc + fTsample_2 * fAccZ_NED;
	float fClimbRate = ahrs_state->vertSpeedKF.f32Speed - ahrs_state->dt * ahrs_state->vertSpeedKF.f32BiasAcc + ahrs_state->dt * fAccZ_NED;
	float fBiasAccZ = ahrs_state->vertSpeedKF.f32BiasAcc;
	// Correction
	float fAglErr = measurement->agl - fAlt;
	ahrs_state->vertSpeedKF.f32Distance  = fAlt + KF_CR_ALT * fAglErr;
	ahrs_state->vertSpeedKF.f32Speed     = fClimbRate + KF_CR_CR * fAglErr;
	ahrs_state->vertSpeedKF.f32BiasAcc   = fBiasAccZ + KF_CR_BIAS * fAglErr;
}

void Ahrs::accelBody2NED (const struct Gauge::measurementT* measurement, struct ahrs_stateT* ahrs_state, float *accel_NED, bool simulation)
{
	// North
	float north_dcm01 = cos(ahrs_state->ahrs_theta);
	float north_dcm02 = 0.0f;
	float north_dcm03 = ahrs_state->ahrs_dcm02;

	float AccX = north_dcm01 * measurement->accXL + north_dcm02 * measurement->accYL + north_dcm03 * measurement->accZH;
	if (simulation)
		accel_NED[AXIS_X] = G * AccX;
	else
		accel_NED[AXIS_X] = -G * AccX;

	// East
	float east_dcm01 = -ahrs_state->ahrs_dcm12 * ahrs_state->ahrs_dcm02;
	float east_dcm02 =  ahrs_state->ahrs_dcm22;
	float east_dcm03 =  ahrs_state->ahrs_dcm12;

	float AccY = east_dcm01 * measurement->accXL + east_dcm02 * measurement->accYL + east_dcm03 * measurement->accZH;
	if (simulation)
		accel_NED[AXIS_Y] = G * AccY;
	else
		accel_NED[AXIS_Y] = -G * AccY;
}

void Ahrs::estimateSpeed (float measurement, float accel_NED, float dt, struct StateKalmanFilter* position, bool update)
{
	// Prediction
	float fTsample_2 = dt * dt/2;
	float fDistance  = position->f32Distance + dt * position->f32Speed   - fTsample_2 * position->f32BiasAcc + fTsample_2 * accel_NED;
	float fSpeed     = position->f32Speed    - dt * position->f32BiasAcc + dt * accel_NED;
	float fBiasAcc   = position->f32BiasAcc;
	// Correction
	//float fMeasureErr = measurement - fDistance;
	// GroundSpeed for Refferent
	float fMeasureErr = measurement - fSpeed;
	if(update)
	{
		position->f32Distance  = fDistance + KF_GPS_X  * fMeasureErr;
		position->f32Speed     = fSpeed    + KF_GPS_VX * fMeasureErr;
		position->f32BiasAcc   = fBiasAcc  + KF_GPS_BX * fMeasureErr;
	}
}

void Ahrs::estimatePosition (float measurement, float accel_NED, float dt, struct StateKalmanFilter* position, bool update)
{
    // Prediction
    //float fTsample_2 = dt * dt/2;
    //float fDistance  = position->f32Distance + dt * position->f32Speed   - fTsample_2 * position->f32BiasAcc + fTsample_2 * accel_NED;
    //float fSpeed     = position->f32Speed    - dt * position->f32BiasAcc + dt * accel_NED;
    //float fBiasAcc   = position->f32BiasAcc;
    float fDistance  = position->f32Distance;
    float fSpeed     = position->f32Speed;
    float fBiasAcc   = position->f32BiasAcc;
    // Correction
    //float fMeasureErr = measurement - fDistance;
    // Position for Refferent
    float fMeasureErr = measurement - fDistance;
    if(update)
    {
        position->f32Distance  = fDistance + KF_CR_ALT  * fMeasureErr;
        position->f32Speed     = fSpeed    + KF_CR_CR * fMeasureErr;
        position->f32BiasAcc   = fBiasAcc  + KF_CR_BIAS * fMeasureErr;
    }
}

bool Ahrs::ahrsCheckResetUpdatedCondition (float psdAngle, float updatedAngle, float err_deg, int &resetCounter, int second)
{
	bool resetUpdatedConditionFlag = false;

	if( abs(psdAngle - updatedAngle) < err_deg * DEG_2_RAD)
	{
		resetCounter = second * gaugeCyclesPerSecond;
	}
	else
	{
		if(resetCounter < 0)
		{
			resetUpdatedConditionFlag = true;
		}

		resetCounter = resetCounter - 1;
	}

	return resetUpdatedConditionFlag;
}

