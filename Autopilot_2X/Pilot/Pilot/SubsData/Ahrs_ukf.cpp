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
const float Ahrs::AHRS_DT      = 1.0f / 100.01f;

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
  double s0,s1,s2,s3;
  double ahrs_FP[4][4];
  double ahrs_Pdot[4][4];
  //F represents the Jacobian of the derivative of the system with respect
  //its states.  We do not allocate the bottom three rows since we know that
  //the derivatives of bias_dot are all zero.
  double ahrs_F[4][4] = {0};
  double candidate_Pfin[4][4] = {0};
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


    //   compute Phik
	float ahrs_Phik[4][4] = {0};
	ahrs_Phik[0][0] = ahrs_Phik[1][1] = ahrs_Phik[2][2] = ahrs_Phik[3][3] = 1.0f;
	ahrs_Phik[1][0] = ahrs_Phik[2][3] = gyro[0] * 0.5f * ahrs_dt;
	ahrs_Phik[2][0] = ahrs_Phik[3][1] = gyro[1] * 0.5f * ahrs_dt;
	ahrs_Phik[3][0] = ahrs_Phik[1][2] = gyro[2] * 0.5f * ahrs_dt;

	ahrs_Phik[0][1] = ahrs_Phik[3][2] = -ahrs_Phik[1][0];
	ahrs_Phik[0][2] = ahrs_Phik[1][3] = -ahrs_Phik[2][0];
	ahrs_Phik[0][3] = ahrs_Phik[2][1] = -ahrs_Phik[3][0];



  // update state 
  // quat_dot = Wxq(pqr) * quat 
  state->ahrs_q0     = ahrs_Phik[0][0] * state->ahrs_q0 + ahrs_Phik[0][1] * state->ahrs_q1 + ahrs_Phik[0][2] * state->ahrs_q2 + ahrs_Phik[0][3] * state->ahrs_q3;
  state->ahrs_q1     = ahrs_Phik[1][0] * state->ahrs_q0 + ahrs_Phik[1][1] * state->ahrs_q1 + ahrs_Phik[1][2] * state->ahrs_q2 + ahrs_Phik[1][3] * state->ahrs_q3;
  state->ahrs_q2     = ahrs_Phik[2][0] * state->ahrs_q0 + ahrs_Phik[2][1] * state->ahrs_q1 + ahrs_Phik[2][2] * state->ahrs_q2 + ahrs_Phik[2][3] * state->ahrs_q3;
  state->ahrs_q3     = ahrs_Phik[3][0] * state->ahrs_q0 + ahrs_Phik[3][1] * state->ahrs_q1 + ahrs_Phik[3][2] * state->ahrs_q2 + ahrs_Phik[3][3] * state->ahrs_q3;
  
  
  // normalize quaternion 
  AHRS_NORM_QUAT(state);
  
  AHRS_DCM_OF_QUAT(state);
  AHRS_EULER_OF_DCM(state);

  // update covariance
  // Pdot = F*P*F' + Q
  // P += Pdot * dt

  for (int j=0; j<4; j++) {
      s0=s1=s2=s3=0.0f;
      for (int k=0; k<4; k++) {
          double tmp = state->ahrs_Pinit[k][j];
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
      for (int k=0; k<4; k++) {
          double tmp = ahrs_F[j][k];
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

  //thanhnt: Xay dung ma tran cap nhat

  for (int j=0; j<4; j++){
      candidate_Pfin[0][j] = state->ahrs_Pinit[0][j] + (ahrs_Pdot[0][j] * ahrs_dt);
      candidate_Pfin[1][j] = state->ahrs_Pinit[1][j] + (ahrs_Pdot[1][j] * ahrs_dt);
      candidate_Pfin[2][j] = state->ahrs_Pinit[2][j] + (ahrs_Pdot[2][j] * ahrs_dt);
      candidate_Pfin[3][j] = state->ahrs_Pinit[3][j] + (ahrs_Pdot[3][j] * ahrs_dt);
  }


  //thanhnt: kiem tra dieu khien xac dinh duong cua ma tran truoc khi nap vao Pfin
  double Pfin_current[4][4];
  for (int i=0; i<4;i++)
	  {
		  for (int j=0; j<4;j++)
		  {
			  Pfin_current[i][j] = state->ahrs_Pfin[i][j];
		  }
	  }
  for (int i=0; i<4;i++)
	  {
		  for (int j=0; j<4;j++)
		  {
			  state->ahrs_Pfin[i][j] = candidate_Pfin[i][j];
		  }
	  }
  if (!Cholesky_Decomposition(state))
  {
	  for (int i=0; i<4;i++)
	  {
		  for (int j=0; j<4;j++)
		  {
			  state->ahrs_Pfin[i][j] = Pfin_current[i][j];
		  }
	  }
  }
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

void Ahrs::run_kalman( struct ahrs_stateT *state, float Rnoise[3][3], float error[3], int active) const
{
  const int L=4; //dimension
  double alpha = 0.0001f; //parameter
  float beta = 2.0f ;// parameter
  double lambda;
  double gamma;
  lambda = alpha*alpha*L-L; //parameter
  gamma = alpha*sqrtf(float(L)); //parameter
  // vector of variables
  float X[L];
  X[0] = state->ahrs_q0;
  X[1] = state->ahrs_q1;
  X[2] = state->ahrs_q2;
  X[3] = state->ahrs_q3;
  
  ahrs_stateT *state_compare;
  state_compare = state;
  
  //thanhnt : check if the matrix is possitive definite before updating.
  if (Cholesky_Decomposition(state))
  {
	   
	  //thanhnt: build conference matrix Chi and output Y
	  double Chi[L][2*L+1];
	  memset(Chi, 0, sizeof(Chi));
	  double Y[3][2*L+1];
	  for (int i=0; i<2*L+1; i++)
	  {
		  if (i<L)
		  {
			  for (int j=0; j<L;j++)
			  {
				  Chi[j][i] = X[j] + gamma*state->chol_lower[j][i];
			  }
		  }
		  else if (i<2*L)
		  {   
			  for (int j=0; j<L;j++)
			  {
				  Chi[j][i] = X[j] - gamma*state->chol_lower[j][i-L];
			  }
		  }
		  else
		  {
			  for (int j=0; j<L;j++)
			  {
				  Chi[j][i]=X[j];
			  }
		  }
		  // thanhnt: build output Y based on structure ahrs_stateT
		  state_compare->ahrs_q0 = Chi[0][i];
		  state_compare->ahrs_q1 = Chi[1][i];
		  state_compare->ahrs_q2 = Chi[2][i];
		  state_compare->ahrs_q3 = Chi[3][i];

		  AHRS_NORM_QUAT(state_compare);
		  AHRS_DCM_OF_QUAT(state_compare);
		  AHRS_EULER_OF_DCM(state_compare);
		  Y[0][i] = state_compare->ahrs_phi;
		  Y[1][i] = state_compare->ahrs_theta;
		  Y[2][i] = state_compare->ahrs_psi;
	  }
	  //thanhnt: output predict Y_pre
	  double Y_pre[3];
	  Y_pre[0] = 2*lambda*Y[0][2*L];
	  Y_pre[1] = 2*lambda*Y[1][2*L];
	  Y_pre[2] = 2*lambda*Y[2][2*L];
	  for (int i=0;i<2*L;i++)
	  {
		  Y_pre[0]+=Y[0][i];
		  Y_pre[1]+=Y[1][i];
		  Y_pre[2]+=Y[2][i];
	  }
	  Y_pre[0] /= (2*(L+lambda));
	  Y_pre[1] /= (2*(L+lambda));
	  Y_pre[2] /= (2*(L+lambda));
	  //thanhnt: error between Y and Y_pre
	  double er_Y[3][2*L+1];
		for (int i=0;i<2*L+1;i++)
		{
			for (int j = 0; j<3;j++)
			{
				er_Y[j][i] = Y[j][i];
			}
		}
	  for (int i=0;i<2*L+1;i++)
	  {
		  er_Y[0][i] = Y[0][i] - Y_pre[0];
		  er_Y[1][i] = Y[1][i] - Y_pre[1];
		  er_Y[2][i] = Y[2][i] - Y_pre[2];
	  }
	  //thanhnt: build correlative matrix Pyy
	  double P_yy[3][3];
	  for (int i =0; i<3; i++)
	  {
		  for(int j =0; j<3; j++)
		  {
			  P_yy[j][i] = Rnoise[j][i] * 2 * (L + lambda) + 2 * (lambda + (L + lambda) * (1.0f - alpha * alpha + beta))* er_Y[j][2*L] * er_Y[i][2*L];
		  }
	  }
	  //thanhnt: build correlative matrix Pxy
	  double P_xy[4][3];
	  memset(P_xy,0,sizeof(P_xy));
	  for (int i=0; i<2*L; i++)
	  {
		  for(int j=0; j<3; j++)
		  {
			  for(int k=0; k<3; k++)
			  {
				  P_yy[k][j] += er_Y[k][i] * er_Y[j][i];
			  } 
			  for(int k=0; k<L; k++)
			  {
				  P_xy[k][j] += (Chi[k][i]-X[k]) * er_Y[j][i];		  
			  }
		  }
	  }
	  for (int i=0; i<3; i++)
	  {
		  for (int j=0; j<L; j++)
		  {
			  P_xy[j][i] /= (2*(L+lambda));
		  }
		  for (int j=0; j<3;j++)
		  {
			  P_yy[j][i] /= (2*(L+lambda));
		  }
	  }
	  invMatrix_3(state, P_yy);
	  memset(state->ahrs_K,0,sizeof(state->ahrs_K));
	  for (int i=0; i<L; i++)
	  {
		  for (int j=0; j<3; j++)
		  {
			  for (int k=0; k<3; k++)
			  {
			     state->ahrs_K[i][j] += P_xy[i][k]*state->invPyy[k][j];  
			  }
		  }		  
	  }
	  //thanhnt: build matrix Pinit for updating.
	  memset(state->ahrs_Pinit,0,sizeof(state->ahrs_Pinit));
	  for  (int i=0; i<L; i++)
	  {
		  for (int j=0; j<L; j++)
		  {
			  for (int k=0; k<3; k++)
			  {
				  state->ahrs_Pinit[i][j] -= P_xy[i][k]*state->ahrs_K[j][k]; 
			  }
			  state->ahrs_Pinit[i][j] += state->ahrs_Pfin[i][j];
		  }
		  
	  }
	  //thanhnt: calculate K*error to update attitude.
	  double K_times_error[L];
	  memset(K_times_error, 0, sizeof(K_times_error));
	  for (int i=0; i<L;i++)
	  {
		  for (int j=0; j<3;j++)
		  {
			  K_times_error[i] += state->ahrs_K[i][j]*error[j];  
		  }
		  
	  }
	  float  temp_ahrs_q[4];
	  temp_ahrs_q[0] = state->ahrs_q0;
	  temp_ahrs_q[1] = state->ahrs_q1;
	  temp_ahrs_q[2] = state->ahrs_q2;
	  temp_ahrs_q[3] = state->ahrs_q3;

	  state->ahrs_q0 = state->ahrs_q0 + K_times_error[0];
      state->ahrs_q1 = state->ahrs_q1 + K_times_error[1];
      state->ahrs_q2 = state->ahrs_q2 + K_times_error[2];
      state->ahrs_q3 = state->ahrs_q3 + K_times_error[3];

	  if (!((Numbers::isValid(state->ahrs_q0))&&(Numbers::isValid(state->ahrs_q1))&&(Numbers::isValid(state->ahrs_q2))&&(Numbers::isValid(state->ahrs_q3))))
	  {
		state->ahrs_q0 = temp_ahrs_q[0];
		state->ahrs_q1 = temp_ahrs_q[1];
		state->ahrs_q2 = temp_ahrs_q[2];
		state->ahrs_q3 = temp_ahrs_q[3];
		memset(state->ahrs_Pinit,0,sizeof(state->ahrs_Pinit));
		for (int i=0; i<4; i++)
		{
			state->ahrs_Pinit[i][i] = 1;
		}
		memset(state->ahrs_Pfin,0,sizeof(state->ahrs_Pfin));
	  }

	  AHRS_NORM_QUAT(state);
  }//thanhnt: do nothing if the matrix Pfin is not possitive definite.
}
//thanhnt: inverse of 3-order matrix
void Ahrs::invMatrix_3(struct ahrs_stateT* state, double m[3][3]) const
{
	
	double d = m[0][0]*m[1][1]*m[2][2] + m[0][1]*m[1][2]*m[2][0] + m[0][2]*m[1][0]*m[2][1]
  - m[0][2]*m[1][1]*m[2][0] - m[0][1]*m[1][0]*m[2][2] - m[0][0]*m[1][2]*m[2][1];
	
  if ((d<0.001f)&&(d>-0.001f))
  {
	  memset(state->invPyy,0,sizeof(state->invPyy));
  }
  else  
  {
	state->invPyy[0][0] = (m[1][1]*m[2][2] - m[1][2]*m[2][1])/d;
	state->invPyy[1][0] = - (m[1][0]*m[2][2] - m[2][0]*m[1][2])/d;
	state->invPyy[2][0] = (m[1][0]*m[2][1] - m[2][0]*m[1][1])/d;
	state->invPyy[0][1] = - (m[0][1]*m[2][2] - m[2][1]*m[0][2])/d;
	state->invPyy[1][1] = (m[0][0]*m[2][2] - m[2][0]*m[0][2])/d;
	state->invPyy[2][1] = - (m[0][0]*m[2][1] - m[2][0]*m[0][1])/d;
	state->invPyy[0][2] = (m[0][1]*m[1][2] - m[1][1]*m[0][2])/d;
	state->invPyy[1][2] = - (m[0][0]*m[1][2] - m[1][0]*m[0][2])/d;
	state->invPyy[2][2] = (m[0][0]*m[1][1] - m[1][0]*m[0][1])/d;
  }

}
//thanhnt: algorithm cholesky decomposition
bool Ahrs::Cholesky_Decomposition(struct ahrs_stateT *state) const
{
	double d3;
	double d4;
	d3 = det_3(state->ahrs_Pfin);
	d4 = det_4(state->ahrs_Pfin);
	if ((state->ahrs_Pfin[0][0]>0)&&(state->ahrs_Pfin[0][0]*state->ahrs_Pfin[1][1]-state->ahrs_Pfin[0][1]*state->ahrs_Pfin[1][0])&&(d3>0)&&(d4>0))
	{
	    memset(state->chol_lower,0,sizeof(state->chol_lower));
    // Decomposing a matrix into Lower Triangular 
		for (int i = 0; i < 4; i++) 
		{ 
			for (int j = 0; j <= i; j++) 
			{ 
				double sum = 0; 
	
				if (j == i) // summation for diagnols 
				{ 
					for (int k = 0; k < j; k++) 
						sum += state->chol_lower[j][k]* state->chol_lower[j][k]; 
					if (state->ahrs_Pfin[j][j]-sum > 0.0f)
					{
						state->chol_lower[j][j] = sqrtf(state->ahrs_Pfin[j][j]-sum); 
					}
					else
					{
						return false;
					}
				} 
				else 
				{ 
					// Evaluating L(i, j) using L(j, j) 
					for (int k = 0; k < j; k++) 
						sum += (state->chol_lower[i][k] * state->chol_lower[j][k]); 
					if (abs(state->chol_lower[j][j]) < 0.01f)
					{
						return false;
					}
					else
					{
						state->chol_lower[i][j] = (state->ahrs_Pfin[i][j] - sum) / state->chol_lower[j][j]; 
					}
				} 
			} 
		} 
		return true;
	}
	else
	{
		return false;
	}
} 
//thanhnt: determinant of 3-order matrix
double Ahrs::det_3(double m[4][4]) const
{  
  double a;
  a = m[0][0]*m[1][1]*m[2][2] + m[0][1]*m[1][2]*m[2][0] + m[0][2]*m[1][0]*m[2][1]
  - m[0][2]*m[1][1]*m[2][0] - m[0][1]*m[1][0]*m[2][2] - m[0][0]*m[1][2]*m[2][1]; 
  
  return a;
}
//thanhnt: determinant of 4-order matrix
double Ahrs::det_4(double m[4][4]) const
{  
  double m00[4][4];
  double m10[4][4];
  double m20[4][4];
  double m30[4][4];
  // calculate m00
  for (int i=0;i<3;i++)
  {
	  for (int j=0;j<3;j++)
		  m00[i][j] = m[i+1][j+1];
  }
  // calculate m10
  for (int i=0;i<3;i++)
  {  
	  m10[0][i] = m[0][i+1];
	  if (i>0)
	  {
	  for (int j=0;j<3;j++)
		  m10[i][j] = m[i+1][j+1];
	  }
  }
  //calculate m20
  for (int i=0;i<3;i++)
  {  
	  m20[0][i] = m[0][i+1];
	  m20[1][i] = m[1][i+1];
	  if (i>1)
	  {
	  for (int j=0;j<3;j++)
		  m20[i][j] = m[i+1][j+1];
	  }
  }
  //calculate m30
  for (int i=0;i<3;i++)
  {  
	  for (int j=0;j<3;j++)
		  m30[i][j] = m[i][j+1];
  }

  double a;
  a = m[0][0] * det_3(m00) - m[1][0] * det_3(m10) + m[2][0] * det_3(m20) - m[3][0] * det_3(m30);
  return a;
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
  float Rnoise_phi[3][3] = {0};
  Rnoise_phi[0][0] = AHRS_R_PHI;
  float error_phi[3] = {0};
  error_phi[0] = err_phi;
  run_kalman( state,Rnoise_phi, error_phi, active );
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
			  
			  float error_theta[3] = {0};
              error_theta[1] = err_theta;
              
              if(abs(incd)>0.04f)
              {
				  float Rnoise_theta[3][3] = {0};
                  Rnoise_theta[1][1] = 6.25f;
                  run_kalman( state, Rnoise_theta , error_theta, active );
              }else{
				  float Rnoise_theta[3][3] = {0};
                  Rnoise_theta[1][1] = 5.5f*5.5f;
                  run_kalman( state, Rnoise_theta , error_theta, active );
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
  float Rnoise_theta[3][3] = {0};
  Rnoise_theta[1][1] = AHRS_R_THETA;
  float error_theta[3] = {0};
  error_theta[1] = err_theta;
  run_kalman( state, Rnoise_theta ,error_theta , active );
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
			  
              float error_phi[3] = {0};
              error_phi[0] = err_phi;
              if(abs(incd)>0.04f)
              {
				  float Rnoise_phi[3][3] = {0};
                  Rnoise_phi[0][0] = 2.5f*2.5f;
                  run_kalman( state, Rnoise_phi , error_phi, active );
              }else{
				  float Rnoise_phi[3][3] = {0};
                  Rnoise_phi[0][0] = 5.5f*5.5f;
                  run_kalman( state, Rnoise_phi, error_phi, active );
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

  float I = Pi_2minusInclination;
  float D = Declination;
  float incd = 0.0f;

  AHRS_COMPUTE_H_PSI(state);

  if (!BMAG_UPDATE)
  {
      mag_psi = ahrs_psi_of_mag(state, mag);
      if(active != 0)
          err_psi = mag_psi - state->ahrs_psi;

      AHRS_WARP( &err_psi, PI);
	  float Rnoise_psi[3][3] = {0};
      Rnoise_psi[2][2] = AHRS_R_PSI;
      float error_psi[3] = {0};
      error_psi[2] = err_psi;
      run_kalman( state, Rnoise_psi, error_psi, active );
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
	  float Rnoise_psi[3][3] = {0};
      Rnoise_psi[2][2] = AHRS_R_PSI;
      float error_psi[3] = {0};
      error_psi[2] = err_psi;
      run_kalman( state, Rnoise_psi, error_psi, active );
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
  if(fabsf(dcm02) < 1.0f)
  {
	  euler[1] = -asinf( dcm02 );
  }
  else
  {
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
  }
  else
  {
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


	//thanhnt : initializing Pfin and Pinit.
	memset(state->ahrs_Pfin,0,sizeof(state->ahrs_Pfin));
	memset(state->ahrs_Pinit,0,sizeof(state->ahrs_Pinit));
	for (int i=0;i<4;i++)
	{
		state->ahrs_Pinit[i][i] = 1.0f;
	}


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

#if (PILOT_TYPE == VUA_SC_6G) && (USE_LASER == ENABLED)
	gAhrsData.f32EstAltLaser = 0.0f;
	gAhrsData.f32EstDeltaAltLaser = 0.0f;
	gAhrsData.f32EstVertSpeedLaser = 0.0f;
#endif

#if (PILOT_TYPE == VUA_SC_6G)
	gAhrsData.f32EstEastPosition    = 0.0f;
	gAhrsData.f32EstEastSpeed       = 0.0f;
	gAhrsData.f32EstNorthPosition   = 0.0f;
	gAhrsData.f32EstNorthSpeed      = 0.0f;
	gAhrsData.f32RawEastPosition    = 0.0f;
	gAhrsData.f32RawNorthPosition   = 0.0f;
	gAhrsData.f32RawNorthAcc        = 0.0f;
	gAhrsData.f32RawEastAcc         = 0.0f;
	gAhrsData.originUpdate          = false;
	gAhrsData.deltaNorth            = 0.0f;
	gAhrsData.deltaEast             = 0.0f;
#endif

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


#if ((PILOT_TYPE == VUA_SC_6G) && (USE_LASER == ENABLED))
	state->vertSpeedKFLaser.f32Distance = 0.0f;
	state->vertSpeedKFLaser.f32BiasAcc  = 0.0f;
	state->vertSpeedKFLaser.f32Speed    = 0.0f;
#endif

#if (PILOT_TYPE == VUA_SC_6G)
	state->northSpeedKF.f32Distance = 0.0f;
	state->northSpeedKF.f32Speed    = 0.0f;
	state->northSpeedKF.f32BiasAcc  = 0.0f;

	state->eastSpeedKF.f32Distance  = 0.0f;
	state->eastSpeedKF.f32Speed     = 0.0f;
	state->eastSpeedKF.f32BiasAcc   = 0.0f;
#endif
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
void Ahrs::ahrsCompute(struct ahrs_stateT* ahrsState, const float* gyro, float* accel, float psiFromHeading , float airspeed, float *mag, int magEnable, int flightMode, int version, float groundspeed, bool gpsGood)
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

    /*
	if(!gpsGood)
	{
		if((airspeed > 20.0f))
		{
			ahrsAccelerometerModify(ahrsState, gyro, accel, airspeed);
		}
	}
	else
	{
		if(airspeed > 40.0f)
		{
			ahrsAccelerometerModify(ahrsState, gyro, accel, airspeed);
		}
		else
		{
			ahrsQuadAccelModify(ahrsState, gyro, accel, ahrsState->velocityXHistory[0], ahrsState->velocityYHistory[0]);
		}
	}
	*/

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
#if PILOT_TYPE == VUA_SC_6G
			&&(abs(ahrsState->ahrs_q)<0.1f)&&(abs(ahrsState->ahrs_r)<0.1f)&&(abs(accX)<2.0f))
        		||
				((airspeed<20.0f) && (lockUpdateCounter<=0)))
#else

            &&(abs(ahrsState->ahrs_q)<0.1f)&&(abs(ahrsState->ahrs_r)<0.1f)&& (abs(accX)<2.0f)&&(lockUpdateCounter<=0))||((airspeed<20.0f)&&(lockUpdateCounter<=0)))
#endif
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
#if PILOT_TYPE == VUA_SC_6G
				   (abs(gyro[1])<0.02f)
#else
				   (abs(gyro[1])<0.05f) &&
				   (lockUpdateCounter<=0)
#endif
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

void Ahrs::ahrsMain(const struct Gauge::measurementT* measurement, const struct Gauge::magnetMeasurementT* magnetMeasurement, float airspeed, unsigned int time, int pressureSource, int magEnable,int flightMode, int version, int debugMsg, float groundSpeed, float track_deg, bool gpsGood)
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
        ahrsState->dt = 1.0f/AHRS_DT;
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
        ahrsCompute(ahrsState, gyro, accel, psiFromHeading, airspeed, mag, magEnable, flightMode, version, groundSpeed, gpsGood);
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

	//thanhnt: initializing Pfin and Pinit.
	memset(state->ahrs_Pfin,0,sizeof(state->ahrs_Pfin));
	memset(state->ahrs_Pinit,0,sizeof(state->ahrs_Pinit));
	for (int i=0;i<4;i++)
	{
		//state->ahrs_Pfin[i][i] = 0.0f;
		state->ahrs_Pinit[i][i] = 1.0f;
	}



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

#if (PILOT_TYPE == VUA_SC_6G)
	gAhrsData.f32EstEastPosition    = 0.0f;
	gAhrsData.f32EstEastSpeed       = 0.0f;
	gAhrsData.f32EstNorthPosition   = 0.0f;
	gAhrsData.f32EstNorthSpeed      = 0.0f;
	gAhrsData.f32RawEastPosition    = 0.0f;
	gAhrsData.f32RawNorthPosition   = 0.0f;
	gAhrsData.f32RawEastAcc         = 0.0f;
	gAhrsData.f32RawNorthAcc        = 0.0f;
	gAhrsData.originUpdate          = false;
	gAhrsData.deltaNorth            = 0.0f;
	gAhrsData.deltaEast             = 0.0f;

	state->northSpeedKF.f32Distance = 0.0f;
	state->northSpeedKF.f32Speed    = 0.0f;
	state->northSpeedKF.f32BiasAcc  = 0.0f;

	state->eastSpeedKF.f32Distance  = 0.0f;
	state->eastSpeedKF.f32Speed     = 0.0f;
	state->eastSpeedKF.f32BiasAcc   = 0.0f;
#endif

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

void Ahrs::ahrsMainFG(const struct Gauge::measurementT* measurement, float airspeed, unsigned int time,  float phi, float theta, float psi, int version, float groundSpeed, float track_deg, bool gpsGood)
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
	if(deltaTime>(10.0f/AHRS_DT))
	{
		ahrsState->ahrsInitialized = 0;
	}

	ahrsState->lastTime = actTime;
	ahrsState->dt_1 = ahrsState->dt;
	if(deltaTime == 0.0f)
    {
        ahrsState->dt = 1.0f/AHRS_DT;
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
        ahrsCompute(ahrsState, gyro, accel, psiFromHeading, airspeed, mag, magEnable, 1, version, groundSpeed, gpsGood);
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

	float fAccZ_NED_G = fDcm1 * measurement->accXL + fDcm2 * measurement->accYL + fDcm3 * measurement->accZL;
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

#if (PILOT_TYPE == VUA_SC_6G)
void Ahrs::estimateNED (const struct Gauge::measurementT* measurement, GpsPosition currentPosition, float amsl, float gpsNorthSpeed, float gpsEastSpeed, bool update, bool simulation)
{
	float accel_NED[2];
	struct ahrs_stateT* ahrsState = &gAhrsState;

	// init
	if(update)
	{
		if(!gAhrsData.originUpdate)
		{
			gAhrsData.origin = currentPosition;
			gAhrsData.originUpdate = true;
		}
	}

	if(gAhrsData.originUpdate)
	{
		// tranfer Acceleromet from body to ned
		accelBody2NED(measurement, ahrsState, accel_NED, simulation);

		// changing GS84 to NED
		float positionNorth;
		float positionEast;
		GpsPosition::getVectorXY(gAhrsData.origin, currentPosition, positionEast, positionNorth);
        
        float groundSpeed = sqrt(gpsEastSpeed * gpsEastSpeed + gpsNorthSpeed * gpsNorthSpeed);
        // estimate north speed
        struct StateKalmanFilter* northSpeed = &ahrsState->northSpeedKF;
        // estimate east speed
        struct StateKalmanFilter* eastSpeed = &ahrsState->eastSpeedKF;
        
        estimateSpeed(gpsNorthSpeed, accel_NED[AXIS_X], ahrsState->dt, northSpeed, update);
        estimateSpeed(gpsEastSpeed, accel_NED[AXIS_Y], ahrsState->dt, eastSpeed, update);

		gAhrsData.f32EstNorthSpeed    = ahrsState->northSpeedKF.f32Speed;
		gAhrsData.f32EstEastSpeed     = ahrsState->eastSpeedKF.f32Speed;

		gAhrsData.f32RawEastPosition  = positionEast;
        gAhrsData.f32RawNorthPosition = positionNorth;
        
        if (groundSpeed > 1.2f)
        {
			positionNorth = positionNorth - gAhrsData.deltaNorth;
			positionEast  = positionEast - gAhrsData.deltaEast;
            estimatePosition(positionEast, accel_NED[AXIS_Y], ahrsState->dt, eastSpeed, update);
            estimatePosition(positionNorth, accel_NED[AXIS_X], ahrsState->dt, northSpeed, update);
        }
		else
		{
			gAhrsData.deltaNorth = positionNorth - ahrsState->northSpeedKF.f32Distance;
			gAhrsData.deltaEast  = positionEast - ahrsState->eastSpeedKF.f32Distance;
		}

		// update
		gAhrsData.f32EstNorthPosition = ahrsState->northSpeedKF.f32Distance;
		gAhrsData.f32EstEastPosition  = ahrsState->eastSpeedKF.f32Distance;

		gAhrsData.f32RawEastAcc       = accel_NED[AXIS_Y];
		gAhrsData.f32RawNorthAcc      = accel_NED[AXIS_X];
	}
}
#endif

#if (PILOT_TYPE == VUA_SC_6G) && (USE_LASER == ENABLED)
void Ahrs::estimateVertLaser(const struct Gauge::measurementT* measurement, float laserAlt)
{
	struct ahrs_stateT* ahrsState = &gAhrsState;

	//< dongnt11 for Estimating the climb rate by integrating the AccZ in NED
	AHRS_DCM_OF_QUAT(ahrsState);
	float fDcm1 = -ahrsState->ahrs_dcm02 * ahrsState->ahrs_dcm22;
	float fDcm2 = -ahrsState->ahrs_dcm12;
	float fDcm3 =  ahrsState->ahrs_dcm22;

	float fAccZ_NED_G = fDcm1 * measurement->accXL + fDcm2 * measurement->accYL + fDcm3 * measurement->accZL;
	float fAccZ_NED = -G * fAccZ_NED_G - G;

	// integratation
	float fTsample_2 = ahrsState->dt * ahrsState->dt/2;
	float fAlt = ahrsState->vertSpeedKFLaser.f32Distance + ahrsState->dt * ahrsState->vertSpeedKFLaser.f32Speed - fTsample_2 * ahrsState->vertSpeedKFLaser.f32BiasAcc + fTsample_2 * fAccZ_NED;
	float fClimbRate = ahrsState->vertSpeedKFLaser.f32Speed - ahrsState->dt * ahrsState->vertSpeedKFLaser.f32BiasAcc + ahrsState->dt * fAccZ_NED;
	float fBiasAccZ = ahrsState->vertSpeedKFLaser.f32BiasAcc;
	
	// Correction
	float fAglErr = laserAlt - fAlt;
	ahrsState->vertSpeedKFLaser.f32Distance  = fAlt + KF_CR_ALT * fAglErr;
	ahrsState->vertSpeedKFLaser.f32Speed     = fClimbRate + KF_CR_CR * fAglErr;
	ahrsState->vertSpeedKFLaser.f32BiasAcc   = fBiasAccZ + KF_CR_BIAS * fAglErr;

	// Update data
	gAhrsData.f32EstAltLaser = ahrsState->vertSpeedKFLaser.f32Distance;
	gAhrsData.f32EstDeltaAltLaser = ahrsState->vertSpeedKFLaser.f32Speed * ahrsState->dt;
	gAhrsData.f32EstVertSpeedLaser = ahrsState->vertSpeedKFLaser.f32Speed;
}
#endif
