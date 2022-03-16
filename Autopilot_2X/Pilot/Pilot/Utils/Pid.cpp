/**
*                                                                   
* @class Pid									                             
*                                                                   
* @brief Class describing PID controller.
*
* Each of the controller could work in two modes:
* 
* --- NORMAL_PID_MODE---
*
* Normal mode of the controller:
* of transmittance: 
*
*            G(s)=K[1 + 1/(Ti*s) + Td*s])
*
* with extensions:
*   
*    1. "Clean" transmittance part D (K*Td*s) was replaced with:
*        D(s) = K*Td*s/(1 + s*Td/N)]  (usually N=8-20, default N=10)
*		
*	 2. "Back Calculation" - protection against saturation integrator
*      (anti windup).  Constant Tt determines the integrator reset time;
*	    when Tt=0 =>antiwindup turned off. 
*      Should be fulfilled relation:
*         Td < Tt < Ti, typowo: Tt=sqrt(Td*Ti)
*		Modification: exclusion of part D of the updates of the integral
*
*   3. "Setpoint Weighting" - to decide whether we are operating on errors or on the same values​​. 
*		This applies regardless of the proportional component and the differential (Integral part always an error):
*
*          eP = wP*Vref - wX*V,  eD = wD*Vref - V
*
*       Typically assumed that wP=1 (although sometimes wP<1) and wD=0.
*		Often wD = 1 for regulators control servos. Setpoint Weighing is off when wP = wD = 1; 
*		wX setting to 0 allows you to override the controller (default wX = 1)
*
*	4 For controllers servo control is modified gain 
*		 Proportional to the square of the velocity (the ratio is given as a parameter):
*           K = Kp * Kas, (Kas is from upper limited)
*   5. "Slew rate" to reference value (maximum change of reference value / 1 sek.)
*   6. Limitation the value of the integral Imin, Imax (note: the value of 0.0f means no limit)   
* Final formula:
*
*    P(k)   = K * eP(k)					
*    I(k+1) = I(k) + K*Ts/Ti*e(k) + Ts/Tt*(u(k)-(v(k)-D(k)))      // e = Vref - V
*    D(k)   = Td * [D(k-1) + N*K*(eD(k)-eD(k-1))] / (Ts*N + Td);
*
*    v = P + I + D
*    u = sat(v)   // cutoff (saturation)
*
* The values ​​saved in the following steps are: D(k-1), eD(k-1), I(k), Vref(k-1)
*
* --- TWO_STATE_PID_MODE ---
*
* bistable mode with hysteresis, with a possible inverse min-max 
*
*/

#include <PilotIncludes.h>

//  Static variables initialization
const float Pid::MIN_TS = 0.001f;  ///< The minimum value of the sampling time
const float Pid::MAX_TS = 2.0f;    ///< The maximum value of the sampling time
const float PidParams::DEFAULT_MAX_KAS = 4.0f; ///< The maximum factor from airspeed 

 /**
 * Reset controller
 */
void Pid::clearState()
{
	state->D_1 = state->eD_1 = state->I = state->ref_1 = 0.0f; // dla regulatorów PID
	state->output_1   = cp->minValue; // 
	state->output_1m  = cp->minValue; // dla regulatorów 2-stanowych
	state->computed = false;
	state->timeout = false;
}

/**
* Calculate controller stearing signal
* 
* Parameters:
* /param time  - time of current sample
* /param input - input value
* /param output - output value
* /param reference - desire value
* /param Kas - correction factor from speed
*
* Return:
*   false - output value not calculated (controller is turned off)
*   true -  output value is calculated
*/

bool Pid::compute(int time100, float input, float reference, float &output, float Kas)
{
	float	u;	   		// calculated output value after cutoff
	float	v;			// calculated output value before cutoff
	float   eP;			// weighted error for proportional part: eP=wP*Vref-V
	float	eD;			// weighted error for derivative part: eD=wD*Vref-V
	float	Ts;			// sample time [s]
	PidBank* bank=0;
	float   K;			// gain factor adjusted to the speed
	float	P, I, D;	// calculates P, I, D parts
	float	umin, umax;	// auxiliary values ​​at reducing output	
	float	sRefTs;		// auxiliary variable (sRef * Ts)
	float	sOutTs;		// auxiliary variable (sOut * Ts)
	float   Ds;			// averaged value D from last two samples

	state->computed = false;

	// if PID is turned off
	if (!cp->enable) 
	{ 
		state->enabled = false;
		return false; 
	}

	// if PID is being turned on also during switching to CIC
	if (!state->enabled) 
	{ 
		state->enabled = true;
		state->time100_1 = time100; 
		clearState();
	    state->output_1  = output; // starts from current output value
	    state->output_1m = output; 
        state->I = output;		  // beginning value of integral works as trimer
		state->ref_1 = input;	  // beginning reference value (for "slew-rate")
		return false;
	}
  
    //  float values controll
    if (!Numbers::assure (input, 0.0f))         Log.errorPrint("Pid_compute_4");
    if (!Numbers::assure (reference, 0.0f))     Log.errorPrint("Pid_compute_5");
    if (!Numbers::assure (state->I, 0.0f))      Log.errorPrint("Pid_compute_1");
    if (!Numbers::assure (state->D_1, 0.0f))    Log.errorPrint("Pid_compute_2");
    if (!Numbers::assure (state->eD_1, 0.0f))   Log.errorPrint("Pid_compute_3");


	Ts = static_cast<float>(time100 - state->time100_1) * 0.0001f;
	if ((time100 == 0) || (Ts < MIN_TS)) 
	{
		state->time100_1 = time100; // ignore but saving time
		return false;  
	}

    if (Ts > MAX_TS) 
	{
		if ((state->time100_1 != 0) && (!state->timeout)) 
		{
			// to long waiting time for next sample
			Log.errorPrint("Error: FlightController_Pid_1.");
			state->timeout = true;	
		}
		state->time100_1 = time100; // ignore but saving time
		return false;
	}
	state->timeout = false;

	
    bank = &param->bank[cp->bank];

	switch (mode) 
	{
		
        case NORMAL_PID:

            // consider factor from speed
			if (Kas > param->maxKas) Kas = param->maxKas;
			K = bank->Kp * Kas;

            // modify reference value
            if (_refModifier != NULL)
                if (!_refModifier->modify (reference))
                {
        			Log.errorPrint("Error: FlightController_Pid_5.");
                    return false;
                }
         

			// "Slew rate" on reference value
			sRefTs = bank->sRef * Ts;
			if (bank->sRef != 0.0f) 
			{
				if (state->ref_1 > input && reference < input || state->ref_1 < input && reference > input) 
					state->ref_1 = input;
                if ((reference - state->ref_1) > sRefTs) 
					reference = state->ref_1 + sRefTs;
				else if ((reference - state->ref_1) < -sRefTs)
					reference = state->ref_1 - sRefTs;
			}

			// part P
			eP = bank->wP * reference - bank->wX * input; 
			
			P = K * eP;

			// part I
			if (bank->Ti != 0.0f) 
			{
				I = state->I;
				// I limitations
				if (bank->Imax != 0.0f) if (I > bank->Imax) I = bank->Imax;
				if (bank->Imin != 0.0f) if (I < bank->Imin) I = bank->Imin;
			}	
			else 
			{
				I = 0.0f;
			}

			// part D
			eD = bank->wD * reference - input;
			D = (bank->Td == 0.0f)? 0.0f : bank->Td * (state->D_1 + bank->N * K * (eD - state->eD_1)) / (Ts * bank->N + bank->Td);
			
			Ds = (D + state->D_1) / 2.0f;  // średnia z 2 ostatnich próbek

			// sum P+I+D
			v = P + I + Ds;

            // modify output
            if (_outModifier != NULL)
                if (!_outModifier->modify (v))
                {
        			Log.errorPrint("Error: FlightController_Pid_2.");
                    return false;
                }


			u = v;

			// cutoff
			umin = cp->minValue;
			umax = cp->maxValue;
		
			if      (u > umax) u = umax;
			else if (u < umin) u = umin;

			output = u;

			// set telemetry data
			tlmData.P = P;
			tlmData.I = I;
			tlmData.D = Ds;

			// part I (for next measurements)
			if (bank->Ti != 0.0f) 
			{
				I += K * Ts / bank->Ti * (reference - input);

				// I limitations
				if (bank->Imax != 0.0f) if (I > bank->Imax) I = bank->Imax;
				if (bank->Imin != 0.0f) if (I < bank->Imin) I = bank->Imin;

				// anti-windup
				if ((bank->Tt != 0.0f) && (v != u)) 
				{
					I += Kas * Ts / bank->Tt * (u - (v - Ds));
				}
			}

			// "Slew rate" on output value
			sOutTs = bank->sOut * Ts;
			if (bank->sOut != 0.0f) 
			{
				if ((output - state->output_1) > sOutTs) 
					output = state->output_1 + sOutTs;
				else if ((output - state->output_1) < -sOutTs)
					output = state->output_1 - sOutTs;
			}

            // save state
			state->eD_1 = eD;
			state->D_1 = D;
			state->I = I;
			state->ref_1 = reference;
			state->output_1 = output;

			state->time100_1 = time100;

			break;

		case STATE_2H:  // twostate controllers
            
            output = state->output_1m;
			if (!cp->invMargins) 
			{
				if (input > (reference + cp->marginHigh)) 
					output = cp->minValue;
   				if (input < (reference + cp->marginLow)) 
					output = cp->maxValue;
			}
			else 
			{
				if (input > (reference + cp->marginHigh)) 
					output = cp->maxValue;
   				if (input < (reference + cp->marginLow)) 
					output = cp->minValue;
			}
            state->output_1m = output;  // NOTE: value output_1m defines previous controller putput with no slew-rate

            // "Slew rate" on output value
			sOutTs = bank->sOut * Ts;
			if (bank->sOut != 0.0f) 
			{
				if ((output - state->output_1) > sOutTs) 
					output = state->output_1 + sOutTs;
				else if ((output - state->output_1) < -sOutTs)
					output = state->output_1 - sOutTs;
			}

            state->output_1  = output;
			state->time100_1 = time100;

			break;

		case MULTI_STEP:
			Log.errorPrint("Error: FlightController_Pid_3.");
			return false;
		
		default:
			Log.errorPrint("Error: FlightController_Pid_4.");
			return false;
	}

	state->computed = true;

	// set telemetry data
	tlmData.tlm.time = time100 / 10;
	tlmData.tlm.input = input;
	tlmData.tlm.output = output;
	tlmData.tlm.reference = reference;
	
	return true;
}

/**
* Initiate controller (call in the FlightController constructor)
*/
void Pid::init(ControllerID id, FPRealData::ControllerProperties *cp0, PidState *state0, 
			   PidParams *param0, PidMode mode0, PidModifierBase *outModifier, PidModifierBase *refModifier)
{

	tlmData.tlm.id = static_cast<INT8U>(id);
	tlmData.P = tlmData.I = tlmData.D = 0.0f;

	cp	  = cp0;		// pointer to ControllerProperties (from FPRealizera)
	state = state0;		// pointer to varaibles state controller
	param = param0;		// pointer to internal controllers parameters

	// data to the external parameters of controllers (from FPRealizer)
	cp->bank = 0;
	cp->enable = false;
	cp->invMargins = false;
	cp->marginHigh = 0.0f;
	cp->marginLow = 0.0f;
	cp->minValue = 0.0f;
	cp->maxValue = 0.0f;

	// State variables
	state->enabled = false;
	clearState();   // rest of variables are reset in 'clearState' method

	// internal pcontrollers parameters
	param->SetDefault();
	mode = mode0;				// working mode

    _outModifier = outModifier;
    _refModifier = refModifier;
}


/**
* Send data to communication chanel
*/
void Pid::sendTlmData(int formatL, int formatT, bool fLog, bool fComm)
{
	char buf[LINESIZE] = "";
	int bufFormat = -1;  // define which data format are in the buffer

	if (!state->enabled) return;
	if (!state->computed) return;
	if (!param->tlmEnable && !param->logEnable) return;
	if (!fLog && !fComm) return;

	// log
	if (fLog && param->logEnable) 
	{
		switch (formatL) 
		{
			case TLM_CONTROLLER_SHORT: 
				if (!Base64::encode(&tlmData.tlm, sizeof(tlmData.tlm), buf, sizeof(buf))) return;
				bufFormat = TLM_CONTROLLER_SHORT;  
				Log.tlmPrint(TLM_CONTROLLER_SHORT, buf, true, false);
				break;
			case TLM_CONTROLLER_LONG:
				if (!Base64::encode(&tlmData, sizeof(tlmData), buf, sizeof(buf))) return;
				bufFormat = TLM_CONTROLLER_LONG;
				Log.tlmPrint(TLM_CONTROLLER_LONG, buf, true, false);
				break;
			default: return;
		}
	}
	// tlm
	if (fComm && param->tlmEnable) 
	{
		if (formatT == bufFormat) 
		{
			// if telemetry format is the same as log format data is alrready prepared
			Log.tlmPrint(static_cast <enum TLM_FORMAT>(formatT), buf, false, true);
			return;
		}
		switch (formatT) 
		{
			case TLM_CONTROLLER_SHORT: 
				if (!Base64::encode(&tlmData.tlm, sizeof(tlmData.tlm), buf, sizeof(buf))) return;
				Log.tlmPrint(TLM_CONTROLLER_SHORT, buf, false, true);
				break;
			case TLM_CONTROLLER_LONG:
				if (!Base64::encode(&tlmData, sizeof(tlmData), buf, sizeof(buf))) return;
				Log.tlmPrint(TLM_CONTROLLER_LONG, buf, false, true);
				break;
			default: return;
		}
	}
}		


PidBank::PidBank(void)
{
    setDefault();
}


void PidBank::setDefault(void)
{
    Kp  =  0.0f;
    Ti  =  0.0f;
    Tt  =  0.0f;
    Td  =  0.0f;
    N   = 10.0f;
    wP  =  1.0f;
    wD  =  1.0f;
    wX  =  1.0f;
    sRef = 0.0f;
    sOut = 0.0f;
    Imin = 0.0f;
    Imax = 0.0f;
}


PidParams::PidParams(void)
{
    SetDefault(true);
}

/**
* PidParams::SetDefault()
*/
void PidParams::SetDefault(bool fromCtor)
{
    // Calling from constructor could not initiate child objects
    if (!fromCtor)
    {
        for (int i=0; i<BANK_COUNT; i++) 
		{
            bank[i].setDefault();
        }
    }

	maxKas = DEFAULT_MAX_KAS;	// maximum factor from airspeed
	tlmEnable = false;
	logEnable = false;
}

/**
* PidParamsSet::SetDefault()
*/
void PidParamsSet::SetDefault(void)
{
	Alr_P.SetDefault(); 
	Elv_Q.SetDefault(); 
	Rdr_R.SetDefault(); 
	Rdr_Yacc.SetDefault(); 
	Thr_Speed.SetDefault(); 
	Thr_Alt.SetDefault(); 
	Thr_Alt_2State.SetDefault(); 
	Btfly_Alt_2State.SetDefault(); 
	Flp_Speed.SetDefault(); 
	Abr_GPErr.SetDefault(); 
	FAlr_Alr.SetDefault(); 
	P_Phi.SetDefault(); 
	Q_Theta.SetDefault(); 
	R_Psi.SetDefault(); 
	R_CoordExp.SetDefault(); 
	R_Track.SetDefault(); 
	Theta_Alt.SetDefault(); 
	Theta_Speed.SetDefault(); 
	Phi_Track.SetDefault(); 
	Phi_CTrack.SetDefault(); 
	Phi_Psi.SetDefault(); 
	Track_TrackCorr.SetDefault(); 
	Track_Wpt.SetDefault(); 
	TrackCorr_Cte.SetDefault();
	Btfly_GPath_2State.SetDefault();
	Theta_GPath_2State.SetDefault();
	Theta_VertSpeed.SetDefault();
	VertSpeed_Alt.SetDefault();
}
