/**
*                                                                   
* @class PidModifierPhi                                                    
*                                                                   
* @brief Class modifying angle Theta calculated by the controller.
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>


void PidModifierTheta::init (const PStateData& psd, const FPRealData& fprd, const float& pTrimThetaFun,
                             const float& pTrimThetaFunNoSpd, const float& pThrottle)
{
    //  psd and fprd are not NULL because they are passed by refrence not by pointer.
    _psd = &psd;
    _fprd = &fprd;
    _pTrimThetaFun = &pTrimThetaFun;
    _pTrimThetaFunNoSpd = &pTrimThetaFunNoSpd;
	_pThrottle = &pThrottle;  // variable adrress with calculated throtle value.
}


bool PidModifierTheta::modify (float &vmod)
{
	// The controller calculates Theta_Speed ​​angle Theta in demand on the speed of the aircraft. 
	// Trimming is done by integration in the controller. Enable or disable the engine requires a change in trim, which is quite slow. 
	// This feature adds to the calculated Theta angle calculated theoretically on the basis of summing vectors engine thrust, resistance and component weight. 
	
	// Full formula:
	//      theta2 = arc sin (sin (theta1) + S/Q)
    //          S - engine thrust 
    //          Q - weight of the aircraft
	// Formula can be simplified for | theta | <0.8 (ie, our full range): 
	//      theta2 = theta1 + S/Q

    if (!FPReal->isUseThetaModifier())
        return true;

    if (!_fprd->other.doNotUseAirspeed)
        //  no speed indicator malfunction
	    vmod = vmod + *_pThrottle * *_pTrimThetaFun; 
    else
        //  Speed indicator malfunction (flight with constant Theta)
	    vmod = vmod + *_pThrottle * *_pTrimThetaFunNoSpd; 

    return true;
}
