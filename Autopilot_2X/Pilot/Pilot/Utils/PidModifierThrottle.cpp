/**
*                                                                   
* @class PidModifierAlr                                                    
*                                                                   
* @brief Class to modify Alr_P controller output.    		             
*                                                                   
* 2010 Roman Filipowski & Witold Kruczek @ Flytronic                
*/

#include <PilotIncludes.h>

PidModifierThrottle::PidModifierThrottle(const float& pAirspeed, const float& pTrimThrV1, 
		const float& pTrimThrV2, const float& pTrimThrT1, const float& pTrimThrT2)
{
	_airspeed = &pAirspeed;
	_v1 = &pTrimThrV1;
	_v2 = &pTrimThrV2;
	_t1 = &pTrimThrT1;
	_t2 =  &pTrimThrT2;
}

bool PidModifierThrottle::modify(float& vmod)
{
	// Thr_Alt controller must give on the output value <0..1>
	// Therefore, to calculate the controller is added constant depending on the desire speed.
	// First version: linear: t1 = f(v1), t2 = f(v2), f(v) = a * v + b

	if (*_v1 == *_v2)
	{
		vmod += (*_t1 + *_t2) / 2.0f;  // average 
	}
	else
	{
		float a = (*_t1 - *_t2) / (*_v1 - *_v2);
		float b = *_t1 - a * *_v1;
		vmod += a * *_airspeed + b;
	}
	
	return true;
}
