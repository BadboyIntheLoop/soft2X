/**
*                                                                   
* @class PidModifierAlr                                                    
*                                                                   
* @brief Class to modify Alr_P controller output.    		             
*                                                                   
* 2010 Roman Filipowski & Witold Kruczek @ Flytronic                
*/


#ifndef PIDMODIFIERTHROTTLE_H
#define PIDMODIFIERTHROTTLE_H

class PidModifierThrottle :	public PidModifierBase
{
public:
	PidModifierThrottle(const float& pAirspeed, const float& pTrimThrV1, 
		const float& pTrimThrV2, const float& pTrimThr1, const float& pTrimThr2);
    
	virtual bool modify (float& vmod);   ///<  internal modification of controller

private:
	const float* _airspeed;
	const float* _v1;
	const float* _v2;
	const float* _t1;
	const float* _t2;
};

#endif  // PIDMODIFIERTHROTTLE_H
