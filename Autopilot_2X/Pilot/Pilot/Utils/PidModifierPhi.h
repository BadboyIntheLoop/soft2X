/**
*                                                                   
* @class PidModifierPhi                                                    
*                                                                   
* @brief Modify Phi angle calculated by the controller.           
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#ifndef PIDMODIFIERPHI_H
#define PIDMODIFIERPHI_H

class PStateData;
class FPRealData;

class PidModifierPhi: public PidModifierBase
{
public:
    PidModifierPhi (const PStateData& psd, const FPRealData& fprd, const float& pZeroPhiThrottle,
        const float& pThrottle)
    {
        init (psd, fprd, pZeroPhiThrottle, pThrottle);
    };
    virtual bool modify (float &vmod);                      ///<  internal modification of controller

private:
    const PStateData* _psd;
    const FPRealData* _fprd;
    const float* _pZeroPhiThrottle;
    bool   _wasError;
	const float* _pThrottle;

    void init (const PStateData& psd, const FPRealData& fprd, const float& pZeroPhiThrottle,
        const float& pThrottle);  ///< Initialization
};

#endif  // PIDMODIFIERPHI_H
