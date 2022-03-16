/**
*                                                                   
* @class PidModifierPhi                                                    
*                                                                   
* @brief Class modifying angle Theta calculated by the controller.
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#ifndef PIDMODIFIERTHETA_H
#define PIDMODIFIERTHETA_H

class PStateData;
class FPRealData;

class PidModifierTheta: public PidModifierBase
{
public:
    PidModifierTheta (const PStateData& psd, const FPRealData& fprd, const float& pTrimThetaFun,
        const float& pTrimThetaFunNoSpd, const float& pThrottle)
    {
        init (psd, fprd, pTrimThetaFun, pTrimThetaFunNoSpd, pThrottle);
    };
    virtual bool modify (float &vmod);                           ///<  internal modification of controller

private:
    const PStateData* _psd;
    const FPRealData* _fprd;
    const float* _pTrimThetaFun;
    const float* _pTrimThetaFunNoSpd;
	const float* _pThrottle;

    void init (const PStateData& psd, const FPRealData& fprd, const float& pTrimThetaFun,
        const float& pTrimThetaFunNoSpd, const float& pThrottle);  ///< Initialization
};

#endif  // PIDMODIFIERPHI_H
