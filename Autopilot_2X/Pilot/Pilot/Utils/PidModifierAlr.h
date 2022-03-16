/**
*                                                                  
* @class PidModifierAlr                                                    
*                                                                  
* @brief Class that modify output of the Alr_P controller.		             
*                                                                   
* 2010 Roman Filipowski & Witold Kruczek @ Flytronic                
*/

#ifndef PIDMODIFIERALR_H
#define PIDMODIFIERALR_H

class PStateData;
class FPRealData;

class PidModifierAlr: public PidModifierBase
{
public:
    PidModifierAlr (const PStateData& psd, const FPRealData& fprd,
        const float& deadzone, const float& redMin, const float& red1Phi);
    virtual bool modify (float& vmod);   // modifications inside of controller

private:
    const PStateData* _psd;
    const FPRealData* _fprd;
    const float* _pDeadZone;
    const float* _pRedMin;
    const float* _pRed1Phi;
};

#endif  // PIDMODIFIERALR_H
