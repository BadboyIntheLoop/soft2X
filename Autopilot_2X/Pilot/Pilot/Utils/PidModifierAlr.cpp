/**
*                                                                  
* @class PidModifierAlr                                                    
*                                                                  
* @brief Class that modify output of the Alr_P controller.		             
*                                                                   
* 2010 Roman Filipowski & Witold Kruczek @ Flytronic                
*/

#include <PilotIncludes.h>

PidModifierAlr::PidModifierAlr (const PStateData& psd, const FPRealData& fprd,
                                const float& deadzone, const float& redMin, const float& red1Phi):
_psd (&psd),
_fprd (&fprd),
_pDeadZone (&deadzone),
_pRedMin (&redMin),
_pRed1Phi (&red1Phi)
{
    //_oldDeadZone = *_pDeadZone;
    //_mrgH = *_pDeadZone / 2.0f;
    //_mrgL = - _mrgH;
}

bool PidModifierAlr::modify (float& vmod)
{
    //  Handling ailerons depending on Phi
    // Choosing is larger absolute value of real Phi or desire one.
    float c1 = fabsf(_psd->phi);
    float c2 = fabsf(_fprd->fRef.fRefLowLevel.phi);
    float p = c1>c2 ? c1:c2;

    float a = (1.0f - *_pRedMin) / (*_pRed1Phi);   // slope of factor gain change of Phi
    float k = (p * a) + *_pRedMin;                 // multiplier Phi output

    if (k > 1.0f)
        k = 1.0f;

    //  In case of an error output value would not been changed.
    Numbers::assure(k, 1.0f);
    vmod *= k;

    return true;
}
