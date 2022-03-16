/**
*                                                                   
* @class PidModifierPhi                                                    
*                                                                   
* @brief Modify Phi angle calculated by the controller.
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#include <PilotIncludes.h>


void PidModifierPhi::init (const PStateData& psd, const FPRealData& fprd, const float& pZeroPhiThrottle,
                           const float& pThrottle)
{
    //  psd and fprd are not NULL because they are passed by refrence not by pointer.
    _psd = &psd;
    _fprd = &fprd;
    _pZeroPhiThrottle = &pZeroPhiThrottle;
    _wasError = false;
	_pThrottle = &pThrottle;
}


bool PidModifierPhi::modify (float &vmod)
{
    //  Assume that Phi_Track and Phi_CTrack controllers calculates internal angular velocity PsiDot.
    //  That speed must be recalculated to desire Phi using formula for centripetal force
    
   
    //  factor that allows the behavior of the controller gain before the change in the formula
    static const float EXP1 = KPH_2_MS * 70.0f / (G * 1.7f);
    static const float MAX_K = 2.0f;

    //  In case of speed indicator malfunction, set speed to constant value.
    float v = (_fprd->other.doNotUseAirspeed ? 70.0f : _psd->airspeed);
    float k = _psd->groundspeed / v;

    //  speed factor limit
    //  K>0 during flight with wind. Limitations in normal conditions could happen very rare, but can be important when the speed measurement failure
    if (k > MAX_K)
        k = MAX_K;

    float vmod2 = atanf (vmod * EXP1 * k);

    //  Reset roll to protect against:
    //      1) torque from the engine to be enabled when the plane is in the turn,
    //      2) track error during strong steep ascenting
    if (*_pThrottle > *_pZeroPhiThrottle)
        vmod2 = 0.0f;

    //  invalid number protection
    //  In case of an error leave previous vmod value

    if (Numbers::assure (vmod2, 0.0f))
        vmod = vmod2;
    else if (!_wasError)
    {
        // Error notification
        _wasError = true;
        Log.errorPrintf("PidModifierPhi_modify_1");
    }

    return true;
}
