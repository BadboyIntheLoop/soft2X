/**
*                                                                   
* @class PidModifierBase                                                   
*                                                                   
* @brief Base class for classes that modify controllers PID output.
* Child object could be passed do the controller where after output value calculations (before limitations min max) is is call "modify" function of this object       
*                                                                   
* 2009 Witold Kruczek @ Flytronic                                   
*/

#ifndef PIDMODIFIERBASE_H
#define PIDMODIFIERBASE_H

class PidModifierBase
{
public:
    virtual bool modify (float &vmod) = 0;  //  Internal controllers values modification

protected:
    // Destructor lock - object could not be destroyed
    virtual ~PidModifierBase(void){}; 
};

#endif  // PIDMODIFIERBASE_H
