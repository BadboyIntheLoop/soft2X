/**
*                                                                   
* @class FPRealTrigger                                                     
*                                                                  
* @brief Class describes conditions that could appear asynchronously in any time during the flight.                                
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                  
*/

#ifndef FPREALTRIGGER_H
#define FPREALTRIGGER_H

class PStateData;

class FPRealTrigger
{
public:
    FPRealTrigger (void);

    void reset (void);
    bool checkTrigger (const PStateData& psd);

    void setAccX (float accX);
    void setGSpeed (float gSpeed);

private:
    static const int MAX_CYCLE_INTERVAL_100 = 10000; ///< Maximum time interval between the time cycles upper which data are ignored [seconds]

    bool        _bEnabled;          ///< There are turned on conditions.

    bool        _bAccX;
    float       _accX;
    float       _lastAccX;

    bool        _bGSpeed;
    float       _gSpeed;
};

#endif  // FPREALTRIGGER_H
