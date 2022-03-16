/**
*                                                                
* @class WindData                                                         
*                                                                   
* @brief Wind data class.
*                                                                   
* 2010 Witold Kruczek @ Flytronic                                   
*/

#ifndef WINDDATA_H
#define WINDDATA_H

class WindData
{
public:
    WindData (void): from(0.0f), speed(0.0f) {};

    float from;     ///< Direction from which the wind blows [degrees]
    float speed;    ///< Speed of the wind [kph]
};

#endif  //  WINDDATA_H
