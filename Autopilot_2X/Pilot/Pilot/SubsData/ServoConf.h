/*                                                                   
* @class ServoConf                                                         
*                                                                   
*                                                                   
* @brief Structure that describes single servomechanism.
*                                                                   
* Marcin Pczycki (c) Flytronic 2008                                 
*/

#ifndef SERVOCONF_H
#define SERVOCONF_H

struct ServoConf
{
	int		address;		///< phisical adress in I/O - NOT USED!!!
	int		trim;			///< neutral position		<0;4096> timer PWM is 12 bits
	int		minValue;		///< max deflection in minus	<0;4096> timer PWM is 12 bits
	int		maxValue;		///< max deflection in plus	<0;4096> timer PWM is 12 bits
	int		angle;			///< information variable (servo deflection z scope) - NOT USED!!!

	float	gain;			///< Internal gain

	float	coeffs_p[15];	///< Factor for positive values array (subsequent values ​​defined by ServoManager::InputNames)
	float	coeffs_n[15];	///< Factor for negative values array (subsequent values ​​defined by ServoManager::InputNames)
};

#endif // SERVOCONF_H

