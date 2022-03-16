/**
* @class Camera.h
*
* 2009 Grzegorz Tyma @ Flytronic
*/

#include <PilotIncludes.h>

Camera::Camera(void) { }

void Camera::heaterOn()
{
	magnetBitsT magbits;

	// za³¹czenie grza³ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.mainHeater = 1;
}

void Camera::heaterOff()
{
	magnetBitsT magbits;

	// za³¹czenie grza³ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.mainHeater = 0;
}

void Camera::preHeaterOn()
{
	magnetBitsT magbits;

	// za³¹czenie grza³ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.preHeater = 1;
}

void Camera::preHeaterOff()
{
	magnetBitsT magbits;

	// za³¹czenie grza³ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.preHeater = 0;
}

void Camera::ledOn()
{	
	magnetBitsT magbits;
	
	// za³¹czenie diody prawego skrzyd³a (przez p³ytkê magnetometru) 
	magbits.word = 0;
	magbits.bits.led2On = 1;
}

void Camera::ledOff()
{
	magnetBitsT magbits;

	// wy³¹czenie diody prawego skrzyd³a (przez p³ytkê magnetometru) 
	magbits.word = 0;
	magbits.bits.led2On = 0;
}
