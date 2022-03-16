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

	// za��czenie grza�ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.mainHeater = 1;
}

void Camera::heaterOff()
{
	magnetBitsT magbits;

	// za��czenie grza�ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.mainHeater = 0;
}

void Camera::preHeaterOn()
{
	magnetBitsT magbits;

	// za��czenie grza�ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.preHeater = 1;
}

void Camera::preHeaterOff()
{
	magnetBitsT magbits;

	// za��czenie grza�ki (bit MAIN)
	magbits.word = 0;
	magbits.bits.preHeater = 0;
}

void Camera::ledOn()
{	
	magnetBitsT magbits;
	
	// za��czenie diody prawego skrzyd�a (przez p�ytk� magnetometru) 
	magbits.word = 0;
	magbits.bits.led2On = 1;
}

void Camera::ledOff()
{
	magnetBitsT magbits;

	// wy��czenie diody prawego skrzyd�a (przez p�ytk� magnetometru) 
	magbits.word = 0;
	magbits.bits.led2On = 0;
}
