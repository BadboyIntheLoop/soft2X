/**
* @class Camera.h
*
* 2009 Grzegorz Tyma @ Flytronic
*/

#ifndef CAMERA_H
#define CAMERA_H

class Camera
{
public:
	Camera(void);

    void ledOn();
    void ledOff(); 
	void heaterOn();
	void heaterOff();
	void preHeaterOn();
	void preHeaterOff();

private:
    typedef char byte;
	static const int MAG_BITS                    = 0x1A*4;

	typedef union magnetBitsT {
		struct bits {
			u16 led1On		     : 1; // bit0 = Led#1 on/off
			u16 led2On			 : 1; // bit1 = Led#2 on/off
			u16 dummy1			 : 4; // 2,3,4,5
			u16 preHeater		 : 1; // bit6 = pre heater on/off
			u16 dummy2			 : 8; // 7,8,9,10,11,12,13,14
			u16 mainHeater       : 1; // bit15 = main heater on/off
		} bits;
		u16 word;
	} magnetBitsT;
};   


#endif  // CAMERA_H
