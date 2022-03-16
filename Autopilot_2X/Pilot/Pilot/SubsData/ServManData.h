/**
*                                                                  
* @class ServManData                                                       
*                                                                   
*                                                                   
* Public structures of the ServoManager subsystem.                      
*                                                                   
* Marcin Pczycki (c) Flytronic 2008                                 
*/

#ifndef SERVMANDATA_H
#define SERVMANDATA_H


/**
* Data shared by the ServoManager subsystem.
*/
struct ServManData
{
	INT32U  time;   // [ms]
	INT16U	PicnCic;		// 0 or 1
};


// Structure packing is necessary to provide unambiguous mapping fields on the hardware registers
// NOTE: Could not be pack(1) because of the way access to hardware registers that do not support selects when byte accessing. 
// As a result of this record older byte overwrites the previously stored byte younger!!! (eg instead value 05DC value 0505 is saved)


#pragma pack(2)

/**
* Direct access to the servos registry.
* Structure must be mapped to the appropriate base address.
*/
struct ServoRegs
{
      // SERVO Register Offsets
      volatile INT16U SERVO_CONTROL_c;			// servo control register
      volatile INT16U dummy1;
      volatile INT16U SERVO1_c;                 // servo1 data register
      volatile INT16U dummy2;
      volatile INT16U SERVO2_c;
      volatile INT16U dummy3;
      volatile INT16U SERVO3_c;
      volatile INT16U dummy4;
      volatile INT16U SERVO4_c;
      volatile INT16U dummy5;
      volatile INT16U SERVO5_c;
      volatile INT16U dummy6;
      volatile INT16U SERVO6_c;
      volatile INT16U dummy7;
      volatile INT16U SERVO7_c;
      volatile INT16U dummy8;
      volatile INT16U SERVO8_c;
      volatile INT16U dummy9;
      volatile INT16U SERVO9_c;
      volatile INT16U dummy10;
      volatile INT16U SERVO10_c;
      volatile INT16U dummy11;
      volatile INT16U SERVO11_c;
      volatile INT16U dummy12;
      volatile INT16U SERVO12_c;
      volatile INT16U dummy13;
      volatile INT16U SERVO13_c;
      volatile INT16U dummy14;
      volatile INT16U SERVO14_c;
      volatile INT16U dummy15;
      volatile INT16U SERVO15_c;
      volatile INT16U dummy16;
      volatile INT16U SERVO16_c;
      volatile INT16U dummy17;   
      volatile INT16U SERVO17_c;
      volatile INT16U dummy18;
      volatile INT16U SERVO18_c;
      volatile INT16U dummy19;     
};


/**
* Direct access to the RC IN registry.
* Structure must be mapped to the appropriate base address.
*/
struct RCinRegs
{
      // RC_IN Register Offsets
      volatile INT16U RC_IN_CONTROL_c;
      volatile INT16U dummy1;
      volatile INT16U RC_1_c;
      volatile INT16U dummy2;
      volatile INT16U RC_2_c;
      volatile INT16U dummy3;
      volatile INT16U RC_3_c;
      volatile INT16U dummy4;
      volatile INT16U RC_4_c;
      volatile INT16U dummy5;
      volatile INT16U RC_5_c;
      volatile INT16U dummy6;
      volatile INT16U RC_6_c;
      volatile INT16U dummy7;
      volatile INT16U RC_7_c;
      volatile INT16U dummy8;
      volatile INT16U RC_8_c;
      volatile INT16U dummy9;
      volatile INT16U RC_PIC_NCIC_c;
      volatile INT16U dummy10;
};

#pragma pack()


#endif // SERVMANDATA_H


