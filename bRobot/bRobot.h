// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef bRobot_H_
#define bRobot_H_
#include "Arduino.h"
//add your includes for the project bRobot here


#define ToRad(x) ((x)*0.01745329252) // *pi/180
#define ToDeg(x) ((x)*57.2957795131) // *180/pi
#define lowpassFilter(val, change, q) ((1-q) * val + (change * q))


/**
 * L3G4200D gyro:
 *
 * FS = 250 dps ; 8.75 mdps/digit  milli degres per second
 * FS = 500 dps ; 17.50 mdps/digit
 * FS = 2000 dps ; 70 mdps/digit
 *
 */
#define L3G4200D_Gain 0.00875
#define L3G4200D_Gain_Offset 10
#define L3G4200D_SENSETIVITY 0b00000000
/**
 * LSM303 accelerometer:
 * Linear acceleration sensitivity
 *
 * 00 ; 1 mg/digit (mili garvity)
 * 01 ; 2 mg/digit
 * 11 ; 3.9 mg/digit
 *
 */
#define LSM303_SCALE 0b00000000
#define LSM303_Gain 0.001
#define LSM303_Gain_Offset 480 // in raw digits

/*

  #### DEFAULTS THAT WORK QUITE WELL ####

#define ANGLE_TORQUE_RATIO 75 // convert angle to torgque
#define MIN_TORQUE 10 // release motor from
#define ANGLE_KILL 25 // kill motor above
#define ACC_FILTER 0.05 // lowpass filter for acc
#define ACC_GYRO_RATIO 0.01 // acc to gyro ratio
*/

#define ANGLE_KILL 35 // kill motor above
#define ACC_FILTER 0.01 // lowpass filter for acc
#define ACC_GYRO_RATIO 0.005 // acc to gyro ratio


#define PID_Ki 0.39 // 0.39 analogRead(A0)
#define PID_Kp 19 // 19.8 analogRead(A1)
#define PID_Kd 44   //44  analogRead(A2)

class pidController {
public:

	float p;
	float i;
	float d;
	float Kp;
	float Kd;
	float Ki;
	pidController();

	float run(float setPoint, float currentPoint );

};

struct angleStruct{
	float current;
	float last;
	float offset;

};

struct balanceStruct{
	unsigned long lastRead; // millis
	float g; //gyro angle in deg
	float a; // accelerometer angle in deg
	float a_raw;
};




//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project bRobot here




//Do not add code below this line
#endif /* bRobot_H_ */
