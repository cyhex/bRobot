// Do not remove the include below
#include "bRobot.h"

#include <Wire.h>
#include <LSM303.h>
#include <L3G4200D.h>
#include <math.h>
#include <mc33926.h>

L3G4200D gyro;
LSM303 imu;

DCMotor m1(5,3);
DCMotor m2(11,9);

unsigned long serialOutLast = 0;
int speed = 0;
int ledsPins[4];

pidStruct speedPid;
angleStruct angle;
balanceStruct balance;

void callibrateManual(){
	double gr = 0;
	double acc = 0;
	while(1){
		imu.read();
		gyro.read();

		gr = lowpassFilter(gr,gyro.g.y, 0.2);
		acc = lowpassFilter(acc,imu.a.z,0.2);
		Serial.print("gyro: ");
		Serial.println(gr );
		Serial.print("acc: ");
		Serial.println(acc);
		delay(100);
	}

}


void ledAngle(){
	int idicator = abs(angle.current) * 255;
	for (int i = 0; i < (int) sizeof(ledsPins); i++) {
		if (idicator > i * 255) {
			digitalWrite(ledsPins[i], 1);
		} else {
			digitalWrite(ledsPins[i], 0);
		}
	}
}
void readAngle(){
	angle.last = angle.current;
	imu.read();
	gyro.read();
	float dt = (millis() - balance.lastRead)/1000.0;

	balance.g = (gyro.g.y-L3G4200D_Gain_Offset) * L3G4200D_Gain * dt;

	balance.a_raw = lowpassFilter(balance.a_raw, imu.a.z, ACC_FILTER );
	balance.a = ToDeg(asin((balance.a_raw+LSM303_Gain_Offset) * LSM303_Gain));

	angle.current = (1-ACC_GYRO_RATIO) * (angle.current + balance.g) + (ACC_GYRO_RATIO)*(balance.a);

	balance.lastRead = millis();
}

void setup() {
	// init pid
	speedPid.p = 0.0;
	speedPid.i = 0.0;
	speedPid.d = 0.0;
	speedPid.Kp = 0.0;
	speedPid.Kd = 0.0;
	speedPid.Ki = 0.0;

	// init angle
	angle.current=0.0;
	angle.last=0.0;

	// init balance
	balance.a = 0;
	balance.g = 0;
	balance.lastRead = 0;
	balance.a_raw = 0;


	//Serial.begin(9600);
	//Serial.println("Hello");
	Wire.begin();

	gyro.enableDefault();
	gyro.writeReg(L3G4200D_CTRL_REG4, L3G4200D_SENSETIVITY); // set 250 dps
	imu.init();
	imu.enableDefault();
	imu.writeAccReg(LSM303_CTRL_REG4_A, LSM303_SCALE); // 2 g full scale

	int cycle = 10;
	while (cycle > 0) {
		readAngle();
		delay(100);
		cycle--;
	}
}

bool serialOut() {
	if(millis() - serialOutLast <= 100){
		return false;
	}
	serialOutLast = millis();
	Serial.print("plot");
	Serial.print(":");
	Serial.print(balance.g);
	Serial.print(":");
	Serial.print(balance.a);
	Serial.print(":");
	Serial.print(angle.current);
	Serial.print(":");
	Serial.print(speed); // torque
	Serial.println("");

	Serial.print("PID");
	Serial.print(" Ki: ");
	Serial.print(speedPid.Ki);
	Serial.print(" Kp: ");
	Serial.print(speedPid.Kp);
	Serial.print(" Kd:");
	Serial.print(speedPid.Kd);
	Serial.println("");

	return false;
}


void pid() {
	speedPid.Ki = PID_Ki;
	speedPid.Kp = PID_Kp;
	speedPid.Kd = PID_Kd;

	speedPid.p = angle.current * speedPid.Kp;
	speedPid.d = (angle.current - angle.last) * speedPid.Kd;
	speedPid.i = speedPid.i + (angle.current * speedPid.Ki);

	float pid =  speedPid.p + speedPid.d + speedPid.i;
	speed = pid;

	speed = max(speed, -255);
	speed = min(speed, 255);
	if (abs(angle.current) >= ANGLE_KILL) {
		speed = 0;
	}



}

void loop() {
	//callibrateManual();
	readAngle();
	//serialOut();
	//ledAngle();

	pid();
	m1.run(speed);
	m2.run(speed);
	//delay(20);
}


