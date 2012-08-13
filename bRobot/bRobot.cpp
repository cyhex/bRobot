// Do not remove the include below
#include "bRobot.h"

#include <Wire.h>
#include <LSM303.h>
#include <L3G4200D.h>
#include <math.h>
#include <mc33926.h>
#include <SoftwareSerial.h>

pidController::pidController(){};
float pidController::run(float setPoint, float currentPoint){
	p = setPoint * Kp;
	d = (setPoint - currentPoint) * Kd;
	i = i + (setPoint * Ki);
	return p+i+d;
}



L3G4200D gyro;
LSM303 imu;

DCMotor m1(5,3);
DCMotor m2(11,9);

SoftwareSerial blueSerial(2,4); // RX, TX

unsigned long serialOutLast = 0;
int speed = 0;

pidController pidBalance;
pidController pidMotion;

angleStruct angle;
balanceStruct balance;



class serialBluetooth {
public:
	float x;
	float y;
	float z;
	float fwMotion; // in deg
	float sideMotion;// in deg

	serialBluetooth(){
		ready = false;
		fwMotion=0;
		sideMotion=0;
	}
	bool read() {
		readLine();
		if( z > 0 ){
			// maps speed 50 = 0, 0 = 0.04
			fwMotion = 0.04 - (speed * 0.0006);
			fwMotion = 0.06;
		}else{
			fwMotion = 0;
		}
		if(z < 80 && z > 50){
			fwMotion *= -1;
		}
		return ready;
	}

private:
	char str[18];
	int i;
	char c;
	char * tok;
	bool ready;

	void parseArray(){
		tok = strtok(str, ";");
		x = atof(tok);
		tok = strtok(NULL, ";");
	    y = atof(tok);
	    tok = strtok(NULL, ";");
		z = atof(tok);
	}

	void readLine() {
		ready = false;
		if (blueSerial.available()) {
			c = blueSerial.read();
			if (c == 19) {
				parseArray();
				ready = true;
			} else if (c == 'A') {
				i = 0;
			} else {
				str[i] = c;
				i++;
			}
		}
	}

} bluetooth;





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
	pidBalance.Kd = PID_Kd;
	pidBalance.Ki = PID_Ki;
	pidBalance.Kp = PID_Kp;

	pidMotion.Kd = PID_Kd;
	pidMotion.Ki = PID_Ki;
	pidMotion.Kp = PID_Kp;

	// init angle
	angle.current=0.0;
	angle.last=0.0;
	angle.offset = 0.0;
	// init balance
	balance.a = 0;
	balance.g = 0;
	balance.lastRead = 0;
	balance.a_raw = 0;


	Serial.begin(9600);
	Serial.println("Hello");
	blueSerial.begin(9600);

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

	return false;
}


void getSpeed() {
	speed = pidBalance.run(angle.current,angle.last);
	speed = max(speed, -255);
	speed = min(speed, 255);
	if (abs(angle.current) >= ANGLE_KILL) {
		speed = 0;
	}

}

void remoteControll() {
	//bluetooth.read();

	if (abs(speed ) < 100 ) {
		//0.12
		angle.current += (100 - abs(speed)) * 0.0012;
	}

}


void loop() {
	//callibrateManual();
	readAngle();
	serialOut();
	//remoteControll();
	getSpeed();
	m1.run(speed);
	m2.run(speed);
	//delay(20);
}


