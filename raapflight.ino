#include <Wire.h>
#include <Servo.h>
#include "FlySkyIBus.h"

// ----------------- PID gains -----------------
const float roll_p_gain = 0.7;
const float roll_i_gain = 0.005;
const float roll_d_gain = 2.3;

// pitch gains are lower because of tighter weight distribution on the pitch axis
const float pitch_p_gain = 0.6;
const float pitch_i_gain = 0.005;
const float pitch_d_gain = 2.0;

const float yaw_p_gain = 3.0;
const float yaw_i_gain = 0.02;


// ----------------- RC rates -----------------
const float rp_rcrate = 1.4;  // simple mutliplier after aplying expo
const float yaw_rcrate = 1.2;


// ----------------- PID loop settings -----------------
const int looptime = 4000; // looptime in microseconds; 4000ms -> 250Hz update rate
const int pid_output_limit = 400; // limit the pid controller output
const int motor_idle = 1100; // idle motor value in us
const float gyro_credibility = 0.2; // lower values lead to more filtering and dampened response; increase for sharper and more direct response, but possibly more noise entering the PID controller (between 0 and 1)


// ----------------- HW settings -----------------
const int MPU_addr = 0x68; // I2C address of the MPU-6050


// ----------------- inits -----------------
unsigned long loop_start; // var to keep track of the current looptime
boolean armed = false;

float gyro_roll, gyro_pitch, gyro_yaw;
int throttle;
float roll_setpoint, pitch_setpoint, yaw_setpoint;
float pid_output_roll, pid_output_pitch, pid_output_yaw;

// init motor outputs with the Servo lib
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;


// ----------------- setup -----------------
void setup() {
  // begin serial comm for debugging. Serial1 rx is used by receiver, tx can still be used
  Serial.begin(115200);

  // iBus receiver setup on Serial1 rx
  IBus.begin(Serial);

  // gyro setup
  setupGyro();
  calibrateGyro();

  // ESC setup
  ESC1.attach(3);
  ESC2.attach(5);
  ESC3.attach(6);
  ESC4.attach(9);
}


// ----------------- loop -----------------
void loop() {
  loop_start = micros();
  readIbus();
  if (armed) {
    // armed loop
    readGyro();
    calculatePID();
    mixMotors();
  } else {
    // output sub 1000us pulse to completely turn off motors
    ESC1.writeMicroseconds(900);
    ESC2.writeMicroseconds(900);
    ESC3.writeMicroseconds(900);
    ESC4.writeMicroseconds(900);
  }

  // do nothing until the specified looptime is reached to keep the looptime steady
  while (micros() - loop_start < looptime);
}
