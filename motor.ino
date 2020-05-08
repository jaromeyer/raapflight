// functions for mixing the motor outputs with the values calculated by the pid controller
int motor1_output, motor2_output, motor3_output, motor4_output;


// mix throttle and pid_outputs and clip the output at 1100us for the motors to never stop and at 2000us
void mixMotors() {
  motor1_output = motor_idle + throttle + pid_output_roll - pid_output_pitch - pid_output_yaw;
  if (motor1_output < motor_idle)
    motor1_output = motor_idle;
  if (motor1_output > 2000)
    motor1_output = 2000;
  motor2_output = motor_idle + throttle + pid_output_roll + pid_output_pitch + pid_output_yaw;
  if (motor2_output < motor_idle)
    motor2_output = motor_idle;
  if (motor2_output > 2000)
    motor2_output = 2000;
  motor3_output = motor_idle + throttle - pid_output_roll - pid_output_pitch + pid_output_yaw;
  if (motor3_output < motor_idle)
    motor3_output = motor_idle;
  if (motor3_output > 2000)
    motor3_output = 2000;
  motor4_output = motor_idle + throttle - pid_output_roll + pid_output_pitch - pid_output_yaw;
  if (motor4_output < motor_idle)
    motor4_output = motor_idle;
  if (motor4_output > 2000)
    motor4_output = 2000;

  // generate PWM signals using the Servo lib
  ESC1.writeMicroseconds(motor1_output);
  ESC2.writeMicroseconds(motor2_output);
  ESC3.writeMicroseconds(motor3_output);
  ESC4.writeMicroseconds(motor4_output);
}
