// Actual PID calculactions
float roll_error, pitch_error, yaw_error;
float roll_integral, pitch_integral, yaw_integral;
float roll_prev_error, pitch_prev_error;

// reset inegrals and previous errors when arming
void resetPID() {
  roll_integral = 0;
  pitch_integral = 0;
  yaw_integral = 0;
  roll_prev_error = 0;
  pitch_prev_error = 0;
}

void calculatePID () {
  // ----------------- ROLL -----------------
  // calculate pid_error in deg/sec
  roll_error = gyro_roll - roll_setpoint;
  roll_integral += roll_i_gain * roll_error;

  // clip integral sum if exceeding maximum
  if (roll_integral > pid_output_limit)
    roll_integral = pid_output_limit;
  else if (roll_integral < -pid_output_limit)
    roll_integral = -pid_output_limit;

  // calc pid output in us
  pid_output_roll = roll_p_gain * roll_error + roll_integral + roll_d_gain * (roll_error - roll_prev_error);
  roll_prev_error = roll_error;

  // clip output if exceeding maximum
  if (pid_output_roll > pid_output_limit)
    pid_output_roll = pid_output_limit;
  else if (pid_output_roll < -pid_output_limit)
    pid_output_roll = -pid_output_limit;


  // ----------------- PITCH -----------------
  pitch_error = gyro_pitch - pitch_setpoint;
  pitch_integral += pitch_i_gain * pitch_error;

  if (pitch_integral > pid_output_limit)
    pitch_integral = pid_output_limit;
  else if (pitch_integral < -pid_output_limit)
    pitch_integral = -pid_output_limit;

  pid_output_pitch = pitch_p_gain * pitch_error + pitch_integral + pitch_d_gain * (pitch_error - pitch_prev_error);
  pitch_prev_error = pitch_error;

  if (pid_output_pitch > pid_output_limit)
    pid_output_pitch = pid_output_limit;
  else if (pid_output_pitch < -pid_output_limit)
    pid_output_pitch = -pid_output_limit;


  // ----------------- YAW -----------------
  yaw_error = gyro_yaw - yaw_setpoint;
  yaw_integral += yaw_i_gain * yaw_error;

  if (yaw_integral > pid_output_limit)
    yaw_integral = pid_output_limit;
  else if (yaw_integral < -pid_output_limit)
    yaw_integral = -pid_output_limit;

  pid_output_yaw = yaw_p_gain * yaw_error + yaw_integral;

  if (pid_output_yaw > pid_output_limit)
    pid_output_yaw = pid_output_limit;
  else if (pid_output_yaw < -pid_output_limit)
    pid_output_yaw = -pid_output_limit;
}
