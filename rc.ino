// Functions for reading the rc input
void readIbus() {
  IBus.loop();

  // update arm status
  if (IBus.readChannel(4) > 1500 && IBus.readChannel(2) < 1010 && !armed) {
    // reset PID controller and arm if switch is above 1500us
    resetPID();
    armed = true;
  } else if (IBus.readChannel(4) < 1500 && armed) {
    armed = false;
  }

  throttle = float(IBus.readChannel(2) - 1000) / 1000 * (2000 - motor_idle);

  // convert and scale rc input using a expo curve [deg/s]
  roll_setpoint = pow((float(IBus.readChannel(0)) - 1500) / 500, 3) * 200 * rp_rcrate;
  pitch_setpoint = pow((float(IBus.readChannel(1)) - 1500) / 500, 3) * 200 * rp_rcrate;
  yaw_setpoint = -pow((float(IBus.readChannel(3)) - 1500) / 500, 3) * 200 * yaw_rcrate;
}
