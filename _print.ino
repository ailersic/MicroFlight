void printStatus() {
  Serial.println("---");
  Serial.print("Flight mode: "); Serial.println(flight_mode);

  Serial.print("Accel X: "); Serial.print(aX);
  Serial.print(" | Accel Y: "); Serial.print(aY);
  Serial.print(" | Accel Z: "); Serial.println(aZ);
  Serial.print("Gyro X: "); Serial.print(gX);
  Serial.print(" | Gyro Y: "); Serial.print(gY);
  Serial.print(" | Gyro Z: "); Serial.println(gZ);
  Serial.print("Mag X: "); Serial.print(mX);
  Serial.print(" | Mag Y: "); Serial.print(mY);
  Serial.print(" | Mag Z: "); Serial.println(mZ);

  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Yaw: "); Serial.print(yaw, 2);
  Serial.println();

  for (int i = 0; i < 6; i++) {
    Serial.print(" CH");
    Serial.print(i + 1);
    Serial.print(" PWM: ");
    Serial.print(radioChannels[i]);
  }
  Serial.println();

  Serial.print(" AIL PWM: "); Serial.print(ail_pwm_out);
  Serial.print(" ELV PWM: "); Serial.print(elv_pwm_out);
  Serial.print(" THR PWM: "); Serial.print(thr_pwm_out);
  Serial.print(" RUD PWM: "); Serial.print(rud_pwm_out);
  Serial.print(" AUX PWM: "); Serial.println(aux_pwm_out);
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}