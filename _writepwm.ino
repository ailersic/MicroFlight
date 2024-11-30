
void setupServos() {
  ail_servo.attach(AIL_PIN, 900, 2100); //Pin, min PWM value, max PWM value
  elv_servo.attach(ELV_PIN, 900, 2100);
  thr_servo.attach(THR_PIN, 900, 2100);
  rud_servo.attach(RUD_PIN, 900, 2100);
  aux_servo.attach(AUX_PIN, 900, 2100);
}

void writeToServos() {
  ail_servo.writeMicroseconds(ail_pwm_out);
  elv_servo.writeMicroseconds(elv_pwm_out);
  thr_servo.writeMicroseconds(thr_pwm_out);
  rud_servo.writeMicroseconds(rud_pwm_out);
  aux_servo.writeMicroseconds(aux_pwm_out);
}