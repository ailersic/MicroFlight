const int AIL_SCALE =  (int) (1000.0 / (AIL_NORM_ULIM - AIL_NORM_LLIM));
const int AIL_OFFSET = (int) (2000.0 - (AIL_NORM_ULIM * AIL_SCALE));
const int ELV_SCALE =  (int) (1000.0 / (ELV_NORM_ULIM - ELV_NORM_LLIM));
const int ELV_OFFSET = (int) (2000.0 - (ELV_NORM_ULIM * ELV_SCALE));
const int THR_SCALE =  (int) (1000.0 / (THR_NORM_ULIM - THR_NORM_LLIM));
const int THR_OFFSET = (int) (2000.0 - (THR_NORM_ULIM * THR_SCALE));
const int RUD_SCALE =  (int) (1000.0 / (RUD_NORM_ULIM - RUD_NORM_LLIM));
const int RUD_OFFSET = (int) (2000.0 - (RUD_NORM_ULIM * RUD_SCALE));
const int AUX_SCALE =  (int) (1000.0 / (AUX_NORM_ULIM - AUX_NORM_LLIM));
const int AUX_OFFSET = (int) (2000.0 - (AUX_NORM_ULIM * AUX_SCALE));

const float P_AIL = 0.01;
const float P_ELV = 0.01;

const float MAX_BANK_ANGLE = 45.0;
const float MAX_PITCH_ANGLE = 45.0;

void controlByMode() {
  switch (flight_mode) {
    case MODE_FAILSAFE:
      controlFailsafe();
      break;

    case MODE_MANUAL:
      controlManual();
      break;

    case MODE_FLYBYWIRE:
      controlFlyByWire();
      break;

    default:
      controlFailsafe();
      break;
  }
}

void controlFailsafe() {
  ail_pwm_out = AIL_FAILSAFE;
  elv_pwm_out = ELV_FAILSAFE;
  thr_pwm_out = THR_FAILSAFE;
  rud_pwm_out = RUD_FAILSAFE;
  aux_pwm_out = AUX_FAILSAFE;
}

void controlManual() {
  ail_pwm_out = radioChannels[AIL_CHANNEL];
  elv_pwm_out = radioChannels[ELV_CHANNEL];
  thr_pwm_out = radioChannels[THR_CHANNEL];
  rud_pwm_out = radioChannels[RUD_CHANNEL];
  aux_pwm_out = radioChannels[AUX_CHANNEL];
}

void controlFlyByWire() {
  float roll_cmd = ((float) radioChannels[AIL_CHANNEL] - AIL_OFFSET) / AIL_SCALE;
  float roll_err = MAX_BANK_ANGLE*roll_cmd - roll;
  ail_norm_out = P_AIL * roll_err;

  float pitch_cmd = ((float) radioChannels[ELV_CHANNEL] - ELV_OFFSET) / ELV_SCALE;
  float pitch_err = MAX_PITCH_ANGLE*pitch_cmd - pitch;
  elv_norm_out = P_ELV * pitch_err;

  clampAllNorm();
  normToPWM();

  thr_pwm_out = radioChannels[THR_CHANNEL];
  rud_pwm_out = radioChannels[RUD_CHANNEL];
  aux_pwm_out = radioChannels[AUX_CHANNEL];
}

void normToPWM() {
  ail_pwm_out = (int) (ail_norm_out * AIL_SCALE + AIL_OFFSET);
  elv_pwm_out = (int) (elv_norm_out * ELV_SCALE + ELV_OFFSET);
  thr_pwm_out = (int) (thr_norm_out * THR_SCALE + THR_OFFSET);
  rud_pwm_out = (int) (rud_norm_out * RUD_SCALE + RUD_OFFSET);
  aux_pwm_out = (int) (aux_norm_out * AUX_SCALE + AUX_OFFSET);
}

void clampAllNorm() {
  clamp(ail_norm_out, AIL_NORM_ULIM, AIL_NORM_LLIM);
  clamp(elv_norm_out, ELV_NORM_ULIM, ELV_NORM_LLIM);
  clamp(thr_norm_out, THR_NORM_ULIM, THR_NORM_LLIM);
  clamp(rud_norm_out, RUD_NORM_ULIM, RUD_NORM_LLIM);
  clamp(aux_norm_out, AUX_NORM_ULIM, AUX_NORM_LLIM);
}

void clamp(float &x, float ul, float ll) {
  if (x < ll) x = ll;
  else if (x > ul) x = ul;
}
