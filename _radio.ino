int channelIndex = 0;
unsigned long time_ms = 0;

void setupPPM() {
  // Set the PPM pin as input
  pinMode(PPM_PIN, INPUT_PULLUP);
  delay(20);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), getPPM, CHANGE);
}

void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_PIN);
  if (trig==1) { //Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    if (dt_ppm > 3000) { //Waiting for long pulse to indicate a new pulse train has arrived
      channelIndex = 0;
    }
    else if (channelIndex < 6) {
      radioChannels[channelIndex] = dt_ppm;
      channelIndex = channelIndex + 1;
    }
    
  }
}
