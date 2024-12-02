//Teensy Flight Controller - MicroFlight
//Author: Andrew Ilersich
//Based on dRehmFlight by Nicholas Rehm
//Project Start: 31/5/2024
//Last Updated: 1/12/2024
//Version: 1.0

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

#include <Wire.h>     //I2c communication
#include <Servo.h>

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

// Radio channels
const int NUM_CHANNELS = 6;

const int AIL_CHANNEL = 0;
const int ELV_CHANNEL = 1;
const int THR_CHANNEL = 2;
const int RUD_CHANNEL = 3;
const int AUX_CHANNEL = 4;
const int MODE_CHANNEL = 5;

const float AIL_NORM_LLIM = -1, AIL_NORM_ULIM = 1; // ail_norm_out range: [-1, 1]
const float ELV_NORM_LLIM = -1, ELV_NORM_ULIM = 1; // elv_norm_out range: [-1, 1]
const float THR_NORM_LLIM =  0, THR_NORM_ULIM = 1; // thr_norm_out range: [ 0, 1]
const float RUD_NORM_LLIM = -1, RUD_NORM_ULIM = 1; // rud_norm_out range: [-1, 1]
const float AUX_NORM_LLIM =  0, AUX_NORM_ULIM = 1; // aux_norm_out range: [ 0, 1]

// Radio thresholds for valid signals. Outside of these, controller will enter failsafe mode.
const int RADIO_MIN = 800;
const int RADIO_MAX = 2200;

// Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
const int AIL_FAILSAFE = 1500; // middle position
const int ELV_FAILSAFE = 1500; // middle position
const int THR_FAILSAFE = 1000; // low position
const int RUD_FAILSAFE = 1500; // middle position
const int AUX_FAILSAFE = 2000; // high position

// list of all flight modes
const int MODE_FAILSAFE = 0;
const int MODE_MANUAL = 1;
const int MODE_FLYBYWIRE = 2;

// three position switch on mode channel supports three flight modes
const int MODE_1 = MODE_FAILSAFE;
const int MODE_2 = MODE_MANUAL;
const int MODE_3 = MODE_FLYBYWIRE;

// threshold for mode switch
const int MODE_1_2_THRES = 1300;
const int MODE_2_3_THRES = 1700;

// frequency of main loop
const int LOOP_FREQ = 2000; // Hz

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Define the pin connected to the PPM signal
const int PPM_PIN = 23;

//Define control output pins
const int AIL_PIN = 6;
const int ELV_PIN = 7;
const int THR_PIN = 8;
const int RUD_PIN = 9;
const int AUX_PIN = 10;

//========================================================================================================================//

//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Flight mode
unsigned int flight_mode;

// Normalized control outputs
float ail_norm_out, elv_norm_out, thr_norm_out, rud_norm_out, aux_norm_out;
// PWM control outputs
int ail_pwm_out, elv_pwm_out, thr_pwm_out, rud_pwm_out, aux_pwm_out;

//Radio communication:
unsigned long radioChannels[NUM_CHANNELS];
Servo ail_servo, elv_servo, thr_servo, rud_servo, aux_servo;

//Sensors available
bool MPU6050_avail = false;
bool QMC5883L_avail = false;
bool BMP180_avail = false;

//Sensor values:
float aX0 = 0, aY0 = 0, aZ0 = 0; // in Gs
float gX0 = 0, gY0 = 0, gZ0 = 0; // in deg/s
float mXul = 0, mXll = 0;
float mYul = 0, mYll = 0;
float mZul = 0, mZll = 0;
float pres0 = 0; // in kPa

float aX = 0, aY = 0, aZ = 0; // in Gs
float gX = 0, gY = 0, gZ = 0; // in deg/s
float mX = 0, mY = 0, mZ = 0;
float pres = 0; // in kPa

//State
float roll, pitch, yaw; // in degrees

//Flight status
bool armedFly = false;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 

  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  setupPPM(); //Initialize radio communication
  setupServos(); //Initialize output PWM servos

  Wire.begin(); //Initialize I2C
  enableBypass();
  checkSensors(); //Check which sensors are available

  if (MPU6050_avail) setupAccelGyro(); //Initialize accel/gyro communication
  if (QMC5883L_avail) setupMag(); //Initialize magnetometer communication
  if (BMP180_avail) setupBaro(); //Initialize barometer communication

  initEKF();
  
  delay(5);
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Get vehicle state
  if (MPU6050_avail) getAccelGyro(); //Pulls raw gyro, accelerometer data
  if (QMC5883L_avail) getMag(); //Pulls raw magnetometer data
  if (BMP180_avail) getBaro(); //Pulls raw barometer data
  
  getRollPitchYaw();

  updateMode();
  controlByMode();

  writeToServos();

  printAttitude();
  //printStatus();
  
  //Regulate loop rate
  loopRate(LOOP_FREQ); //Do not exceed 2000Hz
}

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//

void updateMode() {
  // If radio connection isn't working, switch into failsafe mode
  bool check = false;
  for (int i = 0; i < NUM_CHANNELS; i++) check = check || radioChannels[i] > RADIO_MAX || radioChannels[i] < RADIO_MIN;

  if (check) flight_mode = MODE_FAILSAFE;
  else {
    // Once we know we have a good radio connection, follow flight mode switch
    int mode_pwm = radioChannels[MODE_CHANNEL];
    if (mode_pwm < MODE_1_2_THRES) flight_mode = MODE_1;
    else if (mode_pwm < MODE_2_3_THRES) flight_mode = MODE_2;
    else flight_mode = MODE_3;
  }
}

void loopRate(int freq) {
  // Regulate main loop rate to specified frequency in Hz
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  // Blink LED on board to indicate main loop is running
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  // Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}
