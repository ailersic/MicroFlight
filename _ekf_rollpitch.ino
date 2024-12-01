#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

//const float PI = 3.14159265;
const float eps = 0.000001;

Matrix<2> state;
Matrix<2,2> cov;
Matrix<2,2> init_cov;

Matrix<2> state_pred;
Matrix<2,2> cov_pred;

Matrix<2,2> proc_cov;
Matrix<3,3> meas_cov;

Matrix<2,2> identity2;
Matrix<3,3> identity3;

float boundAngleDeg(float angle) {
  float angle360 = fmod(angle, 360.0);
  if (angle360 > 180.0) return angle360 - 360.0;
  else return angle360;
}

void initEKF() {
  state.Fill(0);
  cov = {0.1, 0.0, 0.0, 0.1};
  init_cov = {0.1, 0.0, 0.0, 0.1};

  proc_cov = {0.1, 0.0, 0.0, 0.1};
  meas_cov = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

  identity2 = {1.0, 0.0, 0.0, 1.0};
  identity3 = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

void getRollPitch() {
  prediction();
  correction();

  for (int i = 0; i < state.Rows; i++) state(i) = boundAngleDeg(state(i));

  if (isnan(state(0)) || isnan(state(1))) {
    state(0) = roll;
    state(1) = pitch;
    cov = init_cov;
  }
  else {
    roll = state(0);
    pitch = state(1);
    yaw = 0.0;
  }
}

void prediction() {
  float roll_rad = state(0)*PI/180.0;
  float pitch_rad = state(1)*PI/180.0;

  float d_roll = gX + sin(roll_rad)*tan(pitch_rad)*gY + cos(roll_rad)*tan(pitch_rad)*gZ;
  float d_pitch = cos(roll_rad)*gY - sin(roll_rad)*gZ;
  Serial.print(d_roll); Serial.print(" ... "); Serial.println(d_pitch);

  Matrix<2> d_state = {d_roll, d_pitch};
  Matrix<2,2> jacobian = {cos(roll_rad)*tan(pitch_rad)*gY - sin(roll_rad)*tan(pitch_rad)*gZ, (sin(roll_rad)*gY + cos(roll_rad)*gZ)/(cos(pitch_rad)*cos(pitch_rad)),
                          -sin(roll_rad)*gY - cos(roll_rad)*gZ, 0};

  state_pred = state + d_state*dt;
  cov_pred = jacobian * cov * ~jacobian + proc_cov;
}

void correction() {
  float roll_rad = state_pred(0)*PI/180.0;
  float pitch_rad = state_pred(1)*PI/180.0;

  Matrix<3> g_pred = {sin(pitch_rad)*cos(roll_rad), -sin(roll_rad), cos(pitch_rad)*cos(roll_rad)};
  Matrix<3> g_obs = {aX, aY, aZ};
  Matrix<3,2> jacobian = {-sin(roll_rad)*sin(pitch_rad), cos(pitch_rad)*cos(roll_rad),
                          -cos(roll_rad), 0.0,
                          -cos(pitch_rad)*sin(roll_rad), -sin(pitch_rad)*cos(roll_rad)};

  Matrix<3> innov = g_obs - g_pred;
  Matrix<3,3> innov_cov = jacobian * cov_pred * ~jacobian + meas_cov;
  Matrix<2,3> kalman = cov_pred * ~jacobian * Inverse(innov_cov + eps*identity3);

  state = state_pred + kalman * innov;
  cov = (identity2 - (kalman * jacobian)) * cov_pred;
}
