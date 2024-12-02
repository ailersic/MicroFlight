#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

const float eps = 0.000001;

Matrix<4> state;
Matrix<4,4> cov;
Matrix<4,4> init_cov;

Matrix<4> state_pred;
Matrix<4,4> cov_pred;

Matrix<4,4> proc_cov;
Matrix<6,6> meas_cov;

Matrix<4,4> identity4;
Matrix<6,6> identity6;

void initEKF() {
  state.Fill(0); state(0) = 1.0;
  cov.Fill(0); for (int i = 0; i < cov.Rows; i++) cov(i, i) = 1.0;

  Matrix<3> gyro_offsets = {gX0, gY0, gZ0};
  float gyro_norm = Norm(gyro_offsets);
  float gyro_var = gyro_norm*gyro_norm/(LOOP_FREQ*LOOP_FREQ);

  Matrix<proc_cov.Rows> proc_var = {gyro_var, gyro_var, gyro_var, gyro_var};
  Matrix<meas_cov.Rows> meas_var = {0.0001, 0.0001, 0.0001, 0.01, 0.01, 0.01};

  init_cov.Fill(0); for (int i = 0; i < init_cov.Rows; i++) init_cov(i, i) = 1.0;
  proc_cov.Fill(0); for (int i = 0; i < proc_cov.Rows; i++) proc_cov(i, i) = proc_var(i);
  meas_cov.Fill(0); for (int i = 0; i < meas_cov.Rows; i++) meas_cov(i, i) = meas_var(i);

  identity4.Fill(0); for (int i = 0; i < identity4.Rows; i++) identity4(i, i) = 1.0;
  identity6.Fill(0); for (int i = 0; i < identity6.Rows; i++) identity6(i, i) = 1.0;
}

void getRollPitchYaw() {
  prediction();
  correction();

  float q0 = state(0), q1 = state(1), q2 = state(2), q3 = state(3);
  
  // Roll (rotation around X-axis)
  roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180.0/PI;
  
  // Pitch (rotation around Y-axis)
  // Use 90 degrees when out of range
  if (fabs(2*(q0*q2 - q3*q1)) >= 1) pitch = copysign(PI/2, 2*(q0*q2 - q3*q1)) * 180.0/PI;
  else pitch = asin(2*(q0*q2 - q3*q1)) * 180.0/PI;
  
  // Yaw (rotation around Z-axis)
  yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0/PI;
}

void prediction() {
  Matrix<4,4> omega = {0, -gX, -gY, -gZ,
                      gX,   0,  gZ, -gY,
                      gY, -gZ,   0,  gX,
                      gZ,  gY, -gX,  0};

  Matrix<4> dq = omega * state * (float) (dt / 2.0);
  Matrix<4,4> jacobian = omega * (float) (dt / 2.0);

  // gyro is too noisy to use without messing up prediction
  state_pred = state;// + dq;
  //state_pred /= Norm(state_pred);
  cov_pred = jacobian * cov * ~jacobian + proc_cov;
}

void correction() {
  float q0 = state(0), q1 = state(1), q2 = state(2), q3 = state(3);

  Matrix<3,3> rot = {1 - 2*q2*q2 - 2*q3*q3,     2*q1*q2 - 2*q0*q3,     2*q1*q3 + 2*q0*q2,
                         2*q1*q2 + 2*q0*q3, 1 - 2*q1*q1 - 2*q3*q3,     2*q2*q3 - 2*q0*q1,
                         2*q1*q3 - 2*q0*q2,     2*q2*q3 + 2*q0*q1, 1 - 2*q1*q1 - 2*q2*q2};

  Matrix<3> grav_obs = {aX, aY, aZ};
  Matrix<3> mag_obs = {mX, mY, mZ};

  float mag_angle = PI/2;//acos((~grav_obs * mag_obs)(0, 0) / (Norm(grav_obs) * Norm(mag_obs)));
  Matrix<3> grav_ned = {0, 0, 1};
  Matrix<3> mag_ned = {sin(mag_angle), 0, cos(mag_angle)}; // got to revise this for case where mag is not orthogonal to grav

  Matrix<6> obs_pred = (rot * grav_ned) && (rot * mag_ned);
  Matrix<6> obs = (grav_obs / Norm(grav_obs)) && (mag_obs / Norm(mag_obs));

  Matrix<3,4> jacobian_grav = {2*q2,  2*q3,  2*q0,  2*q1,
                              -2*q1, -2*q0,  2*q3,  2*q2,
                                  0, -4*q1, -4*q2,     0};
  Matrix<3,4> jacobian_mag = {                       2*q2*cos(mag_angle),                        2*q1*cos(mag_angle),  2*q0*cos(mag_angle) - 4*q2*sin(mag_angle),  2*q1*cos(mag_angle) - 4*q3*sin(mag_angle),
                               2*q3*sin(mag_angle) - 2*q1*cos(mag_angle),  2*q2*sin(mag_angle) - 2*q0*cos(mag_angle),  2*q1*sin(mag_angle) + 2*q3*cos(mag_angle),  2*q0*sin(mag_angle) + 2*q2*cos(mag_angle),
                                                    -2*q2*sin(mag_angle),  2*q3*sin(mag_angle) - 4*q1*cos(mag_angle), -2*q0*sin(mag_angle) - 4*q2*cos(mag_angle),                        2*q1*sin(mag_angle)};
  /*{   0,     0, -4*q2, -4*q3,
                              2*q3,  2*q2,  2*q1,  2*q0,
                             -2*q2,  2*q3, -2*q0,  2*q1};*/

  Matrix<6,4> jacobian = jacobian_grav && jacobian_mag;

  Matrix<6> innov = obs - obs_pred;
  Matrix<6,6> innov_cov = jacobian * cov_pred * ~jacobian + meas_cov;
  Matrix<4,6> kalman = cov_pred * ~jacobian * Inverse(innov_cov + eps*identity6);

  state = state_pred + kalman * innov;
  state /= Norm(state);
  cov = (identity4 - (kalman * jacobian)) * cov_pred;
}
