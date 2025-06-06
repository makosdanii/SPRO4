#ifndef _FLIGHTCONTROLLER_H_
#define _FLIGHTCONTROLLER_H_

#include <stdint.h> // For uint8_t type
#include <string.h>

class PID
{
public:
  /// @param  Kp Proportional gain
  /// @param  Ki Integral gain
  /// @param  Kd Derivative gain
  /// @param  Ts Sampling time (in seconds)
  /// @param  Max max output value
  /// @param  Min min output value
  /// @param  N ???
  PID(float Kp, float Ki, float Kd, float Ts, uint16_t Max, uint16_t Min, float N)
    : k_p(Kp), k_i(Ki), k_d(Kd), T_s(Ts), max(Max), min(Min), N(N) {} // Constructor initializer

  static void initializeMotors();
  float calculatePID(float setpoint, float measured); // Calculates

private:
  const float k_p, T_s, k_i, k_d, N; // Constant member values
  const uint16_t max, min;        // Output clamping values (to avoid actuator saturation)
  float previous_error = 0;       // sets Initial value
  float previous_ef = 0;          // Previous filtered error
  float integral = 0;             // sets Initial value
  float alpha = (T_s*N)/(2+T_s*N);// first-order low-pass IIR filter Coefficinet??? or something
};

// Declare PID objects
extern PID Attitude;  // Attitude PID Controller
extern PID Roll;      // Roll PID Controller
extern PID Pitch;     // Pitch PID Controller
extern PID Yaw;       // Yaw PID Controller

// Function Headers
void MotorMixingAlgorithm(float F_z, float tau_phi, float tau_theta, float tau_psi);
void MotorControl(float Omega, Servo& motor);

#endif
