/*
 * basic PID controller:
 *  e[k] = r[k] - y[k]                      (error between setpoint and true position)
 *  e_d[k] = (e[k] - e[k-1]) / Tₛ           (filtered derivative)
 *  e_i[k+1] = e_i[k] + Tₛ e[k]             (integral)
 *  u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k] (control signal) (PID formula)
 *
 * phi = roll
 * theta = pitch
 * psi = yaw
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include <Servo.h>

#include "FlightController.h"

//---------CONSTANTS------------
#define m 10          // Total drone mass [Kg]
#define g 9.81        // Gravitational acceleration [m/s^2]
#define I_XX 8.2e-3   // Moments of inertia [kg·m²]
#define I_YY 8.2e-3   // Moments of inertia [kg·m²]
#define I_ZZ 14.2e-3  // Moments of inertia [kg·m²]
#define d_t 1.1e-6    // Torque drag coefficient [N·m·s²]
#define l 0.24        // Distance from CoG to propeller axis [m]
#define b 54.2e-6     // Thrust coefficient [N·s²]

#define MIN_PULSE_WIDTH 1000 // minimum pulse width in microseconds [us]
#define MAX_PULSE_WIDTH 2000 // Maximum pulse with in microseconds [us]
#define MAX_RPM 9000

// setup servo objects??
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Create PID Controllers (Objects)
PID Attitude(1, 1, 1, 1, 250, 20); // Attitude PID Controller
PID Roll(1, 1, 1, 1, 250, 20);     // Roll PID Controller
PID Pitch(1, 1, 1, 1, 250, 20);    // Pitch PID Controller
PID Yaw(1, 1, 1, 1, 250, 20);      // Yaw PID Controller

//Global variables
bool state = 0;

// Initialize
void PID::initializeMotors()
{
  // Initialize pins to control ESC
  motor1.attach(0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // Motor 1
  motor2.attach(0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // Motor 2
  motor3.attach(0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // Motor 3
  motor4.attach(0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // Motor 4
}

float PID::calculatePID(float setpoint, float measured)
{
  // Calculate error
  float error = setpoint - measured;

  // Calculate Proportional term
  float P_out = k_p * error;

  // Calculate Integral term
  float integral = previous_integral + error * T_s; // Summing up the error over time (helps eliminate steady-state error)
  float I_out = k_i * integral;

  // Calculate Derivative term
  float derivative = (error - previous_error) / T_s;
  float D_out = k_d * derivative;

  // Calculate total output
  double PIDoutput = P_out + I_out + D_out;

  // Clamp output to within defined limits (saturation)
  if(PIDoutput > max)
    PIDoutput = max;
  else if(PIDoutput < min)
    PIDoutput = min;
  else // Anti-windup
    previous_integral = integral; // only update integral if within saturation limits

  // Save error to previous error
  previous_error = error;

  return PIDoutput;
}

void MotorMixingAlgorithm(float F_z, float tau_phi, float tau_theta, float tau_psi)
{
  // Calculate the angular velocity of each propeller??
  float Omega1 = sqrt(1/(4*b)*F_z - 1/(2*b*l)*tau_theta - 1/(4*d_t)*tau_psi);
  float Omega2 = sqrt(1/(4*b)*F_z - 1/(2*b*l)*tau_phi + 1/(4*d_t)*tau_psi);
  float Omega3 = sqrt(1/(4*b)*F_z + 1/(2*b*l)*tau_theta - 1/(4*d_t)*tau_psi);
  float Omega4 = sqrt(1/(4*b)*F_z + 1/(2*b*l)*tau_phi + 1/(4*d_t)*tau_psi);

  // Update motor RPM
  MotorControl(Omega1, motor1);
  MotorControl(Omega2, motor2);
  MotorControl(Omega3, motor3);
  MotorControl(Omega4, motor4);
}

void MotorControl(float Omega, Servo& motor)
{
  // Convert angular velocity to ESC
  float RPM = (Omega * 60) / PI;

  // Calculate ESC signal (1000us-2000us)
  uint16_t esc_signal = MIN_PULSE_WIDTH + (RPM / MAX_RPM) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);

  // Output to ESC (i.e. update motor speed)
  if(state) // ON
    motor.writeMicroseconds(esc_signal);
  else // OFF
    motor.writeMicroseconds(MIN_PULSE_WIDTH);

}
