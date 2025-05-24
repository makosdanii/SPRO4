#include "SensorFusion.h"
#include <Arduino.h>

bool SensorFusion::checkAvailability()
{
    unsigned long initial_time = millis();
    while (!IMU.gyroscopeAvailable()){
        if (millis() - initial_time > 1000) {
            Serial.println("Gyroscope not available");
            return false;
        }
    }
    while (!IMU.accelerationAvailable()){
        if (millis() - initial_time > 1000) {
            Serial.println("Acceleration not available");
            return false;
        }
    }
    while (!IMU.magneticFieldAvailable()){
        if (millis() - initial_time > 1000) {
            Serial.println("Magneto not available");
            return false;
        }
    }

    return true;
}

void SensorFusion::convert_gyro(){
    IMU.readGyroscope(x, y, z);

    Rg = x*(1.0f/SAMPLING_RATE);
    Pg = y*(1.0f/SAMPLING_RATE);
}

void SensorFusion::convert_accel(){
    IMU.readAcceleration(x, y, z);
    Ra = atan2(y , z);
    float math_haha = sqrt(y*y + z*z);
    Pa = atan2((- x) , math_haha);
}

float SensorFusion::comp_filt_roll(float ra, float rg, float prev_angle){
  float angle;

    angle = ( ( rg+prev_angle )*( 1-TUNE ) )+( ra*TUNE );

  return angle;
}

float SensorFusion::comp_filt_pitch(float pa, float pg, float prev_angle){
  float angle;

    angle = ( ( pg+prev_angle )*( 1-TUNE ) )+( pa*TUNE );

  return angle;
}

float SensorFusion::get_roll(){
    IMU.readMagneticField(x, y, z);
    convert_gyro();
    convert_accel();
    roll_f = true;
    new_roll = comp_filt_roll(Ra, Rg, roll);
    roll_f = false;

    pitch_f = true;
    new_pitch = comp_filt_pitch(Pa, Pg, pitch);
    pitch_f = false;

    roll = new_roll;
    return roll;
}

float SensorFusion::get_pitch(){
    IMU.readMagneticField(x, y, z);
    convert_gyro();
    convert_accel();
    roll_f = true;
    new_roll = comp_filt_roll(Ra, Rg, roll);
    roll_f = false;

    pitch_f = true;
    new_pitch = comp_filt_pitch(Pa, Pg, pitch);
    pitch_f = false;

    pitch = new_pitch;
    return pitch;
}