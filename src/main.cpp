#include "Arduino_BMI270_BMM150.h"

#define SAMPLING_RATE 100 // in Hz
#define TUNE 0.7 // for accelerometer

void convert_accel(void);
void convert_gyro(void);
float comp_filt_roll(float, float, float);
float comp_filt_pitch(float, float, float);

float x, y, z;
float Rg = 0;
float Pg = 0;
float Ra, Pa;
float roll, pitch, yaw;
float new_roll, new_pitch;
bool roll_f = 0;
bool pitch_f = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
}

void loop() {
  while (!IMU.gyroscopeAvailable()){}
  while (!IMU.accelerationAvailable()){}
  while (!IMU.magneticFieldAvailable()){}

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
    pitch = new_pitch;

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch:");
    Serial.println(pitch);
}

void convert_gyro(){
    IMU.readGyroscope(x, y, z);

    Rg = x*(1.0f/SAMPLING_RATE);
    Pg = y*(1.0f/SAMPLING_RATE);
}

void convert_accel(){
    IMU.readAcceleration(x, y, z);
    Ra = atan2(y , z);
    float math_haha = sqrt(y*y + z*z);
    Pa = atan2((- x) , math_haha);
}

float comp_filt_roll(float ra, float rg, float prev_angle){
  float angle;

    angle = ( ( rg+prev_angle )*( 1-TUNE ) )+( ra*TUNE );

  return angle;
}

float comp_filt_pitch(float pa, float pg, float prev_angle){
  float angle;

    angle = ( ( pg+prev_angle )*( 1-TUNE ) )+( pa*TUNE );

  return angle;
}


