#include "SensorFusion.h"
SensorFusion imu_sensor;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
    if(!imu_sensor.checkAvailability()) {
      return;
    }

    Serial.print("Roll: ");
    Serial.print(imu_sensor.get_roll());
    Serial.print(" Pitch:");
    Serial.println(imu_sensor.get_pitch());
}


