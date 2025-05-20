#include <Arduino.h>
#include <ArduinoBLE.h>
#include "BLE.h"

BLEService controlService("180A");  // Custom BLE Service
BLEByteCharacteristic stateCharacteristic("2A57", BLERead | BLEWrite);

int states = 0;  // Variable to be controlled via Bluetooth
BLEDevice central;  // Store the central device???

void initializeBLE()
{
  BLE.begin();
  BLE.setLocalName("BLE-State-Control");
  BLE.setAdvertisedService(controlService);

  controlService.addCharacteristic(stateCharacteristic);
  BLE.addService(controlService);

  stateCharacteristic.writeValue(states);  // Set initial value

  BLE.advertise();
  Serial.println("BLE ready.");
}

void checkBLE()
{
  // Only call BLE.central() once to detect new connections
  if (!central) {
    central = BLE.central();
  }

  // Check if central is connected
  if (central && central.connected()) {
    if (stateCharacteristic.written()) {
      states = stateCharacteristic.value();
      Serial.print("New state received via BLE: ");
      Serial.println(states);
    }
  } else {
    central = BLEDevice();  // Reset if disconnected
  }
}
