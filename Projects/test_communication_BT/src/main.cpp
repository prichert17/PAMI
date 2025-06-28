#include <Arduino.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);         // debug (pas obligatoire)
  SerialBT.begin("PAMI_BT");    // nom qui apparaîtra sur Windows
}

void loop() {
  int sensorValue = analogRead(34); // ou la pin que tu veux
  SerialBT.print("SENSOR:");
  SerialBT.println(sensorValue);
  delay(1000);
}

