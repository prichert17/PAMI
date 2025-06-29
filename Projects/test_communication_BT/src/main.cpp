#include <Arduino.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);         // debug (pas obligatoire)
  SerialBT.begin("PAMI_BT");    // nom qui apparaîtra sur Windows
  SerialBT.println("Bluetooth Serial Started");
}

void loop() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("RESET")) {
      ESP.restart();
    }
  }
  int sensorValue = analogRead(34);
  SerialBT.print("SENSOR:");
  SerialBT.println(sensorValue);
  delay(1000);
}
