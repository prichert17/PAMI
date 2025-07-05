#include <Arduino.h>
#include <Wire.h>
#include <VL53L8CX.h>

#define PWREN_PIN   -1
#define LPN_PIN     18

TwoWire &DEV_I2C = Wire;
VL53L8CX sensor(&DEV_I2C, LPN_PIN, -1);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(PWREN_PIN, OUTPUT);
  digitalWrite(PWREN_PIN, HIGH);
  delay(10);

  Serial.println("Init VL53L8CX...");
  sensor.begin();
  sensor.init();
  sensor.start_ranging();
  Serial.println("VL53L8CX prêt !");
}

void loop() {
  uint8_t ready = 0;
  while (!ready) sensor.check_data_ready(&ready);
  
  VL53L8CX_ResultsData results;
  sensor.get_ranging_data(&results);

  Serial.println("Distances zones 8×8 :");
  for (int i = 0; i < 64; ++i) {
    int distance = results.distance_mm[i];
    if (distance > 0 && distance < 4000) {
      Serial.printf("%2d:%4d mm\n", i, distance);
    }
  }
  Serial.println();
  delay(200);
}
