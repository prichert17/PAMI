#include <Arduino.h>
#include <Wire.h>
#include <VL53L8CX.h>

#define LPN_PIN     18

TwoWire &DEV_I2C = Wire;
VL53L8CX sensor(&DEV_I2C, LPN_PIN, -1);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(10);

  Serial.println("Init VL53L8CX...");
  sensor.begin();
  sensor.init();
  sensor.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor.set_ranging_frequency_hz(5);
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
      Serial.printf("Centre (zone 27) : %d mm\n", results.distance_mm[27]);
    }
  }
  Serial.println();
  delay(200);
}
