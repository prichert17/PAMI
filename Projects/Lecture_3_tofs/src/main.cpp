#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(1000);
  Serial.println("Scan I2C en cours...");
  for (uint8_t i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
      Serial.printf("Device I2C trouvé à l'adresse: 0x%02X\n", i);
  }
  Serial.println("Scan terminé");
}

void loop() {}
