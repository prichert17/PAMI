#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "types.h"

void Task_IHM(void *pvParameters) {
    Serial.println("[IHM] Init...");
    
    // Init PWM servos (50Hz pour servos standard)
    ledcSetup(0, 50, 16);
    ledcSetup(1, 50, 16);
    ledcSetup(2, 50, 16);
    ledcAttachPin(PIN_SERVO_1, 0);
    ledcAttachPin(PIN_SERVO_2, 1);
    ledcAttachPin(PIN_SERVO_3, 2);
    
    // Init LED strip
    pinMode(PIN_LED_DATA, OUTPUT);
    
    Serial.println("[IHM] Prêt");

    for (;;) {
        // TODO: MAJ LEDs selon état robot
        // TODO: Commandes servos
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}
