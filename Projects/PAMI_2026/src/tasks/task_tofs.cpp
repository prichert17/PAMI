#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tofs.h"

void Task_Tofs(void *pvParameters) {
    Serial.println("[TOFS] Init...");
    setup_tof();
    Serial.println("[TOFS] Prêt");

    for (;;) {
        loop_tof();
        calcule_points_tof();
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}
