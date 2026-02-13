#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pami_com.h"

void Task_Comms(void *pvParameters) {
    initPAMI();
    Serial.println("[COMMS] Démarré");

    for (;;) {
        receiveFromSTM32();
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}
