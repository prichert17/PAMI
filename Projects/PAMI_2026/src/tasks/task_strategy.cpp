#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "types.h"

// Variables globales
extern float current_x, current_y, current_theta;
extern float target_x, target_y;
extern bool mode_auto;
extern SemaphoreHandle_t xPoseMutex;

static RobotState state = STATE_WAIT;
static unsigned long matchStartTime = 0;

void Task_Strategy(void *pvParameters) {
    Serial.println("[STRATEGY] Démarré");

    for (;;) {
        switch (state) {
            case STATE_WAIT:
                // Attente tirette
                if (digitalRead(PIN_TIRETTE) == LOW) {
                    state = STATE_GAME;
                    matchStartTime = millis();
                    mode_auto = true;
                    Serial.println("[STRATEGY] GO!");
                }
                break;

            case STATE_GAME:
                // Fin de match ?
                if ((millis() - matchStartTime) >= MATCH_DURATION_MS) {
                    state = STATE_END;
                    mode_auto = false;
                    Serial.println("[STRATEGY] FIN");
                }
                // TODO: Logique de déplacement
                break;

            case STATE_END:
                // Arrêt complet
                break;

            case STATE_MANUAL:
                // Debug
                break;

            case STATE_ERROR:
                mode_auto = false;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}
