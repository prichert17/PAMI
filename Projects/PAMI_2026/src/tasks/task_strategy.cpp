#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "types.h"
#include "pami_com.h"
#include "actuators.h"

// Variables globales
extern float current_x, current_y, current_theta;
extern float target_x, target_y;
extern bool mode_auto;
extern SemaphoreHandle_t xPoseMutex;

static unsigned long matchStartTime = 0;
static unsigned long tiretteTime = 0;
static const unsigned long PAMI_DELAY_MS = 85000; // 85s avant départ PAMI

void Task_Strategy(void *pvParameters) {
    Serial.println("[STRATEGY] Démarré");

    // Init servos et moteurs
    initServos();
    initMotors();

    // Vérifier si mode debug activé au démarrage
    if (digitalRead(PIN_SW_DEBUG) == LOW) {
        state = STATE_TEST;
        Serial.println("[STRATEGY] Mode TEST activé");
    }

    for (;;) {
        switch (state) {
            case STATE_WAIT:
                // Attente tirette
                if (digitalRead(PIN_TIRETTE) == LOW) {
                    tiretteTime = millis();
                    state = STATE_DELAY;
                    Serial.println("[STRATEGY] Tirette! Attente 85s...");
                }
                break;

            case STATE_DELAY:
                // Attente 85s avant départ
                if ((millis() - tiretteTime) >= PAMI_DELAY_MS) {
                    state = STATE_GAME;
                    matchStartTime = millis();
                    mode_auto = true;
                    sendPosition(target_x, target_y); // Envoi de la position cible initiale
                    Serial.println("[STRATEGY] GO!");
                }
                break;

            case STATE_GAME:
                // Fin de match ?
                if ((millis() - matchStartTime) >= MATCH_DURATION_MS) {
                    state = STATE_END;
                    mode_auto = false;
                    stopMotors();
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

            case STATE_TEST:
                Serial.println("[STRATEGY] Mode TEST");
                // Mode test - permet de tester les actionneurs/capteurs
                // TODO: Ajouter ici les séquences de test
                // Exemple: test servos, LEDs, TOFs, moteurs...
                break;

            case STATE_ERROR:
                mode_auto = false;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}
