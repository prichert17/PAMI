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
static unsigned long delayStartTime = 0; // Pour tracker le timing du delay
// PAMI delay avant de démarrer : réduit à quelques secondes pour test (sera 85000ms plus tard)
static const unsigned long PAMI_DELAY_MS = 3000; 

void Task_Strategy(void *pvParameters) {
    Serial.println("[STRATEGY] Démarré");

    // Init servos et moteurs
    initServos();
    initMotors();

    // Vérifier le mode au démarrage (priorité: manette > debug > normal)
    if (digitalRead(PIN_SW_MODE) == LOW) {
        state = STATE_MANUAL;
        Serial.println("[STRATEGY] Mode MANETTE activé");
    }
    else if (digitalRead(PIN_SW_DEBUG) == LOW) {
        state = STATE_TEST;
        Serial.println("[STRATEGY] Mode TEST activé");
    }

    static unsigned long lastPrint = 0;
    static RobotState lastReportedState = STATE_WAIT;

    for (;;) {
        // --- Relecture continue des switchs ---
        bool swMode = (digitalRead(PIN_SW_MODE) == LOW);
        bool swDebug = (digitalRead(PIN_SW_DEBUG) == LOW);
        
        // Mise à jour de la couleur d'équipe en temps réel (prioritaire en mode WAIT)
        if (state == STATE_WAIT) {
            teamColor = (digitalRead(PIN_SW_COLOR) == LOW) ? COLOR_BLUE : COLOR_YELLOW;
        }

        // Changement de mode en temps réel
        if (swMode && state != STATE_MANUAL) {
            state = STATE_MANUAL;
            mode_auto = false;
            stopMotors();
        } else if (!swMode && state == STATE_MANUAL) {
            // Retour en attente quand on désactive le switch manuel
            state = STATE_WAIT;
        }

        if (swDebug && state != STATE_TEST && state != STATE_MANUAL) {
            state = STATE_TEST;
        } else if (!swDebug && state == STATE_TEST) {
            state = STATE_WAIT;
        }

        // --- Print continu de l'état (toutes les 500ms) ---
        if (millis() - lastPrint >= 500 || state != lastReportedState) {
            const char* stateStr = "?";
            switch(state) {
                case STATE_WAIT:   stateStr = "WAIT";   break;
                case STATE_DELAY:  stateStr = "DELAY";  break;
                case STATE_GAME:   stateStr = "GAME";   break;
                case STATE_END:    stateStr = "END";    break;
                case STATE_MANUAL: stateStr = "MANUAL"; break;
                case STATE_TEST:   stateStr = "TEST";   break;
                case STATE_ERROR:  stateStr = "ERROR";  break;
            }
            Serial.printf("[STRAT] state=%s SW_MODE=%d SW_DEBUG=%d auto=%d\n",
                          stateStr, swMode ? 1 : 0, swDebug ? 1 : 0, mode_auto ? 1 : 0);
            lastPrint = millis();
            lastReportedState = state;
        }

        switch (state) {
            case STATE_WAIT: {
                // Attente tirette : on s'assure qu'elle soit d'abord insérée (LOW) puis retirée (HIGH)
                static bool tirette_mise = false;
                if (digitalRead(PIN_TIRETTE) == LOW) {
                    tirette_mise = true;
                } else if (digitalRead(PIN_TIRETTE) == HIGH && tirette_mise) {
                    tiretteTime = millis();
                    state = STATE_DELAY;
                    Serial.println("[STRATEGY] Tirette retirée! Attente du chrono...");       
                }
                break;
            }

            case STATE_DELAY: {
                // Machine à état pour le délai d'attente avant le départ
                static bool delayInitDone = false;
                
                // À la première entrée dans STATE_DELAY
                if (!delayInitDone) {
                    Serial.println("[STRATEGY] Reset STM32...");
                    resetSTM32();
                    vTaskDelay(pdMS_TO_TICKS(500)); // Attendre le reset
                    
                    Serial.println("[STRATEGY] Passage en mode AUTO...");
                    setModeAuto();
                    vTaskDelay(pdMS_TO_TICKS(500)); // Attendre confirmation
                    
                    delayStartTime = millis();
                    delayInitDone = true;
                }
                
                // Vérifier si le délai est écoulé (3s pour test, 85s en compétition)
                if ((millis() - delayStartTime) >= PAMI_DELAY_MS) {
                    state = STATE_GAME;
                    matchStartTime = millis();
                    mode_auto = true;
                    
                    // Envoyer la première position cible
                    sendPosition(target_x, target_y);
                    Serial.println("[STRATEGY] GO!");
                    
                    delayInitDone = false; // Réinitialiser pour la prochaine fois
                }
                break;
            }

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
                break;

            case STATE_MANUAL:
                // Mode manette - contrôle via Bluepad32
                break;

            case STATE_TEST:
                // Mode test
                break;

            case STATE_ERROR:
                mode_auto = false;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}
