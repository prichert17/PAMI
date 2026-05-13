#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "types.h"
#include "pami_com.h"
#include "actuators.h"
#include "tofs.h"

// Variables globales
extern float current_x, current_y, current_theta;
extern float target_x, target_y;
extern bool mode_auto;
extern bool debugMode;
extern SemaphoreHandle_t xPoseMutex;

static unsigned long matchStartTime = 0;
static unsigned long tiretteTime = 0;
static unsigned long delayStartTime = 0; // Pour tracker le timing du delay

static const uint8_t SERVO_END_POS_2 = 80; // Servo sur D15 (125° sinon)
static const uint8_t SERVO_END_POS_3 = 70;  // Servo sur D27
static const uint8_t SERVO_END_STEP = 1;
static const unsigned long SERVO_END_STEP_MS = 30;

// Constantes de timing selon le mode debug
static const unsigned long DEBUG_DELAY_MS = 3000;     // 3s en debug
static const unsigned long NORMAL_DELAY_MS = 85000;   // 85s en normal
static const unsigned long DEBUG_MATCH_DURATION_MS = 17000;   // 17s en debug
static const unsigned long NORMAL_MATCH_DURATION_MS = 99000;  // 99s en normal 

void Task_Strategy(void *pvParameters) {
    Serial.println("[STRATEGY] Démarré");

    // Init servos et moteurs
    initServos();
    initMotors();

    // Vérifier le mode au démarrage
    if (digitalRead(PIN_SW_MODE) == LOW) {
        state = STATE_MANUAL;
        Serial.println("[STRATEGY] Mode MANETTE activé");
    }
    
    // Vérifier le debug au démarrage (désactive les logs STM32)
    debugMode = (digitalRead(PIN_SW_DEBUG) == HIGH);
    if (debugMode) {
        Serial.println("[STRATEGY] Mode DEBUG: logs STM32 désactivés");
    }

    static unsigned long lastPrint = 0;
    static RobotState lastReportedState = STATE_WAIT;

    for (;;) {
        // --- Vérification continue des TOF pour détection proximité ---
        static bool motorsStoppedByTOF = false;
        static unsigned long tofStopTime = 0;
        
        bool tofProximityDetected = false;
        
        // Vérifier TOUTES les zones pour détecter un obstacle < 10cm
        for (int i = 0; i < 3 && !tofProximityDetected; i++) {
            for (int j = 0; j < 8; j++) {
                if (distances_tof[i][j] > 0 && distances_tof[i][j] < 100) { // <10cm = 100mm
                    tofProximityDetected = true;
                    break;
                }
            }
        }
        
        // Arrêt/Reprise sur détection TOF
        tofObstacleDetected = tofProximityDetected;
        
        if (tofProximityDetected && !motorsStoppedByTOF && state == STATE_GAME) {
            // ENTRER en arrêt
            motorsStoppedByTOF = true;
            tofStopTime = millis();
            stopMotors();
            Serial.println("[STRATEGY] ARRÊT: Obstacle détecté < 10cm!");
        }
        else if (motorsStoppedByTOF && state == STATE_GAME) {
            // SORTIR de l'arrêt si obstacle a disparu
            unsigned long timeStopped = millis() - tofStopTime;
            
            if (!tofProximityDetected) {
                // Obstacle disparu immédiatement
                motorsStoppedByTOF = false;
                sendPosition(target_x, target_y);
                Serial.println("[STRATEGY] Reprise du mouvement (obstacle disparu).");
            }
        }
        
        // --- Relecture continue des switchs ---
        bool swMode = (digitalRead(PIN_SW_MODE) == LOW);
        bool swDebug = (digitalRead(PIN_SW_DEBUG) == HIGH);
        
        // Mise à jour de la couleur d'équipe en temps réel (prioritaire en mode WAIT)
        if (state == STATE_WAIT) {
            teamColor = (digitalRead(PIN_SW_COLOR) == LOW) ? COLOR_BLUE : COLOR_YELLOW;
        }

        // Changement de mode en temps réel
        if (swMode && state != STATE_MANUAL) {
            state = STATE_MANUAL;
            setModeManuel();
            // Mode manuel : servo pin 15 → 80°, servo pin 19 → 0°
            setServoAngle(2, 80);  // Servo 2 = pin 15
            setServoAngle(3, 0);   // Servo 3 = pin 27
        } else if (!swMode && state == STATE_MANUAL) {
            // Retour en attente quand on désactive le switch manuel
            state = STATE_WAIT;
            // Mode auto : servo pin 15 → 0°, servo pin 27 → 0°
            setServoAngle(2, 0);  // Servo 2 = pin 15
            setServoAngle(3, 0);    // Servo 3 = pin 27
        }

        // Mise à jour du mode debug en temps réel
        debugMode = (digitalRead(PIN_SW_DEBUG) == HIGH);

        // --- Print continu de l'état (toutes les 500ms) ---
        if (millis() - lastPrint >= 500 || state != lastReportedState) {
            const char* stateStr = "?";
            switch(state) {
                case STATE_WAIT:   stateStr = "WAIT";   break;
                case STATE_DELAY:  stateStr = "DELAY";  break;
                case STATE_GAME:   stateStr = "GAME";   break;
                case STATE_END:    stateStr = "END";    break;
                case STATE_MANUAL: stateStr = "MANUAL"; break;
                case STATE_ERROR:  stateStr = "ERROR";  break;
            }
            Serial.println();
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
                    resetSTM32((int)ROBOT_INIT_X, (int)ROBOT_INIT_Y, (int)ROBOT_INIT_Z);
                    vTaskDelay(pdMS_TO_TICKS(500)); // Attendre le reset
                    
                    Serial.println("[STRATEGY] Passage en mode AUTO...");
                    setModeAuto();
                    vTaskDelay(pdMS_TO_TICKS(500)); // Attendre confirmation
                    
                    delayStartTime = millis();
                    delayInitDone = true;
                }
                
                // Vérifier si le délai est écoulé (3s en debug, 85s en normal)
                unsigned long delayMS = debugMode ? DEBUG_DELAY_MS : NORMAL_DELAY_MS;
                if ((millis() - delayStartTime) >= delayMS) {
                    state = STATE_GAME;
                    matchStartTime = millis();
                    setModeAuto(); // S'assurer d'être en mode auto au début du match                   

                    
                    delayInitDone = false; // Réinitialiser pour la prochaine fois
                }
                break;
            }

            case STATE_GAME: {
                static bool gameStartSent = false;
                
                // Envoyer la position cible une seule fois au début
                if (!gameStartSent) {
                    sendPosition(target_x, target_y);
                    gameStartSent = true;
                }
                
                // Fin de match ? (18s ou 99s selon debug)
                unsigned long matchDurationMS = debugMode ? DEBUG_MATCH_DURATION_MS : NORMAL_MATCH_DURATION_MS;
                if ((millis() - matchStartTime) >= matchDurationMS) {
                    state = STATE_END;
                    stopMotors();
                    gameStartSent = false; // Réinitialiser pour la prochaine partie
                    Serial.println("[STRATEGY] FIN");
                }
                //Serial.println("[STRATEGY] GO!");
                break;
            }

            case STATE_END:
                {
                    static bool endSeqInitDone = false;
                    static uint8_t servo1Angle = 0;
                    static uint8_t servo3Angle = 0;
                    static bool goingUp = true;
                    static unsigned long lastServoStep = 0;

                    if (!endSeqInitDone) {
                        stopMotors();
                        servo1Angle = 0;
                        servo3Angle = 0;
                        goingUp = true;
                        lastServoStep = millis();
                        setServoAngle(2, servo1Angle); // D15
                        //setServoAngle(3, servo3Angle); // D19
                        endSeqInitDone = true;
                        Serial.println("[STRATEGY] Séquence de fin de match démarrée");
                    }

                    if (millis() - lastServoStep >= SERVO_END_STEP_MS) {
                        lastServoStep = millis();

                        if (goingUp) {
                            if (servo1Angle < SERVO_END_POS_2) {
                                servo1Angle = min<uint8_t>(SERVO_END_POS_2, servo1Angle + SERVO_END_STEP);
                            }
                            /*
                            if (servo3Angle < SERVO_END_POS_3) {
                                servo3Angle = min<uint8_t>(SERVO_END_POS_3, servo3Angle + SERVO_END_STEP);
                            }*/

                            setServoAngle(2, servo1Angle); // D15
                            //setServoAngle(3, servo3Angle); // D19

                            //if (servo1Angle >= SERVO_END_POS_2 && servo3Angle >= SERVO_END_POS_3) {
                            if (servo1Angle >= SERVO_END_POS_2) {
                                goingUp = false;
                            }
                        } else {
                            if (servo1Angle > 0) {
                                servo1Angle = (servo1Angle > SERVO_END_STEP) ? (servo1Angle - SERVO_END_STEP) : 0;
                            }
                            /*
                            if (servo3Angle > 0) {
                                servo3Angle = (servo3Angle > SERVO_END_STEP) ? (servo3Angle - SERVO_END_STEP) : 0;
                            }
                            */

                            setServoAngle(2, servo1Angle); // D15
                            //setServoAngle(3, servo3Angle); // D19

                            if (servo1Angle == 0 && servo3Angle == 0) {
                                goingUp = true;
                            }
                        }
                    }
                }
                break;

            case STATE_MANUAL:
                // Mode manette - contrôle via Bluepad32
                break;



            case STATE_ERROR:
                mode_auto = false;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}
