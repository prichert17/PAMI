#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "types.h"
#include "pami_com.h"
#include "actuators.h"
#include "tofs.h"

// Dimensions du terrain
#define TERRAIN_SIZE_X 3000
#define TERRAIN_SIZE_Y 2000

// Variables globales
extern float current_x, current_y, current_theta;
extern float target_x, target_y;
extern bool mode_auto;
extern bool debugMode;
extern bool actionneurON;
extern SemaphoreHandle_t xPoseMutex;

// Fonction helper pour calculer les coordonnées symétriques selon la couleur
static float symmetrizeX(float x) {
    return (teamColor == COLOR_BLUE) ? (TERRAIN_SIZE_X - x) : x;
}

static float symmetrizeY(float y) {
    return y;
}

static unsigned long matchStartTime = 0;
static unsigned long tiretteTime = 0;
static unsigned long delayStartTime = 0; // Pour tracker le timing du delay

static const uint8_t SERVO_END_POS_2 = 80; // Servo sur D15 (125° sinon)
static const uint8_t SERVO_END_POS_3 = 70;  // Servo sur D27
static const uint8_t SERVO_END_STEP = 1;
static const unsigned long SERVO_END_STEP_MS = 30;

struct RouteWaypoint {
    float x;
    float y;
    const char* label;
};

static const RouteWaypoint matchRoute_base[] = {
    {550.0f, 600.0f, "Tout droit"},
    {1200.0f, 600.0f, "Tourne"},
};

static const uint8_t MATCH_ROUTE_COUNT_BASE = sizeof(matchRoute_base) / sizeof(matchRoute_base[0]);

// Tableau des waypoints ajustés selon la couleur (dimensionné pour ajouter un waypoint bonus)
static RouteWaypoint matchRoute[10];

static uint8_t currentRouteIndex = 0;
static uint8_t actualRouteCount = MATCH_ROUTE_COUNT_BASE; // Nombre actuel de waypoints
static bool bonusWaypointAdded = false; // Pour tracker si le bonus a déjà été ajouté
static bool routeFinished = false;

// Fonction pour initialiser les waypoints en fonction de la couleur
static void initializeRoute() {
    for (int i = 0; i < MATCH_ROUTE_COUNT_BASE; i++) {
        matchRoute[i].x = symmetrizeX(matchRoute_base[i].x);
        matchRoute[i].y = symmetrizeY(matchRoute_base[i].y);
        matchRoute[i].label = matchRoute_base[i].label;
    }
    
    // Adapter le deuxième waypoint selon la couleur
    if (MATCH_ROUTE_COUNT_BASE > 1) {
        matchRoute[1].y = (teamColor == COLOR_BLUE) ? 575.0f : 550.0f;
    }
    
    actualRouteCount = MATCH_ROUTE_COUNT_BASE;
    bonusWaypointAdded = false;
    Serial.printf("[STRATEGY] Route initialisée pour couleur %s\n", 
                  (teamColor == COLOR_BLUE) ? "BLEU" : "JAUNE");
    if (teamColor == COLOR_BLUE) {
        for (int i = 0; i < actualRouteCount; i++) {
            Serial.printf("[STRATEGY] Waypoint %d: (%.0f, %.0f) - %s\n", 
                          i, matchRoute[i].x, matchRoute[i].y, matchRoute[i].label);
        }
    }
}

static void addBonusWaypoint() {
    if (!bonusWaypointAdded && actualRouteCount < 10) {
        // Ajouter le waypoint bonus symétrisé selon la couleur
        matchRoute[actualRouteCount].x = symmetrizeX(1800.0f);
        matchRoute[actualRouteCount].y = symmetrizeY(600.0f);
        matchRoute[actualRouteCount].label = "Bonus obstacle";
        
        Serial.printf("[STRATEGY] ⚠ Waypoint BONUS détecté! Ajout: (%.0f, %.0f)\n",
                      matchRoute[actualRouteCount].x, matchRoute[actualRouteCount].y);
        
        actualRouteCount++;
        bonusWaypointAdded = true;
    }
}

static void sendCurrentWaypoint() {
    if (currentRouteIndex < actualRouteCount) {
        sendPosition(matchRoute[currentRouteIndex].x, matchRoute[currentRouteIndex].y);
        Serial.printf("[STRATEGY] Cible %u/%u: %s (X=%.1f, Y=%.1f)\n",
                      currentRouteIndex + 1,
                      actualRouteCount,
                      matchRoute[currentRouteIndex].label,
                      matchRoute[currentRouteIndex].x,
                      matchRoute[currentRouteIndex].y);
    }
}

// Constantes de timing selon le mode debug
static const unsigned long DEBUG_DELAY_MS = 5000;     // 5s en debug
static const unsigned long NORMAL_DELAY_MS = 85000;   // 85s en normal
static const unsigned long DEBUG_MATCH_DURATION_MS = 19000;   // 19s en debug
static const unsigned long NORMAL_MATCH_DURATION_MS = 99000;  // 99s en normal 

void Task_Strategy(void *pvParameters) {
    Serial.println("[STRATEGY] Démarré");

    // Init servos et moteurs
    initServos();
    initMotors();
    
    // Initialiser la route en fonction de la couleur
    initializeRoute();

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
        
        // Vérifier SEULEMENT le capteur TOF central (indice 2) sur les lignes du milieu/bas
        int centralSensor = 2; // TOF3 = Centre
        for (int j = 0; j < 8; j++) {
            if (distances_tof[centralSensor][j] > 0 && distances_tof[centralSensor][j] < 100) { // <10cm = 100mm
                tofProximityDetected = true;
                break;
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
                    
                    // Vérification minimale du reset
                    xSemaphoreTake(xPoseMutex, portMAX_DELAY);
                    float verifyX = current_x;
                    float verifyY = current_y;
                    xSemaphoreGive(xPoseMutex);
                    
                    if (abs(verifyX - ROBOT_INIT_X) < 10 && abs(verifyY - ROBOT_INIT_Y) < 10) {
                        Serial.printf("[STRATEGY] ✓ Reset OK: (%.0f, %.0f)\n", verifyX, verifyY);
                    } else {
                        Serial.printf("[STRATEGY] ⚠ Reset MISMATCH: got (%.0f, %.0f), expected (%.0f, %.0f)\n", 
                                      verifyX, verifyY, ROBOT_INIT_X, ROBOT_INIT_Y);
                    }
                    
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
                    currentRouteIndex = 0;
                    routeFinished = false;
                    bonusWaypointAdded = false; // Réinitialiser pour la partie
                    actualRouteCount = MATCH_ROUTE_COUNT_BASE; // Réinitialiser le nombre de waypoints
                    sendCurrentWaypoint();
                    gameStartSent = true;
                }

                if (!routeFinished && checkPosition()) {
                    // Vérifier si on vient de terminer le 1er waypoint (index 0)
                    if (currentRouteIndex == 0) {
                        // Vérifier détection TOF du bas (zones 0-3 du capteur central)
                        bool obstacleDetectedAtWaypoint0 = false;
                        int centralSensor = 2; // TOF central
                        for (int j = 0; j < 4; j++) {  // Zones du bas
                            if (distances_tof[centralSensor][j] > 0 && distances_tof[centralSensor][j] < 100) {
                                obstacleDetectedAtWaypoint0 = true;
                                break;
                            }
                        }
                        
                        if (obstacleDetectedAtWaypoint0) {
                            addBonusWaypoint();
                        }
                    }
                    
                    if (currentRouteIndex + 1 < actualRouteCount) {
                        currentRouteIndex++;
                        sendCurrentWaypoint();
                    } else {
                        routeFinished = true;
                        stopMotors();
                        state = STATE_END;
                        gameStartSent = false;
                        Serial.println("[STRATEGY] Route terminee");
                        break;
                    }
                }
                
                // Fin de match ? (18s ou 99s selon debug)
                unsigned long matchDurationMS = debugMode ? DEBUG_MATCH_DURATION_MS : NORMAL_MATCH_DURATION_MS;
                if ((millis() - tiretteTime) >= matchDurationMS) {
                    state = STATE_END;
                    stopMotors();
                    gameStartSent = false; // Réinitialiser pour la prochaine partie
                    routeFinished = false;
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
                    if (actionneurON){
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
