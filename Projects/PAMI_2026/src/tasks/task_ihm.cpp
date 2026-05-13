#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FastLED.h>
#include "config.h"
#include "types.h"

// Configuration LED WS2812B
#define NUM_LEDS 1
#define LED_BRIGHTNESS 128

static CRGB leds[NUM_LEDS];
static bool blinkState = false;
static unsigned long lastBlinkTime = 0;
static const unsigned long BLINK_INTERVAL_MS = 250; // Clignotement à 2Hz

// Clignotement rapide en mode debug (0.7s on, 0.3s off)
static bool debugBlinkState = true;  // true = allumé, false = éteint
static unsigned long lastDebugBlinkTime = 0;
static const unsigned long DEBUG_BLINK_ON_MS = 700;   // 0.7s allumé
static const unsigned long DEBUG_BLINK_OFF_MS = 300;  // 0.3s éteint

void Task_IHM(void *pvParameters) {
    Serial.println("[IHM] Init...");
    
    
    // Init LED WS2812B
    FastLED.addLeds<WS2812B, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    Serial.println("[IHM] Prêt");

    for (;;) {
        // Gestion du clignotement normal
        unsigned long now = millis();
        if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
            blinkState = !blinkState;
            lastBlinkTime = now;
        }

        // Gestion du clignotement rapide pour le debug (0.7s on, 0.3s off)
        unsigned long debugInterval = debugBlinkState ? DEBUG_BLINK_ON_MS : DEBUG_BLINK_OFF_MS;
        if (now - lastDebugBlinkTime >= debugInterval) {
            debugBlinkState = !debugBlinkState;
            lastDebugBlinkTime = now;
        }

        // Logique de couleur LED (priorité décroissante)
        if (lowBattery) {
            // Rouge - Batterie faible (priorité max)
            leds[0] = CRGB::Red;
        }
        else if (tofObstacleDetected) {
            // Orange clignotant - Obstacle détecté par TOF
            leds[0] = blinkState ? CRGB(255, 64, 0) : CRGB::Black;
        }
        else if (state == STATE_TEST) {
            // Blanc - Mode test/debug
            leds[0] = CRGB::White;
        }
        else if (state == STATE_ERROR) {
            // Rouge clignotant - Erreur
            leds[0] = blinkState ? CRGB::Red : CRGB::Black;
        }
        else if (state == STATE_DELAY) {
            // Vert clignotant - Tirette déclenchée, attente 85s
            leds[0] = blinkState ? CRGB::Green : CRGB::Black;
        }
        else if (state == STATE_GAME) {
            // Vert fixe - Match en cours
            leds[0] = CRGB::Green;
        }
        else if (state == STATE_MANUAL) {
            // Magenta clignotant - Mode manette
            leds[0] = blinkState ? CRGB::Magenta : CRGB::Black;
        }
        else if (state == STATE_WAIT) {
            // Jaune ou Bleu selon équipe - Attente tirette
            // En mode debug, clignoter rapidement (0.7s on, 0.3s off)
            if (debugMode == false) {
                CRGB teamLed = (teamColor == COLOR_YELLOW) ? CRGB::Yellow : CRGB::Blue;
                leds[0] = debugBlinkState ? teamLed : CRGB::Black;
            } else {
                leds[0] = (teamColor == COLOR_YELLOW) ? CRGB::Yellow : CRGB::Blue;
            }
        }
        else if (state == STATE_END) {
            // Blanc - Match terminé
            leds[0] = CRGB::White;
        }
        else {
            // Par défaut éteint
            leds[0] = CRGB::Black;
        }

        FastLED.show();
        
        // TODO: Commandes servos
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz pour un clignotement fluide
    }
}
