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

void Task_IHM(void *pvParameters) {
    Serial.println("[IHM] Init...");
    
    
    // Init LED WS2812B
    FastLED.addLeds<WS2812B, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    Serial.println("[IHM] Prêt");

    for (;;) {
        // Gestion du clignotement
        unsigned long now = millis();
        if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
            blinkState = !blinkState;
            lastBlinkTime = now;
        }

        // Logique de couleur LED (priorité décroissante)
        if (lowBattery) {
            // Rouge - Batterie faible (priorité max)
            leds[0] = CRGB::Red;
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
        else if (state == STATE_WAIT) {
            // Jaune ou Bleu selon équipe - Attente tirette
            leds[0] = (teamColor == COLOR_YELLOW) ? CRGB::Yellow : CRGB::Blue;
        }
        else if (state == STATE_END) {
            // Vert fixe - Match terminé
            leds[0] = CRGB::Green;
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
