/**
 * PAMI ESP32 - Main Entry Point
 * Architecture: FreeRTOS avec séparation par Tâches
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Inclusion des configurations et définitions partagées
#include "config.h"
#include "types.h"
#include "tasks.h" // Contient les prototypes des fonctions Task_...
#include "pami_com.h" // Prototypes de fonctions de communication avec STM32
#include "tofs.h" // Prototypes de fonctions liées aux capteurs ToF

// --- VARIABLES GLOBALES SYSTÈME ---
// Définition du Mutex (déclaré extern dans types.h)
SemaphoreHandle_t xPoseMutex = NULL;

void setup() {
    // 1. Initialisation Debug
    Serial.begin(115200);
    // Petit délai de sécurité pour que le port série s'ouvre bien
    vTaskDelay(500 / portTICK_PERIOD_MS); 
    Serial.println("\n--- PAMI ESP32: System Booting ---");

    // 2. Initialisation des Pins Globaux (si nécessaire ici)
    // Note : Les pins spécifiques sont init dans les setups de chaque tâche,
    // mais on peut init les inputs communs ici.
    pinMode(PIN_TIRETTE, INPUT_PULLUP);

    // 3. Création des Objets de Synchronisation
    xPoseMutex = xSemaphoreCreateMutex();
    if (xPoseMutex == NULL) {
        Serial.println("!!! ERREUR CRITIQUE : Mutex creation failed !!!");
        while(1); // On bloque tout si pas de mémoire
    }

    // 4. Création des Tâches (Multitasking)
    
    // Tâche COMMS (Priorité 5 - HAUTE) -> Core 1
    // Doit être réactive pour ne pas rater de caractères UART
    xTaskCreatePinnedToCore(
        Task_Comms,   "Comms",    4096, NULL, 5, NULL, 1
    );

    // Tâche STRATEGY (Priorité 4 - MOYENNE) -> Core 1
    // Le cerveau du robot, tourne sur le même coeur que la Comms pour accès rapide cache
    xTaskCreatePinnedToCore(
        Task_Strategy,"Strategy", 4096, NULL, 4, NULL, 1
    );

    // Tâche IHM (Priorité 1 - BASSE) -> Core 0
    // Gère les servos et LEDs. Core 0 est moins chargé (gère le Wifi s'il y en a)
    xTaskCreatePinnedToCore(
        Task_IHM,     "HMI",      2048, NULL, 1, NULL, 0
    );

    // Tâche TOFS (Priorité 2 - MOYENNE/BASSE) -> Core 0
    // Lecture I2C (peut être bloquante ou lente), on la sépare du Core 1
    xTaskCreatePinnedToCore(
        Task_Tofs,    "ToFs",     4096, NULL, 2, NULL, 0
    );

    Serial.println("--- PAMI ESP32: RTOS Scheduler Started ---");
}

void loop() {
    // Dans FreeRTOS avec Arduino, le loop tourne dans une tâche de priorité 1.
    // Comme nous n'en avons pas besoin, on supprime cette tâche pour libérer la RAM.
    vTaskDelete(NULL);
}