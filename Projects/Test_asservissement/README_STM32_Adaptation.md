# Adaptation du code pour STM32 L432KC

## Changements effectués

### 1. Configuration PlatformIO (platformio.ini)
- **Avant** : `platform = espressif32`, `board = esp32dev`
- **Après** : `platform = ststm32`, `board = nucleo_l432kc`
- Framework Arduino maintenu pour compatibilité

### 2. Pins utilisées (adaptées au STM32 L432KC)
| Fonction | ESP32 Original | STM32 L432KC | Timer/Port |
|----------|---------------|--------------|------------|
| Moteur 1 PWM 1 | 25 | PA6 | TIM3_CH1 |
| Moteur 1 PWM 2 | 26 | PA7 | TIM3_CH2 |
| Encodeur 1 A   | 27 | PA8 | TIM1_CH1 |
| Encodeur 1 B   | 14 | PA9 | TIM1_CH2 |
| Moteur 2 PWM 1 | 32 | PA10 | TIM1_CH3 |
| Moteur 2 PWM 2 | 33 | PA11 | TIM1_CH4 |
| Encodeur 2 A   | 35 | PB3 | TIM2_CH2 |
| Encodeur 2 B   | 34 | PB4 | TIM3_CH1 |

### 3. Modifications logicielles
- Ajout de `INPUT_PULLUP` pour les pins d'encodeur (stabilité du signal)
- Utilisation de `constrain()` dans `set_vitesse()` pour la sécurité
- Cast explicite en `(int)` pour `analogWrite()`

### 4. Fonctionnalités conservées
- ✅ Contrôle PWM des moteurs
- ✅ Lecture des encodeurs avec interruptions
- ✅ Calcul PID
- ✅ Calcul de position (odométrie)
- ✅ Communication série (115200 baud)

## Vérifications à effectuer

1. **Connectiques** :
   - Vérifier que les pins choisies sont accessibles sur la carte NUCLEO-L432KC
   - S'assurer que les timers ne sont pas en conflit

2. **Compilation** :
   - Installer PlatformIO si nécessaire
   - Compiler avec `platformio run` ou via VS Code
   
3. **Test** :
   - Vérifier la génération PWM sur les pins moteurs
   - Tester les interruptions des encodeurs
   - Valider la communication série

## Notes importantes

- Le STM32 L432KC fonctionne à 3.3V (vs ESP32 aussi 3.3V) - pas de problème de compatibilité
- Fréquence d'horloge différente mais `millis()` reste précis
- Les fonctions Arduino de base (`pinMode`, `analogWrite`, `attachInterrupt`) sont supportées