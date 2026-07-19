# Adaptation du code pour STM32 L432KC

## Changements effectués

### 1. Configuration PlatformIO (platformio.ini)
- **Avant** : `platform = espressif32`, `board = esp32dev`
- **Après** : `platform = ststm32`, `board = nucleo_l432kc`
- Framework Arduino maintenu pour compatibilité

### 2. Pins utilisées (adaptées au STM32 L432KC)
| Fonction | ESP32 Original | STM32 L432KC | Commentaire |
|----------|---------------|--------------|-------------|
| Moteur gauche PWM 1 | 25 | PA8 | TIM1_CH1 |
| Moteur gauche PWM 2 | 26 | PA9 | TIM1_CH2 |
| Moteur droit PWM 1 | 32 | PA10 | TIM1_CH3 |
| Moteur droit PWM 2 | 33 | PA11 | TIM1_CH4 |
| Encodeur gauche A | 27 | PA0 | GPIO avec interrupt |
| Encodeur gauche B | 14 | PA1 | GPIO avec interrupt |
| Encodeur droit A | 35 | PA6 | GPIO avec interrupt |
| Encodeur droit B | 34 | PA7 | GPIO avec interrupt |
| UART TX | - | PA2 | USART2 → Serial2 |
| UART RX | - | PA3 | USART2 → Serial2 |

### 3. Modifications majeures

#### **Encodeurs en quadrature (X4)**
- **Avant** : Interruptions simples sur un seul signal par encodeur
- **Après** : Décodage quadrature complet X4 avec interruptions sur A et B
- **Avantage** : 4x plus de résolution, détection de sens automatique
- **Logique** : Table de décodage quadrature dans ISR `enc1_ISR()` et `enc2_ISR()`

#### **Contrôle PID par timer matériel**  
- **Avant** : Loop principale avec `millis()` 
- **Après** : ISR timer matériel `ctrlISR()` à 1kHz (TIM6)
- **Avantage** : Précision temporelle parfaite, CPU libéré

#### **PWM moteurs**
- **Signature conservée** : `set_vitesse(float cmd1, float cmd2)` identique
- **Pins** : Regroupées sur TIM1 (PA8..PA11) pour cohérence  
- **Fréquence** : Prêt pour 20kHz (actuellement fréquence par défaut Arduino)

#### **Communication UART**
- **Ajout** : `Serial2` sur PA2/PA3 pour communication ESP32
- **Format** : CSV `speed1,speed2,X,Y,theta` toutes les 500ms
- **Debug** : `Serial` USB maintenu

### 4. Fonctionnalités conservées
- ✅ Contrôle PWM sign-magnitude des moteurs (fonction `set_vitesse` inchangée)
- ✅ Régulation PID vitesse (mêmes paramètres kp, ki, kd)
- ✅ Calcul de position (odométrie) 
- ✅ Communication série (ajout UART vers ESP32)
- ✅ Variables robot (robotX, robotY, robotTheta) 

### 5. Améliorations apportées
- **Résolution encodeurs** : X4 vs simple comptage
- **Précision temporelle** : Timer matériel vs polling
- **Robustesse** : Décodage quadrature avec table d'états
- **Performance** : ISR dédiées, sections critiques optimisées
- **Communication** : Dual Serial (USB debug + UART vers ESP32)

## Tests d'acceptation réalisés
1. ✅ **Compilation** : Code compile sans erreur pour `nucleo_l432kc`
2. ✅ **Signature** : `set_vitesse(cmd1, cmd2)` inchangée
3. ✅ **Encodeurs** : Décodage quadrature X4 implémenté  
4. ✅ **PID** : Logique et paramètres conservés
5. ✅ **Timer** : ISR 1kHz pour contrôle précis

## Utilisation

### Compilation
```bash
platformio run
```

### Upload  
```bash
platformio run --target upload
```

### Monitoring
```bash 
platformio device monitor --baud 115200
```

### Communication ESP32
- **Port** : USART2 (PA2/PA3) à 115200 baud
- **Format** : `speed1,speed2,X,Y,theta\n`
- **Fréquence** : 2Hz (toutes les 500ms)

## Notes techniques

### Pinout NUCLEO-L432KC
- **PA8-PA11** : TIM1 CH1-4 (PWM moteurs)
- **PA0-PA1** : GPIO interrupt (encodeur gauche)  
- **PA6-PA7** : GPIO interrupt (encodeur droit)
- **PA2-PA3** : USART2 TX/RX (vers ESP32)
- **TIM6** : Timer contrôle PID

### Fréquences
- **PID** : 1 kHz (1ms précis)
- **Debug/UART** : 2 Hz (500ms)
- **PWM** : ~1 kHz (extensible 20 kHz)

### Résolution encodeurs
- **Avant** : 1400 CPR simple
- **Après** : 1400 × 4 = 5600 CPR (quadrature X4)