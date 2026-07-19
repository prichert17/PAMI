# Projet PAMI (2025-2026) - Eurobot

Ce dépôt documente le projet PAMI (Petit Actionneur Motorisé Indépendant) sur lequel j'ai travaillé pendant un an, dans le cadre de la coupe Eurobot 2026. Il centralise l'ensemble de la conception matérielle (deux PCB, structure mécanique) et logicielle du robot. 

<p align="center">
  <img src="20260719_123606.jpg" width="80%">
  <br>
  <em>Légende : Photo du robot PAMI assemblé</em>
</p>

<p align="center">
  <img src="image.png" width="80%">
  <br>
  <em>Légende : Rendu 3D du robot avec ses bras d'actionnement</em>
</p>


## Codes principaux

Pour mieux comprendre l'architecture du système, voici les trois éléments importants du dépôt :

*   **[`PAMI_2026/`](./Projects/PAMI_2026)** : Contient le code haut niveau qui est exécuté sur le microcontrôleur **ESP32**.
*   **[`Projects/Asservissement_G4/`](./Projects/Asservissement_G4)** : Contient tout le code de l'asservissement, de l'odométrie et le contrôle des moteurs. Ce code tourne sur la carte **NUCLEO-G431KB** (STM32).
*   **[`Projects/IHM_PAMI_2026.py`](./Projects/IHM_PAMI_2026.py)** : Une interface homme-machine (IHM) développée en Python qui permet de lire et visualiser en temps réel les coordonnées et l'état du PAMI lors des phases de tests.

---

## Architecture Bas Niveau : PCB Principal (Contrôle Moteurs & Alimentation)

Le robot dispose de deux cartes électroniques (PCB). La partie bas niveau, gérée par le PCB principal s'occupe de l'alimentation générale et de la motricité du robot. Le PAMI est assez compact : ses dimensions finales sont de 10.5cm par 19cm avec ses bras repliés (et 24cm bras dépliés), ce qui a imposé des contraintes sur le routage du circuit et le choix des composants.

### 1. Gestion de la Batterie et Alimentation
Le robot est alimenté par une unique cellule de batterie LiFePO4 au format 26650. Le circuit d'alimentation a subi plusieurs modifications suite à des problèmes rencontrés lors du développement :

*   **Circuit de Charge :** La charge est gérée directement sur le PCB par un composant dédié à ce type de batterie, le **CN3058E**, avec des LEDs indicatrices (Charge en cours / Charge terminée).
*   **Sécurité Batterie :** La batterie est surveillée par un IC de protection **HY2112**, associé à un double MOSFET N-Channel (FS8205A) pour couper le circuit en cas de problème.
*   **Régulation de tension :** La tension de la cellule LiFePO4 pouvant monter à 3.6V (ce qui est trop élevé pour l'ESP32 et certains capteurs), j'ai opté pour un convertisseur **Buck-Boost (TPS63000)** au lieu d'un simple circuit Boost. Il assure un 3.3V parfaitement stable en abaissant ou élevant la tension de la batterie selon les besoins, pour un courant jusqu'à 1A.
*   **Le problème des appels de courant :** Un problème est survenu avec l'alimentation des servomoteurs. Leurs appels de courant créaient d'énormes chutes de tension (la tension de la batterie chutait jusqu'à 2.4V pendant un court instant). Le robot ne parvenait pas à se lever de manière autonome, la tension de la batterie passait en dessous du seuil limite du circuit boost 6V. La cause : la résistance interne du boitier de la batterie 26650 et des pistes de cuivre trop fines (1.5mm) qui ne supportaient pas les pics d'intensité très élevés (pouvant monter jusqu'à 3A sur le rail 6V, soit quasiment 6A en sortie de la batterie). Le problème a été résolu en soudant un condensateur de 2200µF (10V) directement à l'entrée du boost 6V pour agir comme une réserve d'énergie, ce qui diminuait fortement la chute de tension.

### 2. Odométrie et Asservissement (NUCLEO-G431KB)
La carte intègre un microcontrôleur STM32 (NUCLEO-G431KB) programmé en C++ pour gérer la dynamique temps-réel du robot. Un travail conséquent a été réalisé sur la fiabilité des déplacements et la prévention du patinage.

*   **Odométrie Haute Fréquence (500Hz) :** La position exacte du robot (X, Y, Angle) est calculée 500 fois par seconde. Les encodeurs incrémentiels des roues renvoient des signaux en quadrature décodés de manière purement matérielle par les Timers du STM32. Le code gère les débordements de compteurs (overflow des timers 16 bits) pour assurer un suivi sans erreur.
*   **Asservissement PID en 2 Phases (100Hz) :** Le contrôle de la trajectoire s'effectue via une boucle PID (Proportionnel, Intégral, Dérivé) actualisée à 100Hz. Pour rejoindre une cible, l'algorithme utilise une logique en deux temps :
    1. **Rotation pure :** Le robot pivote sur lui-même pour s'aligner parfaitement vers sa cible.
    2. **Avance avec correction :** Une fois aligné, il avance en ligne droite tout en maintenant son angle de manière active. Si le robot doit dévier fortement, la commande linéaire est automatiquement réduite pour privilégier la rotation et garder un mouvement fluide.
*   **Traction Control & Anti-Patinage :** Les robots de la coupe Eurobot sont sujets au patinage (poussière, accélération forte), ce qui fausse instantanément l'odométrie. J'ai donc un algorithme de détection de glissement : si la différence de vitesse mesurée entre la roue gauche et droite devient anormalement élevée, le STM32 détecte la perte d'adhérence et réduit la commande PWM envoyée aux moteurs.
*   **Performances :** À pleine vitesse, les moteurs tournent à environ 5 tours/seconde, propulsant le PAMI à 68 cm/s (2.4 km/h). La dérive odométrique pure a été mesurée à environ 2cm d'erreur pour 1 mètre parcouru en ligne droite.
*   **Communication Inter-Cartes :** Le STM32 transmet continuellement ses données d'odométrie et son état au cerveau central (l'ESP32) via une liaison série (UART), dont le protocole est détaillé dans `pami_com.h`.

<p align="center">
  <img src="diagramme_asservissement.png" width="90%">
  <br>
  <em>Légende : Schéma de l'architecture d'asservissement du STM32.</em>
</p>

### Schématique du PCB Principal

Voici la schématique électrique du PCB principal (Moteurs & Alimentation) montrant l'interconnexion entre la NUCLEO, les drivers moteurs, le circuit de charge de la batterie et le régulateur Buck-Boost :

<p align="center">
  <img src="image-1.png" width="100%">
  <br>
  <em>Légende : Schématique détaillée du PCB d'alimentation</em>
</p>

<p align="center">
  <img src="Pasted image 20251218120624.png" width="100%">
  <br>
  <em>Légende : Routage du PCB d'alimentation</em>
</p>