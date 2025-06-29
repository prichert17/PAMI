#include <Arduino.h>
#include <TOF_2.h>
#include <asser_position.h>
#include <SoftTimers.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
SoftTimer timer_main;
SoftTimer toggle_timer;
const long interval = 1500;
extern VL53L5CX_ResultsData measurementData;
#define TIRETTE 12
#define STRAT   13

bool couleur;  //on dit que true = jaune et false = bleu

extern bool presence_obstacle;

extern Terrain ter;


enum State {RUN,STOP,STOP_EMERGENCE,CONTOURNEMENT}; //on définit les états possibles du robot
State current_state = RUN;
int numberOfCoordinates = 0;



void setup(){
    
    Serial.begin(115200);
    SerialBT.begin("PAMI_BT");    // nom qui apparaîtra sur Windows
    pinMode(STRAT, INPUT_PULLUP); // switch pour choisir la couleur
    Serial.print("Strat = ");
    Serial.println(digitalRead(STRAT));
    SerialBT.print("Strat = ");
    SerialBT.println(digitalRead(STRAT));
    pinMode(TIRETTE, INPUT_PULLUP);
    //while (digitalRead(TIRETTE) == HIGH) {    //on attend que la tirette soit en position 0
      //Serial.print("TIRETTE = ");
      //Serial.println(digitalRead(TIRETTE));
      //delay(200); // Limite l'affichage pour éviter de saturer la console
    //}
    if (digitalRead(STRAT) == HIGH) { //si le switch est sur 1, on dit que le terrain est jaune
      couleur = true; //on dit que le terrain est jaune
    }else{ //si le switch est sur 0, on dit que le terrain est bleu
      couleur = false; //on dit que le terrain est bleu
    }
    (digitalRead(STRAT) == HIGH) ? ter.jaune() : ter.bleu(); //on dit que 1 = jaune et 0 = bleu
    //fonction qui rajoute les coordonnées de l'estrade aux obstacles

    
    if (couleur == true){
      Serial.println("Terrain Jaune");
      SerialBT.println("Terrain Jaune");
      ter.jaune();
    }else if (couleur == false){
      Serial.println("Terrain Bleu");
      SerialBT.println("Terrain Bleu");
      ter.bleu();
    }
    tof_setup_spark(); //set_up de TOF et ultrasons 
    asser_position_setup(); // set _up de l'asservissement de position
      
    numberOfCoordinates = ter.getNumberOfCoordinates(); //on récupère le nombre de coordonnées (utile dans l'évitement d'obstacles)
    timer_main.setTimeOutTime(100000); // on initialise le timer à 100s
    
    timer_main.reset(); // le timer commence ici (enleve pas cette ligne)
    toggle_timer.reset();
    
  }

void loop() {
    
    if(timer_main.getElapsedTime() > 3500){ //après 85 secondes, on commence notre strategie
      tof_loop_spark();

      current_state = (timer_main.hasTimedOut())? STOP :current_state ; //si on arrive vers 100s on arrête tout, sinon current_state reste inchangé      
      switch (current_state){
          case RUN: {
            asser_position_loop(true);
            bool libre = ter.arret_urgence();
            current_state  = (libre)? STOP_EMERGENCE:RUN;             
            
            if (presence_obstacle) {
              current_state = CONTOURNEMENT;
            }

            break;
          }
          case (STOP):{
            asser_position_loop(false);

          break;
          }
          case (CONTOURNEMENT):{    //on met temporairement le robot en pause avant d'avoir trouvé un nouveau chemin (fonction inutilisée, tout est fait dans le code du tof)
            asser_position_loop(false);
            //Serial.println("CONTOURNEMENT");

            if (!presence_obstacle) {
              current_state = RUN;
            }

          break;
          }
          case (STOP_EMERGENCE):
          {
            asser_position_loop(false);
            //Serial.println("STOP EMERGENCE");
            if (!ter.arret_urgence()) {     //si l'obstacle n'est plus là, on retourne à l'état RUN
              current_state = RUN; break; 
            }   
          
            break;
          }
          default:
          {

   
          break; 
          }
        }
  }
  
  
  
}
