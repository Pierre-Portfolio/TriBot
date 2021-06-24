#include <SoftPWM.h>
#include <HX711.h>
#include <assert.h>


/////// DEBUT REGLAGES


#define PIN_POID_DATA 15
#define PIN_POID_CLOCK 14


//Pont en H A - CHenille - PID & INTERRUPT
//moteur 1
#define D1A 6
#define D2A 7
#define PIN_HALL_MOTOR1 2 //INTERRUPT
//moteur 2
#define D3A 8
#define D4A 9
#define PIN_HALL_MOTOR2 3 //INTERRUPT

//Pont en H B - Poubelles - PWM Fixe & lecture analogue
//moteur 3
#define D1B 10
#define D2B 11
#define PIN_HALL_MOTOR3 4 //NOT INTERRUPT
//moteur 4
#define D3B 12
#define D4B 13
#define PIN_HALL_MOTOR4 5 //NOT INTERRUPT


//nombre de point utilisé pour le calcul de vitesse stabilisé via moyenne, 4 semble fonctionner pas mal sans introduire trop de latence
#define AVG 4

//distance de test
#define DISTANCE_MM 200


//temps en MS d'attente pour que les moteurs cessent de tourner. IMPORTANT : Le moteur doit toujours être à l'arrêt avant de changer de direction, sinon grille pont en H. Ce temos doit suffir pour qu'un moteur qui tourne soit (a peu près) arrêté
#define WAIT_STOP_MS 250

#define STALL_FACTOR 2.0f //Facteur de temps entre la durée de la dernière interruption et le point à partir du quel on considère que le moteur est 'calé' et donc qu'il ne tourne plus (Et ne passe plus devant le capteur, donc on ne peut plus lire ça vitesse)

#define MINIMUM_INTERUPT_MICROS 2000 //Temps minimum possible entre deux interuptions (Un demi tour de l'axe moteur). (Pour éviter les signaux parasite) 2000 microsecondes -> 15000 RPM, peu de chance que le moteur aille plus vite que ça

//interval entre les mise à jour de la puissance par PID en MILLISECONDES, 30ms -> 33hz semble bon
#define PID_LOOP_MS 30

//rayon de la chenille
#define WHEEL_RADIUS_MM (130/2)


//Facteur PID google it
#define kp (1.4*0.55f)
#define ki (2.53*0.50f)
#define kd (0.1395*0.35f)

/*
 *  OLD
#define kp 0.25f
#define ki 0.1f
#define kd 0.85f
 */


////// FIN REGLAGES
////// DEBUG LOGIQUE

typedef struct{
  //Les interruption simulées sont prises en compte dans ces valeurs
  unsigned long long time_last_pass; //heure en MICROSECONDES de la dernière interruption. 
  unsigned long long duration_last_pass; //durée en MICROSECONDES entre les deux dernière interruptions
  float rps_actual; //rotation par secondes du dernier tour
  float rps_last; //rotation par secondes de l'avant dernier tour

  unsigned long long time_real_last_pass;
  
  //Nombre de points pour faire la moyenne de vitesse (Corrige l'instabilité des capteurs de vitesse)
  int cnt_avg;
  float tab_avg[AVG]; //tableau roulant pour avoir une moyenne sur les 4 dernières valeurs

  long long total_pass;
  float error_sum;
  float last_error;

} PID_data;

void init_PID_data(PID_data * data){
  data->time_last_pass = -1; //heure en MICROSECONDES de la dernière interruption. 
  data->duration_last_pass = -1; //durée en MICROSECONDES entre les deux dernière interruptions
  data->rps_actual = 0; //rotation par secondes du dernier tour
  data->rps_last = 0; //rotation par secondes de l'avant dernier tour
  
  //Nombre de points pour faire la moyenne de vitesse (Corrige l'instabilité des capteurs de vitesse)
  data->cnt_avg = 0;
  data->cnt_avg = 0;
  for(int i = 0 ; i< AVG ; i++){
    data->tab_avg[i] = 0;
  }

  data->total_pass = 0;
  data->error_sum = 0;
  data->last_error = 0;

}


volatile PID_data PID_motor_1;
volatile PID_data PID_motor_2;


unsigned long long compute_duration_hall_effect(unsigned long long now, PID_data * data){ //Calcul combien de temps s'est passé depuis la dernière interruption
  if(data->time_last_pass == -1) return -1;
  return now-data->time_last_pass;
}



//Enregistre une interruption
//Stall : Mettre oui si le moteur ne tourne plus et qu'on veut SIMULER un passage de capteur afin de permettre le recalcul de la vitesse (On ne peut pas mesurer la vitesse si le moteur ne tourne pas puis-ce qu'il ne passe pas devant le capteur)
void hall_tick(bool stall, PID_data * data){
  unsigned long long now = micros();
  
  if(now-data->time_real_last_pass < (unsigned long long)MINIMUM_INTERUPT_MICROS) return; //SI L'INTERRUPTION EST TROP RAPIDE -> REJET DES ACTIVATIONS PARASITES

  if(!stall){ // Si tout se passe bien
    
    data->duration_last_pass = compute_duration_hall_effect(now, data);
    data->time_last_pass = now;  
    data->time_real_last_pass = now;
    data->total_pass++;

  } else { //Si le moteur ne tourne plus
    
    data->duration_last_pass = data->duration_last_pass*STALL_FACTOR*2;  //On simule une valeur de vitesse qui ralentie
  }

  if(data->duration_last_pass > 0){
    data->tab_avg[data->cnt_avg] = 1000000/(float)data->duration_last_pass; //calcul des tour/s en fonction de la durée en microsecondes
    data->cnt_avg++;
    if(data->cnt_avg == AVG){
      data->cnt_avg = 0;
    }

    //moyenne de la vitesse
    data->rps_last = data->rps_actual;
    data->rps_actual = 0;
    for(int i = 0 ; i< AVG ; i++){
      data->rps_actual += data->tab_avg[i]/AVG;
    }
    
  } else if(data->duration_last_pass == -1){ //Toute première interruption, on ne peut pas calculer la vitesse puis-ce qu'il en faut 2
    data->rps_actual = 0;
    data->rps_last = 0;
  }
  
}


//Récepteur d'interruption
void ISR_hall_effect_motor1() { 
  hall_tick(false,&PID_motor_1);
}

//Récepteur d'interruption
void ISR_hall_effect_motor2() { 
  hall_tick(false,&PID_motor_2);
}

void stop_motors_A(){
  SoftPWMSet(LED_BUILTIN, 0);

  SoftPWMSet(D1A, 0);
  SoftPWMSet(D2A, 0);
  SoftPWMSet(D3A, 0);
  SoftPWMSet(D4A, 0);

  delay(WAIT_STOP_MS);
}

void stop_motors_B(){
  SoftPWMSet(LED_BUILTIN, 0);

  SoftPWMSet(D1B, 0);
  SoftPWMSet(D2B, 0);
  SoftPWMSet(D3B, 0);
  SoftPWMSet(D4B, 0);

  delay(WAIT_STOP_MS);
}

void set_power(int pin, int power){
    SoftPWMSet(pin, power);
    SoftPWMSet(LED_BUILTIN, power);
}




//calcul rapide
#define mm_per_pass ((float)((2*3.14f*WHEEL_RADIUS_MM)/7.5f)/2.0)


//PID google it
void do_PID(PID_data * data,int pin, float rps_target){

    unsigned long long now = micros();
    unsigned long long current_duration = compute_duration_hall_effect(now,data);

    //Si on calcule que l'impulsion actuelle dure depuis beaucoup trop longtemps (Et donc que le moteur a probablement calé)
    if(current_duration > data->duration_last_pass*STALL_FACTOR){
      Serial.print("S");
      hall_tick(true,data); //On simule une interuption en mode 'calé'
    }
    

    float error = rps_target-data->rps_actual;
    data->error_sum += error*(PID_LOOP_MS/1000.0f);
    float error_derivate = (error-data->last_error)/(PID_LOOP_MS/1000.0f);;
    data->last_error = error;

    float kpe =  kp*error ;
    float kie = ki*data->error_sum;
    float kid = kd*error_derivate;

    float command = kpe + kie + kid;
    
    for(int i = 0 ; i < 100 ; i++){
      if((long)data->rps_actual == i){
        Serial.print("#");
      }else if((long)rps_target == i){
        Serial.print("|");
      }else{
        Serial.print(" ");          
      }
    }
    Serial.print("= rps_actual ");
    Serial.print(data->rps_actual);
    Serial.print(" cmd ");
    Serial.print(command);
    Serial.print(" rps_target ");
    Serial.print(rps_target);
    Serial.print(" ms ");
    Serial.println(millis());

    if(command < 0) command = 0;
    if(command > 255) command = 255;
    set_power(pin,command);
}


//distance en mm, mm_per_sec en mm/second, direction -> vrai va dans un sens, faux dans l'autre (Ne me demande pas lequel est lequel ça dépend du cablage du moteur et tout...)
void run_motors_chenille(long distance_mm, long mm_per_sec, bool direction){

  //calcul du nombre d'impulsions (interruptions ou pass) attendues pour la vitesse souhaitée (2 par tour de moteur)
  float mm_per_turn = (mm_per_pass*2);
  float rpm = (mm_per_sec*60/mm_per_turn);
  float rps_target = rpm/60.0f;
  long wanted_total_pass = (long) ((distance_mm+0.5f)/mm_per_pass);

  //stop les moteurs pour être sur que personne ne tourne
  stop_motors_A();

  Serial.print("Run chenille");
  Serial.print(" for ");
  Serial.print(distance_mm);
  Serial.print(" mm at ");
  Serial.print(mm_per_sec);
  Serial.print(" mm/s (");
  Serial.print(mm_per_pass);
  Serial.print(" mm_per_pass ");
  Serial.print(wanted_total_pass );
  Serial.print(" pulses (");
  Serial.print(rpm);
  Serial.println(" RPM)");
  


//  hall_tick(false);

  float last_error = 0;
  float error_sum = 0;

  long last_loop_millis = -999;
  long start_time_millis = millis();

  int pin_1 = direction?D1A:D2A;
  int pin_2 = direction?D3A:D4A;

  init_PID_data(&PID_motor_1);
  init_PID_data(&PID_motor_2);

  for(long i = 0 ; 1 ; i++){
    if( (long)millis() < last_loop_millis+PID_LOOP_MS) continue; //on saute la boucle si le temps n'est pas encore arrivée de l'exectuer. (Plus précis que delay)
    last_loop_millis = millis();
    do_PID(&PID_motor_1,pin_1,rps_target);
    do_PID(&PID_motor_2,pin_2,rps_target);

    if(PID_motor_1.total_pass >= wanted_total_pass && PID_motor_2.total_pass >= wanted_total_pass){
      break;
    } else if (PID_motor_1.total_pass >= wanted_total_pass || PID_motor_2.total_pass >= wanted_total_pass){
      Serial.println("Motor async, one is done and not the other");

    }
  }

  stop_motors_A();
  Serial.println("Run motor end");
}



//FIN LOGIQUE







//DEBUT POID




//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TU DOIS CALIBRER CES VALEURS TOI MEME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   https://github.com/bogde/HX711#how-to-calibrate-your-load-cell   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!         IL TE FAUD DEUX POIDS CONNUS        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!        TU CHOISIS L'UNITE QUE TU VEUX, C'EST IMPORTANT PLUS BAS       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define LOADCELL_OFFSET 340884 
#define LOADCELL_DIVIDER 262.5f
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



//nombre de mesure de poids utilise pour faire la moyenne (stabilisation du résultat)
#define LOADCELL_AVG 50

float poid_machine;
HX711 scale;


void setup_poid(){
  scale.begin(PIN_POID_DATA, PIN_POID_CLOCK);
  scale.set_scale(LOADCELL_DIVIDER);
  scale.set_offset(LOADCELL_OFFSET);

  delay(200);
  
  Serial.print("Poid sur capteur :");
  Serial.println(scale.get_units(LOADCELL_AVG));
  
  poid_machine = scale.get_units(LOADCELL_AVG);
}

float mesurer_poid(){
  return scale.get_units(LOADCELL_AVG)-poid_machine;
}


//FIN POID


































































//==================Fonctions===============
//
//void run_motors_chenille(long distance_mm, long mm_per_sec, bool direction)
// distance_mm => 50mm
// mm_per_sec => 500 mm/seconde (ENVIRON SELON LE DIAMETRE : PROBABLEMENT 270 MINIMUM POUR PAS CALER (1km/h) ET 15000 MAX MOTEUR (55km/h !!!)) - VAS Y DOUCEMENT STP
// direction => false (Ou true ou l'autre sens)
//
//float mesurer_poid()
// retourne le poid dans l'unité que tu aura choisis en faisant la calibration plus haut





void setup() {
  Serial.begin(57600);
  SoftPWMBegin();

  //pour faire joli
  SoftPWMSet(LED_BUILTIN, 0);
  SoftPWMSetFadeTime(LED_BUILTIN, 0, 0);
  
  pinMode(D1A, OUTPUT);
  pinMode(D2A, OUTPUT);
  pinMode(D3A, OUTPUT);
  pinMode(D4A, OUTPUT);
  
  pinMode(D1B, OUTPUT);
  pinMode(D2B, OUTPUT);
  pinMode(D3B, OUTPUT);
  pinMode(D4B, OUTPUT);

  //soft PWM car l'arduino n'en a que 4 ou 5 en hardware et il en faut 8.
  SoftPWMSet(D1A, 0);
  SoftPWMSetFadeTime(D1A, 0, 0);
  SoftPWMSet(D2A, 0);
  SoftPWMSetFadeTime(D2A, 0, 0);
  SoftPWMSet(D3A, 0);
  SoftPWMSetFadeTime(D3A, 0, 0);
  SoftPWMSet(D4A, 0);
  SoftPWMSetFadeTime(D4A, 0, 0);

  SoftPWMSet(D1B, 0);
  SoftPWMSetFadeTime(D1B, 0, 0);
  SoftPWMSet(D2B, 0);
  SoftPWMSetFadeTime(D2B, 0, 0);
  SoftPWMSet(D3B, 0);
  SoftPWMSetFadeTime(D3B, 0, 0);
  SoftPWMSet(D4B, 0);
  SoftPWMSetFadeTime(D4B, 0, 0);  

  pinMode(PIN_HALL_MOTOR1, INPUT_PULLUP); //pullup semble réduire les interférence, enfin peut-être, j'ai pas (encore) les outils pour vérifier.
  pinMode(PIN_HALL_MOTOR2, INPUT_PULLUP); //pullup semble réduire les interférence, enfin peut-être, j'ai pas (encore) les outils pour vérifier.
  pinMode(PIN_HALL_MOTOR3, INPUT_PULLUP); //pullup semble réduire les interférence, enfin peut-être, j'ai pas (encore) les outils pour vérifier.
  pinMode(PIN_HALL_MOTOR4, INPUT_PULLUP); //pullup semble réduire les interférence, enfin peut-être, j'ai pas (encore) les outils pour vérifier.

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_MOTOR1),ISR_hall_effect_motor1,RISING);//recommended for arduino board
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_MOTOR2),ISR_hall_effect_motor2,RISING);//recommended for arduino board

  //setup_poid();
}


//Ton poid d'activation. L'unité est celle que tu aura choisis d'utiliser quand tu a calibré ton capteur au dessus.
#define POID_ACTIVATION 666

void loop() {

  while(1){
      run_motors_chenille((long)DISTANCE_MM,1000, false);
  }
  
  delay(100);
}
