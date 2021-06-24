#include <SoftPWM.h>
#include <assert.h>

#define D1 6
#define D2 5
#define D3 3
#define D4 9

#define SEC_DIRECTION 10
#define SEC_RIEN 1

#define RUN_PWM 30
#define RUN_PWM2 30

#define WAIT_STOP_MS 250
#define WAIT_START_MS 50
#define STALL_CORRECTION_POWER 75 
#define OVERSPEED_RECOVERY_PWR 0
#define POWER_ABSOLUTE_MIN 30
#define POWER_ABSOLUTE_MAX 60

#define PIN_HALL_EFFECT 2

volatile bool pass_treated = true;
volatile unsigned long long  time_last_pass = 0;
volatile unsigned long long duration_last_pass = 0;
volatile bool hall_stuck_mode = false;
volatile bool hall_overspeed_mode = false;
volatile unsigned long stabilisation_counter = 0;

volatile float rps_actual = 0;
volatile float rps_last = 0;

#define AVG 4
volatile int cnt_avg = 0;
//volatile float avg = 0;
volatile float tab_avg[AVG];
volatile unsigned long long last_mic = 0;

#define STALL_FACTOR 2.0f


unsigned long long compute_duration_hall_effect(unsigned long long now){
  if(time_last_pass == -1) return -1;
  return now-time_last_pass;
}

void hall_tick(bool stall){
  unsigned long long now = micros();
  if(now-last_mic < (unsigned long long)3000) return;
  if(!stall){
//Serial.println("NOT STALL");
    duration_last_pass = compute_duration_hall_effect(now);
    time_last_pass = now;  
    last_mic = now;
  } else {
//Serial.println("STALL");
    duration_last_pass = duration_last_pass*STALL_FACTOR*2;    
  }
  pass_treated = false;
  hall_stuck_mode = false;
  hall_overspeed_mode = false;
  if(stabilisation_counter > 0){
    stabilisation_counter--;
  }

//Serial.println((float)duration_last_pass);
  if(duration_last_pass > 0){
    tab_avg[cnt_avg] = 1000000/(float)duration_last_pass;;
    cnt_avg++;
    if(cnt_avg == AVG){
      cnt_avg = 0;
    }
  
  //  Serial.println(micros());
    rps_last = rps_actual;
   
    rps_actual = 0;
    for(int i = 0 ; i< AVG ; i++){
      rps_actual += tab_avg[i]/AVG;
    }
    
  } else if(duration_last_pass == -1){
    rps_actual = 0;
    rps_last = 0;
  }
  
}


void ISR_hall_effect() { 
  hall_tick(false);
//    Serial.println("RPM");

}

void setup() {
  Serial.begin(57600);
  SoftPWMBegin();
  SoftPWMSet(LED_BUILTIN, 0);
  SoftPWMSetFadeTime(LED_BUILTIN, 0, 0);
  
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);

  pinMode(PIN_HALL_EFFECT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT),ISR_hall_effect,RISING);//recommended for arduino board
}

void stop_motors(){
  SoftPWMSet(LED_BUILTIN, 0);

  analogWrite(D1, 0);
  analogWrite(D2, 0);
  analogWrite(D3, 0);
  analogWrite(D4, 0);

  delay(WAIT_STOP_MS);
}

void set_power(int pin, int power){
    analogWrite(pin, power);
    SoftPWMSet(LED_BUILTIN, power);
}

void start_pin(int pin){
  stop_motors();
  
  set_power(pin, STALL_CORRECTION_POWER);
  delay(WAIT_START_MS);
}

#define HISTERYSIS_MS_FACTOR 0.1f
#define LOOP_MS 30
#define POWER_CHANGE 2.5f
#define OVERSPEED_FACTOR 0.2f

#define RESPONSE_FACTOR 0.01f

#define error_count 100
float error_array[error_count];
long error_pos = 0;

float total_error(){
  float res = 0;
  for(int i = 0 ; i <  error_count ; i++){
    res += error_array[i];
  }
  return res;
}

void init_error(){
  error_pos = 0;
  for(int i = 0 ; i <  error_count ; i++){
    error_array[i] = 0;
  }
}

void next_error(float error){
  error_array[error_pos] = error;
  error_pos++;
  if(error_pos >= error_count){
    error_pos = 0;
  }
}

void run_motor(long ms, int pin, long rpm){

  float ms_per_impulse = 1000000/(rpm/60.0f)/2;
  stop_motors();
//  start_pin(pin);
  float power = RUN_PWM; //start PWR

  Serial.print("Run pin");
  Serial.print(pin);
  Serial.print(" for ");
  Serial.print(ms);
  Serial.print(" ms at ");
  Serial.print(rpm);
  Serial.println("RPM");

  Serial.print(ms_per_impulse);
  Serial.println(" microseconds between impulses");

  time_last_pass = -1;
  duration_last_pass = -1;
  cnt_avg = 0;
  for(int i = 0 ; i< AVG ; i++){
    tab_avg[i] = 0;
  }
  hall_tick(false);

  float last_error = 0;
  float error_sum = 0;
  init_error();

  long last = -999;
  long strt = millis();

  for(long i = 0 ; millis() < strt+ms||1 ; i+= LOOP_MS){
    long cur = millis();
    if(cur < last+LOOP_MS) continue;
    last = cur;

    unsigned long long now = micros();
    unsigned long long current_duration = compute_duration_hall_effect(now);

    if(current_duration > duration_last_pass*STALL_FACTOR){
      Serial.print("S");
      hall_tick(true);


    }
/*

    if(current_duration > ms_per_impulse*STALL_FACTOR && !hall_stuck_mode){
      //stuck, engage recovery
      hall_stuck_mode = true;
      pass_treated = true;
      power += POWER_CHANGE;      

      Serial.print("Stuck detected after ");
      Serial.print(current_duration);
      Serial.print(" ms temporarily going to power ");
      Serial.println(STALL_CORRECTION_POWER);
      Serial.print(" and +");
      Serial.print(POWER_CHANGE);
      Serial.println(" to power");

      stabilisation_counter = 100;
    }


    if(!pass_treated){

      if(duration_last_pass < ms_per_impulse*OVERSPEED_FACTOR && duration_last_pass != -1){
        //way too fast, engage recovery
        hall_overspeed_mode = true;
        pass_treated = true;        
        power -= POWER_CHANGE;              

        Serial.print("Overspeed detected with ");
        Serial.print(duration_last_pass);
        Serial.print(" ms temporarily going to power ");
        Serial.println(OVERSPEED_RECOVERY_PWR);
        Serial.print(" and -");
        Serial.print(POWER_CHANGE);
        Serial.println(" to power");        


      }
    }

    */
    //if(!pass_treated)
    
    {          
#define CMD_OFFSET 0
#define kp (1.4*0.55f)
#define ki (2.53*0.50f)
#define kd (0.1395*0.35f)
/*
 *  OLD
#define kp 0.25f
#define ki 0.1f
#define kd 0.85f
 */
      float rps_target = 1000000/ms_per_impulse;

      float error = rps_target-rps_actual;
      error_sum += error*(LOOP_MS/1000.0f);
      float error_derivate = (error-last_error)/(LOOP_MS/1000.0f);;
      last_error = error;

      float kpe =  kp*error ;
      float kie = ki*error_sum;
      float kid = kd*error_derivate;

      float command = kpe + kie + kid;
      
/*
      */
      /*
      Serial.print(" rps_target ");
      Serial.print(rps_target);
      Serial.print(" error ");
      Serial.print(error);
      Serial.print(" error_sum ");
      Serial.print(error_sum);
      Serial.print(" error_derivate ");
      Serial.print(error_derivate);
      */
      
      for(int i = 0 ; i < 100 ; i++){
        if((long)rps_actual == i){
          Serial.print("#");
        }else if((long)rps_target == i){
          Serial.print("|");
        }else{
          Serial.print(" ");          
        }
      }
      Serial.print("= rps_actual ");
      Serial.print(rps_actual);
/*
      Serial.print(" ke ");
      Serial.print(kpe);
      Serial.print(" ki ");
      Serial.print(error_sum);
      Serial.print(" kd ");
      Serial.print(error_derivate);
      */
      Serial.print(" cmd ");
      Serial.print(command);
      Serial.print(" ms ");
      Serial.println(millis());

      

      
/*
      Serial.print("cmd ");
      Serial.println(command);
*/
      power = command + CMD_OFFSET;
      
      /*
      if(duration_last_pass > ms_per_impulse*(1.0f+HISTERYSIS_MS_FACTOR) && duration_last_pass != -1 && (stabilisation_counter == 0 || 1)){
        //increase_current
        float error_factor = 
        float factor = 1.0f + error_factor*RESPONSE_FACTOR;
        power *= factor;

        Serial.print("Too slow: ");
        Serial.println(duration_last_pass);
        Serial.print(" VS ");
        Serial.println(ms_per_impulse);
        Serial.print("Power *=");
        Serial.print(factor);
        Serial.print("to ");
        Serial.println(power);

        pass_treated = true;
      }
      if(duration_last_pass < ms_per_impulse*(1.0f-HISTERYSIS_MS_FACTOR) && duration_last_pass != -1 && stabilisation_counter == 0){
        //decrease_current
        float error_factor = ((float)duration_last_pass-(float)ms_per_impulse)/(float)ms_per_impulse;
        float factor = 1.0f + error_factor*RESPONSE_FACTOR;
        assert(factor < 2);
        power *= factor;

        //power -= POWER_CHANGE;

        Serial.print("Too fast: ");
        Serial.println(duration_last_pass);
        Serial.print(" VS ");
        Serial.println(ms_per_impulse);
        Serial.print("Power *=");
        Serial.print(factor);
        Serial.print("to ");
        Serial.println(power);

        pass_treated = true;

      }
      */
    }

    if(power < 0) power = 0;
    if(power > 255) power = 255;

    //if(power > POWER_ABSOLUTE_MAX) power=POWER_ABSOLUTE_MAX; 
    //if(power < POWER_ABSOLUTE_MIN) power=POWER_ABSOLUTE_MIN; 
    /*
    if(hall_stuck_mode){
      set_power(pin,STALL_CORRECTION_POWER);
    } else if(hall_overspeed_mode) {
      set_power(pin,OVERSPEED_RECOVERY_PWR);
    } else {
    */
      set_power(pin,power);
    /*
    }
*/
    
/*
    Serial.print(current_duration);
    Serial.println("c");

    Serial.print(hall_stuck_mode);
    Serial.println("s");

    Serial.print(hall_overspeed_mode);
    Serial.println("o");

    Serial.print(pass_treated);
    Serial.println("t");
*/
    //delay(LOOP_MS);
  }

  stop_motors();
  Serial.println("Run motor end");
}


void loop() {


    for(int i = 1 ; i <= 6 ; i++){
      run_motor((long)SEC_DIRECTION*1000,D1,i*300);
      
    }

    delay(SEC_RIEN*1000);

}
