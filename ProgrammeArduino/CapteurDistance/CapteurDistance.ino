int sensorpin = 0;                 // analog pin used to connect the sharp sensor
int val = 0;                 // variable to store the values from sensor(initially zero)

void setup()
{
  Serial.begin(9600);               // starts the serial monitor
}
 
void loop()
{
  val = analogRead(sensorpin);       // reads the value of the sharp sensor
  Serial.println(val);            // prints the value of the sensor to the serial monitor
  delay(1000);                    // wait for this much time before printing next value
}

// CAPTEUR DE DISTANCE VALUE
// RIEN A VOIR = 30 a 63
// 20 cm => 300
// 10 CM -> 500
// 5 cm -> 600
