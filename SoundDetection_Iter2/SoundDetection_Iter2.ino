//Global Variable Definition

#define PIN_IN 2 //Digital PIN, Envelop signal
#define PIN_LED_OUT 13 //LED PIN on arduino (default)
#define LOW_LED_OUT 3 //LED PIN for low sound levels
#define HIGH_LED_OUT 4 //LED PIN for high sound levels
#define PIN_ANALOG A0 //ANALOG PIN, gate signal

void setup() {
  Serial.begin(9600);
  
  // Output Initialization of LEDs
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(PIN_LED_OUT, OUTPUT);
  
  // configure input to interrupt
  pinMode(PIN_IN, INPUT);
  attachInterrupt(PIN_IN, soundISR, RISING);
}
void loop() {
  SoundAnalyzer();

}
void soundISR() //Interupt function
{
  int pin_val; //local variable of pin value 
  pin_val = digitalRead(PIN_ANALOG); //Setting pin value of PIN_A0
  digitalWrite(PIN_LED_OUT, pin_val); //Allows to see if board recieving constant data stream
}

void SoundAnalyzer(){ //Main function 
  int value;
  
  // Check the envelope input
  value = analogRead(PIN_ANALOG); 
  
  // Convert GATE value into a message
  Serial.print("Status: "); //Prints Status to the serial monitor
  Serial.print(value); 
  
  if(value <= 500) //Condition statement in which incubator's sound level too low
  {
    Serial.println(" - Surroundings too Quiet.");
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }
  else if( (value > 500) && ( value <= 1000)) //Condition in which potential error in air-flow/system
  {
    Serial.println(" - Check Air Flow.");
    digitalWrite(3, HIGH);
  }
  else if(value > 1000 && value < 1200) //Condition at which all systems should be running
  {
    Serial.println(" - System Optimal.");
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
  }
  else if (value > 1200) //Condition in which surroundings too loud
  {
    Serial.println(" - Surroundings too Loud."); //Prompt user that the surroundings are too loud
    digitalWrite(4, HIGH);
  }
}
