#include "OneWire.h"
#include <SoftwareSerial.h>    //"soft" serial port to prevent display corruption during upload
#define SensorPin A1           //pH meter Analog output to Arduino Analog Input 0
#define Offset 2.67            //deviation compensation
#define LED 12
#define samplingInterval 20
#define printInterval 800
#define ArrayLength  40          //times of collection


int pHArray[ArrayLength];        //Store the average value of the sensor feedback
int pHArrayIndex=0;  
int DS18S20_Pin = 5;            //DS18S20 temperature sensor signal pin on digital 4
//int button1 = 7;        //button 1 pin (calibration)
//int button2 = 8;        //button 2 pin (test)


//Temperature chip i/o
OneWire ds(DS18S20_Pin);        //DS18S20 temperature sensor on digital pin 4

//LCD Serial i/o
SoftwareSerial mySerial(8,6);    //Attach RX to digital pin 2



#include "dht.h"

const int transistorPin = 9; //I/O pin for controlling heating pad
dht DHT; //temperature sensor object
#define DHT22_DATA_PIN 7
unsigned long lastTime = 0;
unsigned long waitTime = 2000;


const uint8_t PIN_IN = 2; //Digital PIN, Envelop signal
const uint8_t PIN_LED_OUT = 13; //LED PIN on arduino (default)
const uint8_t LOW_LED_OUT = 3; //LED PIN for low sound levels
const uint8_t HIGH_LED_OUT = 4; //LED PIN for high sound levels
const uint8_t PIN_ANALOG = A0; //ANALOG PIN, gate signal

//const uint16_t lowSound = 500;
//const uint16_t airFlowSound = 1000;
//const uint16_t loudSound = 1200;


void setup()
{
  Serial.begin(115200);

  //I/O pin for controlling heating pad
  pinMode(transistorPin, OUTPUT); 

  
  // Output Initialization of LEDs
  pinMode(LOW_LED_OUT, OUTPUT);
  pinMode(HIGH_LED_OUT, OUTPUT);
  pinMode(PIN_LED_OUT, OUTPUT);
  
  // configure input to interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_IN), soundISR, RISING);


  pinMode(LED,OUTPUT);
  mySerial.begin(9600);  //Begin LCD serial  
  //Serial.println("pH meter experiment!");    //Test the serial monitor
  delay(500);    //Wait for serial to boot
  mySerial.write("                "); // clear display + legends
  mySerial.write("                ");
  
  getTemp();
}

void loop()
{
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  float temperature;

  //Read pH into pH array based on sampling time and sampling interval
  //Also get temperature reading
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLength)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLength)*5.0/1024;    //To filter noise, the average of the array is used
      pHValue = 3.5*voltage+Offset;   //Convert voltage to pH, and incorporate the offset
      temperature = getTemp();
      samplingTime=millis();
  }
  
  //Print the voltage, pH, and temperature values to the serial
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    printToSerial(pHValue, temperature);  
    printToLCD(pHValue, temperature);
    printTime=millis();
  }
    delay(300); //just here to slow down the output so it is easier to read







  if((millis() - lastTime) > waitTime)
  {
    TempHumidity();
    SoundAnalyzer();
    lastTime = millis();
  }
}


void SoundAnalyzer() //Main function 
{
  const uint16_t lowSound = 500;
  const uint16_t airFlowSound = 1000;
  const uint16_t loudSound = 1200;
  
  int value = 0;
  
  // Check the envelope input
  value = analogRead(PIN_ANALOG); 
  
  // Convert GATE value into a message
  Serial.print("Status: "); //Prints Status to the serial monitor
  Serial.println(value); 
  
  //if(value <= 500) //Condition statement in which incubator's sound level too low
  if(value <= lowSound) //Condition statement in which incubator's sound level too low
  {
    Serial.println(" - Surroundings too Quiet.");
    digitalWrite(LOW_LED_OUT, LOW);
    digitalWrite(HIGH_LED_OUT, LOW);
  }
  //else if( (value > 500) && ( value <= 1000)) //Condition in which potential error in air-flow/system
  else if( (value > lowSound) && ( value <= airFlowSound)) //Condition in which potential error in air-flow/system
  {
    Serial.println(" - Check Air Flow.");
    digitalWrite(LOW_LED_OUT, HIGH);
    digitalWrite(HIGH_LED_OUT, LOW);
  }
  //else if(value > 1000 && value < 1200) //Condition at which all systems should be running
  else if(value > airFlowSound && value < loudSound) //Condition at which all systems should be running
  {
    Serial.println(" - System Optimal.");
    digitalWrite(LOW_LED_OUT, HIGH);
    digitalWrite(HIGH_LED_OUT, HIGH);
  }
  //else if (value > 1200) //Condition in which surroundings too loud
  else if (value > loudSound) //Condition in which surroundings too loud
  {
    Serial.println(" - Surroundings too Loud."); //Prompt user that the surroundings are too loud
    digitalWrite(LOW_LED_OUT, LOW);
    digitalWrite(HIGH_LED_OUT, HIGH);
  }
}


void soundISR() //Interupt function
{
  int pin_val; //local variable of pin value 
  pin_val = digitalRead(PIN_ANALOG); //Setting pin value of PIN_A0
  digitalWrite(PIN_LED_OUT, pin_val); //Allows to see if board recieving constant data stream
}


void TempHumidity()
{
  // DHT 22 - read data
  int chk = DHT.read22(DHT22_DATA_PIN);
  
  // Turns on heating pad
  digitalWrite(transistorPin, HIGH);

  //Print the temperature and humidity
  Serial.print("DHT22:\t");
  Serial.print(DHT.humidity, 1);
  Serial.print(" (RH%) ");
  Serial.print(DHT.temperature, 1);
  Serial.println(" (deg C)");

  // Print new line
  Serial.print("\n");

  // DHT 22 needs 2 sec minimum between sampling
  //delay(2000);
}






//Clear LCD screen
void clearScreen(){
    mySerial.write(254);
    mySerial.write(128);
    mySerial.write("                ");
    
    mySerial.write(254);
    mySerial.write(192);
    mySerial.write("                ");
}
//Print outputs to LCD
void printToLCD(float pH, float temp){
    float p = pH;
    float t = temp;
    char ps[10]; 
    char ts[10];
    
    //Convert pH and temp to strings
    dtostrf(pH, 4,2,ps);
    dtostrf(temp, 5,2,ts);
    
    mySerial.write(254);
    mySerial.write(128);
    mySerial.write("pH:");
    
    mySerial.write(254);
    mySerial.write(192);
    mySerial.write(B11011111);
    
    mySerial.write(254);
    mySerial.write(193);
    mySerial.write("C:");
    
    mySerial.write(254);
    mySerial.write(135);
    mySerial.write(ps);
    
    mySerial.write(254);
    mySerial.write(198);
    mySerial.write(ts);
    //delay(500);
}
 
//Print outputs to serial
void printToSerial(float pH, float temp){
    Serial.print("\npH value: ");
    Serial.println(pH,2);
    
    Serial.print("Temperature:   ");
    Serial.println(temp,2);
    
    digitalWrite(LED,digitalRead(LED)^1);
}

//Averages the voltage reading taken from the pH sensor to reduce error
//Returns an averaged pH value
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  
  //Error check
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  
  //If number < 5, make no changes
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }
  
  else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  
  return avg;
}



//Receive signal from temperature sensor and convert to degrees Celsius
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}
