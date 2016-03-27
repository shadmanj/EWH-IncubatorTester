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



IncuTester myIncuTester;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);  //Begin LCD serial  
}

void loop()
{
  myIncuTester.getTemp();
  myIncuTester.getpH();
  myIncuTester.getTempHumidity();
  myIncuTester.analyzeSound();
  
  myIncuTester.printToSerial(pHValue, temperature);  
  myIncuTester.printToLCD(pHValue, temperature);

  delay(2000);
}



void soundISR() //Interupt function
{
  int pin_val; //local variable of pin value 
  pin_val = digitalRead(PIN_ANALOG); //Setting pin value of PIN_A0
  digitalWrite(PIN_LED_OUT, pin_val); //Allows to see if board recieving constant data stream
}


