/*
* FILENAME: IncuTesterClass.h
* AUTHORS:   
* EMAIL:    
* VERSION:  0.0


* AFFILIATIONS
* Purdue University Engineering World Health


* DESCRIPTION
  

* UPDATES
* Version 0.0
* 2015/03/27:1450>




* DISCLAIMER


*/


#ifndef INCUTESTERCLASS_H
#define INCUTESTERCLASS_H



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


const int transistorPin = 9; //I/O pin for controlling heating pad
#define DHT22_DATA_PIN 7



#include "OneWire.h"
#include <SoftwareSerial.h> 
#include "Arduino.h"


class IncuTesterClass
{
private:

  dht DHT;

public:

  //Class Constructors
  IncuTesterClass();
  
  
  getTempHumidity();
  analyzeSound();

  clearLCDScreen();


  
  double getTemp();
  

};

#endif
