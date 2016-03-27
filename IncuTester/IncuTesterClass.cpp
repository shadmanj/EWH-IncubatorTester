#include "IncuTesterClass.h"


IncuTesterClass::IncuTesterClass()
{
}


void IncuTester::begin()
{
  //I/O pin for controlling heating pad
  pinMode(transistorPin, OUTPUT); 

  
  // Output Initialization of LEDs
  pinMode(LOW_LED_OUT, OUTPUT);
  pinMode(HIGH_LED_OUT, OUTPUT);
  pinMode(PIN_LED_OUT, OUTPUT);
  
  // configure input to interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_IN), soundISR, RISING);


  pinMode(LED,OUTPUT);
}



void IncuTester::analyzeSound()
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


void IncuTesterClass::getTempHumidity()
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
}



//Clear LCD screen
void IncuTesterClass::clearLCDScreen()
{
  mySerial.write(254);
  mySerial.write(128);
  mySerial.write("                ");
  
  mySerial.write(254);
  mySerial.write(192);
  mySerial.write("                ");
}




double IncuTesterClass::getpH()
{
  return (analogRead(SensorPin)*5/1023)*3.5+pHOffset;   //Convert voltage to pH, and incorporate the offset
}


//Receive signal from temperature sensor and convert to degrees Celsius
double IncuTesterClass::getTemp()
{
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



//Print outputs to LCD
void IncuTesterClass::printToLCD(float pH, float temp)
{
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
}



