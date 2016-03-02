void setup() {
  // put your setup code here, to run once:
  
  // defines heating pad pin
  int transistorPin = 8;

  // Set to 115200 baud to match serial monitor
  Serial.begin(115200);

  // sets up heating pad
  pinMode(transistorPin, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // call temperature and humidity sensing function
  TempHumidity();

}

void TempHumidity() {

  //include DHT22 libraries
  #include <dht.h>
  #define DHT22_DATA_PIN 2
  dht DHT;

  // Defines pin location of the heating pad
  int transistorPin = 8;

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
  delay(2000);
  
}

