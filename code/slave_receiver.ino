// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include <SD.h>
const int chipSelect = 10;
const int temperaturePin = 0;
String lineData = "";
int freshData = 0;
void setup()
{
  Wire.begin(17);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
    while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop()
{
  float voltage, degreesC, degreesF;

  voltage = getVoltage(temperaturePin);

  degreesC = (voltage - 0.5) * 100.0;  
  
  degreesF = degreesC * (9.0/5.0) + 32.0;
  
  if(freshData==1){
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.print("voltage: ");
    dataFile.print(voltage);
    dataFile.print("  deg C: ");
    dataFile.print(degreesC);
    dataFile.print("  deg F: ");
    dataFile.println(degreesF);
    dataFile.println(lineData);
    dataFile.close();
    Serial.print("voltage: ");
    Serial.print(voltage);
    Serial.print("  deg C: ");
    Serial.print(degreesC);
    Serial.print("  deg F: ");
    Serial.println(degreesF);
    Serial.println(lineData); 
    freshData=0;
  }
  else{
    lineData = "";
  }
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  freshData = 1;
  while(0 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    lineData = String(lineData + c);
    //Serial.print(c);         // print the character
  }
  
  int x = Wire.read();    // receive byte as an integer

}
float getVoltage(int pin)
{
  return (analogRead(pin) * 0.004882814);
}
