
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345.h>
#include <Adafruit_L3GD20.h>
int batteryPin = 'A0';
const int temperaturePin = 'A1';

SoftwareSerial mySerial(3, 2);
Adafruit_ADXL345 accel = Adafruit_ADXL345(12345);
Adafruit_GPS GPS(&mySerial);
Adafruit_BMP085 bmp;
Adafruit_L3GD20 gyro;

#define GPSECHO  false

const int chipSelect = 10;

boolean usingInterrupt = false;
float gpsCoords[2] = {
};

int intData[10] = {
};
float floatData[11] = {
};


void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy  



void setup()  
{
  Serial.begin(9600);
//Serial.println("Alive!");
    Wire.begin();
//  pinMode(A0,INPUT);
  pinMode(10,OUTPUT);

  gyro.begin(gyro.L3DS20_RANGE_250DPS);
//  Serial.println("Gyro PASS");
  bmp.begin();
//  Serial.println("BMP PASS");
  accel.begin();
//  Serial.println("Accel PASS");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  //GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);

  delay(1000);
  
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();


void loop()
{
  //Process interrupt 
  if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  // every five seconds, get data from all of the sensors
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    sensors_event_t event; 
    accel.getEvent(&event);
    gyro.read();
    int battValue = analogRead(batteryPin);
    float battVolt =  battValue * ( 5.0 / 1023.0);
    battVolt = 2 * battVolt;
    float voltage, degreesC, degreesF;
    voltage = getVoltage(temperaturePin);
    degreesC = (voltage - 0.5) * 100.0;
    degreesF = degreesC * (9.0/5.0) + 32.0;

    Serial.println("Loop");

    intData[0] = GPS.hour;          // hr,min,sec,sat,grX,grY,grZ,lat,lng,alt,pres,temp,accX,accY,accZ,volt,degF
    intData[1] = GPS.minute;
    intData[2] = GPS.seconds;
    intData[3] = GPS.satellites;
    intData[4] = (int)gyro.data.x;
    intData[5] = (int)gyro.data.y;
    intData[6] = (int)gyro.data.z;
    floatData[0] = GPS.latitude;
    floatData[1] = GPS.longitude;
    floatData[2] = bmp.readAltitude();
    floatData[3] = bmp.readPressure();
    floatData[4] = bmp.readTemperature();
    floatData[5] = event.acceleration.x;
    floatData[6] = event.acceleration.y;
    floatData[7] = event.acceleration.z;
    floatData[8] = battVolt;
    floatData[9] = degreesF;

    String dataMessage = "";  //base string for message, need to write CSV data into it
    String headerMessage = "|";  //base header to send 500ms before main message is "|"[pipe] and one byte with count of total message bytes
    char headerBuf[2]; //buffer for the 2 byte header message, with a null terminating char


    for(int i=0;i<7;i++){
      dataMessage = dataMessage + intData[i] + ",";    //copy integer data into message
    }

    for(int i=0;i<10;i++){
      long floatPart = floatData[i];  //split off integer part
      int decPart = (floatData[i] - floatPart)*100; // split off decimal part
      dataMessage = dataMessage + floatPart + "." + abs(decPart) + ",";     //rebuild as string into data message
    }
    //Get the message length, add it as a byte to the header
    int dataLen = dataMessage.length();  
//    headerMessage = headerMessage + (char)dataLen;
    Serial.print(headerMessage);

    headerMessage.toCharArray(headerBuf,2);


    Wire.beginTransmission(17);
    Wire.write(headerBuf);

    Wire.endTransmission();
    Serial.println();
    delay(100);

    for(int i = 0; i <= dataMessage.length()/29; i++){
      char charBuf[30];  //buffer for i2c max length 30, plus a terminating null character
      String mySubString;  //holds 30 char string to send      

      mySubString = mySubString + dataMessage.substring(i*29,(i*29)+29);    
      mySubString.toCharArray(charBuf,30);
     Serial.println(mySubString);

//    Send buffered text
      Wire.beginTransmission(17);
      int bytesWritten = Wire.write(charBuf);
//         Serial.print(bytesWritten);
      delay(90);
      Wire.endTransmission();
//      delay(150);
    }

  }







}
float getVoltage(int pin)
{
  return (analogRead(pin) * 0.004882814);
}
