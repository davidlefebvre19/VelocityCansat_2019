
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//Xbee config
SoftwareSerial xbee(3,4);

//BME config
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

//BNO config
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


float TempBME,PresBME,AltBME,Humidity,Gas;


void getBME() {
  if (!bme.performReading()) { 
    Serial.println("Failed to perform reading :("); 
    return;
  }
  // vous avez oublie de declarer toutes les variables
  TempBME = bme.temperature;
  PresBME = bme.pressure / 100;
  AltBME = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Humidity = bme.humidity;
  Gas = bme.gas_resistance / 1000.0;
  //  saveData((String)F("BMP: ") + TempBMP + F(";") + PresBMP + F(";") + AltBMP+F(";")+Humidity+F(";")+Gas);
  saveData((String)F("BMP: ") + TempBME + F(";") + PresBME + F(";") + AltBME+F(";")+Humidity+F(";")+Gas);
}

void getBNO() {
  //nv event pr bno
  sensors_event_t event;
  bno.getEvent(&event);

    /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  Serial.println("");

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

String dat;


void getNano() {
    Wire.requestFrom(5,1020);

  dat = "";
  while( Wire.available() )
  {
    int x = Wire.read();
    //Serial.println((String)char(x)+":"+x);
    dat += char(x);
  }

  saveData((String)"GPSM: " + dat);

}

int GPSStat() {

  if(dat[0] == ';') return 1;
  return 0;
}


void saveData(String dump) {
   Serial.println(dump);
  File dataFile = SD.open("ATT.TXT", FILE_WRITE);

  dataFile.println(dump);
  dataFile.close();

  Serial.println(dump);
  xbee.println(dump);
}


void setup (){
    Serial.begin(9600);
    xbee.begin(9600);
    //bme initialisation
    if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

    //bno initialisation
    if(!bno.begin()){
    Serial.print("no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);

    delay(1000);
  }

    // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop () {
  getBME();
  getBNO();
  getNano();
}
