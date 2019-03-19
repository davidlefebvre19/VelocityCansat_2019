
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>


//Xbee config
SoftwareSerial xbee(19,18);

//BME config
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

//BNO config
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const unsigned long LOAD_INTERVAL = 500;
unsigned long previousLoad = 0;


float TempBME,PresBME,AltBME,Humidity,Gas,ori_x,ori_z,ori_y;

void getBME() {
  if (!bme.performReading()) {
    return;
  }
  // vous avez oublie de declarer toutes les variables
  TempBME = bme.temperature;
  PresBME = bme.pressure / 100;
  AltBME = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Humidity = bme.humidity;
  Gas = bme.gas_resistance / 1000.0;
  saveData((String)F("BMP: ") + TempBME + F(";") + PresBME + F(";") + AltBME+F(";")+Humidity+F(";")+Gas);
  //saveData((String)F("BMP: ") + TempBME + conca+ PresBME + conca+ AltBME+conca+Humidity+conca+Gas);
}

void getBNO() {
  //nv event pr bno
  sensors_event_t event;
  bno.getEvent(&event);

  ori_x = event.orientation.x;
  ori_y = event.orientation.y;
  ori_z = event.orientation.z;
  //saveData((String)F("ORI: ")+ori_x+conca+ori_y+conca+ori_z);
    saveData((String)F("ORI: ")+ori_x+F(";")+ori_y+F(";")+ori_z);

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

String dat;


void getNano() {
    Wire.requestFrom(5,1020);

  dat = F("");
  while( Wire.available() )
  {
    int x = Wire.read();
    dat += char(x);
  }

  saveData((String)F("GPSM: ") + dat);

}


void saveData(String dump) {
  File dataFile = SD.open(F("ATT.TXT"), FILE_WRITE);

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
    while (1);
  }

    //bno initialisation
    if(!bno.begin()){
    while(1);
  }

    // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop () {
  unsigned long currentMillis = millis();
  
  // Si BLINK_INTERVAL_1 ou plus millisecondes se sont écoulés
  if(currentMillis - previousLoad >= LOAD_INTERVAL) {
    
    // Garde en mémoire la valeur actuelle de millis()
    previousLoad = currentMillis;
  getBME();
  getBNO();
  getNano();
  }

}
