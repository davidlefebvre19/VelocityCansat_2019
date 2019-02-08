
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>


//Xbee config
SoftwareSerial xbee(3,4);

//BME config
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

//BNO config
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
const String nameofcomp[] = {"BME : ","BNO :", "GPS : "};
const PROGMEM unsigned long LOAD_INTERVAL = 500;
unsigned long previousLoad = 0;


float TempBME,PresBME,AltBME,Humidity,Gas,ori_x,ori_z,ori_y;
const PROGMEM char conca[] = ";";

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
  //  saveData((String)F("BMP: ") + TempBMP + F(";") + PresBMP + F(";") + AltBMP+F(";")+Humidity+F(";")+Gas);
  //saveData((String)F("BMP: ") + TempBME + conca+ PresBME + conca+ AltBME+conca+Humidity+conca+Gas);
  float dat[] = {0,TempBME,PresBME,AltBME,Humidity,Gas,135791};
  saveData(dat);
}

void getBNO() {
  //nv event pr bno
  sensors_event_t event;
  bno.getEvent(&event);

  ori_x = event.orientation.x;
  ori_y = event.orientation.y;
  ori_z = event.orientation.z;
  float dat[] = {1,ori_x,ori_y,ori_z,135791};
  saveData(dat);
  //saveData((String)F("ORI: ")+ori_x+conca+ori_y+conca+ori_z);
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
stringSaver((String)F("GPSM: ") + dat);
  //saveData((String)F("GPSM: ") + dat);
//   float dat[] = {F("BMP: "),ori_x,ori_y,ori_z,135791};
  //saveData(dat);



}

void stringSaver(String dump){
    File dataFile = SD.open(F("ATT.TXT"), FILE_WRITE);
  dataFile.println(dump);
  dataFile.close();

  Serial.println(dump);
  xbee.println(dump);
}
void saveData(float dump[]) {
  File dataFile = SD.open(F("ATT.TXT"), FILE_WRITE);
  int i = 0;
  String dumpString = "";
  while(true){
    Serial.println(dump[i]);
    if(dump[i] != 135791.00){
      if(i==0){
        dumpString += nameofcomp[int(dump[i])];
      }else{
        dumpString += dump[i];
        dumpString +=conca;
      }
      i++;
    }else{
      break;
    }
  }
  dataFile.println(dumpString);
  dataFile.close();

  Serial.println(dumpString);
  xbee.println(dumpString);
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
