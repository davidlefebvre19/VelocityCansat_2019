
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//BME config
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

/* il faut declarer ses variable */
float TempBME,PresBME,AltBME,Humidity,Gas;
void getBME() {
  // il faut lire la doc les enfants ... :
  if (! bme.performReading()) { // je lance la lecture
    Serial.println("Failed to perform reading :("); // si ca pete j affiche et je retourne rien poru arreter
    return;
  }
  // vous avez oublie de declarer toutes les variables
  TempBME = bme.temperature;
  PresBME = bme.pressure / 100;
  AltBME = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Humidity = bme.humidity;
  Gas = bme.gas_resistance / 1000.0;
  //  saveData((String)F("BMP: ") + TempBMP + F(";") + PresBMP + F(";") + AltBMP+F(";")+Humidity+F(";")+Gas);
  // c est quoi cette connerie , vous n avez meme pas mis les memes noms de variables
  saveData((String)F("BMP: ") + TempBME + F(";") + PresBME + F(";") + AltBME+F(";")+Humidity+F(";")+Gas);
}


void saveData(String dump) {

  //Send((String)dump); // vous n avez pas declarer send ...
   Serial.println(dump);
  File dataFile = SD.open("ATT.TXT", FILE_WRITE);

  dataFile.println(dump);
  dataFile.close();

}
void setup (){
    if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
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
}
