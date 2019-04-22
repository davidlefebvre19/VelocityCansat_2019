
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>u
#include "pitches.h"


//SD config
File myFile;

//Xbee config
SoftwareSerial xbee(19,18);

//BME config
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

//BNO config
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Pitot config
float V_0 = 5.0; // supply voltage to the pressure sensor
float rho = 1.204; // density of air 

int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

const unsigned long LOAD_INTERVAL = 500;
unsigned long previousLoad = 0;

//Buzzer and impact config
int fsrPin = 1;     // the FSR and 10K pulldown are connected to a1
int fsrReading;     // the analog reading from the FSR resistor divider

int buzzerPin = 15;

int melody[] = {
  NOTE_G4, NOTE_C5, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_E4, 
  NOTE_A4, NOTE_G4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_C4, 
  NOTE_D4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, 
  NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_G4, 
  NOTE_C5, NOTE_B4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_E4, 
  NOTE_A4, NOTE_G4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_C4, 
  NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4, NOTE_B4, NOTE_C5, NOTE_D5, 
  NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_G4, NOTE_G4, NOTE_B4, NOTE_C5, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_B4,
  NOTE_C5, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_F5,
  NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_C5,
  NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_A4, NOTE_A4,
  NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4, NOTE_C4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5
};

int noteDurations[] = {
  8, 4, 6, 16, 4, 8, 8, 
  4, 6, 16, 4, 8, 8, 
  4, 8, 8, 4, 8, 8, 4, 8, 8, 2,
  4, 6, 16, 4, 8, 8, 
  4, 6, 16, 4, 8, 8, 
  4, 6, 16, 4, 6, 16, 
  4, 6, 16, 8, 8, 8, 8, 
  2, 8, 8, 8, 8, 3, 8, 8, 8, 8, 8,
  2, 8, 8, 8, 8, 3, 8, 8, 8, 8, 8,
  4, 6, 16, 4, 6, 16, 4, 8, 8, 2,
  2, 8, 8, 8, 8, 3, 8, 2,
  2, 8, 8, 8, 8, 3, 8, 2,
  4, 6, 16, 4, 4, 2, 4, 4, 1
};


//
float TempBME,PresBME,AltBME,Humidity,Gas,acc_x,acc_z,acc_y;

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

  //imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gyrosc = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //imu::Vector<3> lacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  
  double AccX = acc.x();
  double AccY = acc.y();
  double AccZ = acc.z();
  saveData((String)"BNOA: " + AccX + ";" + AccY + ";" + AccZ);

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

String dat;

void getPitot() {
  float adc_avg = 0; float veloc = 0.0;
  
// average a few ADC readings for stability
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(A0)-offset;
  }
  adc_avg/=veloc_mean_size;
  
  // make sure if the ADC reads below 512, then we equate it to a negative velocity
  if (adc_avg>512-zero_span and adc_avg<512+zero_span){
  } else{
    if (adc_avg<512){
      veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }
  saveData((String)F("PITOT: ") + veloc);
}

void getBuzzer() {
  //buzzer et impact
  fsrReading = analogRead(fsrPin);
  saveData((String)F("FSR: ") + fsrReading);
  if (fsrReading > 10) {
      for (int thisNote = 0; thisNote < sizeof(melody) / 2; thisNote++) {
    int noteDuration = 2000 / noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(buzzerPin);
  }
  }
}

void saveData(String dump) {
  File dataFile = SD.open(F("ATT.txt"), FILE_WRITE);

  dataFile.println(dump);
  dataFile.close();

  Serial.println(dump);
  xbee.println(dump);
}


void setup (){
    Serial.begin(9600);
    xbee.begin(9600);
    
    //SD init - config
    pinMode(53, OUTPUT);
    
    if (!SD.begin()) {
    while (1);
    }
    
    //bme initialisation
    if (!bme.begin()) {
    while (1);
    }

    //bno initialisation
    if(!bno.begin()){
    while(1);
    }

    //Pitot config
    for (int ii=0;ii<offset_size;ii++){
    offset += analogRead(A0)-(1023/2);
    }
  
    offset /= offset_size;
  
    // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  //Buzzer
}

void loop () {
  unsigned long currentMillis = millis();
  
  // Si BLINK_INTERVAL_1 ou plus millisecondes se sont écoulés
  if(currentMillis - previousLoad >= LOAD_INTERVAL) {
    
    // Garde en mémoire la valeur actuelle de millis()
    previousLoad = currentMillis;
  getBME();
  getBNO();
  getPitot();
  getBuzzer();
  }  
}
