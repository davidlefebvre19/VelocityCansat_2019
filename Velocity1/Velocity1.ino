
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

//GPS config
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//RFM config
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
 
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


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

uint32_t timer = millis();
void getGPS() {
    // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
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

    //GPS initalisation
     GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);


    //RFM initialisation
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (!rf95.init()) {
    while (1);
    }

    if (!rf95.setFrequency(RF95_FREQ)) {
    while(1);
    }
    rf95.setTxPower(23, false);

    
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
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
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
  getGPS();
  }

}
