
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>u
#include "pitches.h"
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Adafruit_GPS.h>

//GPS 
HardwareSerial mySerial = Serial2;
Adafruit_GPS GPS(&Serial2);
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);


//LED
#define BLUE A2
#define GREEN A4
#define RED A6

int redValue;
int greenValue;
int blueValue;

//LoRa config
#define RFM95_CS 4
#define RFM95_RST 29
#define RFM95_INT 2

#define RF95_FREQ 433.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);


// Singleton instance of the radio driver
RH_RF95 driver;
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W


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
boolean buzzerActif= false;
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

int16_t packetnum = 0;  // packet counter, we increment per xmission

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
  saveData((String)F("BNOA: ") + AccX + ";" + AccY + ";" + AccZ);

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

//GPS
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

uint32_t timer = millis();
char c;
void getGPS() {
 if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (timer > millis())  timer = millis();

  delay(10);

  if (millis() - timer > 200) {
    timer = millis();

    if (GPS.fix==1) {
    saveData((String)F("GPS_dataLat : ") + (GPS.latitudeDegrees,4));
    saveData((String)F("GPS_dataLong : ") + (GPS.longitudeDegrees,4));
    saveData((String)F("GPS_dataAlt : ") + (GPS.altitude));
    saveData((String)F("GPS_dataSp : ") + (GPS.speed));
    } 
    else{
      saveData((String)F("GPS_data = 0,0,0,0"));
    }
}
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
  if (buzzerActif ==true){
    tone(buzzerPin,1000,90000);
  }
}

void saveData(String dump) {
  startSD();
  File dataFile = SD.open(F("ATT.txt"), FILE_WRITE);

  dataFile.println(dump);
  dataFile.close();

  Serial.println(dump);
  xbee.println(dump);
  
  startLora();
  
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  int16_t packetnum = 0;

    packetnum += 1; // increment 

    // --- Compose the Message to send ------------
    String packet_str = String("LORA-"+dump );
    // send to Serial
    Serial.print( packet_str.c_str() );
    // Send over Radio
    rf95.send((uint8_t *)(packet_str.c_str()), packet_str.length());
    rf95.waitPacketSent();

    // Now wait for a reply
    uint8_t buf[4]; // We limit the quantity received data
    uint8_t len = sizeof(buf);
 
    if (rf95.waitAvailableTimeout(200))  { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len)) {
          Serial.print(": ");
          Serial.println((char*)buf);
      } else {
          Serial.println("Receive failed");
      }
    } else {
        Serial.println("   caution : NO REPLY");
    }
startSD();
}

void getLora() {

}
void startLora(){
  digitalWrite(53,HIGH);
  digitalWrite(RFM95_CS,LOW);
}
void startSD(){
   digitalWrite(RFM95_CS,HIGH);

  digitalWrite(53,LOW);
}


void setup (){
    Serial.begin(9600);
    xbee.begin(9600);

    //GPS
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
    GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);

    delay(1000);

    mySerial.println(PMTK_Q_RELEASE);
    
    //RGBconfig
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);

    redValue = 255; // choose a value between 1 and 255 to change the color.
    greenValue = 0;
    blueValue = 0;

    analogWrite(RED, redValue);
    analogWrite(GREEN, greenValue);
    analogWrite(BLUE, greenValue);

    //LoRa
    pinMode(53,OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    startLora();
    delay(1000);
    digitalWrite(RFM95_RST, HIGH);

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
    }
    Serial.println("LoRa radio init OK!");

    rf95.setTxPower(23, false);
    
    //SD init - config
    startSD();
    delay(1500);
    if (!SD.begin()) {
    Serial.println("SD FAILED");
    while (1);
    }

    
    //bme initialisation
    if (!bme.begin()) {
    Serial.println("BME FAILED");
    while (1);
    }
    //bno initialisation
    if(!bno.begin()){
    Serial.println("BNO FAILED");
    while(1);
    }
       Serial.println("test");

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
  getGPS();
  getBuzzer();
  getLora();
  
  redValue = 0;
  greenValue = 0;
  blueValue = 255;

  analogWrite(RED, redValue);
  analogWrite(GREEN, greenValue);
  analogWrite(BLUE, greenValue);
 }  
}
