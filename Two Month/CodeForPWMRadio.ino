#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <SPI.h>
#include "Adafruit_BMP3XX.h"
#include "ms4525do.h"
#include "printf.h"
#include "RF24.h"

#define SEALEVELPRESSURE_HPA (1015)

#define buzzerPin 2
#define buzzerSWPin 7
#define BatteryPin A1
#define LEDPin 22



Adafruit_BMP3XX bmp; // sets the barometer as object bmp
Adafruit_LSM6DSOX sox; // sets the GYRO as object sox
bfs::Ms4525do pres; // sets the airspeed sensor as object pres
RF24 telemRadio(15,17);
uint64_t address = 82;
bool telemOff = false;


void setup()
{
  pinMode(buzzerSWPin, INPUT); // setup the PWM in pin
  Serial.begin(9600); // serial for debugging
  Serial.println("BEGIN LOGGING");
  pinMode(LEDPin, OUTPUT); // setup the LED flash pin
  pinMode(buzzerPin, OUTPUT); // setup the buzzer pin
 

///////////////////////////////
// Initialize SD card writer
///////////////////////////////
  Serial1.begin(9600); // serial for the SD card writer
  Serial1.println("BEGIN LOGGING");
// ALWAYS LEAVE ATLEAST ONE LOG IN THE SD CARD TO ENSURE IT DOESNT GET CORRUPTED

///////////////////////////////
// Initialize BMP, LSOX, pitot tube
///////////////////////////////
  bmp.begin_I2C(); // initialize the BMP390 for altitude 
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  sox.begin_I2C(); // initialize the gyro/accelerometer

  pres.Config(&Wire, 0x28, -.3f, .3f); // initialize the Pitot Tube
  pres.Begin(); //
///////////////////////////////
// Initialize Telemetry Radio
///////////////////////////////
  telemRadio.begin();
  telemRadio.openWritingPipe(address);
  telemRadio.stopListening();
  /*bmpRead();
  altOffset +=BMPVals[2];
  bmpRead();
  altOffset +=BMPVals[2];
  bmpRead();
  altOffset +=BMPVals[2];
  altOffset= altOffset/3;
  airTubeRead();
  speedOffset+= PitotVals[0];
  airTubeRead();
  speedOffset+= PitotVals[0];
  airTubeRead();
  speedOffset+= PitotVals[0];
  speedOffset= speedOffset/3;*/

  
}


//---------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------//


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Buzz Buzzer function /////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//declare variables for this fuction //
unsigned long previousTime1; // just copy pasted from the other function
unsigned long timer1;
// Purpose: to beep the buzzer
/////////////////////////////////////
void beeper() {
  if (millis() >= previousTime1 + 800) {
    previousTime1 = millis(); // prevents a bouncy output
    digitalWrite(buzzerPin, HIGH); // sets the mosfet gate high
    timer1 = previousTime1; // start a timer for when to turn off
  }
  //turn the LED OFF after 50ms of being on
  if ((millis() >= timer1 + 150)&&(millis() < timer1 + 5000)) { // the timer variable is started when the on condition is met
    digitalWrite(buzzerPin, LOW); // sets the mosfet gate low
    timer1 = 0;
  }
}

//Function to return the battery voltage from the onboard sensor////////////////////////////////////////////////
float readBatSensor() {
  float batteryVolt;
  batteryVolt = analogRead(BatteryPin); // gets the 10 bit value from the analog pin BatSensorPin
  batteryVolt = map(batteryVolt,0, 982, 0, 1260); // converts the 10 bit value to a calibrated voltage
  return batteryVolt;
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LED Flash function ////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//declare variables for this fuction //
unsigned long previousTime;
unsigned long timer;
// Purpose: to flash the LED
/////////////////////////////////////
void LEDFlash() {
  //turn the LED ON every 1200ms
  if (millis() >= previousTime + 1200) {
    //Serial.println("LED ON");
    previousTime = millis(); // prevents a bouncy output
    digitalWrite(LEDPin, HIGH); // sets the mosfet gate high
    timer = previousTime; // start a timer for when to turn off
  }
  //turn the LED OFF after 50ms of being on
  if ((millis() >= timer + 50)&&(millis() < timer + 5000)) { // the timer variable is started when the on condition is met
    //Serial.println("LED OFF");
    digitalWrite(LEDPin, LOW); // sets the mosfet gate low
    //Serial.println("LED OFF");
    timer = 0;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// Read airspeed sensor function //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Declare variables for this function ///
double speedOffset = 15.3;
double airDense = 1.225;
float ave;
float PitotVals[3]; // array to store the pitot tube values
// Purpose: reading data from the airspeed sensor
//////////////////////////////////
void airTubeRead(){
  for(int i=0;i<100;i++){
  if (pres.Read()) {
    ave+=pres.pres_pa();
    delay(5);
  }
  }
  PitotVals[0] =(ave/100)-speedOffset;
  PitotVals[1] =sqrt(2*((ave/100)-speedOffset)/airDense);
  PitotVals[2]= pres.die_temp_c();
  ave = 0;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to read data from the barometer /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// declare variables for this function //
int altOffset=134.9;
float BMPVals[3]; // array to store the barometer values
/////////////////////////////////
void bmpRead(){
  BMPVals[0] = bmp.temperature;
  BMPVals[1] = bmp.pressure / 100.0;
  BMPVals[2] = bmp.readAltitude(SEALEVELPRESSURE_HPA)-altOffset;
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to read data from the GYRO /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
float SOXVals[6]; // array to store all the gyro values

void LSOXRead(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);
  SOXVals[0] = accel.acceleration.x;
  SOXVals[1] = accel.acceleration.y;
  SOXVals[2] = accel.acceleration.z;
  SOXVals[3] = gyro.gyro.x;
  SOXVals[4] = gyro.gyro.y;
  SOXVals[5] = gyro.gyro.z;

  
}

//---------------------------------------------------------------------------------------------------------------------------------//
// TELEMETRY
//---------------------------------------------------------------------------------------------------------------------------------//
struct TelemStruct {
  float bmp1;
  float bmp2;
  float bmp3;
  float tube1;
  float tube2;
  float battry;
};
TelemStruct telemetry;

void telemetRun(int batteVolt){
  if(telemOff){
  }else{
    telemetry.battry = batteVolt;
    telemetry.bmp1 = BMPVals[0];
    telemetry.bmp2 = BMPVals[1];
    telemetry.bmp3 = BMPVals[2];
    telemetry.tube1 = PitotVals[0];
    telemetry.tube2 = PitotVals[1];
    /*telemetry.battry = 45;
    telemetry.bmp1 = 82;
    telemetry.bmp2 = 94.76;
    telemetry.bmp3 = 67;
    telemetry.tube1 = 1234;
    telemetry.tube2 = 9868;*/
    

   bool report = telemRadio.write(&telemetry, sizeof(telemetry));  // transmit & save the report
   if (report) {
      Serial.println(F("Transmission successful! "));
   }
  }
}

//------------------------------------------------------------------------------------------------//
// PWM INPUT
//------------------------------------------------------------------------------------------------//
//Function to read PWM values from GPIO
byte GetPWM(byte pin)
{
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}

//---------------------------------------------------------------------------------------------------------------------------------//
// Main Loop 
//---------------------------------------------------------------------------------------------------------------------------------//

float batteryVolt;

void loop()
{
  LEDFlash();
  bmpRead(); // read from bmp
  LSOXRead(); // read from gyro
  airTubeRead();  // was having trouble with this one disable if you can't flash it
  batteryVolt = readBatSensor();

  
  

  if (GetPWM(buzzerSWPin) >= 7) { // if the duty cycle on TeleSWPin is greater than 70%. ie if the radio switch is pressed
    //digitalWrite(buzzerPin, HIGH); // sound the buzzer
    //Serial.println("******************************"); // vusual buzzer for debug
    beeper();
  }
  else {
    digitalWrite(buzzerPin, LOW); // turn off the buzzer

  }



  //---------------------------------------------------------------------------------------------------------------------------------------//
  // Write to SD Card 
  // --------------------------------------------------------------------------------------------------------------------------------------//
/*
  if (Serial) { // only runs this code if it is plugged into USB
    Serial.print(millis());
    Serial.printf(", %f, %f, %f,",PitotVals[0],PitotVals[1],PitotVals[2]); // print for debug
    Serial.printf(" %f, %f, %f,",SOXVals[0],SOXVals[1],SOXVals[2]);
    Serial.printf(" %f, %f, %f,",SOXVals[3],SOXVals[4],SOXVals[5]);
    Serial.printf(" %f,", batteryVolt);
    Serial.printf(" %f, %f, %f \n",BMPVals[0],BMPVals[1],BMPVals[2]);
  }
  */



  Serial1.print(millis());
  Serial1.printf(", %f, %f, %f,",PitotVals[0],PitotVals[1],PitotVals[2]); // print for debug
  Serial1.printf(" %f, %f, %f,",SOXVals[0],SOXVals[1],SOXVals[2]);
  Serial1.printf(" %f, %f, %f,",SOXVals[3],SOXVals[4],SOXVals[5]);
  Serial1.printf(" %f,", batteryVolt);
  Serial1.printf(" %f, %f, %f \n",BMPVals[0],BMPVals[1],BMPVals[2]);
  telemetRun(batteryVolt);
  
}

