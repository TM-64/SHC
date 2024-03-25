#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "ms4525do.h"
#include "printf.h"
#include "RF24.h"
#include <LiquidCrystal.h>

RF24 controlRadio(2,1);
RF24 telemRadio(14,17);
struct TelemStruct {
  float bmp1;
  float bmp2;
  float bmp3;
  float tube1;
  float tube2;
  float battry;
};
TelemStruct telemetree;
struct ControlStruct {
  float throt;
  float rud;
  float elev;
  float propOff;
  float telemSOff;
};
ControlStruct controlio;
//uint8_t address[][6] = {"ST1CT", "ST1TU"};
uint64_t address[] = {82, 99};
float bmp01;
float bmp02;
float bmp03;
float tube01;
float tube02;
float battry0;
const int rs = 2, en = 3, d4 = 6, d5 = 7, d6 = 8, d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
void controls(){
  int throtty = analogRead(A2);
  int ruddy = analogRead(A0);
  int elevy = analogRead(A1);
  controlio.throt = float(map(throtty,0,1023,0,180));
  controlio.rud = float(map(ruddy,0,1023,0,1023));
  controlio.elev = float(map(elevy,0,1023,0,180));
}
void telemRadioRun(){
  uint8_t pipe;
  telemRadio.read(&telemetree, sizeof(telemetree));
  if(telemetree.battry!=0){
  bmp01 = telemetree.bmp1;
  bmp02 = telemetree.bmp2;
  bmp03 = telemetree.bmp3;
  tube01 = telemetree.tube1;
  tube02 = telemetree.tube2;
  battry0 = telemetree.battry;
  }
  /*bmp01 = 20.1;
  bmp02 = 3.2;
  bmp03 = 56.3;
  tube01 = 4.21;
  tube02 = 6.71;
  battry0 = 8.5;*/
    
}
void controlRadioRun(){
  /*if(digitalRead(1)==HIGH){
    controlio.propOff = 1.0;
  }else{
    controlio.propOff = 0.0;
  }
  if(digitalRead(2)==HIGH){
    controlio.telemSOff = 1.0;
  }else{
    controlio.telemSOff = 0.0;
  }*/
  controlio.throt= 76;
  controlio.elev= 120;
  controlio.propOff= 680;
  controlio.telemSOff= 346;
  controlio.rud= 2345;

   bool report = controlRadio.write(&controlio, sizeof(controlio));  // transmit & save the report
   if (report) {
      Serial.print(F("Transmission successful! "));
   }else{
   // Serial.println(F("Uhh ohh scooby doo were gonna find a mystery"));
   }
}
void lcdeer(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("T:%2.1f A:%3.1f",bmp01,bmp03);
  //lcd.print("T")
  //lcd.setCursor(0, 0);
  lcd.setCursor(0, 1);
  lcd.printf("S:%2.1f B:%2.1f",tube01,battry0);
}
void setup() {
  Serial.begin(9600);
  pinMode(A2,INPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(1,INPUT);
  pinMode(2,INPUT);
  controlRadio.begin();
  telemRadio.begin();
  controlRadio.setPALevel(RF24_PA_HIGH);
  telemRadio.setPALevel(RF24_PA_MAX);
  telemRadio.openReadingPipe(1,address[0]);
  controlRadio.openWritingPipe(address[1]);
  telemRadio.startListening();
  controlRadio.stopListening();
  lcd.begin(16, 2);

}

void loop() {
  // put your main code here, to run repeatedly:
//controls();
telemRadioRun();
//controlRadioRun();
lcdeer();
//Serial.println(controlio.throt);
//Serial.println(controlio.elev);
//delay(30);
/*Serial.println(telemetree.battry);
Serial.println(telemetree.bmp1);
Serial.println(telemetree.bmp2);
Serial.println(telemetree.bmp3);
Serial.println(telemetree.tube1);
Serial.println(telemetree.tube2);*/
delay(300);
}
