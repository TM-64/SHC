#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include "Adafruit_BMP3XX.h"
#include "ms4525do.h"
#include "printf.h"
#include "RF24.h"



#define SEALEVELPRESSURE_HPA (1015)

Adafruit_BMP3XX bmp;
bfs::Ms4525do pres;
Adafruit_LSM6DSOX sox;
RF24 controlRadio(15,17);
RF24 telemRadio(14,13);
uint8_t address[][6] = {"1Node", "2Node"};
uint8_t address2[][6] = {"3Node", "4Node"};
double airDense = 1.225;
float ave;
double offset = 17;
double bmpData[3];
double soxData[2][3];
double airTubeData[3];
/*Struct telemPayload{
  int iden;
  double data
};*/
Servo elevator;
Servo rudder;
Servo throttle;
int elevPos = 0;
int rudPos = 0;
int thrott = 0;


void bmpRun(){
   bmpData[0] = bmp.temperature;
   bmpData[1] = bmp.pressure / 100.0;
   bmpData[2] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}
void LSOXRun(){
   sensors_event_t accel;
   sensors_event_t gyro;
   sensors_event_t temp;
   sox.getEvent(&accel, &gyro, &temp);
   soxData[0][0] = accel.acceleration.x;
   soxData[0][1] = accel.acceleration.y;
   soxData[0][2] = accel.acceleration.z;
   soxData[1][0] = gyro.gyro.x;
   soxData[1][1] = gyro.gyro.y;
   soxData[1][2] = gyro.gyro.z;
}
void airTubeRun(){
  for(int i=0;i<100;i++){
  if (pres.Read()) {
    ave+=pres.pres_pa();
    delay(5);
  }
  }
  airTubeData[0] =(ave/100)-offset;
  airTubeData[1] =sqrt(2*((ave/100)-offset)/airDense);
  airTubeData[2]= pres.die_temp_c();
  ave = 0;
}
void telemRadioRun(){
  int rand = 6;
 bool report = telemRadio.write(&rand,sizeof(rand));
 Serial.println(report);

}

void controlRadioRun(){
  if (controlRadio.available()) { 
      uint8_t bytes = controlRadio.getPayloadSize(); 
      int goof ;
      controlRadio.read(&goof,bytes); 
      Serial.print(goof);
      Serial.print(bytes);
    }
}
void controls(){
  elevator.write(elevPos);
  //rudder.write(rudPos);
  //throttle.write(thrott);

}
void failsafe(){
  Serial.print("DEATH");
}
void writeSDCard(){
 // Serial1.printf("%f,"micros());
  Serial1.printf("%f,%f,%f,",airTubeData[0],airTubeData[1],airTubeData[2]);
  Serial1.printf("%f,%f,%f,",soxData[0][0],soxData[0][1],soxData[0][2]);
  Serial1.printf("%f,%f,%f,",soxData[1][0],soxData[1][1],soxData[1][2]);
  Serial1.printf("%f,%f,%f\n",bmpData[0],bmpData[1],bmpData[2]);
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  while(!Serial);
  controlRadio.begin();
  telemRadio.begin();
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  sox.begin_I2C();
  pres.Config(&Wire, 0x28, -0.3f, 0.3f);
  pres.Begin();
  elevator.attach(9);
  rudder.attach(11);
  throttle.attach(10);
  pinMode(LED_BUILTIN, OUTPUT);
  controlRadio.setPALevel(RF24_PA_LOW);
  telemRadio.setPALevel(RF24_PA_LOW);
  telemRadio.enableDynamicPayloads(); 
  controlRadio.enableDynamicPayloads();
  //telemRadio.enableAckPayload();
  //controlRadio.enableAckPayload(); 
  telemRadio.openWritingPipe(address[0]);
  telemRadio.openReadingPipe(1, address[1]);
  controlRadio.openWritingPipe(address2[0]);
  controlRadio.openReadingPipe(1, address2[1]);
  telemRadio.stopListening();
  controlRadio.startListening();


}

void loop() {
  digitalWrite(LED_BUILTIN,LOW);
  bmpRun();
  LSOXRun();
  airTubeRun();
  digitalWrite(LED_BUILTIN, HIGH);
  writeSDCard();
  telemRadioRun();
  controlRadioRun();
  delay(1000);

}
