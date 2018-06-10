//altimetrotwks1.3

//ENDEREÇOS I2C
#define ACCEL 0x53
#define GYROS 0x68
#define MAGNT 0x1E

//REGMAP ACELEROMETRO
#define ACCXLO 0X32
#define ACCXHI 0x33
#define ACCYLO 0x34
#define ACCYHI 0x35
#define ACCZLO 0x36
#define ACCZHI 0x37

#define GYROS 0x68
#define GYR_R 0x1B

#include "Arduino.h"
#include "SFE_BMP180.h"
//#include "MPU6050.h"
#include <SoftWire.h>
#include "I2Cdev.h"
//#include "SD.h"


#define IGNITOR 7
#define LED 33
#define TIME 10000

int16_t altura;
int16_t alturamax;
int16_t pressao;
int16_t temperatura;
int i=0;
int flag=0;

int16_t MAGx,GYRx,ACCx;
int16_t MAGy,GYRy,ACCy;
int16_t MAGz,GYRz,ACCz;

SoftWire FWire(PB6, PB7);

SFE_BMP180 bmp180;
char filename[7]="00.TXT";  //nome do arquivo inicial

void setup()
{
  Serial.begin(9600);
  while (!Serial) ;
    Serial.println("Start");
    Serial.println("1");
    bmp180.begin();
    Serial.println("2");
    FWire.begin();
    Serial.println("3");
 //   pinMode(SDFILE_PIN_CS, OUTPUT);
    Serial.println("4");
    pinMode(IGNITOR, OUTPUT);
    Serial.println("5");
      pinMode(LED, OUTPUT);
    Serial.println("Foi!!! =D");
 /* if (!SD.begin())
  {
    Serial.println(F("KEBAB REMOVED"));
    while(1);
  }
  Serial.println(F("É HORA DO SHOW."));
  while(SD.exists(filename))
  {
    if(i<10)
    sprintf(filename,"0%d.TXT",i);
    else
    sprintf(filename,"%2d.TXT",i);
    i++;
  }
  sdFile = SD.open(filename, FILE_WRITE);*/
  digitalWrite(LED,HIGH);
    altura = bmp180.altitude();
    alturamax=altura;
    pressao = bmp180.getPressure();
    temperatura = bmp180.getTemperatureC();

  FWire.beginTransmission(ACCEL);

    FWire.write(0x2D); // modo medição
    FWire.write(8);
  FWire.endTransmission(); 
    
    FWire.beginTransmission(ACCEL);
    FWire.write(0x21); // desativa modo double tap
    FWire.write(0);
    FWire.endTransmission();
    FWire.beginTransmission(ACCEL);
    FWire.write(0x22); // idem
    FWire.write(0);
    FWire.endTransmission();
    FWire.beginTransmission(ACCEL);
    FWire.write(0x23); // idem
    FWire.write(0);
    FWire.endTransmission();
    FWire.beginTransmission(ACCEL);
    FWire.write(0x31); // resolução completa, escala +/- 16g
    FWire.write(0x0B);  // 0.004 g por LSB
    FWire.endTransmission();


  FWire.begin();
  
  FWire.beginTransmission(MAGNT); //start talking
  
  FWire.write(0x02); // Set the Register
  FWire.write(0x00); // Tell the HMC5883 to Continuously Measure
  
  int j = FWire.endTransmission();
    switch (j){
    case 0: Serial.println("MAGNT SETUP OK"); break;
    case 1: Serial.println("MAGNT SETUP TRANSMIT BUFF OVF"); break;
    case 2: Serial.println("MAGNT SETUP ADDRESS NACK"); break;
    case 3: Serial.println("MAGNT SETUP DATA NACK"); break;
    case 4: Serial.println("MAGNT SETUP BELGIUM"); break;
}

  FWire.beginTransmission(GYROS);
    FWire.write(0x16);
    FWire.write(0x1C);
  int i = FWire.endTransmission();
  switch (i){
    case 0: Serial.println("GYROS SETUP OK"); break;
    case 1: Serial.println("GYROS SETUP TRANSMIT BUFF OVF"); break;
    case 2: Serial.println("GYROS SETUP ADDRESS NACK"); break;
    case 3: Serial.println("GYROS SETUP DATA NACK"); break;
    case 4: Serial.println("GYROS SETUP BELGIUM"); break;
  }

}
void loop(){
   altura = bmp180.altitude();
    pressao = bmp180.getPressure();
    temperatura = bmp180.getTemperatureC();
   
        if(altura<(alturamax-20.0) && !flag)
        {
            digitalWrite(IGNITOR,HIGH);//abre o paraquedas
            flag=1;
        Serial.print(millis());
        Serial.print(", ");
        Serial.println("abriu");
        }

 	if(altura > alturamax){
      alturamax = altura;
    }
// ------------------ LER MAGNETOMETRO -----------------------------
  
  FWire.beginTransmission(MAGNT);
  FWire.write(0x03); //start with register 3.
  FWire.endTransmission();
  
  FWire.requestFrom(MAGNT, 6);

  if(6==FWire.available()){

    MAGx = FWire.read()<<8; //MSB  x 
    MAGx |= FWire.read(); //LSB  x
    MAGy = FWire.read()<<8; //MSB  z
    MAGy |= FWire.read(); //LSB z
    MAGz = FWire.read()<<8; //MSB y
    MAGz |= FWire.read(); //LSB y
  }
//--------------------------------------------------

// ------------------ LER GYROS -----------------------------

  FWire.beginTransmission(GYROS);
  FWire.write(GYR_R); //start with register 3.
  FWire.endTransmission();
  
  FWire.requestFrom(GYROS, 8);

  if(FWire.available() == 8){

    int temp = FWire.read()<<8; //MSB  x 
    temp |= FWire.read(); //LSB  x
    GYRx = FWire.read()<<8; //MSB  z
    GYRx |= FWire.read(); //LSB z
    GYRy = FWire.read()<<8; //MSB y
    GYRy |= FWire.read(); //LSB y
    GYRz = FWire.read()<<8; //MSB y
    GYRz |= FWire.read(); //LSB y

  }

  
//--------------------------------------------------

// ------------------ LER ACELEROMETRO -----------------------------


  FWire.beginTransmission(ACCEL);
  FWire.write(ACCXLO);
  FWire.write(ACCXHI);
  FWire.endTransmission();
  
  FWire.requestFrom(ACCEL, 2);
  if (FWire.available() <= 2){
    ACCx = FWire.read() ;
    ACCx |= FWire.read()<< 8;
  }
  
  // LER ACCEL EIXO Y
  
  FWire.beginTransmission(ACCEL);
  FWire.write(ACCYLO);
  FWire.write(ACCYHI);
  FWire.endTransmission();
  
  FWire.requestFrom(ACCEL, 2);
  if (FWire.available() <= 2){
    ACCy = FWire.read();
    ACCy |= FWire.read() << 8;
  }
  
  
  // LER ACCEL EIXO Z

  FWire.beginTransmission(ACCEL);
  FWire.write(ACCZLO);
  FWire.write(ACCZHI);
  FWire.endTransmission();
  
  FWire.requestFrom(ACCEL, 2);
  if (FWire.available() <= 2){
    ACCz = FWire.read();
    ACCz |= FWire.read()<<8;
  }
  
//--------------------------------------------------

  Serial.print(millis());
  Serial.print("\t");
  Serial.print(altura);
  Serial.print("\t");
  Serial.print(pressao);
  Serial.print("\t");
  Serial.print(temperatura);
  Serial.print("\t");
  Serial.print(ACCx);
  Serial.print("\t");
  Serial.print(ACCy);
  Serial.print("\t");
  Serial.print(ACCz);
  Serial.print("\t");
  Serial.print(GYRx);
  Serial.print("\t");
  Serial.print(GYRy);
  Serial.print("\t");
  Serial.print(GYRz);
  Serial.print("\t");
  Serial.print(MAGx);
  Serial.print("\t");
  Serial.print(MAGy);
  Serial.print("\t");
  Serial.print(MAGz);
  Serial.println();

  /*
      if(millis()>TIME)
    {
       sdFile.close();
       digitalWrite(LED,LOW);
    }
    */
}-