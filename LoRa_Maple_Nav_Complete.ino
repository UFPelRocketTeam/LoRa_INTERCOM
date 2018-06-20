/**
	SPI_1 and SPI_2 port example code

	Description:
	This sketch sends one byte with value 0x55 over the SPI_1 or SPI_2 port.
	The received byte (the answer from the SPI slave device) is stored to the <data> variable.

	The sketch as it is, works with SPI_1 port. For using the SPI_2 port, just
	un-comment all the nessesary code lines marked with <SPI_2> word.

	Created on 10 Jun 2015 by Vassilis Serasidis
	email:  avrsite@yahoo.gr

	Using the first SPI port (SPI_1)
	SS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
	SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
	MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
	MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN

	Using the second SPI port (SPI_2)
	SS    <-->  PB12 <-->  BOARD_SPI2_NSS_PIN
	SCK   <-->  PB13 <-->  BOARD_SPI2_SCK_PIN
	MISO  <-->  PB14 <-->  BOARD_SPI2_MISO_PIN
	MOSI  <-->  PB15 <-->  BOARD_SPI2_MOSI_PIN
*/


#define LR_RegFifo                       0x00
#define LR_RegOpMode                     0x01
#define LR_RegBitrateMsb                 0x02
#define LR_RegBitrateLsb                 0x03
#define LR_RegFdevMsb                    0x04
#define LR_RegFdMsb                      0x05
#define LR_RegFrMsb                      0x06
#define LR_RegFrMid                      0x07
#define LR_RegFrLsb                      0x08
#define LR_RegPaConfig                   0x09
#define LR_RegPaRamp                     0x0A
#define LR_RegOcp                        0x0B
#define LR_RegLna                        0x0C
#define LR_RegFifoAddrPtr                0x0D
#define LR_RegFifoTxBaseAddr             0x0E
#define LR_RegFifoRxBaseAddr             0x0F
#define LR_RegFifoRxCurrentaddr          0x10
#define LR_RegIrqFlagsMask               0x11
#define LR_RegIrqFlags                   0x12
#define LR_RegRxNbBytes                  0x13
#define LR_RegRxHeaderCntValueMsb        0x14
#define LR_RegRxHeaderCntValueLsb        0x15
#define LR_RegRxPacketCntValueMsb        0x16
#define LR_RegRxPacketCntValueLsb        0x17
#define LR_RegModemStat                  0x18
#define LR_RegPktSnrValue                0x19
#define LR_RegPktRssiValue               0x1A
#define LR_RegRssiValue                  0x1B
#define LR_RegHopChannel                 0x1C
#define LR_RegModemConfig1               0x1D
#define LR_RegModemConfig2               0x1E
#define LR_RegSymbTimeoutLsb             0x1F
#define LR_RegPreambleMsb                0x20
#define LR_RegPreambleLsb                0x21
#define LR_RegPayloadLength              0x22
#define LR_RegMaxPayloadLength           0x23
#define LR_RegHopPeriod                  0x24
#define LR_RegFifoRxByteAddr             0x25
#define LR_RegModemConfig3               0x26
#define REG_LR_DIOMAPPING1               0x40
#define REG_LR_DIOMAPPING2               0x41
#define REG_LR_VERSION                   0x42
#define REG_LR_PLLHOP                    0x44
#define REG_LR_TCXO                      0x4B
#define REG_LR_PADAC                     0x4D
#define REG_LR_FORMERTEMP                0x5B
#define REG_LR_AGCREF                    0x61
#define REG_LR_AGCTHRESH1                0x62
#define REG_LR_AGCTHRESH2                0x63
#define REG_LR_AGCTHRESH3                0x64
#define payload_length 16

#include <SPI.h>
#include <Servo.h>


#define SPI1_NSS_PIN PA4    //SPI_1 Chip Select pin is PA4. You can change it to the STM32 pin you want.
#define SPI2_NSS_PIN PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

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
#include "SD.h"

#define IGNITOR 7
#define LED 33
#define SDFILE_PIN_CS  7
File sdFile;

char filename[7]="00.TXT";  //nome do arquivo inicial


Servo pudim;

SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port

int8_t i=0;
int8_t flag=0;

int16_t altura;
int16_t alturamax;
int16_t pressao;
int16_t temperatura;

int16_t MAGx,GYRx,ACCx;
int16_t MAGy,GYRy,ACCy;
int16_t MAGz,GYRz,ACCz;



int16_t payload[16]={0,0,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0,0};

SoftWire FWire(PB6, PB7);

SFE_BMP180 bmp180;

void setup() {

  // Setup SPI 1
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV8);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(SDFILE_PIN_CS, OUTPUT);

  // Setup SPI 2
  SPI_2.begin(); //Initialize the SPI_2 port.
  SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
  pinMode(SPI2_NSS_PIN, OUTPUT);
// setup cartão

	if (!SD.begin())
    {
        //Serial.println(F("não funciona ou não está presente"));
        while(1);
    }
     //Serial.println(F("Cartão de memória inicializado."));
    while(SD.exists(filename))
    {
      if(i<10)
        sprintf(filename,"0%d.TXT",i);
      else
      sprintf(filename,"%2d.TXT",i);
      i++;
    }
    sdFile = SD.open(filename, FILE_WRITE);
    digitalWrite(13,HIGH);

// config do lora
	
	delay(100);
	// put in sleep mode to configure
	SPIwriteRegister(LR_RegOpMode,0x0);
	// sleep mode, high frequency
	delay(1000);
	SPIwriteRegister(REG_LR_TCXO,0x09);				// external crystal
	SPIwriteRegister(LR_RegOpMode,0x80);			// LoRa mode, sleep
	SPIwriteRegister(LR_RegFrMsb,0x6C);
	SPIwriteRegister(LR_RegFrMid,0x40);
	SPIwriteRegister(LR_RegFrLsb,0x13);				// frequency：433 MHz
	SPIwriteRegister(LR_RegPaConfig,0xFF);   		// max output power PA_BOOST enabled
	SPIwriteRegister(LR_RegPaRamp,0x08);			//pa ramp = 50 us
	SPIwriteRegister(LR_RegOcp,0x0B);				// close over current protection  (ocp)
	SPIwriteRegister(LR_RegLna,0x23);				// Enable LNA
	SPIwriteRegister(LR_RegModemConfig1,0x72); 		// signal bandwidth：125kHz, error coding= 4/5, explicit header mode
	SPIwriteRegister(LR_RegModemConfig2,0x77);		// spreading factor：7, CRC on
	SPIwriteRegister(LR_RegModemConfig3,0x04);		// Low Noise amp controlado pelo controle automatico de ganho
	SPIwriteRegister(LR_RegSymbTimeoutLsb,0xFF);    // max receiving timeout
	SPIwriteRegister(LR_RegPreambleMsb,0x00);		//
	SPIwriteRegister(LR_RegPreambleLsb,16);         // preamble 16 bytes  
	SPIwriteRegister(LR_RegPayloadLength,25);		// payload size, bytes
	SPIwriteRegister(LR_RegMaxPayloadLength,27);	// payload max, bytes
	SPIwriteRegister(REG_LR_PADAC,0x87);            // transmission power 20dBm
	SPIwriteRegister(LR_RegHopPeriod,0x00);         // no frequency hoping
	SPIwriteRegister(REG_LR_DIOMAPPING2,0x01);      // DIO5=ModeReady,DIO4=CadDetected
	SPIwriteRegister(REG_LR_DIOMAPPING1,0x41); 		// DIO0=TxDone,DIO1=RxTimeout,DIO3=ValidHeader
	SPIwriteRegister(LR_RegFifoRxBaseAddr, 0x00);
	delay(200);
	SPIwriteRegister(LR_RegOpMode,0x81);            //LoRa standby
	Serial.println("\nCONFIG DONE");
	int size = sizeof(payload);
	Serial.print("Packet Size:");
	Serial.println(size);
	
	int a=SPIreadRegister(LR_RegOpMode);
	a=a+SPIreadRegister(REG_LR_TCXO);
	a=a+SPIreadRegister(LR_RegFrMsb);
	a=a+SPIreadRegister(LR_RegFrMid);
	a=a+SPIreadRegister(LR_RegFrLsb);
	a=a+SPIreadRegister(LR_RegPaConfig);
	a=a+SPIreadRegister(LR_RegOcp);
	a=a+SPIreadRegister(LR_RegLna);
	a=a+SPIreadRegister(LR_RegModemConfig1);
	a=a+SPIreadRegister(LR_RegModemConfig2);
	a=a+SPIreadRegister(LR_RegModemConfig3);
	a=a+SPIreadRegister(LR_RegSymbTimeoutLsb);
	a=a+SPIreadRegister(LR_RegPreambleMsb);
	a=a+SPIreadRegister(LR_RegPreambleLsb);
	a=a+SPIreadRegister(LR_RegPayloadLength);
	a=a+SPIreadRegister(LR_RegMaxPayloadLength);
	a=a+SPIreadRegister(REG_LR_PADAC);
	a=a+SPIreadRegister(LR_RegHopPeriod);
	a=a+SPIreadRegister(REG_LR_DIOMAPPING2);
	a=a+SPIreadRegister(REG_LR_DIOMAPPING1);
	a=a+SPIreadRegister(LR_RegFifoRxBaseAddr);

	SPIwriteRegister(LR_RegOpMode,0x85);             // standby mode, high frequency
	Serial.print(a);
	pudim.attach(27);
	delayMicroseconds(10);
	pudim.write(270);
	delay(1000);
	pudim.write(0);
	delay(120000); //tempo pra ver se a portinha trancou

// ==== configuração dos instrumentos
	bmp180.begin();
	FWire.begin();
	pinMode(IGNITOR, OUTPUT);
	pinMode(LED, OUTPUT);
	digitalWrite(LED,HIGH);
	digitalWrite(IGNITOR,LOW);
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
	int m = FWire.endTransmission();
	  switch (m){
	  case 0: Serial.println("ACCEL SETUP OK"); break;
	  case 1: Serial.println("ACCEL SETUP TRANSMIT BUFF OVF"); break;
	  case 2: Serial.println("ACCEL SETUP ADDRESS NACK"); break;
	  case 3: Serial.println("ACCEL SETUP DATA NACK"); break;
	  case 4: Serial.println("ACCEL SETUP BELGIUM"); break;
	}

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

void loop() {
	//================ aquisição ====================


	altura = bmp180.altitude();
    pressao = bmp180.getPressure();
    temperatura = bmp180.getTemperatureC();
   
        if(altura<(alturamax-20.0) && !flag)
        {
        	flag = 1;
        	pudim.write(270); //abre o drogue
         	alturamax = bmp180.altitude();
         	flag = 2;

        }

        if(altura<400 && (flag == 2)){
        	digitalWrite(IGNITOR,HIGH);
        	flag = 3;

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
	//==================== monta o buffer ================== 

	int16_t altura_broadcast;
	int16_t pressao_broadcast;
	int16_t temperatura_broadcast;
	int16_t altmax_broadcast;


	altura_broadcast = (int16_t)(altura+0.5);
	pressao_broadcast = (int16_t)(pressao+0.5);
	temperatura_broadcast = (int16_t)(temperatura+0.5);
	temperatura_broadcast = (int16_t)(temperatura+0.5);

	//| 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10 | 11 | 12 | 13 | 14 |
	//|TIME|ALTI|PRES|TEMP|ACCX|ACCY|ACCZ|GYRX|GYRY|GYRZ|MAGX|MAGY|MAGZ|ALTM|NULL|	.
	payload[0] = 0;
	payload[1] = millis()*10;
	payload[2] = altura_broadcast;
	payload[3] = pressao_broadcast;
	payload[4] = temperatura_broadcast;
	payload[5] = ACCx;
	payload[6] = ACCy;
	payload[7] = ACCz;
	payload[8] = GYRx;
	payload[9] = GYRy;
	payload[10] = GYRz;
	payload[11] = MAGx;
	payload[12] = MAGy;
	payload[13] = MAGz;
	payload[14] = altmax_broadcast;
	payload[15] = 0;
	payload[16] = 0;

	if ((flag == 1)||(flag ==2)){
		payload[0] =  0;
		payload[1] =  1;
		payload[2] =  altura;
		payload[3] =  2;
		payload[4] =  altura;
		payload[5] =  4;
		payload[6] =  altura;
		payload[7] =  6;
		payload[8] =  altura;
		payload[9] =  8;
		payload[10] = altura;
		payload[11] = 6;
		payload[12] = altura;
		payload[13] = 4;
		payload[14] = altura;
		payload[15] = 2;
		payload[16] = 1;

	}

	// =============== transmissão =================
	SPIwriteRegister(LR_RegFifoTxBaseAddr, 0x00);
	unsigned char addr,temp;
	SPIwriteRegister(LR_RegIrqFlagsMask,0xf7);// enabling txdone	
	SPIwriteRegister(LR_RegIrqFlags,0xff);// clearing interupt
	SPIwriteRegister(LR_RegPayloadLength,payload_length);// payload length
	addr = SPIreadRegister(LR_RegFifoTxBaseAddr);// read TxBaseAddr        
	SPIwriteRegister(LR_RegFifoAddrPtr,addr);// TxBaseAddr->FifoAddrPtr          
	//SPIwriteBurst(0x00,payload,payload_length);   // write data in fifo
	for (int m=0;m<=payload_length;m++){
		SPIwriteRegister(0x00, payload[m]);
		Serial.print("\t");
		Serial.print(payload[m]);
		delay(10);
	}
	Serial.println();
	delay(100);
	/*for (int m=0;m<=payload_length;m++){
		Serial.print(SPIreadRegister(0x00));
	}*/
	SPIwriteRegister(LR_RegOpMode,0x83);
	Serial.println("\nTX on");	
	temp=SPIreadRegister(LR_RegIrqFlags);
	// read interput flag
	unsigned long int t0 = micros();
	while(!(temp&0x08)){// wait for txdone flag
		temp=SPIreadRegister(LR_RegIrqFlags);
		Serial.print(".");
		unsigned long int to = micros();
		if ((to-t0)>17368968){
			Serial.println("\n --------- transmit timeout --------");
			break;
			return;

		}
	}
	if(temp&0x08){Serial.println("\nTX done");}
	Serial.println();
	// ================= DATALOGGER ====================
	
 	sdFile.print(millis());
 	sdFile.print(altura);
 	sdFile.print(", ");
 	sdFile.print(pressao);
 	sdFile.print(", ");
 	sdFile.print(temperatura);
 	sdFile.print(",");
 	sdFile.print(ACCx);
 	sdFile.print(",");
 	sdFile.print(ACCy);
 	sdFile.print(",");
 	sdFile.print(ACCz);
 	sdFile.print(",");
 	sdFile.print(GYRx);
 	sdFile.print(",");
 	sdFile.print(GYRy);
 	sdFile.print(",");
 	sdFile.print(GYRz);
 	sdFile.print(",");
 	sdFile.print(MAGx);
 	sdFile.print(",");
 	sdFile.print(MAGy);
 	sdFile.print(",");
 	sdFile.print(MAGz);
 	sdFile.println();
 	sdFile.flush();

 	// ============= GPS =================

}

byte SPIwriteRegister(byte addr, byte val){
	digitalWrite(SPI2_NSS_PIN,LOW);
	byte wnr = 0x80;
	byte addrpacket = addr | wnr;
	SPI_2.transfer(addrpacket);
	delayMicroseconds(10);
	byte f=SPI_2.transfer(val);
	delayMicroseconds(10);
	digitalWrite(SPI2_NSS_PIN,HIGH);
	return f;

}
byte SPIreadRegister(byte addr){
	digitalWrite(SPI2_NSS_PIN,LOW);
	byte wnr = 0x00;
	byte addrpacket = addr | wnr;
	byte temp=SPI_2.transfer(addrpacket); //descarta o primeiro resultado na mosi
	delayMicroseconds(10);
	byte f=SPI_2.transfer(addrpacket);
	delayMicroseconds(10);
	digitalWrite(SPI2_NSS_PIN,HIGH);
	return f;
}