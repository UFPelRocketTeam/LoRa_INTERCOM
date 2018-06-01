// NAV COMPLETE + RF BROADCAST
// DEV BY n.jOy[dream]


#include "Arduino.h"
#include "SFE_BMP180.h"
//#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
//#include "SD.h"
#include "SPI.h"


//ENDEREÇOS I2C nav
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


//regmap gyros
#define GYROS 0x68
#define GYR_R 0x1B


// defines do modulo de cartão sd
#define SDFILE_PIN_CS  10
#define IGNITOR 7
#define LED 4
#define TIME 10000


// defines do lora

#define MOSI   11
#define MISO   12
#define SCK    13
#define SS     10
#define NRESET 7
#define TXEN   9
#define RXEN   8
#define LED1   A4
#define LED2   A5

// regmaps

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

#define payload_length 14
// ============================================
int i=0;
int pqd_flag=0;



/* ESTRUTURA DO PAYLOAD

| 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10 | 11 | 12 | 13 | 14 |
|TIME|ALTI|PRES|TEMP|ACCX|ACCY|ACCZ|GYRX|GYRY|GYRZ|MAGX|MAGY|MAGZ|ALTM|NULL|

*/


SFE_BMP180 bmp180;
char filename[7]="00.TXT";  //nome do arquivo inicial

int payload[14]={0,0,0,0,0,0,0,0,0,0,0,0,0};


void setup(){

	// configura os instrumentos

	Serial.begin(9600);
	Wire.begin();
	payload[1] = bmp180.altitude();
	payload[13]=payload[1];
	payload[2] = bmp180.getPressure();
	payload[3] = bmp180.getTemperatureC();

  Wire.beginTransmission(ACCEL);

	Wire.write(0x2D); // modo medição
	Wire.write(8);
  Wire.endTransmission(); 
	
	Wire.beginTransmission(ACCEL);
	Wire.write(0x21); // desativa modo double tap
	Wire.write(0);
	Wire.endTransmission();
	Wire.beginTransmission(ACCEL);
	Wire.write(0x22); // idem
	Wire.write(0);
	Wire.endTransmission();
	Wire.beginTransmission(ACCEL);
	Wire.write(0x23); // idem
	Wire.write(0);
	Wire.endTransmission();
	Wire.beginTransmission(ACCEL);
	Wire.write(0x31); // resolução completa, escala +/- 16g
	Wire.write(0x0B);  // 0.004 g por LSB
	Wire.endTransmission();


	
  
	Wire.beginTransmission(MAGNT); //start talking
  
	Wire.write(0x02); // Set the Register
	Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  
	int j = Wire.endTransmission();
	switch (j){
	case 0: Serial.println("MAGNT SETUP OK"); break;
	case 1: Serial.println("MAGNT SETUP TRANSMIT BUFF OVF"); break;
	case 2: Serial.println("MAGNT SETUP ADDRESS NACK"); break;
	case 3: Serial.println("MAGNT SETUP DATA NACK"); break;
	case 4: Serial.println("MAGNT SETUP BELGIUM"); break;
	}

  Wire.beginTransmission(GYROS);
	Wire.write(0x16);
	Wire.write(0x1C);
  int i = Wire.endTransmission();
  switch (i){
	case 0: Serial.println("GYROS SETUP OK"); break;
	case 1: Serial.println("GYROS SETUP TRANSMIT BUFF OVF"); break;
	case 2: Serial.println("GYROS SETUP ADDRESS NACK"); break;
	case 3: Serial.println("GYROS SETUP DATA NACK"); break;
	case 4: Serial.println("GYROS SETUP BELGIUM"); break;
  }

	// =========================

	byte temp = 0;  
	// Initializing serial port, usefull for debuging 
	Serial.begin(9600);
	// Initializing SPI pins
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(SCK,OUTPUT);
	pinMode(SS,OUTPUT);
	digitalWrite(SS,HIGH); //disabling LoRa module
	// Initializing other I/O pins
	pinMode(NRESET, OUTPUT);
	pinMode(TXEN, OUTPUT);
	pinMode(RXEN, OUTPUT);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	digitalWrite(NRESET,HIGH); // Deassert reset
	digitalWrite(TXEN,LOW); // Disabling tx antenna
	digitalWrite(RXEN,LOW); // Disabling rx antenna
	digitalWrite(LED1,LOW); 
	digitalWrite(LED2,LOW); 
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
	temp=SPSR;  //Reading and discarding previous data
	temp=SPDR;  //Reading and discarding previous data
	delay(10);


// reset lora

	digitalWrite(NRESET, LOW);
	delay(100);
	digitalWrite(NRESET, HIGH);
	delay(200);    

	// ================== config do LoRa

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
	SPIwriteRegister(LR_RegPayloadLength,0x38);		// payload 54 bytes
	SPIwriteRegister(LR_RegMaxPayloadLength,0x3C);	// payload max 60 bytes
	SPIwriteRegister(REG_LR_PADAC,0x87);            // transmission power 20dBm
	SPIwriteRegister(LR_RegHopPeriod,0x00);         // no frequency hoping
	SPIwriteRegister(REG_LR_DIOMAPPING2,0x01);      // DIO5=ModeReady,DIO4=CadDetected
	SPIwriteRegister(REG_LR_DIOMAPPING1,0x41); 		// DIO0=TxDone,DIO1=RxTimeout,DIO3=ValidHeader
	SPIwriteRegister(LR_RegFifoRxBaseAddr, 0x00);
	delay(200);
	SPIwriteRegister(LR_RegOpMode,0x81);            //LoRa standby
	Serial.println("\nCONFIG DONE");
}


void loop(){
	bool acc_rdy, mag_rdy, gyr_rdy = false;
	payload[0] = millis();
	payload[1] = bmp180.altitude();
	payload[2] = bmp180.getPressure();
	payload[3] = bmp180.getTemperatureC();

		if(payload[1]<(payload[13]-10.0f) && !pqd_flag)
		{
			digitalWrite(IGNITOR,HIGH);//abre o paraquedas
			pqd_flag=1;
			Serial.println ("abriu");
		}

	if(payload[1] > payload[13]){
	  payload[13] = payload[1];
	}

	acc_rdy = read_acc();
	mag_rdy = read_mag();
	gyr_rdy = read_gyro(); 

	if (acc_rdy && mag_rdy && gyr_rdy){
		transmit();
		/*for (int j = 0; j<14; j++){
			Serial.print(payload[j]);
			Serial.print("\t");

		}
		*/
		Serial.println("tx complete");
	}
}

byte SPIreadRegister(byte addr) {
	byte result;
	digitalWrite(SS, LOW);          // Select LoRa module
	SPDR = addr;                    // Send address & Start transmission. In READ mode bit 7 of address is always 0! for sx1276
	while (!(SPSR & (1<<SPIF)))     // Wait for transmission to finish
	{
	};
	result = SPDR;               // Discard first reading
	SPDR = 0x0;                     // Sending dummy byte to get the result
	while (!(SPSR & (1<<SPIF)))     // Wait for transmission to finish
	{
	};
	result = SPDR;               // Reading register value
	digitalWrite(SS, HIGH);         // Deselect LoRa module
	return (result);
}


byte SPIwriteRegister(byte addr,byte value) {
	byte result;
	digitalWrite(SS, LOW);          // Select LoRa module
	SPDR = addr | 0x80;              // Send address & Start transmission. In WRITE mode bit 7 of address is always 1! for sx1276
	while (!(SPSR & (1<<SPIF)))     // Wait for transmission to finish
	{
	};

	result = SPDR;                   // Discard first reading
	SPDR = value;                     // Sending byte 
	while (!(SPSR & (1<<SPIF)))       // Wait for transmission to finish
	{
	};

	result = SPDR;                   // Discard second reading
	digitalWrite(SS, HIGH);         // Deselect LoRa module
}


void SPIwriteBurst(unsigned char addr, int *ptr, unsigned char len){ 
	unsigned char i;
	unsigned char result;
	digitalWrite(SS, LOW);          // Select LoRa module
	SPDR = addr | 0x80;              // Send address & Start transmission. In WRITE mode bit 7 of address is always 1! for sx1276
	while (!(SPSR & (1<<SPIF)))     // Wait for transmission to finish
	{
	};
	result = SPDR;               // Discard first reading
	for (i=0; i <= len; i++){
		 SPDR = *ptr;                     // Sending bytes 
		 while (!(SPSR & (1<<SPIF)))     // Wait for transmission to finish
		 {
		 };
		 result = SPDR;               // Discard second reading  
		 //DEBUG DEBUG DEBUG
		 //Serial.print(*ptr, HEX);
		 //DEBUG DEBUG DEBUG
		 ptr++;
	} 
	//DEBUG DEBUG DEBUG
	//Serial.print("\n");
	// DEBUG DEBUG DEBUG        
	digitalWrite(SS, HIGH);         // Deselect LoRa module
}


void SPIreadBurst(unsigned char addr, unsigned char *ptr, unsigned char len){
	unsigned char i;
	unsigned char result;
	digitalWrite(SS, LOW);          // Select LoRa module
	SPDR = addr;                // Send address & Start transmission. In READ mode bit 7 of address is always 0! for sx1276
	while (!(SPSR & (1<<SPIF)))     // Wait for transmission to finish
	{
	};
	result = SPDR;               // Discard first reading
	for (i=0; i <= len; i++){
	   SPDR = 0;                     // Sending dummy byte to get the result
	   while (!(SPSR & (1<<SPIF)))   // Wait for transmission to finish
	   {
	   };
	   *ptr = SPDR;               // move pointer
	   ptr++;
	} 
		//DEBUG DEBUG DEBUG
		Serial.print("\n");
		// DEBUG DEBUG DEBUG        
		digitalWrite(SS, HIGH);         // Deselect LoRa module
}

void transmit(void) {

	SPIwriteRegister(LR_RegFifoTxBaseAddr, 0x00);
	unsigned char addr,temp;
	SPIwriteRegister(LR_RegIrqFlagsMask,0xf7);// enabling txdone	
	SPIwriteRegister(LR_RegIrqFlags,0xff);// clearing interupt
	SPIwriteRegister(LR_RegPayloadLength,payload_length);// payload length
	addr = SPIreadRegister(LR_RegFifoTxBaseAddr);// read TxBaseAddr        
	SPIwriteRegister(LR_RegFifoAddrPtr,addr);// TxBaseAddr->FifoAddrPtr          
	SPIwriteBurst(0x00,payload,payload_length);   // write data in fifo
	delay(100);
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
	Serial.println();
}



bool read_acc(void){

	Wire.beginTransmission(ACCEL);
	Wire.write(ACCXLO);
	Wire.write(ACCXHI);
	Wire.endTransmission();
	
	Wire.requestFrom(ACCEL, 2);
	if (Wire.available() <= 2){
		payload[4] = Wire.read() ;
		payload[4] |= Wire.read()<< 8;
	}
	
	// LER ACCEL EIXO Y
	
	Wire.beginTransmission(ACCEL);
	Wire.write(ACCYLO);
	Wire.write(ACCYHI);
	Wire.endTransmission();
	
	Wire.requestFrom(ACCEL, 2);
	if (Wire.available() <= 2){
		payload[5] = Wire.read();
		payload[5] |= Wire.read() << 8;
	}
	
	
	// LER ACCEL EIXO Z
	Wire.beginTransmission(ACCEL);
	Wire.write(ACCZLO);
	Wire.write(ACCZHI);
	Wire.endTransmission();
	
	Wire.requestFrom(ACCEL, 2);
	if (Wire.available() <= 2){
		payload[6] = Wire.read();
		payload[6] |= Wire.read()<<8;
	}

	return true;
}

bool read_mag(void){

	Wire.beginTransmission(MAGNT);
	Wire.write(0x03); //start with register 3.
	Wire.endTransmission();
	
	Wire.requestFrom(MAGNT, 6);
	if(6==Wire.available()){

		payload[10] = Wire.read()<<8; //MSB  x 
		payload[10] |= Wire.read(); //LSB  x
		payload[11] = Wire.read()<<8; //MSB  y
		payload[11] |= Wire.read(); //LSB y
		payload[12] = Wire.read()<<8; //MSB z
		payload[12] |= Wire.read(); //LSB z

	}

	return true;

}

bool read_gyro(void){

	Wire.beginTransmission(GYROS);
	Wire.write(GYR_R); //start with register 3.
	Wire.endTransmission();
	
	Wire.requestFrom(GYROS, 8);
	if(Wire.available() == 8){

		int temp = Wire.read()<<8;
		temp |= Wire.read(); // dummy descarta
		payload[7] = Wire.read()<<8; //MSB  x
		payload[7] |= Wire.read(); //LSB x
		payload[8] = Wire.read()<<8; //MSB y
		payload[8] |= Wire.read(); //LSB y
		payload[9] = Wire.read()<<8; //MSB z
		payload[9] |= Wire.read(); //LSB z

	}

	return true;
}


