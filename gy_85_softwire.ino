//Portando o Legado pro maple mini ARM-Cortex M3

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


#include <SoftWire.h>

SoftWire SWire(PB6, PB7);
/*

	Well, they are supposed to die. Then you come in here and buy another!

*/

int16_t MAGx,GYRx,ACCx;
int16_t MAGy,GYRy,ACCy;
int16_t MAGz,GYRz,ACCz;
int16_t temp;
float tempC;



void setup(){
	Serial.begin(9600);
	SWire.begin();
	pinMode(33,OUTPUT);
	SWire.beginTransmission(MAGNT); //start talking
		
	SWire.write(0x02); // Set the Register
	SWire.write(0x00); // Tell the HMC5883 to Continuously Measure
	
	int j = SWire.endTransmission();
		switch (j){
		case 0: Serial.println("MAGNT SETUP OK"); break;
		case 1: Serial.println("MAGNT SETUP TRANSMIT BUFF OVF"); break;
		case 2: Serial.println("MAGNT SETUP ADDRESS NACK"); break;
		case 3: Serial.println("MAGNT SETUP DATA NACK"); break;
		case 4: Serial.println("MAGNT SETUP BELGIUM"); break;
	}

// ====================================================================

	SWire.beginTransmission(ACCEL);

		SWire.write(0x2D); // modo medição
		SWire.write(8);
		
		SWire.write(0x21); // desativa modo double tap
		SWire.write(0);
		SWire.write(0x22); // idem
		SWire.write(0);
		SWire.write(0x23); // idem
		SWire.write(0);
		SWire.write(0x31); // resolução completa, escala +/- 2g
		SWire.write(0x08);  

		int k = SWire.endTransmission();
		switch (k){
		case 0: Serial.println("ACCEL SETUP OK"); break;
		case 1: Serial.println("ACCEL SETUP TRANSMIT BUFF OVF"); break;
		case 2: Serial.println("ACCEL SETUP ADDRESS NACK"); break;
		case 3: Serial.println("ACCEL SETUP DATA NACK"); break;
		case 4: Serial.println("ACCEL SETUP BELGIUM"); break;
	}

// ===================================================================


	SWire.beginTransmission(GYROS);
		SWire.write(0x16);
		SWire.write(0x1C);
	int i = SWire.endTransmission();
	switch (i){
		case 0: Serial.println("GYROS SETUP OK"); break;
		case 1: Serial.println("GYROS SETUP TRANSMIT BUFF OVF"); break;
		case 2: Serial.println("GYROS SETUP ADDRESS NACK"); break;
		case 3: Serial.println("GYROS SETUP DATA NACK"); break;
		case 4: Serial.println("GYROS SETUP BELGIUM"); break;
	}

}


void loop(){
	digitalWrite(33,!digitalRead(33));
	unsigned long int t0 = micros();
// ------------------ LER MAGNETOMETRO -----------------------------
	
	SWire.beginTransmission(MAGNT);
	SWire.write(0x03); //start with register 3.
	SWire.endTransmission();
	
	SWire.requestFrom(MAGNT, 6);

	if(6<=SWire.available()){

		MAGx = SWire.read()<<8; //MSB  x 
		MAGx |= SWire.read(); //LSB  x
		MAGy = SWire.read()<<8; //MSB  z
		MAGy |= SWire.read(); //LSB z
		MAGz = SWire.read()<<8; //MSB y
		MAGz |= SWire.read(); //LSB y
	}
//--------------------------------------------------

// ------------------ LER GYROS -----------------------------

	SWire.beginTransmission(GYROS);
	SWire.write(GYR_R); //start with register 3.
	SWire.endTransmission();
	
	SWire.requestFrom(GYROS, 8);

	if(SWire.available() == 8){

		temp = SWire.read()<<8; //MSB  x 
		temp |= SWire.read(); //LSB  x
		GYRx = SWire.read()<<8; //MSB  z
		GYRx |= SWire.read(); //LSB z
		GYRy = SWire.read()<<8; //MSB y
		GYRy |= SWire.read(); //LSB y
		GYRz = SWire.read()<<8; //MSB y
		GYRz |= SWire.read(); //LSB y

	}

	tempC = temp/280 + 82.142857;
//--------------------------------------------------

// ------------------ LER ACELEROMETRO -----------------------------

	SWire.beginTransmission(ACCEL);
	SWire.write(ACCXLO);
	SWire.write(ACCXHI);
	SWire.endTransmission();
	
	SWire.requestFrom(ACCEL, 2);
	if (SWire.available() <= 2){
		ACCx = SWire.read() ;
		ACCx |= SWire.read()<< 8;
	}
	
	// LER ACCEL EIXO Y
	
	SWire.beginTransmission(ACCEL);
	SWire.write(ACCYLO);
	SWire.write(ACCYHI);
	SWire.endTransmission();
	
	SWire.requestFrom(ACCEL, 2);
	if (SWire.available() <= 2){
		ACCy = SWire.read();
		ACCy |= SWire.read() << 8;
	}
	
	
	// LER ACCEL EIXO Z

	SWire.beginTransmission(ACCEL);
	SWire.write(ACCZLO);
	SWire.write(ACCZHI);
	SWire.endTransmission();
	
	SWire.requestFrom(ACCEL, 2);
	if (SWire.available() <= 2){
		ACCz = SWire.read();
		ACCz |= SWire.read()<<8;
	}
unsigned long int tf = micros();

	unsigned long int dt = (tf - t0);




// --------------------------------- Cuspir resultados
	Serial.print(ACCx);
	Serial.print(",");
	Serial.print(ACCy);
	Serial.print(",");
	Serial.print(ACCz);
	Serial.print(",");
	Serial.print(GYRx);
	Serial.print(",");
	Serial.print(GYRy);
	Serial.print(",");
	Serial.print(GYRz);
	Serial.print(",");
	Serial.print(MAGx);
	Serial.print(",");
	Serial.print(MAGy);
	Serial.print(",");
	Serial.print(MAGz);
	Serial.print(",");
	Serial.print(dt);
	Serial.print(",");
	Serial.print(tempC);
	Serial.println(" ");

}