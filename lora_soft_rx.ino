/* 
NiceRF LoRa1276 Module Arduino NANO Clone V3
NANO        LoRa1276
D11 MOSI    6  MOSI
D12 MISO    5  MISO
D13 SCK     4  SCK      RXQUEEN
D10         7  NSS
by absolutelyautomation.com
*/
// using SPI library:
#include <SPI.h>
// Digital pins definition
#define MOSI   11
#define MISO   12
#define SCK    13
#define SS     10
#define NRESET 7
/*
#define TXEN   9
#define RXEN   8 TXEN / RXEN DEPRECADO VER 2.0
*/
// register definition
#define LR_RegFifo                       0x00
#define LR_RegOpMode                     0x01
#define LR_RegFrMsb                      0x06 //msb frequencia portadora
#define LR_RegFrMid                      0x07 //byte intermediario freq portadora
#define LR_RegFrLsb                      0x08 //lsb freq portadora
#define LR_RegPaConfig                   0x09 //controle amp de potencia
#define LR_RegPaRamp                     0x0A //controle do ramp do pa
#define LR_RegOcp                        0x0B //protecao sobrecorrente registrador
#define LR_RegLna                        0x0C 
#define LR_RegFifoAddrPtr                0x0D //ponteiro da fifo
#define LR_RegFifoTxBaseAddr             0x0E //regiao tx da fifo
#define LR_RegFifoRxBaseAddr             0x0F //regiao rx da fif0
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
#define REG_LR_TCXO                      0x4B
#define REG_LR_PADAC                     0x4D
#define REG_LR_FORMERTEMP                0x5B
#define REG_LR_AGCREF                    0x61
#define REG_LR_AGCTHRESH1                0x62
#define REG_LR_AGCTHRESH2                0x63
#define REG_LR_AGCTHRESH3                0x64

// payload length
#define payload_length  7
// tx packet
unsigned char txbuf[payload_length]={'t','e','s','t','i','n','g'};
// rx packet
unsigned char rxbuf[30];
unsigned long int packets_received = 0;
byte rxflag = 0;
// Initialization
void setup() {
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
	digitalWrite(NRESET,HIGH); // Deassert reset

	/* Initializing SPI registers
	description of every SPCR register bits
	| 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
	| SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
	SPIE - Enable SPI interupt (logic 1)
	SPE  - Enable SPI (logic 1)
	DORD - Send Least Significant Bit (LSB) first (logic 1) , Send Most Significant Bit (MSB) first (logic 0) 
	MSTR - Enable SPI master mode (logic 1), slave mode (logic 0)
	CPOL - Setup clock signal inactive in high (logic 1), inactive in low (logic 0)
	CPHA - Read data on Falling Clock Edge (logic 1), Rising edge (logic 0)
	SPR1 y SPR0 - Setup SPI data rate: 00 Fastest (4MHz), 11 Slowest (250KHz)
	// SPCR = 01010011
	//interupt disabled,spi enabled,most significant bit (msb) first,SPI master,clock inactive low,
	data fech rising clock edge, slowest data rate*/
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
	temp=SPSR;  //Reading and discarding previous data
	temp=SPDR;  //Reading and discarding previous data
	delay(100);
	Serial.println("---------- setup ok ----------");
	reset_sx1276(); 
	Config_SX1276();  
	SPIwriteRegister(LR_RegOpMode, 0x87); // coloca o rádio em modo de detecção
	unsigned int irrqflags = SPIreadRegister(LR_RegIrqFlags);
	Serial.println("\nWaiting for Channel Activity");
	while ((irrqflags&1))
	{
		irrqflags = SPIreadRegister(LR_RegIrqFlags);
		Serial.print(".");
		if(irrqflags&4){Serial.println("No Signal. Retrying....");}
	}

	if (irrqflags&1){init_rx();}
	
}
void loop() {
	int status = SPIreadRegister(LR_RegModemStat);	
	int flag = 0;
	byte valid = SPIreadRegister(0x1C);
	char temp;
	Serial.println("\n =.=.=.=.=.=.=.=.=.=.=.=.=.= ");
	unsigned char payload_size;
	unsigned int irrqflags = SPIreadRegister(LR_RegIrqFlags);                 // read interupt
	// analisa retorno do status
	if (status&32){Serial.println("\n--------- modem clear ---------");}
	if (status&16){Serial.println("\n--------- header info valid ---------");}
	if (status&4){Serial.println("\n--------- rx underway ---------");}
	if (status&2){Serial.println("\n--------- sync ok ---------");}
	if (status&1){Serial.println("\n--------- signal detected ---------");}

	if (irrqflags & 64){
		if ((valid&0x40) && (!(irrqflags&32))){
		Serial.println("\nA"); //debug
		SPIwriteRegister(LR_RegIrqFlags,0xff);// clear interupt
		Serial.println("\nB"); //debug
		temp = SPIreadRegister(LR_RegFifoRxCurrentaddr);// read RxCurrentaddr
		Serial.println("\nC");//debug
		SPIwriteRegister(LR_RegFifoAddrPtr,temp);// RxCurrentaddr -> FiFoAddrPtr
		Serial.println("\nD");// debug
		payload_size = SPIreadRegister(LR_RegRxNbBytes);// read packet size
		Serial.println("\n========== message ===============");//debug
		for (int j = 0; j < payload_length; j++){
			int a = SPIreadRegister(LR_RegFifo); // lê a posição apontada pelo RegFifoAddrPtr
			// *** ponteiro incrementa automaticamente ***
			Serial.print(a);
			}
		Serial.println("\n========== end of message ===============");//debug
		SPIwriteRegister(LR_RegFifoAddrPtr, 0x00);//resetar o ponteiro pra base do buffer
		}
	}
	else {Serial.println("\n INVALID CRC? READ FLAGS");}
	if (!(valid&40)) {N}
	int ValidPackets = (SPIreadRegister(0x16)<<8) | SPIreadRegister(0x17);
	Serial.print("\n>>>> VALID PACKETS = ");
	Serial.print(ValidPackets);
	Serial.println("\n ==================== FLAGS ==================== ");
	if (irrqflags&64){Serial.print(" |RX DONE|");}
	if (irrqflags&32){Serial.print(" |CRC ERROR|");}
	if (irrqflags&16){Serial.print(" |VALID HEADER|");}
	if (irrqflags&4){Serial.print(" |CHANNEL ACTIVITY FINISHED|");}
	if (irrqflags&2){Serial.print(" |CHANNEL CHANGE|");}
	if (irrqflags&1){Serial.print(" |CHANNEL ACTIVITY DETECTED|");}
	Serial.println("\n =============================================== ");
	SPIwriteRegister(LR_RegIrqFlags, 0xff); // reseta as flags
	SPIwriteRegister(LR_RegFifoAddrPtr, 0x00);//resetar o ponteiro pra base do buffer
	delay(500);
}
			


/*
void storeMessage(){
	byte baseaddr = SPIreadRegister(LR_RegFifoRxBaseAddr);
	char *ptr = SPIreadRegister(LR_RegFifoRxCurrentaddr);
	for (int j = 0; j < sizeof(rxbuf); j++){
		rxbuf[j] = SPIreadRegister(LR_RegFifoAddrPtr+j)
	}
}
*/
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


void SPIwriteBurst(unsigned char addr, unsigned char *ptr, unsigned char len){ 
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
	 Serial.print(*ptr, HEX);
	 //DEBUG DEBUG DEBUG
	 ptr++;
	} 
	//DEBUG DEBUG DEBUG
	Serial.print("\n");
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
	 result = SPDR;
	 *ptr = SPDR;               // move pointer
	 ptr++;
} 
	//DEBUG DEBUG DEBUG
	Serial.print("\n");
	// DEBUG DEBUG DEBUG        
	digitalWrite(SS, HIGH);         // Deselect LoRa module

}


void reset_sx1276(void){

	digitalWrite(NRESET, LOW);
	delay(100);
	digitalWrite(NRESET, HIGH);
	delay(200);    

}  

void Config_SX1276(void){

	// put in sleep mode to configure
	SPIwriteRegister(LR_RegOpMode,0x00);
	// sleep mode, high frequency
	delay(1000);

	SPIwriteRegister(REG_LR_TCXO,0x09);// external crystal
	SPIwriteRegister(LR_RegOpMode,0x80);// LoRa mode, high frequency
	SPIwriteRegister(LR_RegFrMsb,0x6C);
	SPIwriteRegister(LR_RegFrMid,0x40);
	SPIwriteRegister(LR_RegFrLsb,0x13);       // frequency：433 MHz
	SPIwriteRegister(LR_RegPaConfig,0xFF);   // max output power PA_BOOST enabled
	//SPIwriteRegister(LR_RegPaRamp,0x08);			//pa ramp = 50 us
	SPIwriteRegister(LR_RegOcp,0x0B);// close over current protection  (ocp)
	SPIwriteRegister(LR_RegLna,0x23);// Enable LNA
	SPIwriteRegister(LR_RegModemConfig1,0x72);   // signal bandwidth：125kHz, error coding= 4/5, explicit header mode
	SPIwriteRegister(LR_RegModemConfig2,0x77);// spreading factor：7, CRC on
	SPIwriteRegister(LR_RegModemConfig3,0x08);// LNA? optimized for low data rate
	SPIwriteRegister(LR_RegSymbTimeoutLsb,0xFF);     // max receiving timeout
	SPIwriteRegister(LR_RegPreambleMsb,0x00);
	SPIwriteRegister(LR_RegPreambleLsb,16);          // preamble 16 bytes  
	SPIwriteRegister(REG_LR_PADAC,0x87);             // transmission power 20dBm
	SPIwriteRegister(LR_RegHopPeriod,0x00);          // no frequency hoping
	SPIwriteRegister(REG_LR_DIOMAPPING2,0x01);       // DIO5=ModeReady,DIO4=CadDetected
	SPIwriteRegister(REG_LR_DIOMAPPING1,0x00);       // DIO0=RXdone,DIO1=RXtimeout
	SPIwriteRegister(LR_RegFifoRxBaseAddr, 0x00);
	delay(200);
	SPIwriteRegister(LR_RegOpMode,0x81);             // standby mode, high frequency

}
/*
void mode_tx(void) {
	
	unsigned char addr,temp;
	SPIwriteRegister(REG_LR_DIOMAPPING1,0x41); 
	// DIO0=TxDone,DIO1=RxTimeout,DIO3=ValidHeader
	SPIwriteRegister(LR_RegIrqFlags,0xff);
	// clearing interupt
	SPIwriteRegister(LR_RegIrqFlagsMask,0xf7);
	// enabling txdone
	SPIwriteRegister(LR_RegPayloadLength,payload_length);
	// payload length
	addr = SPIreadRegister(LR_RegFifoTxBaseAddr);
	// read TxBaseAddr        
	SPIwriteRegister(LR_RegFifoAddrPtr,addr);
	// TxBaseAddr->FifoAddrPtr          
	SPIwriteBurst(0x00,txbuf,payload_length);   // write data in fifo
	SPIwriteRegister(LR_RegOpMode,0x03);
	// mode tx, high frequency
	digitalWrite(LED1, !digitalRead(LED1));
	temp=SPIreadRegister(LR_RegIrqFlags);
	// read interput flag
	while(!(temp&0x08)){          // wait for txdone flag

	temp=SPIreadRegister(LR_RegIrqFlags);
	}

	SPIwriteRegister(LR_RegIrqFlags,0xff);
	// clearing interupt
	SPIwriteRegister(LR_RegOpMode,0x01);  
	// standby mode, high frequency

}
*/
void init_rx(void){

	unsigned char addr; 
	//DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE
	SPIwriteRegister(LR_RegOpMode, 0x85);//free run rx
	SPIwriteRegister(LR_RegIrqFlags,0xff);// clearing interupt
	SPIwriteRegister(LR_RegIrqFlagsMask,0x00);// disable rx timeout
	
	addr = SPIreadRegister(LR_RegFifoRxBaseAddr);// read RxBaseAddr
	
	SPIwriteRegister(LR_RegFifoAddrPtr,addr);// RxBaseAddr->FifoAddrPtr
	rxflag = 1;
	Serial.println("---------- RX FREE RUN ENABLED ----------");

}