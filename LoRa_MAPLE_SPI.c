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
#define payload_length 26

#include <SPI.h>

#define SPI1_NSS_PIN PA4    //SPI_1 Chip Select pin is PA4. You can change it to the STM32 pin you want.
#define SPI2_NSS_PIN PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port
int16_t payload[14]={0,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
void setup() {

  // Setup SPI 2
  SPI_2.begin(); //Initialize the SPI_2 port.
  SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
  pinMode(SPI2_NSS_PIN, OUTPUT);

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


}

void loop() {

  	
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
		delay(10);
	}
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

  delayMicroseconds(10);    //Delay 10 micro seconds.
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