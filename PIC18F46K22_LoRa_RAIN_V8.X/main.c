/*
 * File:   main.c
 * Author: andym
 * 
 * LoRa Rain Gauge
 * 
 * Transmits when a rain tip occurs or every 2 minutes if there is no rainfall.
 * Keeps a count of the total tips which is transmitted (32 bit unsigned integer).
 * The counter is not reset unless the power is removed.
 * 
 * Sleep current consumption is 12µA with standard PIC18F46K22.
 * Could reduce to 1µA using PIC18LF46K22.
 * Runs on 3V battery.
 * 
 * AN0 reads battery voltage through a resistor divider (30k/10k) with 1.024V internal reference
 * AN1 reads local temperature through 10k NTC and 10k resistor as a divider from 3.3V
 * RB2 (INT1) is rain tip input.
 *
 * Created on 19 March 2021, 13:55
 * Version 2, 3rd April 2021, 20:44  Added internal temperature measurement and battery measurement.
 * Version 3, 5th April 2021, 12:20  Changed frequency to 866.5MHz, changed sync word to 0x55, added 8-byte address
 * Version 4, 16th May 2021, 19:37  Added UVLO to prevent transmitter operation below 2.4V.  Should stop repeated starts.
 * Version 4, 11th June 2021, 20:40  Changed interrupt pin to RB1 (INT1)
 * Version 7, 8th Aug 2021, 15:26 Branch from correct working version 4 to add CRC16 to data stream.
 * Version 8, 18th Sept 2021, 12:04  Move to common 50 byte packet format
 */


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic18f46k22.h>
#include "config.h"
#include "defines.h"
#include "usart2.h"
#include "LoRa.h"
#include "CRC16.h"

#define DEBUG 0
#define TX_FREQ 866.5
#define SYNC_WORD 0x55
#define BATT_UVLO 2000
#define BATT_UVLO_ATOD BATT_UVLO/4
#define DATA_PACKET_LENGTH 50
#define ID0 0x00
#define ID1 0x01
#define SOFTWARE_VERSION 0x08

/**
 * Functions
 */
void configureIO(void);
void disablePeripherals(void);
void transmitData(void);
uint16_t readBattery();
uint16_t readTemperature();
void setupAtoD();

/**
 * Variables
 */
volatile uint32_t tips=0;
uint32_t messageCount=0; //Increments by 1 for each message transmitted.
uint8_t txData[DATA_PACKET_LENGTH]; //Transmit buffer
uint16_t batt=0; //Battery voltage A to D reading
uint16_t temp=0; //Temperature A to D reading
uint8_t address[8] = {0xE6,0xBA,0x08,0xFB,0x3A,0x4F,0x5E,0xCE}; //This should be unique

void main(void) {
    INTCON2bits.INTEDG1=0; //Interrupt on falling edge
    start:
    configureIO();
    setupAtoD(); //Setup to read AN0 (reads supply voltage [battery])
    __delay_ms(5); //Wait for things to power up
    if(DEBUG){
        printf("LoRa Rain Gauge\r\n");
    }
    batt = readBattery();
    temp = readTemperature();
    if(DEBUG){
        printf("BATT %d\r\n", batt);
        printf("TEMP %d\r\n", temp);
    }
    if(batt>BATT_UVLO_ATOD){
        transmitData();
    }
    else{
        //Flash the red LED 3 times
        RED_LED=1; //Red LED on
        __delay_ms(300);
        RED_LED=0;
        __delay_ms(300);
        RED_LED=1; //Red LED on
        __delay_ms(300);
        RED_LED=0;
        __delay_ms(300);
        RED_LED=1; //Red LED on
        __delay_ms(300);
        RED_LED=0;
        __delay_ms(300);
    }
    if(DEBUG){
        printf("Message count %lu\r\n", messageCount);
        printf("Rain tips %lu\r\n", tips);
    }
    disablePeripherals();
    if(DEBUG){
        printf("Sleeping\r\n");
    }
    SLEEP();

    goto start;
}

void configureIO(){
    PMD0bits.UART2MD=0; //Turn on UART2
    PMD0bits.SPI2MD=0; //Turn on SPI2
    PMD2bits.ADCMD=0; //Turn on ADC
    ANSELAbits.ANSA2=0; //Analogue off
    TRISAbits.RA2=0; //Output
    LATAbits.LATA2=0; //External circuitry on
    ANSELEbits.ANSE1=0; //Turn off analogue on RE1
    ANSELEbits.ANSE2=0; //Turn off analogue on RE2
    ANSELBbits.ANSB4=0; //Turn off analogue on RB4
    TRISEbits.RE1=0; //Green LED for status
    TRISEbits.RE2=0; //Red LED for status
    ANSELBbits.ANSB1=0; //Turn off analogue on RB1
    TRISBbits.RB1=1; //RB1 is input (INT1)
    USART2_Start(BAUD_57600); //Start USART2
    //INTCONbits.INT0IE=1; //Enable interrupt on INT0 pin
    
    //INTCONbits.INT0IF=0; //Clear INT0 flag
    INTCON3bits.INT1E=1; //Enable interrupt on INT1 pin
    INTCON3bits.INT1F=0; //Clear INT1 flag
    INTCONbits.GIE=1; //Enable global interrupts
}

void disablePeripherals(){
    ADCON0bits.ADON=0; //Turn off A to D module
    //Set all pins as outputs
    TRISA=0;
    TRISB=0x02; //Set all outputs except RB1
    TRISC=0;
    TRISD=0;
    TRISE=0;
    LATA=0;
    LATB=0;
    LATC=0;
    LATD=0;
    LATE=0;
    LATAbits.LA2=1; //Turn off external peripherals
    //Deal with SDI in case LoRa module is driving it
    TRISDbits.RD1=1; //SDIx must have corresponding TRIS bit set (input)
    ANSELDbits.ANSD1=0; //Input buffer enabled
    LATDbits.LATD3=1; //Set SS high so LoRa chip is not selected
    PMD0bits.UART2MD=1; //Turn off UART2
    PMD0bits.UART1MD=1; //Turn off UART1
    PMD0bits.TMR6MD=1; //Turn off timer 6
    PMD0bits.TMR5MD=1; //Turn off timer 5
    PMD0bits.TMR4MD=1; //Turn off timer 4
    PMD0bits.TMR3MD=1; //Turn off timer 3
    PMD0bits.TMR2MD=1; //Turn off timer 2
    PMD0bits.TMR1MD=1; //Turn off timer 1
    PMD0bits.SPI2MD=1; //Turn off SPI2
    PMD0bits.SPI1MD=1; //Turn off SPI1
    PMD1=0xFF; //Turn off all peripherals in PMD1 (including MSSP2 for SPI2)
    PMD2=0xFF; //Turn off all peripherals in PMD2 (ADC, comparators, CTMU)
}

void transmitData(){
    if(DEBUG){
        printf("Transmitting...\r\n");
    }
    
    txData[0] = DATA_PACKET_LENGTH;
    txData[1] = ID0; //Copy in the ID
    txData[2] = ID1; //Copy in the ID
    for(uint8_t i=0;i<8;i++){
        txData[i+3] = address[i]; //Copy in the address
    }
    txData[11] = SOFTWARE_VERSION;
    
    //Message count
    txData[12]=(uint8_t)((messageCount>>24)&0xFF); //MSB
    txData[13]=(uint8_t)((messageCount>>16)&0xFF); //Upper middle
    txData[14]=(uint8_t)((messageCount>>8)&0xFF); //Lower middle
    txData[15]=(uint8_t)((messageCount & 0xFF)); //LSB
    
    //Supply voltage value (10-bit in 2 bytes)
    txData[16]=(uint8_t)((batt>>8)&0xFF); //MSB
    txData[17]=(uint8_t)(batt & 0xFF); //LSB
    
    //Sensor local temperature value (16-bit)
    txData[18]=(uint8_t)((temp>>8)&0xFF); //MSB
    txData[19]=(uint8_t)(temp & 0xFF); //LSB
    
    //V1 Voltage (0)
    txData[20] = 0;
    txData[21] = 0;
    
    //V2 Voltage (0)
    txData[22] = 0;
    txData[23] = 0;
    
    
    //Rain tip count
    txData[24]=(uint8_t)((tips>>24)&0xFF); //MSB
    txData[25]=(uint8_t)((tips>>16)&0xFF); //Upper middle
    txData[26]=(uint8_t)((tips>>8)&0xFF); //Lower middle
    txData[27]=(uint8_t)((tips & 0xFF)); //LSB
    
    //Fill the rest of the data area with 0
    for(uint8_t i=28;i<48;i++){
        txData[i] = 0;
    }
    
    //Calculate CRC16 and add to end of message
    unsigned short int calcCRC = CRC16(txData, DATA_PACKET_LENGTH-2);
    txData[49] = (calcCRC&0xFF00u)>>8u; //MSB
    txData[48] = (calcCRC&0xFF); //LSB

    
    //Set the transmitter up and send the data
    LoRaStart(TX_FREQ, SYNC_WORD); //Configure module
    if(DEBUG){
        printf("TXF: %f\r\n", LoRaGetFrequency());
    }
    LoRaClearIRQFlags();
    RED_LED=1; //Red LED on
    LoRaTXData(txData, DATA_PACKET_LENGTH); //Send data
    if(DEBUG){
        printf("Wait for end of transmission...\r\n");
    }
    uint8_t j=0;
    for(j=0;j<50;j++){
        uint8_t flags = LoRaGetIRQFlags();
        //printf("IRQ %d %d \r\n",j, flags);
        if(flags>0){
            break;
        }
        __delay_ms(10); //We are done with transmission
    }
    if(DEBUG){
        if(j>48){
            printf("TX Fail\r\n");
        }
        else{
            printf("Done.\r\n");
        }
    }
    LoRaSleepMode(); //Put module to sleep
    __delay_ms(10);
    messageCount++;
    RED_LED=0; //Red LED off
}

/**
 * Reads the supply voltage A to D
 */
uint16_t readBattery(){
    ADCON1bits.PVCFG=0b10; //A/D Vref+ connected to internal reference FVR BUF2
    ADCON0bits.GO_NOT_DONE=1; //Start the A to D process
    while(ADCON0bits.GO_NOT_DONE){
        //Wait for conversion to complete (about 15µs)
    }
    uint16_t result = ADRESH * 256 + ADRESL; //Read A to D result
    return result;
}

uint16_t readTemperature(){
    ADCON1bits.PVCFG=0; //A/D Vref+ connected to Vdd
    //Select channel 1 for A to D
    ADCON0bits.CHS=1;
    ADCON0bits.GO_NOT_DONE=1; //Start the A to D process
    while(ADCON0bits.GO_NOT_DONE){
        //Wait for conversion to complete (about 15µs)
    }
    uint16_t result = ADRESH * 256 + ADRESL; //Read A to D result
    return result;
}

void setupAtoD(){
    //Setup AN0 for a to d converter (RA0)
    
    //Set ANSELbit to disable digital input buffer
    ANSELAbits.ANSA0=1;
    ANSELAbits.ANSA1=1;
    
    //Set TRISXbit to disable digital output driver
    TRISAbits.RA0=1;
    TRISAbits.RA1=1;
    
    //Set voltage references
    ADCON1bits.PVCFG=0; //A/D Vref+ connected to Vdd
    ADCON1bits.NVCFG=0; //A/D Vref- connected to internal signal AVss
    VREFCON0bits.FVRS=0b01; //Fixed voltage reference is 1.024V
    VREFCON0bits.FVREN=1; //Enable internal reference
    
    //Select channel 0 for A to D
    ADCON0bits.CHS=0;
    
    //Set A to D acquisition time
    ADCON2bits.ACQT=0b010; //Tacq = 4 Tad (4µs)
    
    //Set A to D clock period
    ADCON2bits.ADCS=0b110; //Clock period set to Fosc/64 = 1µs (64MHz clock)
    
    //Set result format
    ADCON2bits.ADFM = 1; //Data is mostly in the ADRESL register with 2 bits in the ADRESH register
    
    //Turn on the A to D module
    ADCON0bits.ADON=1;
}

void __interrupt() Isr(void){
    if(INTCON3bits.INT1F==1){
        tips++; //Increase rain tip count
        INTCON3bits.INT1F=0; //Clear INT1 flag
        RED_LED=1;
    }
}


