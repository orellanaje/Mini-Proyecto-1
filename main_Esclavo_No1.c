/*
 * File:   main_Esclavo_No1.c
 * Author: JOSE EDUARO ORELLANA
 *
 * Created on 19 de febrero de 2021, 01:19 PM
 */

//Esclavo No. 1 
// Jose Orellana 18832

// Es esclavo 1 lee los datos del POT.



// PIC16F887 Configuration Bit Settings 
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ   4000000


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "spi.h"   

uint8_t ADC_DATA = 0;
uint8_t SPI_ISR = 0;

void setup(void);  //Creacion de Funciones 
void ADC_Read(void);



void setup(void) {
    ANSEL = 0x01;   //Confi puerto ANAG
    ANSELH = 0x00;
//LIMPIAR PUERTOS
    TRISA = 00000001;
    TRISB = 00000000;
    TRISC = 00001000;

    PORTA = 00000000;
    PORTB = 00000000;
    PORTC = 00000000;

    ADCON0 = 0b01000001; 
    ADCON1bits.ADFM = 0;// justicar derecha
    ADCON1bits.VCFG0 = 0; // V de Ref
    ADCON1bits.VCFG1 = 0; // V de Ref
    INTCONbits.GIE = 1; // Inicio Interrup
    INTCONbits.PEIE = 1;// Inicio Interrup
    PIE1bits.ADIE = 1;// Inicio Interrup
    PIR1bits.ADIF = 0; // Inicio Interrup
    
    return;
}// Vonfi PIC e inicar PIC

void main(void) {
    setup();
    
    while (1) {
        ADC_Read();
        PORTB = ADC_DATA; // Bucle para mostrar datos del ADC en el los puertos B
        
    }
}

void ADC_Read(void) {
    if (ADCON0bits.GO_DONE == 0) {
        asm("NOP");
        asm("NOP");
        asm("NOP"); // Cada NOP es una delay 
        asm("NOP");
        asm("NOP");  // Para que no haga nada durante la conversion ]
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        ADCON0bits.GO_DONE = 1; 
    }
}

void __interrupt() isr(void) {

    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0; // Interrup referecias DATA SHEET
        ADC_DATA = ADRESH;
    }
    if (SSPIF == 1) {       //interrupcion para recibir dato del SPI master
        SPI_ISR = SSPBUF;
        SSPIF = 0;}
}
    