/*
 * File:   main_Esclavo_No3.c
 * Author: JOSE EDUARDO ORELLANA
 *
 * Created on 19 de febrero de 2021, 8:03PM
 */

//Esclavo 3
//EZ :v
//  SEMAFORO QUE INDICARA CON RANGOS LA TEMP LEIDA 
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

uint8_t TempDATA = 0; // Variable para gaurdar datos 

void Semaforo(void);
void setup(void);
void ADC_Read(void);    //Creacion de funciones


void __interrupt() isr(void) {

    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0; //Justificar derecha 
        TempDATA = ADRESL;
        TempDATA = TempDATA >> 1;
    }

} // Linea de interrupciones

void ADC_Read(void) {
    if (ADCON0bits.GO_DONE == 0) {
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");  // No hacer nada mientras convierte
        asm("NOP");
        asm("NOP");
        asm("NOP");
        ADCON0bits.GO_DONE = 1; // iniciar cconversion
    }
}// Config para las lecturas del ADC


void Semaforo(void){
    if (TempDATA < 25)
    {
        PORTEbits.RE2 = 1;
        PORTEbits.RE1 = 0;
        PORTEbits.RE0 = 0;
    }
    else if ((TempDATA > 25) && (TempDATA < 36))// Semaforo sencillo con los rangos establecidos
    {
        PORTEbits.RE2 = 0;
        PORTEbits.RE1 = 1;
        PORTEbits.RE0 = 0;
    }
    else if (TempDATA > 36)
    {
        PORTEbits.RE2 = 0;
        PORTEbits.RE1 = 0;
        PORTEbits.RE0 = 1;
    }
    else
    {
        return;
    }
} //Confi Semaforo


void setup(void) {
    ANSEL = 00000001;
    ANSELH = 00000000;

    TRISA = 00000001;
    TRISB = 00000000;
    TRISE = 00000000;

    PORTA = 00000000;
    PORTB = 00000000;
    PORTE = 00000000;

    ADCON0 = 0b01000001; //Confi "canal 0"
    ADCON1 = 0b10001000;
    
    ADCON1bits.ADFM = 1;
    // VCC y GND de referencia
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0; // 10us para empezar conversor

    INTCONbits.GIE = 1;  
    INTCONbits.PEIE = 1;
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0; //Interrupciones para el ADC
    
    return;
}

void main(void) {
    setup();
    
    while (1) {
        ADC_Read();
        Semaforo();
        PORTB = TempDATA;
    }
}





