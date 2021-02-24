/*
 * File:   main_MASTER.c
 * Author: orell
 *
 * Created on 20 de febrero de 2021, 05:29 PM
 */
//Master
// 

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

// incluimos las libreias que descargamos
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <pic16f887.h>
#include "lcd.h"
#include "spi.h"               

uint8_t SPI_ISR = 0;             
uint8_t select_esclavo = 0;      // Variable donde se guarda la seleccion del Esclavo 
uint8_t Esclavo1_SPI = 0;
uint8_t Esclavo2_SPI = 0;
uint8_t Esclavo3_SPI = 0;      // variables para recibir datos del SPI
char Esclavo1[20];
char Esclavo2[20];
char Esclavo3[20];             
//char str[20];                

void setup(void);           
void UART_ON(void);       //Funciones que se utilizran para el funcionamiento


void Uart_Text_Convert(char *text)  {
  for(int i=0;text[i]!='\0';i++){
    TXREG = (text[i]);
  __delay_ms(15);}
}// Funcion para convertir caracteres y mostrar en la pantalla

void UART_ON(void) {                          //  Tomado del DataSheet para comunicaion UART
    SPBRG = 25;
    BRGH = 1;
    SYNC = 0;
    SPEN = 1;
    TRISC7 = 1;
    TRISC6 = 0;
    TXSTAbits.TX9 = 0;
    CREN = 1;
    TXEN = 1;
}// Inicia el UART



void __interrupt() isr(void) {
    if (PIR1bits.SSPIF == 1) {  //Valor optenido por el SPI al activar la flag
        SPI_ISR = SSPBUF;
        SSPIF = 0;              //TURN OFF FLAG 
    }   
}



void setup(void) {          
    ANSEL = 0x00;
    ANSELH = 0x00;         //DIGITAL PORTS

    TRISA = 0x00;
    TRISB = 0x00;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;  //SPI Bits par el PICMASTER
    TRISD = 0000000;
    TRISE = 00000000;       

    PORTA = 00000000;           //CLEAN ALL PORTS
    PORTB = 00000000;
    PORTD = 00000000;
    PORTE = 00000000;       

    lcd_init();         // STATR UP da la LCD
    lcd_cmd(0x0c);      

    UART_ON();        //UART startUP

    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);    //STARTUP del SPI del PICMASTER
    PIE1bits.SSPIE = 1;// Interrupcion del SPI acticada
    PIR1bits.SSPIF = 0; 
    return;
}




void main(void) {
    setup();                    
    while (1) {                 
        lcd_clear();            //CLEAN SCREEN 
        if (select_esclavo == 2) { // DEPENDE DEL ESCLAVO SELESCIONADO LOS DEMAS SE APAGAN
            PORTAbits.RA0 = 0;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;   
            SSPBUF = 0xFF;      //CLOCK ON del Esclavo
            __delay_ms(10);
            Esclavo1_SPI = SSPBUF; //Buffer recibe el dato
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;  //TURN OF ALL ESCLAVOS
            select_esclavo = 0;   //RST CONTADOR
            
            // Para los demas IF es el mismo procedimiento pero con diferente esclavo
        }
        if (select_esclavo == 1) { 
            PORTAbits.RA0 = 1;  
            PORTAbits.RA1 = 0;
            PORTAbits.RA2 = 1;  
            SSPBUF = 0x0F;      
            __delay_ms(10);
            Esclavo2_SPI = SSPBUF; 
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;  
            select_esclavo++;     
        }
        if (select_esclavo == 0) { 
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 0;  
            SSPBUF = 0xFC;     
            __delay_ms(10);
            Esclavo3_SPI = SSPBUF; 
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;  
            select_esclavo++;     
        }

        lcd_write_string("Pot  Cont  Temp");    //DATOS ENVIADOS A LA PANTALLA
        lcd_move_cursor(1, 0);
        
        sprintf(Esclavo1, "%d", Esclavo1_SPI);    
        lcd_write_string(Esclavo1);              //pRINT DE LOS DATOS DEL PRIMER ESCLAVO
        lcd_move_cursor(1, 6);
        
        sprintf(Esclavo2, "%d", Esclavo2_SPI);    
        lcd_write_string(Esclavo2);              //PRINT DE LOS DARTOS DEL SEGUDNO ESCLAVO
        
        lcd_move_cursor(1, 11);
        sprintf(Esclavo3, "%d", Esclavo3_SPI);    
        lcd_write_string(Esclavo3);              //PRINT DATOS DEL TERCER ESCALVO
  
        Uart_Text_Convert("Pot  Cont  Temp");    //PEINT DE LOS TEXTOS UTILIZADOS
        TXREG = (0x0d);                        

        TXREG = (Esclavo1[0]);                   //TODOS LOS DATOS MOSTRADOS EN AL PANTALLA
        __delay_ms(15);
        TXREG = (Esclavo1[1]);
        __delay_ms(15);
        TXREG = (Esclavo1[2]);
        __delay_ms(15);  
        Uart_Text_Convert("   ");                  // BARRA ESPACIADORA
        
        // MISMO PROCEDIMEINTO PARA CADA ESCLAVO, SON 3
        

        TXREG = (Esclavo2[0]);                  
        __delay_ms(15);
        TXREG = (Esclavo2[1]);
        __delay_ms(15);
        TXREG = (Esclavo2[2]);
        __delay_ms(15);
        Uart_Text_Convert("    ");                      

        TXREG = (Esclavo3[0]);                  
        __delay_ms(15);
        TXREG = (Esclavo3[1]);
        __delay_ms(15);
        TXREG = (Esclavo3[2]);
        __delay_ms(15);
        TXREG = (' ');  
        TXREG = (0x0d);                         
    }
}














