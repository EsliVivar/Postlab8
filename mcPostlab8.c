/*
 * File:   mcPostlab8.c
 * Author: emviv
 *
 * Created on 23 de abril de 2022, 07:40 PM
 */

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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000
#define _tmr0_value 217       // 5 ms tiempo de retardo

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char display[10]={
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01101111,
}; // Se hizo un STRUCT para guardar para hacer la traducciones a los display

uint8_t banderas;
uint8_t unidades = 0;
uint8_t decenas = 0 ; 
uint8_t centenas = 0 ;
uint8_t contador = 0;
uint8_t ponteciometro;


/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void isr (void);                 //Interrupciones
int divisiones(void);            //Donde se hara las divisiones de unidade, decenas y centenas

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if(T0IF == 1){             // Fue interrupción del TMR0
        PORTE=0X00;            // Limpia para poder habilitar que display encender
        
        
        if (banderas == 0b00000000){  //Si esta en la bandera 0 entonces se enciende el display de las unidades
            PORTEbits.RE2 = 0;
            PORTEbits.RE0 = 1;
            PORTD = display[unidades];
            banderas = 0b00000001;    // Se limpia la bandera
        }
    
        else if (banderas == 0b00000001){ //Si esta en la bandera 0 entonces se enciende el display de las decenas
            PORTEbits.RE0 = 0;
            PORTEbits.RE1 = 1;
            PORTD = display[decenas];
            banderas = 0b00000010;        
        }
        else if (banderas == 0b00000010){ //Si esta en la bandera 0 entonces se enciende el display de las centenas
            PORTEbits.RE1 = 0;
            PORTEbits.RE2 = 1;
            PORTD = display[centenas];
            banderas = 0b00000000;            
        } 
        INTCONbits.T0IF = 0;
        TMR0 = _tmr0_value;
    }
    
    if(PIR1bits.ADIF){              // Fue interrupción del ADC?
        if(ADCON0bits.CHS == 0){    // Verificamos sea AN0 el canal seleccionado
            PORTC = ADRESH;         // Mostramos ADRESH en PORTC
        }
        
        // *En caso de estar usando mas de un canal analógico
        else if (ADCON0bits.CHS == 1){
            contador = ADRESH;
            ponteciometro = contador ;
        }
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupción
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            // *En caso de estar usando mas de un canal analógico

            if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0001;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0001)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            __delay_us(40);                 // Tiempo de adquisición

            ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
        }
        
        divisiones();               // Llamamos a la funcion de diviciones para dividir en unidades,decenas y centenas
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00000011; // AN0 como entrada analógica
    ANSELH = 0;         // I/O digitales)
    
    //ANSEL = 0b00000111; // AN0, AN1 y AN2 como entrada analógica
    
    TRISA = 0b00000011; // AN0 como entrada
    //TRISA = 0b00000111; // AN0, AN1 y AN2 como entrada
    PORTA = 0; 
    TRISB = 0;
    PORTB = 0;    
    TRISC = 0;
    PORTC = 0;
    TRISD = 0;
    PORTD = 0;
    TRISE = 0;
    PORTE = 0;
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b0110;   // 4MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales

    INTCONbits.T0IE = 1;        // Habilitamos interrupciones del TMR0
    INTCONbits.T0IF = 0;        // Limpiamos bandera de interrupción del TMR0
    
    OPTION_REGbits.PSA = 0;     //Un TMR0 con un Prescaler 1:256
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;     
    
    TMR0= _tmr0_value;          // Reiniciamos el TMR0 a 217 para tener un retardo de 5ms
    
    banderas = 0b00000000;      // Limpiamos la variable de banderas    
}

int divisiones(void){
    centenas = ponteciometro%10;    //Dividimos lo que esta en PORTC entre 100 para obtener las centenas
    decenas = (ponteciometro%100 - centenas)/10;     //Ocupamos el modulo de division para obtener el residuo
    unidades = (ponteciometro - centenas*10 - centenas)/100;       // Ocupamos el residuo restante del modulo para ponerlo en las unidades
}
