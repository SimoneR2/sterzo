#define USE_AND_MASKS
//==============================================================================
// PROGRAMM : STERZO
// WRITTEN BY : MOSER & RIGHETTI & CLEMENTI
// DATA : 08/12/2015
// VERSION : 1.0
// FILE SAVED AS : STERZO.C
// FOR PIC : 18F4480
// CLOCK FREQUENCY : 16 MHZ
// PROGRAMM FUNCTION: PROGRAMMA CHE GESTISCE IL SERVO DELLO STERZO
//==============================================================================

//==============================================================================
//Dichiarazione librerie
//==============================================================================

#include <xc.h>
#include "sterzo.h"
#include "CANlib.h"
#include "delay.h"
#include "delay.c"
#include "timers.h"
#include "adc.h"
#include "idCan.h"
//==============================================================================
//Dichiarazione frequenza clock e codici centraline
//==============================================================================
#define _XTAL_FREQ 16000000

void configurazione_iniziale(void);
void send_data(void);
void calibrazione(void);
CANmessage msg;

//==============================================================================
//Dichiarazione variabili
//==============================================================================
unsigned int limiteDx = 120; //da variare per limitare l'angolo di sterzata
bit remote_frame = 0;
bit noChange = 0; //tiene fermo il servo finchè non arriva la prima impostazione
signed int currentSteering = 90;
//char pastSteering = 1;
char theorySteering = 0;
//unsigned long counter = 0;
unsigned long id = 0;
//unsigned long timeCounter = 0; //1 = 10mS
//unsigned long previousTimeCounter = 0;
unsigned long duty_cycle = 0; //da controllare
signed int calibration = 0;
unsigned long timer = 0;
unsigned int Ton = 0;
signed long ADCResult = 0;
unsigned int Toff = 0;
//int errore = 0;
//int correzione = 0;
//int potenza = 2;
//BYTE counter_array [8] = 0; //variabili per il CAN BUS
//BYTE currentSteering_array [8] = 0;
BYTE data_array [8] = 0;

//==============================================================================
//ISR Alta priorità (creazione PWM a 50 Hz)
//==============================================================================

__interrupt(high_priority) void ISR_alta(void) {
    if (noChange == 1) {
        PORTCbits.RC0 = ~PORTCbits.RC0;
        T0CONbits.TMR0ON = 0;
        if (PORTCbits.RC0 == 1) { //Ton
            WriteTimer0(Ton);
            T0CONbits.TMR0ON = 1;
        }
        if (PORTCbits.RC0 == 0) { //Toff
            WriteTimer0(Toff);
            T0CONbits.TMR0ON = 1;
        }
    }
    INTCONbits.TMR0IF = 0; //reset flag

}
//==============================================================================
//ISR Bassa priorità (gestione can bus)
//==============================================================================

__interrupt(low_priority) void ISR_bassa(void) {
    if (CANisRxReady()) {
        CANreceiveMessage(&msg);
        if (msg.identifier == STEERING_CHANGE) { //cambio angolatura sterzo
            id = msg.identifier;
            remote_frame = msg.RTR;
            theorySteering = msg.data[0];
            if (theorySteering < calibration) {
                currentSteering = 0;
            }
            currentSteering = theorySteering + calibration; //aggiunta calibrazione
            if (currentSteering > 180) {
                currentSteering = 180;
            }
            if (currentSteering < 1) {
                currentSteering = 0;
            }
            currentSteering = (limiteDx * currentSteering) / 180;
            
            
            //            currentSteering = (limiteDx * theorySteering) / 180;
            noChange = 1;
        }
        if (msg.identifier == ECU_STATE_EPS) {
            PORTCbits.RC1 = ~PORTCbits.RC1; //debug
            data_array [0] = 0x02;
            CANsendMessage(ECU_STATE_EPS, data_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0); //DEBUG
        }
        PIR3bits.RXB0IF = 0; //reset flag
        PIR3bits.RXB1IF = 0; //reset flag
        //  }

    }
}

//==============================================================================
//Main
//==============================================================================

int main(void) {
    configurazione_iniziale();
    //    PORTAbits.RA1 = 1;
    //    PORTCbits.RC1 = 1;
    //    delay_ms(500);
    //    PORTAbits.RA1 = 0;
    //    PORTCbits.RC1 = 0;
    //    delay_ms(100);
    while (1) {
 delay_ms(10);
        calibrazione();
        duty_cycle = currentSteering;
        if (PORTCbits.RC0 == 0) {
            timer = (((duty_cycle * 700) / 90) + 800) *2;
            Ton = 65536 - timer;
            Toff = 20000 - (timer / 2);
            Toff = (65536 - (Toff * 2));
        }
    }
    if (remote_frame == 1) {
        send_data();
    }
    if ((CANisTXwarningON() == 1) || (CANisRXwarningON() == 1)) {
        PORTAbits.RA1 = 1;
    }
}

//==============================================================================
//Settaggi per invio dati sul CAN BUS
//==============================================================================

void send_data(void) {
    if (CANisTxReady()) {
        if (remote_frame == 1) {
            //avere lo stesso id della richiesta
            CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG &
                    CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        remote_frame = 0;
    }
}

//==============================================================================
//Lettura tramite ADC del potenziometro (per calibrare lo sterzo)
//==============================================================================

void calibrazione(void) { //lettura ADC calibrazione
    ConvertADC();
    while (BusyADC());
    ADCResult = ReadADC();
    calibration = (ADCResult - 511) / 30;
}

//==============================================================================
//Configurazione bit
//==============================================================================

void configurazione_iniziale(void) {

    //==========================================================================
    //configurazione CAN BUS
    //==========================================================================
    RCONbits.IPEN = 1; //abilita priorità interrupt
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);

    //==========================================================================
    //azzeramento flag
    //==========================================================================
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    INTCONbits.TMR0IF = 0; //azzera flag timer0
    //PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3

    //==========================================================================
    //configurazione priorità
    //==========================================================================
    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    INTCON2bits.TMR0IP = 1; //interrupt alta priorità timer0
    // IPR2bits.TMR3IP = 0; //interrupt bassa priorità timer 3

    //==========================================================================
    //Enable interrupts
    //==========================================================================
    RCONbits.IPEN = 1; //abilita priorità interrupt
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    INTCONbits.TMR0IE = 1; //abilita interrupt timer0
    // PIE2bits.TMR3IE = 0; //abilita interrupt timer 3
    INTCONbits.GIEH = 1; //abilita interrupt alta priorità
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorità periferiche

    //==========================================================================
    //impostazione timer0 per PWM
    //==========================================================================

    OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_16_TAD, ADC_CH0 & ADC_REF_VDD_VSS & ADC_INT_OFF, ADC_1ANA); //re2

    //==========================================================================
    //impostazione periodo timer prima volta
    //==========================================================================

    T0CONbits.TMR0ON = 1;
    PORTCbits.RC0 = 0;
    //==========================================================================
    //impostazione uscite/ingressi
    //==========================================================================
    LATA = 0x00;
    TRISA = 0b11111101;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0x00;

    LATD = 0x00;
    TRISD = 0x00;

    LATE = 0x00;
    TRISE = 0xFF;
    T0CON = 0x80; //imposta timer0, prescaler 1:2
}