#define USE_AND_MASKS
/* 
 * File:   newmain.c
 * Author: Simone
 *
 * Created on 3 dicembre 2015, 8.09
 */
#include <xc.h>
#include "newfile.h"
#include <CANlib.h>
#include "delay.h"
#include "delay.c"
#include <pwm.h>
#include "timers.h"
#include "adc.h"
//#include <pic18f4480.h>
#define _XTAL_FREQ 16000000
#define Steering_info 0b10000000000000000000000000000
#define Steering_change 0b00000000000000000000000000011
#define emergency 0b00000000000000000000000000001
void configurazione_iniziale(void);
void send_data(void);
void calibrazione(void);
CANmessage msg;
bit remote_frame = 0;

//da variare per limitare l'angolo di sterzata
unsigned int limiteDx = 0;
unsigned int limiteSx = 180;

char currentSteering = 180;
char pastSteering = 1;
unsigned char theorySteering = 0;
unsigned long counter = 0;
unsigned long id = 0;
unsigned long timeCounter = 0; //1 = 10mS
unsigned long previousTimeCounter = 0;
unsigned long duty_cycle = 0; //da controllare
int calibration = 0;
unsigned long timer = 0;
unsigned int timer1 = 0;
unsigned int ADCResult = 0;
unsigned char valore = 0;
unsigned valore1 = 0;
unsigned int periodo = 0;
BYTE counter_array [8] = 0;
BYTE currentSteering_array [8] = 0;
BYTE data_array [8] = 0;
//*************************************
//ISR Alta priorit�
//*************************************

__interrupt(high_priority) void ISR_alta(void) {
    PORTCbits.RC0 = ~PORTCbits.RC0;
    T0CONbits.TMR0ON = 0;
    if (PORTCbits.RC0 == 1) {
        WriteTimer0(timer1);
        T0CONbits.TMR0ON = 1;
    }
    if (PORTCbits.RC0 == 0) {
        WriteTimer0(periodo);
        T0CONbits.TMR0ON = 1;
    }

    INTCONbits.TMR0IF = 0;
}
//*************************************
//ISR Bassa priorit� (gestione can bus)
//*************************************

__interrupt(low_priority) void ISR_bassa(void) {
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            if (msg.RTR == 1) {
                id = msg.identifier;
                remote_frame = msg.RTR;
            }
            if (msg.identifier == Steering_info) {
                data_array[0] = theorySteering;
                if (theorySteering <= limiteDx) {
                    theorySteering = limiteDx;
                }
                if (theorySteering >= limiteSx) {
                    theorySteering = limiteSx;
                }
            }
            if (msg.identifier == Steering_change) {
                //pastSteering = currentSteering;
                theorySteering = msg.data[0];
                currentSteering = theorySteering + calibration; //aggiunta calibrazione
                previousTimeCounter = timeCounter;
                     }
        }
        PIR3bits.RXB0IF = 0;
        PIR3bits.RXB1IF = 0;
    }
    if (PIR2bits.TMR3IF) {
        timeCounter++;
        TMR3H = 0x63;
        TMR3L = 0xC0;
        PIR2bits.TMR3IF = 0;
    }
}

int main(void) {
    configurazione_iniziale();
    PORTC = 0xFF;
    delay_ms(100);
    PORTC = 0x00;
    delay_ms(100);

    TMR0H = 0xdd;
    TMR0L = 0xa0;
    T0CONbits.TMR0ON = 1;
    periodo = 0x4588;
    while (1) {
        
        calibrazione();
        if (timeCounter - previousTimeCounter > 2) {
            if (currentSteering - pastSteering >0){
            if (currentSteering - pastSteering > 50) {
                duty_cycle = duty_cycle + 20;
            } else {
                if (currentSteering - pastSteering > 20) {
                    duty_cycle = duty_cycle + 5;
                } else {
                    if (currentSteering - pastSteering < 5) {
                        duty_cycle = currentSteering;
                    }
                }
            }
            }
            else{
            if (pastSteering - currentSteering > 50) {
                duty_cycle = duty_cycle - 20;
            } else {
                if (pastSteering - currentSteering > 20) {
                    duty_cycle = duty_cycle - 5;
                } else {
                    if (pastSteering - currentSteering < 5) {
                        duty_cycle = currentSteering;
                    }
                }
            }
            }
            pastSteering = duty_cycle;
            previousTimeCounter = timeCounter;
        }
        if (PORTCbits.RC0 == 0) {
            timer = (((duty_cycle * 700) / 90) + 800) / 0.5;
            timer1 = 65536 - timer;
            periodo = 20000 - (timer * 0.5);
            periodo = (65536 - (periodo / 0.5));
        }
        if (remote_frame == 1) {
            send_data();
        }
        if ((CANisTXwarningON() == 1) || (CANisRXwarningON() == 1)) {
            PORTAbits.RA5 = 1;
        }
        //if ((timeCounter - previousTimeCounter) > 100) {
        //     PORTAbits.RA5 = 1;
        //  }
    }
}

void send_data(void) {
    if (CANisTxReady()) {
        if (remote_frame == 1) {
            //avere lo stesso id della richiesta
            CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG &
                    CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);

            //  delay_ms(5);
            //  CANsendMessage(id, data_array, 8, CAN_CONFIG_STD_MSG &
            //         CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        remote_frame = 0;
    }
}

void calibrazione(void) { //lettura ADC calibrazione
    ConvertADC();
    while (BusyADC());
    ADCResult = ReadADC();
    calibration = (ADCResult - 511) / 45;
}

void configurazione_iniziale(void) {
    LATA = 0x00;
    TRISA = 0b11111100;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0x00;

    LATD = 0x00;
    TRISD = 0x00;

    LATE = 0x00;
    TRISE = 0xFF;

    PORTC = 0xff;
    delay_ms(100);
    PORTC = 0x00;
    delay_ms(100);
    PORTC = 0xff;
    delay_ms(100);
    PORTC = 0x00;
    delay_ms(100);
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);
    RCONbits.IPEN = 1; //abilita priorit� interrupt
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    IPR3bits.RXB1IP = 0; //interrupt bassa priorit� per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorit� per can
    INTCONbits.GIEH = 1; //abilita interrupt alta priorit�
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorit� periferiche
    INTCON2bits.INTEDG0 = 1; //interrupt su fronte di salita
    INTCON2bits.TMR0IP = 1; //interrupt alta priorit� timer0
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    PORTC = 0xff;
    delay_ms(100);
    PORTC = 0x00;
    T0CON = 0x80; //imposta timer0, prescaler 1:2
    //impostazione timer3 per contatore
    T3CON = 0x01; //abilita timer
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3
    IPR2bits.TMR3IP = 0; //interrupt bassa priorit� timer 3
    TMR3H = 0x63;
    TMR3L = 0xC0;
    PIE2bits.TMR3IE = 1; //abilita interrupt timer 3

    //impostazione porte

    OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_16_TAD, ADC_CH1 & ADC_REF_VDD_VSS & ADC_INT_OFF, ADC_2ANA); //re2
}
