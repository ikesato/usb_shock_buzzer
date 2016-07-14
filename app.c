#include "system.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include "usb.h"
#include "usb_config.h"
#include "usb_device_cdc.h"

#include "app.h"
#include "adxl213.h"
#include "shock_detector.h"
#include "util.h"
#include "gtime.h"
#include "sound.h"


/** DEFINES ********************************************************/
#define _XTAL_FREQ (48000000)
#define PORT_BUTTON     PORTAbits.RA3
#define PORT_LED_WARN   PORTCbits.RC7
#define PORT_LED_ALERT  PORTCbits.RC6
#define PORT_BEEP       PORTCbits.RC4
#define PORT_USB_VDD    PORTCbits.RC1
#define PORT_LED_ACTIVE PORTCbits.RC2


#define T0CNT (65536-375)
#define T1CNT (65536-256)


/** VARIABLES ******************************************************/

static bool buttonPressed;
static char buttonMessage[] = "Button pressed.\r\n";
static uint8_t readBuffer[CDC_DATA_OUT_EP_SIZE];
static uint8_t writeBuffer[CDC_DATA_IN_EP_SIZE];
static ADXL213 accel;
static ShockDetector detector;
static unsigned short gtime_counter=0;
unsigned short gtime=0; // 時間単位は1/16秒
static unsigned short last_pressed=0;
static char growing = 0; // bit 0:WARN, 1:ALERT
static unsigned short growing_counter = 0;
static unsigned short last_stabled=0;
static unsigned char playing=0;

void go2sleep(void);


void setup(void)
{
    // Input Pin: RA3,RB4,RB5,RB6,RB7,RC0,RC1,RC3
    // PWM: RC4
    // Pullup: RB4,RB5,RB6,RB7
    TRISA = 0b00001000;
    TRISB = 0b11110000;
    TRISC = 0b00001011;
    INTCON2bits.RABPU = 0; // enable pull-up
    WPUB  = 0b11110000;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;

    // ADC
    REFCON0 = 0b10100000;  // bit4,5 FVR 01:1.014V 10:2.048V 11:4.096V
    PIE1bits.ADIE = 0; // Disable ADC interrupt
    ADCON0bits.CHS = 7; // AN7
    ADCON1 = 0b1000;    // positive reference, PVCFG<2:3> -> 00:VDD 01:Vref+ 10:FVR
    // negative reference, NVcfg<0:1> -> 00:VSS 01:Vref-
    ADCON2bits.ADFM = 1; // Right justified
    ADCON2bits.ACQT = 0b101; // 12TAD
    ADCON2bits.ADCS = 0b110; // FRC/64 => 107msec, FRC is 600kHz
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_DONE = 1;
    ANSELH = 0;
    ANSEL = 0b10000000;

    // timer0 for playing audio
    // USB Bootloader では 48MHz で動作しているので
    // 8kHz を作るには
    //   48MHz/4 * x = 8kHz としたい
    //   x = (48/4)*1000/8 = 1500
    //   prescaler を 1:4 とすると 1500/4 = 375 (T0CNT)
    //
    T0CONbits.T08BIT = 0;     // 16bit timer
    T0CONbits.T0PS = 0b001;   // prescaler 1:4
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;        // use prescaler
    T0CONbits.TMR0ON = 1;
    TMR0 = T0CNT;
    INTCON2bits.TMR0IP = 1;
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    TMR1 = T1CNT;
    T1CONbits.TMR1CS = 0;    // 内部クロック (FOSC/4)
    T1CONbits.T1CKPS = 0b01; // prescaler 1:2
    T1CONbits.RD16 = 1;      // 16bit
    T1CONbits.TMR1ON = 1;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;


    // PWM settings
//CCP1CONbits.CCP1M = 0b0000; // PWM off
    CCP1CONbits.CCP1M = 0b1100; // P1A、P1C をアクティブ High、P1B、P1D をアクティブ High
    CCP1CONbits.DC1B  = 0b11;   // デューティ サイクル値の最下位 2 ビット
    CCP1CONbits.P1M   = 0b00;   // シングル出力
    PSTRCONbits.STRA = 0;
    PSTRCONbits.STRB = 1;
    PSTRCONbits.STRC = 0;
    PSTRCONbits.STRD = 0;
    PSTRCONbits.STRSYNC = 1;


    // timer2 for PWM
    // 8ビットのデーティー幅とする場合は PR2 が 0x3F となる
    // 16MHz の場合
    //   16MHz/4 = 4MHz
    //   4MHz / (0x3F+1) = 4000kHz/64 = 62.5kHz
    // 48MHz の場合
    //   48MHz/4 = 12MHz
    //   12MHz / (0x3F+1) = 12000kHz/64 = 187.5kHz
    //CCPR1L  = 0x3F;              // デューティ値
    CCPR1L  = 0x00;
    PR2     = 0x3F;            // PWM周期 187.5kHz @48MHz

    TMR2 = 0;
    T2CONbits.T2CKPS = 0b00;  // prescaler 1:1
    T2CONbits.T2OUTPS = 0;    // postscaler 1:1
    T2CONbits.TMR2ON = 1;     // Timer ON

    PORT_LED_ACTIVE = 1;
    // queue
    //queue_init(&queue, queue_buffer, sizeof(queue_buffer));

    // for i2c
    //mc24c64_init();

    // timer3 for ADXL213
    // 48HMHz で prescaler 1:1 なので
    //   48MHz/4 = 12MHz
    //   1 は 1/12 [usec] == 0.0833[usec]
    TMR3 = 0;
    T3CONbits.RD16 = 1;      // 16bit mode
    T3CONbits.T3CKPS = 0b00; // prescaler 1:1
    T3CONbits.T3CCP1 = 1;    // use Timer3 clock source
    T3CONbits.TMR3CS = 0;    // internal clock
    //T3CONbits.T3SYNC = 1;
    T3CONbits.TMR3ON = 1;    // Timer ON
    PIR2bits.TMR3IF = 0;     // clear overflow

    // for ADXL213 interrupt pins
    INTCONbits.RABIF = 1;
    INTCONbits.RABIE = 1;
    INTCON2bits.RABIP = 1;   // high level interrupt
    IOCBbits.IOCB5 = 1;
    IOCBbits.IOCB7 = 1;

    adxl213_init(&accel);
    shock_detector_init(&detector);

    // for wakeup
    INTCONbits.INT0IE = 1;
    INTCON2bits.INTEDG0 = 0;
    INTCON3bits.INT1IE = 1;
    INTCON2bits.INTEDG1 = 1;

    // app init
    buttonPressed = false;
}

void interrupted(void)
{
    //i2c_interrupt();

    if (INTCONbits.TMR0IF == 1) {
        TMR0 = T0CNT;
        INTCONbits.TMR0IF = 0;
        if (++gtime_counter >= 500) { // 500==8000/16, 8000 == 1[sec]
            gtime_counter = 0;
            gtime++;
        }
        if (playing) {
            unsigned char v = sound_update();
            CCPR1L = (v >> 2) & 0x3F;
            CCP1CONbits.DC1B = (v& 0x3);
        } else {
            CCPR1L = 0;
            CCP1CONbits.DC1B = 0;
        }
    }
    if (PIR1bits.TMR1IF == 1) {
        PIR1bits.TMR1IF = 0;
        TMR1 = T1CNT;
        if (growing!=0) {
            growing_counter++;
            unsigned char strength = calc_growing((unsigned char)(growing_counter>>8));
            //unsigned char strength = calc_growing((unsigned char)(growing_counter));
            unsigned char led = (((unsigned char)(growing_counter & 0xff)) < strength);
            PORT_LED_WARN  = (growing & 0x01) ? led : 0;
            PORT_LED_ALERT = (growing & 0x02) ? led : 0;
        }
    }
    if (INTCONbits.RABIF == 1) {
        INTCONbits.RABIF = 0;
        unsigned short now = TMR3;
        adxl213_update(&accel, PORTBbits.RB5, PORTBbits.RB7, now);
        adxl213_low_pass_filter(&accel);
    }
    if (INTCONbits.INT0IF) {
        // wake up from sleep mode
        INTCONbits.INT0IF = 0;
    }
    if (INTCON3bits.INT1IF) {
        // wake up from sleep mode
        INTCON3bits.INT1IF = 0;
    }
}

void loop(void)
{
    unsigned short difft;
    unsigned char play = 0;
    shock_detector_update(&detector, &accel, gtime);
    switch (detector.mode) {
    case SD_MODE_NOT_STARTED:
        PORT_LED_WARN = PORT_LED_ALERT = 0;
        growing = 0;
        if (PORT_BUTTON == 0)
            detector.mode = SD_MODE_STARTED;
        break;
    case SD_MODE_STARTED:
        growing = 0x3;
        break;
    case SD_MODE_UNLOCKING:
        difft = diff_time(gtime, last_pressed);
        PORT_LED_WARN = PORT_LED_ALERT = ((difft & 0xF) == 0xF);
        growing = 0;
        if (PORT_BUTTON == 1) {
            if (abs(TIME_SEC(3) - difft) < TIME_HALF_SEC) { //2.5秒から3.5以内ならロック解除
                detector.mode = SD_MODE_NOT_STARTED;
            } else {
                // TODO: warning buzzer
                detector.mode = SD_MODE_STARTED;
            }
        }
        break;
    case SD_MODE_DETECTING:
        switch (detector.shocked) {
        case SD_SHOCK_LITTLE:
            PORT_LED_ALERT = PORT_LED_WARN = ((gtime>>1) & 0x01);
            growing = 0;
            play = 1;
            break;
        case SD_SHOCK_LARGE:
            PORT_LED_WARN = 0;
            PORT_LED_ALERT = ((gtime>>1) & 0x01);
            growing = 0;
            play = 2;
            break;
        default:
            growing = 0x1;
            last_stabled = gtime;
            break;
        }
        if (PORT_BUTTON == 0) {
            detector.mode = SD_MODE_UNLOCKING;
            last_pressed = gtime;
        }
        break;
    }

    if (play != 0) {
        difft = diff_time(gtime, last_stabled);
        if (difft > TIME_HALF_SEC) {
            if (playing != play) {
                sound_play(play);
                // initialize NJU72501
                // いきなりPWMだとshutdownモードから復帰できない模様
                // なので 1msec 使ってしっかりと復帰させる
                CCP1CONbits.CCP1M = 0b0000; // PWM off
                PORT_BEEP = 0;
                __delay_ms(10);
                PORT_BEEP = 1;
                __delay_ms(10);
                CCP1CONbits.CCP1M = 0b1100; // PWM on
            }
            playing = play;
        } else {
            playing = 0;
        }
    } else {
        playing = 0;
    }

    if (detector.mode == SD_MODE_NOT_STARTED && PORT_USB_VDD == 0) {
        go2sleep();
    }


    /* If the user has pressed the button associated with this demo, then we
     * are going to send a "Button Pressed" message to the terminal.
     */
    if(PORT_BUTTON == 0)
    {
        /* Make sure that we only send the message once per button press and
         * not continuously as the button is held.
         */
        if(buttonPressed == false)
        {
            buttonPressed = true;
            /* Make sure that the CDC driver is ready for a transmission.
             */
            if(mUSBUSARTIsTxTrfReady() == true)
            {
                //putrsUSBUSART(buttonMessage);
            }
            //detector.mode = (detector.mode == SD_MODE_NOT_STARTED ? SD_MODE_STARTED : SD_MODE_NOT_STARTED);
        }
    }
    else
    {
        /* If the button is released, we can then allow a new message to be
         * sent the next time the button is pressed.
         */
        buttonPressed = false;
    }

    /* Check to see if there is a transmission in progress, if there isn't, then
     * we can see about performing an echo response to data received.
     */
    if( USBUSARTIsTxTrfReady() == true)
    {
        uint8_t i;
        uint8_t numBytesRead;

        numBytesRead = getsUSBUSART(readBuffer, sizeof(readBuffer));

        /* For every byte that was read... */
        for(i=0; i<numBytesRead; i++)
        {
            switch(readBuffer[i])
            {
                /* If we receive new line or line feed commands, just echo
                 * them direct.
                 */
                case 0x0A:
                case 0x0D:
                    //writeBuffer[i] = readBuffer[i];
                    break;

                /* If we receive something else, then echo it plus one
                 * so that if we receive 'a', we echo 'b' so that the
                 * user knows that it isn't the echo enabled on their
                 * terminal program.
                 */
                default:
                    //writeBuffer[i] = readBuffer[i] + 1;
                    break;
            }
        }

        if(numBytesRead > 0)
        {
            /* After processing all of the received data, we need to send out
             * the "echo" data now.
             */
            //putUSBUSART(writeBuffer,numBytesRead);
        }

        //{
        //    writeBuffer[0] = 4;
        //    writeBuffer[1] = 8;
        //    *((unsigned short *)(&writeBuffer[2])) = accel.axis[0].on;
        //    *((unsigned short *)(&writeBuffer[4])) = accel.axis[0].off;
        //    *((unsigned short *)(&writeBuffer[6])) = accel.axis[1].on;
        //    *((unsigned short *)(&writeBuffer[8])) = accel.axis[1].off;
        //    putUSBUSART(writeBuffer, writeBuffer[1]+2);
        //}
        {
            writeBuffer[0] = 20;
            writeBuffer[1] = 26;
            writeBuffer[2] = detector.mode;
            writeBuffer[3] = detector.shocked;
            *((unsigned short *)(&writeBuffer[4])) = detector.axis[0].stable_value;
            *((unsigned short *)(&writeBuffer[6])) = detector.axis[1].stable_value;

            *((unsigned short *)(&writeBuffer[8])) = accel.axis[0].value;
            *((unsigned short *)(&writeBuffer[10])) = accel.axis[0].on;
            *((unsigned short *)(&writeBuffer[12])) = accel.axis[0].off;
            *((unsigned short *)(&writeBuffer[14])) = accel.axis[0].last_time;

            *((unsigned short *)(&writeBuffer[16])) = accel.axis[1].value;
            *((unsigned short *)(&writeBuffer[18])) = accel.axis[1].on;
            *((unsigned short *)(&writeBuffer[20])) = accel.axis[1].off;
            *((unsigned short *)(&writeBuffer[22])) = accel.axis[1].last_time;

            writeBuffer[24] = accel.axis[0].last_pin;
            writeBuffer[25] = accel.axis[1].last_pin;
            putUSBUSART(writeBuffer, writeBuffer[1]+2);
        }
    }
}

void go2sleep(void)
{
    INTCONbits.TMR0IE = 0;
    INTCONbits.GIEL = 0;
    INTCONbits.RABIE = 0;
    SSPIE = 0;
    BCLIE = 0;
    PEIE  = 0;

    PORTA = PORTB = PORTC = 0;

    SLEEP();
    NOP();

    // wake up code
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.RABIE = 1;
    SSPIE = 1;
    BCLIE = 1;
    PEIE  = 1;
    PORT_LED_ACTIVE = 1;
}
