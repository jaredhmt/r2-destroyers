/* 
 * File:   aimMilestone.c
 * Author: jared.hale
 *
 * Created on November 8, 2019
 * 
 * Version History Tracked with Git
 * Milestone X
 * 
 */

#include <xc.h>
//#include "MechaLibrary.h"
#pragma config FNOSC = FRCDIV
#pragma config OSCIOFNC = OFF
// #pragma config SOSCSRC = DIG

// DEFINE GLOBAL VARIABLES HERE
#define fwd 1
#define bkw 2
#define lft 3
#define rit 4
#define vThresh 1000
#define degrees 50

enum state {PORTSIDE, AHEAD, STARBOARD};

int currentAim, desiredAim; // currentAim and desiredAim range from -50 to 50 

// SET CLOCK TO USE HERE


// SET ISR's HERE

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void){
    // This OC Interrupt controls the aiming of the turret.
    _OC2IF = 0;
    
    if (_RA1 == 1){
        currentAim = currentAim + 1;
    } else{
        currentAim = currentAim - 1 ;
    }
    
    if (currentAim == desiredAim){
        OC2R = 0;
    }
    //if (currentAim > degrees || currentAim < -degrees){
   //     OC2R = 0;
   // }
    if (currentAim >3){
        _LATA0 = 0;
    }
}

// MAIN FUNCTION
int main(){
    
    ANSA = 0;
        ANSB = 0;
        // Cherry Pick Analog Pins (7,8,9,18)
        _ANSA2 = 1;
        _ANSA3 = 1;
        _ANSB4 = 1;
        _ANSB15 = 1;

        // Cherry Pick Digital In
        _TRISA4 = 1;
        _TRISB12 = 1;
        _TRISB13 = 1;

        // Cherry Pick Digital Out
        _TRISA0 = 0;
        _TRISA1 = 0;
        _TRISB0 = 0;
        _TRISB1 = 0;
        _TRISB2 = 0;
        _TRISB7 = 0;
        _TRISB8 = 0;
        _TRISB9 = 0;
        _TRISA6 = 0;
        _TRISB14 = 0;
    
        OC2CON1 = 0;
        OC2CON2 = 0;
        // Set period and duty cycle
        OC2R = 0;               
        OC2RS = 43999;        
         // OC2 INTERUPT
        _OC2IE = 1;
        _OC2IF = 0;
        _OC2IP = 4;                          
        // Configure OC1
        OC2CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
        OC2CON2bits.SYNCSEL = 0x1F; 
        OC2CON2bits.OCTRIG = 0;     
        OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode       
    //startupConfig();
    
        // -----SETUP ANALOG CHANNELS-----
        _ADON = 0;    // AD1CON1<15> -- Turn off A/D during config

        // AD1CON1 register
        _ADSIDL = 0;  // AD1CON1<13> -- A/D continues in idle mode
        _MODE12 = 1;  // AD1CON1<10> -- 12-bit A/D operation
        _FORM = 0;    // AD1CON1<9:8> -- Unsigned integer output
        _SSRC = 7;    // AD1CON1<7:4> -- Auto conversion (internal counter)
        _ASAM = 1;    // AD1CON1<2> -- Auto sampling

        // AD1CON2 register
        _PVCFG = 0;   // AD1CON2<15:14> -- Use VDD as positive
                      // ref voltage
        _NVCFG = 0;   // AD1CON2<13> -- Use VSS as negative
                      // ref voltage
        _BUFREGEN = 1;// AD1CON2<11> -- Result appears in buffer
                      // location corresponding to channel
        _CSCNA = 1;   // AD1CON2<10> -- Scans inputs specified
                      // in AD1CSSx registers
        _SMPI = 3;	  // AD1CON2<6:2> -- Every 4th conversion sent
                      // to buffer (if sampling 4 channels)
        _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

        // AD1CON3 register
        _ADRC = 0;    // AD1CON3<15> -- Use system clock
        _SAMC = 0;    // AD1CON3<12:8> -- Auto sample every A/D
                      // period TAD
        _ADCS = 0x3F; // AD1CON3<7:0> -- A/D period TAD = 64*TCY

        // AD1CSS registers
        // SET THE BITS CORRESPONDING TO CHANNELS THAT YOU WANT
        // TO SAMPLE
        _CSS13 = 1; //scan chanel 13
        _CSS14 = 1; //scan chanel 14
        _CSS15 = 1; //scan chanel 15
        _CSS9 = 1; //scan chanel 9
        _ADON = 1;    // AD1CON1<15> -- Turn on A/D
        
    currentAim = 0;
    desiredAim = 0;
    _LATA0 = 1;
   
    
    while(1){
        
        while (ADC1BUF14 >= vThresh){
            desiredAim = 0;
            if (currentAim < 0){
                _LATA1 = 1;
                OC2R = 1000;
            }
            else if(currentAim > 0){
                _LATA1 = 0;
                OC2R = 1000;
            }
        }
        while (ADC1BUF13 >= vThresh){
            desiredAim = -degrees;
            if (currentAim > -degrees){
                _LATA1 = 0;
                OC2R = 1000;
            }
        }
        while (ADC1BUF15 >= vThresh){
            desiredAim = degrees;
            if (currentAim < degrees){
                _LATA1 = 1;
                OC2R = 1000;
            }
        }
    }
}