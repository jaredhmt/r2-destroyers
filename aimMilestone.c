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
#include "MechaLibrary.h"

#pragma config OSCIOFNC = OFF
// #pragma config SOSCSRC = DIG

// DEFINE GLOBAL VARIABLES HERE
#define fwd 1
#define bkw 2
#define lft 3
#define rit 4
#define vThresh 1000

enum state {PORTSIDE, AHEAD, STARBOARD};

int currentAim, desiredAim; // currentAim and desiredAim range from -50 to 50 

// SET CLOCK TO USE HERE


// SET ISR's HERE

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void){
    // This OC Interrupt controls the aiming of the turret.
    _OC2IF = 0;
    
    if (_LATA1 == 1){
        currentAim = currentAim + 1;
    } else{
        currentAim = currentAim - 1 ;
    }
    
    if ((currentAim - desiredAim) > -5 && (currentAim - desiredAim) < 5){
        OC2R = 0;
    }
}

// MAIN FUNCTION
int main(){
    startupConfig();
    OC2RS = 14999;
    currentAim = 0;
    desiredAim = 0;
    _LATA0 = 1;
    
    while(1){
        if (currentAim != 0){
            _LATA0 = 0;
        }
        if (ADC1BUF14 >= vThresh){
            desiredAim = 0;
            if (currentAim < -5){
                _LATA1 = 1;
                OC2R = 1000;
            }
            else if(currentAim > 5){
                _LATA1 = 0;
                OC2R = 1000;
            }
        }
        else if (ADC1BUF13 >= vThresh){
            desiredAim = -50;
            if (currentAim > -45){
                _LATA1 = 0;
                OC2R = 1000;
            }
        }
        else if (ADC1BUF15 >= vThresh){
            desiredAim = 50;
            if (currentAim < 45){
                _LATA1 = 1;
                OC2R = 1000;
            }
        }
    }
}