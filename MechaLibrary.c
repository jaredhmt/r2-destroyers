/* 
 * File:   HaleMechaLib.c
 * Author: jared.hale
 *
 * Created on September 26, 2019, 10:34 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

/*
 * 
 */

void PinIOConfig(int pin, int mode){
    // This could potentially be overkill on comp. speed. Maybe find a way to
    // use pointers instead of if/else statements.
    if(pin == 1){
        _TRISA6 = mode; // From my understanding this should be RA5, but i am
                        // getting an error and it only accepts RA6... not sure
                        // why.
    }
    else if(pin == 2){
        _TRISA0 = mode;
    }
    else if(pin == 3){
        _TRISA1 = mode;
    }
    else if(pin == 4){
        _TRISB0 = mode;
    }
    else if(pin == 5){
        _TRISB1 = mode;
    }
    else if(pin == 6){
        _TRISB2 = mode;
    }
    else if(pin == 7){
        _TRISA2 = mode;
    }
    else if(pin == 8){
        _TRISA3 = mode;
    }
    else if(pin == 9){
        _TRISB4 = mode;
    }
    else if(pin == 10){
        _TRISA4 = mode;
    }
    else if(pin == 11){
        _TRISB7 = mode;
    }
    else if(pin == 12){
        _TRISB8 = mode;
    }
    else if(pin == 13){
        _TRISB9 = mode;
    }
    else if(pin == 14){
        _TRISA6 = mode;
    }
    else if(pin == 15){
        _TRISB12 = mode;
    }
    else if(pin == 16){
        _TRISB13 = mode;
    }
    else if(pin == 17){
        _TRISB14 = mode;
    }
    else if(pin == 18){
        _TRISB15 = mode;
    }
}

void Timer1Setup(int power, int preScale, int period, int timeReset){
    T1CONbits.TON = power; // Turn off/on Timer 1
    T1CONbits.TCS = 0; // Internal Clock don't change this... unless necessary
    if(preScale == 0 || preScale == 1){
        preScale = 0b00;
    }
    else if(preScale == 8){
        preScale = 0b01;
    }
    else if(preScale == 64){
        preScale = 0b10;
    }
    else if(preScale == 256){
        preScale = 0b11;
    }
    T1CONbits.TCKPS = preScale; // Use a pre-scaler
    PR1 = period; // Timer Period
    if(timeReset == 1){
        TMR1 = 0;           // Reset timer to 0
    }
}

