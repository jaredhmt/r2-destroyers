/* 
 * File:   HaleMechaLib.c
 * Author: jared.hale
 *
 * Created on September 26, 2019, 10:34 AM
 * 
 * Version History Tracked with Git
 * 
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

/*void Timer1Setup(int power, int preScale, int period, int timeReset){
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
} */

int moveBot(char dir, float dist){
    // To call moveBot, use notation:     globalStepVar = moveBot('rit',45);
    // INPUTS:
        // dir: "fwd", "bkw", "lft", "rit"
        // dist: for linear motion, distance in inches
        //       for rotational motion, distance in angular degree
    // OUTPUT: stepCount - stepCount to compare to in OCR ISR
    // Pinout notation for step dir: 1- FWD, 0- BKW
    int stepCount;
    if(dir == 'fwd'||dir == 'bkw'){
        stepCount = dist * (200 / 8.472875); //convert linear distance to step count
        if(dir == 'fwd'){
            _RB8 = 1; //set left dir to 1
            _RB9 = 1; //set right dir to 1
        }
        else{
            _RB8 = 0; //set left dir to 0
            _RB9 = 0; //set right dir to 0
        }
    }
    else if(dir == 'lft'||dir == 'rit'){
        stepCount = dist * (51.875 / 24.273); //convert angular distance to step count
        if(dir == 'lft'){
            _RB8 = 0; //set left dir to 0
            _RB9 = 1; //set right dir to 1
        }
        else{
            _RB8 = 1; //set left dir to 1
            _RB9 = 0; //set right dir to 0
        }
    }
    return stepCount;
}

void startupConfig(){
    // -----SETUP PINS-----
        // Set ALL Digital
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
    
    
    // -----SETUP PWM SIGNALS-----
        // PIN 4 - AIMING MOTOR STEPS
        // Clear control bits
        OC2CON1 = 0;
        OC2CON2 = 0;

        // Set period and duty cycle
        OC2R = 0;                // Set Output Compare value to achieve
                                    // desired duty cycle. This is the number
                                    // of timer counts when the OC should send
                                    // the PWM signal low. The duty cycle as a
                                    // fraction is OC1R/OC1RS.
        OC2RS = 3999;               // Period of OC1 to achieve desired PWM 
                                    // frequency, FPWM. See Equation 15-1
                                    // in the data sheet. For example, for
                                    // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                    // register contains the period when the
                                    // SYNCSEL bits are set to 0x1F (see FRM)

        // Configure OC1
        OC2CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
        OC2CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                    // (self synchronization) -- Although we
                                    // selected the system clock to determine
                                    // the rate at which the PWM timer increments,
                                    // we could have selected a different source
                                    // to determine when each PWM cycle initiates.
                                    // From the FRM: When the SYNCSEL<4:0> bits
                                    // (OCxCON2<4:0>) = 0b11111, they make the
                                    // timer reset when it reaches the value of
                                    // OCxRS, making the OCx module use its
                                    // own Sync signal.
        OC2CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                    // triggering with the OC1 source
                                    // "In Synchronous operation, the internal
                                    // timer resets to zero when the source
                                    // selected by the Trigger/Synchronization
                                    // Source Selection (SYNCSEL) bits sends a
                                    // Sync signal. In Trigger mode, the internal
                                    // timer is held in the Reset state until the
                                    // selected Trigger source sends a Sync
                                    // signal." - Section 3.0 "Modes of Operation"
                                    // "The synchronous operation of the timer is
                                    // enabled when the OCTRIG bit = 0.... Whenever
                                    // the selected module receives a
                                    // synchronization signal, the timer will roll
                                    // over to 0x0000 on the next positive edge of
                                    // the selected clock." - Section 3.3.7

        OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
        
        // PIN 5 - SERVO
        // Clear control bits
        OC3CON1 = 0;
        OC3CON2 = 0;

        // Set period and duty cycle
        OC3R = 0;
        OC3RS = 3999;

        // Configure OC1
        OC3CON1bits.OCTSEL = 0b111;
        OC3CON2bits.SYNCSEL = 0x1F;
        OC3CON2bits.OCTRIG = 0;
        OC3CON1bits.OCM = 0b110;
        
        // PIN 14 - DRIVER MOTORS STEPS
        // Clear control bits
        OC1CON1 = 0;
        OC1CON2 = 0;

        // Set period and duty cycle
        OC1R = 0;
        OC1RS = 3999;

        // Configure OC1
        OC1CON1bits.OCTSEL = 0b111;
        OC1CON2bits.SYNCSEL = 0x1F;
        OC1CON2bits.OCTRIG = 0;
        OC1CON1bits.OCM = 0b110;
        
    // -----SETUP NOTIFICATION INTERRUPTS-----
        
        // PIN 15 - KILL SWITCH (Could move to pin 11 or 17 to use INT0/INT1)
        _CN14IE = 1;
        _CN14PUE = 0;
        _CNIP = 7;
        
        // PIN 16 - START SWITCH
        _CN13IE = 1;
        _CN13PUE = 0;
        
        // PIN 10 - LIMIT SWITCHES
        _CN0IE = 1;
        _CN0PUE = 0;
        
        _CNIF = 0;
        _CNIE =1;
        
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
        _CSS13 = 1; //scan chanel 1
        _CSS14 = 1; //scan chanel 1
        _CSS15 = 1; //scan chanel 1
        _CSS9 = 1; //scan chanel 1
        _ADON = 1;    // AD1CON1<15> -- Turn on A/D
        
        
}