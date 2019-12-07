/*
 * File:   Demo_V2.c
 * Author: acam2
 *
 * Created on December 13, 2019, 9:38 PM
 */
/* 
 * ============================ MILESTONE 10 INSTRUCTIONS ============================
 * 
Demonstrate the ability of your robot to find the dispenser, collect three 
 * balls, navigate to an appropriate position for scoring, and attempt to score. 
 * Submit a video of the robot in action via Learning Suite. For full points, 
 * this should be accomplished under battery power.
 */

/*This code uses a state machine
 * state = 1 retrieve balls state
 * state = 2 return to center state
 */

#include "xc.h"
//set up the delay stuff
#define FCY 4000000UL
#include <libpic30.h>

#pragma config SOSCSRC = DIG //required to use pin 9 and 10 as digital I/O
#pragma config OSCIOFNC = OFF//required to use pin 8 as digital I/O

#pragma config FNOSC = FRCDIV//set the oscillator

#define vThresh 1000//threshold for IR detection

int  distanceForward = 20; // inches

int OC2count = 0; // Set Global Variable OC1count
int desiredSteps = 0; // Set Global Variable desiredSteps
int state = 0; // Set Global Variable state
int desiredSteps_movement = 0;
int OC1count = 0;

void _ISR _OC2Interrupt(void){

    _OC2IF = 0;  // Remember to clear the OC2 interrupt flag when
    // this ISR is entered.

    // PLACE CUSTOM CODE HERE
    OC2count++;
    if(OC2count >= desiredSteps){
        OC2R = 0;
        //_LATA1 = 0;
        OC2count = 0;
    }
}
void _ISR _OC1Interrupt(void){

    _OC1IF = 0;  // Remember to clear the OC1 interrupt flag when
    // this ISR is entered.

    // PLACE CUSTOM CODE HERE
    OC1count++;
    if(OC1count >= desiredSteps_movement){
        OC1R = 0;
        _LATB9 = 0; // pin 13 is high - the sleeps the pin on both move steppers
    }
}
void _ISR _CNInterrupt(void){
_CNIF = 0; // Clear interrupt flag (IFS1 register)
// Figure out which pin changed if you?re looking at multiple pins
// This will handle the limit switch interrupt when the robot moves into the corner.
//It will flash the LED on and off a certain number of times
    int flashcount = 0;
    
    if(_RA4 == 1){
        OC1R = 0;
        _LATB9 = 0;
        while(flashcount < 5){
         _LATB14 = 1;//turn on pin 17
         __delay_ms(500);
         _LATB14 = 0;
         __delay_ms(500);
         flashcount++;
         state = 2;
        }
    }
    else{
        _LATB14 = 0;
    }
} 
void PinIOConfig(void){
    
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
    _SMPI = 2;	  // AD1CON2<6:2> -- Every 4th conversion sent
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
    //_CSS9 = 1; //scan chanel 9
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
    
   // Configure the digital I/O ports
//    // PLACE CODE TO CONFIGURE THE DIGITAL I/O PORTS HERE
 ANSA = 0;
 ANSB = 0;
 
 // Configure Pin Outs and Ins
 _TRISA1 = 0; // pin 3 is an output - AIMDIR
 _TRISA6 = 0; // pin 14 is an output - MVSTEP
 _TRISB2 = 0; // pin 6 is an output - AIMSLP
 _TRISB9 = 0; //pin 13 is an output - MVSLP
 _TRISB8 = 0; //pin 12 is an output - MVLDIR
 _TRISB7 = 0; //pin 11 is an output - MVRDIR
 _TRISB0 = 0; //pin 4 is an output - AIMSTEP
 _TRISA0 = 0; //pin 2 is an output - DCMOTORSON
 _TRISA4 = 1; //pin 10 is an input - LMTSW
 _TRISB14 = 0; //pin 17 is an output - IREMITT(note that this is actually a green LED, not IR)
    
 // Set Initial values for Dir outputs
 _LATA1 = 0; // pin 3 sets the direction for the aim stepper]
 _LATB2 = 1; // pin 6 is high - disabling the slp pin on the aim stepper
 _LATB9 = 0; // pin 13 is high - disablling the slp pin on both move steppers
 _LATB8 = 1; // pin 12 is high sets direction of left stepper
 _LATB7 = 1; // pin 11 is low sets direction for right stepper
 _LATA0 = 0; // pin 2 is low to keep the motors off
 
 //Turn off LED initially
 _LATB14 = 0;//turn off LED initially
     
//-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC1 (on pin 14)
    
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   
    // Set period and duty cycle
    OC1R = 0;              // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC1RS = 20000;             // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
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
    OC1CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    
    
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC2 (on pin 4)
    
    // Clear control bits initially
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period and duty cycle
    OC2R = 0;               // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = 49999;             // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
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
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
///////////////////////////////////////////////////////////////////////    
    
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC3 (on pin 5)
    
    // Clear control bits initially
    OC3CON1 = 0;
    OC3CON2 = 0;
   
    // Set period and duty cycle
    OC3R = 0;               // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC3RS = 49999;             // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC3CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC3CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
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
    OC3CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC3CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
/////////////////////////////////////////////////////////////////////// 
    // OC2 INTERUPT
 _OC2IE = 0;
 _OC2IF = 0;
 _OC2IP = 4;
 
  // OC1 INTERUPT
 _OC1IE = 0;
 _OC1IF = 0;
 _OC1IP = 5;

 // Configure CN interrupt
_CN0IE = 1; // Enable CN on pin 10 (CNEN1 register)
_CN0PUE = 0; // Disable pull-up resistor (CNPU1 register)
_CNIP = 6; // Set CN interrupt priority (IPC4 register)
_CNIF = 0; // Clear interrupt flag (IFS1 register)
_CNIE = 1; // Enable CN interrupts (IEC1 register)   
}

int main(void) {
    PinIOConfig();
    
    state = 1;
    while(1){
        if(state == 1){
            _LATB9 = 1;//activate the move motor
            OC1R = 200;//turn on the move PWM
            desiredSteps_movement = 300;
        }
        else if(state ==2){
            _LATB8 = 0; // pin 12 is high sets direction of left stepper
            _LATB7 = 0; // pin 11 is low sets direction for right stepper
            desiredSteps_movement = 300;
         
        }
    }
    return 0;
}