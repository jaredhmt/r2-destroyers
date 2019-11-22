/* 
 * File:   movement_demo.c
 * Author: jared.hale
 *
 * Created on October 11, 2019, 13:34
 */

/* 
 * ============================ DEMO 2 INSTRUCTIONS ============================
 * 
 * Demonstrate controlled mobility of your base, including driving forward in a 
 * straight line, making a 90-degree turn, and driving forward again. You may 
 * want to place a weight on your base to simulate the final weight of the 
 * robot. For full points, this should be accomplished under battery power using 
 * all components that you actually want in your final robot. Demonstrate 
 * mobility to the instructor. If your robot is not going to have any mobility, 
 * talk to the instructor about a different milestone.
 * 
 */

/*
 * ---------------- CODE FUNCTIONALITY -------------------
 * This code utilizes 4 states.
 * State 0 - no movement
 * State 1 - moving forward before the turn
 * State 2 - turning 90 degrees CW
 * State 3 - moving forward after the turn
 * 
 * the code has 3 outputs and 1 input.
 * -OUTPUTS
 * pin 2 - direction for left stepper
 * pin 6 - direction for right stepper
 * pin 14 - pwm signal - indicate motor step
 * 
 * -INPUTS
 * pin 5 - switch change notification
 * 
 * The code uses 3 global variables:
 * OC1count - counts number of signals sent through OC1 (pin 14)
 * desiredSteps - number of steps before state should change again
 * state - which state is the robot in
 * 
 * = Motor driver circuit has been breadboarded.
 * 
 * = Distance Forward is now coded in inches as an input. INPUT distanceForward
 * is an input in inches.
 * 
 * ================ CHANGES NEEDED ===================
 * = breadboard needs to be 'rewired' to accept battery supply power. Note we
 * still need to have a common ground. This should free the system from the
 * power supply 
 * 
 */

#include <xc.h>
//#include "MechaLibrary.h"
#pragma config FNOSC = FRCDIV
#define vThresh 1000
#define FCY 400000UL
#include<libpic30.h>
#pragma config OSCIOFNC = OFF

int  distanceForward = 20; // inches

int OC1count = 0; // Set Global Variable OC1count
int desiredSteps = 0; // Set Global Variable desiredSteps
int state = 0; // Set Global Variable state

//-----------------------------------------------------
// Change Notification Interrupt Service Routine (ISR)
// This function executes every time the micro receives
// an interrupt originating from any of the CN pins. The
// micro knows the interrupt is from the one of the CN 
// when the change notification interrupt flag (CNIF)
// is set.
void _ISR _OC1Interrupt(void)
{

    _OC1IF = 0;  // Remember to clear the OC1 interrupt flag when
    // this ISR is entered.

    // PLACE CUSTOM CODE HERE
    OC1count++;
    if(OC1count >= desiredSteps){
        OC1R = 0;
        _LATA1 = 0;
        OC1count = 0;
    }
}

//-----------------------------------------------------
// Main Function
int main(){

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
 _TRISA0 = 0; // pin 2 is an output - Dir Aim Motor
 _TRISA6 = 0; // pin 14 is an output - Pulse OC1
 _TRISA1 = 0; // pin 14 is an output - Pulse OC1
 
 // Set Initial values for Dir outputs
 _LATA0 = 0;
 
 // OC1 INTERUPT
 _OC1IE = 0;
 _OC1IF = 0;
 _OC1IP = 4;

// 
  // Other initialization stuff
    // PLACE OTHER INITIALIZATION CODE HERE


    // Loop and wait - Note that it's empty because the only
    // time anything is done is when the interrupts occur,
    // sending the code to the appropriate ISR (see above)
    // where all the action happens
 
    //idk what this is but Nate has it in his notes
    OC1CON1 = 0;
    OC1CON2 = 0;
    
    //period
    OC1RS = 23999;
    //duty cycle
    OC1R = 0;
    
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = 0;
    OC1CON1bits.OCM = 0b110;
    
   
    
    
    
    
    
    
    desiredSteps = 50;
    
    _LATA1 = 1;
    OC1R = 0;
    
    
    __delay_ms(1000);
    
    //moves bot forward to center
    void moveBot(1,20);
    
    
    //ROTATES HEAD
    while (1) { 
        
        if (ADC1BUF15 >= vThresh && OC1count == 0 && state == 1 ){
            _OC1IE = 1;
            _LATA0 = 1;
            OC1R = 1600;
            state = 0;
            desiredSteps = 50;
        }
        else if (ADC1BUF15 >= vThresh && OC1count == 0 && state == 2 ){
            _OC1IE = 1;
            _LATA0 = 0;
            OC1R = 1600;
            state = 0;
            desiredSteps = 50;
        }
        else if (ADC1BUF13 >= vThresh && OC1count == 0 && state == 0){
            _OC1IE = 1;
            _LATA0 = 0;
            OC1R = 1600;
            state = 1;
            desiredSteps = 50;
        }
        else if (ADC1BUF13 >= vThresh && OC1count == 0 && state == 2){
            _OC1IE = 1;
            _LATA0 = 0;
            OC1R = 1600;
            state = 1;
            desiredSteps = 100;
        }
        else if (ADC1BUF14 >= vThresh && OC1count == 0 && state == 0){
            _OC1IE = 1;
            _LATA0 = 1;
            OC1R = 1600;
            state = 2;
            desiredSteps = 50;
        }
        else if (ADC1BUF14 >= vThresh && OC1count == 0 && state == 1){
            _OC1IE = 1;
            _LATA0 = 1;
            OC1R = 1600;
            state = 2;
            desiredSteps = 100;
        }
    }
    return(0);
}
