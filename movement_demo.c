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
 * 
 * ================ CHANGES NEEDED ===================
 * = currently desiredSteps for each state is random. We need to calculate
 * the correct number of steps for a 90 degree rotation. Measurements
 * about the wheel base and diameters needs to be taken.
 * 
 * = physical connections still need to be made
 */

#include <xc.h>
// MECHALIBRARY WAS NOT INCLUDED IN THE LAB
#include "MechaLibrary.h"
#pragma config FNOSC = FRCDIV


int OC1count = 0; // Set Global Variable OC1count
int desiredSteps = 0; // Set Global Variable desiredSteps
int state = 0; // Set Global Variable state

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    // PLACE CODE TO CLEAR THE CN INTERRUPT FLAG HERE
   _CNIF = 0;  // Remember to clear the CN interrupt flag when
                // this ISR is entered.
   // Code For Change Notification Interrupt
   
   // When the Switch is turned ON, start the sequence
    if(_RB1 == 1){
        state = 1; // Change state to be driving forward
        OC1R = 4999; // Set Output Duty Cycle to begin outputting
        desiredSteps = 600;
        OC1count = 0;
    }
    else{
        OC1R = 0;
    }
}


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
       if(state == 1){
           state = 2; // Change State to Turning right
           // set direction pins
           _RA0 = 0;
           _RB2 = 1;
           // set new desired steps
           desiredSteps = 200;
       }
       else if(state == 2){
           state = 3; // Change State to Turning right
           // set direction pins
           _RA0 = 0;
           _RB2 = 0;
           // set new desired steps
           desiredSteps = 600;
       }
       else if(state == 3){
           state = 0;
           // Stop the pulses
           OC1R = 0;
       }
       OC1count = 0;
   }
}

//-----------------------------------------------------
// Main Function
int main()
{

   // Configure the digital I/O ports
//    // PLACE CODE TO CONFIGURE THE DIGITAL I/O PORTS HERE
 ANSA = 0;
 ANSB = 0;
 
 // Configure Pin Outs and Ins
 PinIOConfig(2,0); // pin 2 is an output - Dir Left Motor
 PinIOConfig(5,1); // pin 5 is an input - Switch input
 PinIOConfig(6,0); // pin 6 is an output - Dir Right Motor
 PinIOConfig(14,0); // pin 14 is an output - Pulse OC1
 
 // Set Initial values for Dir outputs
 _RA0 = 0;
 _RB2 = 0;
 
 // OC1 INTERUPT
 _OC1IE = 1;
 _OC1IF = 0;
 _OC1IP = 4;

// 
  // Other initialization stuff
    // PLACE OTHER INITIALIZATION CODE HERE


    // Loop and wait - Note that it's empty because the only
    // time anything is done is when the interrupts occur,
    // sending the code to the appropriate ISR (see above)
    // where all the action happens
  
    //Change notification interrupt
    _CN5IE = 1;
    _CN5PUE = 0;
    _CNIP = 6;
    _CNIF = 0;
    _CNIE =1;
 
    //idk what this is but Nate has it in his notes
    OC1CON1 = 0;
    OC1CON2 = 0;
    
    //period
    OC1RS = 15999;
    //duty cycle
    OC1R = 0;
    
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = 0;
    OC1CON1bits.OCM = 0b110;
    
    while (1) {
        
    }
    return(0);
}
