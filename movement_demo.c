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
 * 
 * 
 */

// COPY CODE FROM Lab 5
// Code has not been adjusted. Current functionality is to rotate one stepper
// motor 180 degrees CW, 270 degrees CCW, then 720 degrees CW at double speed.

#include <xc.h>
// MECHALIBRARY WAS NOT INCLUDED IN THE LAB
#include <MechaLibrary.h>
#pragma config FNOSC = FRCDIV


int OC1count = 0; // Set Global Variable OC1count
int desiredSteps = 0; // Set Global Variable desiredSteps

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    // PLACE CODE TO CLEAR THE CN INTERRUPT FLAG HERE
   _CNIF = 0;  // Remember to clear the CN interrupt flag when
                // this ISR is entered.
   // WHEN PIN 5 is changed
   if(_RB1==1) {
      OC1R = 3999;
      desiredSteps = 200;
      _RA0 = 0;
      OC1count = 0;
      OC1RS = 15999;
   }
   else {
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
    // PLACE CODE TO CLEAR THE CN INTERRUPT FLAG HERE


    // Place in this ISR whatever code should be executed
    // when a change in the button state is detected.
    // You will need to write code to distinguish between
    // a button press and a button release.
    // PLACE CUSTOM CODE HERE
   OC1count++;
   if(OC1count >= desiredSteps){
       if(desiredSteps == 200){
           desiredSteps = 300;
           //Change Dir
           _RA0 = 1;
           
       }
       else if(desiredSteps == 300){
           desiredSteps = 800;
           OC1RS = 7999;
           //Change Dir Again
           _RA0 = 0;
       }
       else{
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
 _TRISA0 = 0; // pin 2 out -> dir
// _TRISB2 = 0; // pin 6 out ->I2
 
 // THIS LINE WAS UNCOMMENTED IN THE LAB.
// _TRISB1 = 1; //pin 5 is an input...switch
 
 // THIS LINE WAS NOT INCLUDED IN THE LAB. THIS IS A TEST TO SEE IF THE LIBRARY
 // IS FUNCTIONAL
 
 PinIOConfig(5,0);
 
 
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
 
  //idk what this is but I have it in my notes
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
    
    // Set init direction
    _RA0 = 0;
    
    //duty cycle
//    OC1R = 3999;
    desiredSteps = 200;
    
    while (1) {
        
    }
    
    return(0);
}
