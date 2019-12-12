/*
 * File:   Demo_V2.c
 * Author: Ta mere
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
 * state = 0 find dispensing corner
 * state = 1 retrieve balls state
 * state = 2 return to center state
 * state = 3 scan sensors state
 * state = 4 aim shooter state
 */

#include "xc.h"
//set up the delay stuff
#define FCY 4000000UL
#include <libpic30.h>

#pragma config SOSCSRC = DIG //required to use pin 9 and 10 as digital I/O
#pragma config OSCIOFNC = OFF//required to use pin 8 as digital I/O

#pragma config FNOSC = FRCDIV//set the oscillator

#define vThresh 2400//threshold for IR detection
#define vThresh2 1000 // threshold for aiming IR detection
#define servo_closed 45000 //PWM signal for closed servo on OC3
#define servo_open 47500 //PWM signal for closed servo on OC3
#define servo_wait 750 //The time it delays for the shooter to turn before opening the servo
#define aim_steps 51 //the number of steps to go 90 degrees

int  distanceForward = 20; // inches

int OC2count = 0; // Set Global Variable OC1count
int desiredSteps_aim = 0; // Set Global Variable desiredSteps
int state = 0; // Set Global Variable state
int substate = -1;//this defines where the shoooter is currently aimed
//0 = front motor, 1 = left goal, 2 = right goal
int desiredSteps_movement = 3000;
int OC1count = 0;
int currentGoal = 0;//defines where the shooter is currently aimed
int desiredGoal = 0;//defines where the shooter should be aimed
int OC3count = 0;
int pollCount = 0;
int IR_High = -1;
//front goal  = 0, left goal = 1, right goal = -1

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void){

    _OC2IF = 0;  // Remember to clear the OC2 interrupt flag when
    // this ISR is entered.
    // PLACE CUSTOM CODE HERE
    OC2count++;
    if(OC2count >= desiredSteps_aim){
        OC2R = 0;
        //_LATA1 = 0;
        OC2count = 0;
    }
}
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){

    _OC1IF = 0;  // Remember to clear the OC1 interrupt flag when
    // this ISR is entered.

    // PLACE CUSTOM CODE HERE
    OC1count++;
    if(state == 0){               //if the state is the initial state
        OC1count = 0;
        if(ADC1BUF15 > vThresh){  //And if the IR sensor toward the dispenser is greater than the IR threshold
            OC1R = 0;             // All right stop, collaborate and listen Ice is back with my brand new invention
            state = 1;            //this sets the state to the move forward and collect ball state
            
        }
        
    }
    else if(OC1count >= desiredSteps_movement && state == 2){
        OC1R = 0;
        _LATB9 = 0; // pin 13 is high - the sleeps the pin on both move steppers
        state = 3;//change to the aim shooter state
        OC2count = 0;
    }
}
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void){
    _OC3IF = 0;  // Remember to clear the OC3 interrupt flag when
    if (state == 3 && OC3R == servo_open){
         OC3count++;
         if (OC3count > 160){
             state = 1;
            OC3R = servo_closed;
            _LATA0 = 0; //turns off DC motors
            _LATB7 = 1;//set directions to backwards
            _LATB8 = 1;//set directions to backwards
            _LATB9 = 1; //Turns on Move steppers
            OC3count = 0; //resets OC3 count 
            if (substate != 0){
                if(substate == 1){
                    OC2count = 0;
                    _LATA1 = 1;
                    desiredSteps_aim = aim_steps;
                    OC2R = 1600;
                }
                else if(substate == 2){
                    OC2count = 0;
                    _LATA1 = 0;
                    desiredSteps_aim = aim_steps;
                    OC2R = 1600;
                }
                substate = -1;
            }
             
        }
    }
}
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    
    // This will handle the limit switch interrupt when the robot moves into the corner.
    //It will flash the LED on and off a certain number of times
    int flashcount = 0;
    
    if(_RA4 == 1 && state == 1){
        OC1R = 0;
        // _LATB9 = 0;
        while(flashcount < 9){
            _LATB14 = 1;//turn on pin 17
            __delay_ms(150);
            _LATB14 = 0;
            __delay_ms(150);
            flashcount++;
        }
        state = 2;//now that we have dispensed the balls, change the state
        OC1count = 0;
        desiredSteps_movement = 600;   
        OC1R = 1000;
    }
    else{
        _LATB14 = 0;
    }
} 
void scan(void){
    if (ADC1BUF14 > ADC1BUF13 && ADC1BUF14 > ADC1BUF9 && ADC1BUF14 > vThresh2){
        IR_High = 1;
    }
    else if (ADC1BUF13 > ADC1BUF14 && ADC1BUF13 > ADC1BUF9 && ADC1BUF13 > vThresh2){
        IR_High = 2;
    }
    else if (ADC1BUF9 > ADC1BUF14 && ADC1BUF9 > ADC1BUF13 && ADC1BUF9 > vThresh2 ){
        IR_High = 0;
    }
    else{
        IR_High = -1;
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
    _CSS13 =  1; //scan chanel 13 = left sensor on pin 7
    _CSS14 = 1; //scan chanel 14 = right sensor on pin 8
    _CSS15 = 1; //scan chanel 15 = back sensor on pin 9(facing dispenser)
    _CSS9 = 1; //scan chanel 9 = front sensor om pin 18
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
    OC3R = servo_closed;               // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC3RS = 49999;             // Period of OC3 to achieve desired PWM 
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
    // OC3 INTERUPT
 _OC3IE = 0;
 _OC3IF = 0;
 _OC3IP = 4;
 
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
//    __delay_ms(1000);
    while (_RB13 == 0){
    }
    __delay_ms(500);
    state = 0;

    while(1){
        if(state ==0){
            _LATB8 = 0;               //Sets the direction pin - Directions don't matter as long as the two are different
            _LATB7 = 1;               //Sets the direction pin - See above
            _OC1IE = 1;
            OC1R = 1000;              //You spin my head right round right round like a record baby right round round round.
            _LATB9 = 1;
        }
        if(state == 1){//move to dispenser and retrieve balls
            OC1RS = 7500;             // Period of OC1 to achieve desired PWM 
            _LATB9 = 1;//activate the move motor
            _LATB7 = 1;//set directions to backwards
            _LATB8 = 1;//set directions to backwards
            desiredSteps_movement = 30000;
            OC1R = 1000;//turn on the move PWM
            
        }
        else if(state == 2){//return to center
            
            _LATB9 = 1;
            _LATB8 = 0; // pin 12 is high sets direction of left stepper
            _LATB7 = 0; // pin 11 is low sets direction for right stepper
            desiredSteps_movement = 600;   
            OC1R = 1000;
        }
        else if(state == 3){// AIM AND SHOOT
            _OC3IE = 1;
            _OC2IE = 1;
            _LATB9 = 0; // SLEEP MOVEMENT STEPPERS
            _LATA0 = 1; // TURN ON DC MOTORS
/*
 *  THIS SECTION VERIFIES THAT EACH IR SENSOR WORKS.
 *  USING THIS CODE, I DISCOVERED THAT THE FRONT IR (9) IS THE MOST SENSITIVE
 *  AND CAN SENSE IR CLICKER FROM ALMOST A FOOT AWAY. THE TWO SIDE SENSORS ARE
 *  MUCH LESS SENSITIVE AND THE CLICKER MUST BE WITHIN AN INCH OF THE SENSOR.
 *  I HAVE NOW VERIFIED THAT OC2count AND substate ARE BOTH 0 WHEN ENTERING
 *  STATE 3. NEXT STEP, UNCOMMENT LOGIC AND HAVE PIN 17 GO HIGH/LOW BASED ON 
 *  substate.
            if(ADC1BUF9 > 0.5 * vThresh && OC2count == 0 && substate == 0){
                _LATB14 = 1;
            }
            else{
                _LATB14 = 0;
            }
*/
            
            // TURN OFF MOVEMENT STEPPERS AND PWM
//                        _LATA0 = 1; // START DC MOTORS
            scan();
            // AIM AND SHOOT
            if (IR_High == 0 && OC2count == 0 && substate == 1 && state == 3){//if front goal is active, and we are currently aimed at left goal
                OC3R = servo_closed;
                _OC2IE = 1;
                _LATA1 = 1;
                desiredSteps_aim = aim_steps;
                OC2R = 1600;
                substate = 0;
                __delay_ms(servo_wait);
                OC3R = servo_open;
                _LATA0 = 1;
            }
            else if (IR_High == 0 && OC2count == 0 && substate == 2 && state == 3){//if front goal is active and we are currently aimed at right goal
                OC3R = servo_closed;
                _LATA1 = 0;
                desiredSteps_aim = aim_steps;
                OC2R = 1600;
                substate = 0;
                __delay_ms(servo_wait);
                OC3R = servo_open;
                _LATA0 = 1;
            }
            else if (IR_High == 1 && OC2count == 0 && (substate == 0 || substate == -1) && state == 3){//if right goal is active and we are currently aimed at front goal
                OC3R = servo_closed;
                _LATA1 = 0;
                desiredSteps_aim = aim_steps;
                OC2R = 1600;
                substate = 1;
                __delay_ms(servo_wait);
                OC3R = servo_open;
                _LATA0 = 1;
            }
            else if (IR_High == 1 && OC2count == 0 && substate == 2 && state == 3){
                OC3R = servo_closed;
                _LATA1 = 0;
                desiredSteps_aim = aim_steps*2;
                OC2R = 1600;
                substate = 1;
                __delay_ms(servo_wait*2);
                OC3R = servo_open;
                _LATA0 = 1;
            }
            else if (IR_High == 2 && OC2count == 0 && (substate == 0 || substate == -1) && state == 3){
                OC3R = servo_closed;
                _LATA1 = 1;
                desiredSteps_aim = aim_steps;
                OC2R = 1600;
                substate = 2;
                __delay_ms(servo_wait);
                OC3R = servo_open;
                _LATA0 = 1;                                                                                                                                                                                                                                                                                                          
            }
            else if (IR_High == 2 && OC2count == 0 && substate == 1 && state == 3){
                OC3R = servo_closed;
                _LATA1 = 1;
                desiredSteps_aim = aim_steps*2;
                OC2R = 1600;
                substate = 2;
                __delay_ms(servo_wait*2);
                OC3R = servo_open;
                _LATA0 = 1;
            } 
            else if (IR_High == 0 && (substate == 0 || substate == -1) && state == 3){
               // pollCount++; //__delay_ms(500);
                //if(pollCount > 000){
                    substate = 0;
                    OC3R = servo_open;
                //    pollCount = 0;
                //}
            }
            //if (ADC1BUF9 < vThresh2 && state == 3){
            //    pollCount = 0;
            //}
        }

    }
    return 0;
}