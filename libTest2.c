/* 
 * File:   libTest.c
 * Author: jared.hale
 *
 * Created on November 7, 2019
 * 
 * Version History Tracked with Git
 * Test Library Setup
 * 
 */

#include <xc.h>
#include "MechaLibrary.h"

#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

// DEFINE GLOBAL VARIABLES HERE


// SET CLOCK TO USE HERE


// SET ISR's HERE
void _ISR _CNInterrupt(void)
{
_CNIF = 0; // Clear interrupt flag (IFS1 register)
// Figure out which pin changed if you?re looking at multiple pins 
_LATB0 = _LATB0 ^ 1;
_LATB1 = _LATB1 ^ 1;
_LATB2 = _LATB2 ^ 1;
_LATB7 = _LATB7 ^ 1;
_LATB8 = _LATB8 ^ 1;
_LATB9 = _LATB9 ^ 1;
_LATA6 = _LATA6 ^ 1;
_LATB14 = _LATB14 ^ 1;
// Do something here

}



// MAIN FUNCTION
int main(){
    startupConfig();
    
    _LATB0 = 0;
    _LATB1 = 0;
    _LATB2 = 0;
    _LATB7 = 0;
    _LATB8 = 0;
    _LATB9 = 0;
    _LATA6 = 0;
    _LATB14 = 0;
    
    while(1){
        
    }
}
