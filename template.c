/* 
 * File:   newTemplate.c
 * Author: jared.hale
 *
 * Created on November 7, 2019
 * 
 * Version History Tracked with Git
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

// Do something here

}



// MAIN FUNCTION
int main(){
    startupConfig();
    
}