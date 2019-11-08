/* 
 * File:   HaleMechaLib.c
 * Author: jared.hale
 *
 * Created on September 26, 2019, 10:34 AM
 */
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <xc.h>
// ATTEMPTING TO CREATE A LIBRARY
/*
 * 
 */

void PinIOConfig(int pin, int mode);

void Timer1Setup(int power, int preScale, int period, int timeReset);

int moveBot(char dir, float dist);

void startupConfig();