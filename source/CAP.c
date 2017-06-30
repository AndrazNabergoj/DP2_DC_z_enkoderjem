/*
 * CAP.c
 *
 *  Created on: 21 Jun 2017
 *      Author: Andraz Nabergoj
 */

#include "CAP.h"

//pozitivna flanka
#define RISING_EDGE		0x00
// resetiranje števca
#define RESET_DELTA		0x01

float time=0;

/**********************************************************
 * inicializacija GPIO za CAP driver
 * return:void
 *********************************************************/
void CAP_init(void)
{
// konfiguracija ECAP1 (enhanced capture)
// konfiguracija registrov ECCTL1
	ECap1Regs.ECCTL1.bit.PRESCALE = 0x8;			// bypass na delilniku, torej uposteva vsak 8i pulz v CAP enoto
	ECap1Regs.ECCTL1.bit.CAP1POL = RISING_EDGE;		// nastavi detekcijo na pozitivno flanko
	ECap1Regs.ECCTL1.bit.CTRRST1 = RESET_DELTA;		// nastavi reset stevca ob capture dogodku
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;				// omogoci nalaganje CAP1-4 registra ob capture dogodkih

// konfiguracija registrov ECCTL2
	ECap1Regs.ECCTL2.bit.CAP_APWM = 0;				// nastavi klasicen ECAP nacin delovanja str. 422
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0x2;			// onemogoci sync-out signal str. 434
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 0x0;			// onemogoci sync-in nacin (nalaganje stevcev CTR sinhrono) str. 434
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0x1;			// TC-CTR free running str. 434
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0x0;			// continuous nacin str. 435

// nastavitve pinov ECAP1 (enhanced capture)
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0x3;   // GPIO5 nastavljen na CAP1 (0x3 = 11)
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;    // GPIO5 je vhod -> 0 input; 1 output)

    EDIS;                                 // Disable EALLOW

}


/**********************************************************
 * merjenje casa med dvema prekinitvama
 * return:void
 *********************************************************/
float CAP_time(void)
{
	/****** gledamo ce ze prozena prekinitev *******/
	if (ECap1Regs.ECFLG.bit.CEVT1 == 1)	// gledamo ce je bit CEVT1 za interrupt postavljen
	{



		ECap1Regs.ECCLR.bit.CEVT1 = 1;	// ce je postavljena CEVT1 prekinitev, pomeni da lahko beremo
		time = ECap1Regs.CAP1;
	}



	return time;

}




