/*
 * CAP.h
 *
 *  Created on: 21 Jun 2017
 *      Author: Andraz Nabergoj
 */

#ifndef INCLUDE_CAP_H_
#define INCLUDE_CAP_H_

#include    "DSP28x_Project.h"

#include    "define.h"
#include    "globals.h"

/**********************************************************
 * inicializacija GPIO za CAP driver
 * return:void
 *********************************************************/
extern void CAP_init(void);


/**********************************************************
 * merjenje casa med dvema prekinitvama
 * return:void
 *********************************************************/
extern float CAP_time(void);



#endif /* INCLUDE_CAP_H_ */
