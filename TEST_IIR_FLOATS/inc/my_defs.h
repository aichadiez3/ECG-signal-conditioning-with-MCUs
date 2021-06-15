#ifndef MY_DEF_H
#define MY_DEF_H

#include "elliptical_IIR.h"

// PIT timer parameters
#define F_CLOCK (24e6)
#define F_SAMPLING (500)  // sampling frequency in Hz
#define PIT_LDVAL ((uint32_t)(F_CLOCK/F_SAMPLING)-1)  // Value of register PIT_LDVAL


// PIN information
#define MASK(x) (1UL << (x))
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)	// on port B
#define SWITCH_POS (0)	// on port B

// AND: Channel ADC0_SE3 is PTE22
#define ADC_CHANNEL (3) 
#define ADC_INPUT_POS (22) // portE 

// READ_SYNTH_SAMPLE parameters
#define F_BEAT (55./60.)		// beat frequency in Hz (beats per minute/60)
#define SYNTH_PERIOD	((int)(F_SAMPLING/F_BEAT))	// this is for the read_sensor() function
																				    // every PERIOD cycles a new beat will be synthesized
// Beat detection parameters
#define BEAT_THRESHOLD (60)

#define ABS_BW (16)												// absolute value of the maximum for bias wandering [-ABS_BW,+ABS_BW]
#define PEAK_VAL (64)											// peak value
#define PEAK_WIDTH (3)										// peak width


// BUFFERS
// Size of arrays
#define Mb (NUMERATOR_TAPS)	// Number of 	numerator coefficients
#define Ma (DENOM_TAPS)		// Number of denumerator coefficients

#define OSZ (1000)						// Size of the ouput buffer
#define ISZ Mb+1	    					// the size of the input vector must be at least equal to the size of h(n)
#define SIZE_OF_BUFFER 10

/*	-----------> JORGE THINGS <-------------
#define LDV 23900 							// it controls the sampling frequency. we put here the velocity
#define MAX_COEFF 100						// no. of FIR  coefficients (Used in Jorge's tests)

// ANS: LDV = round(T*fcount - 1) = round (1 ms*24 Mhz -1) (Unit 6: slide 14)
//      LDV = round(1e-3*24e6 -1 )=round(24000-1)=23999
//			fs=  1 KHz -> 23999
//      fs= 10 KHz -> 2399
//			fs=100 KHZ -> 239
//			fs=200 KHZ -> 119 (about the limit)
//      check also (UNIT 6: slide 13)

*/


#endif
