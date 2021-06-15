#ifndef MY_DEF_H
#define MY_DEF_H

#include "float32.h"
#include "notch32.h"

// PIT timer parameters
#define F_CLOCK (24e6)
#define F_SAMPLING (500)  // sampling frequency in Hz -> Max Fs: 15473 Hz
#define PIT_LDVAL ((uint32_t)(F_CLOCK/F_SAMPLING)-1)  // Value of register PIT_LDVAL


// PIN information
#define MASK1(x) (1UL << (x))
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)	// on port B
#define BLUE_LED_POS (1)    // on port D
#define SWITCH_POS (0)	// on port B

// AND: Channel ADC0_SE3 is PTE22
#define ADC_CHANNEL (3) 
#define ADC_INPUT_POS (22) // portE 

// READ_SYNTH_SAMPLE parameters
#define F_BEAT (55./60.)		// beat frequency in Hz (beats per minute/60)
#define SYNTH_PERIOD	((int)(F_SAMPLING/F_BEAT))	// this is for the read_sensor() function
																				    // every PERIOD cycles a new beat will be synthesized
// Beat detection parameters
#define BEAT_THRESHOLD (4)

#define ABS_BW (16)												// absolute value of the maximum for bias wandering [-ABS_BW,+ABS_BW]
#define PEAK_VAL (64)											// peak value
#define PEAK_WIDTH (3)										// peak width


// BUFFERS
// Size of arrays
#define Mb (NUMERATOR_TAPS)	// Number of 	numerator coefficients of iir filter
#define Ma (DENOMINATOR_TAPS)		// Number of denumerator coefficients of iir filter
#define Na (NOTCH_NUMERATOR)	// Number of 	numerator coefficients of notch filter
#define Nb (NOTCH_DENOMINATOR)	// Number of 	denumerator coefficients of notch filter

#define ISZ Mb+1	    					// the size of the input vector must be at least equal to the size of b(n)
#define INSZ Nb+1							 // the size of the notch input vector (output1) must be at least equal to the size its b(n)
#define OSZ (100)						// Size of the ouput buffer



#endif
