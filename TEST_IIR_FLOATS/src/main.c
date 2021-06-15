#include <MKL25Z4.H>
#include <stdio.h>
#include "my_defs.h"


//uint32_t counter=0;
float input[ISZ], window[F_SAMPLING];
float output1[OSZ], output2[OSZ];
float sample, filtered_sample;

// input and ouput indexes
int Win=0, Rin=0, Rout=0, Rout2=0, Rin_minus = Mb-1 - (NUMERATOR_TAPS-1), Wout=0; 		
// Win -> Write index for input buffer -> Used by the producer
// Rin -> Read index for input buffer	pointing at x[n] -> Used by the consumer
// Rout -> Read index for output1 buffer pointing at output1[n]
// Rout2 -> Read index for output2 buffer pointing at output2[n]
// Rin_minus -> Read index for input buffer	pointing at x[n-(Mb-1)] 


int8_t read_sensor_synth(){
	static uint16_t index=(SYNTH_PERIOD+40)%SYNTH_PERIOD;			// this is required to keep track of time
	int8_t sample;																// the generated sample
	static int8_t bw=16;				// bias wandering. it's a triangular wave with period 16*PERIOD and range [-ABS_BW,+ABS_BW]
	static uint8_t noise8=1;		// the noise added to the signal with range [-2,1]
	int8_t noise2=0;						// temporary variable to generate the noise
	static int8_t up_down=1;		// it controls the direction of the bias wandering
	uint8_t bit0, bit1, bit2;		// used to generate the noise with an LFSR
		
	if (index<PEAK_WIDTH)					// sample is a train of pulse, width=10
		sample=PEAK_VAL;						
	else 
		sample =0;	// delta
	noise2=(noise8&3)-2;			// keep only the two LSB and make it signed
	sample=sample+noise2+bw;  // add noise and bias wandering to sample
	
	
	// bias wandering -> triangular wave
	if ((index&0x1F)==0) {// each 32 step the bias wandering is varied
		
		if (bw>=ABS_BW||bw<=-ABS_BW)
			up_down=-up_down;
		bw=bw+up_down;
	}
	// noise generation with 8 bit LFSR
	bit0=noise8&1;
	bit1=(noise8&4)>>2;
	bit2=(noise8&128)>>7;
	
	noise8=(noise8<<1)|(bit0^bit1^bit2);
	

	index++;
  if (index>SYNTH_PERIOD)
			index=0;
	return sample;
	
}

int8_t read_ADC(){
		uint32_t sample;
	
		ADC0->SC1[0] &= ~(ADC_SC1_ADCH_MASK | ADC_SC1_DIFF_MASK);  
	  ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_CHANNEL);
    // Read new sample
	  while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
		sample = ADC0->R[0];
		return (sample);	
}

void Init_pins(void) {
 // 1. Enable the clock for the ports used
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

 // 2. Configure the MUX field in the PCR of pins
	PORTB->PCR[SWITCH_POS] &= ~(PORT_PCR_MUX_MASK|PORT_PCR_PE_MASK);          
	PORTB->PCR[SWITCH_POS] |= ( PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK );          
	PORTB->PCR[RED_LED_POS] &= ~(PORT_PCR_MUX_MASK|PORT_PCR_PE_MASK);          
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
	PORTB->PCR[GREEN_LED_POS] &= ~(PORT_PCR_MUX_MASK);          
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);     
	PORTD->PCR[BLUE_LED_POS] &= ~(PORT_PCR_MUX_MASK);          
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);   	
	
 // 3. Configure the direction (input or outputs) of pins
	PTB->PDDR |= (MASK1(RED_LED_POS) | MASK1(GREEN_LED_POS));
	PTB->PDDR &= ~(MASK1(SWITCH_POS));
	PTD->PDDR &= ~MASK1(SWITCH_POS);
	PTD->PDDR |= MASK1(BLUE_LED_POS);
	
	// 4. Switch off all LEDs
	PTB->PSOR = (MASK1(RED_LED_POS) | MASK1(GREEN_LED_POS) | MASK1(BLUE_LED_POS));
	PTD->PSOR = MASK1(BLUE_LED_POS);
}

void Init_PIT(void) {
 // 1. Enable clock to PIT module
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

 // 2. Enable PIT module, freeze timers in debug mode
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	PIT->MCR |= PIT_MCR_FRZ_MASK;
	
 // 3. Configure that timers are not chained
  PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_CHN_MASK; 

 // 4. Configure the PIT channel to generate interrupt requests
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;

 // 5. Initialize PIT0 to count down from starting_value
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL;	
	
	NVIC_SetPriority(PIT_IRQn, 128); // 0, 64, 128 or 192
  NVIC_ClearPendingIRQ(PIT_IRQn); 
  NVIC_EnableIRQ(PIT_IRQn);	

// 9. Enable timer 
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}



void Init_ADC(void) {
 // 1. Enable clock for ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; // enable ADC
 // 2. Enable clock for the port of the ADC input pin
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

 // 3. Configure input pin (MUX), no pull resistor
	PORTE->PCR[ADC_INPUT_POS] &= ~(PORT_PCR_MUX_MASK|PORT_PCR_PE_MASK);          
	 
	// 4. Configure resolution, clock source and clock division of ADC
	ADC0->CFG1 &= ~(ADC_CFG1_ADIV_MASK | ADC_CFG1_MODE_MASK | ADC_CFG1_ADICLK_MASK); 
	ADC0->CFG1 |= ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(0);  
	// 5. Configure voltage reference and software trigger for ADC
	// The voltage reference for this board is not possible to change it, so we do not do anything
	
	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;		// software trigger (ADTRG=0)
}


void PIT_IRQHandler() {
	uint8_t new_sample_uns;
	float real_sample;
	int8_t new_sample;

	// 1. NVIC: Clear pending IRQ
	NVIC_ClearPendingIRQ(PIT_IRQn);

  // 2. Determine which channel triggered interrupt (even if there is only one possibility)
  if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {

	// 3. Clear interrupt request flag for channel
		PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;  

  // 4. Read new sample
		new_sample_uns = read_ADC();		 
	  //new_sample_uns = read_sensor_synth();		      // read the new sample. The ADC provides an unsigned value
		new_sample = (uint16_t)(new_sample_uns)-128; // now the input value is signed
		real_sample = (float)new_sample;
		input[Win] = real_sample;
		if (real_sample<0.) 
			new_sample_uns=new_sample_uns;

		Win++;				
		if(Win>= ISZ){
			Win=0;
		}
		
		
		if( Win == Rin_minus){  // Check delayed data is going to be overwritten
			PTB->PCOR = MASK1(RED_LED_POS); // Shine RED LED and stay here forever
			PTB->PSOR = MASK1(GREEN_LED_POS); // Shine RED LED and stay here forever
			PTD->PSOR = MASK1(BLUE_LED_POS); // Shine BLUE LED and stay here forever
			
			while(1);										// infinite loop
		}

	}
				
}


/*********************************************************************
 ************************ float signal_threshold() ****************************
 **                         																				
 ** This function gets the value of the peak of every beat from the filtered_samples											
 ** and returns the 30% of this value. The result is stored as the beat threshold 										 
 ** constant.
 ** 
 *********************************************************************/

float signal_threshold(float window[]){
	float thres, max_value=0;
	int i;
	
		// Gets the maximum amplitude value of the window
		for(i=0; i<F_SAMPLING; i++){
			if(window[i]>max_value)
				max_value = window[i];
			}
	thres = max_value - max_value*0.3;
	
	return thres;
	
}
/*********************************************************************
 ************************ void iir_filter() ****************************
 **                         																				
 ** This function filter the input data with an elliptical IIR filter order 6														
 **                         																				
 ** Internally it uses:
 **       	Rin -> location of x[n] in array input[]
 ** 				Rin_minus -> location of x[n-(Mb-1)] in array input[]
 ** 				Wout -> location of y[n]=x[n]*h[n] in array output[]
 **                         																				
 ** output: 
				y[n] = a[m]*y[n-m] +...+ b[m]*x[n-m] +...
 **
 ** IMPORTANT: this function must be called only if Win!=Rin
 **						 to avoid reading old data						 
 **
 *********************************************************************/

void iir_filter(float *x, float *y){
	
	float combination=0, sum=0;
	float FIR_output, IIR_output, notch_fir, notch_iir;
	int i,j,k,m;

	
	j = Rin;
	k = Rout;
	
	for(i=0; i<Mb; i++){
		if (j<0)
			j = ISZ-1;
		if (k<0)
			k = OSZ-1;
		
		  FIR_output = input[j]*B_numerator_taps[i];
			IIR_output = output1[k]*(-A_denominator_taps[i]);
			combination = FIR_output+IIR_output;
			sum += combination;
			j--;
			k--;
		
	}
	
			output1[Wout]=sum;
	
	// NOTCH FILTER APPLICATION
	m = Rout2;
	k = Rout;
	sum = 0;
	combination =0;
	
	for(i=0; i<Nb; i++){
		if (k<0)
			k = OSZ-1;
		if (m<0)
			m = OSZ-1;
		
			notch_fir = output1[k]*NOTCH_NUM[i];
			notch_iir = output2[m]*(-NOTCH_DEN[i]);
			combination = notch_fir+notch_iir;
			sum += combination;
			k--;
			m--;
		
		}
	
			output2[Wout]=sum;

	
	
		// Return values
		*x = input[Rin];
		*y = output2[Wout];
			
	// input index is managed by the PIT timer interrupt
		
		// output index
			Wout++;
			if (Wout==OSZ) 
				Wout=0; // this creates the circular buffer

		// Reading indexes
			Rin++;
			if (Rin==ISZ) 
				Rin=0; // this creates the circular buffer
			
			Rout++;
			if (Rout==OSZ) 
				Rout=0;
			
			Rout2++;
			if (Rout2==OSZ) 
				Rout2=0;
			
			Rin_minus++;
			if (Rin_minus==ISZ) 
				Rin_minus=0; // this creates the circular buffer

		return;
		
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	int i, counter=0;
	float threshold = BEAT_THRESHOLD; // As initialziation, threshold=40
	
	Init_ADC();			// initialize ADC
	Init_PIT();			// initialize periodic timer
	Init_pins();		// intitialize LEDs pins
	
	if (ISZ != OSZ){
		for(i=0;i<ISZ;i++)
			input[i] = 0;
		for(i=0;i<OSZ;i++)
			output1[i] = 0;
		for(i=0;i<OSZ;i++)
			output2[i] = 0;
		
	} else {
			for(i=0;i<ISZ;i++)
				input[i] = 0;
				output1[i] = 0;
				output2[i] = 0;
	}
	
	__enable_irq();		// Sampling starts, so array[] will start being filled in (data from ECG)
	
	
		while (1){
		
		if (Rin!=Win){
			
				
			iir_filter(&sample, &filtered_sample);
				
			if (counter < sizeof(window)) {
					window[counter] = filtered_sample;
					counter++;
					
			} else {
				
					threshold = signal_threshold(window);
				
					if (filtered_sample > threshold)
						PTB->PCOR = MASK1(GREEN_LED_POS); // Shine GREEN LED while y[n]>threshold
								
					else {
						PTB->PSOR = MASK1(GREEN_LED_POS); // Don't shine RED LED while y[n]<=threshold
						PTD->PSOR = MASK1(BLUE_LED_POS);
					}
					
					if ( (PTB->PDIR & MASK1(SWITCH_POS))==0 ){ // When botton pushed once, Terminates the algorithm
							
							PTD->PCOR = MASK1(BLUE_LED_POS);
							return 0; 
						}
				
				
			}
			
		}
		
	}
		
}
