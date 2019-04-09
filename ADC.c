

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "pin_mux.h"

#include "adConv1.h"

#include "clockMan1.h"
/* Timeout for PDB in microseconds */
#define PDLY_TIMEOUT 	1000000UL


/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/

#define EVB

#ifdef EVB
	#define ADC_INSTANCE 	0UL
	#define ADC_CHN			12U
	#define ADC_VREFH  		5.0f
	#define ADC_VREFL  		0.0f
	#define PDB_INSTANCE	0UL
#else
	#define ADC_INSTANCE 	1UL
	#define ADC_CHN			10U
	#define ADC_VREFH  		1.2f
	#define ADC_VREFL  		0.0f
	#define PDB_INSTANCE	1UL
#endif


#define welcomeStr "This is an ADC example, it will show you the value converted \
				   \r\nfrom ADC1 Input 11 (or ADC0 Input 12 for EVB)\r\n"
#define headerStr  "ADC result: "

/* Flag used to store if an ADC IRQ was executed */
volatile bool adcConvDone;
/* Variable to store value from ADC conversion */
volatile uint16_t adcRawValue;


uint16_t getAdcValue(void){
	uint16_t value;

	/* Configure ADC channel and software trigger a conversion */
			ADC_DRV_ConfigChan(ADC_INSTANCE, 0U, &adConv1_ChnConfig0);
			/* Wait for the conversion to be done */
			ADC_DRV_WaitConvDone(ADC_INSTANCE);
			/* Store the channel result into a local variable */
			ADC_DRV_GetChanResult(ADC_INSTANCE, 0U, &value);
	return value;
}

void myinit(void){

	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

		ADC_DRV_ConfigConverter(ADC_INSTANCE, &adConv1_ConvConfig0);
		adConv1_ChnConfig0.channel = ADC_CHN;


}



