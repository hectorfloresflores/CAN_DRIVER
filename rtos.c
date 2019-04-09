/*
 * CAN.c
 *
 *  Created on: 24/03/2019
 *      Author: Cognati
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* SDK includes. */
#include "gpio_hal.h"
#include "interrupt_manager.h"
#include "clock_manager.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "BoardDefines.h"
#include "S32K144.h" /* include peripheral declarations S32K144 */
#include "CAN.h"/** Include the library that includes the API to use the CAN protocol*/
#include "LPSPI.h"
#include "clocks_and_modes.h"
#include "ADC.h"



/*Data structures for CAN protocol*/
static CAN_transmission_config_t trans_configuration;
static CAN_reception_t reception_data;


/* Priorities at which the tasks are created. */
#define RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define FREQUENCY_MS			( 200 / portTICK_PERIOD_MS )

/* The LED will remain on until the button has not been pushed for a full
5000ms. */
#define mainBUTTON_LED_TIMER_PERIOD_MS		( 5000UL / portTICK_PERIOD_MS )


/* The LED toggle by the queue receive task (blue). */
#define mainTASK_CONTROLLED_LED				( 1UL << 0UL )

/* The LED turned on by the button interrupt, and turned off by the LED timer
(green). */
#define mainTIMER_CONTROLLED_LED			( 1UL << 1UL )

/* The vector used by the GPIO port C.  Button SW7 is configured to generate
an interrupt on this port. */
#define mainGPIO_C_VECTOR					( 61 )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK						( 0UL )

/*-----------------------------------------------------------*/

/*
 * Setup the NVIC, LED outputs, and button inputs.
 */
static void prvSetupHardware( void );
/*
 * The tasks as described in the comments at the top of this file.
 */
static void canAppRx( void *pvParameters );
static void canAppTx( void *pvParameters );
static void adc( void *pvParameters );

/*Global variables*/
bool flagGpio = false;
/*Get global Flag*/
bool getFlagGpio(void);
/*Set global Flag*/
void setFlagGpio(bool flag);
/*Interrup function for button*/
void gpioISR12(void);
/*Initialize gpioInit*/
void gpioInit(void);


/*
 * The LED timer callback function.  This does nothing but switch off the
 * LED defined by the mainTIMER_CONTROLLED_LED constant.
 */
static void prvButtonLEDTimerCallback( TimerHandle_t xTimer );

/*-----------------------------------------------------------*/



/* The LED software timer.  This uses prvButtonLEDTimerCallback() as its callback
function. */
static TimerHandle_t xButtonLEDTimer = NULL;

/*-----------------------------------------------------------*/

void rtos_start( void )
{

	/**Transmission initialization*/
	trans_configuration.can_selected=CAN0;
	trans_configuration.DLC=8;
	trans_configuration.id_standar=0x111;
	trans_configuration.word1=0xB;
	trans_configuration.word2=0xB;
	trans_configuration.trans_semaphore=xSemaphoreCreateMutex();

	/**Reception initialization*/
	reception_data.can_pointer=CAN0;
	reception_data.recv_semaphore=xSemaphoreCreateMutex();

	/**Set the IDs that we want to listen*/
	setID_Rx(0x524, 2);
	setID_Rx(0x234, 3);


	/* Configure the NVIC, LED outputs and button inputs. */
	prvSetupHardware();

	/* Start the two tasks as described in the comments at the top of this
	file. */

	xTaskCreate( canAppRx, "RX", configMINIMAL_STACK_SIZE, NULL, RECEIVE_TASK_PRIORITY, NULL );
	xTaskCreate( canAppTx, "TX", configMINIMAL_STACK_SIZE, NULL, SEND_TASK_PRIORITY, NULL );
	xTaskCreate( adc, "TX", configMINIMAL_STACK_SIZE, NULL, SEND_TASK_PRIORITY, NULL );



	/* Create the software timer that is responsible for turning off the LED
	if the button is not pushed within 5000ms, as described at the top of
	this file. */
	xButtonLEDTimer = xTimerCreate( "ButtonLEDTimer", 			/* A text name, purely to help debugging. */
								mainBUTTON_LED_TIMER_PERIOD_MS,	/* The timer period, in this case 5000ms (5s). */
								pdFALSE,						/* This is a one shot timer, so xAutoReload is set to pdFALSE. */
								( void * ) 0,					/* The ID is not used, so can be set to anything. */
								prvButtonLEDTimerCallback		/* The callback function that switches the LED off. */
							);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();


	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvButtonLEDTimerCallback( TimerHandle_t xTimer )
{
	/* Casting xTimer to void because it is unused */
	(void)xTimer;

	/* The timer has expired - so no button pushes have occurred in the last
	five seconds - turn the LED off. */
	GPIO_HAL_SetPins(PORTC_BASE, (1 << LED1));
}
/*-----------------------------------------------------------*/

/* The ISR executed when the user button is pushed. */
void vPort_C_ISRHandler( void )
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* The button was pushed, so ensure the LED is on before resetting the
	LED timer.  The LED timer will turn the LED off if the button is not
	pushed within 5000ms. */
    GPIO_HAL_ClearPins(LED_GPIO, (1 << LED1));
	/* This interrupt safe FreeRTOS function can be called from this interrupt
	because the interrupt priority is below the
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY setting in FreeRTOSConfig.h. */
	xTimerResetFromISR( xButtonLEDTimer, &xHigherPriorityTaskWoken );

	/* Clear the interrupt before leaving. */

PORT_HAL_ClearPinIntFlagCmd(PORTC,12);
	/* If calling xTimerResetFromISR() caused a task (in this case the timer
	service/daemon task) to unblock, and the unblocked task has a priority
	higher than or equal to the task that was interrupted, then
	xHigherPriorityTaskWoken will now be set to pdTRUE, and calling
	portEND_SWITCHING_ISR() will ensure the unblocked task runs next. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*Task for receiving CAN protocol*/
static void canAppRx( void *pvParameters )
{
	/*Create tick timer*/
	TickType_t xNextWakeTime;
	/*Get function parameters*/
	(void)pvParameters;
	/*Get the tick count from FreeRTOS*/
	xNextWakeTime = xTaskGetTickCount();

	for( ;; ){
		/*If detect a receive flag go through*/
		if(get_Rx_status(&reception_data)){

			receive_CAN(&reception_data);

		}
		/*Task Delay*/
		vTaskDelayUntil( &xNextWakeTime, FREQUENCY_MS );

	}
}

/*CAN for trasmiting CAN protocol*/
static void canAppTx( void *pvParameters )
{
	/*Tick time var*/
	TickType_t xNextWakeTime;
	/*Get function parameters*/
	(void)pvParameters;
	/*Get Tick count*/
	xNextWakeTime = xTaskGetTickCount();

	for( ;; ){

		/*Transmit CAN*/
		transmit_CAN(&trans_configuration);
		/*Toogle led for transmiting*/
		GPIO_HAL_TogglePins(LED_GPIO, (1 << LED1) | (1 << LED2));
		/*Task delay*/
		vTaskDelayUntil( &xNextWakeTime, FREQUENCY_MS );


	}
}

/*ADC task for read voltage*/
static void adc( void *pvParameters )
{

	/*Tick time var*/
	TickType_t xNextWakeTime;
	/*Get function parameters*/
	(void)pvParameters;
	/*Get Tick count*/
	xNextWakeTime = xTaskGetTickCount();
	/*Var to save the value and volatile for avoid optimizing*/
	volatile uint16_t var = 0;
	for( ;; ){

		/*If a button is pressed send ADC value*/
		if(true == getFlagGpio()){
			/*Set flag to false to avoid sending adc value next time until is pressed again*/
			setFlagGpio(false);
			/*Get ADC value*/
			var = getAdcValue();
			/*Load the ADC value in to the CAN struct*/
			trans_configuration.word1 = var;
			/*Task delay*/
			vTaskDelayUntil( &xNextWakeTime, FREQUENCY_MS );

		}

	}
}



/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    WDOG_disable();/*Disable watchdog*/
	SOSC_init_8MHz(); /* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz(); /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	init_CAN(0, 0x555, 250000); /* Initialize CAN peripheral with initial conditions, which they are the frequency and the ID_RX*/
	PORT_init_CAN0(); /* Configure ports */
	LPSPI1_init_master(); /* Initialize LPSPI1 for communication with MC33903 */
	LPSPI1_init_MC33903(); /* Configure SBC via SPI for CAN transceiver operation */



	/* Change BTN1 to input */
	GPIO_HAL_SetPinsDirection(BTN_GPIO, ~(1 << BTN_PIN));

	/* Start with LEDs off. */
	GPIO_HAL_SetPins(LED_GPIO, (0 << LED1) | (0 << LED2));

	/* Install Button interrupt handler */
    INT_SYS_InstallHandler(BTN_PORT_IRQn, vPort_C_ISRHandler, (isr_t *)NULL);


	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS,g_pin_mux_InitConfigArr);

    /* Enable Button interrupt handler */
    INT_SYS_EnableIRQ(BTN_PORT_IRQn);

    /* The interrupt calls an interrupt safe API function - so its priority must
    be equal to or lower than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
    INT_SYS_SetPriority( BTN_PORT_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );


   // Set falling edge isr
    		PORT_HAL_SetPinIntSel(PORTC_BASE,12,PORT_INT_FALLING_EDGE);

    //		Create a enum with PORTC ISR
    		IRQn_Type gpio_irq_id = PORTC_IRQn;

    	//    Set ISR function
    	    INT_SYS_InstallHandler(gpio_irq_id, &gpioISR12, (isr_t*) 0);

    	  //  Enable GPIO ISR
    	    INT_SYS_EnableIRQ(gpio_irq_id);

    	    INT_SYS_SetPriority( gpio_irq_id, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
}




bool getFlagGpio(void){

	return flagGpio;

}

void setFlagGpio(bool flag){

	flagGpio = flag;

}

void gpioISR12(void){

	/*Clear flag so stop interrupting constantly*/
	PORT_HAL_ClearPinIntFlagCmd(PORTC_BASE,12);

	/*Set flag true that will activate the ADC transmission*/
	flagGpio = true;

}

void gpioInit(void){
	/*Initialize gpio ports*/
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS,g_pin_mux_InitConfigArr);


	/*Set falling edge isr*/
	PORT_HAL_SetPinIntSel(PORTC_BASE,12,PORT_INT_FALLING_EDGE);

	/*Create a enum with PORTC ISR*/
	IRQn_Type gpio_irq_id = PORTC_IRQn;

    /*Set ISR function*/
    INT_SYS_InstallHandler(gpio_irq_id, &gpioISR12, (isr_t*) 0);

    /*Enable GPIO ISR*/
    INT_SYS_EnableIRQ(gpio_irq_id);

}



/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	if( xFreeHeapSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}

}
/*-----------------------------------------------------------*/

/* The Blinky build configuration does not include run time stats gathering,
however, the Full and Blinky build configurations share a FreeRTOSConfig.h
file.  Therefore, dummy run time stats functions need to be defined to keep the
linker happy. */
void vMainConfigureTimerForRunTimeStats( void ) {}
unsigned long ulMainGetRunTimeCounterValue( void ) { return 0UL; }

/* A tick hook is used by the "Full" build configuration.  The Full and blinky
build configurations share a FreeRTOSConfig.h header file, so this simple build
configuration also has to define a tick hook - even though it does not actually
use it for anything. */
void vApplicationTickHook( void ) {}

/*-----------------------------------------------------------*/
