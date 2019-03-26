/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_flexcan.h"
#include "board.h"
#include "fsl_adc16.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 12U
/* Led */
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN
/* Period */
#define OS_TICK_PERIOD_100MS 100
#define OS_TICK_PERIOD_50MS 50
/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/* CAN defines */
#define EXAMPLE_CAN CAN0
#define EXAMPLE_CAN_CLKSRC kCLOCK_BusClk
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define RX1_MESSAGE_BUFFER_NUM (10)
#define RX2_MESSAGE_BUFFER_NUM (9)
#define TX100_MESSAGE_BUFFER_NUM (8)
#define TX50_MESSAGE_BUFFER_NUM (7)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void task_100ms(void *pvParameters);
static void task_50ms(void *pvParameters);
static void task_rx(void *pvParameters);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool txComplete = false;
volatile bool receiving = false;
volatile bool message_received = false;
volatile uint32_t received_mb_idx;
flexcan_handle_t flexcanHandle;
flexcan_mb_transfer_t tx100Xfer, tx50Xfer, rx1Xfer, rx2Xfer;
flexcan_frame_t tx100Frame, tx50Frame, rx1Frame, rx2Frame;
#define K641 1
#if K64 == 1
uint32_t tx100Identifier = 0x99;
uint32_t tx50Identifier = 0x49;
uint32_t rx1Identifier = 0x106; /*Led*/
uint32_t rx2Identifier = 0x107;	/*ADC*/
#else K64 == 2
uint32_t tx100Identifier = 0x100;
uint32_t tx50Identifier = 0x50;
uint32_t rx1Identifier = 0x101; /*Led*/
uint32_t rx2Identifier = 0x102;	/*ADC*/
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
/*!
 * @brief FlexCAN Call Back function
 */
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {
        /* Process FlexCAN Rx event. */
        case kStatus_FLEXCAN_RxIdle:
        	//receiving = false;
        	message_received = true;
        	received_mb_idx = result;

            break;

        /* Process FlexCAN Tx event. */
        case kStatus_FLEXCAN_TxIdle:
        	txComplete = true;
            break;

        default:
            break;
    }
}

void CAN_Init(void)
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	PRINTF("\r\n==FlexCAN example -- Start.==\r\n\r\n");


	    /* Init FlexCAN module. */
	    /*
	     * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	     * flexcanConfig.baudRate = 125000U;
	     * flexcanConfig.maxMbNum = 16;
	     * flexcanConfig.enableLoopBack = false;
	     * flexcanConfig.enableSelfWakeup = false;
	     * flexcanConfig.enableIndividMask = false;
	     * flexcanConfig.enableDoze = false;
	     */
	    FLEXCAN_GetDefaultConfig(&flexcanConfig);
	#if (!defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)) || !FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE
	    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
	    flexcanConfig.enableLoopBack = false;
	    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

	    /* Create FlexCAN handle structure and set call back function. */
	    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

	    /* Set Rx Masking mechanism. */
	    //FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(rx1Identifier, 0, 0));

	    /* Setup Rx Message Buffers. */
	    mbConfig.format = kFLEXCAN_FrameFormatStandard;
	    mbConfig.type = kFLEXCAN_FrameTypeData;
	    mbConfig.id = FLEXCAN_ID_STD(rx1Identifier);
	    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX1_MESSAGE_BUFFER_NUM, &mbConfig, true);

	    mbConfig.id = FLEXCAN_ID_STD(rx2Identifier);
	    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX2_MESSAGE_BUFFER_NUM, &mbConfig, true);

	    /* Setup Tx Message Buffers. */
	    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX100_MESSAGE_BUFFER_NUM, true);
	    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX50_MESSAGE_BUFFER_NUM, true);
}

void adcInit(void){

	  adc16_config_t adc16ConfigStruct;
	  adc16_channel_config_t adc16ChannelConfigStruct;

	    BOARD_InitPins();
	    BOARD_BootClockRUN();
	    BOARD_InitDebugConsole();

	    PRINTF("\r\nADC16 polling Example.\r\n");

	    /*
	     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	     * adc16ConfigStruct.enableAsynchronousClock = true;
	     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	     * adc16ConfigStruct.enableHighSpeed = false;
	     * adc16ConfigStruct.enableLowPower = false;
	     * adc16ConfigStruct.enableContinuousConversion = false;
	     */
	    ADC16_GetDefaultConfig(&adc16ConfigStruct);
	#ifdef BOARD_ADC_USE_ALT_VREF
	    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif
	    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	    {
	        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	    }
	    else
	    {
	        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	    }
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
	    PRINTF("Press any key to get user channel's ADC value ...\r\n");

	    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

}

uint32_t getAdcValue(){
	adc16_channel_config_t adc16ChannelConfigStruct;

	 ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		        while (0U == (kADC16_ChannelConversionDoneFlag &
		                      ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
		        {
		        }
		        PRINTF("ADC Value: %d\r\n", ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP));


	return ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);

}

void gpioInit(){
	/* Define the init structure for the output LED pin*/
			    gpio_pin_config_t led_config = {
			        kGPIO_DigitalOutput, 0,
			    };
	  /* Init output LED GPIO. */
		 GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
}

int main(void)
{
	/* Initialize board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	CAN_Init();
	adcInit();
	gpioInit();


    xTaskCreate(task_100ms, "100ms Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
    xTaskCreate(task_50ms, "50ms Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
    xTaskCreate(task_rx, "rx Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
    vTaskStartScheduler();

    for (;;);

}

/*!
 * @brief Task responsible for sending the 100ms message.
 */
static void task_100ms(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = OS_TICK_PERIOD_100MS;
	volatile uint32_t can_flags = 0;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	/* Send a Can message */
    	tx100Frame.id = FLEXCAN_ID_STD(tx100Identifier);
    	tx100Frame.format = kFLEXCAN_FrameFormatStandard;
    	tx100Frame.type = kFLEXCAN_FrameTypeData;
    	tx100Frame.length = 8;
    	tx100Xfer.frame = &tx100Frame;
    	tx100Xfer.mbIdx = TX100_MESSAGE_BUFFER_NUM;
    	FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &tx100Xfer);

    	tx100Frame.dataByte0 = 100;
    	tx100Frame.dataByte1++;

        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}

/*!
 * @brief Task responsible for sending the 50ms message.
 */
static void task_50ms(void *pvParameters)
{

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = OS_TICK_PERIOD_50MS;
	volatile uint32_t can_flags = 0;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	/* Send a Can message */
    	tx50Frame.id = FLEXCAN_ID_STD(tx50Identifier);
    	tx50Frame.format = kFLEXCAN_FrameFormatStandard;
    	tx50Frame.type = kFLEXCAN_FrameTypeData;
    	tx50Frame.length = 8;
    	tx50Xfer.frame = &tx50Frame;
    	tx50Xfer.mbIdx = TX50_MESSAGE_BUFFER_NUM;
    	FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &tx50Xfer);



    	tx50Frame.dataWord0 =getAdcValue();


        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}

/*!
 * @brief Task responsible for checking rx messages.
 */
static void task_rx(void *pvParameters)
{
	volatile uint32_t can_flags = 0;
	flexcan_frame_t* rxFrame;

	// Initialize the xLastWakeTime variable with the current time.
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	if(!receiving) {
        	/* Start receive data through Rx Message Buffer. */

        	rx1Xfer.frame = &rx1Frame;
        	rx1Xfer.mbIdx = RX1_MESSAGE_BUFFER_NUM;
    		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx1Xfer);

        	/* Start receive data through Rx Message Buffer. */
        	rx2Xfer.frame = &rx2Frame;
        	rx2Xfer.mbIdx = RX2_MESSAGE_BUFFER_NUM;
    		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx2Xfer);

    		receiving = true;
    	}

    	if(message_received){
    		switch(received_mb_idx){
    		case RX1_MESSAGE_BUFFER_NUM:
    			/* Start the reception over */
            	rx1Xfer.frame = &rx1Frame;
            	rx1Xfer.mbIdx = RX1_MESSAGE_BUFFER_NUM;
        		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx1Xfer);
    			rxFrame = &rx1Frame;
    			break;
    		case RX2_MESSAGE_BUFFER_NUM:
    			/* Start the reception over */
            	rx2Xfer.frame = &rx2Frame;
            	rx2Xfer.mbIdx = RX2_MESSAGE_BUFFER_NUM;
        		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx2Xfer);
    			rxFrame = &rx2Frame;
    			break;
    		default:
    			break;
    		}



    		if(rx1Frame.dataByte0 == 1) GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);




    		PRINTF("Received message: MB: %d, ID: 0x%x, data: %d,%d,%d,%d,%d,%d,%d,%d\r\n", received_mb_idx,
    				rxFrame->id>>18,
    				rxFrame->dataByte0, rxFrame->dataByte1, rxFrame->dataByte2, rxFrame->dataByte3,
					rxFrame->dataByte4, rxFrame->dataByte5, rxFrame->dataByte6, rxFrame->dataByte7
					);
    		message_received = false;
    	}
    	vTaskDelay(2);
    }
}
