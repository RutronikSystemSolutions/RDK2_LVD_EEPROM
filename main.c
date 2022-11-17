/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62_LVD_EEPROM
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-10-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Flughafenstr. 4, Erfurt
*  Author: GMO
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

/* The size of data to store in EEPROM. Note flash size used will be
 * closest multiple of flash row size */
#define DATA_SIZE                       CY_EM_EEPROM_FLASH_SIZEOF_ROW
/* The Simple Mode is turned off */
#define SIMPLE_MODE                     (1u)
/* Increases the flash endurance twice */
#define WEAR_LEVELING                   (10u)
/* The Redundant Copy is turned off */
#define REDUNDANT_COPY                  (0u)
/* The Blocking Write is turned on */
#define BLOCKING_WRITE                  (1u)

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_em_eeprom.h"

cy_stc_eeprom_context_t eepromContext;

void handle_error(void);
void LVD_Interrupt(void);
void Setup_LVD(void);

/* Timer object used */
cyhal_timer_t timer_obj;

/* EEPROM */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)
const uint8_t emEepromStorage[CY_EM_EEPROM_GET_PHYSICAL_SIZE(DATA_SIZE,
		SIMPLE_MODE, WEAR_LEVELING, REDUNDANT_COPY)] = { 0u };

cy_stc_eeprom_config_t eepromConfig = { .eepromSize = DATA_SIZE, .simpleMode =
		SIMPLE_MODE, .wearLevelingFactor = WEAR_LEVELING, .redundantCopy =
		REDUNDANT_COPY, .blockingWrite = 1u, .userFlashStartAddr =
		(uint32_t) &(emEepromStorage[0u]), };

const cyhal_timer_cfg_t timer_cfg =
{
    .compare_value = 0,                 /* Timer compare value, not used */
    .period = 0xFFFFFFFF-1,             /* Timer period*/
    .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
    .is_compare = false,                /* Don't use compare mode */
    .is_continuous = true,             	/* Run timer indefinitely */
    .value = 0                          /* Initial value of counter */
};


int main(void)
{
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	Setup_LVD();

	__enable_irq();

	/*Initialize the timer counting up 1000 times per second*/
	result = cyhal_timer_init(&timer_obj, NC, NULL);
	if (result != CY_RSLT_SUCCESS) {
		handle_error();
	}
	result = cyhal_timer_configure(&timer_obj, &timer_cfg);
	if (result != CY_RSLT_SUCCESS) {
		handle_error();
	}

	result = cyhal_timer_start(&timer_obj);
	if (result != CY_RSLT_SUCCESS) {
		handle_error();
	}

	/*Counter for ticks*/
	Cy_TCPWM_Counter_Enable(TCPWM0, 0);
	/* Sets the value of the counter. By reading the EEPROM from last shut down*/
	Cy_TCPWM_Counter_SetCounter(TCPWM0, 0, 0);
	/* Then start the counter */
	Cy_TCPWM_TriggerStart_Single(TCPWM0, 0);

	/*Enable debug output via KitProg UART*/
	result = cy_retarget_io_init(KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
	if (result != CY_RSLT_SUCCESS) {
		handle_error();
	}
	CyDelay(300);

	/*EEPROM*/
	uint32_t numOfStarts = 0;

	Cy_Em_EEPROM_Init(&eepromConfig, &eepromContext);

	Cy_Em_EEPROM_Read(0u, &numOfStarts, 4u, &eepromContext);
	numOfStarts++;
	Cy_Em_EEPROM_Write(0u, &numOfStarts, 4u, &eepromContext);

	uint32_t timeFromEEPROM;
	Cy_Em_EEPROM_Read(4u, &timeFromEEPROM, 4u, &eepromContext);
	Cy_TCPWM_Counter_SetCounter(TCPWM0, 0, timeFromEEPROM);

	for (;;) {
		printf("\x1b[2J\x1b[;H");
		printf("\"RDK2 LVD EEPROM\" is running.\r\n");
		printf("Number of system start ups: %i \n\r", (int) numOfStarts);
		printf("total service life: %is \n\r", (int) Cy_TCPWM_Counter_GetCounter(TCPWM0, 0) / 10);
		printf("session time: %is \r\n", (int) (Cy_TCPWM_Counter_GetCounter(TCPWM0, 0) / 10 - timeFromEEPROM / 10));

		CyDelay(1000);
	}
}

void Setup_LVD(void) {
	Cy_LVD_ClearInterruptMask();
	Cy_LVD_SetThreshold(CY_LVD_THRESHOLD_3_1_V);
	Cy_LVD_SetInterruptConfig(CY_LVD_INTR_FALLING);
	Cy_LVD_Enable();
	Cy_SysLib_DelayUs(20U);
	Cy_LVD_ClearInterrupt();
	Cy_LVD_SetInterruptMask();

	/* Scenario: Configure and enable the LVD interrupt. */
	const cy_stc_sysint_t LVD_IRQ_cfg = { .intrSrc = srss_interrupt_IRQn, /* Interrupt source is the SAR interrupt */
	.intrPriority = 7UL /* Interrupt priority is 7 */
	};

	/* Enable the interrupt. */
	NVIC_EnableIRQ(LVD_IRQ_cfg.intrSrc);
	/* Configure the interrupt with vector at SAR_Interrupt(). */
	(void) Cy_SysInt_Init(&LVD_IRQ_cfg, LVD_Interrupt);

}

void LVD_Interrupt(void) {
	uint32_t buffer;
	buffer = Cy_TCPWM_Counter_GetCounter(TCPWM0, 0);
	Cy_Em_EEPROM_Write(4u, &buffer, 4u, &eepromContext);
	Cy_LVD_ClearInterrupt();
	while (1) {
	}

}

void handle_error(void) {
	/* Disable all interrupts. */
	__disable_irq();

	CY_ASSERT(0);
}
/* [] END OF FILE */


