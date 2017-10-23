/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Deep sleep application
 *
 * Copyright (C) 2015 - 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */


#include <stdio.h>
#include <string.h>

#include "osal.h"
#include "resmgmt.h"

#include "sys_clock_mgr.h"
#include "sys_power_mgr.h"
#include "sys_watchdog.h"
#include "hw_wkup.h"
#include "hw_timer0.h"
#include "hw_uart.h"
#include "hw_gpio.h"
#include "hw_cpm.h"

/* The bit on port 0 to which the LED is wired. */
#define mainLED_BIT		( 1UL << 7UL )

/* Initial sleep mode */
#ifndef INITIAL_SLEEP_MODE
#define INITIAL_SLEEP_MODE      pm_mode_extended_sleep
#endif

/* The configCHECK_FOR_STACK_OVERFLOW setting in FreeRTOSConifg can be used to
check task stacks for overflows.  It does not however check the stack used by
interrupts.  This demo has a simple addition that will also check the stack used
by interrupts if mainCHECK_INTERRUPT_STACK is set to 1.  Note that this check is
only performed from the tick hook function (which runs in an interrupt context).
It is a good debugging aid - but won't catch interrupt stack problems until the
tick interrupt next executes. */
//#define mainCHECK_INTERRUPT_STACK			1
#if mainCHECK_INTERRUPT_STACK == 1
	const unsigned char ucExpectedInterruptStackValues[] = { 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };
#endif

#if dg_configUSE_WDOG
INITIALISED_PRIVILEGED_DATA int8_t idle_task_wdog_id = -1;
#endif

/*-----------------------------------------------------------*/

PRIVILEGED_DATA static unsigned long ulLEDState = 0UL;
PRIVILEGED_DATA static uint32_t blink_cnt;
PRIVILEGED_DATA static OS_TIMER xTimerBlink;

static OS_TASK xHandle = NULL;

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware( void );

/*
 * The hardware only has a single LED.  Simply toggle it.
 */
void vMainToggleLED( void );

/*-----------------------------------------------------------*/

void wkup_handler(void)
{
        hw_wkup_reset_interrupt();
        blink_cnt = 0;

        OS_TIMER_START_FROM_ISR( xTimerBlink );
}

/*-----------------------------------------------------------*/

void vTimerCallback(OS_TIMER pxTimer)
{
        vMainToggleLED();

        blink_cnt++;
        hw_cpm_trigger_sw_cursor();
        if (blink_cnt == 10) {                  /* 10 blinks */
                OS_TIMER_STOP( xTimerBlink, OS_TIMER_FOREVER);
                blink_cnt = 0;
        }
}
/*-----------------------------------------------------------*/

static void system_init(void *pvParameters)
{
        OS_BASE_TYPE status;
        char buf[] = "This is a test message!\n";

        /* Prepare clocks. Note: cm_cpu_clk_set() and cm_sys_clk_set() can be called only from a
         * task since they will suspend the task until the XTAL16M has settled and, maybe, the PLL
         * is locked.
         */
        cm_sys_clk_init(sysclk_XTAL16M);
        cm_apb_set_clock_divider(apb_div1);
        cm_ahb_set_clock_divider(ahb_div1);
        cm_lp_clk_init();

        /*
         * Initialize platform watchdog
         */
        sys_watchdog_init();

#if dg_configUSE_WDOG
        // Register the Idle task first.
        idle_task_wdog_id = sys_watchdog_register(false);
        ASSERT_WARNING(idle_task_wdog_id != -1);
        sys_watchdog_configure_idle_id(idle_task_wdog_id);
#endif

        /* Prepare the hardware to run this demo. */
        prvSetupHardware();

        /* init resources */
        resource_init();

        /* Set the desired sleep mode. */
        pm_set_sleep_mode(INITIAL_SLEEP_MODE);

        cm_sys_clk_set(sysclk_XTAL16M);
        hw_uart_send(HW_UART2, buf, strlen(buf)+1, NULL, NULL);

        /* Program WKUPCT to react to the button. */
        hw_wkup_init(NULL);
        hw_wkup_set_debounce_time(32);
        hw_wkup_set_counter_threshold(1);
        hw_wkup_set_pin_state(HW_GPIO_PORT_1, HW_GPIO_PIN_6, true);
        hw_wkup_set_pin_trigger(HW_GPIO_PORT_1, HW_GPIO_PIN_6, HW_WKUP_PIN_STATE_LOW);
        hw_wkup_register_interrupt(wkup_handler, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

        /* Create the Timers. */
        xTimerBlink = OS_TIMER_CREATE( "Timer0",
                                OS_MS_2_TICKS(1000),            /* Expire after 1 sec */
                                pdTRUE,                         /* Run repeatedly */
                                (void *) 0,                     /* Timer id == index */
                                vTimerCallback );               /* call-back */
        OS_ASSERT(xTimerBlink != NULL);


        (void) status;          // To satisfy the compiler...

        OS_TASK_SUSPEND( xHandle );
}
/*-----------------------------------------------------------*/

int main(void)
{
        OS_BASE_TYPE status;

        cm_clk_init_low_level();                                /* Basic clock initializations. */

        /* In RAM projects, the debugger must be detached in order for sleep to take place. */
        if (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE) {
                pm_wait_debugger_detach(INITIAL_SLEEP_MODE);
        }

        /* Start the two tasks as described in the comments at the top of this
        file. */
        status = OS_TASK_CREATE("SysInit",                                      /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                                system_init,                                    /* The System Initialization task. */
                                ( void * ) 0,                                   /* The parameter passed to the task. */
                                configMINIMAL_STACK_SIZE * OS_STACK_WORD_SIZE,  /* The size of the stack to allocate to the task. */
                                OS_TASK_PRIORITY_HIGHEST,                       /* The priority assigned to the task. */
                                xHandle);                                       /* The task handle is required. */
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);

        /* Start the tasks and timer running. */
        vTaskStartScheduler();

        /* If all is well, the scheduler will now be running, and the following
        line will never be reached.  If the following line does execute, then
        there was insufficient FreeRTOS heap memory available for the idle and/or
        timer tasks     to be created.  See the memory management section on the
        FreeRTOS web site for more details. */
        for( ;; );
}
/*-----------------------------------------------------------*/

#       if dg_configBLACK_ORCA_MB_REV == BLACK_ORCA_MB_REV_D
#               define LED_PORT         HW_GPIO_PORT_1
#               define LED_PIN          HW_GPIO_PIN_5
#       else
#               error "Unknown value for dg_configBLACK_ORCA_MB_REV!"
#       endif
void vMainToggleLED( void )
{
        if( ulLEDState == 0UL )
                hw_gpio_set_active(LED_PORT, LED_PIN);
        else
                hw_gpio_set_inactive(LED_PORT, LED_PIN);
	ulLEDState = !ulLEDState;
}
/*-----------------------------------------------------------*/

/**
 * \brief Initialize the peripherals domain after power-up.
 *
 * \return void
 *
 */
static void periph_init(void)
{
        /*
         * Set-up Peripheral GPIOs
         */

        /*
         * Power up peripherals
         */

        /*
         * GPIO initializations
         */
        /* Configure GPIO for LED output. */
        hw_gpio_set_pin_function(LED_PORT, LED_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
        if (ulLEDState == 1UL) {
                hw_gpio_set_active(LED_PORT, LED_PIN);
        }
        else {
                hw_gpio_set_inactive(LED_PORT, LED_PIN);
        }

        /* Configure GPIO for button. */
        hw_gpio_set_pin_function(HW_GPIO_PORT_1, HW_GPIO_PIN_6, HW_GPIO_MODE_INPUT_PULLUP, HW_GPIO_FUNC_GPIO);
}
/*-----------------------------------------------------------*/



static void prvSetupHardware(void)
{
#if mainCHECK_INTERRUPT_STACK == 1
        extern unsigned long _vStackTop[], _pvHeapStart[];
        unsigned long ulInterruptStackSize;
#endif

        /* Init hardware */
        pm_system_init(periph_init);

        uart_config cfg = {
                .baud_rate      = HW_UART_BAUDRATE_115200,
                .data           = HW_UART_DATABITS_8,
                .stop           = HW_UART_STOPBITS_1,
                .parity         = HW_UART_PARITY_NONE,
                .use_dma        = 0,
                .use_fifo       = 1,
                .rx_dma_channel = HW_DMA_CHANNEL_0,
                .tx_dma_channel = HW_DMA_CHANNEL_1,
        };

        /*
         * First init should be done before anything else tries to write to UART so we don't use
         * any resource locking here.
         */
        hw_uart_init(HW_UART2, &cfg);

        hw_gpio_set_pin_function(HW_GPIO_PORT_1, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                HW_GPIO_FUNC_UART2_TX);
        hw_gpio_set_pin_function(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                HW_GPIO_FUNC_UART2_RX);


#if mainCHECK_INTERRUPT_STACK == 1
	/* The size of the stack used by main and interrupts is not defined in
	the linker, but just uses whatever RAM is left.  Calculate the amount of
	RAM available for the main/interrupt/system stack, and check it against
	a reasonable number.  If this assert is hit then it is likely you don't
	have enough stack to start the kernel, or to allow interrupts to nest.
	Note - this is separate to the stacks that are used by tasks.  The stacks
	that are used by tasks are automatically checked if
	configCHECK_FOR_STACK_OVERFLOW is not 0 in FreeRTOSConfig.h - but the stack
	used by interrupts is not.  Reducing the conifgTOTAL_HEAP_SIZE setting will
	increase the stack available to main() and interrupts. */
	ulInterruptStackSize = ( ( unsigned long ) _vStackTop ) - ( ( unsigned long ) _pvHeapStart );
	OS_ASSERT( ulInterruptStackSize > 350UL );

	/* Fill the stack used by main() and interrupts to a known value, so its
	use can be manually checked. */
	memcpy( ( void * ) _pvHeapStart, ucExpectedInterruptStackValues, sizeof( ucExpectedInterruptStackValues ) );
#endif
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to OS_MALLOC() fails.
	OS_MALLOC() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to OS_MALLOC() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call OS_QUEUE_GET() with a block time
	specified, or call OS_DELAY()).  If the application makes use of the
	OS_TASK_DELETE() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */

#if dg_configUSE_WDOG
        sys_watchdog_notify(idle_task_wdog_id);
#endif
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(OS_TASK pxTask, char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
#if mainCHECK_INTERRUPT_STACK == 1
extern unsigned long _pvHeapStart[];

	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */

	/* Manually check the last few bytes of the interrupt stack to check they
	have not been overwritten.  Note - the task stacks are automatically
	checked for overflow if configCHECK_FOR_STACK_OVERFLOW is set to 1 or 2
	in FreeRTOSConifg.h, but the interrupt stack is not. */
	OS_ASSERT( memcmp( ( void * ) _pvHeapStart, ucExpectedInterruptStackValues, sizeof( ucExpectedInterruptStackValues ) ) == 0U );
#endif /* mainCHECK_INTERRUPT_STACK */
}
/*-----------------------------------------------------------*/





