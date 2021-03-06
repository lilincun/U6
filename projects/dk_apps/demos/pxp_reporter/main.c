/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Proximity Reporter
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <string.h>
#include <stdbool.h>

#include "osal.h"
#include "resmgmt.h"
#include "ad_ble.h"
#include "ad_nvms.h"
#include "ad_nvparam.h"
#include "ble_mgr.h"
#include "hw_cpm.h"
#include "hw_gpio.h"
#include "sys_clock_mgr.h"
#include "sys_power_mgr.h"
#include "sys_watchdog.h"
#include "platform_devices.h"


/* Task priorities */
#define mainPXP_REPORTER_TASK_PRIORITY              ( OS_TASK_PRIORITY_NORMAL )

#define SGM2019_EN_PORT HW_GPIO_PORT_3
#define SGM2019_EN_PIN HW_GPIO_PIN_4

#if (dg_configTRACK_OS_HEAP == 1)
/*
 * ConstantsVariables used for Tasks Stack and OS Heap tracking
 * Declared global to avoid IDLE stack Overflows
 */
#define mainMAX_NB_OF_TASKS           10
#define mainMIN_STACK_GUARD_SIZE      8 /* words */
#define mainTOTAL_HEAP_SIZE_GUARD     64 /*bytes */

TaskStatus_t pxTaskStatusArray[mainMAX_NB_OF_TASKS];
uint32_t ulTotalRunTime;
#endif /* (dg_configTRACK_OS_HEAP == 1) */

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

OS_TASK pps960_task2 = NULL ;
OS_TASK wkup_handle  = NULL;
OS_TASK updata_handle  = NULL;

OS_TASK updata_handle_1 = NULL;

uint8_t pm_mode = 0 ;
/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware( void );

void pxp_reporter_task( void *pvParameters); // ble

void mc3630_sensor_task(void *params);//yoko


void pps960_sensor_task(void *params);//skn pps960
void pps960_sensor_task2(void *params);//pps960 alg



void wkup_task(void *params);//sleep

void updata_task(void *params);//updata

//void timer0_task(void *params);//pwm motor
//void breath_task(void *params);//breat light

//void test_task(void *params); //test demo

void rtc_task(void *params);//rtc

#if 0
void test_task(void *params)
{
        for(;;)
        {
                OS_DELAY_MS(2000);
                printf("test 123 \r\n");
        }
}
#endif

void periph_setup(void);//SKN  WU pin init

#if defined CONFIG_RETARGET
        extern void retarget_init(void);
#endif

static void system_init( void *pvParameters )
{

        OS_TASK handle;
        //hw_gpio_configure_pin(SGM2019_EN_PORT,SGM2019_EN_PIN,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,0);//SGM2019 Control PIN disable
        //hw_gpio_set_active(SGM2019_EN_PORT, SGM2019_EN_PIN);//SGM2019 Control PIN enable

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

         /* Set system clock */
        cm_sys_clk_set(sysclk_XTAL16M);

        /* Prepare the hardware to run this demo. */
        prvSetupHardware();

//#if defined CONFIG_RETARGET
//        retarget_init();
//#endif

        /* Set the desired sleep mode. */
        pm_set_wakeup_mode(true);
#if 1
        #if 1
                ad_flash_read(0x100000,&pm_mode, 1);
                if(pm_mode == 1)
                {
                        uint8_t mode_data = 0 ;
                        ad_flash_write(0x100000, &mode_data, 1);
                        pm_set_sleep_mode(pm_mode_hibernation);
                }
                else//pm_mode == 0
                {
                        pm_set_sleep_mode(pm_mode_extended_sleep);
                        //uint8_t  aaaaa = 2 ;
                        //ad_flash_write(0x100001, &aaaaa, 1);
                }
                //memset(buf,0,10);
        #else
                pm_mode = 1;
                pm_set_sleep_mode(pm_mode_hibernation);
        #endif

#else
        // 每次擦除 4k 所以地址要以4k对齐
        ad_flash_erase_region(0x100000, 1);

        uint8_t init_pm_mode = 1 ;
        ad_flash_write(0x100000, &init_pm_mode, 1);

        pm_set_sleep_mode(pm_mode_extended_sleep);
#endif


        pm_stay_alive();
        //pm_resume_sleep();

#if CFG_DEMO_SENSOR_PPS960 || CFG_DEMO_SENSOR_MC3630
        ad_i2c_init();
#endif


#if CFG_DEMO_SENSOR_PPS960
        I2C_BUS_INIT(I2C1);
        I2C_DEVICE_INIT(PPS960);
#endif

#if CFG_DEMO_SENSOR_MC3630
       I2C_BUS_INIT(I2C2);
       I2C_DEVICE_INIT(MC3630);
#endif

        /* init resources */
        resource_init();

        /* Initialize BLE Manager */
        ble_mgr_init();

        hw_gpio_configure_pin(SGM2019_EN_PORT,SGM2019_EN_PIN,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,0);//SGM2019 Control PIN disable
        hw_gpio_set_active(SGM2019_EN_PORT, SGM2019_EN_PIN);//SGM2019 Control PIN enable

#if CFG_DEMO_PXP_BLE
        /* Start the PXP reporter application task. */
        OS_TASK_CREATE("PXP Reporter",                  /* The text name assigned to the task, for
                                                           debug only; not used by the kernel. */
                       pxp_reporter_task,               /* The function that implements the task. */
                       NULL,                            /* The parameter passed to the task. */
#if (dg_configDISABLE_BACKGROUND_FLASH_OPS == 1)
                       512,                             /* The number of bytes to allocate to the
                                                           stack of the task. */
#else
                       //756,                             // The number of bytes to allocate to the
                       512+256,                                   //stack of the task.
#endif
                       mainPXP_REPORTER_TASK_PRIORITY,  /* The priority assigned to the task. */
                       updata_handle_1);                         /* The task handle. */
        OS_ASSERT(updata_handle_1);
#endif



#if CFG_DEMO_SENSOR_MC3630
         /* Start the sensor application task. */
        OS_TASK_CREATE("mc3630_sensor_task",                   /* The text name assigned to the task, for
                                                                           debug only; not used by the kernel. */
                              mc3630_sensor_task,                      /* The function that implements the task. */
                              NULL,                                   /* The parameter passed to the task. */
                              // 512+256,                                     /* The number of bytes to allocate to the
                              1024,                                   /* stack of the task. */
                              3,                                      /* The priority assigned to the task. */
                              handle);                                /* The task handle. */

                OS_ASSERT(handle);
#endif



#if CFG_DEMO_SENSOR_PPS960
        /* Start the sensor application task. */
        OS_TASK_CREATE("pps960_sensor_task",                   /* The text name assigned to the task, for
                                                                   debug only; not used by the kernel. */
                      pps960_sensor_task,                      /* The function that implements the task. */
                      NULL,                                    /* The parameter passed to the task. */
                      512+256,                                 /* The number of bytes to allocate to the
                                                                  stack of the task. */
                      3,                                       /* The priority assigned to the task. */
                      handle);                                 /* The task handle. */

        OS_ASSERT(handle);

        /* Start the sensor application task. */
        OS_TASK_CREATE("pps960_sensor_task2",                  /* The text name assigned to the task, for
                                                                   debug only; not used by the kernel. */
                      pps960_sensor_task2,                     /* The function that implements the task. */
                      NULL,                                    /* The parameter passed to the task. */
                      //512+256,
                      1024,                                    /* The number of bytes to allocate to the
                                                                  stack of the task. */
                      2,                                       /* The priority assigned to the task. */
                      pps960_task2);                           /* The task handle. */

        OS_ASSERT(pps960_task2);
#endif



#if 1
        /* Start the sensor application task. */
        OS_TASK_CREATE("wkup_task",                   /* The text name assigned to the task, for
                                                         debug only; not used by the kernel. */
                      wkup_task,                      /* The function that implements the task. */
                      NULL,                           /* The parameter passed to the task. */
                      512+128,                        /* The number of bytes to allocate to the
                                                         stack of the task. */
                      3,                              /* The priority assigned to the task. */
                      wkup_handle);                   /* The task handle. */

        OS_ASSERT(wkup_handle);
#endif



#if 1
        /* Start the sensor application task. */
        OS_TASK_CREATE("updata_task",                   /* The text name assigned to the task, for
                                                           debug only; not used by the kernel. */
                      updata_task,                      /* The function that implements the task. */
                      NULL,                             /* The parameter passed to the task. */
                      1024+1024,                        /* The number of bytes to allocate to the
                                                           stack of the task. */
                      3,                                /* The priority assigned to the task. */
                      updata_handle);                   /* The task handle. */

        OS_ASSERT(updata_handle);
#endif



#if 1
        /* Start the sensor application task. */
        OS_TASK_CREATE("rtc_task",                     /* The text name assigned to the task, for
                                                          debug only; not used by the kernel. */
                      rtc_task,                        /* The function that implements the task. */
                      NULL,                            /* The parameter passed to the task. */
                      512,                             /* The number of bytes to allocate to the
                                                          stack of the task. */
                      3,                               /* The priority assigned to the task. */
                      handle);                         /* The task handle. */

        OS_ASSERT(handle);
#endif



// pwm  motor
#if 0
          /* Start the display application task. */
        OS_TASK_CREATE("timer0_task",                  /* The text name assigned to the task, for
                                                           debug only; not used by the kernel. */
                       timer0_task,               /* The function that implements the task. */
                       NULL,                            /* The parameter passed to the task. */
                       256,                             /* The number of bytes to allocate to the
                                                           stack of the task. */
                       2,  /* 1  The priority assigned to the task. */
                       handle);                         /* The task handle. */

        OS_ASSERT(handle);
#endif

//light0 breath light
#if 0
          /* Start the display application task. */
        OS_TASK_CREATE("breath_task",                  /* The text name assigned to the task, for
                                                           debug only; not used by the kernel. */
                       breath_task,               /* The function that implements the task. */
                       NULL,                            /* The parameter passed to the task. */
                       512,                             /* The number of bytes to allocate to the
                                                           stack of the task. */
                       2,  /* 1  The priority assigned to the task. */
                       handle);                         /* The task handle. */

        OS_ASSERT(handle);
#endif

#if 0
          /* Start the display application task. */
        OS_TASK_CREATE("test_task",                  /* The text name assigned to the task, for
                                                           debug only; not used by the kernel. */
                       test_task,               /* The function that implements the task. */
                       NULL,                            /* The parameter passed to the task. */
                       512,                             /* The number of bytes to allocate to the
                                                           stack of the task. */
                       3,  /* 1  The priority assigned to the task. */
                       handle);                         /* The task handle. */

        OS_ASSERT(handle);
#endif

        /* the work of the SysInit task is done */
        OS_TASK_DELETE(OS_GET_CURRENT_TASK());

}

/**
 * @brief BLE FW demo main creates a BLE task
 */
int main( void )
{
        OS_TASK handle;
        OS_BASE_TYPE status __attribute__((unused));

        cm_clk_init_low_level();                          /* Basic clock initializations. */

        /* Start SysInit task. */
        status = OS_TASK_CREATE("SysInit",                /* The text name assigned to the task, for
                                                             debug only; not used by the kernel. */
                                system_init,              /* The System Initialization task. */
                                ( void * ) 0,             /* The parameter passed to the task. */
                                1024,                     /* The number of bytes to allocate to the
                                                             stack of the task. */
                                OS_TASK_PRIORITY_HIGHEST, /* The priority assigned to the task. */
                                handle );                 /* The task handle */
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);


        /* Start the tasks and timer running. */
        vTaskStartScheduler();

        /* If all is well, the scheduler will now be running, and the following
        line will never be reached.  If the following line does execute, then
        there was insufficient FreeRTOS heap memory available for the idle and/or
        timer tasks to be created.  See the memory management section on the
        FreeRTOS web site for more details. */
        for( ;; );
}
#if 0

static void periph_init(void)
{
#if defined CONFIG_RETARGET
        /*
         * Workaround for JLink emulated serial port.
         *
         * JLink serial port does not set its output UART pin high (UART idle state) unless
         * there is something transmitted from PC to board.
         * Pin state is kept by level shifter low after board reset.
         * Configuring pin as UART_RX does not turn on pull up resistor, hence RX line stays low
         * and this state is usually detected as break condition.
         * With low state on UART RX, configuration of UART is unsuccessful: as soon as baud rate
         * is set up UART goes into busy state, all other settings are ignored.
         *
         * Workaround sets up pin that will be used as UART RX as output with high state.
         * This will result in level shifter holding this state until JLink starts to drive this
         * line for transmission.
         */
        hw_gpio_configure_pin(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_GPIO, 1);

        hw_gpio_set_pin_function(HW_GPIO_PORT_1, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_UART2_TX);
        hw_gpio_set_pin_function(HW_GPIO_PORT_2, HW_GPIO_PIN_3, HW_GPIO_MODE_OUTPUT,
                                                                        HW_GPIO_FUNC_UART2_RX);
#endif

}
#endif

static void prvSetupHardware( void )
{
#if mainCHECK_INTERRUPT_STACK == 1
        extern unsigned long _vStackTop[], _pvHeapStart[];
        unsigned long ulInterruptStackSize;
#endif

        /* Init hardware */
        //pm_system_init(periph_init);
        pm_system_init(periph_setup);//skn wu

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

/**
 * @brief Malloc fail hook
 */
void vApplicationMallocFailedHook( void )
{
        /* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
        taskDISABLE_INTERRUPTS();
        for( ;; );
}

/**
 * @brief Application idle task hook
 */
void vApplicationIdleHook( void )
{
        /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
           to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
           task. It is essential that code added to this hook function never attempts
           to block in any way (for example, call xQueueReceive() with a block time
           specified, or call vTaskDelay()).  If the application makes use of the
           vTaskDelete() API function (as this demo application does) then it is also
           important that vApplicationIdleHook() is permitted to return to its calling
           function, because it is the responsibility of the idle task to clean up
           memory allocated by the kernel to any task that has since been deleted. */

#if (dg_configTRACK_OS_HEAP == 1)
        OS_BASE_TYPE i = 0;
        OS_BASE_TYPE uxMinimumEverFreeHeapSize;

        // Generate raw status information about each task.
        UBaseType_t uxNbOfTaskEntries = uxTaskGetSystemState(pxTaskStatusArray,
                                                        mainMAX_NB_OF_TASKS, &ulTotalRunTime);

        for (i = 0; i < uxNbOfTaskEntries; i++) {
                /* Check Free Stack*/
                OS_BASE_TYPE uxStackHighWaterMark;

                uxStackHighWaterMark = uxTaskGetStackHighWaterMark(pxTaskStatusArray[i].xHandle);
                OS_ASSERT(uxStackHighWaterMark >= mainMIN_STACK_GUARD_SIZE);
        }

        /* Check Minimum Ever Free Heap against defined guard. */
        uxMinimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
        OS_ASSERT(uxMinimumEverFreeHeapSize >= mainTOTAL_HEAP_SIZE_GUARD);
#endif /* (dg_configTRACK_OS_HEAP == 1) */

#if dg_configUSE_WDOG
        sys_watchdog_notify(idle_task_wdog_id);
#endif
}

/**
 * @brief Application stack overflow hook
 */
void vApplicationStackOverflowHook( OS_TASK pxTask, char *pcTaskName )
{
        ( void ) pcTaskName;
        ( void ) pxTask;

        /* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
        taskDISABLE_INTERRUPTS();
        for( ;; );
}

/**
 * @brief Application tick hook
 */
void vApplicationTickHook( void )
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

