/**
 ****************************************************************************************
 *
 * @file sensor_task.c
 *
 * @brief sensor drivers implementation
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <osal.h>
#include <platform_devices.h>
#include <timer0.h>

#define PWM_TIMER_NOTIF (1 << 1)

//OS_EVENT sensor_evt;


/* Poll timer callback */
static void poll_timer_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        OS_TASK_NOTIFY(task, PWM_TIMER_NOTIF, eSetBits);       //向sensor_task发出通知
}

void timer0_task(void *params)
{

   // OS_EVENT sensor_evt;
    //OS_EVENT_CREATE(sensor_evt);//xSemaphoreCreateBinary()  1
    //OS_ASSERT(sensor_evt);

    /*
    * Create timer for HR-sensor + g-sensor which will be used to send measurement every 1 second
    */
    PRIVILEGED_DATA static OS_TIMER *pwm_timer;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
    pwm_timer = OS_TIMER_CREATE("Poll_timer", 500 / OS_PERIOD_MS, OS_TIMER_SUCCESS,
                                            (void *) OS_GET_CURRENT_TASK(), poll_timer_cb);
    OS_ASSERT(pwm_timer);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行


    // kx022_i2c_init();


    OS_TIMER_START(pwm_timer, 1000 / OS_PERIOD_MS);

    OS_BASE_TYPE ret;
    uint32_t notif;

    timer0_slow_blink_func();

    for (;;) {
           // printf("start x y z \r");
        /*
         * Wait on polling_timer expired of the notification bits, then clear them all
         */
        ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
        OS_ASSERT(ret == OS_OK);

        /* Notified from polling_timer */
        if (notif & PWM_TIMER_NOTIF) {

               //OS_EVENT_WAIT(sensor_evt,1000 / OS_PERIOD_MS);
#if 1

              // printf("pwm\r\n");
              //fflush(stdout);
#endif

               //OS_EVENT_SIGNAL(sensor_evt);
        }//if
    }//for(;;)
}











