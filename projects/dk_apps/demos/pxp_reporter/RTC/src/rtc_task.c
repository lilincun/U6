/*
 * rtc_task.c
 *
 *  Created on: 2017年3月6日
 *      Author: Administrator
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <osal.h>
#include <platform_devices.h>
//#include "oled_drv.h"

#define RTC_NOTIF (1 << 1)

//OS_EVENT sensor_evt;


/* Poll timer callback */
static void rtc_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        OS_TASK_NOTIFY(task, RTC_NOTIF, eSetBits);       //向sensor_task发出通知
}

void rtc_task(void *params)
{

   // OS_EVENT sensor_evt;
    //OS_EVENT_CREATE(sensor_evt);//xSemaphoreCreateBinary()  1
    //OS_ASSERT(sensor_evt);

    /*
    * Create timer for HR-sensor + g-sensor which will be used to send measurement every 1 second
    */
    PRIVILEGED_DATA static OS_TIMER *rtc_timer;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
    rtc_timer = OS_TIMER_CREATE("Poll_timer", OS_MS_2_TICKS(1000), OS_TIMER_SUCCESS,
                                            (void *) OS_GET_CURRENT_TASK(), rtc_cb);
    OS_ASSERT(rtc_timer);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行


    // kx022_i2c_init();


    OS_TIMER_START(rtc_timer, OS_TIMER_FOREVER);

    OS_BASE_TYPE ret;
    uint32_t notif;


    for (;;) {
           // printf("start x y z \r");
        /*
         * Wait on polling_timer expired of the notification bits, then clear them all
         */
        ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
        OS_ASSERT(ret == OS_OK);

        /* Notified from polling_timer */
        if (notif & RTC_NOTIF)
        {

               //OS_EVENT_WAIT(sensor_evt,1000 / OS_PERIOD_MS);
#if 1
              //app_calendar_timer_handler();//week
               //printf("pwm111111111111111111111\r\n");
              //fflush(stdout);
#endif

               //OS_EVENT_SIGNAL(sensor_evt);
        }//if
    }//for(;;)
}
