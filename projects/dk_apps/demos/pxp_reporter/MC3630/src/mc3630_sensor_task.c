/*
 * mc3630_sensor_task.c
 *
 *  Created on: 2017年8月31日
 *      Author: cong
 *     objects.mk   -lmCube_Pedo_M0
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <osal.h>

#include <userconfig.h>
#include <platform_devices.h>
#include <hw_i2c.h>
#include "mc36xx_sample.h"



#define POLL_TIMER_NOTIF (1 << 1)
uint8_t record_buf[255];
uint8_t axis[3][90];
extern OS_TASK alg_handle;
extern OS_TASK pps960_task2;//add
OS_EVENT sensor_evt;

/* Poll timer callback */
static void poll_timer_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        OS_TASK_NOTIFY(task, POLL_TIMER_NOTIF, eSetBits);       //向sensor_task发出通知

       // OS_TASK_NOTIFY(alg_handle, POLL_TIMER_NOTIF, eSetBits); //向algorithem_task发出通知
}
extern  unsigned char mcube_read_regs(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes);
extern  unsigned char mcube_write_regs(unsigned char register_address, unsigned char *value, unsigned char number_of_bytes);


void mc3630_sensor_task(void *params)
{



        // OS_EVENT sensor_evt;
         OS_EVENT_CREATE(sensor_evt);//xSemaphoreCreateBinary()  1
         OS_ASSERT(sensor_evt);

         /*
         * Create timer for HR-sensor + g-sensor which will be used to send measurement every 1 second
         */
         PRIVILEGED_DATA static OS_TIMER *polling_timer;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
         polling_timer = OS_TIMER_CREATE("Poll_timer", 500 / OS_PERIOD_MS, OS_TIMER_SUCCESS,
                                                 (void *) OS_GET_CURRENT_TASK(), poll_timer_cb);
         OS_ASSERT(polling_timer);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行

         gsensor_init();

         OS_TIMER_START(polling_timer, 1000 / OS_PERIOD_MS);

         OS_BASE_TYPE ret;
         uint32_t notif;
         uint8_t data_lenght;
         uint8_t xyz_counts;

         for (;;) {
             /*
              * Wait on polling_timer expired of the notification bits, then clear them all
              */
             ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
             OS_ASSERT(ret == OS_OK);


             /* Notified from polling_timer */
             if (notif & POLL_TIMER_NOTIF) {
                     OS_EVENT_WAIT(sensor_evt,500 / OS_PERIOD_MS);

        #if 0
                    unsigned char data = 0x00;
                   // mcube_write_regs(0x1A, &data, 1);

                    mcube_read_regs(0x18, &data, 1);// 71
                    printf("data1 = %u \r",data);
                    OS_DELAY_MS(1000);
                    mcube_read_regs(0x18, &data, 1);// 71
                    //mcube_read_regs(0x01, &data, 1); //04
                    printf("data2 = %u \r",data);
                    fflush(stdout);
        #endif

                    mcube_fifo_timer_handle();
                 //给PPS960task2发送任务通知

                   OS_TASK_NOTIFY_GIVE(pps960_task2);//add


                    OS_EVENT_SIGNAL(sensor_evt);
             }


         }//end for(;;)
}













