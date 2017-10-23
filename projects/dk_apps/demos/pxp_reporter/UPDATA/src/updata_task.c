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
#include <updata.h>
#include <bas.h>//add
#include "mc36xx_sample.h"//add
//#include <oled_drv.h>
//extern Display_Type  Displaymsg;
//#include <skynoon.h>

#include "time_to_s.h"

#define UPDATA_TIMER_3_NOTIF (1 << 1)

//OS_TASK_NOTIFY(task, UPDATA_TIMER_3_NOTIF, eSetBits);       //向sensor_task发出通知


#define BLE_UPDATA_TIMER_NOTIF (1 << 2)
extern uint8_t time_data[10] ;
//OS_EVENT sensor_evt;

uint8_t g_data_12_munites[4*8]={0};//12minutes
uint8_t times = 0 ;
uint16_t one_3_day_index = 0;
uint16_t one_12_day_index = 0;


uint32 g_data_step = 0;

uint8_t erase_flag   = 0;

uint8_t  save_12_number = 0;//120

//uint8_t section_days = 1;
uint8_t section_days = 2;  //test


void updata_task(void *params)
{

    //OS_EVENT sensor_evt;
    //OS_EVENT_CREATE(sensor_evt);//xSemaphoreCreateBinary()  1
    //OS_ASSERT(sensor_evt);

    OS_BASE_TYPE ret;
    uint32_t notif;

    memset(g_data_12_munites,0,4*8);
    uint8_t ret_index = 0;
    SysTime_t current_time ={0};

    for (;;) {
        /*
         * Wait on polling_timer expired of the notification bits, then clear them all
         */
        printf("into_OS_TASK_NOTIFY_WAIT\r%d\n",__LINE__);//add

        ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);

        printf("finish_OS_TASK_NOTIFY_WAIT\r%d\n",__LINE__);//add
        OS_ASSERT(ret == OS_OK);

        /* Notified from polling_timer */
        if (notif & UPDATA_TIMER_3_NOTIF) //  3 minut
        {
                //OS_EVENT_WAIT(sensor_evt,1000 / OS_PERIOD_MS);
                printf("UPDATA_TIMER_3_NOTIF \r%d\n",__LINE__);

                if (erase_flag == 0 )
                {
                        erase_flag  = 1 ;
                        save_12_number = 0;
                        erase_flash_section(section_days);
                        write_month_day(section_days,Displaymsg.dyear-2000, Displaymsg.dmouth,Displaymsg.dday);
                }
                current_time.year  =  Displaymsg.dyear;
                current_time.month =  Displaymsg.dmouth;
                current_time.day   =  Displaymsg.dday;
                current_time.hour  =  Displaymsg.dhour;
                current_time.min   =  Displaymsg.dmin;
                current_time.sec   =  Displaymsg.dsec;
                current_time.milliseconds =  0;
                one_3_day_index = (current_time.hour*60 + current_time.min)/3 ; // 1 479(57) 0
                //(23*60 +57)/3 = 479   ...479.xx....    (23*60 +60)/3 =480   (0*60 + 0) /3 = 0
                one_12_day_index = (current_time.hour*60 + current_time.min)/12 ; //1 119(48) 0
                //(23*60 +48)/12 = 119   ...119.xx....    (23*60 +60)/12 =120   (0*60 + 0) /12 = 0
                g_data_step = cur_total_step ;
                // times  0  1  2  3   0  1   2   3
                ret_index = three_minuts_save_one_data(current_time,times,g_data_12_munites,g_data_step);//GetTodaySteps(1)
                times++;        //  1 2 3 4  1 2 3 4
                printf("11111111111111 times = %d\r\n",times);
                if(times == 4)
                {
                        times = 0;
                        printf("444444444444444444444444 \r\n");
                        //save_12_number   0 1 2 3 4 5 6 119
                        write_data_to_flash(section_days,g_data_12_munites,save_12_number);

                        save_12_number++ ; // 1 2 3 4 5 6  .... 120
                        printf("11111111111111 save_12_number = %d\r\n",save_12_number);
                        memset(g_data_12_munites,0,4*8);
                }
                //if(one_3_day_index == 0 && one_12_day_index == 0 && save_12_number > 0)//test
                if(one_3_day_index == 0 && one_12_day_index == 0 )//test
                //if(one_3_day_index ==0 && save_12_number == 120)//test
                {
                        //write_month_day(section_days,Displaymsg.dyear-2000, Displaymsg.dmouth,Displaymsg.dday);
                        mark_the_section_has_write(section_days);
                        write_to_flash_number(section_days,save_12_number);
                        erase_flag = 0;
                        times = 0 ;
                        save_12_number = 0 ;
                        //section_days  1 2 3 4  5 6 .... 15
                        section_days++;// 2 3 4  5 6 7 .... 16
                        if(section_days == 16)
                        {
                                //section_days = 1;
                                section_days = 2;//test
                        }
                        printf("jjjjjjjjjjjjjjjjjjjjjjjjjjjjj\r\n");
                }
               //OS_EVENT_SIGNAL(sensor_evt);
        }//if(notif & UPDATA_TIMER_NOTIF) //  3 minut

        if (notif & BLE_UPDATA_TIMER_NOTIF) {
               //OS_EVENT_WAIT(sensor_evt,1000 / OS_PERIOD_MS);
                #if 1
                        printf("BLE_UPDATA_TIMER_NOTIF2222\r\n");
                        //printf("time_data0 =%d\r\n",time_data[0]);
                        //printf("time_data0 =%d\r\n",time_data[1]);
                        Displaymsg.dyear = 2000+time_data[2];
                        Displaymsg.dmouth = time_data[3];
                        Displaymsg.dday  = time_data[4];
                        Displaymsg.dweek = time_data[5];
                        Displaymsg.dhour = time_data[6];
                        Displaymsg.dmin  = time_data[7];
                        Displaymsg.dsec  = time_data[8];
              //fflush(stdout);
                #endif
              //OS_EVENT_SIGNAL(sensor_evt);
        }//if(notif & BLE_UPDATA_TIMER_NOTIF)
    }//for(;;)
}
