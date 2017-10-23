#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <osal.h>
#include "hw_wkup.h"
#include "sys_power_mgr.h"
#include "platform_devices.h"

#include "hw_gpio.h"

#define TIMER_NOTIF (1 << 2)
#define WKUP_NOTIF  (1 << 1)
extern OS_TASK wkup_handle;
extern  void display_init(void) ;
uint8_t wkup_countes = 0;
volatile uint8_t wkup_flag = 1;
extern  uint8_t resum_flag ;
static void timer_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        OS_TASK_NOTIFY(task, TIMER_NOTIF, eSetBits);       //向sensor_task发出通知
}

static void wkup_intr_cb(void)
{
        hw_wkup_reset_interrupt();
        //printf("Wake up interrupt triggered\r\n");
        OS_TASK_NOTIFY_FROM_ISR(wkup_handle,WKUP_NOTIF,OS_NOTIFY_SET_BITS);
        //printf("Wake up interrupt triggered  end\r\n");
}

volatile uint8_t dis_flag = 2;

static void key_intr_cb(void)
{
        hw_wkup_reset_interrupt();
        wkup_countes = 0;
        dis_flag++;
        if(dis_flag == 5)
            dis_flag = 1;
        //printf("key interrupt triggered\r\n");
}
void wkup_init(void)
{
        hw_wkup_init(NULL);
        hw_wkup_set_counter_threshold(1);
        //hw_wkup_set_debounce_time(32);
        hw_wkup_set_debounce_time(60);
#if 1

        hw_gpio_set_pin_function(HW_GPIO_PORT_3 , HW_GPIO_PIN_3, HW_GPIO_MODE_INPUT_PULLUP,HW_GPIO_FUNC_GPIO);
        //hw_gpio_set_pin_function(HW_GPIO_PORT_4 , HW_GPIO_PIN_4, HW_GPIO_MODE_INPUT_PULLUP,HW_GPIO_FUNC_GPIO);
        //hw_gpio_set_pin_function(HW_GPIO_PORT_4 , HW_GPIO_PIN_4, HW_GPIO_MODE_INPUT_PULLDOWN,HW_GPIO_FUNC_GPIO);

        //hw_gpio_configure_pin(HW_GPIO_PORT_4, HW_GPIO_PIN_4, HW_GPIO_MODE_INPUT_PULLDOWN,HW_GPIO_FUNC_GPIO, true);

        // ...then enable counting on the specific GPIO
        hw_wkup_set_pin_state(HW_GPIO_PORT_3, HW_GPIO_PIN_3, true);
        // first set up the proper polarity...
        hw_wkup_set_pin_trigger(HW_GPIO_PORT_3, HW_GPIO_PIN_3, HW_WKUP_PIN_STATE_LOW);
        //hw_wkup_set_pin_trigger(HW_GPIO_PORT_4, HW_GPIO_PIN_4, HW_WKUP_PIN_STATE_LOW);

#else
        // first set up the proper polarity...
        hw_gpio_configure_pin(HW_GPIO_PORT_1, HW_GPIO_PIN_6, HW_GPIO_MODE_INPUT,
                HW_GPIO_FUNC_GPIO, false);

        hw_wkup_set_pin_trigger(HW_GPIO_PORT_1, HW_GPIO_PIN_6, HW_WKUP_PIN_STATE_HIGH);
        // ...then enable counting on the specific GPIO
        hw_wkup_set_pin_state(HW_GPIO_PORT_1, HW_GPIO_PIN_6, true);
#endif
       // hw_wkup_register_interrupt(wkup_intr_cb, 1);
        hw_wkup_register_interrupt(key_intr_cb, 1);
}


extern uint8_t sleep_flag;
extern OS_TIMER *pps960_timer;
//extern OS_TIMER *update_timer;
OS_TIMER *update_timer = NULL;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
extern sleep_mode_t pm_current_sleep_mode;
extern sleep_mode_t pm_user_sleep_mode;
extern uint8_t pm_mode ;
void wkup_task(void *params)
{
#if 1
    PRIVILEGED_DATA static OS_TIMER *polling_timer;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
    polling_timer = OS_TIMER_CREATE("Poll_timer",
                                    OS_MS_2_TICKS(1000),// 1 seconds.
                                    OS_TIMER_SUCCESS,
                                    (void *) OS_GET_CURRENT_TASK(),
                                    timer_cb);
    OS_ASSERT(polling_timer);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行


    wkup_init();//reg key

    uint32_t notif;
    OS_BASE_TYPE ret;
    OS_TIMER_START(polling_timer, 1000 / OS_PERIOD_MS);

    for (;;)
    {
        //printf("sleep_flag = %d\r\n",sleep_flag);
        ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
        OS_ASSERT(ret == OS_OK);

                /* Notified from polling_timer */
                if (notif & TIMER_NOTIF)
                {
                       // printf("timer task\r\n");
                        wkup_countes++;
                       //if (wkup_countes >5 && wkup_flag == 1 && resum_flag)
                       if (wkup_countes >5 && wkup_flag == 1 )
                       {


                               hw_wkup_unregister_interrupt();
                               hw_wkup_register_interrupt(wkup_intr_cb, 1);

                               OS_TIMER_STOP(update_timer, OS_TIMER_FOREVER);//display

                               OS_TIMER_STOP(pps960_timer, OS_TIMER_FOREVER);//hr

                                       ////////OS_DELAY_MS(100);
                               wkup_flag = 0;//close display refresh
                               dis_flag = 1 ;//return default display windows
                               //pm_set_sleep_mode(pm_mode_extended_sleep);
                               //sleep_mode_t  sleepmode = pm_get_sleep_mode( );
                               //printf("sleepmode = %d\r\n",(uint8_t)sleepmode);
                               //printf("pm_current_sleep_mode = %d\r\n",(uint8_t)pm_current_sleep_mode);
                               //printf("pm_user_sleep_mode = %d\r\n",(uint8_t)pm_user_sleep_mode);
                               if(pm_mode == 1)
                               {
                                       #ifdef DEMOBOARD
                                       hw_gpio_set_inactive(HW_GPIO_PORT_4, HW_GPIO_PIN_0);//hr
                                       #else
                                       hw_gpio_set_inactive(HW_GPIO_PORT_3, HW_GPIO_PIN_5);//hr
                                       #endif
                                       hw_gpio_set_inactive(HW_GPIO_PORT_1, HW_GPIO_PIN_2);//display
                                       //printf("pm_mode if\r\n");
                               }
                               else//(pm_mode == 0)
                               {
                                      // pm_set_sleep_mode(pm_mode_extended_sleep);
                               }
                               //uint8_t data = 3;
                               //ad_flash_read(0x100000, &data, 1);
                               //printf("data = %d\r\n",data);
                               //ad_flash_read(0x100001, &data, 1);
                               //printf("2data = %d\r\n",data);

                               pm_resume_sleep();

                               //printf("2pm_current_sleep_mode = %d\r\n",(uint8_t)pm_current_sleep_mode);
                               //printf("2pm_user_sleep_mode = %d\r\n",(uint8_t)pm_user_sleep_mode);

                               //printf("resume_sleep task\r\n");
                       }

                }//if(notif & TIMER_NOTIF)


                if (notif & WKUP_NOTIF)
                {
                       //printf("wkup task\r\n");

                       //pm_set_sleep_mode(pm_mode_active);
                       if(wkup_flag == 0)
                       {
                               pm_stay_alive();

                               wkup_countes = 0;


//                               display_init();
                               OS_DELAY_MS(10);

                               wkup_flag = 1;

                               OS_TIMER_RESET(update_timer, OS_TIMER_FOREVER);//display

                                   /////init_pps960_sensor();
                               OS_TIMER_RESET(pps960_timer, OS_TIMER_FOREVER);//hr

                               hw_wkup_unregister_interrupt();
                               //key
//                               hw_wkup_register_interrupt(key_intr_cb,1);
                       }

                }//if (notif & WKUP_NOTIF)
    }//for(;;)
#endif
}


//vTaskDelayUntil( &qcy_display_time, OS_MS_2_TICKS(100) );
//qcy_display_time = OS_GET_TICK_COUNT();
//OS_TICK_TIME qcy_display_time;






