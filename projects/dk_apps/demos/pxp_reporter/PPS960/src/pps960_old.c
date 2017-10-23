#if 0

/*
 * pps960.c
 *
 *  Created on: 2016年9月8日
 *      Author: cole
 */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include <hw_wkup.h>
#include <osal.h>

#include <platform_devices.h>
#include <hw_i2c.h>

#include "afe4404_hw.h"
#include "agc_V3_1_19.h"
#include "hqerror.h"
#include "pps960.h"


//#include "ad_uart.h"

#define Wkup_PORT HW_GPIO_PORT_1
#define Wkup_PIN HW_GPIO_PIN_4

//extern void ALGSH_retrieveSamplesAndPushToQueue(void);
// extern afe_data_struct_t afe_struct1;
//afe_32_bit_data_struct_t afe_32bit_struct1;
//afe_32_RAW_bit_data_struct_t afe_32bit_RAW_struct1;
// uint64_t settingsCheckSampleQueue[2];
// uint16_t HRM_sample_count;

uint16_t acc_check=0;

//uint16_t pps_x;
//uint16_t pps_y;
//uint16_t pps_z;

 uint32_t displayHrm = 0;
 uint32_t pps_wkup_count;
 uint32_t wkup_intr_flag=0;
 //int8_t accPushToQueueFlag=0;

void init_pps960_sensor(void)
{
        uint32_t ctr_temp=0;
        //PPS960 PIN CONFIG
        //hw_gpio_set_pin_function(Wkup_PORT,Wkup_PIN,HW_GPIO_MODE_INPUT,HW_GPIO_FUNC_GPIO);//ADC_RDY
        //hw_gpio_set_pin_function(HW_GPIO_PORT_3,HW_GPIO_PIN_5,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO);//REST
        //hw_gpio_set_pin_function(HW_GPIO_PORT_4,HW_GPIO_PIN_0,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO);//TX_SUP Enable
        hw_gpio_configure_pin(HW_GPIO_PORT_3,HW_GPIO_PIN_5,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,1);
        hw_gpio_configure_pin(HW_GPIO_PORT_4,HW_GPIO_PIN_0,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,1);
        //hw_gpio_set_active(HW_GPIO_PORT_3, HW_GPIO_PIN_2);//1
        //hw_gpio_set_active(HW_GPIO_PORT_4, HW_GPIO_PIN_0);//1

        //hw_gpio_configure_pin(HW_GPIO_PORT_3,HW_GPIO_PIN_2,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,1);

        //OS_DELAY(100);//100=200MS
        //hw_gpio_set_inactive(HW_GPIO_PORT_3, HW_GPIO_PIN_2);//0
        //OS_DELAY(100);
        //hw_gpio_set_active(HW_GPIO_PORT_3, HW_GPIO_PIN_2);//1
        PPS960_writeReg(0,0x8);
        //OS_DELAY(50);
        init_PPS960_register();
        PPS960_init();
}

/* ADC_RDY interrupt callback */
static void pps960_wkup_intr_cb(void)
{
        //OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        //OS_TASK_NOTIFY(task, POLL_TIMER_NOTIF, eSetBits);       //向sensor_task发出通知
        /*
         * Interrupt handler should always reset interrupt state, otherwise it will be called again.
         */
        static uint32_t tmpval;
        hw_wkup_reset_interrupt();


        //tmpval = PPS960_readReg(44);

        wkup_intr_flag++;

        //agc_check = 1;
        pps_wkup_count++;
        //printf("triggered\n");
}
void pps960_wkup_init(void)
{

        hw_wkup_init(NULL);
        hw_gpio_set_pin_function(Wkup_PORT,Wkup_PIN,HW_GPIO_MODE_INPUT,HW_GPIO_FUNC_GPIO);//ADC_RDY
        //init_pps960_sensor();
        /*
         * Default value for counter threshold is 0. It's important to change this value before
         * interrupt is registered as otherwise interrupt will be triggered indefinitely due to
         * counter value being equal to threshold.
         */
        hw_wkup_set_counter_threshold(1);
        hw_wkup_register_interrupt(pps960_wkup_intr_cb, 1);

        /*
         * It's also possible to configure pin state and trigger at once in a single call, see
         * hw_wkup_configure_pin() API for this.
         */
         hw_wkup_set_pin_state(Wkup_PORT,Wkup_PIN,1 );//


         /*
         * Note that counter is edge-sensitive. After triggering edge (i.e. pin goes to state which
         * is set as trigger) is detected, a reverse edge must be detected before timer goes back
         * to idle state and can detect and count another event.
         */
         hw_wkup_set_pin_trigger(Wkup_PORT,Wkup_PIN, HW_WKUP_PIN_STATE_HIGH);//state = HW_WKUP_PIN_STATE_LOW : HW_WKUP_PIN_STATE_HIGH;

}


#define PPS960_POLL_TIMER_NOTIFY (1 << 1)

/* Poll timer callback */
static void pps960_timer_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        OS_TASK_NOTIFY(task, PPS960_POLL_TIMER_NOTIFY, eSetBits);       //向sensor_task发出通知
        //printf("pps_timer_ccb \r\n");
}

/* Poll timer callback */
static void pps960_timer2_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);
        OS_TASK_NOTIFY(task, PPS960_POLL_TIMER_NOTIFY, eSetBits);       //向sensor_task发出通知
}

OS_TIMER *pps960_timer = NULL;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
extern uint8_t hr_flag ;
volatile uint8_t hr_init_flag = 0 ;
extern uint8_t wkup_countes ;
void pps960_sensor_task(void *params)
{
        //static int agc_check = 0;
        static int lifeQhrm=0;
        uint32_t ret,notify;
        uint32_t pps_count;
        //static int HRM_sample_count=0;

        //init pps960
        //init_pps960_sensor();

        /*
        * Create timer for HR-sensor + g-sensor which will be used to send measurement every 1 second
        */
        //PRIVILEGED_DATA static OS_TIMER *pps960_timer;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
        //pps960_timer = OS_TIMER_CREATE("pps960_timer", 20 / OS_PERIOD_MS, OS_TIMER_SUCCESS,
        //                                        (void *) OS_GET_CURRENT_TASK(), pps960_timer_cb);

        pps960_timer = OS_TIMER_CREATE("pps960_timer", 20 / OS_PERIOD_MS, OS_TIMER_SUCCESS,
                                                (void *) OS_GET_CURRENT_TASK(), pps960_timer_cb);

        OS_ASSERT(pps960_timer);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行

        //pps960_wkup_init(); //interrupter

        //acc_check = 1;
        //printf("timer1\n");
        OS_TIMER_START(pps960_timer, 40 / OS_PERIOD_MS);
        //OS_TIMER_START(pps960_timer1, 500 / OS_PERIOD_MS);
        //printf("timer2\n");

    for (;;) {
            /*
             * Wait on polling_timer expired of the notification bits, then clear them all
             */
            //printf("timer3\n");
#if 0
            if(wkup_intr_flag>0)
            {
                    wkup_intr_flag = 0;
                    mcube_fifo_timer_handle();//read Gsensor xyz
                    ALGSH_retrieveSamplesAndPushToQueue();//read pps adc

                    ALGSH_dataToAlg();//
                    if(GetHRSampleCount()==25){
                            ClrHRSampleCount();
                            pps_count=pps_wkup_count;
                            pps_wkup_count = 0;

                            //afe_struct1.LifeQhrm = pps_getHR();
                            lifeQhrm = pps_getHR();
                            printf("hrm = %d pps_wkup_count=%d\r\n",lifeQhrm,pps_count);

                            displayHrm = lifeQhrm;//afe_struct1.LifeQhrm;

                            //mcube_fifo_timer_handle();

                    }

         }else{
                ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notify, OS_TASK_NOTIFY_FOREVER);
                OS_ASSERT(ret == OS_OK);

                if(notify & PPS960_POLL_TIMER_NOTIFY){

                }

        }
#else
            ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notify, OS_TASK_NOTIFY_FOREVER);
            OS_ASSERT(ret == OS_OK);

            if(notify & PPS960_POLL_TIMER_NOTIFY){
                    if( hr_flag == 2) //open hr
                    {
                           if (hr_init_flag == 0) //first open pps init
                           {
                             init_pps960_sensor();
                             hr_init_flag = 1;   // second close pps init
                           }
                            wkup_countes = 0;
                            //wkup_intr_flag = 0;
                            //mcube_fifo_timer_handle();//read Gsensor xyz
                            ALGSH_retrieveSamplesAndPushToQueue();//read pps adc
                            ALGSH_dataForAgc();//ppg data 自动增益
                    }
                    else
                    //if( hr_flag == 1) //colse hr
                    {
                            if (hr_init_flag == 1) // second  close pps power
                            {
                                    hw_gpio_set_inactive(HW_GPIO_PORT_4, HW_GPIO_PIN_0);
                                    hr_init_flag = 0;
                            }
                            //if (hr_init_flag == 0)// first  not close  pps power
                            else
                            {

                            }
                    }

#if 0
                    ALGSH_dataToAlg();//
                    if(GetHRSampleCount()==25){
                            ClrHRSampleCount();
                            pps_count=pps_wkup_count;
                            pps_wkup_count = 0;

                            //afe_struct1.LifeQhrm = pps_getHR();
                            lifeQhrm = pps_getHR();
                            printf("hrm = %d pps_wkup_count=%d\r\n",lifeQhrm,pps_count);

                            displayHrm = lifeQhrm;//afe_struct1.LifeQhrm;

                            //mcube_fifo_timer_handle();
                    }
#endif
         }// if(notify & PPS960_POLL_TIMER_NOTIFY){
#endif
//            PPS960_writeReg(0x0,0x0);
//            PPS960_writeReg(0x23,0x124218);
//            PPS960_writeReg(0x0,0x1);
            //displayHrm = PPS960_readReg(0x23);
//            //OS_DELAY(10);//for 50Hz

    }//for(;;)
}


void pps960_sensor_task2(void *params)
{

        static int lifeQhrm=0;
        uint32_t ret,notify;
        uint32_t pps_count;
        uint32_t sample_count;

        //PRIVILEGED_DATA static OS_TIMER *pps960_timer2;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
        //pps960_timer2 = OS_TIMER_CREATE("pps960_timer2", 20 / OS_PERIOD_MS, OS_TIMER_SUCCESS,
        //                                        (void *) OS_GET_CURRENT_TASK(), pps960_timer2_cb);
        //OS_ASSERT(pps960_timer2);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行

        //pps960_wkup_init();

        acc_check = 1;
        //printf("timer4\n");
        //OS_TIMER_START(pps960_timer2, 20 / OS_PERIOD_MS);

    for (;;) {
            //printf("timer5\n");
            /*
             * Wait on polling_timer expired of the notification bits, then clear them all
             */

            //ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notify, OS_TASK_NOTIFY_FOREVER);
            //OS_ASSERT(ret == OS_OK);

            if(acc_check){
                    //if(accPushToQueueFlag)//1s
                    //{
                            //OS_TASK_NOTIFY_GIVE_FROM_ISR(g_kx022_handle);
                            OS_TASK_NOTIFY_TAKE(0, OS_EVENT_FOREVER);

                            cm_sys_clk_set(sysclk_PLL48);
                            uint16_t ledBuffLength = GetLEDBuffLenth();//pps 多少组数据

                            for(int i=0;i<ledBuffLength;i++)
                            {
                                    ALGSH_dataToAlg();//添加 pps数据 到 alg
                            }


                            pps_count=pps_wkup_count;
                            pps_wkup_count = 0;

                            //accPushToQueueFlag = 0;

                            sample_count = GetHRSampleCount();//pps sample
                            ClrHRSampleCount();//claer sample

                            lifeQhrm = pps_getHR();

                            displayHrm = lifeQhrm;//afe_struct1.LifeQhrm;
                            cm_sys_clk_set(sysclk_XTAL16M);

                            printf("hrm = %d w_count=%d s_count=%d len=%d\r\n",lifeQhrm,pps_count,sample_count,ledBuffLength);
                   // }
            }//if(acc_check)
            //OS_DELAY(5);
    }//for(;;)
}

#if 0
void PPS960_writeReg(uint8_t regaddr,uint32_t wdata)
{//24bit data

        uint8_t temp[4],i;
        size_t wr_status = 0;
        uint32_t wd=wdata;

        i2c_device_config *dev = (i2c_device_config *) PPS960;
        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(dev);

        ad_i2c_device_acquire(dev);
        ad_i2c_bus_acquire(dev);

        temp[0]=regaddr;
        temp[1]=(wd>>16) & 0xff;
        temp[2]=(wd>>8) & 0xff;
        temp[3]=wd & 0xff;


        HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;


        //The first writing byte informs to which register rest data will be written.

        hw_i2c_write_byte(id, regaddr);
        wr_status = hw_i2c_write_buffer_sync(id, &temp[1], 3, &abrt_src, HW_I2C_F_WAIT_FOR_STOP);
        if ((wr_status < (ssize_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
                printf("pps960 write failure: %u" NEWLINE, abrt_src);
        }

        hw_i2c_reset_int_tx_abort(id);                //复位中止源寄存器

         ad_i2c_bus_release(dev);
}


uint32_t PPS960_readReg(uint8_t regaddr)
{
        //24bit data

        uint8_t temp[4];
        uint32_t rdtemp;size_t rd_status = 0;

        i2c_device_config *dev = (i2c_device_config *) PPS960;
        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(dev);
        //printf("PPS960_readReg");
        ad_i2c_device_acquire(dev);
        ad_i2c_bus_acquire(dev);
        //printf("PPS960_readReg1");
        HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;

        /*
        * Before reading values from sensor registers we need to send one byte information to it
        * to inform which sensor register will be read now.
        */
        hw_i2c_write_byte(id, regaddr);
        rd_status = hw_i2c_read_buffer_sync(id, temp, 3, &abrt_src, HW_I2C_F_NONE);
        if ((rd_status < (size_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
               printf("pps960 read failure: %u" NEWLINE, abrt_src);
        }
        //printf("PPS960_readReg2");

        hw_i2c_reset_int_tx_abort(id);                //复位中止源寄存器
        ad_i2c_bus_release(dev);
        //printf("PPS960_readReg3");
        rdtemp = temp[0]<<16 | temp[1]<<8 | temp[2];
        return rdtemp;
}
#endif

void PPS960_writeReg(uint8_t regaddr,uint32_t wdata)
{//24bit data

        uint8_t temp[4],i;
        size_t wr_status = 0;
        uint32_t wd=wdata;

        temp[0]=regaddr;
        temp[1]=(wd>>16) & 0xff;
        temp[2]=(wd>>8) & 0xff;
        temp[3]=wd & 0xff;
        HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;

        //i2c_device_config *dev = (i2c_device_config *) PPS960;
        i2c_device dev = ad_i2c_open(PPS960);
        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(dev);

        ad_i2c_device_acquire(dev);
        ad_i2c_bus_acquire(dev);

        vPortEnterCritical();

        //The first writing byte informs to which register rest data will be written.

        hw_i2c_write_byte(id, regaddr);
        wr_status = hw_i2c_write_buffer_sync(id, &temp[1], 3, &abrt_src, HW_I2C_F_WAIT_FOR_STOP);

        vPortExitCritical();

        if ((wr_status < (ssize_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
                printf("pps960 write failure: %u" NEWLINE, abrt_src);
        }

        //hw_i2c_reset_int_tx_abort(id);                //复位中止源寄存器

         ad_i2c_bus_release(dev);
         ad_i2c_device_release(dev);
         ad_i2c_close(dev);
}


uint32_t PPS960_readReg(uint8_t regaddr)
{
        //24bit data

        uint8_t temp[4];
        uint32_t rdtemp;size_t rd_status = 0;
        HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;


       // i2c_device_config *dev = (i2c_device_config *) PPS960;
        i2c_device dev = ad_i2c_open(PPS960);
        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(dev);
        //printf("PPS960_readReg");
        ad_i2c_device_acquire(dev);
        ad_i2c_bus_acquire(dev);
        //printf("PPS960_readReg1");

        /*
        * Before reading values from sensor registers we need to send one byte information to it
        * to inform which sensor register will be read now.
        */

        vPortEnterCritical();

        hw_i2c_write_byte(id, regaddr);
        rd_status = hw_i2c_read_buffer_sync(id, temp, 3, &abrt_src, HW_I2C_F_NONE);

        vPortExitCritical();

        if ((rd_status < (size_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
               printf("pps960 read failure: %u" NEWLINE, abrt_src);
        }
        //printf("PPS960_readReg2");

       // hw_i2c_reset_int_tx_abort(id);                //复位中止源寄存器
        ad_i2c_bus_release(dev);
        ad_i2c_device_release(dev);
        ad_i2c_close(dev);
        //printf("PPS960_readReg3");
        rdtemp = temp[0]<<16 | temp[1]<<8 | temp[2];
        //printf("rdtemp : %d\r\n ",rdtemp);
       // fflush(stdout);
        return rdtemp;
}



#endif

