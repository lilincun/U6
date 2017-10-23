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



#define PPS_Wkup_PORT HW_GPIO_PORT_1
#define PPS_Wkup_PIN HW_GPIO_PIN_4

#ifndef DEMOBOARD
//#define DEMOBOARD
#endif

//PPS960 RESET PIN define
#ifdef DEMOBOARD
#define PPS_REST_PORT HW_GPIO_PORT_3
#define PPS_REST_PIN HW_GPIO_PIN_5
#else
#define PPS_REST_PORT HW_GPIO_PORT_1
#define PPS_REST_PIN HW_GPIO_PIN_7
#endif

//PPS960 TX_SUP Control PIN
#ifdef DEMOBOARD
#define PPS_TX_SUP_PORT HW_GPIO_PORT_4
#define PPS_TX_SUP_PIN HW_GPIO_PIN_0
#else
#define PPS_TX_SUP_PORT HW_GPIO_PORT_3
#define PPS_TX_SUP_PIN HW_GPIO_PIN_5
#endif

//PPS960 RX_SUP Control PIN
//#define PPS_RX_SUP_PORT HW_GPIO_PORT_4
//#define PPS_RX_SUP_PIN HW_GPIO_PIN_0

uint16_t acc_check=0;
uint16_t acc_check2=0;

int8_t hr_okflag=false;
int8_t Stablecnt=0;
int8_t Unstablecnt=0;

int8_t HR_HRV_enable=0;//0=>HR;1=>HRV;2=>HR+HRV;

 uint32_t displayHrm = 0;
 uint32_t pps_wkup_count;
 uint32_t wkup_intr_flag=0;
 int8_t accPushToQueueFlag=0;
 extern uint16_t AccBuffTail;
void pps960_Init_gpio(void)
{
        //PPS960 PIN CONFIG
        //hw_gpio_set_pin_function(Wkup_PORT,Wkup_PIN,HW_GPIO_MODE_INPUT,HW_GPIO_FUNC_GPIO);//ADC_RDY
        //hw_gpio_configure_pin(PPS_RX_SUP_PORT,PPS_RX_SUP_PIN,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,0);//RX_SUP Control PIN disable
        hw_gpio_configure_pin(PPS_REST_PORT,PPS_REST_PIN,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,0);//REST low
        hw_gpio_configure_pin(PPS_TX_SUP_PORT,PPS_TX_SUP_PIN,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,0);//TX_SUP Control PIN disable

        hw_gpio_configure_pin(HW_GPIO_PORT_4,HW_GPIO_PIN_6,HW_GPIO_MODE_OUTPUT,HW_GPIO_FUNC_GPIO,0);//test io


}

void set_test_IO_high(void)
{
        hw_gpio_set_active(HW_GPIO_PORT_4, HW_GPIO_PIN_6);//1
}
void set_test_IO_low(void)
{
        hw_gpio_set_inactive(HW_GPIO_PORT_4, HW_GPIO_PIN_6);//1
}

void pps960_Rest_HW(void)
{
        //RESTZ Pin
        hw_gpio_set_inactive(PPS_REST_PORT, PPS_REST_PIN);//0
        OS_DELAY_MS(2);
        hw_gpio_set_active(PPS_REST_PORT, PPS_REST_PIN);//1
        OS_DELAY_MS(2);
}
void pps960_Rest_SW(void)
{
        //software rest
        PPS960_writeReg(0,0x8);
        OS_DELAY_MS(50);
}
void pps960_Disable_HW(void)
{
        //RESTZ Pin low
        hw_gpio_set_inactive(PPS_REST_PORT, PPS_REST_PIN);//0
        //TX_SUP Control PIN disable
        hw_gpio_set_inactive(PPS_TX_SUP_PORT, PPS_TX_SUP_PIN);//0
        //RX_SUP Control PIN disable
        //hw_gpio_set_inactive(PPS_RX_SUP_PORT, PPS_RX_SUP_PIN);//0
}

void pps960_Enable_HW(void)
{
        //RX_SUP Control PIN enable
        //hw_gpio_set_active(PPS_RX_SUP_PORT, PPS_RX_SUP_PIN);//1
        //TX_SUP Control PIN enable
        hw_gpio_set_active(PPS_TX_SUP_PORT, PPS_TX_SUP_PIN);//1
        OS_DELAY_MS(20);
        //RESTZ Pin high
        hw_gpio_set_active(PPS_REST_PORT, PPS_REST_PIN);//1
}

void init_pps960_sensor(void)
{
        //uint32_t ctr_temp=0;
        pps960_Init_gpio();
        pps960_Enable_HW();
        pps960_Rest_HW();
        init_PPS960_register();
        PPS960_init();
}


#define PPS960_POLL_TIMER_NOTIFY (1 << 1)

/* Poll timer callback */
static void pps960_timer_cb(OS_TIMER timer)
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

        static int lifeQhrm=0;
        uint32_t ret,notify;
        uint32_t pps_count;

        //init_pps960_sensor();

        //PRIVILEGED_DATA static OS_TIMER *pps960_timer;   //系统内存保护，这是FreeRTOS的V8.2系统新增加的事件组
        pps960_timer = OS_TIMER_CREATE("pps960_timer", 21 / OS_PERIOD_MS, OS_TIMER_SUCCESS,
                                                (void *) OS_GET_CURRENT_TASK(), pps960_timer_cb);
        OS_ASSERT(pps960_timer);   //OS_ASSERT：判断hrw_timer是否已建立，若因各种可能原因致创建失败，则向stderr打印一条错误信息，然后程序终止运行

        OS_TIMER_START(pps960_timer, 20 / OS_PERIOD_MS);
        acc_check = 1;

        for (;;)
        {

            ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notify, OS_TASK_NOTIFY_FOREVER);
            OS_ASSERT(ret == OS_OK);

            if(notify & PPS960_POLL_TIMER_NOTIFY)
            {
                    if(1) //open hr   //hr_flag == 2   判断以什么方式打开心率
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
                            #if PPS_ACC_USE_FIFO // for 1 second get gsensor raw data
                            ALGSH_dataForAgc();
                            #endif

                                                    }
                    else
                    //if( hr_flag == 1) //colse hr
                    {
                            if (hr_init_flag == 1) // second  close pps power
                            {
#ifdef DEMOBOARD
                                    hw_gpio_set_inactive(HW_GPIO_PORT_4, HW_GPIO_PIN_0);
#else
                                    hw_gpio_set_inactive(HW_GPIO_PORT_3, HW_GPIO_PIN_5);
#endif
                                    hr_init_flag = 0;
                            }
                            //if (hr_init_flag == 0)// first  not close  pps power
                            else
                            {

                            }
                    }
            }//(notify & PPS960_POLL_TIMER_NOTIFY)
        }//for(;;)
}

extern uint16_t GreenRaw;
#if PPS_ACC_USE_FIFO
void pps960_sensor_task2(void *params)
{

        //static int lifeQhrm=0;
        uint32_t ret,notify;
        uint32_t pps_count;
        uint32_t sample_count;

        acc_check = 1;

        for (;;)
        {

            if(acc_check)
            {
               OS_TASK_NOTIFY_TAKE(0, OS_EVENT_FOREVER);//程序进入此函数NO_WAIT
                //maybe gsensor raw data error.move to sensor task.
                uint16_t ledBuffLength = GetLEDBuffLenth();//pps 多少组数据

                uint16_t accBuffLength = GetAccBuffLenth();


                if(accBuffLength>ledBuffLength)
                {
                        AccBuffTail = accBuffLength-ledBuffLength+AccBuffTail;

                }
                uint16_t sample_count = GetHRSampleCount();//pps sample

                ClrHRSampleCount();//claer sample

                cm_sys_clk_set(sysclk_PLL48);
                //set IO High
                set_test_IO_high();
                for(int i=0;i<ledBuffLength;i++)
                {
                        ALGSH_dataToAlg();//添加 pps数据 到 alg
                }
                //set IO low
                set_test_IO_low();
                cm_sys_clk_set(sysclk_XTAL16M);

                uint16_t lifeQhrm=0;
                uint8_t rr_readyval=0;
                uint8_t isRRinValid=0;//0=>good RR;1=>outlier
                uint16_t retrievedRR;

                if(HR_HRV_enable!=1){
                        lifeQhrm = pps_getHR();
                }
                if(HR_HRV_enable==1 || HR_HRV_enable==2)
                {
                }
                int8_t snrValue = PP_GetHRConfidence();//for snr check
                int8_t skin = PP_IsSensorContactDetected();//for skin detect
                displayHrm = lifeQhrm;
                uint16_t gData=GreenRaw;
                printf("hrm = %d g_count=%d s_count=%d l_len=%d skin=%d SNR=%d gData=%d\r\n",lifeQhrm,accBuffLength,sample_count,ledBuffLength,skin,snrValue,gData);
            }//if(acc_check)
    }//for(;;)
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


void PPS960_readraw(uint16_t* GreenData,uint16_t *AMBData,uint16_t * IRData)
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

        //vPortEnterCritical();
///Green
        hw_i2c_write_byte(id, 44);
        rd_status = hw_i2c_read_buffer_sync(id, temp, 3, &abrt_src, HW_I2C_F_NONE);
        if ((rd_status < (size_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
               printf("pps960 read failure: %u" NEWLINE, abrt_src);
        }
        //printf("PPS960_readReg2");
        rdtemp = temp[0]<<16 | temp[1]<<8 | temp[2];
        *GreenData = afeConvP16(rdtemp);
 /// AMB
        hw_i2c_write_byte(id, 45);
        rd_status = hw_i2c_read_buffer_sync(id, temp, 3, &abrt_src, HW_I2C_F_NONE);
        if ((rd_status < (size_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
               printf("pps960 read failure: %u" NEWLINE, abrt_src);
        }
        //printf("PPS960_readReg2");
        rdtemp = temp[0]<<16 | temp[1]<<8 | temp[2];
        *AMBData = afeConvP16(rdtemp);
///IR
        hw_i2c_write_byte(id, 42);
        rd_status = hw_i2c_read_buffer_sync(id, temp, 3, &abrt_src, HW_I2C_F_NONE);
        if ((rd_status < (size_t)3) || (abrt_src != HW_I2C_ABORT_NONE)) {
               printf("pps960 read failure: %u" NEWLINE, abrt_src);
        }

        //printf("PPS960_readReg2");
        rdtemp = temp[0]<<16 | temp[1]<<8 | temp[2];
        *IRData = afeConvP16(rdtemp);

         //vPortExitCritical();
        // hw_i2c_reset_int_tx_abort(id);                //复位中止源寄存器
         ad_i2c_bus_release(dev);
         ad_i2c_device_release(dev);
         ad_i2c_close(dev);

}





