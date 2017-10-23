/// NOTICE:
/// 
/// 
/// (1) The function prefix with _platform_ is platform dependency.
///     You should implement platform related function
/// (2) The function suffix with _optional_ is optional function.
///     You may need not do this. Remove it form the source.
///

///----------------------------------------------------------------------------
/// Headers
/// ---------------------------------------------------------------------------
#include "mc36xx_sample.h"
#include "mcube_custom_config.h" 
#include "m_drv_mc36xx.h"

//#include "gpio.h"

//#include "user_callback_config.h"
//#include "app_entry_point.h"
//#include "app.h"

//add
#include <platform_devices.h>
#include <hw_i2c.h> //系统默认路径：pxp_reporter/sdk,所以pxp_reporter/sdk/peripherals/include下的hw_i2c.h可以被包含到
#include <ad_i2c.h>
#include "stdio.h"
//#include "sensor.h"
///----------------------------------------------------------------------------
/// Local variables
/// ---------------------------------------------------------------------------


static uint8_t  sample_interval  ;//__attribute__((section("retention_mem_area0"),zero_init)); //ms
const S_M_DRV_MC_UTIL_OrientationReMap *_ptOrienMap = &g_MDrvUtilOrientationReMap[E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP];

#if defined(M_DRV_MC36XX_CFG_BUS_I2C)
extern uint16_t i2c_address_mc36xx ;//                   __attribute__((section("retention_mem_area0"),zero_init));
#endif

#ifdef MC36XX_ACC_FIFO_POLLING
#define GSENSOR_INTERVAL   100   //100*10ms 
#endif

#if SUPPORT_PEDOMETER
//static uint32_t pedo_time            ;//__attribute__((section("retention_mem_area0"),zero_init));
uint32_t pedo_time            ;//__attribute__((section("retention_mem_area0"),zero_init));
static uint32_t working_step_count   ;//__attribute__((section("retention_mem_area0"),zero_init));
static uint32_t running_step_count   ;//__attribute__((section("retention_mem_area0"),zero_init));
static uint32_t pre_total_step       ;//__attribute__((section("retention_mem_area0"),zero_init));
mCubeLibPedState_t cur_step_state    ;//__attribute__((section("retention_mem_area0"),zero_init));

static bool is_pedo_enable           ;//__attribute__((section("retention_mem_area0"),zero_init));
#endif 

#if SUPPORT_SLEEPMETER
static uint32_t sleep_time    __attribute__((section("retention_mem_area0"),zero_init));
static bool is_sleep_enable   __attribute__((section("retention_mem_area0"),zero_init));
#endif 

#if SUPPORT_LIFTARM
static bool is_liftarm_enable  __attribute__((section("retention_mem_area0"),zero_init));
#endif

#if SUPPORT_LONGSIT 
static bool is_longsit_enable  __attribute__((section("retention_mem_area0"),zero_init));
static uint32_t longsit_tsec   __attribute__((section("retention_mem_area0"),zero_init)); 
#define LONGSIT_INTERVAL  100  // 100*10ms 
#endif


#if SUPPORT_PEDOMETER
void enable_pedometer(void)
{
  unsigned long version; 
  if (!is_pedo_enable)
  {
        // init pedometer data;
	pedo_time=0;
	working_step_count = 0;
	running_step_count = 0;
	cur_step_state = 0; 
	cur_total_step = 0;
	pre_total_step = 0;
	version = Ped_GetVersion(); 
	//mcube_printf("Ped:ver: %x\r\n", version);
    if(!Ped_Open())
    {
       //mcube_printf("Ped_Open:fail.\r\n");
    }else
    {		
       //mcube_printf("Ped_Open:success!\r\n");
	}	
	is_pedo_enable = true;
	 
  }	

}

void disable_pedometer(void)
{
  if(is_pedo_enable)
  {
    Ped_Close();
	is_pedo_enable = false;

	//clear step data and stop timer while closing pedo
	pedo_time=0;
	working_step_count = 0;
	running_step_count = 0;
	cur_step_state = 0; 
	cur_total_step = 0;
	pre_total_step = 0;
  }	
}
#endif 


#if SUPPORT_SLEEPMETER

void enable_sleepmeter(void)
{
	unsigned long version; 
	if (!is_sleep_enable)
	{
	    version = mCubeSleep_get_version(); 
		mcube_printf("Sleep:ver: %x\r\n", version);
		mCubeSleep_open_with_sensitivity(SLPMTR_SENS_MEDIUM);
		if(mCubeSleep_is_opened() == 1)
		{
			is_sleep_enable = true;
			sleep_time = 0;
			mcube_printf("mCubeSleep_Open: ok! \r\n");
		}	
	}
}

void disable_sleepmeter(void)
{
    if (is_sleep_enable) {
        mCubeSleep_close(); // Close sleepmeter
        is_sleep_enable = false;
		sleep_time = 0;
    }
}
#endif 


#if SUPPORT_LIFTARM
void enable_liftarm(void)
{
	 if (!is_liftarm_enable)
	 { 
		liftarm_open();
		if(liftarm_is_open() == 1)
		{
		   is_liftarm_enable = true;
		   mcube_printf("liftarm_open: ok! \r\n");
		} 
	 }
}

void disable_liftarm(void)
{
    if (is_liftarm_enable)
	{
		liftarm_close();
		is_liftarm_enable = false;
	}
}
#endif

#if SUPPORT_LONGSIT 
static void mcube_longsit_timer_handle(void);

void enable_longsit(void)
{
	 if (!is_longsit_enable)
	 { 
		LongSit_open();
		if(LongSit_is_open() == 1)
		{
		   is_longsit_enable = true;
		   longsit_tsec = 0;
		   LongSit_set_time_threshold(30);//set timer to notify, 30minutes
		   mcube_printf("longsit_open: ok! \r\n");
		} 
	 }
}

void disable_longsit(void)
{
    if (is_longsit_enable)
	{
		LongSit_close();
		is_longsit_enable = false;
		longsit_tsec = 0;
	}
}
#endif


//unsigned char reg[64];

#ifndef MC36XX_ACC_FIFO_POLLING
void GPIO_SetISR_MC36xx(void)
{
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_2, INPUT, PID_GPIO, true);
    GPIO_RegisterCallback(GPIO2_IRQn, gsensor_handle_int);
    GPIO_EnableIRQ(GPIO_PORT_2, GPIO_PIN_2, GPIO2_IRQn, true, true, 0);
	  NVIC_EnableIRQ(GPIO2_IRQn);
}
#endif

int mcube_accel_init(void)
{
    int ret;
#if defined(M_DRV_MC36XX_CFG_BUS_SPI)
    //dialog_spi_init();
#elif defined(M_DRV_MC36XX_CFG_BUS_I2C)
    i2c_address_mc36xx = 0x4C;
#endif

	ret = M_DRV_MC36XX_Init();	// Initialize MC36XX
	if (M_DRV_MC36XX_RETCODE_SUCCESS == ret)
	{
	  	printf("Accel is MC36xx4C.\r\n");
	}
	else
	{
#if defined(M_DRV_MC36XX_CFG_BUS_I2C)
	    i2c_address_mc36xx = 0x6C;
		ret = M_DRV_MC36XX_Init();	
		if(M_DRV_MC36XX_RETCODE_SUCCESS == ret)
		{
		 printf("Accel is MC36xx_0x6C.\r\n");

		}
		else
#endif	
		{
		 printf("Unknow accel, init fail (%d). \r\n", ret);
			return 0;
		}
	}
	M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);  // Set MC36XX to stanby mode	
	M_DRV_MC36XX_ConfigRegRngResCtrl(E_M_DRV_MC36XX_RANGE_2G, E_M_DRV_MC36XX_RESOLUTION_12BIT);
	//Again4x 
	/*
	M_DRV_MC36XX_ReadDoff_Dgain();
	M_DRV_MC36XX_ReadAofsp();
	M_DRV_MC36XX_SetD0ff4x_DGain4x();
	*/
	//Again4x 
	
	// Enable FIFO in the watermaek mode
	M_DRV_MC36XX_SetSampleRate(E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_28Hz, E_M_DRV_MC36XX_SNIFF_SR_DEFAULT_6Hz);
	sample_interval= 35;  
	M_DRV_MC36XX_EnableFIFO(E_M_DRV_MC36XX_FIFO_CONTROL_ENABLE, E_M_DRV_MC36XX_FIFO_MODE_WATERMARK, FIFO_THRESHOLD);
#ifdef MC36XX_ACC_FIFO_POLLING
	//Disable fifo interrupt	
	M_DRV_MC36XX_ConfigINT(0,0,0,0,0);
#else
	// Enable the interrupt if threshold reached
	GPIO_SetISR_MC36xx();
	M_DRV_MC36XX_ConfigINT(1,0,0,0,0);
     
#endif
	M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_CWAKE);
	//M_DRV_MC36XX_ReadRegMap(reg);
	

 #if SUPPORT_PEDOMETER    
     enable_pedometer(); 
 #endif 

 #if SUPPORT_SLEEPMETER
     enable_sleepmeter();
 #endif 
 
 #if SUPPORT_LIFTARM 
     enable_liftarm();
 #endif

 #if SUPPORT_LONGSIT 
     enable_longsit();
	 app_timer_set(APP_ACCEL_LONGSIT_TIMER,TASK_APP,LONGSIT_INTERVAL);
 #endif

//add
#if 0
 #ifdef MC36XX_ACC_FIFO_POLLING
     app_timer_set(APP_ACCEL_TIMER,TASK_APP,GSENSOR_INTERVAL);
 #endif
#endif

  return 1;
}




/// 
/// IRQ handler for FIFO
/// 
void mcube_fifo_irq_handler(void /* or handler data */)
{
	unsigned char Wake = 0; 
	unsigned char Swake_sniff =0;
	unsigned char Fifo_threshold = 0; 

	S_M_DRV_MC36XX_InterruptEvent evt_mc36xx = {0};

	M_DRV_MC36XX_HandleINT(&evt_mc36xx);
	Wake = evt_mc36xx.bWAKE;
	Swake_sniff = evt_mc36xx.bSWAKE_SNIFF;
	Fifo_threshold = evt_mc36xx.bFIFO_THRESHOLD;

	if (Wake)
	{
	//interrupt for sniff;
	}
	if(Swake_sniff)
	{
	//interrupt for swake_sniff;
	}
	if(Fifo_threshold)
	{
		mcube_fifo_timer_handle();
	}
   
}


typedef struct {
	short RawData[FIFO_DEEP][M_DRV_MC_UTL_AXES_NUM];
	short DataLen; 
	uint32_t TimeStamp[FIFO_DEEP]; //ms 
}AccData;

static AccData GensorData   ;//__attribute__((section("retention_mem_area0"),zero_init));

//////////////////////////////////////////////////////////////////////////////////
//add
uint8_t mc_i2c_read_sensor_data(uint8_t BUF_READ_reg ,uint8_t *record_buf,uint8_t data_len)
{
        //i2c
#if 1
        //uint8_t sample_reg = 0x3C;      //BUF_STATUS_1 reports the number of data bytes in kx022 sample buffer
        //uint8_t BUF_READ_reg =0x3F; //Buffer output register of kx022
        //uint8_t BUF_READ_reg =0x02; //Buffer output register of mc
       // uint8_t data_len =0;

        size_t rd_status = 0;
         HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;

        //i2c_device_config *device = (i2c_device_config *) KX022;
         i2c_device_config *device = (i2c_device_config *) MC3630;
        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(device);

        //add : 11111
        //ad_i2c_open(device);


        ad_i2c_device_acquire(device);
        ad_i2c_bus_acquire(device);
#if 0
        //读0x3C寄存器获得kx022已有的采样个数 0x3c
        hw_i2c_write_byte(id, sample_reg);                //写sample_reg，已读回1byte的数据

        ///hw_i2c_is_tx_fifo_not_full(),1 = no full
        while( !hw_i2c_is_tx_fifo_not_full(id));          // 等，直到FIFO未满,则发读时钟，进行读操作
        hw_i2c_read_byte_trigger(id);                              //产生读模式

                while(!hw_i2c_get_rx_fifo_level(id));     //等FIFO内有至少1个数据，然后读回rx fifo内的值;  0 无数据
                data_len = hw_i2c_read_byte(id);          //从FIFO中读回1个byte的数据 date_len / 3 表示有多少 组（xyz）
                                                          //date_len最大是252个byte
        while (!hw_i2c_is_tx_fifo_empty(id));             //hw_i2c_is_tx_fifo_empty(),0 = not empty;  1 = empty
        while (hw_i2c_is_master_busy(id));                //fsm =0 at idle

        hw_i2c_reset_int_tx_abort(id);                  //复位中止源寄存器

#endif

//ma :add
///================================================================================
        //i2c buffer 中有32 byte
#if 1
        //data_len  = 6 ;
//         hw_i2c_read_byte_trigger(id);                              //产生读模式

        hw_i2c_prepare_dma_ex(id,(uint8_t)device->dma_channel, (uint16_t *)record_buf,data_len,HW_I2C_DMA_TRANSFER_MASTER_READ,NULL,NULL,false);
        while(!hw_i2c_is_tx_fifo_not_full(id));
        hw_i2c_write_byte(id, BUF_READ_reg);               //写BUF_READ_reg，告知要读取的寄存器

        hw_i2c_dma_start(id);   //start

        while (!hw_i2c_is_tx_fifo_empty(id));             //等待fifo中的数据读完
        while (hw_i2c_is_master_busy(id));

#endif

#if 0
        uint8_t  xyz_count = data_len / 32; //i2c buffer 有 32 byte
        //uint8_t  xyz_count = data_len >> 5; //i2c buffer 有 32 byte
        uint8_t i;

        ////add : ma for test
        i2c_cb_data transaction_data;
        transaction_data.config = device;
        transaction_data.success = true;
        transaction_data.abort_source = HW_I2C_ABORT_NONE;
        ///////

        for(i=0; i<xyz_count; i++)// 多少个32 byte）
        //for(i=0; i<2; i++)// 多少个32 byte）
        {
                hw_i2c_read_byte_trigger(id);                              //产生读模式
                //while (  hw_i2c_is_rx_fifo_not_empty(id) );        //检查 1 = not_empty
//                hw_i2c_prepare_dma_ex(id,(uint8_t)device->dma_channel, (uint16_t *)(record_buf+i*32),32,HW_I2C_DMA_TRANSFER_MASTER_READ,NULL,NULL,false);
                hw_i2c_prepare_dma_ex(id,(uint8_t)device->dma_channel, (uint16_t *)(record_buf+i*32),32,HW_I2C_DMA_TRANSFER_MASTER_READ
                        ,ad_i2c_transaction_cb,&transaction_data,false);        ////////add for test

                while(!hw_i2c_is_tx_fifo_not_full(id));            // not_full=1 直到FIFO未满,则发读时钟，进行读操作
                hw_i2c_write_byte(id, BUF_READ_reg);               //写BUF_READ_reg，告知要读取的寄存器

               // while(hw_i2c_get_rx_fifo_level(id))             //0 = empty
                hw_i2c_dma_start(id);   //start

                OS_EVENT_WAIT(device->bus_data->event, OS_EVENT_FOREVER);                       ///////add for test

               // while (!hw_i2c_is_tx_fifo_empty(id));           //等待fifo中的数据发送完
                while (hw_i2c_is_master_busy(id));

                while(hw_i2c_get_rx_fifo_level(id))             //0 = empty
                hw_i2c_reset_int_tx_abort(id);  //复位中止源寄存器
        }

        uint8_t xyz_number = data_len % 32;   //剩下的数据（不够32 byte）
        if (xyz_number != 0)
        {
                //while (  hw_i2c_is_rx_fifo_not_empty(id) );        //检查rx_fifo_not_empty
                hw_i2c_read_byte_trigger(id);                              //产生读时钟

                hw_i2c_prepare_dma_ex(id,(uint8_t)device->dma_channel, (uint16_t *)(record_buf+i*32),xyz_number,HW_I2C_DMA_TRANSFER_MASTER_READ,NULL,NULL,false);
                while(!hw_i2c_is_tx_fifo_not_full(id));            // not_full=1 直到FIFO未满,则发读时钟，进行读操作
                hw_i2c_write_byte(id, BUF_READ_reg);               //写BUF_READ_reg，告知要读取的寄存器

                //while(hw_i2c_get_rx_fifo_level(id))             //0 = empty
                hw_i2c_dma_start(id);                           //start


                //while (!hw_i2c_is_tx_fifo_empty(id));            //等待fifo中的数据发送完
                while (hw_i2c_is_master_busy(id));

                while(hw_i2c_get_rx_fifo_level(id))             //0 = empty
                hw_i2c_reset_int_tx_abort(id);  //复位中止源寄存器
        }

#endif

#if 0
        hw_i2c_read_byte_trigger(id);                              //产生读模式
        while(!hw_i2c_is_tx_fifo_not_full(id));
                hw_i2c_write_byte(id, BUF_READ_reg);               //写BUF_READ_reg，告知要读取的寄存器

        ///发data_len byte长度的对应时钟，读HW_I2C1的buf到record_buf
        rd_status = hw_i2c_read_buffer_sync(id, record_buf, data_len - 1, &abrt_src, HW_I2C_F_NONE);

        if (rd_status < data_len || (abrt_src != HW_I2C_ABORT_NONE)) {
                printf("kx022 read failure: %u" NEWLINE, abrt_src);
        }
        else {
                printf("read from kx022:  %s" NEWLINE, record_buf);
        }
        //hw_i2c_reset_int_tx_abort(id);  //复位中止源寄存器
#endif

        ad_i2c_bus_release(device);
        ad_i2c_device_release(device);

        //add 1111111111111111
        //ad_i2c_close(device);
        return data_len;
        // i2c end
#else
        //spi  start
        uint8_t cmd[3];
        uint8_t rsp[3]={0,0,0};
        spi_device dev;

        dev = ad_spi_open(MCUBE_SPI);
        spi_device_config *device = (spi_device_config *) dev;
        const HW_SPI_ID id = device->bus_id;

        //ad_spi_device_acquire(dev);
       // ad_spi_bus_acquire(dev);

        // void ad_spi_read(spi_device dev, uint8_t *rbuf, size_t rlen)
        //ad_spi_read(dev, destination, number_of_bytes);
        cmd[0] = BUF_READ_reg |0x80|0x40;// 1 1 x x    x x x x    0000 0000 read
        //cmd[1] = register_address |0x80|0x40;
        cmd[1] = 0x00;
        cmd[2] = 0x00;
        //ad_spi_transact(dev, cmd, 1, rsp, 1);  //read record_buf
        ad_spi_transact(dev, cmd, 1, record_buf, data_len);  //read
        //printf("rsp [0]= %u ,rsp [1]= %u,rsp [2]= %u\r",rsp[0],rsp[1],rsp[2]);
        //fflush(stdout);

        //ad_spi_bus_release(dev);
        //ad_spi_device_release(dev);
        ad_spi_close(dev);
#endif
}
////////////////////////////////////////////////////////////////////////////////

short buf_data[3];

void mcube_fifo_timer_handle(void)
{
        //printf("mcube_fifo_timer_handle....%d\r\n",__LINE__);
	unsigned char i, fifolength=0;
	unsigned char buf[6];
	short x,y,z;
	static char j  ;//__attribute__((section("retention_mem_area0"),zero_init));

	for (i = 0; i < FIFO_DEEP; i ++) 
	{
		 mcube_read_regs(0x08, buf, 1);

		 if (buf[0] & 0x10)
		 {
		      //  printf("over....%x\r\n",buf[0]);
			 break;
		 }
		 else 
		{	 // Read out raw data
			//mcube_read_regs(0x02, buf, 6);
			mc_i2c_read_sensor_data(0x02, buf, 6);
		      //  printf("buf[0]= %u ,buf[1]= %u ,buf[2]= %u ,buf[3]= %u ,buf[4]= %u ,buf[5]= %u ,\r",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
		        fflush(stdout);
			fifolength++;
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_X] = (short)(((unsigned short)buf[1] << 8) + (unsigned short)buf[0]);
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Y] = (short)(((unsigned short)buf[3] << 8) + (unsigned short)buf[2]);
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Z] = (short)(((unsigned short)buf[5] << 8) + (unsigned short)buf[4]);
			//printf("RAWData: %d, %d, %d\r\n", GensorData.RawData[i][M_DRV_MC_UTL_AXIS_X], GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Y], GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Z]);
#ifdef M_DRV_MC36XX_SUPPORT_LPF
			_M_DRV_MC36XX_LowPassFilter(&(GensorData.RawData[i][0]));
#endif
			//sign and map
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_X] = _ptOrienMap->bSign[M_DRV_MC_UTL_AXIS_X] * GensorData.RawData[i][_ptOrienMap->bMap[M_DRV_MC_UTL_AXIS_X]];
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Y] = _ptOrienMap->bSign[M_DRV_MC_UTL_AXIS_Y] * GensorData.RawData[i][_ptOrienMap->bMap[M_DRV_MC_UTL_AXIS_Y]];
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Z] = _ptOrienMap->bSign[M_DRV_MC_UTL_AXIS_Z] * GensorData.RawData[i][_ptOrienMap->bMap[M_DRV_MC_UTL_AXIS_Z]];

			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_X] = -GensorData.RawData[i][M_DRV_MC_UTL_AXIS_X];
			GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Y] = -GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Y];
		}
	}
	
	
	GensorData.DataLen = fifolength;
	//mcube_printf("FIFOLen = %d...%d\n",fifolength,__LINE__);



	for(i =0; i< fifolength; i++)
	{
  
			// +/-4g, 12bit, so 512 1x count is 1g; Make 1024 = 1g.
			x = GensorData.RawData[i][M_DRV_MC_UTL_AXIS_X];//<< 1;
			y = GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Y];//<< 1;
			z = GensorData.RawData[i][M_DRV_MC_UTL_AXIS_Z];//<< 1;

                      //  printf("x= %hd,y= %hd,z= %hd\r",x,y,z);
                        fflush(stdout);
                        ///add////////////////
                             if(i == 0)
                             {

                                     buf_data[0] = x;
                                     buf_data[1] = y;
                                     buf_data[2] = z;
                             }
			//printf("\n\r Accel: %d, %d, %d, %d\n\r", pedo_time, x, y, z);

     	  #if SUPPORT_PEDOMETER 			
			 // Push data into pedometer library
			if (is_pedo_enable)
		    { 
			 // save last total step count before process data

			        Ped_ProcessData(x, y, z);  //25hz

				 pre_total_step = cur_total_step;

				 cur_total_step =Ped_GetStepCount();//add
	
				 pedo_time += sample_interval;
				 GensorData.TimeStamp[i] = pedo_time;
				 if(cur_step_state == MCUBE_LIB_WALKING) 
				 	working_step_count += (cur_total_step - pre_total_step);

				 if(cur_step_state == MCUBE_LIB_RUNNING)
				 	running_step_count += (cur_total_step - pre_total_step);

				//printf("working: %d, running: %d, total: %d\r\n", working_step_count, running_step_count, cur_total_step);
              }
			 
          #endif 
		  
		  #if SUPPORT_SLEEPMETER
			if (is_sleep_enable)
			{
				slpmtr_input_t  sleepinput;
				sleepinput.mpss[0] = (int32_t) x*9807/1000;
				sleepinput.mpss[1] = (int32_t) y*9807/1000;
				sleepinput.mpss[2] = (int32_t) z*9807/1000;

				 if (j%6 != 0) 
				 {
					++j;
				 } else 
				 {
					   j= 1;
					   //mcube_printf("sleep:%d, %d, %d, %d \r\n",sleep_time, sleepinput.mpss[0], sleepinput.mpss[1], sleepinput.mpss[2]);
				     mCubeSleep_detect(&sleepinput, sleep_time);
				     sleep_time +=250;
				}

			}
		  #endif
		     
		  #if SUPPORT_LIFTARM
			 if(is_liftarm_enable)
			 {
		     	int32_t input[3] = {0};
				 //x,y,z 1g = 1024, 1024*9.8cnt/g
				input[0] = (int32_t) x*9807/1000;
				input[1] = (int32_t) y*9807/1000;
				input[2] = (int32_t) z*9807/1000; 
                //mcube_printf("LIFTARM: %d, %d, %d \r\n", input[0], input[1], input[2]);
		
				liftarm_process(input);
			
			  }	
		 #endif
 
				 
    }


		
}

#if SUPPORT_LONGSIT 
void mcube_longsit_timer_handle(void)
{

   if(is_longsit_enable && is_pedo_enable)
   {
     uint32_t step = Ped_GetStepCount();
	   mcube_printf("LongSit_Process:step= %d,time=%d\r\n", step, longsit_tsec);
	   LongSit_Process(step, longsit_tsec);
	   longsit_tsec ++;
   }
	

}	
#endif



int gsensor_init(void)
{
	return mcube_accel_init();
}

#ifndef MC36XX_ACC_FIFO_POLLING
void gsensor_handle_int(void)
{
	
	 mcube_fifo_irq_handler();
}
#endif

