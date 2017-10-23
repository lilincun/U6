// mCube custom config header file
#include "mcube_custom_config.h"
// Put your header files here
#include <stdarg.h>
//#include "common_uart.h"
#include "arch.h"

//add
#include "platform_devices.h"
#include "ad_i2c.h"
#include "hw_i2c.h"
#include "ad_spi.h"
#include "stdio.h"
#ifdef M_DRV_MC36XX_CFG_BUS_I2C 

uint16_t i2c_address_mc36xx;//__attribute__((section("retention_mem_area0"),zero_init));

#define TIMEOUT_COUNTER          0x3000UL


#define I2C_SPEED_MODE_MC36XX                          2       // Speed mode: 1 (100 kbits/s), 2 (400 kbits/s)
#define I2C_ADDRESS_MODE_MC36XX                        0       // Addressing mode: 0 (7 bit address), 1 (10 bit address)


#define SEND_I2C_COMMAND(X)                SetWord16(I2C_DATA_CMD_REG, (X))

#define WAIT_WHILE_I2C_FIFO_IS_FULL()      while((GetWord16(I2C_STATUS_REG) & TFNF) == 0)

#define WAIT_UNTIL_I2C_FIFO_IS_EMPTY()     while((GetWord16(I2C_STATUS_REG) & TFE) == 0)

#define WAIT_UNTIL_NO_MASTER_ACTIVITY()    while((GetWord16(I2C_STATUS_REG) & MST_ACTIVITY) !=0)

#define WAIT_FOR_RECEIVED_BYTE()           while(GetWord16(I2C_RXFLR_REG) == 0)


#else
//#include "spi.h"
//#include "user_periph_setup.h"
#define SPI_BUFSIZE 8
static uint8_t         m_tx_buf[SPI_BUFSIZE]    __attribute__((section("retention_mem_area0"),zero_init));           /**< TX buffer. */
#endif

const S_M_DRV_MC_UTIL_OrientationReMap
			 g_MDrvUtilOrientationReMap[E_M_DRV_UTIL_ORIENTATION_TOTAL_CONFIG] =
				  {
					  {{  1,  1,  1 }, { M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z }},
					  {{ -1,  1,  1 }, { M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z }},
					  {{ -1, -1,  1 }, { M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z }},
					  {{  1, -1,  1 }, { M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z }},
					  
					  {{ -1,  1, -1 }, { M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z }},
					  {{  1,  1, -1 }, { M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z }},
					  {{  1, -1, -1 }, { M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z }},
					  {{ -1, -1, -1 }, { M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z }},
				  };


// Interfaces which need our customers to take care of.

// Print function
void mcube_printf(const char *format, ...)
{
#if 0
        va_list argp;
	 char string[256]; 
	 va_start(argp, format);
	 vsprintf(string,format, argp); 
	 
	 printf_string(string);
	 
	 va_end(argp);
#endif
}

#define DELAY_1MSEC         1777                            // delay x * 1 msec
// Delay required miliseconds
void mcube_delay_ms(unsigned int ms)
{

    unsigned long j, jj;
    jj = ms * DELAY_1MSEC;
    for (j = 1; j<=jj; j++)
    {
        //__nop();
        //__nop();
    }
  //#error "Please implement real code"      // Remove this line after this function is implemented
}


  
#if defined(M_DRV_MC36XX_CFG_BUS_SPI)
#if 0
int dialog_spi_init(void)
{
  
    SPI_Pad_t cs_pad_param;
    
    cs_pad_param.pin = GPIO_PIN_0;
    cs_pad_param.port = GPIO_PORT_2;
  
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_HIGH, SPI_PHA_MODE_1, SPI_MINT_DISABLE, SPI_XTAL_DIV_4);     
	return 1;
}
#endif
#elif defined(M_DRV_MC36XX_CFG_BUS_I2C)
int dialog_i2c_init(void)
{

#if 0
    SetBits16(CLK_PER_REG, I2C_ENABLE, 1);                                        // enable  clock for I2C
    SetWord16(I2C_ENABLE_REG, 0x0);                                               // Disable the I2C controller
    SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE | I2C_RESTART_EN); // Slave is disabled
    SetBits16(I2C_CON_REG, I2C_SPEED, I2C_SPEED_MODE_MC36XX);                            // Set speed
    SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, I2C_ADDRESS_MODE_MC36XX);               // Set addressing mode
    SetWord16(I2C_TAR_REG, i2c_address_mc36xx & 0xFF);                             // Set Slave device address
    SetWord16(I2C_ENABLE_REG, 0x1);                                               // Enable the I2C controller
    WAIT_UNTIL_NO_MASTER_ACTIVITY();                                              // Wait for I2C master FSM to become IDLE
    return 1;
#endif

}

void dialog_i2c_release(void)
{
#if 0

    SetWord16(I2C_ENABLE_REG, 0x0);                                                         // Disable the I2C controller
    SetBits16(CLK_PER_REG, I2C_ENABLE, 0);                                          // Disable clock for I2C
#endif
}

#endif


//Customer should implement register read/write functions via i2c or spi. 
unsigned char mcube_write_regs(unsigned char register_address, unsigned char *value, unsigned char number_of_bytes)
{

#ifdef M_DRV_MC36XX_CFG_BUS_I2C 
	{
//for mcube
	        //uint8_t IsReady = 0;
	        i2c_device_config *dev = (i2c_device_config *) MC3630;
	        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(dev); //得到i2c2的寄存器 基地址（void*）0x50001500UL
	        ad_i2c_device_acquire(dev);
	        ad_i2c_bus_acquire(dev);

	#if 0
	        while( !hw_i2c_is_tx_fifo_not_full(id));          //1 = no full
	        hw_i2c_write_byte(id, register_address);        //写register address
	        while( !hw_i2c_is_tx_fifo_not_full(id));          //1 = no full
	        hw_i2c_write_byte(id, *value);            //写operate data
	        while (!hw_i2c_is_tx_fifo_empty(id));           //hw_i2c_is_tx_fifo_empty(),0 = not empty;  1 = empty
	        while (hw_i2c_is_master_busy(id));              //fsm =0 at idle
	                                                       //在时钟上升沿1us后产生停止信号，5uS后再传输第二个byte

	        hw_i2c_reset_int_tx_abort(id);                //复位中止源寄存器
	#endif
	        size_t wr_status = 0;
	        HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;

	        /*
	         * The first writing byte informs to which register rest data will be written.
	         */
	        hw_i2c_write_byte(HW_I2C2, register_address);
	        wr_status = hw_i2c_write_buffer_sync(HW_I2C2, value, number_of_bytes, &abrt_src, HW_I2C_F_WAIT_FOR_STOP);
	        if ( (wr_status < (size_t)number_of_bytes) || (abrt_src != HW_I2C_ABORT_NONE)  ) {
	                //printf("fm75 write failure: %u" NEWLINE, abrt_src);
	        }


	        ad_i2c_bus_release(dev);
	        //add
	        ad_i2c_device_release(dev);

	        return 1;


//old
#if 0
	        ASSERT_ERR(1 == number_of_bytes);
		dialog_i2c_init();

	    SEND_I2C_COMMAND(register_address & 0xFF);          // Set address LSB, write access
	    WAIT_WHILE_I2C_FIFO_IS_FULL();                      // Wait if I2C Tx FIFO is full
	    SEND_I2C_COMMAND(*value & 0xFF);                    // Write the data of the register
	    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                     // Wait until Tx FIFO is empty
	    WAIT_UNTIL_NO_MASTER_ACTIVITY();                    // wait until no master activity

		dialog_i2c_release();
	    return 1;
#endif

	}

#else
	{
	#if SPI_TWO_BYTE_ADDRESS

        #else
	uint8_t cmd[2];
	cmd[0] = register_address|0x40; //write
	cmd[1] = *value;
	spi_device dev;
	dev = ad_spi_open(MCUBE_SPI);
	//ad_spi_device_acquire(dev);
	//ad_spi_bus_acquire(dev);
	//void ad_spi_write(spi_device dev, const uint8_t *wbuf, size_t wlen);

	ad_spi_write(dev, cmd, sizeof(cmd)); // 0 1 xx xxxx  xxxx xxxx  write

	//ad_spi_bus_release(dev);
	//ad_spi_device_release(dev);

	ad_spi_close(dev);
	return 1;
	#endif

	}
#endif
	  
}

//Customer should implement register read/write functions via i2c or spi according to the platform

unsigned char mcube_read_regs(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes)
{
#ifdef M_DRV_MC36XX_CFG_BUS_I2C
	{
//for mcube

	        uint8_t i=0 ;
	        i2c_device_config *device = (i2c_device_config *) MC3630;
	        //i2c_device_config *device = (i2c_device_config *) KX022;
	        const HW_I2C_ID id = ad_i2c_get_hw_i2c_id(device);

	        //add : 11111
	        //ad_i2c_open(device);

	        ad_i2c_device_acquire(device);
	        ad_i2c_bus_acquire(device);

	#if 0

	        for(i=0 ; i<number_of_bytes; i++)
	        //for(i=0 ; i<1; i++)
	        {
	            hw_i2c_write_byte(id, register_address);                //写read register address 的值 加一个 r = 1

	            while( !hw_i2c_is_tx_fifo_not_full(id));         //1 = no full
	            hw_i2c_read_byte_trigger(id);                   //设置I2C_DATA_CMD_REG的bit8 为1（read）

	            while(hw_i2c_get_rx_fifo_level(id))             //0 = empty
	            destination[i] = hw_i2c_read_byte(id);                //从buffer中读回1个byte的数据

	            ///hw_i2c_is_tx_fifo_empty(),0 = Transmit FIFO is not empty;  1 = Transmit FIFO is empty
	            while (!hw_i2c_is_tx_fifo_empty(id));           //若FIFO为空=1，则跳出while

	            while (hw_i2c_is_master_busy(id));              //0: Master FSM is in IDLE state

	            hw_i2c_reset_int_tx_abort(id);                  //复位中止源寄存器

	        }
	#endif

	        size_t rd_status = 0;
	        HW_I2C_ABORT_SOURCE abrt_src = HW_I2C_ABORT_NONE;

	        /*
	         * Before reading values from sensor registers we need to send one byte information to it
	         * to inform which sensor register will be read now.
	         */
	        hw_i2c_write_byte(HW_I2C2, register_address);
	        rd_status = hw_i2c_read_buffer_sync(HW_I2C2, destination, number_of_bytes, &abrt_src, HW_I2C_F_NONE);
	        if ((rd_status < (size_t)number_of_bytes) || (abrt_src != HW_I2C_ABORT_NONE)) {
	               // printf("fm75 read failure: %u" NEWLINE, abrt_src);
	        }

	        ad_i2c_bus_release(device);
	        ad_i2c_device_release(device);

	        //add 1111111111111111
	        //ad_i2c_close(device);
	        return 1;


//old
#if 0
	    int j;
	    ASSERT_ERR((1 == number_of_bytes) || (6 == number_of_bytes));
		dialog_i2c_init();
	    SEND_I2C_COMMAND(register_address & 0xFF);      // Write the address of the register.
		for (j = 0; j < number_of_bytes; j++)
		{
		        WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
		        SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
		}
        
        // Critical section
        GLOBAL_INT_DISABLE();
        WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                 // Wait until I2C Tx FIFO empty
        // Get the received data
        for (j = 0; j < number_of_bytes; j++)                                         
        {
			uint32_t count = 0; 
			//WAIT_FOR_RECEIVED_BYTE();                   // Wait for received data
			while((GetWord16(I2C_RXFLR_REG) == 0) && (count ++ < TIMEOUT_COUNTER ));
			destination[j]=(0xFF & GetWord16(I2C_DATA_CMD_REG));  // Get the received byte
        }

        // End of critical section
        GLOBAL_INT_RESTORE();
		dialog_i2c_release();
		return 1;
#endif

	}

#else
	{
	#if SPI_TWO_BYTE_ADDRESS

        #else
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
	cmd[0] = register_address |0x80|0x40;// 1 1 x x    x x x x    0000 0000 read
	//cmd[1] = register_address |0x80|0x40;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	ad_spi_transact(dev, cmd, 1, rsp, 1);  //read
        printf("rsp [0]= %u ,rsp [1]= %u,rsp [2]= %u\r",rsp[0],rsp[1],rsp[2]);
        fflush(stdout);
	*destination = *rsp;

        //ad_spi_bus_release(dev);
	//ad_spi_device_release(dev);
	ad_spi_close(dev);
	return 1;
        #endif
	}

#endif

}

unsigned long mcube_burst_read(unsigned long address, unsigned char *buf, unsigned long size)
{
    return mcube_read_regs(address, buf, size);
}


