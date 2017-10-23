/**
 ****************************************************************************************
 *
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

#include <updata.h>
//#include <stdbool.h>
#include <ad_flash.h>

#include "time_to_s.h"

#define  M_1     (0X100000)
#define  K_4     (0X1000)


//////////////////////////////////////////////////////////////
uint8_t updata_check_read(uint8_t section)
{
        uint32_t updata_check_addr = M_1 + K_4*section ;
        uint8_t updata_check_flag[2] = {0} ;
        size_t ret ;
        ret = ad_flash_read(updata_check_addr, updata_check_flag, 2);
        if( updata_check_flag[0]== 1 && updata_check_flag[1]==0xff )
        {
                return 1;
        }
        else
        {
                return 0;
        }
}
uint8_t check_number_read(uint8_t section)
{
        uint32_t check_addr = M_1 + K_4*section+ 1+ 1 ;
        uint8_t  read_save_number = 0 ;
        size_t ret ;
        ret = ad_flash_read(check_addr, &read_save_number, 1);
        if( ret == 1  )
        {
                return read_save_number;
        }
        else
        {
                return 0;
        }
}

uint8_t  read_y_m_d_data(uint8_t section,uint8_t *g_data_flash,uint8_t from_adder,uint8_t read_number)
{
        size_t ret ;
        uint32_t check_addr;

        check_addr = M_1 + K_4*section + from_adder ; //+ 1+1+1

        ret = ad_flash_read(check_addr,g_data_flash , read_number);

        if(ret == read_number)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}

uint8_t  read_data_from_flash(uint8_t section,uint8_t *g_data_flash,uint16_t from_adder,uint16_t read_number)
{
        size_t ret ;
        uint32_t check_addr;

        check_addr = M_1 + K_4*section + from_adder ; //+ 1+1+1 + 1+ 1+ 1

        ret = ad_flash_read(check_addr,g_data_flash , read_number);

        if(ret == read_number)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}

uint8_t mark_the_section_has_read(uint8_t section)
{
        uint8_t  has_write_flag = 1;

        uint32_t check_addr = M_1 + K_4*section+1;
        size_t ret ;

        ret = ad_flash_write(check_addr, &has_write_flag, 1);
        if(ret == 1)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}
/////////////////////////////////////////////////////////////////////////////////////////




uint8_t probe_has_write_section(uint8_t section)
{
        uint32_t check_addr = M_1 + K_4*section ;
        //ad_flash_read(uint32_t addr, uint8_t *buf, size_t len)
        uint8_t check_flag = 0 ;
        size_t ret ;
        ret = ad_flash_read(check_addr, &check_flag, 1);
        if(ret == 1)
        {
                if(check_flag == 1 )
                {
                        return 1;
                }
                else
                {
                       return 0;
                }

        }
        else
        {
                return 2;
        }

}

uint8_t mark_the_section_has_write(uint8_t section)
{
        uint8_t  has_write_flag = 1;

        uint32_t check_addr = M_1 + K_4*section;
        size_t ret ;

        ret = ad_flash_write(check_addr, &has_write_flag, 1);
        if(ret == 1)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}

uint8_t write_to_flash_number(uint8_t section,uint8_t number)
{
        uint32_t check_addr = M_1 + K_4*section + 1 + 1 ;
        size_t ret ;

        ret = ad_flash_write(check_addr, &number, 1);
        if(ret == 1)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}


uint8_t write_month_day(uint8_t section, uint8_t years ,uint8_t months,uint8_t days)
{
        uint8_t  write_data[3] ={0} ;
        write_data[0] = years;
        write_data[1] = months;
        write_data[2] = days;
        uint32_t check_addr = M_1 + K_4*section + 1 + 1 + 1 ;
        size_t ret ;

        ret = ad_flash_write(check_addr, write_data, 3);
        if(ret == 3)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}

uint8_t  three_minuts_save_one_data(SysTime_t current_time,uint8_t times ,uint8_t *g_data_flash,uint32 g_data)
{

      uint32_t second_save =  DateSwSec(current_time);
      *(g_data_flash+(times)*8 + 3) = (second_save)&0xff;
      *(g_data_flash+(times)*8 + 2) = (second_save>>8)&0xff;
      *(g_data_flash+(times)*8 + 1) = (second_save>>16)&0xff;
      *(g_data_flash+(times)*8 + 0) = (second_save>>24)&0xff;

      *(g_data_flash+(times)*8 + 7) = (g_data)&0xff;
      *(g_data_flash+(times)*8 + 6) = (g_data>>8)&0xff;
      *(g_data_flash+(times)*8 + 5) = (g_data>>16)&0xff;
      *(g_data_flash+(times)*8 + 4) = (g_data>>24)&0xff;
      return 1;

}
uint8_t erase_flash_section(uint8_t section)
{
        uint32_t check_addr = M_1 + K_4*section;
        // 每次擦除 4k 所以地址要以4k对齐
        ad_flash_erase_region(check_addr, K_4);
        return 1;
}

uint8_t  write_data_to_flash(uint8_t section,uint8_t *g_data_flash,uint16_t index)
{
        size_t ret ;
        uint32_t check_addr;

        check_addr = M_1 + K_4*section + 1 + 1+ 1 + 3 + index*8*4;
        ret = ad_flash_write(check_addr,g_data_flash , 4*8);
        if(ret == 4*8)
        {
                return 1;
        }
        else
        {
                return 0;
        }
}



#if 0
//uint8_t  aaaaa = 2 ;
ad_flash_write(0x100001, &aaaaa, 1);
// 每次擦除 4k 所以地址要以4k对齐
ad_flash_erase_region(0x100000, 1);

//uint8_t init_pm_mode = 1 ;
size_t ad_flash_read(uint32_t addr, uint8_t *buf, size_t len)
#endif
