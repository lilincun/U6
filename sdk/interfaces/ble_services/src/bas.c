/**
 ****************************************************************************************
 *
 * @file bas.c
 *
 * @brief Battery Service sample implementation
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

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "osal.h"
#include "ble_att.h"
#include "ble_bufops.h"
#include "ble_common.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "ble_storage.h"
#include "ble_uuid.h"
#include "svc_defines.h"
#include "bas.h"
#include "string.h"

//#include <oled_drv.h>
#include "protocol.h"
#include <updata.h>
extern OS_TIMER bas_tim;

#define UUID_BATTERY_LEVEL      (0x2A19)

#if 0
#define UUID_SAMPLE                "0783b03e-8535-b5a0-7140-a304d2495cc7"
#define UUID_SAMPLE_SERVER_TX      "0783b03e-8535-b5a0-7140-a304d2495cc9"
#define UUID_SAMPLE_SERVER_RX      "0783b03e-8535-b5a0-7140-a304d2495cc8"
#else
#define UUID_SAMPLE                "0783b03e-8535-b5a0-7140-a304d2495cb7"
#define UUID_SAMPLE_SERVER_TX      "0783b03e-8535-b5a0-7140-a304d2495cb8"
#define UUID_SAMPLE_SERVER_RX      "0783b03e-8535-b5a0-7140-a304d2495cba"
//#define UUID_SPS_FLOW_CTRL      "0783b03e-8535-b5a0-7140-a304d2495cb9"
#endif

extern OS_TASK updata_handle ;
extern OS_TASK updata_handle_1 ;
uint8_t time_data[10] ;
uint8_t ble_up_data = 1;
uint8_t read_sections = 0;

uint8_t updata_cache[240*8] = {0};
uint8_t cache_20[20] = {0};

typedef struct {
        ble_service_t svc;

        // handles
        uint16_t bl_val_h;
        uint16_t bl_ccc_h;
        uint16_t test_r_w_val_h;
} bat_service_t;

static att_error_t do_bl_ccc_write(bat_service_t *bas, uint16_t conn_idx, uint16_t offset,
                                                                uint16_t length, const uint8_t *value)
{
        uint16_t ccc;

        if (offset) {
                return ATT_ERROR_ATTRIBUTE_NOT_LONG;
        }

        if (length != sizeof(ccc)) {
                return ATT_ERROR_APPLICATION_ERROR;
        }

        ccc = get_u16(value);

        ble_storage_put_u32(conn_idx, bas->bl_ccc_h, ccc, true);

        return ATT_ERROR_OK;
}

static void notify_level(ble_service_t *svc, uint16_t conn_idx, uint8_t level);

static void handle_connected_evt(ble_service_t *svc, const ble_evt_gap_connected_t *evt)
{
        //printf("gap_connected 2\r\n");
        bat_service_t *bas = (bat_service_t *) svc;
        uint8_t level = 0x00;
        uint16_t level_len = sizeof(level);
        uint8_t prev_level;
        ble_error_t err;

        ble_gatts_get_value(bas->bl_val_h, &level_len, &level);

        prev_level = level;
        err = ble_storage_get_u8(evt->conn_idx, bas->bl_val_h, &prev_level);

        //printf("c prev_level = %d\r\n",prev_level);
        //printf("c level = %d\r\n",level);

        if (BLE_STATUS_OK == err && prev_level != level)
        {
                notify_level(svc, evt->conn_idx, level);
        }

        ble_storage_put_u32(evt->conn_idx, bas->bl_val_h, level, true);
}

static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
        //printf("bas read 1111111111111111111111\r\n");

        bat_service_t *bas = (bat_service_t *) svc;

        if (evt->handle == bas->bl_ccc_h)
        {
                //printf("read bl_ccc_h \r\n");
                uint16_t ccc = 0x0000;

                ble_storage_get_u16(evt->conn_idx, bas->bl_ccc_h, &ccc);
                //printf("bl_ccc_h read ccc = %d\r\n",ccc);

                // we're little-endian, ok to write directly from uint16_t
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(ccc), &ccc);
        }
        else if (evt->handle == bas->test_r_w_val_h )
        {
                static uint8_t level = 0;
                /* Default alert level - 'No Alert' */
                level++;
                //printf("test_r_w_n_val_h level = %d\r\n",level);

                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(level), &level);
        }
        else//(evt->handle == bas->bl_val_h)
        {
                //printf("read bl_val_h \r\n");
                static uint16_t ccc = 0x0;
                //printf("bl_val_h read ccc = %d\r\n",ccc);
                // we're little-endian, ok to write directly from uint16_t
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(ccc), &ccc);
                ccc++;
                //ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
        }

}

static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{
        printf("bas write 1111111111111111111111111\r\n");
        //printf("evt->length = %d\r\n",evt->length);

        bat_service_t *bas = (bat_service_t *) svc;
        att_error_t status = ATT_ERROR_ATTRIBUTE_NOT_FOUND;
        if (evt->handle == bas->bl_ccc_h)
        {
                //printf("bl_ccc_h evt->value0 = %d\r\n",evt->value[0]);
                //printf("bl_ccc_h evt->value1 = %d\r\n",evt->value[1]);
                status = do_bl_ccc_write(bas, evt->conn_idx, evt->offset, evt->length, evt->value);
                ble_gatts_write_cfm(evt->conn_idx, evt->handle, status);
        }
        else if (evt->handle == bas->test_r_w_val_h )
        {
                printf("test_r_w_val_h evt->value = %d\r\n",evt->value[0]);
                printf("test_r_w_val_h evt->value = %d\r\n",evt->value[1]);
                memcpy(time_data,evt->value,evt->length);
                if(time_data[1] == 0x10 )//CMD_TIME
                {
                        OS_TASK_NOTIFY(updata_handle, (1<<2), eSetBits);  //date time
                }
                else if(time_data[1] == 0x11)//CMD_GET_STEP
                {
                        ble_up_data = 1;
                        if (!OS_TIMER_IS_ACTIVE(bas_tim)) {  //////time  start
                                OS_TIMER_START(bas_tim, OS_TIMER_FOREVER);
                        }
                }
                else if(time_data[1] == 0x12)//CMD_GET_HR
                {
                        ble_up_data =  2;
                        if (!OS_TIMER_IS_ACTIVE(bas_tim)) {  //////time  start
                                OS_TIMER_START(bas_tim, OS_TIMER_FOREVER);
                        }

                }
                else if(time_data[1] == 0x13)//get save  step  data
                {
                        ble_up_data =  3;
                        read_sections ++;
                        printf("read_sections = %d\r\n",read_sections);
                        if (!OS_TIMER_IS_ACTIVE(bas_tim)) {  //////time  start
                                OS_TIMER_START(bas_tim, OS_TIMER_FOREVER);
                        }
                }
                else if(time_data[1] == 0x15)//updata current  step  data
                {
                        ble_up_data =  4;
                        if (!OS_TIMER_IS_ACTIVE(bas_tim)) {  //////time  start
                                OS_TIMER_START(bas_tim, OS_TIMER_FOREVER);
                        }
                }
                ble_gatts_write_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK);
        }
        else  //(evt->handle == bas->bl_val_h)
        {
                //printf("bl_val_h evt->value0 = %d\r\n",evt->value[0]);
                //printf("bl_val_h evt->value1 = %d\r\n",evt->value[1]);
                //printf(" write bl_val_h\r\n");
                printf("updata_handle_1\r\n");
                ble_gatts_write_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK);
                OS_TASK_NOTIFY(updata_handle_1, (1<<5), eSetBits);  //data
        }
}



ble_service_t *bas_init(const ble_service_config_t *config, const bas_battery_info_t *info)
{
        //uint16_t num_descr;
        uint16_t num_attr;
        //uint16_t cpf_h = 0;
        bat_service_t *bas;
        att_uuid_t uuid;
        uint8_t level = 0;

        bas = OS_MALLOC(sizeof(*bas));
        memset(bas, 0, sizeof(*bas));

        // Content Presentation Format descriptor present if 'info' is set

        //num_descr = (info ? 2 : 1);
        //num_attr = ble_service_get_num_attr(config, 1, num_descr);
        num_attr = ble_gatts_get_num_attr( 0, 2, 1);

       // ble_uuid_create16(UUID_SERVICE_BAS, &uuid);
        //ble_uuid_create16(0x1888, &uuid);

        ble_uuid_from_string(UUID_SAMPLE, &uuid);
        ////////////////////////////////////////////////////////
        ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

        //ble_service_config_add_includes(config);

        //ble_uuid_create16(UUID_BATTERY_LEVEL, &uuid);
        //ble_uuid_create16(0x2A1A, &uuid);
        ble_uuid_from_string(UUID_SAMPLE_SERVER_TX, &uuid);

        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_NOTIFY | GATT_PROP_READ |GATT_PROP_WRITE,
                                     //ble_service_config_elevate_perm(ATT_PERM_READ, config),
                                     ATT_PERM_READ | ATT_PERM_WRITE|ATT_PERM_NONE,
                                     //1,
                                     160,

                                     0,
                                     //GATTS_FLAG_CHAR_READ_REQ,

                                     NULL,
                                     &bas->bl_val_h);

        ble_uuid_create16(UUID_GATT_CLIENT_CHAR_CONFIGURATION, &uuid);//这里的uuid 是不可以改变的
        //ble_uuid_create16(0x2999, &uuid);
        ble_gatts_add_descriptor(&uuid,
                                 ATT_PERM_READ | ATT_PERM_WRITE,
                                 160,//uint16_t max_len

                                 0,//gatts_flag_t flags
                                 //GATTS_FLAG_CHAR_READ_REQ,

                                 &bas->bl_ccc_h);//uint16_t *h_offset
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //ble_uuid_create16(0x1233, &uuid);
        ble_uuid_from_string(UUID_SAMPLE_SERVER_RX, &uuid);
        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_READ | GATT_PROP_WRITE,
                                     ATT_PERM_RW,
                                     //sizeof(uint8_t),
                                     160,// uint16_t max_len
                                     0,
                                     //GATTS_FLAG_CHAR_READ_REQ,//gatts_flag_t flags
                                     NULL,//uint16_t *h_offset
                                     &bas->test_r_w_val_h);

//        if (info) {
//                ble_uuid_create16(UUID_GATT_CHAR_PRESENTATION_FORMAT, &uuid);
//                ble_gatts_add_descriptor(&uuid,
//                                        ble_service_config_elevate_perm(ATT_PERM_READ, config),
//                                        7, 0, &cpf_h);
//        }

        ble_gatts_register_service(&bas->svc.start_h,
                                   &bas->bl_val_h,
                                   &bas->bl_ccc_h,
                                   &bas->test_r_w_val_h,
                                   //&cpf_h,
                                   0);

        /* Set initial value for battery level so we always have proper characteristic value set. */
        ble_gatts_set_value(bas->bl_val_h, sizeof(level), &level);

//        if (info) {
//                uint8_t cpf_val[7];
//                uint8_t *p = cpf_val;
//
//                put_u8_inc(&p, 0x04);     // Format=unsigned 8-bit integer
//                put_u8_inc(&p, 0x00);     // Exponent=0
//                put_u16_inc(&p, 0x27AD);  // Unit=percentage
//                put_u8_inc(&p, info->namespace);
//                put_u16_inc(&p, info->descriptor);
//
//                // Content Presentation Format descriptor has static value
//                ble_gatts_set_value(cpf_h, sizeof(cpf_val), cpf_val);
//        }

        bas->svc.connected_evt = handle_connected_evt;
        bas->svc.read_req = handle_read_req;
        bas->svc.write_req = handle_write_req;

        bas->svc.end_h = bas->svc.start_h + num_attr;

        return &bas->svc;
}

static void notify_level(ble_service_t *svc, uint16_t conn_idx, uint8_t level)
{
        bat_service_t *bas = (bat_service_t *) svc;
        uint16_t ccc = 0x0000;

        ble_storage_get_u16(conn_idx, bas->bl_ccc_h, &ccc);

        if (!(ccc & GATT_CCC_NOTIFICATIONS)) {
                //printf("close NOTIFICATIONS\r\n");
                return;
        }
       //printf("open NOTIFICATIONS\r\n");
       ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, sizeof(level), &level);
}
extern uint32_t displayHrm;

extern uint32_t cur_total_step;//add

Display_Type  Displaymsg={
2017,
1,//month
1,//day
7, //
0, //h
0, //m
0, //s
72,//hr
8310,
15430,//run
365,
4300
};//add

uint8_t send_data_head(uint16_t conn_idx, uint16_t handle)
{
        uint8_t data_head[4]= {0x03,0x13,0xaa,0x55};
        ble_error_t  ble_ret ;
        ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &data_head[0]);//一个区start标志
        while(ble_ret != BLE_STATUS_OK ) {
                OS_DELAY_MS(10);
                ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &cache_20[0]);//一个区start标志
        }
}
uint8_t send_data_head_15(uint16_t conn_idx, uint16_t handle)
{
        uint8_t data_head[4]= {0x03,0x15,0xaa,0x55};
        ble_error_t  ble_ret ;
        ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &data_head[0]);//一个区start标志
        while(ble_ret != BLE_STATUS_OK ) {
                OS_DELAY_MS(10);
                ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &cache_20[0]);//一个区start标志
        }
}
uint8_t send_data_tail(uint16_t conn_idx, uint16_t handle)
{
        uint8_t data_head[4]= {0x03,0x13,0x55,0xaa};
        ble_error_t  ble_ret ;
        ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &data_head[0]);//一个区start标志
        while(ble_ret != BLE_STATUS_OK ) {
                OS_DELAY_MS(10);
                ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &cache_20[0]);//一个区start标志
        }
}

uint8_t send_data_tail_15(uint16_t conn_idx, uint16_t handle)
{
        uint8_t data_head[4]= {0x03,0x15,0x55,0xaa};
        ble_error_t  ble_ret ;
        ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &data_head[0]);//一个区start标志
        while(ble_ret != BLE_STATUS_OK ) {
                OS_DELAY_MS(10);
                ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, 4, &cache_20[0]);//一个区start标志
        }
}
uint8_t send_data_y_m_d(uint16_t conn_idx, uint16_t handle,uint8_t number,uint8_t *adder)
{
        ble_error_t  ble_ret ;
        ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, number,adder);//一个区start标志
        while(ble_ret != BLE_STATUS_OK ) {
                OS_DELAY_MS(10);
                ble_ret = ble_gatts_send_event(conn_idx, handle, GATT_EVENT_NOTIFICATION, number,adder);//一个区start标志
        }
}
extern uint16_t  save_12_number;
extern uint8_t   section_days;
static void data_notify_level(ble_service_t *svc, uint16_t conn_idx)
{
        bat_service_t *bas = (bat_service_t *) svc;
        uint16_t ccc = 0x0000;

        ble_storage_get_u16(conn_idx, bas->bl_ccc_h, &ccc);

        if (!(ccc & GATT_CCC_NOTIFICATIONS)) {
                //printf("close NOTIFICATIONS\r\n");
                return;
        }

       uint8_t data[13] = {0};
       uint32_t data_temp = 0 ;
 //      ble_up_data = 2;//add
       if(ble_up_data == 1) //step
       {
             data[0] = 12 ;data[1] = 0x11 ;data_temp = cur_total_step ;data[5] = data_temp & 0xff ;
             data[4] = (data_temp >> 8) & 0xff;data[3] = (data_temp >> 16) & 0xff; data[2] = (displayHrm ) & 0xff;
              //*( (uint32_t *) ( (uint8_t *)data + 2) )= GetTodaySteps(1);
             data[6]  = Displaymsg.dyear -2000;data[7] = Displaymsg.dmouth ;data[8] = Displaymsg.dday ;data[9] = Displaymsg.dweek ;
             data[10] = Displaymsg.dhour ;data[11] = Displaymsg.dmin ;data[12] = Displaymsg.dsec ;
             //printf("open NOTIFICATIONS\r\n");
             printf("....data_temp=%d....",data_temp);//add
             ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 13, data);

       }
       else if (ble_up_data == 2){ //hr
               data[0] = 12 ;data[1] = 0x12 ;data_temp = displayHrm ;data[5] = data_temp & 0xff ;
               data[4] = (data_temp >> 8) & 0xff;data[3] = (data_temp >> 16) & 0xff;data[2] = (data_temp >> 24) & 0xff;
              // ( (uint32_t *) ( (uint8_t *)data + 2) )  = displayHrm;
               data[6]  = Displaymsg.dyear -2000;data[7] = Displaymsg.dmouth ;data[8] = Displaymsg.dday ;data[9] = Displaymsg.dweek ;
               data[10] = Displaymsg.dhour ;data[11] = Displaymsg.dmin ;data[12] = Displaymsg.dsec ;
              //printf("open NOTIFICATIONS\r\n");
               printf("....data_temp=%d....",data_temp);//add
              ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 13, data);
       }
       else if (ble_up_data == 3){//save step
               if (OS_TIMER_IS_ACTIVE(bas_tim)) {  //////time  start
                       OS_TIMER_STOP(bas_tim, OS_TIMER_FOREVER);////////////time stop
               }
               uint8_t num_18 =0,num_18_remainder =0,i = 0,ret =0 ;
               ble_error_t ble_ret ;
               if(read_sections <= 15)
               {
                       //printf("updata_check_readaaaaaaaaaaaaaa\r\n");
                       ret = updata_check_read(read_sections);
                       if(ret == 1)
                       {
                               uint8_t save_4_number = check_number_read(read_sections);//  24*60 / 12 = 120
                               uint8_t number_240_8  = save_4_number*4*8 / (240*8); //1.9
                               uint16_t number_240_8_remainder = save_4_number*4*8 % (240*8);
                               uint8_t y_m_d_data[3]={0};
                               read_y_m_d_data(read_sections,(&y_m_d_data[0]),3,3);//3

                               //if(number_240_8 == 1)
                               if(number_240_8 == 2)
                               {
                                       read_data_from_flash(read_sections,(&updata_cache[0]),6,240*8);//240*8
                                       num_18 = (240*8 )/18; //106
                                       num_18_remainder = (240*8)%18;//12
                                       send_data_head(conn_idx,  bas->bl_val_h);
                                       cache_20[0] = 0x04 ; cache_20[1] = 0x13;cache_20[2] = y_m_d_data[0];cache_20[3] = y_m_d_data[1];cache_20[4] = y_m_d_data[2];
                                       send_data_y_m_d(conn_idx, bas->bl_val_h,5,&cache_20[0]);
                                      // ble_ret = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 5, &cache_20[0]);//一个区start标志

                                       cache_20[0] = 19 ;  cache_20[1] = 0x13;
                                       //printf("updata_check_read 0 cccccccccccccccccc 1 \r\n");
                                       for(i = 0 ;i<num_18;i++){//106
                                               memcpy(&cache_20[2],&updata_cache[i*18],18);
                                               ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                               while(ble_ret != BLE_STATUS_OK ){
                                                       OS_DELAY_MS(10);
                                                       ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                               }
                                       }
                                       if(i == num_18){
                                              // printf("updata_check_read 0 hhhhhhhhhhhhhhhhhhhh 1 \r\n");
                                               cache_20[0] = num_18_remainder + 1 ;
                                               memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);//12
                                               ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                               while(ble_ret != BLE_STATUS_OK ){
                                                       OS_DELAY_MS(10);
                                                       ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                               }
                                       }
                                       memset(&updata_cache[0],0,240*8);
                                       //read_data_from_flash(read_sections,(&updata_cache[0]),6+240*8,number_240_8_remainder);//number_240_8_remainder
                                       //num_18 = (number_240_8_remainder)/18; //106
                                       //num_18_remainder = (number_240_8_remainder)%18;
                                       //////////////////////////////////////////////////////
                                       read_data_from_flash(read_sections,(&updata_cache[0]),6+240*8,240*8);//number_240_8_remainder
                                       num_18 = (240*8)/18; //106
                                       num_18_remainder = (240*8)%18;
                                       cache_20[0] = 19 ; cache_20[1] = 0x13;
                                       //printf("updata_check_read 0 cccccccccccccccccc 2 \r\n");
                                       for(i = 0 ;i<num_18;i++){
                                               memcpy(&cache_20[2],&updata_cache[i*18],18);
                                               ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                               while(ble_ret != BLE_STATUS_OK ){
                                                       OS_DELAY_MS(10);
                                                       ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                               }
                                       }
                                       if(i == num_18){
                                               //printf("updata_check_read 0 hhhhhhhhhhhhhhhhhhhh 2 \r\n");
                                               cache_20[0] = num_18_remainder + 1 ;
                                               memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);
                                               ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                               while(ble_ret != BLE_STATUS_OK ){
                                                       OS_DELAY_MS(10);
                                                       ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                               }
                                       }
                                       send_data_tail(conn_idx, bas->bl_val_h);

                               }//(number_240_8 == 1)
                               else if(number_240_8 == 1)
                               {
                                              read_data_from_flash(read_sections,(&updata_cache[0]),6,240*8);//240*8
                                              num_18 = (240*8 )/18; //106
                                              num_18_remainder = (240*8)%18;//12
                                              send_data_head(conn_idx,  bas->bl_val_h);
                                              cache_20[0] = 0x04 ; cache_20[1] = 0x13;cache_20[2] = y_m_d_data[0];cache_20[3] = y_m_d_data[1];cache_20[4] = y_m_d_data[2];
                                              send_data_y_m_d(conn_idx, bas->bl_val_h,5,&cache_20[0]);
                                             //ble_ret = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 5, &cache_20[0]);//一个区start标志

                                              cache_20[0] = 19 ;  cache_20[1] = 0x13;
                                              //printf("updata_check_read 0 cccccccccccccccccc 1 \r\n");
                                              for(i = 0 ;i<num_18;i++){//106
                                                      memcpy(&cache_20[2],&updata_cache[i*18],18);
                                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                                      while(ble_ret != BLE_STATUS_OK ){
                                                              OS_DELAY_MS(10);
                                                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                                      }
                                              }
                                              if(i == num_18){
                                                      //printf("updata_check_read 0 hhhhhhhhhhhhhhhhhhhh 1 \r\n");
                                                      cache_20[0] = num_18_remainder + 1 ;cache_20[1] = 0x13;
                                                      memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);//12
                                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                                      while(ble_ret != BLE_STATUS_OK ){
                                                              OS_DELAY_MS(10);
                                                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                                      }
                                              }
                                              memset(&updata_cache[0],0,240*8);
                                              //read_data_from_flash(read_sections,(&updata_cache[0]),6+240*8,number_240_8_remainder);//number_240_8_remainder
                                              //num_18 = (number_240_8_remainder)/18; //106
                                              //num_18_remainder = (number_240_8_remainder)%18;
                                              //////////////////////////////////////////////////////
                                              read_data_from_flash(read_sections,(&updata_cache[0]),6+240*8,number_240_8_remainder);//number_240_8_remainder
                                              num_18 = (number_240_8_remainder)/18; //
                                              num_18_remainder = (number_240_8_remainder)%18;
                                              cache_20[0] = 19 ; cache_20[1] = 0x13;
                                              //printf("updata_check_read 0 cccccccccccccccccc 2 \r\n");
                                              for(i = 0 ;i<num_18;i++){
                                                      memcpy(&cache_20[2],&updata_cache[i*18],18);
                                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                                      while(ble_ret != BLE_STATUS_OK ){
                                                              OS_DELAY_MS(10);
                                                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                                      }
                                              }
                                              if(i == num_18){
                                                      //printf("updata_check_read 0 hhhhhhhhhhhhhhhhhhhh 2 \r\n");
                                                      cache_20[0] = num_18_remainder + 1 ;cache_20[1] = 0x13;
                                                      memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);
                                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                                      while(ble_ret != BLE_STATUS_OK ){
                                                              OS_DELAY_MS(10);
                                                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                                      }
                                              }
                                              send_data_tail(conn_idx, bas->bl_val_h);
                               }
                               else//(number_240_8 == 0)
                               {
                                       read_data_from_flash(read_sections,(&updata_cache[0]),6,number_240_8_remainder);//240*8
                                       num_18 = (number_240_8_remainder )/18;
                                       num_18_remainder = (number_240_8_remainder)%18;
                                       send_data_head(conn_idx, bas->bl_val_h);
                                       cache_20[0] = 0x04 ;cache_20[1] = 0x13;cache_20[2] = y_m_d_data[0];cache_20[3] = y_m_d_data[1];cache_20[4] = y_m_d_data[2];
                                       ble_ret = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 5, &cache_20[0]);//一个区start标志
                                       while(ble_ret != BLE_STATUS_OK ){
                                               OS_DELAY_MS(10);
                                               ble_ret = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 5, &cache_20[0]);//一个区start标志
                                       }
                                       cache_20[0] = 19 ;cache_20[1] = 0x13;
                                       //printf("updata_check_read 0 cccccccccccccccccc 3 \r\n");
                                       for(i = 0 ;i<num_18;i++){
                                               memcpy(&cache_20[2],&updata_cache[i*18],18);
                                               ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                               while(ble_ret != BLE_STATUS_OK ){
                                                       OS_DELAY_MS(10);
                                                       ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                                               }
                                       }
                                       if(i == num_18){
                                               //printf("updata_check_read 0 hhhhhhhhhhhhhhhhhhhh 3 \r\n");
                                               cache_20[0] = num_18_remainder + 1 ;cache_20[1] = 0x13;
                                               memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);
                                               ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                               while(ble_ret != BLE_STATUS_OK ){
                                                       OS_DELAY_MS(10);
                                                       ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                                               }
                                       }
                                       send_data_tail(conn_idx, bas->bl_val_h);
                               }//(number_240_8 == 0)
                               //mark_the_section_has_read(read_sections);//标记一个flash区 无效
                       }//(ret == 1)
            }//(read_sections<=15)

            if(read_sections == 15){
                     read_sections = 0;cache_20[0]= 0x03;cache_20[1]= 0x13;cache_20[2]= 0xEE;cache_20[3]= 0xEE;
                     ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 4, &cache_20[0]);//标记 所有区 查询 完成
            }
            else{
                     cache_20[0]= 0x03;cache_20[1]= 0x13;cache_20[2]= 0xCC;cache_20[3]= 0xCC;
                     ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 4, &cache_20[0]);//标记 所有区 continue查询
            }
       }//(ble_up_data == 3)//save step
       else if (ble_up_data == 4){ //get current  step
               //extern uint16_t  current_4_number = 0;
               //extern uint8_t section_days;
               if (OS_TIMER_IS_ACTIVE(bas_tim)) {  //////time  start
                         OS_TIMER_STOP(bas_tim, OS_TIMER_FOREVER);////////////time stop
               }
               uint8_t num_18 = 0;
               uint8_t num_18_remainder =0;
               uint8_t i = 0;
               ble_error_t ble_ret ;
               uint8_t save_4_number = save_12_number;//check_number_read(read_sections);//  24*60 / 12 = 120
               uint8_t number_240_8  = save_4_number*4*8 / (240*8); //1.9
               uint16_t number_240_8_remainder = save_4_number*4*8 % (240*8);//60*4*8   //60*4/60 = 4minutes
               uint8_t sections = section_days ;
               if(number_240_8 == 1)
               {
                      read_data_from_flash(sections,(&updata_cache[0]),6,240*8);//240*8
                      num_18 = (240*8 )/18; //106
                      num_18_remainder = (240*8)%18;//12
                      send_data_head_15(conn_idx,  bas->bl_val_h);
                      cache_20[0] = 19 ;
                      cache_20[1] = 0x15;
                      printf("updata_check_read 1 cccccccccccccccccc 1\r\n");
                      for(i = 0 ;i<num_18;i++){//106
                              memcpy(&cache_20[2],&updata_cache[i*18],18);
                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                              while(ble_ret != BLE_STATUS_OK ){
                                      OS_DELAY_MS(10);
                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                              }
                      }
                      if(i == num_18){
                              printf("updata_check_read 1 hhhhhhhhhhhhhhhhhhhh 1\r\n");
                              cache_20[0] = num_18_remainder + 1 ;
                              cache_20[1] = 0x15;
                              memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);//12
                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                              while(ble_ret != BLE_STATUS_OK ){
                                      OS_DELAY_MS(10);
                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                              }
                      }
                      memset(&updata_cache[0],0,240*8);
                      read_data_from_flash(sections,(&updata_cache[0]),6+240*8,number_240_8_remainder);//number_240_8_remainder
                      num_18 = (number_240_8_remainder)/18;
                      num_18_remainder = (number_240_8_remainder)%18;
                      cache_20[0] = 19 ;
                      cache_20[1] = 0x15;
                      printf("updata_check_read 1 cccccccccccccccccc 2 \r\n");
                      for(i = 0 ;i<num_18;i++){
                              memcpy(&cache_20[2],&updata_cache[i*18],18);
                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                              while(ble_ret != BLE_STATUS_OK ){
                                      OS_DELAY_MS(10);
                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                              }
                      }
                      if(i == num_18){
                              printf("updata_check_read 1 hhhhhhhhhhhhhhhhhhhh 2 \r\n");
                              cache_20[0] = num_18_remainder + 1 ;
                              cache_20[1] = 0x15;
                              memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);
                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                              while(ble_ret != BLE_STATUS_OK ){
                                      OS_DELAY_MS(10);
                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                              }
                      }
                      send_data_tail_15(conn_idx, bas->bl_val_h);
              }//(number_240_8 == 0)
              else{
                      read_data_from_flash(sections,(&updata_cache[0]),6,number_240_8_remainder);//240*8
                      num_18 = (number_240_8_remainder )/18;
                      num_18_remainder = (number_240_8_remainder)%18;
                      send_data_head_15(conn_idx, bas->bl_val_h);
                      cache_20[0] = 19 ;
                      cache_20[1] = 0x15;
                      printf("updata_check_read 2 cccccccccccccccccc\r\n");
                      for(i = 0 ;i<num_18;i++){
                              memcpy(&cache_20[2],&updata_cache[i*18],18);
                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                              while(ble_ret != BLE_STATUS_OK ){
                                      OS_DELAY_MS(10);
                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, 20, &cache_20[0] );
                              }
                      }
                      if(i == num_18){
                              printf("updata_check_read  hhhhhhhhhhhhhhhhhhhh\r\n");
                              cache_20[0] = num_18_remainder + 1 ;
                              cache_20[1] = 0x15;
                              memcpy(&cache_20[2],&updata_cache[i*18],num_18_remainder);
                              ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                              while(ble_ret != BLE_STATUS_OK ){
                                      OS_DELAY_MS(10);
                                      ble_ret  = ble_gatts_send_event(conn_idx, bas->bl_val_h, GATT_EVENT_NOTIFICATION, num_18_remainder+2, &cache_20[0]);
                              }
                      }
                      send_data_tail_15(conn_idx, bas->bl_val_h);
              }//(number_240_8 == 1)
       }//(ble_up_data == 4)//get current  step
}

//void bas_notify_level(ble_service_t *svc, uint16_t conn_idx)
//{
//        bat_service_t *bas = (bat_service_t *) svc;
//        uint8_t level = 0x00;
//        uint16_t level_len = sizeof(level);
//
//        ble_gatts_get_value(bas->bl_val_h, &level_len, &level);
//
//        notify_level(svc, conn_idx, level);
//
//        ble_storage_put_u32(conn_idx, bas->bl_val_h, level, true);
//}

void bas_set_level(ble_service_t *svc, uint8_t level, bool notify)
{
        bat_service_t *bas = (bat_service_t *) svc;
        uint8_t prev_level = 0x00;
        uint16_t prev_level_len = sizeof(prev_level);
        uint8_t num_conn;
        uint16_t *conn_idx;

        if (level > 100) {
                return;
        }

        ble_gatts_get_value(bas->bl_val_h, &prev_level_len, &prev_level);
        //printf("prev_level = %d\r\n",prev_level);
        //printf("level = %d\r\n",level);

        if (level == prev_level) {
                //printf("=====\r\n");
                return;
        }

        ble_gatts_set_value(bas->bl_val_h, sizeof(level), &level);

        /*
         * for each connected device we need to:
         * - notify new value, if requested by caller
         * - put new value to storage to use when device is reconnected
         */

        ble_gap_get_connected(&num_conn, &conn_idx);
        //printf("num_conn = %d\r\n",num_conn);

        while (num_conn--) {
                if (notify) {
                        notify_level(svc, conn_idx[num_conn], level);
                }

                ble_storage_put_u32(conn_idx[num_conn], bas->bl_val_h, level, true);
        }

        if (conn_idx) {
                OS_FREE(conn_idx);
        }
}

void data_updata_level(ble_service_t *svc,  bool notify)
{
        bat_service_t *bas = (bat_service_t *) svc;
        uint8_t num_conn;
        uint16_t *conn_idx;

#if 0
        uint8_t prev_level = 0x00;
        uint16_t prev_level_len = sizeof(prev_level);
        if (level > 100) {
                return;
        }

        ble_gatts_get_value(bas->bl_val_h, &prev_level_len, &prev_level);
        //printf("prev_level = %d\r\n",prev_level);
        //printf("level = %d\r\n",level);

        if (level == prev_level) {
                //printf("=====\r\n");
                return;
        }

        ble_gatts_set_value(bas->bl_val_h, sizeof(level), &level);
#endif
        /*
         * for each connected device we need to:
         * - notify new value, if requested by caller
         * - put new value to storage to use when device is reconnected
         */

        ble_gap_get_connected(&num_conn, &conn_idx);
        printf("num_conn = %d\r\n",num_conn);
#if 1
        while (num_conn--) {

                if (notify)
                {
                       // notify_level(svc, conn_idx[num_conn], level);
                        printf("prepare_into_data_notify_level(svc, conn_idx[num_conn])");//add
                        data_notify_level(svc, conn_idx[num_conn]);
                }
        }
#else
                if (notify)
                {
                          // notify_level(svc, conn_idx[num_conn], level);
                           data_notify_level(svc, conn_idx[num_conn]);
                 }

                //ble_storage_put_u32(conn_idx[num_conn], bas->bl_val_h, level, true);

#endif
        if (conn_idx) {
                OS_FREE(conn_idx);
        }
}
