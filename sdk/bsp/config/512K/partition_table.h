/*
 *****************************************************************************************
 *
 * Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 *****************************************************************************************
 */

PARTITION2( 0x000000 , 0x040000 , NVMS_FIRMWARE_PART        , 0 )
PARTITION2( 0x040000 , 0x010000 , NVMS_LOG_PART             , 0 )
PARTITION2( 0x050000 , 0x020000 , NVMS_GENERIC_PART         , PARTITION_FLAG_VES )
PARTITION2( 0x070000 , 0x00F000 , NVMS_PARAM_PART           , 0 )
PARTITION2( 0x07F000 , 0x001000 , NVMS_PARTITION_TABLE      , PARTITION_FLAG_READ_ONLY )

