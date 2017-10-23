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

/*
 * When partition_table is not overridden by project defined partition_table
 * this file can be used to select partition table by macro definition.
 *
 * To use layout other than SDK one, add include path in project
 * settings that will point to folder with custom partition_table file.
 */

#if defined(USE_PARTITION_TABLE_2MB)
#include <2M/partition_table.h>
#elif defined(USE_PARTITION_TABLE_2MB_WITH_SUOTA)
#include <2M/suota/partition_table.h>
#elif defined(USE_PARTITION_TABLE_512K)
#include <512K/partition_table.h>
#elif defined(USE_PARTITION_TABLE_512K_WITH_SUOTA)
#include <512K/suota/partition_table.h>
#elif defined(USE_PARTITION_TABLE_1MB_WITH_SUOTA)
#include <1M/suota/partition_table.h>
#else
#include <1M/partition_table.h>
#endif

