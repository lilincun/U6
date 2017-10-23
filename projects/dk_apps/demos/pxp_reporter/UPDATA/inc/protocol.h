
/*****************************************************************************
File Name:    protocol.h
Discription:  协议头文件
History:
Date		    Author		     Description
2016-12-22      邝云刚                 Creat
****************************************************************************/
#ifndef  __PROTOCOL__
#define __PROTOCOL__

#include "stdint.h"
#include "stdbool.h"
//#ifdef CFG_PROTOCOL
//解包状态定义
typedef enum tagEnumPTL_STATUS
{
	PTL_NO_ERROR = 0,		//解包正确
	PTL_CHECKSUM_ERROR,		//校验错误
	PTL_LENGTH_ERROR,		//包长度错误
	PTL_HEAD_ERROR,			//包头错误
	PTL_TAIL_ERROR,			//包尾错误
	PTL_DATA_ERROR			//数据域错误
}PTL_STATUS;




PTL_STATUS UnPack( const uint8_t* pBuffer,	   //缓存，输入参数
								uint32_t nLenOfBuf,             //缓存区长度，输入参数
								uint32_t* pIndexOfHead,         //包头索引，输出参数
								uint32_t* pIndexOfTail,         //包尾索引，输出参数
								uint8_t* pCmdID,                //命令ID，输出参数
								uint8_t* pData,                 //数据域缓存，输出参数
								uint32_t* pLenOfData);



//打包
//先生成包，再转义(转义后除了包头包尾，则中间数据不会出现包头包尾字符)
PTL_STATUS Pack( uint8_t nCmdID,
                 const uint8_t*	pDataBuf,
                 uint32_t nLenOfData,
		 uint8_t* pPackBuf,
		 uint32_t* pLenOfPack );


void protocol_test_f(void);
									
									
#endif

//#endif
