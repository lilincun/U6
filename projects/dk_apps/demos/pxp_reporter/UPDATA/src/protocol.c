/*****************************************************************************
File Name:    protocol.c
Discription:  协议层
History:
Date		    Author		     Description
2016-12-22      邝云刚                 Creat
****************************************************************************/
#include "protocol.h"

//#include "app.h"

//#ifdef CFG_PROTOCOL


static uint8_t  GetHeadChar(void) 		   {return 0xAA;} 	//包头
static uint8_t  GetTailChar(void)		     {return 0x55;} 	//包尾
static uint8_t  GetTranslateChar(void)	 {return 0xCC;} 	//转义字符
static uint32_t GetPackMinLength(void)	 {return 6;} 			//包最小长度

uint8_t CheckCrc8(const uint8_t* pBuf,uint32_t nLen );

//解包函数
PTL_STATUS UnPack( const uint8_t* pBuffer,	   //缓存，输入参数
								uint32_t nLenOfBuf,             //缓存区长度，输入参数
								uint32_t* pIndexOfHead,         //包头索引，输出参数
								uint32_t* pIndexOfTail,         //包尾索引，输出参数
								uint8_t* pCmdID,                //命令ID，输出参数
								uint8_t* pData,                 //数据域缓存，输出参数
								uint32_t* pLenOfData)
{
	if( !pBuffer || nLenOfBuf<GetPackMinLength() || !pIndexOfHead || !pIndexOfTail || !pLenOfData )
		return PTL_LENGTH_ERROR;

	//1.寻找头部
	bool bHasHead = false;
	for (uint32_t i= 0;i < nLenOfBuf;i ++)
	{
		if( pBuffer[i] == GetHeadChar() )
		{
			if( i > 0 && pBuffer[i-1] == GetTranslateChar() ) 
				continue;
			bHasHead = true;
			*pIndexOfHead = i;//获取头部索引
			break;
		}
	}
	//若找不到头部，则返回错误
	if(!bHasHead) 	
		return PTL_HEAD_ERROR;

	//2.寻找尾部（从包头开始）
	uint32_t nTailPos = 0;
	for (uint32_t i= *pIndexOfHead +1;i < nLenOfBuf;i ++)
	{
		if( pBuffer[i] == GetTailChar() )
		{
			nTailPos = i;
			break;
		}
		if( pBuffer[i] == GetHeadChar() )
		{
			nTailPos = i-1;
			break;
		}
	}

	//包不完整(包破坏或等待继续接收)
	if( nTailPos < 1 ) 
		return PTL_LENGTH_ERROR;

	//3.对一个包进行反转义(除开包头、包尾)
	uint32_t nLenofActData 	= 0;
	
	uint8_t pTempBuf[256];
	memset(pTempBuf,0,sizeof(pTempBuf));
		
	for( uint32_t i=*pIndexOfHead+1; i<nTailPos; i++)
	{
		uint8_t curChar = pBuffer[i];
		if( curChar ==GetTranslateChar() )
			pTempBuf[nLenofActData++] = pBuffer[++i]-1;
		else
			pTempBuf[nLenofActData++] = pBuffer[i];
	}

	//4.验证包长度:1byte
	uint32_t nLength = pTempBuf[0];
	if( nLenofActData != nLength + 2 )
	{
		*pIndexOfHead = nTailPos +1;
		return PTL_LENGTH_ERROR;
	}

	//获取尾部索引
	*pIndexOfTail = nTailPos;

	//5.验证校验码
	uint8_t cCRC8 = CheckCrc8(pTempBuf,nLength + 1);
	if( cCRC8 != pTempBuf[nLenofActData-1] )
	{
		return PTL_CHECKSUM_ERROR;
	}

	//6.获取命令ID
	*pCmdID = pTempBuf[1];

	//7.获取数据域长度
	*pLenOfData = nLength-1;

	//8.获取数据域
	for(uint32_t i = 0;i < *pLenOfData;i++)
	{
		pData[i] = pTempBuf[i+2];
		
		//COMPrintf(" %d ",pData[i]);
	}

	
	return PTL_NO_ERROR;
}

//打包
//先生成包，再转义(转义后除了包头包尾，则中间数据不会出现包头包尾字符)
PTL_STATUS Pack( 	uint8_t 			nCmdID,							 //命令ID
									const uint8_t*	pDataBuf,					 //需要打包的数据
									uint32_t 		nLenOfData,						 //数据长度
									uint8_t* 		pPackBuf,							 //打包后的数据
									uint32_t* 		pLenOfPack )				 //打包后的数据长度
{
	if( !pPackBuf || !pLenOfPack )
		return PTL_DATA_ERROR;

	uint8_t pPackAct[256];
	memset(pPackAct,0,sizeof(pPackAct));

	//1.填充包头
	pPackAct[0] = GetHeadChar();

	//2. 填充长度(ID，数据字节数)
	uint32_t nLength = nLenOfData +1;
	pPackAct[1] = (uint8_t)(nLength&0xFF);

	//3.填充命令ID
	pPackAct[2] = (uint8_t)nCmdID;

	//4.填充数据域
	for(uint32_t i = 0;i < nLenOfData;i++)
		pPackAct[3+i] = pDataBuf[i];

	//5.填充校验字节(校验长度，控制ID以及数据)
	pPackAct[3+nLenOfData] = CheckCrc8(pPackAct+1,nLenOfData + 2);

	//6.填充包尾
	pPackAct[4+nLenOfData] = GetTailChar();

	//7.对数据包进行转义（除开包头、包尾）
	uint32_t nLenOfTransData = 0;
	pPackBuf[0] = pPackAct[0];
	for(uint32_t i = 1;i < nLenOfData+4;i++)
	{
		uint8_t curChar = pPackAct[i];
		if(curChar == GetHeadChar() || curChar == GetTailChar() || curChar ==GetTranslateChar())
		{
			pPackBuf[nLenOfTransData+1] = GetTranslateChar();
			pPackBuf[nLenOfTransData+2] = curChar+1;
			nLenOfTransData++;
		}
		else
			pPackBuf[nLenOfTransData+1] = curChar;
		
		nLenOfTransData++;
	}
	pPackBuf[nLenOfTransData+1] = pPackAct[4+nLenOfData];

	//返回包长度
	*pLenOfPack = nLenOfTransData+2;

	return PTL_NO_ERROR;
}

//=================================================================
//计算CRC8校验码
//=================================================================
uint8_t CheckCrc8(const uint8_t* pBuf,uint32_t nLen )
{
	if( !pBuf || nLen < 1) 
		return 0xFF;

	uint16_t crc, thisbyte, i, shift, lastbit;
	crc = 0xffff;
	for(i =0 ;i<nLen; i++)
	{
		thisbyte = pBuf[i];
		crc = crc^thisbyte;
		for( shift =1; shift <=8; shift++)
		{
			lastbit = crc&0x0001;
			crc = (crc>>1)&0x7fff;
			if(lastbit == 0x0001)
			{
				crc = crc^0xa001;
			}
		}
	}
	return(crc&0xff);
}



#if 0
//协议测试函数
#define LEN 100
#define BUF_LEN 120
#define CMDID 0x55
void protocol_test_f(void)
{
	uint8_t test_buf[LEN];
	uint8_t send_buf[BUF_LEN];
	uint8_t rec_buf[BUF_LEN];
	uint32_t len_pack = 0;
	for(int i=0;i<LEN;i++)
	{
			test_buf[i] = i+80;
			COMPrintf("%d ",test_buf[i]);
	}

	Pack(CMDID,test_buf,LEN,send_buf,&len_pack);
	uint32_t IndexOfHead = 0;
	uint32_t IndexOfTail = 0;
	uint8_t cmd = 0;
	uint32_t dat_len = 0;
	for(int i=0;i<BUF_LEN;i++)
	{
			if( PTL_NO_ERROR == UnPack(send_buf,i,&IndexOfHead,&IndexOfTail,&cmd,rec_buf,&dat_len))
			{
					COMPrintf(" UnPack PTL_NO_ERROR\r\t\n");
					COMPrintf("CMD id:%d\r\t\n",cmd);
					COMPrintf("dat_len:%d\r\t\n",dat_len);
			}
	}

}
#endif




//#endif

