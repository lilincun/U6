/*****************************************************************************
File Name:    protocol.c
Discription:  Э���
History:
Date		    Author		     Description
2016-12-22      ���Ƹ�                 Creat
****************************************************************************/
#include "protocol.h"

//#include "app.h"

//#ifdef CFG_PROTOCOL


static uint8_t  GetHeadChar(void) 		   {return 0xAA;} 	//��ͷ
static uint8_t  GetTailChar(void)		     {return 0x55;} 	//��β
static uint8_t  GetTranslateChar(void)	 {return 0xCC;} 	//ת���ַ�
static uint32_t GetPackMinLength(void)	 {return 6;} 			//����С����

uint8_t CheckCrc8(const uint8_t* pBuf,uint32_t nLen );

//�������
PTL_STATUS UnPack( const uint8_t* pBuffer,	   //���棬�������
								uint32_t nLenOfBuf,             //���������ȣ��������
								uint32_t* pIndexOfHead,         //��ͷ�������������
								uint32_t* pIndexOfTail,         //��β�������������
								uint8_t* pCmdID,                //����ID���������
								uint8_t* pData,                 //�����򻺴棬�������
								uint32_t* pLenOfData)
{
	if( !pBuffer || nLenOfBuf<GetPackMinLength() || !pIndexOfHead || !pIndexOfTail || !pLenOfData )
		return PTL_LENGTH_ERROR;

	//1.Ѱ��ͷ��
	bool bHasHead = false;
	for (uint32_t i= 0;i < nLenOfBuf;i ++)
	{
		if( pBuffer[i] == GetHeadChar() )
		{
			if( i > 0 && pBuffer[i-1] == GetTranslateChar() ) 
				continue;
			bHasHead = true;
			*pIndexOfHead = i;//��ȡͷ������
			break;
		}
	}
	//���Ҳ���ͷ�����򷵻ش���
	if(!bHasHead) 	
		return PTL_HEAD_ERROR;

	//2.Ѱ��β�����Ӱ�ͷ��ʼ��
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

	//��������(���ƻ���ȴ���������)
	if( nTailPos < 1 ) 
		return PTL_LENGTH_ERROR;

	//3.��һ�������з�ת��(������ͷ����β)
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

	//4.��֤������:1byte
	uint32_t nLength = pTempBuf[0];
	if( nLenofActData != nLength + 2 )
	{
		*pIndexOfHead = nTailPos +1;
		return PTL_LENGTH_ERROR;
	}

	//��ȡβ������
	*pIndexOfTail = nTailPos;

	//5.��֤У����
	uint8_t cCRC8 = CheckCrc8(pTempBuf,nLength + 1);
	if( cCRC8 != pTempBuf[nLenofActData-1] )
	{
		return PTL_CHECKSUM_ERROR;
	}

	//6.��ȡ����ID
	*pCmdID = pTempBuf[1];

	//7.��ȡ�����򳤶�
	*pLenOfData = nLength-1;

	//8.��ȡ������
	for(uint32_t i = 0;i < *pLenOfData;i++)
	{
		pData[i] = pTempBuf[i+2];
		
		//COMPrintf(" %d ",pData[i]);
	}

	
	return PTL_NO_ERROR;
}

//���
//�����ɰ�����ת��(ת�����˰�ͷ��β�����м����ݲ�����ְ�ͷ��β�ַ�)
PTL_STATUS Pack( 	uint8_t 			nCmdID,							 //����ID
									const uint8_t*	pDataBuf,					 //��Ҫ���������
									uint32_t 		nLenOfData,						 //���ݳ���
									uint8_t* 		pPackBuf,							 //����������
									uint32_t* 		pLenOfPack )				 //���������ݳ���
{
	if( !pPackBuf || !pLenOfPack )
		return PTL_DATA_ERROR;

	uint8_t pPackAct[256];
	memset(pPackAct,0,sizeof(pPackAct));

	//1.����ͷ
	pPackAct[0] = GetHeadChar();

	//2. ��䳤��(ID�������ֽ���)
	uint32_t nLength = nLenOfData +1;
	pPackAct[1] = (uint8_t)(nLength&0xFF);

	//3.�������ID
	pPackAct[2] = (uint8_t)nCmdID;

	//4.���������
	for(uint32_t i = 0;i < nLenOfData;i++)
		pPackAct[3+i] = pDataBuf[i];

	//5.���У���ֽ�(У�鳤�ȣ�����ID�Լ�����)
	pPackAct[3+nLenOfData] = CheckCrc8(pPackAct+1,nLenOfData + 2);

	//6.����β
	pPackAct[4+nLenOfData] = GetTailChar();

	//7.�����ݰ�����ת�壨������ͷ����β��
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

	//���ذ�����
	*pLenOfPack = nLenOfTransData+2;

	return PTL_NO_ERROR;
}

//=================================================================
//����CRC8У����
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
//Э����Ժ���
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

