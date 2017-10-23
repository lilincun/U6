
/*****************************************************************************
File Name:    protocol.h
Discription:  Э��ͷ�ļ�
History:
Date		    Author		     Description
2016-12-22      ���Ƹ�                 Creat
****************************************************************************/
#ifndef  __PROTOCOL__
#define __PROTOCOL__

#include "stdint.h"
#include "stdbool.h"
//#ifdef CFG_PROTOCOL
//���״̬����
typedef enum tagEnumPTL_STATUS
{
	PTL_NO_ERROR = 0,		//�����ȷ
	PTL_CHECKSUM_ERROR,		//У�����
	PTL_LENGTH_ERROR,		//�����ȴ���
	PTL_HEAD_ERROR,			//��ͷ����
	PTL_TAIL_ERROR,			//��β����
	PTL_DATA_ERROR			//���������
}PTL_STATUS;




PTL_STATUS UnPack( const uint8_t* pBuffer,	   //���棬�������
								uint32_t nLenOfBuf,             //���������ȣ��������
								uint32_t* pIndexOfHead,         //��ͷ�������������
								uint32_t* pIndexOfTail,         //��β�������������
								uint8_t* pCmdID,                //����ID���������
								uint8_t* pData,                 //�����򻺴棬�������
								uint32_t* pLenOfData);



//���
//�����ɰ�����ת��(ת�����˰�ͷ��β�����м����ݲ�����ְ�ͷ��β�ַ�)
PTL_STATUS Pack( uint8_t nCmdID,
                 const uint8_t*	pDataBuf,
                 uint32_t nLenOfData,
		 uint8_t* pPackBuf,
		 uint32_t* pLenOfPack );


void protocol_test_f(void);
									
									
#endif

//#endif
