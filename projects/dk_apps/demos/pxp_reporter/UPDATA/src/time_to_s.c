#include"time_to_s.h"

SysTime_t CONST_TIME = {1970,1,1,0,0,0,0};

bool CheckIsRunYear( uint16_t year)
{
	bool bRet = false;
	if( ((0 == year%4) && (year%100!=0)) || (0 == year%400))
		bRet = true;

	return bRet;
}
uint32_t DateSwSec(SysTime_t date)
{
	uint32_t retSec = 0;
	uint32_t RunYearNum = 0;
	if(date.year<CONST_TIME.year)
	{
		return retSec;
	}

	uint8_t year = date.year - CONST_TIME.year;

	for(uint32_t i=CONST_TIME.year;i<date.year;i++)
	{
		if( CheckIsRunYear(i) )
		{
			RunYearNum++;
		}
	}
	uint8_t monthMax[12]={31,28,31,30,31,30,31,31,30,31,30,31};

	if( CheckIsRunYear(date.year) )
		monthMax[1]   = 29;
	uint32_t mon = date.month - CONST_TIME.month;
	uint16_t sunMontDay = 0;
	for(int i=0;i<mon;i++)
	{
		sunMontDay += monthMax[i];
	}
	
	uint32_t day = date.day - CONST_TIME.day;
	uint32_t SumDay = year*365 + RunYearNum + sunMontDay + day;
	uint32_t hour = date.hour - CONST_TIME.hour;
	uint32_t min = date.min - CONST_TIME.min;
	uint32_t sec = date.sec - CONST_TIME.sec;

	retSec = SumDay*3600*24 + hour*3600 + min*60 + sec;
	
	return retSec;
}
