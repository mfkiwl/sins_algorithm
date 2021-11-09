/**
 *  athur:  tony.jiang 
*   date :2021 04 23 
*/
#include "typedef.h"
#include "gnss_paraser.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <mathmisc.h>
const static uint8_t NMEA_LENGTH = 240;
const static double EARTH_RADIUS =6378.137 ;

const static double DIMEN_DU =110.9489 ;
const static double DIMEN_MIN =(110.9489)/ 60 ;
const static double DIMEN_SEC= (110.9489) / 3.6 ;
static gnrmc_type gnrmc;
static gngga_type gngga;

uint8_t NMEA_Comma_Pos(uint8_t *buf, uint8_t cx)
{
    uint8_t *p = buf, i;
    for (i = NMEA_LENGTH; cx > 0 && i > 0; i--)
	{
		if (*buf == '*' || *buf<' ' || *buf>'z')return 0XFF;//There is no CX comma
		if (*buf == ',')cx--;
		buf++;
	}
	return buf - p;
}


uint32_t NMEA_Pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	for (; n > 0; n--)
	{
		result *= m;
	}
	return result;
}

void NMEA_Str2num(uint8_t *buf, uint8_t*dx, double * rs)
{
	uint8_t  *p = buf;
	uint32_t ires = 0, fres = 0;
	uint8_t  ilen = 0, flen = 0, i;
	uint8_t   mask = 0, over = 1;
	for (i = 15; over > 0 && i > 0; p++, i--) 
	{
		if (*p == ',' || (*p == '*')) { over--; break; }
		else if (*p == '.')mask |= 0X01;
		else if (*p > '9' || (*p < '0'))	
		{
			ilen = 0;
			flen = 0;
			break;
		}
		if (mask & 0X01)flen++;  
		else ilen++; 
	}

	flen = flen - 1;
	for (i = 0; i < ilen; i++)	
	{
		ires += NMEA_Pow(10, ilen - 1 - i)*(buf[i] - '0');
	}
	if (flen >= 6)flen = 6;	
	*dx = flen;	 		
	for (i = 0; i < flen; i++)	
	{
		fres += NMEA_Pow(10, flen - 1 - i)*(buf[ilen + 1 + i] - '0');
	}
	*rs = ires * NMEA_Pow(10, flen) + fres;
}

void NMEA_Str1num(uint8_t *buf, uint8_t *dx, uint32_t *inte, uint32_t *deci)
{
    uint8_t *p = buf;
    uint32_t ires = 0, fres = 0;
    uint8_t  ilen = 0, flen = 0, i;
    uint8_t  mask = 0, over = 1;
    for (i = 15; over > 0 && i > 0; p++, i--) 
    {
        if (*p == ',' || (*p == '*'))
        {
            over--;
            break;
        } 
        else if (*p == '.')
            mask |= 0X01;                
        else if (*p > '9' || (*p < '0')) 
        {
            ilen = 0;
            flen = 0;
            break;
        }
        if (mask & 0X01)
            flen++; 
        else
            ilen++; 
    }
    for (i = 0; i < ilen; i++) 
    {
        ires += NMEA_Pow(10, ilen - 1 - i) * (buf[i] - '0');
    }
    if (flen >= 6)
        flen = 6;              
    *dx = flen;               
    for (i = 0; i < flen; i++) 
    {
        fres += NMEA_Pow(10, flen - 1 - i) * (buf[ilen + 1 + i] - '0');
    }
    *inte = ires;
    *deci = fres;
}

double NMEAstr2Double(uint8_t *buf)
{
    uint8_t mask = 0;
    uint8_t i;
    double npFlag = 1;

    double result = 0.0;
    double index = 1.0;

    for (i = 0; i < 64; i++)
    {
        if (buf[i] == '.')
            mask |= 0X01;
        else if (buf[i] == '-')
            npFlag = -1;
        else if (buf[i] <= '9' && (buf[i] >= '0'))
        {
            int data = buf[i] - '0';
            if (mask & 0X01)
            {
                index = index * 0.1;
                result = result + data * index;
            }
            else
            {
                result = result * 10 + data;
            }
        }
        else
        {
            break;
        }
    }

    return npFlag * result;
}

char *str(const char *str1, const char *str2)
{
    uint8_t len,
    i;
    char *p1 = (char *)str1 + 1, *p2 = (char *)str2;
    len = strlen(str2);
    for (i = 0; i < len; i++)
    {
        if (*p1 == *p2)
        {
            p1++;
            p2++;
        }
        else
            break;
    }
    if (i == len)
        return (char*)str1;
    else
        return 0;
}

uint8_t GPS_Check(uint8_t *buf) 
{
    uint8_t checksum = 0;
    uint8_t i = 0;

    for (i = 0; i < NMEA_LENGTH; i++)
    {
        if (buf[i] == '*')
            break; 
        else
            checksum ^= buf[i];
    }
    if (i < NMEA_LENGTH-1)
        return checksum; //
    else
        return 0; 
}

uint8_t GPS_verify(uint8_t *buf, NMEA type) 
{
    uint8_t *p1;
    uint8_t posx;
    uint8_t temp=0, temp1=0;

    if (type == GNRMC)
    {
        p1 = (uint8_t *)str((char *)buf, "GNRMC");
        posx = NMEA_Comma_Pos(p1, 12);        
        if (posx != 0XFF)
        {
            temp = (*(p1 + posx + 2) >= 'A' ? (*(p1 + posx + 2) - 55) : (*(p1 + posx + 2) - '0')) * 16; 
            temp1 = *(p1 + posx + 3) >= 'A' ? (*(p1 + posx + 3) - 55) : (*(p1 + posx + 3) - '0');       
            temp = temp + temp1;
        }
    }

    else if (type == GNGGA)
    {
        p1 = (uint8_t *)str((char *)buf, "GNGGA");
        posx = NMEA_Comma_Pos(p1, 14);       
        if (posx != 0XFF)
        {
            temp = (*(p1 + posx + 1) >= 'A' ? (*(p1 + posx + 1) - 55) : (*(p1 + posx + 1) - '0')) * 16; 
            temp1 = *(p1 + posx + 2) >= 'A' ? (*(p1 + posx + 2) - 55) : (*(p1 + posx + 2) - '0');       
            temp = temp + temp1;
        }
    }
    return temp;
}

double radian(double d)
{
    return d * M_PI / 180.0;
}

double getdistance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = radian(lat1);
    double radLat2 = radian(lat2);
    double a = radLat1 - radLat2;
    double b = radian(lng1) - radian(lng2);

    double dst = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));
    dst = dst * EARTH_RADIUS;
    return dst;
}

void NMEA_GNRMC_Analysis(gnrmc_type *gpsx, uint8_t *buf)
{
    uint8_t *p1, dx;
    uint8_t posx;
    uint32_t temp, temp1;
    double rs;

    p1 = (uint8_t *)str((char *)buf, "GNRMC"); 
    if(p1!=NULL)
    {
        posx = NMEA_Comma_Pos(p1, 1); 
        if (posx != 0XFF)
        {
            gpsx->utc.hour = (*(p1 + posx) - '0') * 10 + *(p1 + posx + 1) - '0';
            gpsx->utc.min = (*(p1 + posx + 2) - '0') * 10 + *(p1 + posx + 3) - '0';
            gpsx->utc.sec = (*(p1 + posx + 4) - '0') * 10 + *(p1 + posx + 5) - '0';
            gpsx->utc.pse = (*(p1 + posx + 7) - '0') * 10 + *(p1 + posx + 8) - '0';
        }

        posx = NMEA_Comma_Pos(p1, 2);
        if (posx != 0XFF)
        {
            gpsx->status = *(p1 + posx);
        }

        posx = NMEA_Comma_Pos(p1, 3); 
        if (posx != 0XFF)
        {
            NMEA_Str2num(p1 + posx, &dx, &rs);
            temp = rs;
            gpsx->latitude = temp / NMEA_Pow(10, dx + 2);                   
            rs = temp % NMEA_Pow(10, dx + 2);                               
            gpsx->latitude = gpsx->latitude + (rs / NMEA_Pow(10, dx)) / 60; 
        }
        posx = NMEA_Comma_Pos(p1, 4); 
        if (posx != 0XFF)
            gpsx->nshemi = *(p1 + posx);
        posx = NMEA_Comma_Pos(p1, 5); 
        if (posx != 0XFF)
        {
            NMEA_Str1num(p1 + posx, &dx, &temp, &temp1);
            rs = temp % NMEA_Pow(10, 2) + (double)temp1 / NMEA_Pow(10, dx); 
            temp = temp / NMEA_Pow(10, 2);                                  
            gpsx->longitude = temp + rs / 60;                               
        }
        posx = NMEA_Comma_Pos(p1, 6); 
        if (posx != 0XFF)
            gpsx->ewhemi = *(p1 + posx);

        posx = NMEA_Comma_Pos(p1, 7); 
        if (posx != 0XFF)
        {
            NMEA_Str2num(p1 + posx, &dx, &rs); 
            temp = rs;
            temp1 = temp / NMEA_Pow(10, dx);
            rs = temp % NMEA_Pow(10, dx);
            gpsx->speed = temp1 + rs / NMEA_Pow(10, dx);
        }
        posx = NMEA_Comma_Pos(p1, 8); 
        if (posx != 0XFF)
        {
            NMEA_Str2num(p1 + posx, &dx, &rs);
            temp = rs;
            gpsx->direction = temp / NMEA_Pow(10, dx); 
            rs = temp % NMEA_Pow(10, dx);              
            gpsx->direction = gpsx->direction + rs / NMEA_Pow(10, dx);
        }
        posx = NMEA_Comma_Pos(p1, 9); 
        if (posx != 0XFF)
        {
            gpsx->utc.date = (*(p1 + posx) - '0') * 10 + *(p1 + posx + 1) - '0';
            gpsx->utc.month = (*(p1 + posx + 2) - '0') * 10 + *(p1 + posx + 3) - '0';
            gpsx->utc.year = (*(p1 + posx + 4) - '0') * 10 + *(p1 + posx + 5) - '0' + 2000;
        }
        posx = NMEA_Comma_Pos(p1, 12); 
        if (posx != 0XFF)
        {
            gpsx->mode = *(p1 + posx);
            temp = (*(p1 + posx + 2) >= 'A' ? (*(p1 + posx + 2) - 55) : (*(p1 + posx + 2) - '0')) * 16; //У��ֵ
            temp1 = *(p1 + posx + 3) >= 'A' ? (*(p1 + posx + 3) - 55) : (*(p1 + posx + 3) - '0');       //У��ֵ
            gpsx->check = temp + temp1;
        }

        gpsx->valid = 1;
    }
}

void NMEA_GNGGA_Analysis(gngga_type *gpsx, uint8_t *buf)
{
    uint8_t *p1, dx;
    uint8_t posx;
    uint32_t temp, temp1;
    double rs;

    p1 = (uint8_t *)str((char *)buf, "GNGGA"); 
    if(p1!=NULL)
    {
        posx = NMEA_Comma_Pos(p1, 1); 
        if (posx != 0XFF)
        {
            gpsx->utc.hour = (*(p1 + posx) - '0') * 10 + *(p1 + posx + 1) - '0';
            gpsx->utc.min = (*(p1 + posx + 2) - '0') * 10 + *(p1 + posx + 3) - '0';
            gpsx->utc.sec = (*(p1 + posx + 4) - '0') * 10 + *(p1 + posx + 5) - '0';
            gpsx->utc.pse = (*(p1 + posx + 7) - '0') * 10 + *(p1 + posx + 8) - '0';
        }
        posx = NMEA_Comma_Pos(p1, 2); 
        if (posx != 0XFF)
        {
            //		NMEA_Str2num(p1 + posx, &dx, &rs);
            //		temp = rs;
            //		gpsx->latitude = temp / NMEA_Pow(10, dx + 2);	
            //		rs = temp % NMEA_Pow(10, dx + 2);				
            //		gpsx->latitude = gpsx->latitude + (rs / NMEA_Pow(10, dx)) / 60;
            rs = NMEAstr2Double(p1 + posx);
            gpsx->latitude = (int)(rs / 100) + (rs - (int)(rs / 100) * 100) / 60;
        }
        posx = NMEA_Comma_Pos(p1, 3); 
        if (posx != 0XFF)
            gpsx->nshemi = *(p1 + posx);
        posx = NMEA_Comma_Pos(p1, 4); 
        if (posx != 0XFF)
        {
            //		NMEA_Str1num(p1 + posx, &dx, &temp, &temp1);
            //		rs = temp % NMEA_Pow(10, 2) + (double)temp1 / NMEA_Pow(10, dx);				
            //		temp = temp / NMEA_Pow(10, 2);	
            //		gpsx->longitude = temp + rs / 60;
            rs = NMEAstr2Double(p1 + posx);
            gpsx->longitude = (int)(rs / 100) + (rs - (int)(rs / 100) * 100) / 60;
        }
        posx = NMEA_Comma_Pos(p1, 5); 
        if (posx != 0XFF)
            gpsx->ewhemi = *(p1 + posx);

        posx = NMEA_Comma_Pos(p1, 6); 
        if (posx != 0XFF)
            gpsx->fs = *(p1 + posx) - '0';

        posx = NMEA_Comma_Pos(p1, 7); 
        if (posx != 0XFF)
        {
            if (NMEA_Comma_Pos(p1, 8) - posx >= 2)
                gpsx->numSv = (*(p1 + posx) - '0') * 10 + *(p1 + posx + 1) - '0';
            else
                gpsx->numSv = *(p1 + posx) - '0';
        }

        posx = NMEA_Comma_Pos(p1, 8); 
        if (posx != 0XFF)
        {
            NMEA_Str2num(p1 + posx, &dx, &rs);
            temp = rs;
            temp1 = temp / NMEA_Pow(10, dx);
            rs = temp % NMEA_Pow(10, dx);
            gpsx->hdop = temp1 + rs / NMEA_Pow(10, dx);
        }

        posx = NMEA_Comma_Pos(p1, 9); 
        if (posx != 0XFF)
        {
            NMEA_Str2num(p1 + posx, &dx, &rs);
            temp = rs;
            temp1 = temp / NMEA_Pow(10, dx);
            rs = temp % NMEA_Pow(10, dx);
            gpsx->msl = temp1 + rs / NMEA_Pow(10, dx);
        }

        posx = NMEA_Comma_Pos(p1, 10); 
        if (posx != 0XFF)
            gpsx->uMsl = *(p1 + posx);

        posx = NMEA_Comma_Pos(p1, 11);
        if (posx != 0XFF)
        {
            NMEA_Str2num(p1 + posx, &dx, &rs);
            temp = rs;
            temp1 = temp / NMEA_Pow(10, dx);
            rs = temp % NMEA_Pow(10, dx);
            gpsx->sep = temp1 + rs / NMEA_Pow(10, dx);
        }

        posx = NMEA_Comma_Pos(p1, 12);
        if (posx != 0XFF)
            gpsx->uSep = *(p1 + posx);

        gpsx->valid = 1;
    }
}

void bv_rtk_handle(void)
{
	
    uint8_t *buf=(uint8_t*)"$GNRMC,081555.000,A,3112.663893,N,12135.692249,E,0.049,0.00,021121,,E,A,V*79";
    NMEA_GNRMC_Analysis(&gnrmc, buf);
    /*
    printf("GNRMC gpsx->utc.year =%d \r\n", gnrmc.utc.year );
    printf("GNRMC gpsx->utc.month =%d \r\n",gnrmc.utc.month );
    printf("GNRMC gpsx->utc.date =%d \r\n", gnrmc.utc.date );
    printf("GNRMC gpsx->utc.hour =%d \r\n", gnrmc.utc.hour );
    printf("GNRMC gpsx->utc.min =%d \r\n",  gnrmc.utc.min );
    printf("GNRMC gpsx->utc.sec =%d \r\n",  gnrmc.utc.sec );
    printf("GNRMC gpsx->utc.pse =%d \r\n",  gnrmc.utc.pse );

    printf("GNRMC gpsx->latitude =%.8f \r\n",gnrmc.latitude );
    printf("GNRMC gpsx->nshemi =%c \r\n",    gnrmc.nshemi );
    printf("GNRMC gpsx->longitude =%.8f\r\n",gnrmc.longitude );
    printf("GNRMC gpsx->ewhemi =%c \r\n",    gnrmc.ewhemi );

    printf("GNRMC gpsx->speed=%.8f \r\n",        gnrmc.speed );
    printf("GNRMC gpsx->direction=%.8f \r\n",    gnrmc.direction );
   */
		
}

void *bv_rtk_thread(void)
{
      
}


