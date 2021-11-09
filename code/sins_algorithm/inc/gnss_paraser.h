/**
 *  athur:  tony.jiang 
 *   date :2021 10 26
*/
#ifndef _BV_GNSS_PARASER_H_
#define _BV_GNSS_PARASER_H_

    typedef enum {
        GNRMC,
        GNGGA
    } NMEA;
typedef struct
{
		uint16_t year;	
		uint8_t  month;	
		uint8_t  date;	
		uint8_t  hour; 	
		uint8_t  min; 	
		uint8_t  sec; 	
		uint8_t  pse;
}nmea_utc_time;
//<<user data
typedef struct
{
		double    lat;		
		char      ns;						  
		double    lon;		
		char      ew;	//east west	
		float     x;        
		float     y;        
		uint8_t   state;       
		uint16_t  time_out;    
		float     distance;   
}Nmea_Typedef;

//NMEA 0183 Protocol parsing data
	typedef struct
	{
		nmea_utc_time  utc;	
		char           status;        
		double         latitude;		
		char           nshemi;								  
		double         longitude;		
		char           ewhemi;			  		  
		double         speed; //speed      
		double         direction;  //地面航向 
		char           mode;          
		uint8_t        check;           
		Nmea_Typedef   user;  

		uint8_t        updateHead;
		
		uint8_t         valid;      
	}gnrmc_type;
    typedef struct
	{
		nmea_utc_time utc;	//UTC TIME
		double latitude;		
		char nshemi;				//north 			  
		double longitude;	
		char ewhemi;			  
		char fs;            //ָquality
		char numSv;//卫星数         
		float hdop;        
		float msl; // 海拔高度         
		char uMsl; // 大地水准面高度      
		float sep;  //差分时间         
		char  uSep;  //差分参考基站标号      

		uint8_t check;           
		uint8_t valid;      
	}gngga_type;

    uint8_t NMEA_Comma_Pos(uint8_t *buf, uint8_t cx);
    uint32_t NMEA_Pow(uint8_t m, uint8_t n);

	void NMEA_Str2num(uint8_t *buf, uint8_t*dx, double * rs);
	void NMEA_Str1num(uint8_t *buf, uint8_t*dx, uint32_t *inte, uint32_t *deci);
    double NMEAstr2Double(uint8_t *buf);

    double radian(double d) ;
    double getdistance(double lat1, double lng1, double lat2, double lng2);

    char *str(const char *str1, const char *str2);

    uint8_t GPS_Check(uint8_t *buf);	
	uint8_t GPS_verify(uint8_t *buf, NMEA type); 
	void bv_rtk_handle(void);
	void NMEA_GNRMC_Analysis(gnrmc_type *gpsx, uint8_t *buf);
	void NMEA_GNGGA_Analysis(gngga_type *gpsx, uint8_t *buf);

#endif 