

enum {TEMP,HUMI};
/*#define DATA P0_5
#define SCK  P0_4*/
#define DATA P6_5
#define SCK  P6_4
//#define PWR  P0_2
/*************** OnBoard RH Sensor I/O **********/
//#define DATA P0_2
//#define SCK  P0_3
/***********************************************/
#define noACK 0
#define ACK 1
//adr command r/w
#define STATUS_REG_W 0x06 //000 0011 0
#define STATUS_REG_R 0x07 //000 0011 1
#define MEASURE_TEMP 0x03 //000 0001 1
#define MEASURE_HUMI 0x05 //000 0010 1
#define RESET 0x1E //000 1111 0

#define DATA_HI()          \
  { P6DIR &= 0XDF;         \
  }  

#define DATA_LO()          \
  { P6OUT &= 0xDF;              \
    P6DIR |= 0X20;         \
  }  

#define SCK_HI()          \
  { P6OUT |= 0x10;                \
    P6DIR |= 0X10;        \
  }   

#define SCK_LO()           \
  { P6OUT &= 0xEF;                 \
    P6DIR |= 0X10;         \
  }  

enum
{
	ADC_TEMP_ERROR = 0x02,
	ADC_HUMI_ERROR = 0x03
};

typedef union {
	uint8_t i;
	float f;
} value;