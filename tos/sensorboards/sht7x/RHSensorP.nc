#include "Data.h"


module RHSensorP
{
	provides interface RHSensor;
	uses interface Leds;
//	uses interface StdOut;
}

implementation
{
	uint16_t humi_val,temp_val,temp ;
	uint8_t error,checksum, send='0';
	uint32_t humi_temp =0;

	command void RHSensor.RH_Init()
	{

		P6SEL |= 0x30; //	P0_ALT |= 0x00;                       // selecing all ports as general purpose i/o ports
		P6DIR |= 0x30; //	P0_DIR |= 0x0C;                       // p1 as o/p port
	//	P0_3 = 0;
	//	P0_2 = 0;
	//	P6OUT &= 0xDF;  //To power off IR and smoke sensors
	//	PERCFG |= 0x00;
	
	}

	void delay_5_us()
	{
	/*	T1CNTL&=0x00;         
		T1CTL|=0XE0;           
		T1CC0H=0x00;          
		T1CC0L=0x06; 
		T1CTL|=0X02;          
		while (!(T1CTL&0x10));
		T1CTL &= 0x00;        */
		
	//	while(1)
	//	{
		TACTL = 0x0;
		TACTL |= TACLR;
		BCSCTL1 |= 0x84;
		DCOCTL |= 0x02;
		BCSCTL2 = 0;
		
		CCTL0 = CCIE;                          
		CCR0 = 11;
	        TACTL = TASSEL_1 + MC_1;
		
		while ((TACTL & 0x01) == 0);
	
	 	
		TACTL &= ~(MC_1);
		TACCTL0 &= ~CCIFG;
		 
	//	}

	}


	uint8_t s_read_byte(uint8_t ack)
	{
		uint8_t i,val=0;
		DATA_HI(); 
		for (i=0x80;i>0;i/=2) //shift bit for masking
		  { P6OUT |= 0x10; //SCK=1; //clk for SENSI-BUS
		    delay_5_us(); //pulswith approx. 5 us
		//    if (P6OUT & 0x20) val=(val | i); //read bit
			if (P6IN & 0x20) val=(val | i);
		    P6OUT &= 0xEF; //SCK=0;
		    delay_5_us(); //pulswith approx. 5 us
		  }
		if (ack) {DATA_LO();} //in case of "ack==1" pull down DATA-Line
		else  {DATA_HI();}
		P6OUT |= 0x10; //SCK=1; //clk #9 for ack
		delay_5_us(); //pulswith approx. 5 us
		P6OUT &= 0xEF;//SCK=0;
		DATA_HI(); //release DATA-line
		delay_5_us(); //pulswith approx. 5 us
		return val;
	}

	uint8_t s_write_byte(uint8_t val)
	{
		uint8_t i,error_w=0;
		for (i=0x80;i>0;i/=2) //shift bit for masking
		{ 
		     if (i & val) //masking value with i , write to SENSI-BUS
		     { DATA_HI(); }
		     else {DATA_LO()};
		     P6OUT |= 0x10;//SCK=1; //clk for SENSI-BUS
		     delay_5_us(); //pulswith approx. 5 us
		     P6OUT &= 0xEF;//SCK=0;
		     delay_5_us(); //pulswith approx. 5 us
		}
		DATA_HI(); //release DATA-line
		P6OUT |= 0x10;//SCK=1; //clk #9 for ack
		delay_5_us(); //pulswith approx. 5 us
	//	error_w= (P6OUT&0x20)>>5;//DATA; //check ack (DATA will be pulled down by SHT11)
		error_w= (P6IN&0x20)>>5;
	/*	if((P6OUT & BIT5) == 0)
			error_w = 0;
		else 
			error_w = 1; */
		P6OUT &= 0xEF;//SCK=0;
		return error_w; //error=1 in case of no acknowledge
	}

	void s_transstart()
	{
		DATA_HI(); SCK_LO(); 
		delay_5_us();
		SCK_HI();
		delay_5_us();
		DATA_LO();
		delay_5_us();
		SCK_LO();
		delay_5_us();
		SCK_HI();
		delay_5_us();
		DATA_HI();
		delay_5_us();
		SCK_LO();
	}



	uint8_t s_measure(uint8_t *p_value, uint8_t *p_checksum, uint8_t mode)

	{
		uint8_t error_m=0;
		s_transstart(); //transmission start
		switch(mode){ //send command to sensor
		   case TEMP : error_m+=s_write_byte(MEASURE_TEMP); break;
		   case HUMI : error_m+=s_write_byte(MEASURE_HUMI); break;
		   default : break;
		}

		DATA_HI();     // wait for DATA READY Sensors pulls data line low when data ready.
	//	while((P6OUT&0x20)>>5);  
		while((P6IN&0x20)>>5);
		    
		*(p_value) =s_read_byte(ACK); //read the first byte (MSB)
		*(p_value+1)=s_read_byte(ACK); //read the second byte (LSB)
		*p_checksum =s_read_byte(noACK); //read checksum
		return error_m;
	}



	void s_connectionreset()
	{
		uint8_t i;
		DATA_HI(); 
		SCK_LO(); //Initial state
		for(i=0;i<9;i++) //9 SCK cycles
		{ SCK_HI();
		  delay_5_us();
		  SCK_LO();
		  delay_5_us();
		}
		s_transstart(); //transmission start
		
	}

	
	command uint16_t RHSensor.readTemp()
	{
	//	delay_5_us();
		
		s_connectionreset();
		   
		error=0;
		error+=s_measure((uint8_t*) &temp_val,&checksum,TEMP); //measure temperature
		if(error!=0)
		{
		   s_connectionreset();
		   signal RHSensor.readTempDone(ADC_TEMP_ERROR, temp_val);
		   
		}
		else 
		{
		//   temp_val.f=(float)temp_val.i; //converts integer to float
		//   return temp_val ;
		call Leds.led2Toggle();
		signal RHSensor.readTempDone(SUCCESS,temp_val);
	//	   calc_sth11T(&temp_val.f); //calculate temperature
		}
	}

	command uint16_t RHSensor.readHumidity()
	{
		s_connectionreset();
  
		error=0;
		error+=s_measure((uint8_t*) &humi_val,&checksum,HUMI); //measure Humidity
		error+=s_measure((uint8_t*) &temp_val,&checksum,TEMP); //measure temperature
		if(error!=0)
		{
		   s_connectionreset();
		   signal RHSensor.readHumidityDone(ADC_HUMI_ERROR,humi_temp);
		}
		else 
		{
		  //  humi_val.f=(float)humi_val.i; //converts integer to float
		  //  temp_val.f=(float)temp_val.i; //converts integer to float
	//	  temp = humi_val >> 8;
	//	  humi_val <<= 8;
	//	  humi_val |= temp;
	//	  humi_temp = humi_val;
	//	  humi_temp <<= 16;
	//	  humi_temp |= temp_val;
		//    return humi_temp;
		signal RHSensor.readHumidityDone(SUCCESS,humi_val);
		//    calc_sth11H(&humi_val.f,&temp_val.f); //calculate humidity, temperature
		}
	}

	
/*	async event void StdOut.get(uint8_t v )
	{
	}	*/

}


		

	
