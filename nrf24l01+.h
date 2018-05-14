/*
 * nrf24l01_.h
 *
 * Created: 5/15/2018 2:08:03 AM
 *  Author: ali
 */ 


#ifndef NRF24L01_H_
#define NRF24L01_H_
#include "avr/io.h"


#define SPI (SPID)
#define SPI_PORT (PORTD)
#define SPI_SCK  (1<<7)
#define SPI_MOSI (1<<5)
#define SPI_MISO (1<<6)
#define SPI_SS (1<<4)
#define SPI_SS_R (1<<4)//d4
#define SPI_SS_T (1<<4)//a4


#define SS_LOW (PORTD.OUTCLR=SPI_SS)
#define SS_HIGH (PORTD.OUTSET=SPI_SS)

#define SS_R_LOW (PORTD.OUTCLR=SPI_SS_R)
#define SS_R_HIGH (SPI_PORT.OUTSET=SPI_SS_R)

#define SS_T_LOW (PORTA.OUTCLR=SPI_SS_T)
#define SS_T_HIGH (PORTA.OUTSET=SPI_SS_T)

#define R (ss=1)
#define T (ss=0)

#define CE_R_LOW (PORTC.OUTCLR=1<<0)//C0
#define CE_R_HIGH (PORTC.OUTSET=1<<0)//C0

#define CE_T_LOW (PORTC.OUTCLR=1<<1)//C1
#define CE_T_HIGH (PORTC.OUTSET=1<<1)//C1

unsigned char nrf_read_single_register(unsigned char nrf_register);
void nrf_write_single_register(unsigned char nrf_register,unsigned char value);
void nrf_read_multiple_register(unsigned char nrf_register,unsigned char * saved_value,unsigned char number_of_data);
void nrf_write_multiple_register(unsigned char nrf_register,unsigned char *value,unsigned char number_of_value);


  volatile unsigned char ss=1;

void nrf_load_data_to_transmit(unsigned char *data,unsigned char number_of_bytes)
{
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	
	spi_write(&SPI,0xe1,0);//flush tx fifo

	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	spi(&SPI,0xA0,0);//0xA0->W_TX_PAYLOAD
	for(int i=0;i<number_of_bytes;i++)
	{
		spi(&SPI,data[i],0);
	}
	
	
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	CE_T_HIGH;
	_delay_ms(1);
	CE_T_LOW;
	
	
	while(!(nrf_read_single_register(0x07)&(1<<5)));//wait until to trasmit
	nrf_write_single_register(0x07,1<<5);//clear tx flag
	

}
void nrf_read_data_from_receiver(unsigned char *data,unsigned char number_of_bytes)
{
	R;
	
	
	while(!(nrf_read_single_register(0x07)&(1<<6)));//wait until data come

	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;

	spi(&SPI,0x61,0);//0x61->R_RX_PAYLOAD
	for(int i=0;i<number_of_bytes;i++)
	{
		data[i]=spi(&SPI,0x61,0);//ye cherti mifresti serfan vase inke bekhooni ye chi
	}
	
	nrf_write_single_register(0x07,1<<6);//clear Rx flag

	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
}





unsigned char nrf_read_single_register(unsigned char nrf_register)
{
	unsigned char data=0;
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	spi(&SPI,nrf_register,0);
	data=spi(&SPI,0x01,0);				//just send sth no matter what it is
	_delay_us(1);
	
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	return data;
}
void nrf_read_multiple_register(unsigned char nrf_register,unsigned char * saved_value,unsigned char number_of_data)
{
	//the maximum nrf data is 5 byte
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	
	
	spi(&SPI,nrf_register,0);
	for(int i=0;i<number_of_data;i++)
	{
		saved_value[i]=spi(&SPI,0x01,0);				//just send sth no matter what it is
		_delay_us(1);
	}
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	//SS_HIGH; //because i wanna use 2 spi
}
void nrf_write_single_register(unsigned char nrf_register,unsigned char value)
{
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	spi(&SPI,nrf_register|1<<5,0);
	spi(&SPI,value,0);
	
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	
	//SS_HIGH;
}
void nrf_write_multiple_register(unsigned char nrf_register,unsigned char *value,unsigned char number_of_value)
{
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	
	spi(&SPI,nrf_register|1<<5,0);
	for(int i=0;i<number_of_value;i++)
	{
		spi(&SPI,value[i],0);
	}
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	
	//SS_HIGH;
}
void nrf_init_receiver()
{
	R;//ss=1;
	
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	
	spi_write(&SPI,0xe2,0);//flush rx fifo
	
	
	
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	nrf_write_single_register(0x00,(1<<0)|(1<<1)|(1<<6)|(1<<5)|(1<<4)|(1<<3));//receiver->1    transmitter->0 disable irq pin(1<<6)|(1<<5)|(1<<4)     enacle crc(1<<3)
	
	nrf_write_single_register(0x01,0x00);	//disable auto-ack on any pipe
	nrf_write_single_register(0x02,0x01);	//just enable pipe 0
	nrf_write_single_register(0x03,3);		//5 byte address
	nrf_write_single_register(0x04,0x00);	//re-transmit disable
	nrf_write_single_register(0x05,10);		//set rf cahnnle radio frequency
	//0x06 i personally dont change it .don't touch lna and pll_lock never havaset bashe hatmn ba meqdare default | beshe ya beshe or beshe
	nrf_write_single_register(0x11,5);		//we want 3 byte for pipe 0--   register 11to 16 are rx data width if u fill with zero it doesnt work bas ye chizi qeire sefr benevisi toosh vagarna tamoome karet .vase har kodom ke mikhay rah bendazi albate
	
}
void nrf_init_transmitter()
{
	T;//ss=0;
	
	if(ss==1)
	SS_LOW;
	else if(ss==0)
	SS_T_LOW;
	
	spi_write(&SPI,0xe1,0);//flush tx fifo
	
	
	
	if(ss==1)
	SS_HIGH;
	else if(ss==0)
	SS_T_HIGH;
	
	
	
	nrf_write_single_register(0x00,(1<<1)|(1<<6)|(1<<5)|(1<<4)|(1<<3));//receiver->1    transmitter->0 disable irq pin(1<<6)|(1<<5)|(1<<4)     enacle crc(1<<3)
	
	nrf_write_single_register(0x01,0x00);	//disable auto-ack on any pipe
	nrf_write_single_register(0x02,0x01);	//just enable pipe 0
	nrf_write_single_register(0x03,3);		//5 byte address
	nrf_write_single_register(0x04,0x00);	//re-transmit disable
	nrf_write_single_register(0x05,10);		//set rf cahnnle radio frequency
	//0x06 i personally dont change it .don't touch lna and pll_lock never havaset bashe hatmn ba meqdare default | beshe ya beshe or beshe
	nrf_write_single_register(0x11,5);		//we want 3 byte for pipe 0--   register 11to 16 are rx data width if u fill with zero it doesnt work bas ye chizi qeire sefr benevisi toosh vagarna tamoome karet .vase har kodom ke mikhay rah bendazi albate
	
}








#endif /* NRF24L01+_H_ */