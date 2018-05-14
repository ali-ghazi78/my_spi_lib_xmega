/*
 * MY_SPI.cpp
 *
 * Created: 5/14/2018 7:20:20 PM
 * Author : ali
 don't forget to make ss pin low when you start and at the end make it high 
 */ 


#ifndef MY_SPI_kk
	#include "avr/io.h"
	#define MY_SPI_kk



void spi_init(PORT_t *SPI_PORT,SPI_t *SPI,unsigned char SPI_MOSI_PIN_NUMBER,unsigned char SPI_MISO_PIN_NUMBER,unsigned char SPI_SCK_PIN_NUMBER,PORT_t *SPI_SS_PORT,unsigned char SPI_SS_PIN_NUMBER,unsigned char SPI_PRESCALER,unsigned char SPI_MODE)
{
	
											
	SPI_PORT->DIRSET=(1<<SPI_SCK_PIN_NUMBER)|(1<<SPI_MOSI_PIN_NUMBER);		//set MOSI and SCK and SS as output just set as output don't give any thing to out don't use port.out=1<<X
	SPI_SS_PORT->DIRSET=(1<<SPI_SS_PIN_NUMBER);									// the reason that i use another port typedef for SS is that you can use several spi slave 
	PORTA.DIRSET=1<<4;// THIS SHOULD BE ADDED OTHER WISE IT DOESN WORK
	PORTD.DIRSET=1<<4;
	my_putchar('\n');
	my_put_int(SPI_SS_PORT->DIRSET);
	my_putchar('\n');
	//
	SPI_PORT->DIRCLR=(1<<SPI_MISO_PIN_NUMBER);
	SPI->CTRL|=(SPI_ENABLE_bm)|(SPI_MASTER_bm)|(SPI_PRESCALER)|(SPI_MODE);//mode 1 is chosen	don't forget to set the correct mode for spi otherwise you will interfere with some data shifting and problems hajiii mode kheili moheme yaani nokte asasi entekhabe mode doroste ke sck ro kodom labe bayad puls befreste

	
}
void spi_write(SPI_t *SPI,unsigned char my_data,unsigned char data_order)
{
	//vase khoondane hatmn ye shero veri bayad befresti .aval command midi baad ye cherti mifresti va mikhooni .vase har bar khoondan bas befresti
	//data oreder==0---->msb first   | NOT SURE DATA OREDER 0 IS PREFERED
	//data oreder==1---->lsb first   | NOT SURE DATA OREDER 0 IS PREFERED
	//handy data order
	//DATASHEET NRF DAR MOREDE DATABIT COMMAND VA data ZER ZADE .HAR do TASHOON ye JOOR FERESTEDE MISHN LAZM nis avaz koni data order ro
	
	if(data_order)// i dont wanna use data order when using nrf
	{
		unsigned char temp=0;
		temp|=(((my_data&1)&&(1))<<7);
		temp|=(((my_data&2)&&(1))<<6);
		temp|=(((my_data&4)&&(1))<<5);
		temp|=(((my_data&8)&&(1))<<4);
		temp|=(((my_data&16)&&(1))<<3);
		temp|=(((my_data&32)&&(1))<<2);
		temp|=(((my_data&64)&&(1))<<1);
		temp|=(((my_data&128)&&(1))<<0);
		my_data=temp;
	}
	
	SPI->DATA=my_data;
	
	while(!(SPI->STATUS&(1<<7)));	//wait until sending is complete

}
unsigned char spi(SPI_t *SPI,unsigned char my_data,unsigned char data_order)
{
	spi_write(SPI,my_data,data_order);
	return SPI->DATA;
}
unsigned char spi_read(SPI_t *SPI)
{
	return SPI->DATA;
}
#endif




