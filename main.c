/*
 * myFirstXmega.c
 *
 * Created: 1/24/2018 8:46:48 PM
 * Author : ali
 */ 
#define F_CPU 8000000
#define F_SYS 32000000
#define F_TWI 100000
#include "MY_SPI.h"
#include "util/delay.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "nrf24l01+.h"

#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)
#define TWI_BAUDRATE TWI_BAUD(F_SYS, F_TWI)


#define TWI_ACK		0x00U
#define TWI_NACK	0x01U

#define ENTER (my_putchar('\n'))

#define PWM1 PIN0_bm
#define PWM2 PIN1_bm


#define LED_DOWN	PIN4_bm
#define LED_UP		PIN6_bm
#define LED_RIGHT	PIN7_bm
#define LED_LEFT	PIN5_bm
#define SCL_HIGH PORTE.OUTSET=2
#define SCL_LOW PORTE.OUTSET=2

#define SDA_HIGH PORTE.OUTSET=1
#define SDA_LOW PORTE.OUTCLR=1

#define KEY_UP		(PORTD.IN&PIN2_bm)
#define KEY_DOWN	(PORTD.IN&PIN1_bm)
#define KEY_LEFT	(PORTD.IN&PIN0_bm)
#define KEY_RIGHT	(PORTD.IN&PIN3_bm)

#define LED_DOWN_ON		(PORTC.OUTSET=LED_DOWN) 
#define LED_UP_ON		(PORTC.OUTSET=LED_UP)
#define LED_RIGHT_ON	(PORTC.OUTSET=LED_RIGHT)
#define LED_LEFT_ON		(PORTC.OUTSET=LED_LEFT)

#define LED_DOWN_OFF	(PORTC.OUTCLR=LED_DOWN)
#define LED_UP_OFF		(PORTC.OUTCLR=LED_UP)
#define LED_RIGHT_OFF	(PORTC.OUTCLR=LED_RIGHT)
#define LED_LEFT_OFF	(PORTC.OUTCLR=LED_LEFT)

#define MPU_ADDRESS 0x68
#define MPU_WHO_AM 0x75
#define GYRO_CONFIG 0x1B
#define SLEEP_CONFIG 0x6B
#define SAMPLE_RATE 0x19
#define USER_CONTROL 0x6A

#define SPI (SPID)
#define SPI_PORT (PORTD)
#define SPI_SCK  (1<<7)
#define SPI_MOSI (1<<5)
#define SPI_MISO (1<<6)
#define SPI_SS (1<<4)
#define SPI_SS_R (1<<4)//d4
#define SPI_SS_T (1<<4)//a4











void oled_write(unsigned char a);
void cmp_calibrate();
void test_led(void);
void test_key(void);
void init_pin(void);
void init_clock(void);
void my_putstr(char *p);
void my_putchar(unsigned char input_char);
void init_usart();
void init_usart_interupt();
void twi_intit(TWI_t *port);
void my_usart_asci();
void my_put_int( int number);
unsigned char i2c_start();
void i2c_stop();
void i2c_init();
unsigned char i2c_start_write(unsigned char address_of_module);
unsigned char i2c_write(unsigned char data);
unsigned char i2c_read(unsigned char _ack_1_nack_zero);
unsigned char my_getchar(void);
void pwm_init();


volatile unsigned char output_char='a',is_ready_data=0;//global variable should be volatile
/* 
 void TWI_Start(TWI_t *twi)
 {

	 TWI = twi;
	 IsStart = 1;
 }

 void TWI_Restart(void)
 {	// Issue a repeated START
	 IsStart = 1;
 }

 void TWI_Stop(uint8_t nack)
 {
	 
	 // Issue a STOP
	 if (nack)
	 {
		 TWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
	 }
	 else
	 {
		 TWI->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	 }
 }
 uint8_t TWI_WriteByte(uint8_t data)
 {	// If this is the first byte,
	 if (IsStart)
	 {	// Clear the flag and write to ADDR
		 IsStart = 0;
		 TWI->MASTER.ADDR = data;	// Doesn't necessarily need to be the address,
	 }								//   but this will generate a START condition.
	 else
	 {	// Otherwise, just write to DATA
		 TWI->MASTER.DATA = data;
	 }
	 // Wait until the byte is shifted out
	while (!(TWI->MASTER.STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_RIF_bm| TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
	{


	}
	LED_UP_OFF;
	 // Return error status if we get NACK'ed, if arbitration is lost, or if there is a general bus error
	 if ((TWI->MASTER.STATUS & (TWI_MASTER_RXACK_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
	 {
		 return TWI_ACK;//0
	 }
	 else
	 {
		 return TWI_NACK;//1
	 }
 }

 uint8_t TWI_WriteBytes(uint8_t *data, uint8_t length)
 {	// Sanity check
	 if (!data) return 0;
	 // Track number of bytes
	 uint8_t count = 0;
	 // Transmit data
	 for (; count < length; count++)
	 {	// If we get NACK'ed, let the caller know
		 if (!TWI_WriteByte(data[count])) return count;
	 }
	 // Finally, return the number of bytes written
	 return count;
 }
 n
 uint8_t TWI_ReadByte(uint8_t nack)
 {
	 // Wait until the byte is shifted in
	while (!(TWI->MASTER.STATUS & (  TWI_MASTER_RIF_bm| TWI_MASTER_WIF_bm| TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
	 {
	 }
	 LED_DOWN_OFF;
	 // Initiate a read
	 uint8_t retval = TWI->MASTER.DATA;

	 if (nack == 0)
	 {	// ACK and receive a byte
		 TWI->MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
	 }

	 // Return the read value
	 return retval;
 }

 uint8_t TWI_ReadBytes(uint8_t *data, uint8_t length)
 {	// Sanity check
	 if (!data) return 1;
	 // Offset length
	 length--;
	 // Read bytes
	 for (uint8_t i = 0; i < length; i++)
	 {
		 data[i] = TWI_ReadByte(TWI_ACK);
	 }
	 // Read last byte with NACK
	 data[length] = TWI_ReadByte(TWI_NACK);
	 // Return success
	 return 0;
 }
 
 
 
*/
ISR(USARTC0_RXC_vect)
{
	LED_UP_ON;
	extern volatile unsigned char output_char;
	output_char=USARTC0.DATA;						//LOAD DATA INTO REG
	while((USARTC0.STATUS&USART_RXCIF_bm));		//IS READ COMPLETE
	is_ready_data=1;
	LED_UP_OFF;
}
int main(void)
{
	init_usart_interupt();
	init_pin();
	init_clock();
	test_led();
	
	my_putchar('\n');	
	my_putstr("----------");
	my_putstr("starT");
	my_putstr("----------");
	my_putchar('\n');
	_delay_ms(500);
		
	PORTC.DIRSET=1<<1;//FOR TRANSMITTER CE
	PORTC.DIRSET=1<<0;//FOR Reciever CE
	
	spi_init(&SPI_PORT,&SPID,5,6,7,&PORTA,4,SPI_PRESCALER_DIV64_gc,1);
	spi_init(&SPI_PORT,&SPID,5,6,7,&PORTD,4,SPI_PRESCALER_DIV64_gc,1);
	PORTA.DIRSET=1<<4;
	PORTD.DIRSET=1<<4;//IN 2 TA BAS BASHE VAGARNA KAR NEMDIE
	
	
	LED_RIGHT_ON;
	T;
	nrf_init_transmitter();
	R;
	LED_RIGHT_OFF;
	nrf_init_receiver();

	
	CE_R_HIGH;
	CE_T_LOW;
	unsigned char output_data[5];
	output_data[0]=0;
	output_data[1]=1;
	output_data[2]=2;
	output_data[3]=3;
	output_data[4]=4;
	
	unsigned char input_data[5];
	
	while (1) 
    {
		LED_UP_ON;
		LED_DOWN_OFF;

		output_data[0]++;
		if(output_data[0]==250)
			output_data[0]=0;
		T;
		nrf_load_data_to_transmit(output_data,5);
		LED_DOWN_ON;
		LED_UP_OFF;
		R;
		nrf_read_data_from_receiver(input_data,5);
		
		my_put_int(input_data[0]);
		ENTER;
		_delay_ms(100);
		
		
		
		
	}
}






void pwm_init()
{
	PORTE.DIRSET=PWM1;
	
						//havaset bashe manbaa frekanse frec peripheral haST NA CPU .CPU 32, PERIPHERAL 8
							//DONT FORGET TO SET THE PIN AS OUTPUT FOR PWM 	PORTE.DIRSET=PWM1;
						//TCE-TCB-TCA-FOR SELECTING witch port u need
	TCE0.CTRLA=0x00;	//TIMER HAS STOPED AND HAVE NO CLOCK	 
	TCE0.CTRLA=3;		//INPUT CLOCK(8MHZ)/4=2mHZ->3
	TCE0.CTRLB=1<<4;	//ENABLE OC0A->TCC0 MAKES OC0 AND OC0(A) THE A IN MADE HERE
	TCE0.CTRLB|=3;		//PWM_SINGLE SLOPE MODE
						//PERIOD SHOULD BE 13 MAX AND TOP SHOUD BE AROUND 6 FOR 38KHZ WITH 50 PERCANT DUCTY CYCLE
	TCE0.PERL=52;		//SET THE MAX TO 13 FOR 38KHZ
	TCE0.PERH=0;
	
	TCE0.CCABUFL=27;		//SET TOP TO 6 FOR 50% DUTY CYCLE
	TCE0.CCABUFH=0;
	
	
	
	
}
void oled_write(unsigned char a)
{
	
	i2c_start_write(0x78);
	i2c_write(0);
	i2c_write(a);
	i2c_stop();	
	
}
void init_pin(void)
{
	PORTC.DIRSET=LED_RIGHT;
	PORTC.DIRSET=LED_LEFT;
	PORTC.DIRSET=LED_UP;
	PORTC.DIRSET=LED_DOWN;
	
	PORTD.DIRCLR=KEY_UP;
	PORTD.DIRCLR=KEY_DOWN;
	PORTD.DIRCLR=KEY_LEFT;
	PORTD.DIRCLR=KEY_RIGHT;
}
void test_led(void)
{
	for(int i=0;i<4;i++)
	{
		
		if(i==0)
		{
			PORTC.OUTTGL=LED_UP;
		}
		if(i==1)
		{	
			PORTC.OUTTGL=LED_RIGHT;
		}
		if(i==2)
		{	
			PORTC.OUTTGL=LED_DOWN;
		}
		if(i==3)
		{	
			PORTC.OUTTGL=LED_LEFT;
		}
		_delay_ms(50);
		
	}
	LED_LEFT_OFF;
	LED_RIGHT_OFF;
	LED_DOWN_OFF;
	LED_UP_OFF;
	_delay_ms(25);
	LED_RIGHT_ON;
	LED_UP_ON;
	LED_LEFT_ON;
	LED_DOWN_ON;
	_delay_ms(25);
	LED_LEFT_OFF;
	LED_RIGHT_OFF;
	LED_DOWN_OFF;
	LED_UP_OFF;
	_delay_ms(25);
	LED_RIGHT_ON;
	LED_UP_ON;
	LED_LEFT_ON;
	LED_DOWN_ON;
	_delay_ms(25);
	LED_LEFT_OFF;
	LED_RIGHT_OFF;
	LED_DOWN_OFF;
	LED_UP_OFF;
	_delay_ms(25);
	

}
void test_key(void)
{
	if(!(KEY_DOWN))
		LED_DOWN_ON;
	else
		LED_DOWN_OFF;
	
	if(!(KEY_UP))
		LED_UP_ON;
	else
		LED_UP_OFF;
	
	if(!(KEY_RIGHT))
		LED_RIGHT_ON;
	else
		LED_RIGHT_OFF;
	
	if(!(KEY_LEFT))
		LED_LEFT_ON;
	else
		LED_LEFT_OFF;
}
void init_clock(void)//SET 32MHZ
{
    CCP = CCP_IOREG_gc;             // disable register security for oscillator update
	OSC.CTRL|=1<<1;					//enable 32MHZ ."|" MIKONIM CHON 2MHZ HAM HAST
	while(!(OSC.STATUS&(1<<1)));	//wait for stablize
    CCP = CCP_IOREG_gc;             // disable register security for oscillator update
	CLK.CTRL=1;						//SELECT 32MHZ
	CCP = CCP_IOREG_gc;             // disable register security for oscillator update
	CLK.PSCTRL=CLK_PSBCDIV_4_1_gc;	//DIV FOR PERIPHERAL 4
	//YADET NARE CCP .KOLN CCP BAYAD BEZARI QABL AZ HAE KODOM TA BEHET EJEZA DASTRESI VA NEVESHTN BEDE VASE STATUE LAZEM NIS
}
void init_usart()
{
	PORTC.DIRSET=PIN3_bm;							//DON'T FORGET TO DO THIS OTHERWISE IT WON'T WORK
	PORTC.OUTSET=PIN3_bm;
	USARTC0.CTRLC=((1<<1)|(1));						//8 BIT NO PARITY 1 STOP BIT
	USARTC0.BAUDCTRLA=428%256;						//BAUD RATE CONTROLL 115200  BSELL
	USARTC0.BAUDCTRLB=((-7)<<4)|((int)(428/256));	//BAUD RATE CONTROLL 115200 -3=0B1101
	USARTC0.CTRLB=USART_TXEN_bm|USART_RXEN_bm;		//ENABLE TX AND RX AT FIRST ALL IS WHEN U FULL THE DATA BUFFER
	PORTC.DIRCLR=PIN2_bm;							//MAKE RX INPUT
}
void init_usart_interupt(void)
{
	init_usart();
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm; //programmable mask interupt control enable all level of interupt
	
	SREG=1<<7;////==sei() enable global interupt
	USARTC0.CTRLA=0b00110000;								//enable rx interupt with high peroirity
}
void my_usart_asci()
{
	my_putstr("----------asci table----------");
	for(unsigned int i=0;i<=254;i++)
	{
		my_put_int(i);
		my_putchar('\t');
		my_putchar(i);
		my_putchar('\n');
	}
	my_putstr("----------asci table----------");
}
void my_put_int(int number)
{
	 int counter=0;
	 if(number<0)
	 {
		my_putchar('-');
		number*=-1;
	 }
	 int temp=number;
	 while (temp)
	 {
		 counter++;
		 temp/=10;
	 }
	 temp=number;
	 while(counter&&counter!=1)
	 {
		my_putchar(number/pow(10,counter-1)+'0');
		if(counter>2)
			number%=(int)pow(10,counter-1);
		else
			break;
		counter--;
	 } 
	 my_putchar(temp%10+'0');
	 
}
void my_putchar(unsigned char input_char)
{
	while(!(USARTC0.STATUS&USART_DREIF_bm));		//HASE PREVOIUSE DATA SENT? data regisert is empty 1 yes 0 is full
	USARTC0.DATA=input_char;						//LOAD DATA INTO REG
	while(!(USARTC0.STATUS&USART_DREIF_bm));		//HASE PREVOIUSE DATA SENT? data regisert is empty 1 yes 0 is full
}
void my_putstr(char *p)
{
	for(int i=0;p[i]!='\0';i++)
	{
		my_putchar(p[i]);
	}
}
unsigned char my_getchar(void)
{
	unsigned char output_char;
	while(!(USARTC0.STATUS&USART_RXCIF_bm));		//HASE  DATA received? data regisert is empty 1 yes 0 is full
	output_char=USARTC0.DATA;						//LOAD DATA INTO REG
	return output_char;
}
void i2c_init()
{
	TWIE.MASTER.CTRLA=TWI_MASTER_ENABLE_bm;
	TWIE.MASTER.CTRLB=0x00;				//timout no
	TWIE.MASTER.STATUS=1;				//idle mode
	TWIE.MASTER.BAUD=155;			//100khz
}
unsigned char i2c_start_write(unsigned char address_of_module)
{	
	TWIE.MASTER.ADDR=address_of_module;
	while(!(TWIE.MASTER.STATUS&(TWI_MASTER_RIF_bm|TWI_MASTER_WIF_bm|TWI_MASTER_BUSERR_bm|TWI_MASTER_ARBLOST_bm)));//u can add clock hold later	
	if(!(TWIE.MASTER.STATUS&TWI_MASTER_RXACK_bm))//RXACK--> 0 ACK -------RXAKC ----->1 NACK 
	{
		return 0;//0 MEANS ACK 
	}
	else
		return 1;
}
unsigned char i2c_start()
{
	TWIE.MASTER.CTRLC=1<<2;	//start with no ack if y don't set this 1 bit will transmite by master unneccerily //bayad in karo hatmn bokoni
	TWIE.MASTER.CTRLC|=1;//start
	
	if(!(TWIE.MASTER.STATUS&TWI_MASTER_RXACK_bm))//RXACK--> 0 ACK -------RXAKC ----->1 NACK
	{
		return 0;//0 MEANS ACK
	}
	else
		return 1;

}
unsigned char i2c_write(unsigned char data)
{
	
	TWIE.MASTER.DATA=data;
	while(!((TWI_MASTER_WIF_bm|TWI_MASTER_BUSERR_bm|TWI_MASTER_ARBLOST_bm|TWI_MASTER_RXACK_bm)));//u can add clock hold later
	if(!(TWIE.MASTER.STATUS&TWI_MASTER_RXACK_bm))//RXACK--> 0 ACK -------RXAKC ----->1 NACK
	{
		return 0;//0 MEANS ACK
	}
	else
		return 1;
}
unsigned char i2c_read(unsigned char _ack_1_nack_zero)
{
	if(_ack_1_nack_zero)//yaani ack mikhad
		TWIE.MASTER.CTRLC=0;//nack we want
	else//yaani nack mikhad
		TWIE.MASTER.CTRLC=1<<2;	
	TWIE.MASTER.CTRLB=1;// enable smart mode

	unsigned char data=255;
	while(!(TWIE.MASTER.STATUS&(TWI_MASTER_RIF_bm|TWI_MASTER_BUSERR_bm|TWI_MASTER_ARBLOST_bm)))//u can add clock hold later
	{
			LED_LEFT_ON;
	}
	LED_LEFT_OFF;
	if(TWIE.MASTER.STATUS&(TWI_MASTER_BUSERR_bm|TWI_MASTER_ARBLOST_bm))
		LED_LEFT_ON;
	if(TWI_MASTER_RIF_bm&TWIE.MASTER.STATUS)
		data=TWIE.MASTER.DATA;

	return data;//0 MEANS ACK
}
void i2c_stop()
{
	TWIE.MASTER.CTRLC=1<<2;//hatmn bayad nack ro faal koni va ack ro qeire faal 
	TWIE.MASTER.CTRLC |=  TWI_MASTER_CMD_STOP_gc; //yadet nare | koni (ya) vagarne khate bala az bein mire
	_delay_ms(5);
}
void cmp_calibrate()
{
	while(KEY_RIGHT);
	while(!KEY_RIGHT);
	
	i2c_start_write(0xc0);
	i2c_write(22);
	i2c_write(0xF0);
	i2c_stop();
	LED_UP_ON;

	while(KEY_RIGHT);
	while(!KEY_RIGHT);
	
	i2c_start_write(0xc0);
	i2c_write(22);
	i2c_write(0xF5);
	i2c_stop();
	
	while(KEY_RIGHT);
	while(!KEY_RIGHT);
	
	i2c_start_write(0xc0);
	i2c_write(22);
	i2c_write(0xF5);
	i2c_stop();
	while(KEY_RIGHT);
	while(!KEY_RIGHT);
	
	_delay_ms(5);
	i2c_start_write(0xc0);
	i2c_write(22);
	i2c_write(0xF5);
	i2c_stop();
	while(KEY_RIGHT);
	while(!KEY_RIGHT);
	_delay_ms(5);
	i2c_start_write(0xc0);
	i2c_write(22);
	i2c_write(0xF5);
	i2c_stop();
	LED_UP_OFF;
	
}



