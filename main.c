/*
 * ModbusProtocol.c
 *
 * Created: 10/3/2021 6:07:34 PM
 * Author : Vuko
 */ 

#include <avr/io.h>
#include <inttypes.h>

#define SLAVE_ID 01

static uint8_t modbusData[20];
static uint8_t dataRecived;
static uint8_t* modbusDataPok;
static uint_least8_t i;



void usart_setup(void)
{
	//1 start bit, 7 bits, odd parity, 1 stop bit, 9600 baud rate
	
	PORTF.OUTSET = PIN3_bm;			//Set TxF0 pin High
	PORTF.DIRSET = PIN3_bm;			// PF3 (TXF0) as output.
	PORTF.DIRCLR = PIN2_bm;			// PF2 (RXF0) as input.
	USARTF0.CTRLA=0;
	USARTF0_CTRLB = 0x18; //Transmit, Recive enable
	//Sve radi super kada je asinhroni mod, no parity, 8 bita i 1 stop bit
	/*
		Ne radi dobro:
			Asinhroni, even, 1 stop, 7 bita
			Asinhroni, odd, 1 stop, 7 bita
			Asinhroni, even, 2 stop, 7 bita
			Asinhroni, even, 1 stop, 8 bita

	*/
	USARTF0_CTRLC = 0x03; //Asinhroni, no parity, 8 bita, 1 stop bit

	//Baud rate setup
	USARTF0.BAUDCTRLB = 0x00; //BScale = 0
	USARTF0.BAUDCTRLA=13; //BSel = 13	
}

void clock_setup(void)
{
	//32Mhz / 16 = 2Mhz clock

	//Disable all clock sources
	OSC_CTRL = 0x00;
	//Enable the internal 32.768 kHz RC oscillator
	OSC_CTRL = 0x04; //Enable RC32k
	//Wait for the internal 32.768 kHz RC oscillator to stabilize
	while(!(OSC_STATUS & OSC_RC32KRDY_bm));
	//Enable the internal 32 MHz RC oscillator
	CLK_CTRL = 0x01;
	//Disable protected IOs to update settings
	CPU_CCP = 0xD8;
	//Configure prescalar
	CLK_PSCTRL = 0x1C;  //32MHz devide with 16
	//Configure DFLL for calibration
	OSC_DFLLCTRL = 0x00;
	DFLLRC32M_CTRL = 0x01;
	//Wait for the internal 32 MHz RC oscillator to stabilize
	while(!(OSC_STATUS & OSC_RC32MEN_bm));
	//Disable protected IOs to update settings
	CPU_CCP = 0xD8;
	//Select system clock source
	CLK_CTRL = 0x01;
}

void usart_rx_tx(void)
{
	uint8_t recived = 0;

	while(!(USARTF0.STATUS & USART_RXCIF_bm));
	
	recived = USARTF0.DATA;
	recive_data(recived);

	while(!(USARTF0.STATUS & USART_DREIF_bm));

	send_data();

}

void recive_data(uint8_t data)
{
	if(data == ':')
	{
		i = 0;
		dataRecived = 0;
		modbusDataPok = modbusData;
	}

	else if(data == 10)
	{
		dataRecived = 1;
		modbusData[i] =  data;
		i = 0;
		return;
	}

	modbusData[i] = data;
	i++;
}

void send_data()
{
	if(dataRecived == 1)
	{
		if(*modbusDataPok != 10)
		{
			USARTF0.DATA = *modbusDataPok;
			modbusDataPok++;
		}
		else
		{
			USARTF0.DATA = *modbusDataPok;
			dataRecived = 0;
		}

	}
}

int main(void)
{
	clock_setup();
    usart_setup();

    while (1) 
    {
		usart_rx_tx();
    }
}

