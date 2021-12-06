/*
 * ModbusProtocol.c
 *
 * Created: 10/3/2021 6:07:34 PM
 * Author : Vuko
 */ 

#define SLAVE_ID 1U
#define  F_CPU 32000000U

#define MB_PACKET_ERROR_SIZE    -1
#define MB_PACKET_ERROR_CRC     -2
#define MB_PACKET_ERROR_ADDR    -3
#define MB_PACKET_ERROR_UNKOWN  -4

/* Official modbus errors */
#define MODBUS_NO_ERROR             0
#define MODBUS_ILLEGAL_FUNCTION     1
#define MODBUS_ILLEGAL_ADDRESS      2
#define MODBUS_ILLEGAL_DATA_VALUE   3
#define MODBUS_DEVICE_FAILURE       4
#define MODBUS_ACKNOWLEDGE          5
#define MODBUS_DEVICE_BUSY          6
#define MODBUS_NO_ACKNOWLEDGE       7
#define MODBUS_WRITE_ERROR          8
#define MODBUS_OVERLAPPED_AREA      9

/* Definitions */
#define MAX_WORD_TO_READ      125
#define MAX_WORD_TO_WRITE     123
#define MAX_MODBUS_LENGTH     256
#define MAX_DATA_LENGTH       253
#define MAX_WORD_TO_WRITE_FUNC23    121
#define MAX_BYTES_FUNC_23     (MAX_WORD_TO_WRITE_FUNC23 * 2)

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

static uint_least8_t i;
static uint8_t interrupt_test;



//recive data variables
static uint8_t modbusData[20];
static uint8_t dataRecived;

//packet to send variables
static uint8_t dataLength;
static uint8_t* modbusDataPok;
static uint8_t sendData;
static uint8_t transmit_i;

typedef struct _data
{
	uint8_t slaveAddr;
	uint8_t func;
	uint16_t regStartAddr;
	uint16_t regQuantity;
	uint8_t lrc;

}ModbusDataStruct;

/* 
Funkcionalnosti prekida su odradjene u naredne dve funkcije
*/

ISR(USARTF0_RXC_vect) {
	
	uint8_t data = 0;

	register8_t saved_sreg = SREG;
	cli();

	data = USARTF0.DATA;

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
	}

	modbusData[i] = data;
	i++;
	
	SREG = saved_sreg;
}
// Ako iskljucim TXC prekid nece uposte da mi posalje podatak, objasnjenje
ISR(USARTF0_TXC_vect) {

	register8_t saved_sreg = SREG;//upis statusnog registra da se zapamti prije nego sto se uradi disable prekida
	cli();//disable svih prekida

	
	if(sendData)
	{
		if (transmit_i == (dataLength - 1))
		{
			sendData = 0;
		}
	
		USARTF0.DATA = modbusDataPok[transmit_i++];
	}

	SREG = saved_sreg;//vracanje statusnog registra u stanje prije obrade
}


/*
Podesavanja kontrolera: sata,prekida i usarta
*/

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

void interrupt_setup(void)
{
	sei();	//enable global interrupts

	PMIC.CTRL = 0x07;	//enable hi end interrupts
}


void usart_setup(void)
{
	//1 start bit, 7 bits, odd parity, 1 stop bit, 9600 baud rate
	
	PORTF.OUTSET = PIN3_bm;			//Set TxF0 pin High
	PORTF.DIRSET = PIN3_bm;			// PF3 (TXF0) as output.
	PORTF.DIRCLR = PIN2_bm;			// PF2 (RXF0) as input.

	USARTF0.CTRLA=0x3C;	//usart interrupts enable
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

void convert_mess_to_ascii(uint8_t* packet, uint8_t packetLen)
{
	uint8_t i = 0;

	for(i = 0; i < packetLen; i++)
	{
		if(packet[i] == ':' || packet[i] == 13 || packet[i] == 10)
			continue;

		else
		{
			if (packet[i] >= 0 && packet[i] <= 9)
			{
				packet[i] += 48;
			}

			if (packet[i] >= 10 && packet[i] <= 15)
			{
				packet[i] += 55;	
			}
		}
	}
}

//dobro radi
uint8_t convert_8bit(uint8_t hi_data, uint8_t lo_data)
{
	uint8_t ret_data = 0;

	//Konvertovanje iz ASCII u interni oblik broja

	//HI
	if(hi_data >= 'A' && hi_data <= 'F')
	{
		hi_data -= 55;
	}

	else if(hi_data >= 'a' && hi_data <='f')
	{
		hi_data -= 87;
	}

	else if(hi_data >= '0' && hi_data <= '9')
	{
		hi_data -= 48;
	}

	//LO
	if(lo_data >= 'A' && lo_data <= 'F')
	{
		lo_data -= 55;
	}

	else if(lo_data >= 'a' && lo_data <='f')
	{
		lo_data -= 87;
	}

	else if(lo_data >= '0' && lo_data <= '9')
	{
		lo_data -= 48;
	}

	ret_data = (hi_data << 4) | (lo_data);

	return ret_data;
}
//ako radi dobro 8bit konverzija cenim da i ova konverzija radi dobro
uint16_t convert_16bit(uint8_t hi_dataHi, uint8_t hi_dataLo, uint8_t lo_dataHi, uint8_t lo_dataLo)
{
	uint16_t ret_data = 0;
	uint8_t data16_hi = 0;
	uint8_t data16_lo = 0;

	data16_hi = convert_8bit(hi_dataHi, hi_dataLo);
	data16_lo = convert_8bit(lo_dataHi, lo_dataLo);

	ret_data = (data16_hi << 8) | (data16_lo); 

	return ret_data;
}
//radi dobro
void parse_modbus_data(const uint8_t* modbus_data, ModbusDataStruct* dataStruct)
{
	dataStruct->slaveAddr = convert_8bit(modbus_data[1], modbus_data[2]);
	dataStruct->func = convert_8bit(modbus_data[3], modbus_data[4]);
	dataStruct->regStartAddr = convert_16bit(modbus_data[5], modbus_data[6], modbus_data[7], modbus_data[8]);
	dataStruct->regQuantity = convert_16bit(modbus_data[9], modbus_data[10], modbus_data[11], modbus_data[12]);
	dataStruct->lrc = convert_8bit(modbus_data[13], modbus_data[14]);
}

//radi dobro
uint8_t mess_len(const uint8_t* mess)
{
	uint8_t i = 0;
	while(*(mess + i) != 10)
	{
		i++;
	}
	//Kad dodje do LF zavrsice petlju, ali i LF je deo poruke
	return i + 1;
}
//radi dobro
uint8_t lrc_calc2(const uint8_t* message, uint8_t mess_len)
{
	uint8_t lrc = 0 ; /* LRC char initialized */
	while (mess_len) /* pass through message buffer */
	{
		if(message[mess_len - 1] == ':')
			break;

		lrc += (message[mess_len - 1] - 48); /* add buffer byte without carry */

		mess_len--;
	}
	return ((uint8_t)(-((int8_t)lrc))); /* return twos complement */
}


// Vraca 1 kada je LRC Check dobar
// Vraca 0 kada je LRC Check los
// radi dobro
uint8_t lrc_check(ModbusDataStruct* modbusStruct, const uint8_t* modbusData)
{
	uint8_t lrc = 0U;
	uint8_t messlen = 0U;
	uint8_t retcode = 1U;

	//mess_len(modbusData) - 4 predstavlja poruku, ali bez poslednja 4 podatka koji predstavljaju LRC (2 chara) i CR, LF
	messlen = mess_len(modbusData);
	lrc = lrc_calc2(modbusData, messlen - 4);

	if(lrc != modbusStruct->lrc)
	{
		retcode = 0U;
	}

	return retcode;
}
// modbus_function3 ce mi vratiti kao response_data
// response_data[0] = 0
// response_data[1] i response_data[2] = slave_addr
// response_data[3] i response_data[4] = func_code
// response_data[5] - response_data[reg_number - 1] = podaci koje "citam" iz registara
// i duzinu odgovora - 4
uint8_t modbus_function3(ModbusDataStruct* modbusStruct, uint8_t* responseData, uint8_t slaveAddr, uint8_t* messageLength)
{
	uint8_t retcode = MODBUS_NO_ERROR;
	uint8_t func = 3U;
	uint16_t start_address = 0;
	uint16_t reg_number = 0;
	uint8_t i = 0;

	start_address = modbusStruct->regStartAddr;
	reg_number = modbusStruct->regQuantity;

	if((reg_number >= 0) || (reg_number <= MAX_WORD_TO_READ))
	{
		//citaj neke vrednosti bezveze za sad
		for(i = 0; i < reg_number; i++)
		{
			uint8_t value = 0xFFU; //podaci koji se citaju su obicno 16bitni, pa ih razlazemo u dva bajta, ali mozda i mogu da koristim 8bitni podatak
			
			responseData[5 + i] = value; //samo cu popuniti povratnu poruku nekim glupim vrednostima, ovde tipa 0xFF 
		}
		
		/*
		Malo glupo resenje da se popune prvi elemetni odgovora slave masine,
		ali za sada ce raditi posao, ne znam da li da nadjem pametniji nacin da popunim podatke odgovora
		tj da li mogu da citam neke stvare vrednosti upisane u neku memoriju
		*/

		//da li u ovom delu moram da podatak za slanje moram upisati kao ascii vrednost
		//npr ako imam slaveAddr 0xFF
		//da li cu uraditi 0xF + 48 pa to upisati u responseData[0]
		//ili cu upisati samo 0xF u responseData[0]

		//ali mislim da to moram uraditi posle racunanja lrc-a 

		responseData[0] = 0;//stavio sam 0 zbog racunanja lrc
		responseData[1] = (slaveAddr >> 4U);
		responseData[2] = (slaveAddr & 0x0FU);
		responseData[3] = (func >> 4U);
		responseData[4] = (func & 0x0FU);

		*messageLength = reg_number + 5; //+5 zbog prvih pet podataka poruke
									   	 //daje dobar messageLength
										 //ako reg_number moze biti veci od 256, tj 8bitni je podatak da l da povecam messageLenght, po pravilu treba
										 //al ne znam da li to raditi posto mi je definisan #define MAX_WORD_TO_READ = 125 sto znaci da max mogu da procitam 125 registara sto znaci da je reg_num osmobitni podatak 
									     //ali kada primam poruku onda mi vazi da moze biti veci i od toga zato sto primam 4 bajta koja mi oznacavaju koliko podataka citam, sto mi je kontradiktorno sa ovim komentarom iznad
	}

	else
	{
		retcode = MODBUS_ILLEGAL_DATA_VALUE;
	}

	return retcode;	//vraca dobar retcode
}

uint8_t modbus_get_result(ModbusDataStruct* modbusStruct, uint8_t* responseData, uint8_t* messageLen, uint8_t slaveAddr)
{
	uint8_t retcode = 0U;

	switch(modbusStruct->func)
	{
		case 3:
			retcode = modbus_function3(modbusStruct, responseData, slaveAddr, messageLen);
			break;
		
		default:
			retcode = MODBUS_ILLEGAL_FUNCTION;
			break;
	}

	return retcode;	
}

//main func for modbus
int32_t modbus_process(ModbusDataStruct* modbusStruct, uint8_t* packetToSend, uint8_t* responeMessLen)
{
	
	int32_t retcode = 0;
	uint8_t valid = 1U;	//sluzi nam da proveravamo da li se negde usput javila greska, ako jeste da izadjemo iz procesa i da posaljemo gresku
	uint8_t slaveAddr = SLAVE_ID;
	uint8_t func = 3;
	uint8_t processResult = 0;

	//da li treba proveriti duzinu poruke, ako treba TODO: dodati jos jedan if iznad i proslediti odgovarajuci error
	//LRC Check
	if (lrc_check(modbusStruct, modbusData))
	{
		if ((modbusStruct->slaveAddr == SLAVE_ID) || (modbusStruct->slaveAddr == 0U))
		{
			//Ako je prosao provere dobre duzine poruke(ako treba), ako je dobar LRC i ako je dobra adresa slavea, tj. ako se obraca ovoj masini nastavi obradu poruke

			processResult = modbus_get_result(modbusStruct, packetToSend, &dataLength, slaveAddr);
			
			//popravi ovo ovde je sve skoro lose koliko sad vidim
			//niasm uradio greske, kako to da vratim?
			if(processResult != MODBUS_NO_ERROR)
			{
				USARTF0.DATA = 'd';
				packetToSend[0] = 0;
				packetToSend[1] = slaveAddr >> 8U;
				packetToSend[2] = slaveAddr && 0xFFU;
				packetToSend[3] = func >> 8U;
				packetToSend[4] = func && 0xFFU;
				packetToSend[5] = processResult >> 8U;
				packetToSend[6] = processResult && 0xFFU;

				//checksum
				uint8_t error_lrc = 0;
				error_lrc = lrc_calc2(packetToSend,7U);

				packetToSend[7] = error_lrc >> 8U;
				packetToSend[8] = error_lrc && 0xFFU;
				packetToSend[9] = 13;
				packetToSend[10] = 10;
				packetToSend[0] = ':';

				retcode = -1;
				dataLength = 11;

				//poruka spremna za slanje
				//posalji parvi karakter iniciraj slanje podataka
				USARTF0.DATA = packetToSend[0];
				
			
			}

			else
			{
				//moram sve osim :, CR i LF-a prebaciti u ascii pre slanja poruke i posle racunanja lrc-a
				uint8_t lrc = lrc_calc2(packetToSend, *responeMessLen);
				packetToSend[0] = ':';	//bila je 0 dok sam racunao lrc, sada mogu da postavim ovaj karakter koji oznacava pocetak komunikacije
				//nadovezi lrc na poruku
				packetToSend[*responeMessLen] = (lrc >> 4U);
				packetToSend[*responeMessLen + 1] = (lrc & 0x0FU);
				//nadovezi cr(13) i lf(10) na poruku
				packetToSend[*responeMessLen + 2] = 13;
				packetToSend[*responeMessLen + 3] = 10;

				*responeMessLen += 4U; //produzi zbog lrc, i cr lf
				retcode = *responeMessLen; //ako je sve dobro vrati samo duzinu poruke


				convert_mess_to_ascii(packetToSend, *responeMessLen);

				//poruka spremna za slanje
				//Posalji prvi karkater iniciraj slanje podataka
				sendData = 1;
				transmit_i = 1;	//ovo mi oznacava od kog podatka krecem komunikaciju
								//posto cu posalati : da zapocenem komunikaciju (sto mi je prvi podatak) mogu da nastavim slanje podataka preko prekida od prvog sledeceg
				
				USARTF0.DATA = packetToSend[0];
			}
		}

		else
		{
			retcode = MB_PACKET_ERROR_ADDR;
		}
	}

	else
	{
		retcode = MB_PACKET_ERROR_CRC;
	}

	return retcode;
}

int main(void)
{
	clock_setup();
    usart_setup();
	interrupt_setup();
	ModbusDataStruct modbusStruct;
	int32_t retcode = 0;
    while (1) 
    {
		if (dataRecived)
		{
			dataRecived = 0;
			parse_modbus_data(modbusData, &modbusStruct);
			USARTF0.DATA = modbusStruct.regQuantity;
			retcode = modbus_process(&modbusStruct, modbusDataPok, &dataLength);
			//moze se uraditi neka funkcionalnost u zavisnosti od retcode, ali za sad nek cuti samo
		}
		
	}
}

