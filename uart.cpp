#include <stdarg.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


#include "uart.h"
#include "Mikrokopter_Datastructs.h"
#include "AeroQuad_Datastructs.h"
#include "TrackerGCS.h"

#define MAX_SENDE_BUFF     220
#define MAX_EMPFANGS_BUFF  220

unsigned volatile char SioTmp = 0;
unsigned volatile char NeuerDatensatzEmpfangen = 0;
unsigned volatile char NeueKoordinateEmpfangen = 0;
unsigned volatile char UebertragungAbgeschlossen = 1;
unsigned volatile char CntCrcError = 0;
unsigned volatile char AnzahlEmpfangsBytes = 0;
unsigned volatile char TxdBuffer[MAX_SENDE_BUFF];
unsigned volatile char RxdBuffer[MAX_EMPFANGS_BUFF];

unsigned char *pRxData = 0;
unsigned char RxDataLen = 0;
unsigned volatile char PC_DebugTimeout = 0;
unsigned volatile char PC_MotortestActive = 0;
unsigned char DebugTextAnforderung = 255;

unsigned char PcZugriff = 100;
unsigned char MotorTest[16];
unsigned char MeineSlaveAdresse = 1; // Flight-Ctrl
unsigned char ConfirmFrame;

int Display_Timer, Debug_Timer, Kompass_Timer, Timer3D;
unsigned int DebugDataIntervall = 0, Intervall3D = 0, Display_Interval = 0;
unsigned int AboTimeOut = 0;
volatile unsigned int CountMilliseconds = 0;

volatile TelemetryPacket_t telemetryPacketAeroQuad;
volatile NaviData_t naviData;

ISR(USART0_TX_vect)
{
	static unsigned int ptr = 0;
	unsigned char tmp_tx;

	if (!UebertragungAbgeschlossen)
	{
		ptr++;                    // die [0] wurde schon gesendet
		tmp_tx = TxdBuffer[ptr];
		if ((tmp_tx == '\r') || (ptr == MAX_SENDE_BUFF))
		{
			ptr = 0;
			UebertragungAbgeschlossen = 1;
		}
		UDR0 = tmp_tx;
	}
	else ptr = 0;
}

ISR(USART0_RX_vect)
{
	static unsigned int crc;
	static unsigned char crc1, crc2, buf_ptr;
	static unsigned char UartState = 0;
	unsigned char CrcOkay = 0;

	SioTmp = UDR0;

	if (buf_ptr >= MAX_SENDE_BUFF) UartState = 0;

	if (SioTmp == '\r' && UartState == 2) {
		UartState = 0;
		crc -= RxdBuffer[buf_ptr - 2];
		crc -= RxdBuffer[buf_ptr - 1];
		crc %= 4096;
		crc1 = '=' + crc / 64;
		crc2 = '=' + crc % 64;
		CrcOkay = 0;

		if ((crc1 == RxdBuffer[buf_ptr - 2]) && (crc2 == RxdBuffer[buf_ptr - 1])) CrcOkay = 1;
		else {
			CrcOkay = 0;
			CntCrcError++;
		};

		if (!NeuerDatensatzEmpfangen && CrcOkay) { // Datensatz schon verarbeitet
			NeuerDatensatzEmpfangen = 1;
			AnzahlEmpfangsBytes = buf_ptr + 1;
			RxdBuffer[buf_ptr] = '\r';
		}
	}
	else
		switch (UartState)
	{
		case 0:
			if (SioTmp == '#' && !NeuerDatensatzEmpfangen) UartState = 1;  // Startzeichen und Daten schon verarbeitet
			buf_ptr = 0;
			RxdBuffer[buf_ptr++] = SioTmp;
			crc = SioTmp;
			break;
		case 1: // Adresse auswerten
			UartState++;
			RxdBuffer[buf_ptr++] = SioTmp;
			crc += SioTmp;
			break;
		case 2: //  Eingangsdaten sammeln
			RxdBuffer[buf_ptr] = SioTmp;
			if (buf_ptr < MAX_EMPFANGS_BUFF) buf_ptr++;
			else UartState = 0;
			crc += SioTmp;
			break;
		default:
			UartState = 0;
			break;
	}
}

void AddCRC(unsigned int wieviele)
{
	unsigned int tmpCRC = 0, i;
	for (i = 0; i < wieviele; i++)
	{
		tmpCRC += TxdBuffer[i];
	}

	tmpCRC %= 4096;
	TxdBuffer[i++] = '=' + tmpCRC / 64;
	TxdBuffer[i++] = '=' + tmpCRC % 64;
	TxdBuffer[i++] = '\r';
	UebertragungAbgeschlossen = 0;
	UDR0 = TxdBuffer[0];
}

void Decode64(void)
{
	unsigned char a, b, c, d;
	unsigned char x, y, z;
	unsigned char ptrIn = 3; // start at begin of data block
	unsigned char ptrOut = 3;
	unsigned char len = AnzahlEmpfangsBytes - 6; // von der Gesamtbytezahl eines Frames gehen 3 Bytes des Headers  ('#',Addr, Cmd) und 3 Bytes des Footers (CRC1, CRC2, '\r') ab.

	while (len)
	{
		a = RxdBuffer[ptrIn++] - '=';
		b = RxdBuffer[ptrIn++] - '=';
		c = RxdBuffer[ptrIn++] - '=';
		d = RxdBuffer[ptrIn++] - '=';

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if (len--) RxdBuffer[ptrOut++] = x;
		else break;
		if (len--) RxdBuffer[ptrOut++] = y;
		else break;
		if (len--) RxdBuffer[ptrOut++] = z;
		else break;
	}
	pRxData = (unsigned char*)&RxdBuffer[3]; // decodierte Daten beginnen beim 4. Byte
	RxDataLen = ptrOut - 3;  // wie viele Bytes wurden dekodiert?
}

/**
 * disable the txd pin of usart0
 */
void usart0_DisableTXD(void) {
	UCSR0B &= ~(1 << TXEN0);        // disable TXD in USART
	DDRD &= ~(1 << DDD1);             // set TXD pin as input
}

/**
 * enable the txd pin of usart0
 */
void usart0_EnableTXD(void) {
	DDRD |= (1 << DDD1);              // set TXD pin as output
	UCSR0B |= (1 << TXEN0);         // enable TX in USART
}

void processUsartData(void)
{
	if (!NeuerDatensatzEmpfangen) return;

	if (protocolType == 0) {
		memcpy((char*)(&telemetryPacketAeroQuad), (char*)pRxData, sizeof(TelemetryPacket_t));

		uav_satellites_visible = telemetryPacketAeroQuad.gpsinfo;
		uav_lat = telemetryPacketAeroQuad.latitude / 1.0e7f;
		uav_lon = telemetryPacketAeroQuad.longitude / 1.0e7f;
		uav_alt = telemetryPacketAeroQuad.altitude / 10;
		uav_heading = telemetryPacketAeroQuad.heading;
		uav_gpsheading = telemetryPacketAeroQuad.course;
	}
	else if (protocolType == 1) {
		if (RxdBuffer[2] == 'O') { // NC OSD Data
			Decode64();
			memcpy((char*)(&naviData), (char*)pRxData, sizeof(NaviData_t));

			uav_satellites_visible = naviData.SatsInUse;
			uav_lat = naviData.CurrentPosition.Latitude / 1.0e7f;
			uav_lon = naviData.CurrentPosition.Longitude / 1.0e7f;
			uav_alt = naviData.CurrentPosition.Altitude / 10;
			uav_heading = naviData.Heading;
		}
	}

	NeuerDatensatzEmpfangen = 0;
	pRxData = 0;
	RxDataLen = 0;

	if (uav_satellites_visible > 5) hasGPSFix = true;
	else hasGPSFix = false;
}

/**
 * send a single <character> through usart1
 */
void usart0_putc(unsigned char character) {
	// wait until UDR ready
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = character;
}

/**
 * send a <string> throught usart0
 */
void usart0_puts(char *s) {
	while (*s) {
		usart0_putc(*s);
		s++;
	}
}

/**
 * send a PGM<string> throught usart0
 */
void usart0_puts_pgm(const char* string) {
	while (pgm_read_byte(string) != 0x00)
		usart0_putc(pgm_read_byte(string++));
}

unsigned int SetDelay(unsigned int t) {
	return(CountMilliseconds + t + 1);
}

char CheckDelay(unsigned int t)	{
	return(((t - CountMilliseconds) & 0x8000) >> 9);
}

/**
 * Request UART Redirect from NC to itself
 */
void usart0_request_nc_uart(void) {
	usart0_EnableTXD();
	usart0_putc(0x1B);
	usart0_putc(0x1B);
	usart0_putc(0x55);
	usart0_putc(0xAA);
	usart0_putc(0x00);
	usart0_DisableTXD();
}

void usart0_Init(void) {
	unsigned int ubrr = (unsigned int)((unsigned long)F_CPU / (8 * USART0_BAUD) - 1);

	//Enable TXEN im Register UCR TX-Data Enable & RX Enable
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	// UART Double Speed (U2X)
	UCSR0A |= (1 << U2X0);
	// RX-Interrupt Freigabe
	UCSR0B |= (1 << RXCIE0);
	// TX-Interrupt Freigabe
	UCSR0B |= (1 << TXCIE0);
	// USART0 Baud Rate Register
	// set clock divider
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;

	pRxData = 0;
	RxDataLen = 0;
}







