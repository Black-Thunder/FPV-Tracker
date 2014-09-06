#ifndef UART1_H
#define UART1_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdarg.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <wiring.c>
#include "Mikrokopter_Datastructs.h"
#include "AeroQuad_Datastructs.h"
#include "TrackerGCS.h"
#include "rs.h"

#define USART1_BAUD 57600
#define RXD_BUFFER_LEN 220
#define TXD_BUFFER_LEN 20 // not so much needed

#define AEROQUAD_TELEMETRY_MSGSIZE 24
#define AEROQUAD_TELEMETRY_MSGSIZE_ECC (AEROQUAD_TELEMETRY_MSGSIZE + NPAR)

#define REQUEST_OSD_DATA "#bo?]==EG\r"
#define REQUEST_UART_TO_FC "#cu=IfREv\r"
#define REQUEST_NC_VERSION "#bv====Dl\r"

extern volatile uint8_t *pRxData;

volatile bool isRxdBufferLocked = false;
volatile uint8_t rxdBuffer[RXD_BUFFER_LEN];
volatile uint8_t receivedBytes = 0;
volatile uint8_t *pRxData = 0;
volatile uint8_t rxDataLen = 0;

volatile TelemetryPacket_t telemetryPacketAeroQuad;
volatile NaviData_t telemetryPacketMikrokopter;

/**
* init usart1
*/
void usart1_init() {
	UBRR1H = ((F_CPU / (16UL * USART1_BAUD)) - 1) >> 8;
	UBRR1L = (F_CPU / (16UL * USART1_BAUD)) - 1;

	// Enable receiver and transmitter; enable RX interrupt
	UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
}

/**
* disable the txd pin of usart1
*/
void usart1_DisableTXD(void) {
	UCSR1B &= ~(1 << TXCIE1); // disable TX-Interrupt
	UCSR1B &= ~(1 << TXEN1); // disable TX in USART
	DDRB &= ~(1 << DDB3); // set TXD pin as input
	PORTB &= ~(1 << PORTB3); // disable pullup on TXD pin
}

/**
* enable the txd pin of usart1
*/
void usart1_EnableTXD(void) {
	DDRB |= (1 << DDB3); // set TXD pin as output
	PORTB &= ~(1 << PORTB3); // disable pullup on TXD pin
	UCSR1B |= (1 << TXEN1); // enable TX in USART
	UCSR1B |= (1 << TXCIE1); // enable TX-Interrupt
}

/**
* send a single <character> through usart1
*/
void usart1_putc(unsigned char character) {
	// wait until UDR ready
	while (!(UCSR1A & (1 << UDRE1)));
	UDR1 = character;
}

/**
* send a PGM<string> throught usart1
*/
void usart1_puts_pgm(const char* string) {
	while (pgm_read_byte(string) != 0x00)
		usart1_putc(pgm_read_byte(string++));
}

/**
* transmit interrupt handler
* unused
*/
ISR(USART1_TX_vect) {
}

/*
* receive data through usart1
* portions taken and adapted from
* http://svn.mikrokopter.de/mikrowebsvn/filedetails.php?repname=FlightCtrl&path=%2Fbranches%2FV0.72p+Code+Redesign+killagreg%2Fuart0.c
*/
ISR(USART1_RX_vect) {
	uint8_t c;
	// catch the received byte
	c = UDR1;

	if (isRxdBufferLocked) return; // if rxd buffer is locked immediately return

	static uint8_t c1 = 0;
	static uint8_t c2 = 0;
	static bool isUsartRxOk = false;
	static uint16_t crc;
	static uint8_t ptrRxdBuffer = 0;
	uint8_t crc1, crc2;

	if (!isUsartRxOk) {
		if ((protocolType == MikrokopterProtocol) && (c2 == '#') && (c1 == 'c') && (c == 'O')) { // OSD data from MK, starts with '#cO'
			isUsartRxOk = true;
			rxdBuffer[ptrRxdBuffer++] = c2;
			crc = c2;
			rxdBuffer[ptrRxdBuffer++] = c1;
			crc += c1;
			rxdBuffer[ptrRxdBuffer++] = c;
			crc += c;
			c2 = 0;
			c1 = 0;
		}
		else if ((protocolType == AeroQuadProtocol) && (c1 == 'A') && (c == 'Q')) { // telemetry data from AQ, starts with 'AQ'
			isUsartRxOk = true;
			rxdBuffer[ptrRxdBuffer++] = c1;
			rxdBuffer[ptrRxdBuffer++] = c;
			c2 = 0;
			c1 = 0;
		}
		else {
			c2 = c1;
			c1 = c;
		}
	}
	else if (protocolType == AeroQuadProtocol) {
		if (ptrRxdBuffer < AEROQUAD_TELEMETRY_MSGSIZE_ECC) {
			rxdBuffer[ptrRxdBuffer++] = c; // copy byte to rxd buffer
		}
		else { // all bytes received
			receivedBytes = ptrRxdBuffer; // store number of received bytes
			ptrRxdBuffer = 0; // reset rxd buffer pointer
			isRxdBufferLocked = true; // lock the rxd buffer
			isUsartRxOk = false;
		}
	}
	else if (protocolType == MikrokopterProtocol) {
		if (ptrRxdBuffer < RXD_BUFFER_LEN) { // collect incomming bytes
			if (c != '\r') { // no termination character
				rxdBuffer[ptrRxdBuffer++] = c; // copy byte to rxd buffer
				crc += c; // update crc
			}
			else { // termination character was received
				// the last 2 bytes are no subject for checksum calculation
				// they are the checksum itself
				crc -= rxdBuffer[ptrRxdBuffer - 2];
				crc -= rxdBuffer[ptrRxdBuffer - 1];
				// calculate checksum from transmitted data
				crc %= 4096;
				crc1 = '=' + crc / 64;
				crc2 = '=' + crc % 64;
				// compare checksum to transmitted checksum bytes
				if ((crc1 == rxdBuffer[ptrRxdBuffer - 2]) && (crc2 == rxdBuffer[ptrRxdBuffer - 1])) { // checksum valid
					rxdBuffer[ptrRxdBuffer] = '\r'; // set termination character
					receivedBytes = ptrRxdBuffer + 1; // store number of received bytes
					isRxdBufferLocked = true; // lock the rxd buffer
				}
				else { // checksum invalid
					isRxdBufferLocked = false; // unlock rxd buffer
				}
				ptrRxdBuffer = 0; // reset rxd buffer pointer
				isUsartRxOk = false;
			}
		}
		else { // rxd buffer overrun
			ptrRxdBuffer = 0; // reset rxd buffer
			isRxdBufferLocked = false; // unlock rxd buffer
			isUsartRxOk = false;
		}
	}
}

/**
* Decode the recevied Buffer
* portions taken and adapted from
* http://svn.mikrokopter.de/mikrowebsvn/filedetails.php?repname=FlightCtrl&path=%2Ftags%2FV0.72p%2Fuart.c
*/
void Decode64(void) {
	uint8_t a, b, c, d;
	uint8_t x, y, z;
	uint8_t ptrIn = 3;
	uint8_t ptrOut = 3;
	uint8_t len = receivedBytes - 6;

	while (len) {
		a = rxdBuffer[ptrIn++] - '=';
		b = rxdBuffer[ptrIn++] - '=';
		c = rxdBuffer[ptrIn++] - '=';
		d = rxdBuffer[ptrIn++] - '=';

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if (len--) rxdBuffer[ptrOut++] = x;
		else break;
		if (len--) rxdBuffer[ptrOut++] = y;
		else break;
		if (len--) rxdBuffer[ptrOut++] = z;
		else break;
	}
	pRxData = &rxdBuffer[3];
	rxDataLen = ptrOut - 3;
}

void processUsart1Data(void)
{
	if (!isRxdBufferLocked) return;

	if (protocolType == AeroQuadProtocol) {
		decode_data(rxdBuffer, AEROQUAD_TELEMETRY_MSGSIZE_ECC);

		// Check if data is corrupted and try to correct
		bool isMessageCorrupted = false;

		if (check_syndrome() != 0) {
			isMessageCorrupted = correct_errors_erasures(rxdBuffer, AEROQUAD_TELEMETRY_MSGSIZE_ECC, 0, 0);
		}

		if (!isMessageCorrupted) {
			memcpy((char*)(&telemetryPacketAeroQuad), (char*)rxdBuffer, sizeof(TelemetryPacket_t));

			uavSatellitesVisible = telemetryPacketAeroQuad.gpsinfo >> 12;
			uavLatitude = telemetryPacketAeroQuad.latitude;
			uavLongitude = telemetryPacketAeroQuad.longitude;

			if (telemetryPacketAeroQuad.altitude < 0) telemetryPacketAeroQuad.altitude = 0;
			uavAltitude = telemetryPacketAeroQuad.altitude * 10;
		}
	}
	else if (protocolType == MikrokopterProtocol) {
		if (rxdBuffer[2] == 'O') { // NC OSD Data
			Decode64();
			memcpy((char*)(&telemetryPacketMikrokopter), (char*)pRxData, sizeof(NaviData_t));

			uavSatellitesVisible = telemetryPacketMikrokopter.SatsInUse;
			uavLatitude = telemetryPacketMikrokopter.CurrentPosition.Latitude;
			uavLongitude = telemetryPacketMikrokopter.CurrentPosition.Longitude;

			if (telemetryPacketMikrokopter.Altimeter < 0) telemetryPacketMikrokopter.Altimeter = 0;
			uavAltitude = telemetryPacketMikrokopter.Altimeter * 5;
		}
	}

	lastPacketReceived = millis();
	isTelemetryOk = true;
	isRxdBufferLocked = false;
	pRxData = 0;
	rxDataLen = 0;

	if (uavSatellitesVisible > 5) uavHasGPSFix = true;
	else uavHasGPSFix = false;
}

/**
* Request UART Redirect from NC to itself
*/
void usart1_request_nc_uart(void) {
	usart1_EnableTXD();
	usart1_putc(0x1B);
	usart1_putc(0x1B);
	usart1_putc(0x55);
	usart1_putc(0xAA);
	usart1_putc(0x00);
	usart1_DisableTXD();
}


#endif //_UART_H


