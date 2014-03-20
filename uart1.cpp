#include <stdarg.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <wiring.c>
#include "uart1.h"
#include "Mikrokopter_Datastructs.h"
#include "AeroQuad_Datastructs.h"
#include "TrackerGCS.h"

volatile uint8_t rxd_buffer_locked = 0;
volatile uint8_t rxd_buffer[RXD_BUFFER_LEN];
volatile uint8_t ReceivedBytes = 0;
volatile uint8_t *pRxData = 0;
volatile uint8_t RxDataLen = 0;

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

	// UART Double Speed (U2X), enable RX interrupt
	UCSR1A |= (1 << U2X1) | (1 << RXC1) ;

	// Global interrupt flag
	SREG |= (1 << 7);
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

    if (rxd_buffer_locked) return; // if rxd buffer is locked immediately return
    static uint16_t crc;
    static uint8_t ptr_rxd_buffer = 0;
    static uint8_t c1 = 0;
    static uint8_t c2 = 0;
    static uint8_t usart_rx_ok = 0;
    uint8_t crc1, crc2;
    
    // the rxd buffer is unlocked
    if (usart_rx_ok == 0) {
        if ((c2 == '#') && (c1 == 'b' || c1 == 'c') &&
            (c == 'V' || c == 'O' || c == 'Q' || c == 'C')) { // version, OSD, settings, 3D-Data
            usart_rx_ok = 1;
            rxd_buffer[ptr_rxd_buffer++] = c2;
            crc = c2;
            rxd_buffer[ptr_rxd_buffer++] = c1;
            crc += c1;
            rxd_buffer[ptr_rxd_buffer++] = c;
            crc += c;
            c2 = 0;
            c1 = 0;
        } else {
            c2 = c1;
            c1 = c;
        }
    } else if (ptr_rxd_buffer < RXD_BUFFER_LEN) { // collect incomming bytes
        if (c != '\r') { // no termination character
            rxd_buffer[ptr_rxd_buffer++] = c; // copy byte to rxd buffer
            crc += c; // update crc
        } else { // termination character was received
            // the last 2 bytes are no subject for checksum calculation
            // they are the checksum itself
            crc -= rxd_buffer[ptr_rxd_buffer - 2];
            crc -= rxd_buffer[ptr_rxd_buffer - 1];
            // calculate checksum from transmitted data
            crc %= 4096;
            crc1 = '=' + crc / 64;
            crc2 = '=' + crc % 64;
            // compare checksum to transmitted checksum bytes
            if ((crc1 == rxd_buffer[ptr_rxd_buffer - 2]) && (crc2 == rxd_buffer[ptr_rxd_buffer - 1])) { // checksum valid
                rxd_buffer[ptr_rxd_buffer] = '\r'; // set termination character
                ReceivedBytes = ptr_rxd_buffer + 1; // store number of received bytes
                rxd_buffer_locked = 1; // lock the rxd buffer
            } else { // checksum invalid
                rxd_buffer_locked = 0; // unlock rxd buffer
            }
            ptr_rxd_buffer = 0; // reset rxd buffer pointer
            usart_rx_ok = 0;
        }
    } else { // rxd buffer overrun
        ptr_rxd_buffer = 0; // reset rxd buffer
        rxd_buffer_locked = 0; // unlock rxd buffer
        usart_rx_ok = 0;
    }
}

void processUsart1Data(void)
{
	if (!rxd_buffer_locked) return;

	if (protocolType == 0) {
		memcpy((char*)(&telemetryPacketAeroQuad), (char*)pRxData, sizeof(TelemetryPacket_t));

		uavSatellitesVisible = telemetryPacketAeroQuad.gpsinfo;
		uavLatitude = telemetryPacketAeroQuad.latitude / 1.0e7f;
		uavLongitude = telemetryPacketAeroQuad.longitude / 1.0e7f;
		uavAltitude = telemetryPacketAeroQuad.altitude / 10;
	}
	else if (protocolType == 1) {
		if (rxd_buffer[2] == 'O') { // NC OSD Data
			Decode64();
			memcpy((char*)(&telemetryPacketMikrokopter), (char*)pRxData, sizeof(NaviData_t));

			uavSatellitesVisible = telemetryPacketMikrokopter.SatsInUse;
			uavLatitude = telemetryPacketMikrokopter.CurrentPosition.Latitude / 1.0e7f;
			uavLongitude = telemetryPacketMikrokopter.CurrentPosition.Longitude / 1.0e7f;
			uavAltitude = telemetryPacketMikrokopter.CurrentPosition.Altitude / 10;
		}
	}

	lastPacketReceived = millis();
	isTelemetryOk = true;
	rxd_buffer_locked = 0;
	pRxData = 0;
	RxDataLen = 0;

	if (uavSatellitesVisible > 5) uavHasGPSFix = true;
	else uavHasGPSFix = false;
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
    uint8_t len = ReceivedBytes - 6;

    while (len) {
        a = rxd_buffer[ptrIn++] - '=';
        b = rxd_buffer[ptrIn++] - '=';
        c = rxd_buffer[ptrIn++] - '=';
        d = rxd_buffer[ptrIn++] - '=';

        x = (a << 2) | (b >> 4);
        y = ((b & 0x0f) << 4) | (c >> 2);
        z = ((c & 0x03) << 6) | d;

        if (len--) rxd_buffer[ptrOut++] = x;
        else break;
        if (len--) rxd_buffer[ptrOut++] = y;
        else break;
        if (len--) rxd_buffer[ptrOut++] = z;
        else break;
    }
    pRxData = &rxd_buffer[3];
    RxDataLen = ptrOut - 3;
}

/**
 * Request Data through usart1 until a answer is received
 */
void usart1_request_blocking(unsigned char answer, const char* message) {
    rxd_buffer[2] = answer + 1; // unvalidate answer
    while (rxd_buffer[2] != answer || (rxd_buffer_locked != 1)) {
        rxd_buffer_locked = 0;
        usart1_EnableTXD();
        usart1_puts_pgm(message);
        usart1_DisableTXD();
        static uint8_t wait = 0;
        wait = 0;

                // wait for complete answer
        while (rxd_buffer_locked == 0 && wait < 200) {
            wait++;
            _delay_ms(10);
        }
        _delay_ms(150);
    }
    Decode64();
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







