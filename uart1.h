#ifndef UART1_H
#define UART1_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define USART1_BAUD 57600
#define RXD_BUFFER_LEN 220
#define TXD_BUFFER_LEN 20 // not so much needed

#define AEROQUAD_TELEMETRY_MSGSIZE 24
#define AEROQUAD_TELEMETRY_MSGSIZE_ECC (AEROQUAD_TELEMETRY_MSGSIZE + NPAR)

#define REQUEST_OSD_DATA "#bo?]==EG\r"
#define REQUEST_UART_TO_FC "#cu=IfREv\r"

/**
 * init usart1
 */
void usart1_init();

/**
 * disable the txd pin of usart1
 */
void usart1_DisableTXD(void);

/**
 * enable the txd pin of usart1
 */
void usart1_EnableTXD(void);

/**
 * send a PGM<string> throught usart1
 */
void usart1_puts_pgm(const char*);

/**
 * transmit interrupt handler
 * unused
 */
ISR(USART1_TX_vect);

/**
 * receive data through usart1
 * portions taken and adapted from
 * http://svn.mikrokopter.de/mikrowebsvn/filedetails.php?repname=FlightCtrl&path=%2Fbranches%2FV0.72p+Code+Redesign+killagreg%2Fuart0.c
 */
ISR(USART1_RX_vect);

void processUsart1Data();

/**
 * Decode the recevied Buffer
 * portions taken and adapted from
 * http://svn.mikrokopter.de/mikrowebsvn/filedetails.php?repname=FlightCtrl&path=%2Ftags%2FV0.72p%2Fuart.c
 */
void Decode64(void);

/**
 * Request Data through usart1 until a answer is received
 */
void usart1_request_blocking(unsigned char answer, const char* message);

/**
 * Request UART Redirect from NC to itself
 */
void usart1_request_nc_uart(void);

#endif //_UART_H


