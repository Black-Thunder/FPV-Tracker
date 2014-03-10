#ifndef UART_H
#define UART_H

#define printf_P(format, args...)   _printf_P(&uart_putchar, format , ## args)
#define printf(format, args...)     _printf_P(&uart_putchar, PSTR(format) , ## args)

void BearbeiteRxDaten(void);

extern unsigned volatile char UebertragungAbgeschlossen;
extern unsigned volatile char NeueKoordinateEmpfangen;
extern unsigned volatile char RxdBuffer[];
extern void usart0_Init (void);
extern void Uart1Init(void);
extern void processUsartData(void);
extern void usart0_request_nc_uart(void);
extern void usart0_puts_pgm(const char*);

#define USART0_BAUD 57600

#endif //_UART_H


