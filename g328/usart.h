#ifndef _UART_H_
#define _UART_H_

#define BAUD_TO_UBRR(baud) F_CPU/16/baud-1

void usart_init(uint16_t ubrr);
void usart_send(uint8_t data);
uint8_t usart_receive(void);
int usart_putchar(char c, FILE *stream);
int usart_getchar(FILE *stream);

#endif /* _UART_H_ */
