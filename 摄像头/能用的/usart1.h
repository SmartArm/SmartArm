#ifndef __USART1_H
#define	__USART1_H

#include<stm32f4xx.h>
#include <stdio.h>

void USART1_Config(void);
int fputc(int ch, FILE *f);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
#define DEBUG(FORMAT,...)     do{printf("\r\n");printf(FORMAT,##__VA_ARGS__);printf("\r\n");}while(0)	/*无需打印调试信息时，请将宏内容注释掉*/

#endif /* __USART1_H */
