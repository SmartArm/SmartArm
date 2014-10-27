#include<stm32f4xx.h>
void DMA_Config(void);
static void DMA_NVIC_Config(void);
#define SENDBUFF_SIZE 38400
static uint8_t buffer[SENDBUFF_SIZE];//”√“‘¥Ê¥¢ÕºœÒ
extern int DMAComplete;
extern int DMAstate;
