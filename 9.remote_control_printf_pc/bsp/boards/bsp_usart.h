#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"


extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
