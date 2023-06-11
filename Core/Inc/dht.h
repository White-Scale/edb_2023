#ifndef DHT11_DHT11_H_
#define DHT11_DHT11_H_

#include "stm32f1xx_hal.h"

#define DHT_Pin GPIO_PIN_3
#define DHT_GPIO_Port GPIOA
#define COUNT_TIME 100

void TMstr(uint8_t *str);
void delay_us(uint16_t us);
void DHT_Start();
uint8_t DHT_receive();
uint8_t DHT_ReadByte();
uint8_t DHT_ReadData(uint8_t *data);

#endif