#include "dht.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

void TMstr(uint8_t *str)
{
    HAL_UART_Transmit(&huart1, str, strlen(str), 1000);
}

// 时间延迟函数（单位：us）
void delay_us(uint16_t us)
{
    uint16_t differ = 0xffff - us - 5;
    __HAL_TIM_SET_COUNTER(&htim2, differ); // 设定TIM2计数器起始值
    HAL_TIM_Base_Start(&htim2);            // 启动定时器
    while (differ < 0xffff - 5)
    {
        differ = __HAL_TIM_GET_COUNTER(&htim2); // 查询计数器的计数值
    }
    HAL_TIM_Base_Stop(&htim2);
}

// 向 DHT11 发送开始信号
void DHT_Start()
{
    DHT_Change_Output(); // 修改 GPIO 模式为输出模式
    HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);
    delay_us(20000); // 拉低至少18ms
    HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_SET);
    delay_us(30); // 主机拉高20~40us
    HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);
}

// 接受 DHT11 的响应信号并返回是否有效响应
uint8_t DHT_receive()
{
    DHT_Change_Input(); // 修改 GPIO 为输入模式
    // 用于计时电平时间
    uint8_t count = 0;
    // 先读取低电平 80us
    while (HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin) && count < COUNT_TIME)
    {
        count++;
        delay_us(1);
    }
    if (count >= COUNT_TIME)
        return 0; // 读取低电平失败，返回 0
    count = 0;    // 重置
    // 再读取高电平 80us
    while (!HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin) && count < COUNT_TIME)
    {
        count++;
        delay_us(1);
    }
    if (count >= COUNT_TIME)
        return 0; // 读取高电平失败，返回 0
    return 1;     // 返回成功
}

// 读取 1bit 数据
uint8_t DHT_ReadBit()
{
    while (HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)) // 等待变为低电平
    {
        delay_us(1);
    }
    while (!HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)) // 等待变为高电平
    {
        delay_us(1);
    }
    delay_us(30); // 等待 30us 后根据高低电平判断 1/0
    if (HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin))
        return 1; // 1 的高电平时间 70us
    return 0;     // 0 的高电平时间 26-28us
}

// 读取一个字节的数据
uint8_t DHT_ReadByte()
{
    DHT_Change_Input(); // 修改 GPIO 为输入模式
    uint8_t data = 0, i;
    for (i = 0; i < 8; i++) // 循环读取 8 位
    {
        data <<= 1; // 左移空出新的一位
        data |= DHT_ReadBit();
    }
    return data;
}

uint8_t DHT_ReadData(uint8_t *data)
{
    DHT_Start();
    if (!DHT_receive())
    {
        TMstr("DHT 未正确响应\r\n");
        return 0; // DHT 未正确响应，读取失败
    }

    uint8_t readData[5];
    for (uint8_t i = 0; i < 5; i++) // 读取 5 字节数据
    {
        readData[i] = DHT_ReadByte(); // 读取 1 字节数据
    }
    if (readData[0] + readData[1] + readData[2] + readData[3] == readData[4]) // 数据校验
    {
        *data = readData[0];
        data++;
        *data = readData[2];
    }

    return 1; // 读取成功
}
