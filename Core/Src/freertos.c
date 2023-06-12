/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SXFunc.h"
// #include "tim.h"
#include "usart.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <dht.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NODE_NUM 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t message_buf[256];
uint8_t message_flag;

// two flag for sender to know if a ack arrived
uint8_t wait_flag, got_ack;

// define a dictionary-like to store each node's data
typedef struct {
    uint8_t node;
    uint8_t data[256];
} DictionaryLike;
DictionaryLike data_of_nodes[NODE_NUM];
int dictionarySize = 0;

TaskHandle_t handle_receiver, handle_sender, handle_generator, handle_router;

SemaphoreHandle_t SX1278_sem = NULL;
SemaphoreHandle_t message_sem = NULL;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void mytask_receiver(void *argument);
void mytask_sender(void *argument);
void mytask_generator(void *argument);
void mytask_router(void *argument);

// receiver use
void add_node(uint8_t node, const uint8_t* data);
int find_node(uint8_t node);
void modify_data(uint8_t node, const uint8_t* newData);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */
    srand(time(NULL));
    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    SX1278_sem = xSemaphoreCreateMutex();
    message_sem = xSemaphoreCreateMutex();
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
                                    &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    long ret = 0;

    ret = xTaskCreate((TaskFunction_t)mytask_sender,
                      (const char *)"thread_sender", (uint16_t)256, (void *)NULL,
                      (UBaseType_t)2, (TaskHandle_t *)&handle_sender);
    log("ret after create sender %d", ret);

    ret = xTaskCreate((TaskFunction_t)mytask_receiver,
                      (const char *)"thread_receiver", (uint16_t)256, (void *)NULL,
                      (UBaseType_t)1, (TaskHandle_t *)&handle_receiver);
    log("ret after create recver %d", ret);

    ret = xTaskCreate((TaskFunction_t)mytask_generator,
                      (const char *)"thread_gen", (uint16_t)256, (void *)NULL,
                      (UBaseType_t)2, (TaskHandle_t *)&handle_generator);
    log("ret after create gen %d", ret);
    ret = xTaskCreate((TaskFunction_t)mytask_router,
                      (const char *)"thread_router", (uint16_t)256, (void *)NULL,
                      (UBaseType_t)2, (TaskHandle_t *)&handle_router);
    log("ret after create router %d", ret);
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void mytask_receiver(void *argument)
{
    // 接收器初始化
    log("receiver initialize");

    // 初始化任务的上次唤醒时间和所需的频率
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;

	// 初始化 xLastWakeTime 为当前的时钟计数
	xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // wait for next period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 获取信号量以访问共享资源
        xSemaphoreTake(SX1278_sem, portMAX_DELAY);
//		log("receive once");

        // 接收一个数据包
		SX_packet *pac2 = (SX_packet *)buffer2;
		int received = SX_recv_once(pac2);

		// 检查是否接收到有效的数据包
		if (received && SX_check_CRC(pac2))
		{
			// NOT SURE this else if is correct or not ----zzh
			// 检查是否为等待标志位为真、目的地为 MESH_ADDR_LOCAL，且类型为 TYPE_ACK 的确认数据包
			if (wait_flag && SX_check_dst(pac2, MESH_ADDR_LOCAL) && SX_check_type(pac2, (uint8_t)TYPE_ACK))
			{
				wait_flag = 0;
				got_ack = 1;
			}
			// 检查是否为类型为 TYPE_SOH 且目的地为 MESH_ADDR_LOCAL 的数据包
			else if (SX_check_type(pac2, (uint8_t)TYPE_SOH) && SX_check_dst(pac2, MESH_ADDR_LOCAL))
			{
				printf("[receiver] successfully received a packet.\r\n");
				SX_print(pac2);

				//----------------新增部分---------------------------
				// 存储节点信息
				int node_idx = find_node(pac2->src);
				if(node_idx == -1){
					add_node(pac2->src, pac2->content);
				} else {
					modify_data(pac2->src, pac2->content);
				}
				//--------------------------------------------------

				// 生成一个确认数据包
				SX_packet *pac = (SX_packet *)buffer;
				SX_generate_packet(pac, TYPE_ACK, NULL, pac2->dst, pac2->src, 0, 0);

				// send the packet
				SX_send(pac);
			}
			// 检查是否为类型为 TYPE_INFO 的路由信息数据包
			else if (SX_check_type(pac2, (uint8_t)TYPE_INFO))
			{
				printf("[receiver] successfully received a route info\r\n");
				SX_print(pac2);

				// 更新路由表
				int i = 0;
				for (i = 0; i < 16; i++)
				{
					if (pac2->content[i] != 255)
					{
						if (SX_route_table[i] == 255)
						{
							SX_route_table[i] = pac2->last_hop;
						}
					}
				}
			}
			// 检查是否为下一跳数据包
			else if (SX_check_nexthop(pac2, MESH_ADDR_LOCAL))
			{
				pac2->next_hop = SX_route_table[pac2->dst];
				pac2->last_hop = MESH_ADDR_LOCAL;
				CRC_BYTE(pac2) = CRC8((uint8_t *)pac2, pac2->length - 1);
				SX_send(pac2);
				printf("[receiver] successfully transfer a packet\r\n");
			}
//			else {
//				printf(
//						"[receiver] successfully received a packet and will be ignored\r\n");
//				SX_print(pac2);
//			}
		}
		xSemaphoreGive(SX1278_sem);
    }
}

void mytask_sender(void *argument)
{
    log("sender initialize");
    SX_packet *pac = (SX_packet *)buffer;
    SX_packet *pac2 = (SX_packet *)buffer2;
    static uint8_t retry_flag = 0, retry_cnt;
    static uint32_t retry_time;

    for (;;)
    {
        if (retry_flag)
        { // if now we are re-sending a message
            vTaskDelay(retry_time);
        }
        else
        { // just send a new message
            // wait for Notify
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        xSemaphoreTake(SX1278_sem, portMAX_DELAY);
        {
            // use the resource "message buffer"
            xSemaphoreTake(message_sem, portMAX_DELAY);
            {
                if (message_flag == 0)
                {
                    // if the message is empty
                    // loop
                    xSemaphoreGive(message_sem);
                    xSemaphoreGive(SX1278_sem);
                    continue;
                }
                // generate the packet
                SX_generate_packet(pac, TYPE_SOH, (char *)message_buf,
                                   MESH_ADDR_LOCAL, message_dst, 0, SX_route_table[message_dst]);
                //				message_dst, 0, 1);
                // send the packet
                SX_send(pac);
                // print information
                printf("[sender] successfully sent a packet\r\n");
                SX_print(pac);
                // give the semaphore
                xSemaphoreGive(message_sem);
            }

            // set flags
            wait_flag = 1;
            got_ack = 0;
        }
        xSemaphoreGive(SX1278_sem);
        // wait 2s for a ack
        vTaskDelay(2000);

        if (got_ack)
        {
            retry_flag = 0;
            retry_cnt = 0;

            xSemaphoreTake(message_sem, portMAX_DELAY);
            {
                message_flag = 0;
                xSemaphoreGive(message_sem);
            }

            printf("[sender]get ack, success!\r\n");
        }
        else
        {
            retry_flag = 1;
            retry_cnt++;
            retry_time = (rand() % (1 << retry_cnt)) * 100;
            log("wait %d then resend", retry_time);
            printf("[sender]get no ack, wait %d then resend\r\n", retry_time);
            if (retry_cnt > 5)
            {
                // send failed, give up
                xSemaphoreTake(message_sem, portMAX_DELAY);
                {
                    message_flag = 0;
                    xSemaphoreGive(message_sem);
                }
                retry_flag = 0;
                retry_cnt = 0;
            }
        }
    }
}

void mytask_generator(void *argument)
{

    uint32_t wait_time = 0;
    wait_time = (rand() % 7 + 3) * 1000;
    int seq = 0;

    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        log("in generator()");
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        xSemaphoreTake(message_sem, portMAX_DELAY);
        {
            if (message_flag == 0)
            {
                // fill the message buffer
                //  sprintf((char*) message_buf, "Hello M3! seq:%d", seq);
                DHT_ReadData(message_buf); // 从 DHT 读取数据
                message_dst = MESH_ADDR_REMOTE;
                message_flag = 1;
                seq++;
            }
            log("message generated!");
            xSemaphoreGive(message_sem);
            wait_time = (rand() % 7 + 3) * 1000;
            xTaskNotifyGive(handle_sender);
        }
    }
}

void mytask_router(void *argument)
{

    uint32_t wait_time = 0;
    wait_time = (rand() % 10) * 50 + 10000;
    int seq = 0;

    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        log("in router()");
        xSemaphoreTake(SX1278_sem, portMAX_DELAY);
        {
            SX_packet *pac = (SX_packet *)buffer;
            SX_generate_packet(pac, TYPE_INFO, NULL,
                               MESH_ADDR_LOCAL, 255, 0, 255);
            SX_send(pac);
            xSemaphoreGive(SX1278_sem);
            wait_time = (rand() % 10) * 50 + 10000;
        }
    }
}

void add_node(uint8_t node, const uint8_t* data){
	DictionaryLike entry;
	entry.node = node;
	strncpy((char*)entry.data, (const char*)data, sizeof(data));
	entry.data[sizeof(data)] = '\0';

	data_of_nodes[dictionarySize] = entry;
	dictionarySize++;
}

int find_node(uint8_t node){
	for (int i = 0; i < dictionarySize; i++) {
		if (data_of_nodes[i].node == node) {
			return i;
		}
	}

	return -1;
}

void modify_data(uint8_t node, const uint8_t* newData){
	for (int i = 0; i < dictionarySize; i++) {
		if (data_of_nodes[i].node == node) {
			strncpy((char*)data_of_nodes[i].data, (const char*)newData, sizeof(newData));
			data_of_nodes[i].data[sizeof(newData)] = '\0';
			return;
		}
	}

	printf("Node not found\n");
	return;
}
/* USER CODE END Application */
