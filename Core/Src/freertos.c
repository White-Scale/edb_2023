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
//#include "tim.h"
#include "usart.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t message_buf[256];
uint8_t message_flag;

//two flag for sender to know if a ack arrived
uint8_t wait_flag, got_ack;

TaskHandle_t handle_receiver, handle_sender, handle_generator;

SemaphoreHandle_t SX1278_sem = NULL;
SemaphoreHandle_t message_sem = NULL;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void mytask_receiver(void *argument);
void mytask_sender(void *argument);
void mytask_generator(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
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

	ret = xTaskCreate((TaskFunction_t) mytask_sender,
			(const char*) "thread_sender", (uint16_t) 256, (void*) NULL,
			(UBaseType_t) 2, (TaskHandle_t*) &handle_sender);
	log("ret after create sender %d", ret);

	ret = xTaskCreate((TaskFunction_t) mytask_receiver,
			(const char*) "thread_receiver", (uint16_t) 256, (void*) NULL,
			(UBaseType_t) 1, (TaskHandle_t*) &handle_receiver);
	log("ret after create recver %d", ret);

	ret = xTaskCreate((TaskFunction_t) mytask_generator,
			(const char*) "thread_gen", (uint16_t) 256, (void*) NULL,
			(UBaseType_t) 2, (TaskHandle_t*) &handle_generator);
	log("ret after create gen %d", ret);
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
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void mytask_receiver(void *argument) {
	log("receiver initialize");
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;

	// init xLastWakeTime
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		// wait for next period
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		xSemaphoreTake(SX1278_sem, portMAX_DELAY);
		{
//			log("receive once");
			SX_packet *pac2 = (SX_packet*) buffer2;
			int received = SX_recv_once(pac2);
			if (received) {
				if (wait_flag) {
					if (SX_check_packet(pac2, (uint8_t) TYPE_ACK)) {
						wait_flag = 0;
						got_ack = 1;
					}
				}
				int valid = SX_check_packet(pac2, (uint8_t) TYPE_SOH);

				if (valid) {
					uint8_t crc = CRC_BYTE(pac2);
					CRC_BYTE(pac2) = 0;
					printf(
							"[receiver] succesfully received a packect\r\n[receiver] content: %s\r\n",
							pac2->content);
					CRC_BYTE(pac2) = crc;
					SX_packet *pac = (SX_packet*) buffer;
					//set the packet's attributes
					pac->type = TYPE_ACK;
					pac->src = pac2->dst;
					pac->dst = pac2->src;
					pac->seq = 0;
					//set the content as a empty string,but maybe actually not necessary
					*(pac->content) = '\0';
					int len_content = 0;
					//set other attributes
					pac->reserve = 0;
					pac->length = len_content + sizeof(SX_packet) + 1;
					CRC_BYTE(pac) = CRC8((uint8_t*) pac, pac->length - 1);
					//send the packet
					SX_send(pac);

				}
			}

			xSemaphoreGive(SX1278_sem);
		}
	}
}
void mytask_sender(void *argument) {
	log("sender initialize");
	SX_packet *pac = (SX_packet*) buffer;
	SX_packet *pac2 = (SX_packet*) buffer2;
	static uint8_t retry_flag = 0, retry_cnt;
	static uint32_t retry_time;

	for (;;) {
		if (retry_flag) { // if now we are re-sending a message
			vTaskDelay(retry_time);
		} else { //just send a new message
			// wait for Notify
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		}

		if (message_flag == 0) {
			continue;
		}
		xSemaphoreTake(SX1278_sem, portMAX_DELAY);
		{
			//use the resource "message buffer"
			xSemaphoreTake(message_sem, portMAX_DELAY);
			{
				//move the message to the packet buffer
				int len_content = sprintf((char*) pac->content, "%s",
						(char*) message_buf);
				pac->length = len_content + sizeof(SX_packet) + 1;

			}
			//set attributes of the packet to send
			pac->type = TYPE_SOH;
			pac->src = MESH_ADDR_LOCAL;
			pac->dst = MESH_ADDR_REMOTE;
			//set CRC
			CRC_BYTE(pac) = CRC8((uint8_t*) pac, pac->length - 1);
			//send the packet
			SX_send(pac);
			uint8_t crc = CRC_BYTE(pac);
			CRC_BYTE(pac) = 0;
			printf(
					"[sender] succesfully sent a packect\r\n[sender] content: %s\r\n",
					pac->content);
			CRC_BYTE(pac) = crc;
			xSemaphoreGive(message_sem);

			//set flags
			wait_flag = 1;
			got_ack = 0;
		}
		xSemaphoreGive(SX1278_sem);

		vTaskDelay(2000);

		if (got_ack) {
			retry_flag = 0;
			retry_cnt = 0;

			message_flag = 0;
			printf("[sender]get ack, success!\r\n");
		} else {
			retry_flag = 1;
			retry_cnt++;
			retry_time = (rand() % (1 << retry_cnt)) * 100;
			log("wait %d then resend", retry_time);
			printf("[sender]get no ack, wait %d then resend\r\n", retry_time);
			if (retry_cnt > 5) {
				//send failed, give up
				//					xSemaphoreGive(message_sem);
				message_flag = 0;
				retry_flag = 0;
				retry_cnt = 0;
			}
		}

	}
}

void mytask_generator(void *argument) {

	uint32_t wait_time = 0;
	wait_time = (rand() % 7 + 3) * 1000;
	int seq = 0;

	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		log("in generator()");
		vTaskDelayUntil(&xLastWakeTime, wait_time);
		xSemaphoreTake(message_sem, portMAX_DELAY);
		{
			if (message_flag == 0) {
				//fill the message buffer
				sprintf((char*) message_buf, "Hello M3! seq:%d", seq);
				message_flag = 1;
				seq++;
			}
			log("message genarated!");
			xSemaphoreGive(message_sem);
			wait_time = (rand() % 7 + 3) * 1000;
			xTaskNotifyGive(handle_sender);
		}
	}

}
/* USER CODE END Application */

