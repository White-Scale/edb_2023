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

TaskHandle_t handle_receiver,handle_sender;

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

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	SX1278_sem = xSemaphoreCreateMutex();
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

	xTaskCreate((TaskFunction_t) mytask_receiver, (const char*) "thread_receiver",
			(uint16_t) 512, (void*) NULL, (UBaseType_t) 1,
			(TaskHandle_t*) &handle_receiver);
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
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;

	// init xLastWakeTime
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		// wait for next period
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		xSemaphoreTake(SX1278_sem, portMAX_DELAY);
		{
			SX_packet *pac2 = (SX_packet*) buffer2;
			int received = SX_recv_once(pac2);
			if (received) {
				int valid = SX_check_packet(pac2);
				if (valid) {
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

	SX_packet *pac = (SX_packet*) buffer;

	for (;;) {
		// wait for Notify
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		xSemaphoreTake(SX1278_sem, portMAX_DELAY);
		{
			//use the resource "message buffer"
			xSemaphoreTake(message_sem, portMAX_DELAY);
			{
				//move the message to the packet buffer
				int len_content = sprintf((char*) pac->content, "%s", (char*)message_buf);
				pac->length = len_content + sizeof(SX_packet) + 1;
				xSemaphoreGive(message_sem);
			}
			//set attributes of the packet to send
			pac->type = TYPE_SOH;
			pac->src = MESH_ADDR_LOCAL;
			pac->dst = MESH_ADDR_REMOTE;
			//set CRC
			CRC_BYTE(pac) = CRC8((uint8_t*) pac, pac->length - 1);
			//send the packet
			SX_send(pac);
			xSemaphoreGive(SX1278_sem);
		}
	}
}
/* USER CODE END Application */

