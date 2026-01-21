/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//#include "crc.h"
#include <semphr.h>

#include "i2c.h"
#include "usart.h"
#include "spi.h"

#include "loopbackCRC.h"
#include "packet.h"


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

#define SPI1_DONE (1U << 0)
#define SPI4_DONE (1U << 1)


volatile uint8_t rxSPI1;
volatile uint8_t rxSPI4;

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
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//uint8_t Consolebuff[MAX_PAYLOAD];
packet_t cmd;

QueueHandle_t packetQueue;
QueueHandle_t routeMsgQueue;
static QueueHandle_t periphQueueTable[PERIPH_COUNT];
QueueHandle_t resultQueue;

TaskHandle_t uartRxTask_Handle;
TaskHandle_t dispatcherTask_Handle;
TaskHandle_t routerTask_Handle;
TaskHandle_t UARTPeriphCheckTASK_Handle;
TaskHandle_t I2CPeriphCheckTASK_Handle;
TaskHandle_t SPIPeriphCheckTASK_Handle;
TaskHandle_t uartTx_Handle;



void uartRxTask(void *argument){
	for(;;){
    	 /* 1. WAIT: receive CMD from GBB */
    			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    				 printPacket(&cmd);
    				 if (xQueueSend(routeMsgQueue, &cmd, portMAX_DELAY) != pdTRUE){
    					 printf("Send msg on QUEUE failed!\r\n");
    				 }
    					HAL_UART_Receive_IT(&huart2,(uint8_t*)&cmd,sizeof(packet_t));
    			}
}




//void dispatcherTask(void *arg){
//	packet_t pktCHECK;
//	for(;;)
//	{
//		if(xQueueReceive(packetQueue, &pktCHECK, portMAX_DELAY)==pdTRUE)
//		{
//			fillPacketMeta(pktCHECK,&g_testIdCounter);
//			if(xQueueSend(routeMsgQueue,&pktCHECK,portMAX_DELAY)!=pdTRUE)
//			{
//				printf("sending to routeMsgQueue Failed\r\n");
//			}
//		}
//	}
//}


void routerTask(void *arg){
	packet_t routerPKT;
	for(;;){
		if(xQueueReceive(routeMsgQueue, &routerPKT,portMAX_DELAY)==pdTRUE){
			if(xQueueSend(periphQueueTable[routerPKT.PERIPHERAL],&routerPKT,portMAX_DELAY)!=pdTRUE){
				printf("error sending to peripheral\r\n");
			}
		}
	}
}

void UARTPeriphCheckTASK(void *arg){
	packet_t UART_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(periphQueueTable[UART], &UART_PKT, portMAX_DELAY)==pdTRUE){
			result.testID=UART_PKT.TEST_ID;
			result.status = loopback.uart(UART_PKT.payLoad,UART_PKT.payloadSize,UART_PKT.numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
				printf("sending FAILED\n");
			}
		}
	}
}

void I2CPeriphCheckTASK(void *arg){
	packet_t I2C_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(periphQueueTable[I2C], &I2C_PKT, portMAX_DELAY)!=pdFALSE){
			result.testID=I2C_PKT.TEST_ID;
			result.status = loopback.i2c(I2C_PKT.payLoad,I2C_PKT.payloadSize,I2C_PKT.numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
							printf("sending FAILED\n");
			}
		}
	}
}


void SPIPeriphCheckTASK(void *arg){
	packet_t SPI_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(periphQueueTable[SPI], &SPI_PKT, portMAX_DELAY)!=pdFALSE){
			result.testID=SPI_PKT.TEST_ID;
			result.status = loopback.spi(SPI_PKT.payLoad,SPI_PKT.payloadSize,SPI_PKT.numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
							printf("sending FAILED\n");
			}
		}
	}
}


void transmitTask(void *arg){
    resProtocol resTX;
	for(;;){
		if(xQueueReceive(resultQueue, &resTX, portMAX_DELAY)==pdTRUE){
			printf(
			 "\r\nTEST ID = %lu | PERIPH = %s | PASSED %u TIMES OUT OF %u\r\n",
			 resTX.testID,
			 periphStr[cmd.PERIPHERAL],
			 resTX.status,
			 cmd.numOfIter
			);
			packet_t pkt = makeResultPacket(resTX.testID,resTX.status==cmd.numOfIter);
			size_t tx_len = offsetof(packet_t, payLoad) + pkt.payloadSize;
			HAL_UART_Transmit(&huart2,(uint8_t*)&pkt,tx_len,HAL_MAX_DELAY);
		}
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	printf("hello guys lets begin\r\n");
HAL_UART_Receive_IT(&huart2,(uint8_t*)&cmd,sizeof(packet_t));
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	//doneRxSem = xSemaphoreCreateCounting(10, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
//	packetQueue= xQueueCreate(4,sizeof(packet_t));

	//array of QueueTables
	periphQueueTable[UART] = xQueueCreate(4, sizeof(packet_t));
	periphQueueTable[I2C]  = xQueueCreate(4, sizeof(packet_t));
	periphQueueTable[SPI]  = xQueueCreate(4, sizeof(packet_t));

	routeMsgQueue =xQueueCreate(4,sizeof(packet_t));

	resultQueue=xQueueCreate(4,sizeof(resProtocol));








  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	xTaskCreate(transmitTask,"transmitTask", 256, NULL, 1, &uartTx_Handle);
//	xTaskCreate(dispatcherTask, "dispatcherTask", 256, NULL, 1, &dispatcherTask_Handle);

	xTaskCreate(routerTask, "routerTask", 256, NULL, 1, &routerTask_Handle);

	xTaskCreate(UARTPeriphCheckTASK, "UARTPeriphCheckTASK", 256, NULL, 1, &UARTPeriphCheckTASK_Handle);
	xTaskCreate(I2CPeriphCheckTASK, "I2CPeriphCheckTASK", 256, NULL, 1, &I2CPeriphCheckTASK_Handle);
	xTaskCreate(SPIPeriphCheckTASK, "SPIPeriphCheckTASK", 256, NULL, 1, &SPIPeriphCheckTASK_Handle);

	xTaskCreate(uartRxTask, "uartRxTask", 256, NULL, 1, &uartRxTask_Handle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */



/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*--------------CALLBACKS-----------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart2)
        {
            xTaskNotifyFromISR(
                uartRxTask_Handle,
                0,
                eNoAction,
                &xHigherPriorityTaskWoken
            );
        }

    if (huart == &huart4)
    {
    	   xTaskNotifyFromISR(
    	        UARTPeriphCheckTASK_Handle,
    	        0,
				eNoAction,
    	        &xHigherPriorityTaskWoken
    	    );

    }
    if (huart == &huart6){
    	   xTaskNotifyFromISR(
    	        UARTPeriphCheckTASK_Handle,
    	        0,
				eNoAction,
    	        &xHigherPriorityTaskWoken
    	    );
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(hi2c==&hi2c2)
	{
		 xTaskNotifyFromISR(
				 	 	 	 I2CPeriphCheckTASK_Handle,
		    	    	            0,
		    	    	            eNoAction,
		    	    	            &xHigherPriorityTaskWoken
		    	    	        );
	}
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
       if (hspi == &hspi4) {
    	   rxSPI4=1;
       }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
       if (hspi == &hspi1) {
    	   rxSPI1=1;
       }
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */






/* USER CODE END Application */
