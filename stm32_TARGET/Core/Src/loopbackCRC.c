/*
 * loopbackCRC.c
 *
 *  Created on: Jan 18, 2026
 *      Author: yahel
 */

#include "loopbackCRC.h"
#include "spi.h"
#include "usart.h"
#include "i2c.h"
#include "crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "packet.h"
#include "stdio.h"

#define SLAVE_ADDR 0x34<<1
extern volatile uint8_t rxSPI1;
extern volatile uint8_t rxSPI4;

const loopback_table_t loopback = {
    .uart = uart_loopback_crc,
    .spi  = spi_loopback_crc,
    .i2c  = i2c_loopback_crc
};
void printHEX(const char* str,uint8_t *tx,uint8_t len);

uint8_t uart_loopback_crc(uint8_t *tx,uint8_t len, uint8_t iters)
{
	uint8_t UART4_6successCnt=0,UART6_4successCnt=0;

	uint8_t rx_buff[len];
	uint8_t rx_buff_back[len];
	for(size_t i=0;i<iters;i++)
	{
        /* ===== UART6 -> UART4 ===== */

		if (HAL_UART_Receive_IT(&huart4, rx_buff, len) != HAL_OK) {
		    printf("UART4 RX BUSY!\r\n");
		}
		HAL_UART_Transmit_IT(&huart6, tx, len);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (crc_check(tx, rx_buff, len) == CRC_OK){
		            UART4_6successCnt++;
		        }

        /* ===== UART4 -> UART6 ===== */
		 if (HAL_UART_Receive_IT(&huart6, rx_buff, len) != HAL_OK) {
		 		    printf("UART6 RX BUSY!\r\n");
		 		}
        HAL_UART_Transmit_IT(&huart4, rx_buff, len);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        		if (crc_check(tx, rx_buff, len) == CRC_OK){
        		            UART6_4successCnt++;
        		        }
    }
	//checking UART4-6 and UART6-4 if both passed all
	if((UART4_6successCnt== iters) && (UART6_4successCnt == iters)){
	return (iters);
	}else{
		//returning the big of both
		return ((UART4_6successCnt>UART6_4successCnt)? UART4_6successCnt : UART6_4successCnt );
	}
}



uint8_t i2c_loopback_crc(uint8_t *tx,uint8_t len, uint8_t iters)
{
	uint8_t I2CsuccessCnt=0;
	uint8_t rxPacket[len];
	for(size_t i=0;i<iters;i++)
	{
		HAL_I2C_Slave_Receive_IT(&hi2c2,rxPacket, len);
		HAL_I2C_Master_Transmit_IT(&hi2c1, SLAVE_ADDR, tx, len);

	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			if(crc_check(tx,rxPacket,len)==CRC_OK){
				++I2CsuccessCnt;
	}
}
	return I2CsuccessCnt;
}






uint8_t spi_loopback_crc(uint8_t *tx,uint8_t len, uint8_t iters)
{
	uint8_t rxSPI4_buff[len];
	uint8_t rxSPI1_buff[len],txSPI1_buff[len];
	uint8_t SPI1_4successCnt=0,SPI4_1successCnt=0;
	memset(rxSPI4_buff,0,len);
	memset(rxSPI1_buff,0,len);
	for(size_t i=0;i<iters;i++)
	{

		HAL_SPI_Receive_IT(&hspi4, rxSPI4_buff, len);
		HAL_SPI_Transmit(&hspi1, tx,len,HAL_MAX_DELAY);
	 //   ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		while(!rxSPI4);
		rxSPI4=0;
		if(crc_check(tx,rxSPI4_buff,len)==CRC_OK){
					++SPI1_4successCnt;

		//memcpy(txSPI4_buff, rxSPI4_buff, len);

//		HAL_SPI_TransmitReceive(&hspi1, txSPI1_buff,rxSPI1_buff, len,HAL_MAX_DELAY);
//		HAL_SPI_Transmit(&hspi4, rxSPI4_buff,len,HAL_MAX_DELAY);
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//		while(!rxSPI1);
//			rxSPI1=0;

		if(crc_check(rxSPI4_buff,rxSPI1_buff,len)==CRC_OK){
				++SPI4_1successCnt;
			}
		}
	}

	//checking SPI1-4 and SPI4-1 if both passed all
		if((SPI1_4successCnt== iters) && (SPI4_1successCnt == iters)){
		return (iters);
		}else{
			//returning the big of both
			return ((SPI1_4successCnt>SPI4_1successCnt)? SPI1_4successCnt : SPI4_1successCnt );
		}
}


crc_status_t crc_check(uint8_t *tx,uint8_t *rx,uint8_t len){

	uint32_t crc_tx;
	uint32_t crc_rx;

	crc_tx=HAL_CRC_Calculate(&hcrc,(uint32_t*)tx,len);
	crc_rx=HAL_CRC_Calculate(&hcrc,(uint32_t*)rx, len);
	if(crc_tx==crc_rx){
		return CRC_OK;
	}
	return CRC_FAIL;
}

void printHEX(const char* str,uint8_t *tx,uint8_t len){
	printf("%s\r\n",str);
	for(int i=0;i<len;i++){
		printf("%02X ",tx[i]);
	}
	printf("\r\n");
}

