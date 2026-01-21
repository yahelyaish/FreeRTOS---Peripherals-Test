/*
 * packet.c
 *
 *  Created on: Jan 18, 2026
 *      Author: yahel
 */
#include "inttypes.h"
#include "packet.h"
#include "stdlib.h"

const char* periphStr[]={"UART","I2C","SPI"};

//packet_t* initPacket(const uint8_t *buff, uint8_t len)
//{
//    packet_t *pkt = pvPortMalloc(sizeof(packet_t));
//    if (pkt == NULL)
//    {
//        return NULL;
//    }
//
//    pkt->payLoad = pvPortMalloc(len + 1);
//    if (pkt->payLoad == NULL)
//    {
//        vPortFree(pkt);
//        return NULL;
//    }
//
//    memcpy(pkt->payLoad, buff, len);
//    pkt->payLoad[len] = '\0';
//    pkt->payloadSize = len;
//
//    return pkt;
//}


//void fillPacketMeta(packet_t* pkt,uint32_t* testID){
//	pkt->TEST_ID=++(*testID);
//	pkt->numOfIter=NUM_OF_ITER;
//	pkt->PERIPHERAL=UART;
//}

void printPacket(packet_t* pkt)
{
    if (!pkt) {
        printf("pkt is NULL\r\n");
        return;
    }

    printf("\r\n----- PACKET -----\r\n");

    printf("test_id      = %d\r\n", pkt->TEST_ID);

    if (pkt->PERIPHERAL < PERIPH_COUNT) {
        printf("peripheral   = %s\r\n", periphStr[pkt->PERIPHERAL]);
    } else {
        printf("peripheral   = UNKNOWN (%d)\r\n", pkt->PERIPHERAL);
    }

    printf("num of iter  = %u\r\n", pkt->numOfIter);
    printf("payload size= %u\r\n", pkt->payloadSize);

    printf("payload      = ");
    for (uint8_t i = 0; i < pkt->payloadSize; i++) {
        printf("%02X ", (uint8_t)pkt->payLoad[i]);
    }
    printf("\r\n------------------\r\n");
}

packet_t makeResultPacket(uint32_t testID, uint8_t pass)
{
    packet_t result;

    memset(&result, 0, sizeof(packet_t));

    result.TEST_ID     = testID;
    result.payloadSize = 1;
    result.payLoad[0]  = pass;   // 1 = PASS, 0 = FAIL

    return result;
}
