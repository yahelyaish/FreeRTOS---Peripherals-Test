/*
 * packet.h
 *
 *  Created on: Jan 18, 2026
 *      Author: yahel
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#include <stdint.h>

#define NUM_OF_ITER 100
#define MAX_PAYLOAD 256


typedef enum peripheral_t{
    UART=0,
    I2C,
    SPI,
    PERIPH_COUNT
} peripheral_t;


extern const char* periphStr[];


typedef struct __attribute__ ((packed)) packet_t{
	uint32_t TEST_ID;
	uint8_t PERIPHERAL;
	uint8_t numOfIter;
	uint8_t payloadSize;
	uint8_t payLoad[MAX_PAYLOAD];
}packet_t;

typedef struct resProtocol{
    uint32_t testID;
	uint8_t status;
}resProtocol;

packet_t* initPacket(const uint8_t *buff,uint8_t idx);
void fillPacketMeta(packet_t* pkt,uint32_t* testID);
void printPacket(packet_t* pkt);
packet_t makeResultPacket(uint32_t testID, uint8_t pass);
#endif /* INC_PACKET_H_ */
