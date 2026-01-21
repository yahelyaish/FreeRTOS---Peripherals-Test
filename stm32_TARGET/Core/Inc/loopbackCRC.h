
#ifndef INC_LOOPBACKCRC_H_
#define INC_LOOPBACKCRC_H_
#include <stdint.h>
#include "packet.h"

typedef enum {
    CRC_OK = 0,
    CRC_FAIL
} crc_status_t;

typedef uint8_t (*loopback_crc_t)(uint8_t *txbuff,uint8_t len,uint8_t numOfIter);

typedef struct loopback_table_t{
    loopback_crc_t uart;
    loopback_crc_t spi;
    loopback_crc_t i2c;
} loopback_table_t;

extern const loopback_table_t loopback;


uint8_t uart_loopback_crc(uint8_t *tx,uint8_t len, uint8_t iters);

uint8_t i2c_loopback_crc(uint8_t *tx,uint8_t len, uint8_t iters);

uint8_t spi_loopback_crc(uint8_t *tx,uint8_t len, uint8_t iters);

crc_status_t crc_check(uint8_t *tx,uint8_t *rx,uint8_t len);


#endif /* INC_LOOPBACKCRC_H_ */
