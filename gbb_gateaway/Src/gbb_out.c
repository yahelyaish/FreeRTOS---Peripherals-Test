#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stddef.h>
#include "packet.h"
#include "uart.h"

#define UART_PATH "/dev/ttyS1"
#define UART_BAUD 115200

#define PORT 5000
const char* str[] = {"UART","I2C","SPI"};
int main(void)
{
    int sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    /* 1. create UDP socket */
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    /* 2. setup server address */
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons(PORT);

    /* 3. bind */
    if (bind(sock,
             (struct sockaddr*)&server_addr,
             sizeof(server_addr)) < 0) {
        perror("bind");
        close(sock);
        return 1;
    }

    printf("GBB UDP server listening on port %d...\n", PORT);
    int uart_fd = uart_open(UART_PATH,UART_BAUD);
    /* 4. main server loop */
    while (1)
    {
        uint8_t tx_buff[sizeof(packet_t)];
        memset(tx_buff, 0, sizeof(tx_buff));

        ssize_t len = recvfrom(sock,
                               tx_buff,
                               sizeof(tx_buff),
                               0,
                               (struct sockaddr*)&client_addr,
                               &addr_len);

        if (len <= 0)
            continue;

        size_t header_len = offsetof(packet_t, payload);
        if (len < header_len)
            continue;

        packet_t *cmd = (packet_t *)tx_buff;

        if (cmd->payload_len != len - header_len)
            continue;
        printf("\n-------------------------\n");
        printf("\nCMD received from %s:%d\n",
               inet_ntoa(client_addr.sin_addr),
               ntohs(client_addr.sin_port));

        printf("test_id = %u\nperipheral = %s\niterations = %u\npayload_len = %u\n",
               cmd->test_id, str[cmd->peripheral],
               cmd->iterations, cmd->payload_len);

        /* ---------------- ACK ---------------- */

        packet_t ack;
        build_ack(&ack, cmd);
        send_packet_udp(sock, &client_addr, &ack);
        printf("\nACK sent for test %u\n", ack.test_id);

        /* --------  STM tests ---------- */

        packet_t result;
        size_t uart_rx_len = offsetof(packet_t, payload) + 1;
        uart_write(uart_fd, cmd, sizeof(packet_t));

        if (uart_wait_for_response(uart_fd,
                           &result,
                           uart_rx_len)<0 ){
        printf("STM did not respond\n");
        continue;
        }

        send_packet_udp(sock, &client_addr, &result);
        printf("\nRESULT sent for test %u (status=%s)\n",
        result.test_id,
        result.payload[0] ? "PASS" : "FAIL");
    }


    close(sock);
    return 0;
}
