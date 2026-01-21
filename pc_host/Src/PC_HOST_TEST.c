#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stddef.h>   // offsetof
#include "packet.h"
#include <time.h>
#include "stdlib.h"

#define CLIENT_IP "192.168.7.2"   // GBB
#define PORT 5000 // now socket address = 192.168.7.2:5000


#define PERIPH_UART 0x00
#define PERIPH_I2C  0x01
#define PERIPH_SPI  0x02
#define PERIPH_MAX  3
const char* peripStr[PERIPH_MAX]={"UART","I2C","SPI"};
uint8_t testCnt;

int main(void)
{
    srand(time(NULL)); //for random data

    int sock;
    struct sockaddr_in server_addr;

    /* 1. create UDP socket */
    sock = socket(AF_INET, SOCK_DGRAM, 0); //IPV4 , UDP\DATAGRAM , 0 MEANS THE KERNEL CHOOSE THE PROTOCOL
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    /* 2. setup server address */
    memset(&server_addr, 0, sizeof(server_addr)); //RESETING SERVERADDRESS
    server_addr.sin_family = AF_INET; //IPV4
    server_addr.sin_port   = htons(PORT); //the port the socket listens to 

    if (inet_pton(AF_INET, CLIENT_IP, &server_addr.sin_addr) <= 0) {// convert address from ASCII to BINARY
        perror("inet_pton");
        close(sock);
        return 1;
    }

    //build+send packet
    packet_t rx_pkt;

    for (int periph = PERIPH_UART; periph < PERIPH_MAX ; periph++)
    {
        printf("\n--------------------\n");
        printf("\nsending periph: %s\n",peripStr[periph]);
    packet_t tx_pkt;
    memset(&tx_pkt, 0, sizeof(tx_pkt));

    build_packet(&tx_pkt, periph, ++testCnt);

    /* 1. send CMD */
    if (send_packet_udp(sock, &server_addr, &tx_pkt) < 0) {
        perror("send_packet_udp");
        return 1;
    }

    printPacket(&tx_pkt);
    /* 2. wait for ACK or RES */
    while (1) 
    {
        ssize_t len = recvfrom(sock, &rx_pkt, sizeof(rx_pkt), 0, NULL, NULL);
        if (len <= 0) {
          perror("recvfrom");
         continue;
}
        if (rx_pkt.payload_len == 0) {
            printf("\nACK received for test number %u of periph: %s\n", rx_pkt.test_id,peripStr[periph]);
            continue;
        }
        else {
            printf("\nRESULT received for test number %u of periph: %s\n", rx_pkt.test_id,peripStr[periph]);
            printf("Result = %s\n",
                rx_pkt.payload[0] ? "PASS" : "FAIL");
                   break;
        }
                printf("\n--------------------\n");
    }
    }

    close(sock);
    return 0;
}