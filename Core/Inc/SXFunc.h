/*
 * SXFunc.h
 *
 *  Created on: May 4, 2023
 *      Author: zhangzhihan
 */

#ifndef INC_SXFUNC_H_
#define INC_SXFUNC_H_

#include "sx1278.h"

//#define send

extern SX1278_hw_t sx1278_hw;
extern SX1278_t sx1278;
extern char buffer[];
extern char buffer2[];

extern int message;
extern int message_length;
extern int SX_ret;

/*all the address ,0 and 255 is reserved */
typedef struct packet {
	uint8_t type; /* the type of the packet */
	uint8_t src; /* the original source address */
	uint8_t dst; /* the final destination address */
	uint8_t length; /* the TOTAL length of the packet */
	uint8_t seq; /* the sequence number of the packet */
	uint8_t reserve; /* no use */
	uint8_t last_hop; /* the address where THIS specific packet was sent from */
	uint8_t next_hop; /* the next address where THIS packet will be pocess */
	uint8_t content[0];
//	uint8_t crc; the crc is added after all the contents
} SX_packet;

#define CRC_BYTE(x) (((x)->content )[( ( (x)->length ) - sizeof(SX_packet) - 1 ) ])

#if defined(send)
#define MESH_ADDR_LOCAL 1
#define MESH_ADDR_REMOTE 2
#else
#define MESH_ADDR_LOCAL 2
#define MESH_ADDR_REMOTE 1
#endif

#define MAX_DELAY_TIME 0xFFFF

#define TYPE_SOH 1
#define TYPE_ACK 2
#define TYPE_NAK 3

void SX_init();
void SX_send(SX_packet *pac);
int SX_recv(SX_packet *pac, unsigned int time_out, unsigned freq);
void SX_sender();
void SX_receiver();
//int SX_check_packet(SX_packet *pac2);
int SX_check_packet(SX_packet *pac2, uint8_t target_type);
int SX_recv_once(SX_packet *pac);

#endif /* INC_SXFUNC_H_ */
