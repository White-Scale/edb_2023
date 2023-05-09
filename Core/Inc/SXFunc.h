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

extern int message ;
extern int message_length;
extern int SX_ret;

typedef struct packet{
	uint8_t type;
	uint8_t src;
	uint8_t dst;
//	uint8_t crc;
	uint8_t length;
	uint8_t seq;
	uint8_t reserve;
	uint8_t content[0];
} SX_packet;

#define CRC_BYTE(x) (((x)->content )[( ( (x)->length ) - sizeof(SX_packet) - 1 ) ])

#if defined(send)
#define MESH_ADDR_LOCAL 1
#define MESH_ADDR_REMOTE 2
#else
#define MESH_ADDR_LOCAL 2
#define MESH_ADDR_REMOTE 1
#endif

#define TYPE_SOH 1
#define TYPE_ACK 2
#define TYPE_NAK 3

void SX_init();
void SX_send(SX_packet *pac);
int SX_recv(SX_packet *pac, unsigned int time_out, unsigned freq);
void SX_sender();
void SX_receiver();

#endif /* INC_SXFUNC_H_ */
