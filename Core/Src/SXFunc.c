/*
 * SXFunc.c
 *
 *  Created on: May 4, 2023
 *      Author: zhangzhihan
 */
#include "SXFunc.h"
#include "sx1278.h"
#include "usart.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

SX1278_hw_t sx1278_hw = { { SX_RST_Pin, SX_RST_GPIO_Port },     // rst
		{ SX_D0_Pin, SX_D0_GPIO_Port },       // dio0
		{ SX_NSS_Pin, SX_NSS_GPIO_Port },     // nss
		&hspi1 };
SX1278_t sx1278 = { &sx1278_hw,         // SX1278_hw_t *hw;
		};
char buffer[512];
char buffer2[512];

int message = 0;
int message_length;
int SX_ret;

int isRx = 0;

unsigned char CRC8(unsigned char *ptr, unsigned char len) {

	unsigned char crc;
	unsigned char i;
	crc = 0;
	while (len--) {
		crc ^= *ptr++;
		for (i = 0; i < 8; i++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else
				crc <<= 1;
		}
	}
	return crc;

}

void SX_init() {
	printf("Configuring LoRa module\r\n");
	SX1278_init(&sx1278, 434000000, SX1278_POWER_11DBM, SX1278_LORA_SF_7,
	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
	printf("Done configuring LoRaModule\r\n");
}

void SX_generate_packet(SX_packet *pac, uint8_t type, char *buf, uint8_t src,
		uint8_t dst, uint8_t seq, uint8_t next_hop) {
	log("generate packet");
	int len_content = 0;
	pac->type = type;
	if (type == TYPE_SOH) {
		//move the buffer to the packet content
		len_content = sprintf((char*) pac->content, "%s", buf);
	} else if (type == TYPE_ACK) {
		//set the content as a empty string,but maybe actually not necessary
		*(pac->content) = '\0';
		len_content = 0;
	}
	//set attributes of the packet to send
	pac->length = len_content + sizeof(SX_packet) + 1;
	pac->reserve = 0;
	pac->src = src;
	pac->dst = dst;
	pac->seq = seq;
	pac->last_hop = MESH_ADDR_LOCAL;
	pac->next_hop = next_hop;
	//set CRC
	CRC_BYTE(pac) = CRC8((uint8_t*) pac, pac->length - 1);
}
void SX_print(SX_packet *pac) {
	uint8_t crc = CRC_BYTE(pac);
	CRC_BYTE(pac) = 0;
	printf(
			"type:\t%d\r\nsrc:\t%d\r\ndst:\t%d\r\nlength:\t%d\r\nseq:\t%d\r\nlast_hop:\t%d\r\nnext_hop:\t%d\r\ncontent:\t%s\r\n",
			pac->type, pac->src, pac->dst, pac->length, pac->seq, pac->last_hop,
			pac->next_hop, pac->content);
	CRC_BYTE(pac) = crc;
}

void SX_send(SX_packet *pac) {
//	HAL_Delay(1000);
	log("Sending package...");

	message_length = pac->length;
	SX_ret = SX1278_LoRaEntryTx(&sx1278, message_length, 2000);
	isRx = 0;
	log("Entry: %d", SX_ret);
	uint8_t *tmp = (uint8_t*) pac;
	log("sending a packet with header : %d %d %d %d %d %d", tmp[0], tmp[1],
			tmp[2], tmp[3], tmp[4], tmp[5]);
	uint8_t crc = CRC_BYTE(pac);
	CRC_BYTE(pac) = 0;
	log("sending content %s", (char* )pac->content);
	CRC_BYTE(pac) = crc;
	SX_ret = SX1278_LoRaTxPacket(&sx1278, (uint8_t*) pac, message_length, 2000);

	log("Transmission: %d", SX_ret);
	log("Package sent...");
}

int SX_recv(SX_packet *pac, unsigned int time_out, unsigned freq) {
	log("Receiving package...");
	int cnt = 0;
	SX_ret = SX1278_LoRaEntryRx(&sx1278, 16, 2000);
	isRx = 1;
	//if max delay time,just wait until a packet arrive
	while (time_out == MAX_DELAY_TIME ? 1 : (cnt < time_out)) {
		SX_ret = SX1278_LoRaRxPacket(&sx1278);
//		log("Received: %d", SX_ret);
		if (SX_ret > 0) {
			SX1278_read(&sx1278, (uint8_t*) pac, SX_ret);
			uint8_t crc = CRC_BYTE(pac);
			uint8_t *tmp = (uint8_t*) pac;
			log("received a packet with header : %d %d %d %d %d %d", tmp[0],
					tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
			CRC_BYTE(pac) = 0;
			log("Content (%d): %s", SX_ret, (char* )pac->content);
			CRC_BYTE(pac) = crc;
			log("Package received ...");
			return SX_ret;
		}
//		HAL_Delay(freq);
		//use RTOS 's delay to wait
		vTaskDelay(freq);
		cnt += freq;
	}
	log("Received: %d,nothing", SX_ret);
	return SX_ret;
}
int SX_recv_once(SX_packet *pac) {
//	log("checking buffer..");
	if (!isRx) {
		SX_ret = SX1278_LoRaEntryRx(&sx1278, 16, 2000);
		isRx = 1;
	}
	//if max delay time,just wait until a packet arrive
	SX_ret = SX1278_LoRaRxPacket(&sx1278);
	if (SX_ret > 0) {
		SX1278_read(&sx1278, (uint8_t*) pac, SX_ret);
		uint8_t crc = CRC_BYTE(pac);
		uint8_t *tmp = (uint8_t*) pac;
		log("received a packet with header : %d %d %d %d %d %d", tmp[0], tmp[1],
				tmp[2], tmp[3], tmp[4], tmp[5]);
		CRC_BYTE(pac) = 0;
		log("Content (%d): %s", SX_ret, (char* )pac->content);
		CRC_BYTE(pac) = crc;
		log("Package received ...");
		return SX_ret;
	}
//	log("Received: %d,nothing", SX_ret);
	return SX_ret;
}

void SX_sender() {
	SX_packet *pac = (SX_packet*) buffer;
	SX_packet *pac2 = (SX_packet*) buffer2;
	int seq = 0;
	int time_out = 2000;
	int ack = 0;
	while (1) {
		//set the packet's attributes
		pac->type = TYPE_SOH;
		pac->src = MESH_ADDR_LOCAL;
		pac->dst = MESH_ADDR_REMOTE;
		pac->seq = seq;
		//fulfill the content
		int len_content = sprintf((char*) pac->content, "Message %d", seq);
		if (len_content < 0) {
			log("sprintf() failed!");
		}
		pac->reserve = 0;
		pac->length = len_content + sizeof(SX_packet) + 1;
		CRC_BYTE(pac) = CRC8((uint8_t*) pac, pac->length - 1);
		//send the packet
		SX_send(pac);

		//switch to the receive mode
		SX_ret = SX1278_LoRaEntryRx(&sx1278, 16, 2000);

		//wait 2s for reply
//		HAL_Delay(time_out);
		int recv_success = SX_recv(pac2, 2000, 50);
		if (recv_success) {
			uint8_t *tmp = (uint8_t*) pac2;
			log("%d %d %d %d %d %d %d", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4],
					tmp[5], tmp[6]);
		}
		if (!recv_success) {
			log("no packet received");
			ack = 0;
		} else if (CRC_BYTE(pac2) != CRC8((uint8_t*) pac2, pac2->length - 1)) {
			log("crc wrong");
			log("CRC: %d %d", CRC_BYTE(pac2),
					CRC8((uint8_t* ) pac2, pac2->length - 1));
			ack = 0;
		} else if (pac2->dst != MESH_ADDR_LOCAL) {
			log("dst not this node");
			ack = 0;
		} else if (pac2->type != TYPE_ACK) {
			log("not a ACK packet");
			ack = 0;
		} else {
			log("a good packet");
			ack = 1;
		}
		if (ack) {
			seq++;
		}
		log("now the seq is %d", seq);
	}
}
/**
 * the return value of check should have its meaning
 * such as 1 of a good packet
 * 2 for dst no this node
 * 3 for not a specific packet
 */
int SX_check_packet(SX_packet *pac2, uint8_t target_type) {
	int ack = 1;
	if (CRC_BYTE(pac2) != CRC8((uint8_t*) pac2, pac2->length - 1)) {
		log("CRC: %d %d", CRC_BYTE(pac2),
				CRC8((uint8_t* ) pac2, pac2->length - 1));
		log("CRC wrong");
		ack = 0;
	} else if (pac2->dst != MESH_ADDR_LOCAL) {
		log("dst not this node");
		ack = 0;
	} else if (pac2->type != target_type) {
		log("not a target %d packet", target_type);
		ack = 0;
	} else {
		log("a good type : %d packet", target_type);
		ack = 1;
	}
	return ack;
}

void SX_receiver() {
	log("sizeof(SX_packet): %d", sizeof(SX_packet));
	SX_packet *pac = (SX_packet*) buffer;
	SX_packet *pac2 = (SX_packet*) buffer2;
	int seq = 0;
	int time_out = 2000;
	int ack = 0;
	while (1) {

		//switch to the receive mode
		SX_ret = SX1278_LoRaEntryRx(&sx1278, 16, 2000);

		//wait 2s for reply
//		HAL_Delay(time_out);
		int recv_success = SX_recv(pac2, 2000, 50);
		if (recv_success) {
			uint8_t *tmp = (uint8_t*) pac2;
			log("%d %d %d %d %d %d %d", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4],
					tmp[5], tmp[6]);
		}
		if (!recv_success) {
			log("no packet received");
			ack = 0;
		} else if (CRC_BYTE(pac2) != CRC8((uint8_t*) pac2, pac2->length - 1)) {
			log("CRC: %d %d", CRC_BYTE(pac2),
					CRC8((uint8_t* ) pac2, pac2->length - 1));
			log("crc wrong");
			ack = 0;
		} else if (pac2->dst != MESH_ADDR_LOCAL) {
			log("dst not this node");
			ack = 0;
		} else if (pac2->type != TYPE_SOH) {
			log("not a SOH packet");
			ack = 0;
		} else {
			log("a good packet");
			ack = 1;
		}
		if (ack) {
			pac->type = TYPE_ACK;
		} else {
			pac->type = TYPE_NAK;
		}

		//set the packet's attributes
		pac->src = MESH_ADDR_LOCAL;
		pac->dst = MESH_ADDR_REMOTE;
		pac->seq = seq;
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
