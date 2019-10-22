/*
 * can_cmd.c
 *
 *  Created on: 23 авг. 2019 г.
 *      Author: User
 */

#include "can_cmd.h"
#include "can_tx_stack.h"
#include "main.h"

extern uint8_t group_id;
extern uint8_t pos_in_group;

extern tx_stack can1_tx_stack;
extern tx_stack can2_tx_stack;


extern uint16_t discrete_state;
extern uint8_t pow_data[2];

void send_write_ack(uint8_t id) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = BOOT_WRITE_ACK;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = BOOT;
	p_id->param = 0;
	packet.length = 1;
	packet.data[0] = id;
	add_tx_can_packet(&can1_tx_stack,&packet);
	//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}

void send_erase_ack(uint8_t num) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = BOOT_ERASE_PAGE_ACK;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = BOOT;
	p_id->param = num;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
	//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}

void send_point_state(uint8_t can_num) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = POINT_STATE;
	p_id->param = 0;
	packet.length = 5;
	packet.data[0] = 0;
	packet.data[1] = 0;
	packet.data[2] = 0;
	packet.data[3] = 0;
	packet.data[4] = 201; // version 201-255 for bootloader (0-200 voip point)
	// добавить состояние входов выходов
	if(can_num==1) add_tx_can_packet(&can1_tx_stack,&packet);
	else add_tx_can_packet(&can2_tx_stack,&packet);
}

void next_point(uint8_t t) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = FIND_NEXT_POINT;
	p_id->param = t;
	packet.length = 1;
	packet.data[0] = pos_in_group;
	if(t==1) add_tx_can_packet(&can2_tx_stack,&packet);	// request
	else add_tx_can_packet(&can1_tx_stack,&packet);	// answer
}

void last_point() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = pos_in_group;
	p_id->group_addr = group_id;
	p_id->cmd = LAST_POINT;
	p_id->param = 0;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
}
