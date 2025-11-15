/*
 * main_app.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#include "main_app.h"

#define CAN_QUEUE_CAPACITY 64

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t dlc;
	bool isExtended;
	bool isRemote;
} CanPacket;

static Queue can1_txQueue;
static CanPacket can1_txBuffer[CAN_QUEUE_CAPACITY];
static Queue can2_txQueue;
static CanPacket can2_txBuffer[CAN_QUEUE_CAPACITY];

static void task1kHz();
static void can1_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can2_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can1_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can2_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);

static CanPacket can2_rxPacket = {0};

void setup() {
	timer_startUs();

	Queue_init(&can1_txQueue, can1_txBuffer, sizeof(CanPacket), CAN_QUEUE_CAPACITY, false, disable_irq_nest, enable_irq_nest);
	Queue_init(&can2_txQueue, can2_txBuffer, sizeof(CanPacket), CAN_QUEUE_CAPACITY, false, disable_irq_nest, enable_irq_nest);

	HAL_Delay(500);

	timer_set1kHzTask(task1kHz);
	timer_start1kHzTask();

	can1_setReceivedCallback(can1_rxCallback);
	can2_setReceivedCallback(can2_rxCallback);
	can1_start(&CAN1_FILTER_DEFAULT);
	can2_start(&CAN2_FILTER_DEFAULT);

	UNUSED(can2_transmitQueue);
}

void loop() {
	CanPacket canPacket;
	if (Queue_size(&can1_txQueue) > 0 && can1_txAvailable() > 0) {
		if (Queue_pop(&can1_txQueue, &canPacket) == 0) {
			can1_transmit(canPacket.id, canPacket.data, canPacket.dlc, canPacket.isExtended, canPacket.isRemote);
		}
	}
	if (Queue_size(&can2_txQueue) > 0 && can2_txAvailable() > 0) {
		if (Queue_pop(&can2_txQueue, &canPacket) == 0) {
			can2_transmit(canPacket.id, canPacket.data, canPacket.dlc, canPacket.isExtended, canPacket.isRemote);
		}
	}
}

static void task1kHz() {
	static uint32_t tick = 0;

	gpio_setLedR((tick % 1000 < 500) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	tick++;
}

static void can1_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {

}

static void can2_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	can2_rxPacket.id = id;
	memcpy(can2_rxPacket.data, data, dlc);
	can2_rxPacket.dlc = dlc;
	can2_rxPacket.isExtended = isExtended;
	can2_rxPacket.isRemote = isRemote;
}

static void can1_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (dlc > 8) return;

	CanPacket packet;
	packet.id = id;
	memcpy(packet.data, data, dlc);
	packet.dlc = dlc;
	packet.isExtended = isExtended;
	packet.isRemote = isRemote;

	Queue_push(&can1_txQueue, &packet);
}

static void can2_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (dlc > 8) return;

	CanPacket packet;
	packet.id = id;
	memcpy(packet.data, data, dlc);
	packet.dlc = dlc;
	packet.isExtended = isExtended;
	packet.isRemote = isRemote;

	Queue_push(&can2_txQueue, &packet);
}
