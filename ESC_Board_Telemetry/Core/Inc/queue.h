/*
 * queue.h
 *
 *  Created on: Apr 16, 2021
 *      Author: Conner
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include "queue_api.h"
#include "stm32f0xx_hal.h"

typedef struct _QueueNode
{
	void* data;
	struct _QueueNode* next;
} QueueNode;

typedef struct
{
	QueueNode* first;
	QueueNode* last;
	int size;

	void* dataArray;
	int elementSizeBytes;
} Queue;

void InitializeQueue(Queue* queue, void* dataArray, int elementSizeBytes);
QueueErrorCode FillQueue(Queue* queue, int numArrayElements);
void DeinitializeQueue(Queue* queue);
void AddNodeToQueue(Queue* queue, QueueNode* queueNode);
QueueNode* RemoveNodeFromQueue(Queue* queue);

void byteCopy(uint8_t* source, uint8_t* dest, int numBytes);

#endif /* INC_QUEUE_H_ */
