/*
 * queue.h
 *
 *  Created on: Apr 16, 2021
 *      Author: Conner
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include "stm32f0xx_hal.h"

typedef enum
{
	QUEUE_SUCCESS = 0,
	QUEUE_NO_AVAILABLE_QUEUES = 1,
	QUEUE_INVALID_QUEUE_REQUEST_SIZE = 2,
	QUEUE_MAX_QUEUE_SIZE_REACHED = 3,
	QUEUE_EMPTY = 4,
	QUEUE_ERROR_CODE_COUNT
} QueueErrorCode;

typedef struct _queue_handle_t
{
	uint8_t handle;
} queue_handle_t;
// See https://floooh.github.io/2018/06/17/handles-vs-pointers.html for rationale on using handles.

void InitializeQueueModule(void);
QueueErrorCode CreateQueue(void *dataArray, size_t elementSizeBytes, int numArrayElements, queue_handle_t *queueHandle);
QueueErrorCode AddToQueue(queue_handle_t queueHandle, void const *data);
QueueErrorCode RemoveFromQueue(queue_handle_t queueHandle, void *data);
void FreeQueue(queue_handle_t queueHandle);
int isQueueEmpty(queue_handle_t queueHandle);
int getQueueSize(queue_handle_t queueHandle);

#endif /* INC_QUEUE_H_ */
