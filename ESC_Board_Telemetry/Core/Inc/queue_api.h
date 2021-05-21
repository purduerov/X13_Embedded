/*
 * queue_api.h
 *
 *  Created on: Apr 16, 2021
 *      Author: Conne
 */

#ifndef INC_QUEUE_API_H_
#define INC_QUEUE_API_H_

//  Include this header file in main.c
//  To ensure ONLY "API" functions are called

#ifndef NULL
	#define NULL (void*)0
#endif

typedef enum
{
	QUEUE_SUCCESS = 0,
	QUEUE_NO_AVAILABLE_QUEUES = 1,
	QUEUE_INVALID_QUEUE_REQUEST_SIZE = 2,
	QUEUE_MAX_QUEUE_SIZE_REACHED = 3,
	QUEUE_EMPTY = 4
} QueueErrorCode;

#if 1
//#ifdef NDEBUG
typedef int queue_handle_t;
#else
typedef struct {
	int handle;
} queue_handle_t;
#endif

void InitializeQueueModule();
QueueErrorCode CreateQueue(void *dataArray, int elementSizeBytes, int numArrayElements, queue_handle_t *queueHandle);
QueueErrorCode AddToQueue(queue_handle_t queueHandle, void const *data);
QueueErrorCode RemoveFromQueue(queue_handle_t queueHandle, void **data);
void FreeQueue(queue_handle_t queueHandle);
int isQueueEmpty(queue_handle_t queueHandle);
int getQueueSize(queue_handle_t queueHandle);

#endif /* INC_QUEUE_API_H_ */
