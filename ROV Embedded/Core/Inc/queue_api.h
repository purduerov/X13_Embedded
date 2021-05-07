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

#define NULL (void*)0

typedef enum
{
	QUEUE_SUCCESS = 0,
	QUEUE_NO_AVAILABLE_QUEUES = 1,
	QUEUE_INVALID_QUEUE_REQUEST_SIZE = 2,
	QUEUE_MAX_QUEUE_SIZE_REACHED = 3,
	QUEUE_EMPTY = 4
} QueueErrorCode;

void InitializeQueueModule();
QueueErrorCode CreateQueue(void* dataArray, int elementSizeBytes, int numArrayElements, int* queueHandle);
QueueErrorCode AddToQueue(int queueHandle, void* data);
QueueErrorCode RemoveFromQueue(int queueHandle, void* data);
void FreeQueue(int queueHandle);
int isQueueEmpty(int queueHandle);
int getQueueSize(int queueHandle);

#endif /* INC_QUEUE_API_H_ */
