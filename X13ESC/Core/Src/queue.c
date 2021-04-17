/*
 * queue.c
 *
 *  Created on: Apr 16, 2021
 *      Author: Conne
 */

#include "queue.h"

#define NUM_QUEUES 5
#define NUM_QUEUE_NODES 50

Queue hierarchyQueue;
QueueNode queueHandleNodes[NUM_QUEUES];
int queueHandleNumbers[NUM_QUEUES];

Queue freeNodesQueue;
QueueNode queueNodes[NUM_QUEUE_NODES];

Queue queues[NUM_QUEUES];
Queue freeQueues[NUM_QUEUES];

void InitializeQueueModule()
{
	//  Initialize Hierarchy Queue
	InitializeQueue(&hierarchyQueue, NULL, 0);
	for (int i = 0; i < NUM_QUEUES; i++)
	{
		queueHandleNumbers[i] = i;
		queueHandleNodes[i].data = (void*)&(queueHandleNumbers[i]);
		AddNodeToQueue(&hierarchyQueue, &(queueHandleNodes[i]));
	}

	//  Initialize Free Nodes Queue
	InitializeQueue(&freeNodesQueue, NULL, 0);
	for (int i = 0; i < NUM_QUEUE_NODES; i++)
	{
		AddNodeToQueue(&freeNodesQueue, &(queueNodes[i]));
	}
}

QueueErrorCode CreateQueue(void* dataArray, int elementSizeBytes, int numArrayElements, int* queueHandle)
{
	QueueNode* queueHandleNode;

	if (numArrayElements > freeNodesQueue.size)
	{
		return QUEUE_INVALID_QUEUE_REQUEST_SIZE;
	}

	queueHandleNode = RemoveNodeFromQueue(&hierarchyQueue);
	if (queueHandleNode == NULL)
	{
		return QUEUE_NO_AVAILABLE_QUEUES;
	}
	*queueHandle = *((int*)(queueHandleNode->data));

	InitializeQueue(&(queues[*queueHandle]), dataArray, elementSizeBytes);
	InitializeQueue(&(freeQueues[*queueHandle]), dataArray, elementSizeBytes);
	FillQueue(&(freeQueues[*queueHandle]), numArrayElements);

	return QUEUE_SUCCESS;
}

QueueErrorCode AddToQueue(int queueHandle, void* data)
{
	QueueNode* queueNode;

	//  Remove node from corresponding free node queue
	queueNode = RemoveNodeFromQueue(&(freeQueues[queueHandle]));
	if (queueNode == NULL)
	{
		return QUEUE_MAX_QUEUE_SIZE_REACHED;
	}

	//  Copy Data and node to corresponding queue
	byteCopy((uint8_t*)data, (uint8_t*)queueNode->data, queues[queueHandle].elementSizeBytes);
	AddNodeToQueue(&(queues[queueHandle]), queueNode);

	return QUEUE_SUCCESS;
}

QueueErrorCode RemoveFromQueue(int queueHandle, void** data)
{
	QueueNode* queueNode;

	queueNode = RemoveNodeFromQueue(&(queues[queueHandle]));
	if (queueNode == NULL)
	{
		return QUEUE_EMPTY;
	}

	AddNodeToQueue(&(freeQueues[queueHandle]), queueNode);
	*data = queueNode->data;

	return QUEUE_SUCCESS;
}

void FreeQueue(int queueHandle)
{
	DeinitializeQueue(&(queues[queueHandle]));
	DeinitializeQueue(&(freeQueues[queueHandle]));

	//  Enable Queue Handle to be used by another call
	AddNodeToQueue(&hierarchyQueue, &(queueHandleNodes[queueHandle]));
}

int isQueueEmpty(int queueHandle)
{
	return (queues[queueHandle].size == 0);
}

int getQueueSize(int queueHandle)
{
	return queues[queueHandle].size;
}

void InitializeQueue(Queue* queue, void* dataArray, int elementSizeBytes)
{
	queue->first = NULL;
	queue->last = NULL;
	queue->size = 0;
	queue->dataArray = dataArray;
	queue->elementSizeBytes = elementSizeBytes;
}

QueueErrorCode FillQueue(Queue* queue, int numArrayElements)
{
	uint8_t* dataElement;
	QueueNode* queueNode;

	if (numArrayElements > freeNodesQueue.size)
	{
		return QUEUE_INVALID_QUEUE_REQUEST_SIZE;
	}

	dataElement = (uint8_t*)queue->dataArray;
	for (int i = 0; i < numArrayElements; i++)
	{
		queueNode = RemoveNodeFromQueue(&freeNodesQueue);
		queueNode->data = (void*)dataElement;
		AddNodeToQueue(queue, queueNode);
		dataElement = (uint8_t*)(dataElement + queue->elementSizeBytes);
	}

	return QUEUE_SUCCESS;
}

void DeinitializeQueue(Queue* queue)
{
	QueueNode* queueNode;

	//  Remove all nodes and place in free node queue
	while (queue->first != NULL)
	{
		queueNode = RemoveNodeFromQueue(queue);
		queueNode->data = NULL;
		AddNodeToQueue(&freeNodesQueue, queueNode);
	}

	queue->dataArray = NULL;
	queue->elementSizeBytes = 0;
}

void AddNodeToQueue(Queue* queue, QueueNode* queueNode)
{
	//  If first node
	if (queue->last == NULL)
	{
		queue->first = queueNode;
		queue->last = queueNode;
	}
	else
	{
		(queue->last)->next = queueNode;
		queue->last = queueNode;
	}
	queueNode->next = NULL;
	queue->size++;
}

QueueNode* RemoveNodeFromQueue(Queue* queue)
{
	QueueNode* queueNodeToRemove;

	//  Check if queue is empty
	if (queue->first == NULL)
	{
		return NULL;
	}

	queueNodeToRemove = queue->first;
	queue->first = (queue->first)->next;
	queueNodeToRemove->next = NULL;
	queue->size--;

	return queueNodeToRemove;
}

void byteCopy(uint8_t* source, uint8_t* dest, int numBytes)
{
	for (int i = 0; i < numBytes; i++)
	{
		dest[i] = source[i];
	}
}
