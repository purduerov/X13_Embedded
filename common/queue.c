/*
 * queue.c
 *
 *  Created on: Apr 16, 2021
 *      Author: Conner
 */

#include "queue.h"
#include <stdint.h>
#include <string.h>

#define NUM_QUEUES 5
#define NUM_QUEUE_NODES 50

#define I(handleName) (handleName).handle

typedef struct _QueueNode
{
	void *data;
	struct _QueueNode* next;
} QueueNode;

typedef struct
{
	QueueNode *first;
	QueueNode *last;
	int size;

	void* dataArray;
	int elementSizeBytes;
} Queue;

static void InitializeQueue(Queue *queue, void *dataArray, int elementSizeBytes);
static QueueErrorCode FillQueue(Queue *queue, int numArrayElements);
static void DeinitializeQueue(Queue *queue);
static void AddNodeToQueue(Queue *queue, QueueNode *queueNode);
static QueueNode* RemoveNodeFromQueue(Queue *queue);

Queue hierarchyQueue;
QueueNode queueHandleNodes[NUM_QUEUES];
queue_handle_t queueHandles[NUM_QUEUES];

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
		I(queueHandles[i]) = i;
		queueHandleNodes[i].data = (void*)&(queueHandles[i]);
		AddNodeToQueue(&hierarchyQueue, &(queueHandleNodes[i]));
	}

	//  Initialize Free Nodes Queue
	InitializeQueue(&freeNodesQueue, NULL, 0);
	for (int i = 0; i < NUM_QUEUE_NODES; i++)
	{
		AddNodeToQueue(&freeNodesQueue, &(queueNodes[i]));
	}
}

QueueErrorCode CreateQueue(void *dataArray, int elementSizeBytes, int numArrayElements, queue_handle_t *queueHandle)
{
	QueueNode *queueHandleNode;

	if (numArrayElements > freeNodesQueue.size)
	{
		return QUEUE_INVALID_QUEUE_REQUEST_SIZE;
	}

	queueHandleNode = RemoveNodeFromQueue(&hierarchyQueue);
	if (queueHandleNode == NULL)
	{
		return QUEUE_NO_AVAILABLE_QUEUES;
	}
	*queueHandle = *((queue_handle_t*)(queueHandleNode->data));

	InitializeQueue(&(queues[I(*queueHandle)]), dataArray, elementSizeBytes);
	InitializeQueue(&(freeQueues[I(*queueHandle)]), dataArray, elementSizeBytes);
	FillQueue(&(freeQueues[I(*queueHandle)]), numArrayElements);

	return QUEUE_SUCCESS;
}

QueueErrorCode AddToQueue(queue_handle_t queueHandle, void const *data)
{
	QueueNode *queueNode;

	//  Remove node from corresponding free node queue
	queueNode = RemoveNodeFromQueue(&(freeQueues[I(queueHandle)]));
	if (queueNode == NULL)
	{
		return QUEUE_MAX_QUEUE_SIZE_REACHED;
	}

	//  Copy Data and node to corresponding queue
	memcpy(queueNode->data, data, queues[I(queueHandle)].elementSizeBytes);
	AddNodeToQueue(&(queues[I(queueHandle)]), queueNode);

	return QUEUE_SUCCESS;
}

QueueErrorCode RemoveFromQueue(queue_handle_t queueHandle, void **data)
{
	QueueNode *queueNode;

	queueNode = RemoveNodeFromQueue(&(queues[I(queueHandle)]));
	if (queueNode == NULL)
	{
		return QUEUE_EMPTY;
	}

	AddNodeToQueue(&(freeQueues[I(queueHandle)]), queueNode);
	*data = queueNode->data;

	return QUEUE_SUCCESS;
}

void FreeQueue(queue_handle_t queueHandle)
{
	DeinitializeQueue(&(queues[I(queueHandle)]));
	DeinitializeQueue(&(freeQueues[I(queueHandle)]));

	//  Enable Queue Handle to be used by another call
	AddNodeToQueue(&hierarchyQueue, &(queueHandleNodes[I(queueHandle)]));
}

int isQueueEmpty(queue_handle_t const queueHandle)
{
	return (queues[I(queueHandle)].size == 0);
}

int getQueueSize(queue_handle_t const queueHandle)
{
	return queues[I(queueHandle)].size;
}

void InitializeQueue(Queue *queue, void *dataArray, int elementSizeBytes)
{
	queue->first = NULL;
	queue->last = NULL;
	queue->size = 0;
	queue->dataArray = dataArray;
	queue->elementSizeBytes = elementSizeBytes;
}

QueueErrorCode FillQueue(Queue *queue, int numArrayElements)
{
	uint8_t *dataElement;
	QueueNode *queueNode;

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

void DeinitializeQueue(Queue *queue)
{
	QueueNode *queueNode;

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

void AddNodeToQueue(Queue *queue, QueueNode *queueNode)
{
	//  If first node
	if (queue->first == NULL)
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

QueueNode *RemoveNodeFromQueue(Queue *queue)
{
	QueueNode *queueNodeToRemove;

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
