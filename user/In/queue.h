#ifndef __QUEUE__H
#define __QUEUE__H

#define True  1
#define False 0




#define QUEUE_DATA_T  unsigned char         //队列数据类型定义

typedef unsigned char ElemType;

// 定义循环队列结构体
typedef struct Queue{
	
	unsigned int front;	  // 队列头下标
	unsigned int rear;	  // 队列尾下标
	unsigned int MAXSIZE; //队列缓存长度（初始化时赋值）
	QUEUE_DATA_T *data;
}Queue;


extern Queue LPUart_revQueue;                          //GPS接收缓存队列
extern Queue Uart_revQueuebuff;


void InitQueue(Queue* q,QUEUE_DATA_T * buffer, unsigned int len);

int DeQueue(Queue* , ElemType *);
int EnQueue(Queue* , ElemType );
int QueueLength(Queue *q);

#endif




