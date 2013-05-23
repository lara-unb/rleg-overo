/*****************************************************************************
// File: GQueue.h 
// Contents: Header of C functions for circular queue access and control.
// Author: G. A. Borges.
// Description: Circular queues are FIFOS implemented with limited length vectors.
//				Their control variables are defined in a structure named GQUEUECONTROL. 
//				It means that no data is stored in GQUEUECONTROL, but read/write indexes and
//				control variables. The data is stored in a separate array. Indexes to access
//				the array are computed using GQUEUECONTROL internal values.
//				The circular queues may have up to MAXSIZE_QUEUE_READ_GATES reading heads but 
//				only one writing head. 
//				
//				
*****************************************************************************/

#ifndef GQUEUE_H
#define GQUEUE_H

#define TRUE 1
#define FALSE 0

#define	MAXSIZE_QUEUE_READ_GATES	5
#define QUEUE_READ_GATE_0			0
#define QUEUE_READ_GATE_1			1
#define QUEUE_READ_GATE_2			2
#define QUEUE_READ_GATE_3			3
#define QUEUE_READ_GATE_4			4

#define GQUEUE_RTAI_SUPPORT	0

#if GQUEUE_RTAI_SUPPORT
#include <rtai_spl.h>
#endif

typedef struct{
        int        Size;
        int        ReadIndex[MAXSIZE_QUEUE_READ_GATES];
        int        WriteIndex;
#if GQUEUE_RTAI_SUPPORT
        SPL        SPLAtomicAccess;
#endif
        int        NReaders;
        int        FlagStillNotWritten;
        int        FlagStillNotRead[MAXSIZE_QUEUE_READ_GATES];
        int        FlagFull[MAXSIZE_QUEUE_READ_GATES];
}GQUEUECONTROL, *PGQUEUECONTROL;

/* Example:
#define QUEUESIZE	100

double buffer[QUEUESIZE];
GQUEUECONTROL gQueueControlBuffer;

gQUEUE_Init(&gQueueControlBuffer, QUEUESIZE, 1);

int Index;
if(gQUEUE_RequestWriteIndex(&gQueueControlBuffer, &Index)){
	buffer[Index] = 100;
}

int Index;
if(gQUEUE_RequestReadIndex(&gQueueControlBuffer, QUEUE_READ_GATE_0, &Index)){
	x = buffer[Index];
}

*/

/* Prototypes: */
int gQUEUE_Init(PGQUEUECONTROL pQueueControl, int Size, int NReaders);
int gQUEUE_GetReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index, int IndexHorizon);
int gQUEUE_GetWriteIndex(PGQUEUECONTROL pQueueControl, int* Index, int IndexHorizon);
int gQUEUE_RequestReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index);
int gQUEUE_RequestWriteIndex(PGQUEUECONTROL pQueueControl, int* Index);
int gQUEUE_RequestLastReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index);
int gQUEUE_UnwrapWriteIndex(PGQUEUECONTROL pQueueControl);
int gQUEUE_UnwrapReadIndex(PGQUEUECONTROL pQueueControl, int NReader);
int gQUEUE_GetNumberOfUnreadData(PGQUEUECONTROL pQueueControl, int NReader, int *Index);

#endif
