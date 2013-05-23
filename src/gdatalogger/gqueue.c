/*****************************************************************************
// File: GQueue.c 
// Contents: C functions for circular queue access and control.
// Author: G. A. Borges.
*****************************************************************************/

#include <math.h>
#include <stdio.h>

/* GQueue */
#include "gqueue.h"

#if GQUEUE_RTAI_SUPPORT
	#define BEGIN_ATOMIC() 	rt_spl_lock(&pQueueControl->SPLAtomicAccess)
	#define END_ATOMIC()	rt_spl_unlock(&pQueueControl->SPLAtomicAccess)
#else
	#define BEGIN_ATOMIC() 	
	#define END_ATOMIC()	
#endif

/*****************************************************************************
*** int gQUEUE_Init(PGQUEUECONTROL pQueueControl, int Size, int NReaders)
*** Purpose: Initialize GQUEUECONTROL structure.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- Size: length of the vector where data is stored
***		- NReaders: number of reading heads of the queue
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_Init(PGQUEUECONTROL pQueueControl, int Size, int NReaders)
{
	int n;

	if(NReaders > MAXSIZE_QUEUE_READ_GATES){
		return FALSE;
	}

	pQueueControl->Size = Size,
	pQueueControl->NReaders = NReaders;
	pQueueControl->FlagStillNotWritten = TRUE;
	for(n=0;n<NReaders;++n){
		pQueueControl->ReadIndex[n] = 0;
		pQueueControl->FlagStillNotRead[n] = TRUE;
		pQueueControl->FlagFull[n] = FALSE;
	}
	pQueueControl->WriteIndex = 0;

#if GQUEUE_RTAI_SUPPORT
	rt_spl_init(&pQueueControl->SPLAtomicAccess);
#endif
    return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_RequestLastReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index)
*** Purpose: Request the last read index, i.e., the read index corresponding the most recent
***			 data stored in the queue. The corresponding reading pointer of GQUEUECONTROL 
***			 is updated to this value.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- NReader: number of reading head.
***		- *Index: pointer to an int variable on which the reading index is returned
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_RequestLastReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index)
{
	if(NReader>=pQueueControl->NReaders){
		return FALSE;
	}
	
	if(pQueueControl->FlagStillNotWritten == TRUE){
		return FALSE;
	}

BEGIN_ATOMIC();
	
	if(pQueueControl->FlagStillNotRead[NReader] == TRUE){
		pQueueControl->ReadIndex[NReader] = pQueueControl->WriteIndex;
		*Index = pQueueControl->ReadIndex[NReader];
		pQueueControl->FlagStillNotRead[NReader] = FALSE;
	}
	else{
		if(pQueueControl->ReadIndex[NReader] == pQueueControl->WriteIndex){
			pQueueControl->FlagFull[NReader] = FALSE;	
			*Index = -1;
		}
		else{
			pQueueControl->ReadIndex[NReader] = pQueueControl->WriteIndex;
			gQUEUE_UnwrapReadIndex(pQueueControl,NReader);
			*Index = pQueueControl->ReadIndex[NReader];
		}
	}

END_ATOMIC();

	if(*Index==-1){
		return FALSE;
	}  

    return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_RequestReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index)
*** Purpose: Request the next read index, i.e., the read index corresponding the next data 
***			 to be read according to the storage sequence. The corresponding reading 
***			 pointer of GQUEUECONTROL is updated to this value.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- NReader: number of reading head.
***		- *Index: pointer to an int variable on which the reading index is returned
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_RequestReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index)
{
	if(NReader>=pQueueControl->NReaders){
		return FALSE;
	}

	if(pQueueControl->FlagStillNotWritten == TRUE){
		return FALSE;
	}
    
BEGIN_ATOMIC();
	if(pQueueControl->FlagStillNotRead[NReader] == TRUE){
		*Index = pQueueControl->ReadIndex[NReader];
	        pQueueControl->FlagStillNotRead[NReader] = FALSE;
    	}
	else{
		if(pQueueControl->ReadIndex[NReader] == pQueueControl->WriteIndex){
			pQueueControl->FlagFull[NReader] = FALSE;	
			*Index = -1;
		}
		else{
			++pQueueControl->ReadIndex[NReader];
			gQUEUE_UnwrapReadIndex(pQueueControl,NReader);
			*Index = pQueueControl->ReadIndex[NReader];
		}
	}
END_ATOMIC();

	if(*Index==-1){
		return FALSE;
	}  

    return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_GetReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index, int IndexHorizon)
*** Purpose: Get the index corresponding the next-IndexHorizon data 
***			 to be read according to the storage sequence. The corresponding reading 
***			 pointer of GQUEUECONTROL is NOT updated to this value. IndexHorizon corresponds to
***			 data which have already been read from the queue
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- NReader: number of reading head.
***		- *Index: pointer to an int variable on which the reading index is returned
***     - IndexHorizon: past time horizon. If zero, Index is pointed to the next data to be read.
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_GetReadIndex(PGQUEUECONTROL pQueueControl, int NReader, int* Index, int IndexHorizon)
{
	if(NReader>=pQueueControl->NReaders){
		return FALSE;
	}

	if(pQueueControl->FlagStillNotWritten == TRUE){
        	return FALSE;
    	}
    
BEGIN_ATOMIC();

	*Index = (pQueueControl->ReadIndex[NReader]-IndexHorizon);
	if(*Index<0){
		*Index += pQueueControl->Size;
	}
	gQUEUE_UnwrapReadIndex(pQueueControl, NReader);

END_ATOMIC();


    return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_GetWriteIndex(PGQUEUECONTROL pQueueControl, int* Index, int IndexHorizon)
*** Purpose: Get the index corresponding the next-IndexHorizon data 
***			 to be write according to the storage sequence. The corresponding writing 
***			 pointer of GQUEUECONTROL is NOT updated to this value. IndexHorizon corresponds to
***			 data which have already been stored to the queue
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- *Index: pointer to an int variable on which the reading index is returned
***     - IndexHorizon: past time horizon. If zero, Index is pointed to the next data to be written.
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_GetWriteIndex(PGQUEUECONTROL pQueueControl, int* Index, int IndexHorizon)
{

    if(pQueueControl->FlagStillNotWritten == TRUE){
        return FALSE;
    }

BEGIN_ATOMIC();
    
	*Index = (pQueueControl->WriteIndex-IndexHorizon);
	if(*Index<0){
		*Index += pQueueControl->Size;
	}
	gQUEUE_UnwrapWriteIndex(pQueueControl);

END_ATOMIC();

    return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_RequestWriteIndex(PGQUEUECONTROL pQueueControl, int* Index)
*** Purpose: Request the next data storage index in the queue. The corresponding writing 
***			 pointer of GQUEUECONTROL is updated to this value.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- *Index: pointer to an int variable on which the writing index is returned
***	Output:
***		- TRUE if sucess, FALSE if the queue is full for some reading head.	
*****************************************************************************/
int gQUEUE_RequestWriteIndex(PGQUEUECONTROL pQueueControl, int* Index)
{
	int n;
	int	FlagNotFull = TRUE;

BEGIN_ATOMIC();
    
    if(pQueueControl->FlagStillNotWritten == TRUE){
        *Index = pQueueControl->WriteIndex;
        pQueueControl->FlagStillNotWritten = FALSE;
        }
    else{
		/* Increment write index in two positions to test if queue is full */
        pQueueControl->WriteIndex += 2; 
        gQUEUE_UnwrapWriteIndex(pQueueControl);
        
		for(n=0;n<pQueueControl->NReaders;++n){    
			if(pQueueControl->ReadIndex[n] == pQueueControl->WriteIndex){
				pQueueControl->FlagFull[n] = TRUE;
				FlagNotFull = FALSE;
			}
		}

		/* Decrement write index of one position to return to the correct state */
        pQueueControl->WriteIndex += pQueueControl->Size-1; 
        gQUEUE_UnwrapWriteIndex(pQueueControl);

		*Index = pQueueControl->WriteIndex;
    }

END_ATOMIC();

    return FlagNotFull;
}                      

/*****************************************************************************
*** int gQUEUE_UnwrapReadIndex(PGQUEUECONTROL pQueueControl, int NReader)
*** Purpose: Unwrap the read index in such way that it points to any valid value in the queue.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- NReader: number of the reading head.
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_UnwrapReadIndex(PGQUEUECONTROL pQueueControl, int NReader)
{
	if(NReader>=pQueueControl->NReaders){
		return FALSE;
	}

    	pQueueControl->ReadIndex[NReader] = (pQueueControl->ReadIndex[NReader]) % (pQueueControl->Size); 
  
	return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_UnwrapWriteIndex(PGQUEUECONTROL pQueueControl)
*** Purpose: Unwrap the write index in such way that it points to any valid value in the queue.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- NReader: number of the reading head.
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_UnwrapWriteIndex(PGQUEUECONTROL pQueueControl)
{
	pQueueControl->WriteIndex = (pQueueControl->WriteIndex) % (pQueueControl->Size); 
	
	return TRUE;
}                      

/*****************************************************************************
*** int gQUEUE_GetNumberOfUnreadData(PGQUEUECONTROL pQueueControl, int NReader, int *Index)
*** Purpose: Returns the number of unread data stored associated to a given reading head.
*** Input: 
***		- pQueueControl: pointer to GQUEUECONTROL structure associated to the queue
***		- NReader: number of the reading head
***		- *Index: pointer to an int where the number of unread data is stored
***	Output:
***		- TRUE if sucess, FALSE if failure.	
*****************************************************************************/
int gQUEUE_GetNumberOfUnreadData(PGQUEUECONTROL pQueueControl, int NReader, int *Index)
{
	if(NReader>=pQueueControl->NReaders){
		return FALSE;
	}

	if(pQueueControl->WriteIndex >= pQueueControl->ReadIndex[NReader]){
		*Index = pQueueControl->WriteIndex-pQueueControl->ReadIndex[NReader];
	}
	else{
		*Index = pQueueControl->Size-(pQueueControl->ReadIndex[NReader]-pQueueControl->WriteIndex);
	}
	
	return TRUE;
}                      
