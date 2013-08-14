/**
 * Implentation of Scheduler for tasks:
 * @author Rafael
 * @todo test it
 */

#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef TASK_SCHEDULER_H_INCLUDED
#define TASK_SCHEDULER_H_INCLUDED

/**
 * @defgroup taskS Scheduler for tasks
 */

/**
 * Task to schedule definition
 */
typedef struct taskS{
  timer_t timer;
  volatile double t_global;
  volatile double T_exec_global;
  volatile double T_mean_global;
  volatile double T_max_global;
  volatile double T_min_global;
  volatile int period_us;
  volatile int isFirstExecution; ///< Flag to sign first Exec
  
  void (*run)(int); ///< hook to the action 
} TASK_S;

/**
 * Create a task
 * @ingroup taskS
 * @param *task
 * @param *voidFunction Pointer to the function called to execute
 */
void timer_new_task(TASK_S *task,void (*runFunction)(int));

/**
 * Start some task
 * @ingroup taskS
 */
void timer_start_task(TASK_S *task);

/**
 * Stop some task
 * @ingroup taskS
 * @todo verify use of variable 'erno'
 */
void timer_stop_task(TASK_S *task);
#endif
