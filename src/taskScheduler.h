/**
 * Implentation of Scheduler for tasks:
 * @author Rafael Lima
 * @todo test it
 */

#ifndef TASK_SCHEDULER_H_INCLUDED
#define TASK_SCHEDULER_H_INCLUDED

#include <time.h>
#include <sys/time.h>

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
  
  void (*run)(); ///< hook to the action 
} TASK_S;

/**
 * Create a task
 * @ingroup taskS
 * @param *task
 * @param *voidFunction Pointer to the function called to execute
 */
void timer_new_task(TASK_S *task,void (*runFunction)(void));

/**
 * Start some task
 * @ingroup taskS
 * @param *task Struct whith task data
 * @param period_us Period Value to repeat the task
 * @todo verify how use two tasks
 */
void timer_start_task(TASK_S *task,void (*alertFunction)(int),int period_us);

/**
 * Stop some task
 * @ingroup taskS
 * @todo verify use of variable 'erno'
 */
void timer_stop_task(TASK_S *task);
#endif
