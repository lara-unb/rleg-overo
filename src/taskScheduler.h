/**
 * Implentation of Scheduler for tasks:
 * @autor Rafael
 * @todo test it
 */

#include <unistd.h>
#include <signal.h>
#include <ptread.h>
#include <sys/time.h>
#include <time.h>


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
  
  void (*runFunction)(void); ///< hook to the action
} TASK_S;

/**
 * Create a task
 */
void timer_new_task(TASK_S *task,timer_t timer ,void (*runFuntion)(void));

/**
 * Start some task
 */
void timer_start_task(TASK_S *task);

/**
 * Stop some task
 */
void timer_stop_task(TASK_S *task);
