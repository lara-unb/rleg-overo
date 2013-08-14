/**
 *
 */

#include <string.h>
#include "taskScheduler.h"

void timer_new_task(TASK_S *task,void (*runFunction)(int)){
  (*task).t_global = 0.0;
  (*task).T_exec_global = 0.0;
  (*task).T_mean_global = 0.0;
  (*task).T_max_global = 0.0;
  (*task).period_us = 0.0;
  (*task).isFirstExecution = 1;

  (*task).run = runFunction;
}

void timer_start_task(TASK_S *task){
  struct itimerspec itimer;
  timer_t timer;
  struct sigevent sigev;
  int erno = 0;

  (*task).isFirstExecution = 1;
  //
  struct sigevent evp;

  evp.sigev_value.sival_ptr = &timer;
  evp.sigev_notify = SIGEV_SIGNAL;
  evp.sigev_signo = SIGUSR1;

  if ( timer_create ( CLOCK_REALTIME, &evp, &timer) )
  {
    fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
    exit(erno);
  }


  // Signal handler configuration
  struct sigaction satimer;
  satimer.sa_handler = task->run;
  sigemptyset( &satimer.sa_mask );
  satimer.sa_flags = SA_RESTART;
  if ( sigaction ( SIGUSR1, &satimer, NULL ) < 0)
  {
    printw( "ERROR: sigaction.\n" );
    fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
    exit(erno);
  }

  itimer.it_interval.tv_sec = 0;
  itimer.it_interval.tv_nsec= task->period_us*1000;
  itimer.it_value = itimer.it_interval;

  if(timer_settime(task->timer,0,&itimer,NULL) < 0)
  {
    fprintf(stderr,"[%d]: %s\n", __LINE__, strerror(erno));
    exit(erno);
  }
}

void timer_function_task(TASK_S *task)
{
  int status = 0;
  static struct timeval timereset;
  static struct timeval time;
  static struct timeval time_exec_start;
  static struct timeval time_exec_end;
  static double T_task,t_task,t_task_previous;
  static double T_task_mean,T_task_min,T_task_max,T_task_exec;

  gettimeofday(&time_exec_start, NULL);

  // Time calculation
  if((*task).isFirstExecution)
    {
      gettimeofday(&timereset, NULL);
      /// @todo make a function to set all times to zero
    }

  gettimeofday(&time, NULL);
  t_task = ((time.tv_sec - timereset.tv_sec) + (time.tv_usec - timereset.tv_usec)*1e-6);
  T_task = t_task - t_task_previous;
  t_task_previous = t_task;

  if((*task).isFirstExecution)
    {
      T_task_mean = T_task;
      T_task_min  = 1e200;
      T_task_max  = 0.0;
      //t0 = t_task;
    }
  else
    {
      T_task_mean = 0.05*T_task + 0.95*T_task_mean;
      if(T_task < T_task_min) T_task_min  = T_task;
      if(T_task > T_task_max) T_task_max  = T_task;
    }

  // Run the thread
  //status = task->run();

  gettimeofday(&time_exec_end, NULL);
  T_task_exec = ((time_exec_end.tv_sec - time_exec_start.tv_sec) + (time_exec_end.tv_usec - time_exec_start.tv_usec)*1e-6);

  (*task).t_global = t_task;
  (*task).T_mean_global = T_task_mean;
  (*task).T_min_global = T_task_min;
  (*task).T_max_global = T_task_max;
  (*task).T_exec_global = T_task_exec;

  (*task).isFirstExecution = 0;
}

void timer_stop_task(TASK_S *task){
  int erno = 0;
  if(timer_delete((*task).timer)<0){
    fprintf(stderr,"[%d]:%s\n",__LINE__,strerror(erno));
    exit(erno);
  }
}

/*
#include <time.h>
#include <signal.h>
#include <sys/time.h>

> CONFIGURAÇÃO

>> Primeiramente, usando timer_create

// Periodic POSIX timer configuration
timer_ttimer;
struct sigeventevp;
evp.sigev_value.sival_ptr = &timer;
evp.sigev_notify = SIGEV_SIGNAL;
evp.sigev_signo = SIGUSR1;
if ( timer_create ( CLOCK_REALTIME, &evp, &timer) )
{
printw( "> ERROR: timer_create\n" );
}
>> Em seguida, o que fazer quando SIGUSR1 chamar? Com sigaction definimos que executamos a função (sa_handler) e reiniciamos o timer (SA_RESTART).

// Signal handler configuration
struct sigactionsatimer;
satimer.sa_handler = npi_timer_handler;
sigemptyset( &satimer.sa_mask );
satimer.sa_flags = SA_RESTART;
if ( sigaction ( SIGUSR1, &satimer, NULL ) < 0)
{
printw( "ERROR: sigaction.\n" );
}

> INICIALIZAÇÃO, em que TS_IN_NANO é o período estabelecido

// Timer initialization
struct itimerspec   timerPeriod;
timerPeriod.it_value.tv_sec = 0;
timerPeriod.it_value.tv_nsec = TS_IN_NANO;
timerPeriod.it_interval.tv_sec = 0;
timerPeriod.it_interval.tv_nsec = TS_IN_NANO;
if ( timer_settime ( timer, 0, &timerPeriod, NULL ) != 0)
{
printw( "> ERROR: timer_settime.\n" );
}

> FUNÇÃO PERIÓDICA

static void npi_timer_handler ( int signo )
{
// everything you want here
}

> FECHANDO O TIMER
timer_delete ( timer );
*/
