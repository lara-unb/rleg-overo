/** 
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include "taskScheduler.h"

void timer_new_task(TASK_S *task,void (*runFunction)(void)){
  (*task).t_global = 0.0;
  (*task).T_exec_global = 0.0;
  (*task).T_mean_global = 0.0;
  (*task).T_max_global = 0.0;
  (*task).period_us = 0.0;
  (*task).isFirstExecution = 1;

  (*task).run = runFunction;
}

void timer_start_task(TASK_S *task,void (*alertFunction)(int),int period_us){
  struct itimerspec itimer;
  struct sigevent evp;
  int erno = 0;
  
  task->isFirstExecution = 1;
  task->period_us = period_us;

  /*evp.sigev_value.sival_ptr = &(task->timer);
  evp.sigev_notify = SIGEV_SIGNAL;
  evp.sigev_signo = SIGUSR1;*/

  //memset (&evp, 0, sizeof (struct sigevent));
  evp.sigev_value.sival_int = 0;
  evp.sigev_notify = SIGEV_THREAD;
  evp.sigev_notify_attributes = NULL;
  evp.sigev_notify_function = alertFunction;

  if(timer_create( CLOCK_REALTIME, &evp, &(task->timer))<0)
  {
    fprintf(stderr, "[%d]: %s\n", __LINE__, strerror(erno));
    exit(erno);
  }

  /*/ Signal handler configuration
  struct sigaction satimer;
  satimer.sa_handler = task->run;
  sigemptyset( &satimer.sa_mask );
  satimer.sa_flags = SA_RESTART;*/

  //if ( sigaction( SIGUSR1, &satimer, NULL ) < 0)
  if(timer_create(CLOCK_REALTIME, &evp, task->timer) < 0)
  {
    //printf( "ERROR: sigaction.\n" );
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
  task->run();

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
