/*
 * File to test
 */

#include <stdio.h>
#include <stdlib.h>
#include "taskScheduler.h"

static void ping(int signo);
static void pong(int signo);

int main(){
  TASK_S task1,task2;
  int i=0;

  timer_new_task(&task1,ping);
  timer_new_task(&task2,pong);

  timer_start_task(&task1,1000);
  timer_start_task(&task2,2000);

  while(1) i=1;
  //for(i=1;i<10;i++) printf(" .");
  
  timer_stop_task(&task1);
  timer_stop_task(&task2);

  exit(0);
}

static void ping(int signo){
  printf("ping.");
}

static void pong(int signo){
  printf("pong.");
}
