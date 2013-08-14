/*
 * File to test
 */

#include <stdio.h>
#include "taskScheduler.h"

static void ping(int signo);
static void pong(int signo);

int main(){
  TASK_S task1,task2;
  char a='a';

  timer_new_task(&task1,ping);
  timer_new_task(&task2,pong);

  task1.period_us = 1000;
  task2.period_us = 500;

  timer_start_task(&task1);
  timer_start_task(&task2);

  do{
    scanf("%c",&a);
  }while(a!='q');

  timer_stop_task(&task1);
  timer_stop_task(&task2);

  return 0;
}

static void ping(int signo){
  printf("ping.");
}

static void pong(int signo){
  printf("pong.");
}
