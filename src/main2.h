#ifndef MAIN2_H
#define MAIN2_H

// Returns
#define SUCCESS   1
#define FAILURE   -1

int main(void);
int periodic_task_1(void);
int periodic_task_2(void);
void timer_start_task_1(void);
void timer_start_task_2(void);
void timer_stop_task_1(void);
void timer_stop_task_2(void);
void timer_function_task_1(void);
void timer_function_task_2(void);
int reset_timer(void);
int get_time(double *time_control_task_s, double *Ts_control_task_s, double *mean_time_control_task_s, double *t0_control_task_s);
void exit_program(void);

#endif //MAIN2_H
