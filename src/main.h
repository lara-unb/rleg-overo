/**
 * Main includes
 * @author Rafael Lima
 */

#ifndef MAIN_H
#define MAIN_H

#define FAILURE -1
#define SUCCESS 1

/**
 * @defgroup mainConfig Configuration of the main
 * @{
 */
#define TASK1_PERIOD  100000 //us

/**@}*/

int main(void);

/**
 * Main Periodic Task
 * @param signo 
 */
static void main_task(int signo);

static void ui_task();
static void control_task();

void exit_program(void);

#endif //MAIN_H
