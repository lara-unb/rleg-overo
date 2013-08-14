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
#define TASK1_PERIOD  100 //us

/**@}*/

int main(void);

static void ui_task(int signo);
static void control_task(int signo);

void exit_program(void);

#endif //MAIN_H
