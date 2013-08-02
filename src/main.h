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

int ui_task();
int control_task();

#endif
