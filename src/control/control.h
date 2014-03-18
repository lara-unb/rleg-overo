
#ifndef CONTROL_H
#define CONTROL_H

/* low ajust [degrees] */
#define ANGLE_MAX         90
#define ANGLE_HIGH        ANGLE_MAX - 10 
#define ANGLE_LOW         10

/*  */
#define V_CTRL_MAX        4095

/*
0<=V_ctrl<4096 represents mV
Current in MRA = V_ctrl/4
*/

#endif //CONTROL_H