#ifndef UI_H
#define UI_H

//#define UI_MAVLINK_MODE        1
#define UI_NCURSES_MODE        2

#define UI_MODULE_ENABLED      1
#define UI_DEBUG_MODE          0

#define SUCCESS	1
#define FAILURE	-1

/**
 * @groupdef ui Functions to deal the User Inteface using ncurses
 */

/**
 * Initialize UI
 * @ingroup ui
 */
int ui_init(void);

/**
 * Close UI
 * @ingroup ui
 */
int ui_close(void);

/**
 * Update Screen with new data of sensors
 *@ingroup ui
 */
int ui_update(IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data, int total, int failure);

/**
 * Print IMU data
 * @ingroup ui
 */
int ui_imu_data(IMU_DATA_STRUCT *pimu_data);

int ui_eff_data(EFF_DATA_STRUCT *peff_data);

/**
 * Print MRA data
 * @ingroup ui
 */
int ui_mra_data(MRA_DATA_STRUCT *pmra_data);

/**
 * Print ALL sensors data
 */
int ui_overview_data(int total, int failures, IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data);


//int ui_local_data(CALIBRATION_LOCAL_COORDINATE_SYSTEM_STRUCT *plocal_coordinate_system_data, CALIBRATION_LOCAL_FIELDS_STRUCT *plocal_fields_data, CALIBRATION_ALTIMETER_STRUCT *paltimeter_data);
//int ui_estimation(ESTIMATION_DATA_STRUCT *pestimator_data);
//int ui_control(CONTROL_DATA_STRUCT *pcontrol_data);

#endif //UI_H
