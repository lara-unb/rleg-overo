#ifndef UI_H
#define UI_H

//#define UI_MAVLINK_MODE        1
#define UI_NCURSES_MODE        2

#define UI_MODULE_ENABLED      1
#define UI_DEBUG_MODE          0

#define SUCCESS	1
#define FAILURE	-1

int ui_init(void);
int ui_close(void);
int ui_update(IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data, int total, int failure);
int ui_imu_data(IMU_DATA_STRUCT *pimu_data);
int ui_eff_data(EFF_DATA_STRUCT *peff_data);
int ui_mra_data(MRA_DATA_STRUCT *pmra_data);
int ui_overview_data(int total, int failures, IMU_DATA_STRUCT *pimu_data, EFF_DATA_STRUCT *peff_data, MRA_DATA_STRUCT *pmra_data);


//int ui_local_data(CALIBRATION_LOCAL_COORDINATE_SYSTEM_STRUCT *plocal_coordinate_system_data, CALIBRATION_LOCAL_FIELDS_STRUCT *plocal_fields_data, CALIBRATION_ALTIMETER_STRUCT *paltimeter_data);
//int ui_estimation(ESTIMATION_DATA_STRUCT *pestimator_data);
//int ui_control(CONTROL_DATA_STRUCT *pcontrol_data);

#endif //UI_H
