// This file is automatically generated.
// Do not edit it!
// Edit structure definition instead and than rebuild project.
#define ONBOARD_PARAM_CNT 284

/*
 * volatile array of parameters in RAM
 */
static param_union_t gp_val[ONBOARD_PARAM_CNT] __CCM__;

/**
 *
 */
const GlobalParam_t ParamRegistry::param_db[] = {
{"SYS_id", {.u32 = 1}, {.u32 = 20}, {.u32 = 255}, &gp_val[0], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "System ID.\n This value MUST BE FIRST in param structure. Value 0 reserved for ground station."},
{"SYS_mavtype", {.u32 = 0}, {.u32 = 10}, {.u32 = 16}, &gp_val[1], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Autopilot type (0 - generic, 1 - fixed wing, 10 - ground rover).\nOther types you can found in enum MAV_TYPE \nNOTE! You MUST REBOOT device after changing it."},
{"SYS_vehicle_type", {.u32 = 0}, {.u32 = 0}, {.u32 = 1}, &gp_val[2], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Vehicle type. 0 - Maverick, 1 - hand crafted rover \nNOTE! You MUST REBOOT device after changing it."},
{"SH_over_radio", {.u32 = 0}, {.u32 = 0}, {.u32 = 1}, &gp_val[3], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "When 1 than drop shell on xbee channel and telemetry on USB_CDC and vice versa."},
{"AHRS_accweight", {.f32 = 0.000000}, {.f32 = 0.005000}, {.f32 = 0.500000}, &gp_val[4], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"AHRS_magweight", {.f32 = 0.000000}, {.f32 = 0.050000}, {.f32 = 0.900000}, &gp_val[5], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"AHRS_gpsweight", {.f32 = 0.000000}, {.f32 = 0.050000}, {.f32 = 0.500000}, &gp_val[6], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"AHRS_beta", {.f32 = 0.000000}, {.f32 = 1.000000}, {.f32 = 20.000000}, &gp_val[7], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Error rate of gyro in degrees"},
{"AHRS_zeta", {.f32 = 0.000000}, {.f32 = 1.000000}, {.f32 = 20.000000}, &gp_val[8], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"AHRS_mode", {.u32 = 0}, {.u32 = 0}, {.u32 = 3}, &gp_val[9], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "0 - Starlino, 1 - Madgwick, 2 - Adis, 3 - Kalman"},
{"MARG_acc_src", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[10], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Accelerometer measurement source for MARG (see enum acc_src_t)"},
{"MARG_gyr_src", {.u32 = 0}, {.u32 = 0}, {.u32 = 1}, &gp_val[11], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Angular rate measurement source for MARG (see enum gyr_src_t)"},
{"MARG_mag_src", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[12], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Magnetometer measurement source for MARG (see enum mag_src_t)"},
{"LSMM_gain", {.u32 = 0}, {.u32 = 0}, {.u32 = 7}, &gp_val[13], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "LSM magnetometer gain (see enum mag_sens_t)"},
{"LSMM_xoffset", {.f32 = -500.000000}, {.f32 = 0.000000}, {.f32 = 500.000000}, &gp_val[14], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"LSMM_yoffset", {.f32 = -500.000000}, {.f32 = 0.000000}, {.f32 = 500.000000}, &gp_val[15], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"LSMM_zoffset", {.f32 = -500.000000}, {.f32 = 0.000000}, {.f32 = 500.000000}, &gp_val[16], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"LSMM_vectorlen", {.f32 = 0.000000}, {.f32 = 10.000000}, {.f32 = 512.000000}, &gp_val[17], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Length of magnetic flux vector in uT acquired during sphere offset calculation"},
{"LSMM_xsens", {.f32 = 0.900000}, {.f32 = 1.000000}, {.f32 = 1.100000}, &gp_val[18], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"LSMM_ysens", {.f32 = 0.900000}, {.f32 = 1.000000}, {.f32 = 1.100000}, &gp_val[19], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"LSMM_zsens", {.f32 = 0.900000}, {.f32 = 1.000000}, {.f32 = 1.100000}, &gp_val[20], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"LSMM_xpol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[21], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"LSMM_ypol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[22], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"LSMM_zpol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[23], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"LSMM_sortmtrx", {.u32 = 0}, {.u32 = 273}, {.u32 = 1}, &gp_val[24], PARAM_SORT_MTRX, MAVLINK_TYPE_UINT32_T, "Sorting matrix for acquired gyro values\nto correspond with real device axis"},
{"LSMM_calmode", {.u32 = 0}, {.u32 = 0}, {.u32 = 1}, &gp_val[25], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "0 - simple spherical shift, 1 - egg compensate"},
{"LSMM_ellip1", {.f32 = -5.000000}, {.f32 = 1.000000}, {.f32 = 5.000000}, &gp_val[26], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "ellipsoid correction coefficient"},
{"LSMM_ellip2", {.f32 = -5.000000}, {.f32 = 1.000000}, {.f32 = 5.000000}, &gp_val[27], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "ellipsoid correction coefficient"},
{"LSMM_ellip3", {.f32 = -5.000000}, {.f32 = 1.000000}, {.f32 = 5.000000}, &gp_val[28], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "ellipsoid correction coefficient"},
{"LSMM_ellip4", {.f32 = -5.000000}, {.f32 = 0.000000}, {.f32 = 5.000000}, &gp_val[29], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "ellipsoid correction coefficient"},
{"LSMM_ellip5", {.f32 = -5.000000}, {.f32 = 0.000000}, {.f32 = 5.000000}, &gp_val[30], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "ellipsoid correction coefficient"},
{"LSMM_ellip6", {.f32 = -5.000000}, {.f32 = 0.000000}, {.f32 = 5.000000}, &gp_val[31], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "ellipsoid correction coefficient"},
{"MAG_declinate", {.f32 = -90.000000}, {.f32 = 7.000000}, {.f32 = 90.000000}, &gp_val[32], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Magnetic declination. \nThe declination is positive when the magnetic north is east of true north. \nhttp://www.ngdc.noaa.gov/geomagmodels/Declination.jsp"},
{"MAG_still_thr", {.f32 = 0.000000}, {.f32 = 1.000000}, {.f32 = 20.000000}, &gp_val[33], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Device immobility threshold in parrots"},
{"MAG_still_flen", {.i32 = 1}, {.i32 = 256}, {.i32 = 2048}, &gp_val[34], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Length of filter used in immobility detector"},
{"MAG_zeroflen", {.i32 = 1}, {.i32 = 256}, {.i32 = 2048}, &gp_val[35], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Length of filter used in immobility detector"},
{"MAG_zerocnt", {.u32 = 256}, {.u32 = 512}, {.u32 = 4096}, &gp_val[36], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"ADIS_smplrtdiv", {.u32 = 12}, {.u32 = 24}, {.u32 = 246}, &gp_val[37], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Divider for ADIS's 2460Hz sample rate"},
{"MPU_dlpf", {.u32 = 0}, {.u32 = 5}, {.u32 = 6}, &gp_val[38], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "If dlpf>0 than use internal LPF and internal sample rate divider.Otherwise use 1kHz sample rate with external FIR and external decimator"},
{"MPU_smplrtdiv", {.u32 = 1}, {.u32 = 10}, {.u32 = 50}, &gp_val[39], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Divider for MPU's 1kHz sample rate"},
{"MPU_fir_f", {.i32 = -1}, {.i32 = 0}, {.i32 = 6}, &gp_val[40], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Cut off frequency of the external FIR filter (F = 2^N). Set -1 to disable filter at all."},
{"MPU_gyr_fs", {.u32 = 0}, {.u32 = 1}, {.u32 = 3}, &gp_val[41], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "MPU gyroscope full scale (0 - 250, 1 - 500, 2 - 1000, 3 - 2000) deg/s"},
{"MPU_acc_fs", {.u32 = 0}, {.u32 = 3}, {.u32 = 3}, &gp_val[42], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "MPU accelerometer full scale (0 - 2, 1 - 4, 2 - 8, 3 - 16) g"},
{"MPUG_xt_c0", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[43], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_xt_c1", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[44], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_xt_c2", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[45], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_yt_c0", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[46], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_yt_c1", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[47], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_yt_c2", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[48], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_zt_c0", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[49], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_zt_c1", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[50], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_zt_c2", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[51], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Coefficient for thermal zero compensation polynomial"},
{"MPUG_xsens", {.f32 = 0.900000}, {.f32 = 1.000000}, {.f32 = 1.100000}, &gp_val[52], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Sensitivity correction"},
{"MPUG_ysens", {.f32 = 0.900000}, {.f32 = 1.000000}, {.f32 = 1.100000}, &gp_val[53], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Sensitivity correction"},
{"MPUG_zsens", {.f32 = 0.900000}, {.f32 = 1.000000}, {.f32 = 1.100000}, &gp_val[54], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Sensitivity correction"},
{"MPUG_xpol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[55], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"MPUG_ypol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[56], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"MPUG_zpol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[57], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"MPUG_sortmtrx", {.u32 = 0}, {.u32 = 273}, {.u32 = 1}, &gp_val[58], PARAM_SORT_MTRX, MAVLINK_TYPE_UINT32_T, "Sorting matrix for acquired gyro values\nto correspond with real device axis"},
{"MPUG_zerocnt", {.i32 = 512}, {.i32 = 2048}, {.i32 = 16384}, &gp_val[59], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Sample count for zeroing procedure"},
{"MPUG_zeroflen", {.i32 = 2}, {.i32 = 512}, {.i32 = 2048}, &gp_val[60], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Filter length used in zero calibration routine"},
{"MPUG_stillthr", {.f32 = 0.000000}, {.f32 = 0.100000}, {.f32 = 1.000000}, &gp_val[61], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Stillness threshold Rad/S"},
{"ACC_xoffset", {.i32 = -100}, {.i32 = 0}, {.i32 = 100}, &gp_val[62], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"ACC_yoffset", {.i32 = -100}, {.i32 = 0}, {.i32 = 100}, &gp_val[63], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"ACC_zoffset", {.i32 = -100}, {.i32 = 0}, {.i32 = 100}, &gp_val[64], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"ACC_xsens", {.i32 = 1000}, {.i32 = 8192}, {.i32 = 27000}, &gp_val[65], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "sens LSB/g, nominals: 4096, 8192, 16384"},
{"ACC_ysens", {.i32 = 1000}, {.i32 = 8192}, {.i32 = 27000}, &gp_val[66], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "sens LSB/g, nominals: 4096, 8192, 16384"},
{"ACC_zsens", {.i32 = 1000}, {.i32 = 8192}, {.i32 = 27000}, &gp_val[67], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "sens LSB/g, nominals: 4096, 8192, 16384"},
{"ACC_xpol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[68], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"ACC_ypol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[69], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"ACC_zpol", {.i32 = -1}, {.i32 = 1}, {.i32 = 1}, &gp_val[70], PARAM_POLARITY, MAVLINK_TYPE_INT32_T,  NULL},
{"ACC_sortmtrx", {.u32 = 0}, {.u32 = 273}, {.u32 = 1}, &gp_val[71], PARAM_SORT_MTRX, MAVLINK_TYPE_UINT32_T, "Sorting matrix for acquired gyro values\nto correspond with real device axis"},
{"ACC_still_thr", {.f32 = 0.000000}, {.f32 = 0.006000}, {.f32 = 0.100000}, &gp_val[72], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Device immobility threshold in g"},
{"ACC_still_flen", {.i32 = 1}, {.i32 = 256}, {.i32 = 2048}, &gp_val[73], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Length of filter used in immobility detector"},
{"PMU_above_msl", {.i32 = -200}, {.i32 = 255}, {.i32 = 4000}, &gp_val[74], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Height of barometric sensor above sea level in meters"},
{"PMU_reserved1", {.i32 = -2000000}, {.i32 = 0}, {.i32 = 2000000}, &gp_val[75], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"ADC_car_I_k", {.i32 = -1000000}, {.i32 = 0}, {.i32 = 1000000}, &gp_val[76], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "k coefficient for calculation from ADC values to uA using formulae y=kx+b\nfor ground rover"},
{"ADC_car_I_b", {.i32 = -1000000}, {.i32 = 0}, {.i32 = 1000000}, &gp_val[77], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "b coefficient for calculation from ADC values to uA using formulae y=kx+b\nfor ground rover"},
{"ADC_SV_gain", {.u32 = 0}, {.u32 = 0}, {.u32 = 122400}, &gp_val[78], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"ADC_MV_gain", {.u32 = 0}, {.u32 = 0}, {.u32 = 122400}, &gp_val[79], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"ADC_plane_I_k", {.i32 = -1000000}, {.i32 = 0}, {.i32 = 1000000}, &gp_val[80], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "k coefficient for calculation from ADC values to uA using formulae y=kx+b\nfor fixed wing"},
{"ADC_plane_I_b", {.i32 = -1000000}, {.i32 = 0}, {.i32 = 1000000}, &gp_val[81], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "b coefficient for calculation from ADC values to uA using formulae y=kx+b\nfor fixed wing"},
{"ADC_mpxv_shift", {.u32 = 0}, {.u32 = 128}, {.u32 = 255}, &gp_val[82], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Offset cancelation. Uses AD5200."},
{"BAT_cap", {.u32 = 0}, {.u32 = 3000}, {.u32 = 11000}, &gp_val[83], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Battery capacitance (mAh)"},
{"BAT_fill", {.u32 = 0}, {.u32 = 0}, {.u32 = 100}, &gp_val[84], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Start battery filling in percents"},
{"SINS_restart", {.u32 = 0}, {.u32 = 1}, {.u32 = 65535}, &gp_val[85], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Change this value to enforce sins restart"},
{"SINS_en_gnss", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[86], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_odo", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[87], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_baro", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[88], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_zihr", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[89], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_nonhol", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[90], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_euler", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[91], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_mag", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[92], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_gnss_v", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[93], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_en_zupt", {.u32 = 0}, {.u32 = 1}, {.u32 = 1}, &gp_val[94], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SINS_R_ne_sns", {.f32 = 0.001000}, {.f32 = 5.000000}, {.f32 = 20.000000}, &gp_val[95], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_d_sns", {.f32 = 0.001000}, {.f32 = 10.000000}, {.f32 = 20.000000}, &gp_val[96], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_v_n_sns", {.f32 = 0.001000}, {.f32 = 0.200000}, {.f32 = 20.000000}, &gp_val[97], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_odo", {.f32 = 0.001000}, {.f32 = 0.100000}, {.f32 = 20.000000}, &gp_val[98], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_nonhol", {.f32 = 0.001000}, {.f32 = 0.100000}, {.f32 = 20.000000}, &gp_val[99], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_baro", {.f32 = 0.001000}, {.f32 = 0.300000}, {.f32 = 20.000000}, &gp_val[100], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_mag", {.f32 = 0.001000}, {.f32 = 0.300000}, {.f32 = 20.000000}, &gp_val[101], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_euler", {.f32 = 0.001000}, {.f32 = 0.010000}, {.f32 = 20.000000}, &gp_val[102], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_R_zihr", {.f32 = 0.001000}, {.f32 = 0.010000}, {.f32 = 20.000000}, &gp_val[103], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_Qm_acc", {.f32 = 0.000000}, {.f32 = 0.001000}, {.f32 = 1.000000}, &gp_val[104], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_Qm_gyr", {.f32 = 0.000000}, {.f32 = 0.001000}, {.f32 = 1.000000}, &gp_val[105], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_Qm_acc_x", {.f32 = 0.000000}, {.f32 = 0.000001}, {.f32 = 1.000000}, &gp_val[106], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_Qm_acc_y", {.f32 = 0.000000}, {.f32 = 0.000001}, {.f32 = 1.000000}, &gp_val[107], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_Qm_acc_z", {.f32 = 0.000000}, {.f32 = 0.000001}, {.f32 = 1.000000}, &gp_val[108], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_Qm_gyr_bias", {.f32 = 0.000000}, {.f32 = 0.001000}, {.f32 = 1.000000}, &gp_val[109], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SINS_eu_vh_roll", {.f32 = -180.000000}, {.f32 = 0.000000}, {.f32 = 180.000000}, &gp_val[110], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "deg"},
{"SINS_eu_vh_pitch", {.f32 = -90.000000}, {.f32 = 0.000000}, {.f32 = 90.000000}, &gp_val[111], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "deg"},
{"SINS_eu_vh_yaw", {.f32 = -360.000000}, {.f32 = 0.000000}, {.f32 = 360.000000}, &gp_val[112], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "deg"},
{"SINS_acc_b_x", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[113], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "m/s^2"},
{"SINS_acc_b_y", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[114], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "m/s^2"},
{"SINS_acc_b_z", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[115], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "m/s^2"},
{"SINS_gyr_b_x", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[116], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "rad"},
{"SINS_gyr_b_y", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[117], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "rad"},
{"SINS_gyr_b_z", {.f32 = -1000.000000}, {.f32 = 0.000000}, {.f32 = 1000.000000}, &gp_val[118], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "rad"},
{"GLRT_acc_sigma", {.f32 = 0.000100}, {.f32 = 0.010000}, {.f32 = 1.000000}, &gp_val[119], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"GLRT_gyr_sigma", {.f32 = 0.000010}, {.f32 = 0.010000}, {.f32 = 0.500000}, &gp_val[120], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"GLRT_gamma", {.f32 = 0.000000}, {.f32 = 1.000000}, {.f32 = 100.000000}, &gp_val[121], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"GLRT_samples", {.u32 = 1}, {.u32 = 5}, {.u32 = 30}, &gp_val[122], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_ail_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[123], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_ail_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[124], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_ail_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[125], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_ele_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[126], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_ele_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[127], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_ele_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[128], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_rud_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[129], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_rud_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[130], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_rud_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[131], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_thr_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[132], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_thr_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[133], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_thr_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[134], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_4_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[135], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_4_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[136], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_4_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[137], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_5_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[138], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_5_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[139], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_5_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[140], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_6_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[141], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_6_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[142], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_6_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[143], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_7_min", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[144], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_7_max", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[145], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_7_mid", {.u32 = 1000}, {.u32 = 1500}, {.u32 = 2000}, &gp_val[146], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_rud_dz", {.u32 = 1}, {.u32 = 16}, {.u32 = 1500}, &gp_val[147], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"SRV_thr_dz", {.u32 = 1}, {.u32 = 16}, {.u32 = 1500}, &gp_val[148], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"RC_timeout", {.u32 = 500}, {.u32 = 2000}, {.u32 = 10000}, &gp_val[149], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"RC_override", {.u32 = 0}, {.u32 = 0}, {.u32 = 3}, &gp_val[150], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"RC_map_man", {.i32 = -1}, {.i32 = 0}, {.i32 = 3}, &gp_val[151], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"PID_00_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[152], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_00_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[153], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_00_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[154], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_00_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[155], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_00_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[156], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_00_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[157], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_01_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[158], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_01_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[159], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_01_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[160], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_01_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[161], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_01_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[162], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_01_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[163], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_02_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[164], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_02_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[165], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_02_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[166], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_02_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[167], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_02_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[168], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_02_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[169], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_03_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[170], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_03_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[171], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_03_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[172], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_03_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[173], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_03_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[174], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_03_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[175], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_04_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[176], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_04_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[177], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_04_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[178], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_04_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[179], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_04_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[180], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_04_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[181], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_05_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[182], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_05_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[183], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_05_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[184], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_05_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[185], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_05_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[186], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_05_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[187], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_06_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[188], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_06_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[189], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_06_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[190], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_06_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[191], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_06_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[192], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_06_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[193], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_07_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[194], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_07_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[195], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_07_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[196], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_07_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[197], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_07_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[198], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_07_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[199], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_08_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[200], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_08_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[201], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_08_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[202], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_08_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[203], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_08_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[204], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_08_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[205], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_09_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[206], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_09_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[207], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_09_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[208], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_09_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[209], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_09_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[210], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_09_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[211], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_10_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[212], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_10_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[213], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_10_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[214], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_10_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[215], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_10_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[216], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_10_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[217], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_11_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[218], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_11_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[219], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_11_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[220], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_11_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[221], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_11_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[222], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_11_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[223], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_12_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[224], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_12_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[225], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_12_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[226], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_12_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[227], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_12_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[228], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_12_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[229], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_13_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[230], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_13_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[231], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_13_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[232], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_13_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[233], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_13_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[234], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_13_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[235], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_14_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[236], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_14_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[237], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_14_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[238], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_14_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[239], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_14_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[240], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_14_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[241], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_15_P", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[242], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_15_I", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[243], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_15_D", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[244], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_15_Min", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[245], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_15_Max", {.f32 = -9000.000000}, {.f32 = 0.000000}, {.f32 = 9000.000000}, &gp_val[246], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_15_proc", {.u32 = 0}, {.u32 = 0}, {.u32 = 2}, &gp_val[247], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"PID_vm_scale_00", {.f32 = -10000.000000}, {.f32 = 1.000000}, {.f32 = 10000.000000}, &gp_val[248], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_vm_scale_01", {.f32 = -10000.000000}, {.f32 = 1.000000}, {.f32 = 10000.000000}, &gp_val[249], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_vm_scale_02", {.f32 = -10000.000000}, {.f32 = 1.000000}, {.f32 = 10000.000000}, &gp_val[250], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"PID_vm_scale_03", {.f32 = -10000.000000}, {.f32 = 1.000000}, {.f32 = 10000.000000}, &gp_val[251], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"T_attitude", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[252], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_debug", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[253], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_debug_vect", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[254], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_global_pos", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[255], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_gps_raw_int", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[256], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_highres_imu", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[257], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_nav_output", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[258], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_mission_curr", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[259], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_position_ned", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[260], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_raw_imu", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[261], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, "Interval of sending this data in milliseconds.\nSet it to 0 for disabling"},
{"T_raw_press", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[262], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_rc", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[263], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_rc_scaled", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[264], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_scal_imu", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[265], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_scal_press", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[266], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_sys_status", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[267], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"T_vfr_hud", {.u32 = 10}, {.u32 = 0}, {.u32 = 5000}, &gp_val[268], PARAM_SEND_TMO, MAVLINK_TYPE_UINT32_T, NULL},
{"TIME_zone", {.i32 = -24}, {.i32 = 0}, {.i32 = 24}, &gp_val[269], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  "Simple offset in hours."},
{"FLEN_adc", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[270], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_pres_dyn", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[271], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_pres_stat", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[272], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_climb", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[273], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_gnd_speed", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[274], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_reserved2", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[275], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_reserved3", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[276], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"FLEN_reserved4", {.i32 = 1}, {.i32 = 16}, {.i32 = 32768}, &gp_val[277], PARAM_DEFAULT, MAVLINK_TYPE_INT32_T,  NULL},
{"SPD_pulse2m", {.f32 = 0.000000}, {.f32 = 0.055556}, {.f32 = 1.000000}, &gp_val[278], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    "Multiply odometer pulses count by this coefficient to get\ntrip in meters. Coarse value is 0.05555555"},
{"SPD_trgt_speed", {.f32 = 0.000000}, {.f32 = 0.000000}, {.f32 = 60.000000}, &gp_val[279], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"SPD_speed_max", {.f32 = 0.000000}, {.f32 = 0.000000}, {.f32 = 60.000000}, &gp_val[280], PARAM_DEFAULT, MAVLINK_TYPE_FLOAT,    NULL},
{"GNSS_dyn_model", {.u32 = 0}, {.u32 = 8}, {.u32 = 8}, &gp_val[281], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"GNSS_fix_period", {.u32 = 100}, {.u32 = 200}, {.u32 = 1000}, &gp_val[282], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, NULL},
{"param_end_mark__", {.u32 = 0}, {.u32 = 0}, {.u32 = 1224}, &gp_val[283], PARAM_DEFAULT, MAVLINK_TYPE_UINT32_T, "Fake parameter with maximum allowable name length"},
};
