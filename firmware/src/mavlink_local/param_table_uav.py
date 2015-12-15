#!/usr/bin/env python
# -*- coding: utf-8 -*-


SERVO_MIN   = 1000
SERVO_MAX   = 2000

SEND_MIN    = 10
SEND_MAX    = 5000

PID_MAX     = 9000
PID_MIN     = -PID_MAX

ONBOARD_PARAM_NAME_LENGTH = 15

#    key            min         default     max         type checker        help
param = [
("SYS_id",          1,          20,         255,        "u", "default",     "System ID.\\n This value MUST BE FIRST in param structure. Value 0 reserved for ground station."),
("SYS_mavtype",     0,          10,         16,         "u", "default",     "Autopilot type (0 - generic, 1 - fixed wing, 10 - ground rover).\\nOther types you can found in enum MAV_TYPE \\nNOTE! You MUST REBOOT device after changing it."),
("SYS_vehicle_type",0,          0,          1,          "u", "default",     "Vehicle type. 0 - Maverick, 1 - hand crafted rover \\nNOTE! You MUST REBOOT device after changing it."),
("SH_over_radio",   0,          0,          1,          "u", "default",     "When 1 than drop shell on xbee channel and telemetry on USB_CDC and vice versa."),

#/* veights of different components */
("AHRS_accweight",  0.0,        0.005,      0.5,        "f", "default",     "NULL"),
("AHRS_magweight",  0.0,        0.05,       0.9,        "f", "default",     "NULL"),
("AHRS_gpsweight",  0.0,        0.05,       0.5,        "f", "default",     "NULL"),
("AHRS_beta",       0.0,        1.0,        20.0,       "f", "default",     "Error rate of gyro in degrees"),
("AHRS_zeta",       0.0,        1.0,        20.0,       "f", "default",     "NULL"),
("AHRS_mode",       0,          0,          3,          "u", "default",     "0 - Starlino, 1 - Madgwick, 2 - Adis, 3 - Kalman"),

########### MARG ##############
("MARG_acc_src",    0,          0,          2,          "u", "default",     "Accelerometer measurement source for MARG (see enum acc_src_t)"),
("MARG_gyr_src",    0,          0,          1,          "u", "default",     "Angular rate measurement source for MARG (see enum gyr_src_t)"),
("MARG_mag_src",    0,          0,          2,          "u", "default",     "Magnetometer measurement source for MARG (see enum mag_src_t)"),

########### LSM303 ###############
("LSMM_gain",       0,          0,          7,          "u", "default",     "LSM magnetometer gain (see enum mag_sens_t)"),
# offsets for spherical hard iron compensation
("LSMM_xoffset",    -500,       0,          500,        "f", "default",     "NULL"),
("LSMM_yoffset",    -500,       0,          500,        "f", "default",     "NULL"),
("LSMM_zoffset",    -500,       0,          500,        "f", "default",     "NULL"),
("LSMM_vectorlen",   0,         10,         512,        "f", "default",     "Length of magnetic flux vector in uT acquired during sphere offset calculation"),
# sensitivity correction
("LSMM_xsens",      0.9,        1,          1.1,        "f", "default",     "NULL"),
("LSMM_ysens",      0.9,        1,          1.1,        "f", "default",     "NULL"),
("LSMM_zsens",      0.9,        1,          1.1,        "f", "default",     "NULL"),
# axis polarities. Relative to device axis
("LSMM_xpol",       -1,         1,          1,          "i", "polarity",    "NULL"),
("LSMM_ypol",       -1,         1,          1,          "i", "polarity",    "NULL"),
("LSMM_zpol",       -1,         1,          1,          "i", "polarity",    "NULL"),
("LSMM_sortmtrx",   0,          0b100010001,1,          "u", "sort_mtrx",   "Sorting matrix for acquired gyro values\\nto correspond with real device axis"),
("LSMM_calmode",    0,          0,          1,          "u", "default",     "0 - simple spherical shift, 1 - egg compensate"),
# ellipsoid correction coefficients
("LSMM_ellip1",     -5.0,       1.0,        5.0,        "f", "default",     "ellipsoid correction coefficient"),
("LSMM_ellip2",     -5.0,       1.0,        5.0,        "f", "default",     "ellipsoid correction coefficient"),
("LSMM_ellip3",     -5.0,       1.0,        5.0,        "f", "default",     "ellipsoid correction coefficient"),
("LSMM_ellip4",     -5.0,       0.0,        5.0,        "f", "default",     "ellipsoid correction coefficient"),
("LSMM_ellip5",     -5.0,       0.0,        5.0,        "f", "default",     "ellipsoid correction coefficient"),
("LSMM_ellip6",     -5.0,       0.0,        5.0,        "f", "default",     "ellipsoid correction coefficient"),
#
("MAG_declinate",   -90,        7,          90,         "f", "default",     "Magnetic declination. \\nThe declination is positive when the magnetic north is east of true north. \\nhttp://www.ngdc.noaa.gov/geomagmodels/Declination.jsp"),
("MAG_still_thr",   0,          1,          20,         "f", "default",     "Device immobility threshold in parrots"),
("MAG_still_flen",  1,          256,        2048,       "i", "default",     "Length of filter used in immobility detector"),
("MAG_zeroflen",    1,          256,        2048,       "i", "default",     "Length of filter used in immobility detector"),
("MAG_zerocnt",     256,        512,        4096,       "u", "default",     "NULL"),
# rotation matrix to align magnetometers and accelerometers
#("MAG_dcm_00",      -1.0,   1.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_01",      -1.0,   0.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_02",      -1.0,   0.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_10",      -1.0,   0.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_11",      -1.0,   1.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_12",      -1.0,   0.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_20",      -1.0,   0.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_21",      -1.0,   0.0,    1.0,    "f", "default",     "NULL"),
#("MAG_dcm_22",      -1.0,   1.0,    1.0,    "f", "default",     "NULL"),

# ADIS settings
("ADIS_smplrtdiv",  12,         24,         246,        "u", "default",     "Divider for ADIS's 2460Hz sample rate"),

# common MPU6050 settings
("MPU_dlpf",        0,          5,          6,          "u", "default",     "If dlpf>0 than use internal LPF and internal sample rate divider.\nOtherwise use 1kHz sample rate with external FIR and external decimator"),
("MPU_smplrtdiv",   1,          10,         100,        "u", "default",     "Divider for MPU's 1kHz sample rate"),
("MPU_fir_f",       -1,         0,          6,          "i", "default",     "Cut off frequency of the external FIR filter (F = 2^N). Set -1 to disable filter at all."),
("MPU_gyr_fs",      0,          1,          3,          "u", "default",     "MPU gyroscope full scale (0 - 250, 1 - 500, 2 - 1000, 3 - 2000) deg/s"),
("MPU_acc_fs",      0,          3,          3,          "u", "default",     "MPU accelerometer full scale (0 - 2, 1 - 4, 2 - 8, 3 - 16) g"),

# MPU6050 gyroscopes' settings
("MPUG_xt_c0",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),
("MPUG_xt_c1",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),
("MPUG_xt_c2",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),

("MPUG_yt_c0",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),
("MPUG_yt_c1",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),
("MPUG_yt_c2",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),

("MPUG_zt_c0",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),
("MPUG_zt_c1",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),
("MPUG_zt_c2",      -1000,      0,          1000,       "f", "default",     "Coefficient for thermal zero compensation polynomial"),

("MPUG_xsens",      0.9,        1,          1.1,        "f", "default",     "Sensitivity correction"),
("MPUG_ysens",      0.9,        1,          1.1,        "f", "default",     "Sensitivity correction"),
("MPUG_zsens",      0.9,        1,          1.1,        "f", "default",     "Sensitivity correction"),

("MPUG_xpol",       -1,         1,          1,          "i", "polarity",    "NULL"),
("MPUG_ypol",       -1,         1,          1,          "i", "polarity",    "NULL"),
("MPUG_zpol",       -1,         1,          1,          "i", "polarity",    "NULL"),
("MPUG_sortmtrx",   0,          0b100010001,1,          "u", "sort_mtrx",   "Sorting matrix for acquired gyro values\\nto correspond with real device axis"),

("MPUG_zerocnt",    512,        2048,       16384,      "i", "default",     "Sample count for zeroing procedure"),
("MPUG_zeroflen",   2,          512,        2048,       "i", "default",     "Filter length used in zero calibration routine"),
("MPUG_stillthr",   0,          0.1,        1,          "f", "default",     "Stillness threshold Rad/S"),

# MPU6050 accelerometers' settings
("ACC_xoffset",     -100,       0,          100,        "i", "default",     "NULL"),
("ACC_yoffset",     -100,       0,          100,        "i", "default",     "NULL"),
("ACC_zoffset",     -100,       0,          100,        "i", "default",     "NULL"),
#/* sens LSB/g, nominals: 4096, 8192, 16384 ****/
("ACC_xsens",       1000,       8192,       27000,      "i", "default",     "sens LSB/g, nominals: 4096, 8192, 16384"),
("ACC_ysens",       1000,       8192,       27000,      "i", "default",     "sens LSB/g, nominals: 4096, 8192, 16384"),
("ACC_zsens",       1000,       8192,       27000,      "i", "default",     "sens LSB/g, nominals: 4096, 8192, 16384"),
#/* axis polarities. Relative to device axis  */
("ACC_xpol",        -1,         1,          1,          "i", "polarity",    "NULL"),
("ACC_ypol",        -1,         1,          1,          "i", "polarity",    "NULL"),
("ACC_zpol",        -1,         1,          1,          "i", "polarity",    "NULL"),
("ACC_sortmtrx",    0,      0b100010001,    1,          "u", "sort_mtrx",   "Sorting matrix for acquired gyro values\\nto correspond with real device axis"),
("ACC_still_thr",   0,          0.006,      0.1,        "f", "default",     "Device immobility threshold in g"),
("ACC_still_flen",  1,          256,        2048,       "i", "default",     "Length of filter used in immobility detector"),

#/**** PMU - pressure measurement unit ****/
#// coefficients for thermal compensation
("PMU_above_msl",   -200,       255,        4000,       "i", "default",     "Height of barometric sensor above sea level in meters"),
("PMU_reserved1",   -2000000,   0,          2000000,    "i", "default",     "NULL"),
("PMU_npa700_mid",  8000,       8192,       8384,       "u", "default",     "Mid point of sensor in raw values"),
("PMU_npa700_sens", 5,          5.3402502,  6,          "f", "default",     "Sensor sensitivity Pa/raw"),

#/**** ADC coefficients ****/
("ADC_car_I_k",     -1000000,   0,          1000000,    "i", "default",     "k coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor ground rover"),
("ADC_car_I_b",     -1000000,   0,          1000000,    "i", "default",     "b coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor ground rover"),
#// secondary voltage. на столько надо умножить, чтобы получить uV
("ADC_second_v",    0,          0,          122400,     "u", "default",     "Multiplier for achieving uV from raw ADC value"),
#// main voltage. на столько надо умножить, чтобы получить uV
("ADC_main_v",      0,          0,          122400,     "u", "default",     "Multiplier for achieving uV from raw ADC value"),
("ADC_plane_I_k",   -1000000,   0,          1000000,    "i", "default",     "k coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor fixed wing"),
("ADC_plane_I_b",   -1000000,   0,          1000000,    "i", "default",     "b coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor fixed wing"),
("ADC_mpxv_shift",  0,          128,        255,        "u", "default",     "Offset cancelation. Uses AD5200."),

#/**** Bttery parameters ****/
("BAT_cells",       1,          2,          8,          "u", "default",     "Battery cell count"),
("BAT_chemistry",   0,          0,          1,          "u", "default",     "Battery chemistry type for critical threshold calculation"),
("BAT_ignore",      0,          1,          1,          "u", "default",     "Ignore battery voltage check"),

# Fusion algorithm settings
("SINS_restart",    0,          1,          0xFFFF,     "u", "default",     "Change this value to enforce sins restart"),
("SINS_c_alg_t",    0,          5,          90,         "f", "default",     "Coarse align time"),
("SINS_f_alg_t",    0,          15,         60,         "f", "default",     "Finale align time"),

("SINS_en_gnss",    0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_baro",    0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_odo",     0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_nhl_y",   0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_nhl_z",   0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_roll",    0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_pitch",   0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_yaw",     0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_mg_v",    0,          1,          1,          "u", "default",     "NULL"),
("SINS_en_mg_yaw",  0,          1,          1,          "u", "default",     "NULL"),
("SINS_zupt_src",   0,          1,          2,          "u", "default",     "NULL"),

("SINS_R_pos_sns",  0.001,      5,          10000,      "f", "default",     "NULL"),
("SINS_R_vel_sns",  0.001,      0.2,        10000,      "f", "default",     "NULL"),
("SINS_R_odo",      0.001,      0.1,        100,        "f", "default",     "NULL"),
("SINS_R_nhl_y",    0.001,      0.1,        100,        "f", "default",     "NULL"),
("SINS_R_nhl_z",    0.001,      0.1,        100,        "f", "default",     "NULL"),
("SINS_R_baro",     0.001,      0.3,        100,        "f", "default",     "NULL"),
("SINS_R_mag",      0.001,      0.3,        100,        "f", "default",     "NULL"),
("SINS_R_euler",    0.001,      0.01,       100,        "f", "default",     "NULL"),
("SINS_R_zihr",     0.001,      0.01,       100,        "f", "default",     "NULL"),

("SINS_R_vn_st",    0.0001,     0.01,       10,         "f", "default",     "NULL"),
("SINS_R_vv_st",    0.0001,     0.01,       10,         "f", "default",     "NULL"),
("SINS_R_yaw_st",   0.0001,     0.001,      100,        "f", "default",     "NULL"),
("SINS_R_yaw_mg",   0.0001,     0.001,      100,        "f", "default",     "NULL"),

("SINS_P_ned",      0.1,        50,         1000,       "f", "default",     "NULL"),
("SINS_P_acc_b",    0.01,       0.5,        5,          "f", "default",     "NULL"),
("SINS_P_gyr_b",    0.001,      0.1,        30,         "f", "default",     "NULL"),


("SINS_B_acc_b",    0.0,        1,          50000,      "f", "default",     "NULL"),
("SINS_B_gyr_b",    0.0,        1,          50000,      "f", "default",     "NULL"),

("SINS_Qm_acc",     0.0000001,  0.001,      1,          "f", "default",     "NULL"),
("SINS_Qm_gyr",     0.0000001,  0.001,      1,          "f", "default",     "NULL"),
("SINS_Qm_acc_x",   0.0000001,  0.000001,   1,          "f", "default",     "NULL"),
("SINS_Qm_acc_y",   0.0000001,  0.000001,   1,          "f", "default",     "NULL"),
("SINS_Qm_acc_z",   0.0000001,  0.000001,   1,          "f", "default",     "NULL"),
("SINS_Qm_gyr_bias",0.0000001,  0.001,      1,          "f", "default",     "NULL"),

("SINS_eu_vh_roll", -3.2,       0,          3.2,        "f", "default",     "rad"),
("SINS_eu_vh_pitch",-1.6,       0,          1.6,        "f", "default",     "rad"),
("SINS_eu_vh_yaw",  -6.4,       0,          6.4,        "f", "default",     "rad"),

("SINS_init_lat",   -90,        0,          90,         "f", "default",     "deg"),
("SINS_init_lon",   -180,       0,          180,        "f", "default",     "deg"),
("SINS_init_alt",   -200,       0,          5000,       "f", "default",     "m"),
("SINS_init_yaw",   0,          0,          360,        "f", "default",     "deg"),

("SINS_acc_bias_x", -1000,      0,          1000,       "f", "default",     "m/s^2"),
("SINS_acc_bias_y", -1000,      0,          1000,       "f", "default",     "m/s^2"),
("SINS_acc_bias_z", -1000,      0,          1000,       "f", "default",     "m/s^2"),

("SINS_gyr_bias_x", -1000,      0,          1000,       "f", "default",     "rad"),
("SINS_gyr_bias_y", -1000,      0,          1000,       "f", "default",     "rad"),
("SINS_gyr_bias_z", -1000,      0,          1000,       "f", "default",     "rad"),

("SINS_acc_scale_x",-3,         1,          3,          "f", "default",     "NULL"),
("SINS_acc_scale_y",-3,         1,          3,          "f", "default",     "NULL"),
("SINS_acc_scale_z",-3,         1,          3,          "f", "default",     "NULL"),

("SINS_acc_nort_0", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_acc_nort_1", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_acc_nort_2", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_acc_nort_3", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_acc_nort_4", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_acc_nort_5", -3,         0,          3,          "f", "default",     "NULL"),

("SINS_gyr_scale_x",-3,         1,          3,          "f", "default",     "NULL"),
("SINS_gyr_scale_y",-3,         1,          3,          "f", "default",     "NULL"),
("SINS_gyr_scale_z",-3,         1,          3,          "f", "default",     "NULL"),

("SINS_gyr_nort_0", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_gyr_nort_1", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_gyr_nort_2", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_gyr_nort_3", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_gyr_nort_4", -3,         0,          3,          "f", "default",     "NULL"),
("SINS_gyr_nort_5", -3,         0,          3,          "f", "default",     "NULL"),

("GLRT_acc_sigma",  0.0001,     0.01,       1,          "f", "default",     "NULL"),
("GLRT_gyr_sigma",  0.00001,    0.01,       0.5,        "f", "default",     "NULL"),
("GLRT_gamma",      0,          1,          100,        "f", "default",     "NULL"),
("GLRT_samples",    1,          5,          30,         "u", "default",     "NULL"),

#/**** Servos coefficients ****/
("SRV_ail_min",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_ail_max",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_ail_mid",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_ele_min",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_ele_max",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_ele_mid",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_rud_min",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_rud_max",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_rud_mid",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_thr_min",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_thr_max",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_thr_mid",     SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_4_min",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_4_max",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_4_mid",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_5_min",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_5_max",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_5_mid",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_6_min",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_6_max",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_6_mid",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_7_min",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_7_max",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
("SRV_7_mid",       SERVO_MIN,  1500,       SERVO_MAX,  "u", "default",     "NULL"),
#/* car specific settings */
("SRV_rud_dz",      1,          16,         1500,       "u", "default",     "NULL"),
("SRV_thr_dz",      1,          16,         1500,       "u", "default",     "NULL"),

# Radio control settings
("RC_timeout",      500,        2000,       10000,      "u", "default",     "NULL"),
("RC_map_man",     -1,          0,          3,          "i", "default",     "NULL"),

# PID settings
# ("PID_ail_h_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_h_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_h_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_h_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_ail_h_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_h_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_ail_m_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_m_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_m_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_m_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_ail_m_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_m_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_ail_l_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_l_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_l_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_l_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_ail_l_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ail_l_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_ele_h_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_h_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_h_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_h_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_ele_h_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_h_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_ele_m_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_m_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_m_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_m_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_ele_m_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_m_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_ele_l_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_l_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_l_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_l_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_ele_l_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_ele_l_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_rud_h_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_h_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_h_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_h_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_rud_h_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_h_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_rud_m_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_m_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_m_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_m_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_rud_m_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_m_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_rud_l_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_l_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_l_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_l_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_rud_l_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_rud_l_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_thr_h_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_h_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_h_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_h_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_thr_h_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_h_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_thr_m_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_m_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_m_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_m_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_thr_m_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_m_B",     0,          0,          1,          "u", "default",     "NULL"),
#
# ("PID_thr_l_P",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_l_I",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_l_D",     PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_l_Min",   -PID_MAX,   PID_MIN,    PID_MIN,    "f", "default",     "NULL"),
# ("PID_thr_l_Max",   PID_MIN,    PID_MIN,    PID_MAX,    "f", "default",     "NULL"),
# ("PID_thr_l_B",     0,          0,          1,          "u", "default",     "NULL"),

# brand new PIDs initialization
("PID_00_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_00_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_00_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_00_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_00_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_00_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_01_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_01_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_01_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_01_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_01_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_01_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_02_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_02_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_02_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_02_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_02_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_02_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_03_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_03_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_03_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_03_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_03_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_03_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_04_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_04_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_04_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_04_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_04_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_04_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_05_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_05_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_05_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_05_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_05_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_05_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_06_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_06_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_06_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_06_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_06_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_06_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_07_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_07_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_07_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_07_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_07_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_07_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_08_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_08_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_08_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_08_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_08_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_08_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_09_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_09_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_09_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_09_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_09_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_09_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_10_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_10_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_10_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_10_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_10_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_10_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_11_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_11_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_11_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_11_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_11_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_11_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_12_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_12_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_12_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_12_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_12_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_12_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_13_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_13_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_13_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_13_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_13_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_13_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_14_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_14_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_14_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_14_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_14_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_14_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_15_P",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_15_I",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_15_D",        PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_15_Min",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_15_Max",      PID_MIN,    0,          PID_MAX,    "f", "default",     "NULL"),
("PID_15_proc",     0,          0,          2,          "u", "default",     "NULL"),

("PID_vm_scale_00", -10000,     1,          10000,      "f", "default",     "NULL"),
("PID_vm_scale_01", -10000,     1,          10000,      "f", "default",     "NULL"),
("PID_vm_scale_02", -10000,     1,          10000,      "f", "default",     "NULL"),
("PID_vm_scale_03", -10000,     1,          10000,      "f", "default",     "NULL"),



#/* intervals between sending different data (mS) */
("T_attitude",      SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_debug",         SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_debug_vect",    SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_debug_mnr",     SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_global_pos",    SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_gps_raw_int",   SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_gps_status",    SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_highres_imu",   SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_nav_output",    SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_mission_curr",  SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_position_ned",  SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_raw_imu",       SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "Interval of sending this data in milliseconds.\\nSet it to 0 for disabling"),
("T_raw_press",     SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_rc",            SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_rc_scaled",     SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_scal_imu",      SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_scal_press",    SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_sys_status",    SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_system_time",   SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),
("T_vfr_hud",       SEND_MIN,   0,          SEND_MAX,   "u", "send_tmo",    "NULL"),

#/* Timezone. */
("TIME_zone",       -24,        0,          24,         "i", "default",    "Simple offset in hours."),

#/* Length of filters for different systems. */
("FLEN_adc",        1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_pres_dyn",   1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_pres_stat",  1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_climb",      1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_gnd_speed",  1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_reserved2",  1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_reserved3",  1,          16,         32768,      "i", "default",    "NULL"),
("FLEN_reserved4",  1,          16,         32768,      "i", "default",    "NULL"),

#/*  */
("SPD_pulse2m",     0.0,        0.0555555,  1.0,        "f", "default",    "Multiply odometer pulses count by this coefficient to get\\ntrip in meters. Coarse value is 0.05555555"),
("SPD_trgt_speed",  0.0,        0,          60.0,       "f", "default",    "NULL"),
("SPD_speed_max",   0.0,        0,          60.0,       "f", "default",    "NULL"),

# GNSS settings
("GNSS_dyn_model",  0,          8,          8,          "u", "default",    "0 - portable, 2 - stationary, 3 - pedestrian 4 - automotive, 5 - sea,\\n 6 - airborne with <1g Acceleration, 7 - airborne with <2g Acceleration, 8 - airborne with <4g Acceleration"),
("GNSS_fix_period", 100,        200,        1000,       "u", "default",    "NULL"),

#/**** fake field with 14 symbols name ****/
("param_end_mark__",0,      0,      1224,   "u", "default",    "Fake parameter with maximum allowable name length"),
]


