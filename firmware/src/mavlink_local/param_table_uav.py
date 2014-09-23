#!/usr/bin/env python
# -*- coding: utf-8 -*-


SERVO_MIN   = 1000
SERVO_MAX   = 2000

SEND_MIN    = 10
SEND_MAX    = 5000

ONBOARD_PARAM_NAME_LENGTH = 15

#    key            min     default max     type checker    help
param = [
("SYS_id",          1,      20,     255,    "u", "default",     "System ID.\\n This value MUST BE FIRST in param structure. Value 0 reserved for ground station."),
("SYS_mavtype",     0,      10,     16,     "u", "default",     "Autopilot type (0 - generic, 1 - fixed wing, 10 - ground rover).\\nOther types you can found in enum MAV_TYPE \\nNOTE! You MUST REBOOT device after changing it."),
("SH_over_radio",   0,      0,      1,      "u", "default",     "When 1 than drop shell on xbee channel and telemetry on USB_CDC and vice versa."),

#/* veights of different components */
("AHRS_accweight",  0.0,    0.005,  0.5,    "f", "default",     "NULL"),
("AHRS_magweight",  0.0,    0.05,   0.9,    "f", "default",     "NULL"),
("AHRS_gpsweight",  0.0,    0.05,   0.5,    "f", "default",     "NULL"),
("AHRS_beta",       0.0,    1.0,    20.0,   "f", "default",     "Error rate of gyro in degrees"),
("AHRS_zeta",       0.0,    1.0,    20.0,   "f", "default",     "NULL"),
("AHRS_ahrsmode",   0,      0,      1,      "u", "default",     "0 - Starlino, 1 - Madgwick"),

# /**** Magnetometer ****/
# /* zero offsets */
("MAG_xoffset",     -500,   0,      500,   "f", "default",     "Offset in uT"),
("MAG_yoffset",     -500,   0,      500,   "f", "default",     "Offset in uT"),
("MAG_zoffset",     -500,   0,      500,   "f", "default",     "Offset in uT"),
# /* axis sensitivity */
("MAG_xsens",       0.001,  0.1,    100.0,  "f", "default",     "Divide by this value to acquire uT"),
("MAG_ysens",       0.001,  0.1,    100.0,  "f", "default",     "Divide by this value to acquire uT"),
("MAG_zsens",       0.001,  0.1,    100.0,  "f", "default",     "Divide by this value to acquire uT"),
# /* axis polarities. Relative to device axis */
("MAG_xpol",        -1,     1,      1,      "i", "polarity",    "NULL"),
("MAG_ypol",        -1,     1,      1,      "i", "polarity",    "NULL"),
("MAG_zpol",        -1,     1,      1,      "i", "polarity",    "NULL"),
#
("MAG_declinate",   -90,    7,      90,     "f", "default",     "Magnetic declination. \\nThe declination is positive when the magnetic north is east of true north. \\nhttp://www.ngdc.noaa.gov/geomagmodels/Declination.jsp"),
("MAG_still_thr",    0,     1,      20,     "f", "default",     "Device immobility threshold in parrots"),
("MAG_still_flen",  1,    256,      2048,   "i", "default",     "Length of filter used in immobility detector"),
# /* count of samples to zero calibration */
("MAG_zeroflen",    1,    256,      2048,   "i", "default",     "Length of filter used in immobility detector"),
("MAG_zerocnt",     256,    512,    4096,   "u", "default",     "NULL"),
("MAG_sortmtrx",    0, 0b100010001, 1,      "u", "sort_mtrx",   "Sorting matrix for acquired gyro values\\nto correspond with real device axis"),
("MAG_vectorlen",   0,      10,     512,    "f", "default",     "Length of magnetic flux vector in uT acquired during sphere offset calculation"),
# ellipsoid correction coefficients
("MAG_ellip_1",    -5.0,   1.0,      5.0,    "f", "default",     "ellipsoid correction coefficient"),
("MAG_ellip_2",    -5.0,   1.0,      5.0,    "f", "default",     "ellipsoid correction coefficient"),
("MAG_ellip_3",    -5.0,   1.0,      5.0,    "f", "default",     "ellipsoid correction coefficient"),
("MAG_ellip_4",    -5.0,   0.0,      5.0,    "f", "default",     "ellipsoid correction coefficient"),
("MAG_ellip_5",    -5.0,   0.0,      5.0,    "f", "default",     "ellipsoid correction coefficient"),
("MAG_ellip_6",    -5.0,   0.0,      5.0,    "f", "default",     "ellipsoid correction coefficient"),
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
# ("MAG_picklmode",   0,      0,      1,      "u", "default",     "0 - simple spherical shift, 1 - egg compensate"),

#/**** accelerometer ****/
#/* zero offsets */
("ACC_xoffset",     -100,   0,      100,    "i", "default",     "NULL"),
("ACC_yoffset",     -100,   0,      100,    "i", "default",     "NULL"),
("ACC_zoffset",     -100,   0,      100,    "i", "default",     "NULL"),
#/* sens LSB/g, nominals: 4096, 8192, 16384 ****/
("ACC_xsens",       1000,   8192,   27000,  "i", "default",     "sens LSB/g, nominals: 4096, 8192, 16384"),
("ACC_ysens",       1000,   8192,   27000,  "i", "default",     "sens LSB/g, nominals: 4096, 8192, 16384"),
("ACC_zsens",       1000,   8192,   27000,  "i", "default",     "sens LSB/g, nominals: 4096, 8192, 16384"),
#/* axis polarities. Relative to device axis  */
("ACC_xpol",        -1,     1,      1,      "i", "polarity",    "NULL"),
("ACC_ypol",        -1,     1,      1,      "i", "polarity",    "NULL"),
("ACC_zpol",        -1,     1,      1,      "i", "polarity",    "NULL"),
("ACC_sortmtrx",    0, 0b100010001, 1,      "u", "sort_mtrx",   "Sorting matrix for acquired gyro values\\nto correspond with real device axis"),
("ACC_still_thr",   0,  0.006,      0.1,    "f", "default",     "Device immobility threshold in g"),
("ACC_still_flen",  1,    256,      2048,   "i", "default",     "Length of filter used in immobility detector"),

#/* Gyroscope */
#/* sens (LSB/(deg/s)) */
("GYRO_xsens",      7.0,    14.375, 30.0,   "f", "default",     "NULL"),
("GYRO_ysens",      7.0,    14.375, 30.0,   "f", "default",     "NULL"),
("GYRO_zsens",      7.0,    14.375, 30.0,   "f", "default",     "NULL"),
#/* polarities */
("GYRO_xpol",       -1,     1,      1,      "i", "polarity",    "NULL"),
("GYRO_ypol",       -1,     1,      1,      "i", "polarity",    "NULL"),
("GYRO_zpol",       -1,     1,      1,      "i", "polarity",    "NULL"),
# offsets in parrots
("GYRO_xoffset",   -2000,   0,      2000,   "f", "default",     "Cached zero offset for faster startup"),
("GYRO_yoffset",   -2000,   0,      2000,   "f", "default",     "Cached zero offset for faster startup"),
("GYRO_zoffset",   -2000,   0,      2000,   "f", "default",     "Cached zero offset for faster startup"),
# other
("GYRO_zerocnt",    512,    2048,   16384,  "i", "default",     "Sample count for zeroing procedure"),
("GYRO_sortmtrx",   0,  0b100010001,1,      "u", "sort_mtrx",   "Sorting matrix for acquired gyro values\\nto correspond with real device axis"),
("GYRO_zeroflen",   2,      512,    2048,   "i", "default",     "Filter length used in zero calibration routine"),
("GYRO_sendangle",  0,      1,      1,      "u", "default",     "1 - send intergrated angles, 0 - angular rates"),
("GYRO_stillthr",   0,      0.1,    1,      "f", "default",     "Rad/S"),

#/**** PMU - pressure measurement unit ****/
#// coefficients for thermal compensation
("PMU_above_msl",   -200,   255,    4000,   "i", "default",     "Height of barometric sensor above sea level in meters"),
("PMU_reserved1",   -2000000, 0, 2000000,   "i", "default",     "NULL"),

#/**** ADC coefficients ****/
("ADC_car_I_k",     -1000000, 0, 1000000,   "i", "default",     "k coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor ground rover"),
("ADC_car_I_b",     -1000000, 0, 1000000,   "i", "default",     "b coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor ground rover"),
#// secondary voltage. на столько надо умножить, чтобы получить uV
("ADC_SV_gain",     0,      0,      122400, "u", "default",     "NULL"),
#// main voltage. на столько надо умножить, чтобы получить uV
("ADC_MV_gain",     0,      0,      122400, "u", "default",     "NULL"),
("ADC_plane_I_k",   -1000000, 0, 1000000,   "i", "default",     "k coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor fixed wing"),
("ADC_plane_I_b",   -1000000, 0, 1000000,   "i", "default",     "b coefficient for calculation from ADC values to uA using formulae y=kx+b\\nfor fixed wing"),

#/**** Bttery parameters ****/
("BAT_cap",         0,      3000,   11000,  "u", "default",     "Battery capacitance (mAh)"),
("BAT_fill",        0,      0,      100,    "u", "default",     "Start battery filling in percents"),

#/**** Servos coefficients ****/
("SERVO_0_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_0_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_0_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_1_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_1_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_1_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_2_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_2_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_2_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_3_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_3_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_3_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_4_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_4_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_4_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_5_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_5_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_5_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_6_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_6_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_6_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_7_min",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_7_max",     SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_7_neutr",   SERVO_MIN, 1500, SERVO_MAX, "u", "default","NULL"),
#/* car specific settings (throttle) */
("SERVO_car_max",   1,         1500, SERVO_MAX, "u", "default","NULL"),
("SERVO_car_dz",    1,         16,   64,        "u", "default","NULL"),

#/* intervals between sending different data (mS) */
("T_raw_imu",       SEND_MIN, 0, SEND_MAX, "u", "send_tmo","Interval of sending this data in milliseconds.\\nSet it to 0 for disabling"),
("T_scal_imu",      SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_highres_imu",   SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_raw_press",     SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_scal_press",    SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_attitude",      SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_vfr_hud",       SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_gps_int",       SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_sys_status",    SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_position_ned",  SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),
("T_nav_output",    SEND_MIN, 0, SEND_MAX, "u", "send_tmo","NULL"),

#/* Timezone. */
("TIME_zone",       -24,    0,      24,     "i", "default",    "Simple offset in hours."),

#/* Length of filters for different systems. */
("FLEN_adc",        1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_pres_dyn",   1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_pres_stat",  1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_climb",      1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_gnd_speed",  1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_reserved2",  1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_reserved3",  1,      16,     32768,  "i", "default",    "NULL"),
("FLEN_reserved4",  1,      16,     32768,  "i", "default",    "NULL"),

#/*  */
("SPD_pulse2m",     0.0,    0.0555555, 1.0, "f", "default",    "Multiply odometer pulses count by this coefficient to get\\ntrip in meters. Coarse value is 0.05555555"),
#/**** pid coefficients for speed control ****/
("SPD_iGain",       -1.0,     0, 1.0,       "f", "default",    "NULL"),
("SPD_pGain",       -200.0,   0, 200.0,     "f", "default",    "NULL"),
("SPD_dGain",       -10000.0, 0, 10000.0,   "f", "default",    "NULL"),
("SPD_iMin",        -10000.0, 0, 10000.0,   "f", "default",    "NULL"),
("SPD_iMax",        -10000.0, 0, 10000.0,   "f", "default",    "NULL"),
#/**** pid coefficients for heading control ****/
("HDG_iGain",      -1.0,     0,  1.0,       "f", "default",    "NULL"),
("HDG_pGain",      -200.0,   0,  200.0,     "f", "default",    "NULL"),
("HDG_dGain",      -10000.0, 0,  10000.0,   "f", "default",    "NULL"),
("HDG_iMin",       -10000.0, 0,  10000.0,   "f", "default",    "NULL"),
("HDG_iMax",       -10000.0, 0,  10000.0,   "f", "default",    "NULL"),
#/* device's speed limits */
("SPD_speed",       0.0,    0,      60.0,   "f", "default",    "NULL"),
("SPD_speed_max",   0.0,    0,      60.0,   "f", "default",    "NULL"),
#/**** pid coefficients for cross track correction ****/
("XTRACK_iGain",    -1.0,     0, 1.0,       "f", "default",    "NULL"),
("XTRACK_pGain",    -200.0,   0, 200.0,     "f", "default",    "NULL"),
("XTRACK_dGain",    -10000.0, 0, 10000.0,   "f", "default",    "NULL"),
("XTRACK_iMin",     -10000.0, 0, 10000.0,   "f", "default",    "NULL"),
("XTRACK_iMax",     -10000.0, 0, 10000.0,   "f", "default",    "NULL"),
# MPU6050 tuning
("MPU_dlpf",        1,      1,      6,      "u", "default",    "NULL"),

#/**** Variables for debugging purpose ****/
("DBG_lat",         -360,   0,      360,    "f", "default",    "NULL"),
("DBG_lon",         -360,   0,      360,    "f", "default",    "NULL"),
("DBG_reserved2",   0,      0,      10000,  "f", "default",    "NULL"),

#/**** fake field with 14 symbols name ****/
("param_end_mark",  0,      0,      1224,   "u", "default",    "Fake parameter with maximum allowable name length"),
]


