/*
 * Some functions was moved to this file to reduce copypasta size
 * between test and main code
 */



void Navi6dWrapper::read_settings(void) {

  param_registry.valueSearch("T_debug",         &T_debug);
  param_registry.valueSearch("T_debug_vect",    &T_debug_vect);

  param_registry.valueSearch("SINS_en_gnss",    &en_gnss);

  param_registry.valueSearch("SINS_en_odo",     &en_odo);
  param_registry.valueSearch("SINS_en_nhl_y",   &en_nhl_y);
  param_registry.valueSearch("SINS_en_nhl_z",   &en_nhl_z);

  param_registry.valueSearch("SINS_en_baro",    &en_baro);
  param_registry.valueSearch("SINS_en_roll",    &en_roll);
  param_registry.valueSearch("SINS_en_pitch",   &en_pitch);
  param_registry.valueSearch("SINS_en_yaw",     &en_yaw);

  param_registry.valueSearch("SINS_en_mg_v",    &en_mg_v);
  param_registry.valueSearch("SINS_en_mg_yaw",  &en_mg_yaw);
  param_registry.valueSearch("SINS_zupt_src",   &zupt_src);

  param_registry.valueSearch("SINS_R_ne_sns",   &R_ne_sns);
  param_registry.valueSearch("SINS_R_d_sns",    &R_d_sns);
  param_registry.valueSearch("SINS_R_v_n_sns",  &R_v_n_sns);
  param_registry.valueSearch("SINS_R_odo",      &R_odo);
  param_registry.valueSearch("SINS_R_nhl_y",    &R_nhl_y);
  param_registry.valueSearch("SINS_R_nhl_z",    &R_nhl_z);
  param_registry.valueSearch("SINS_R_baro",     &R_baro);
  param_registry.valueSearch("SINS_R_mag",      &R_mag);
  param_registry.valueSearch("SINS_R_euler",    &R_euler);

  param_registry.valueSearch("SINS_R_vn_st",    &R_v_nav_st);
  param_registry.valueSearch("SINS_R_vv_st",    &R_v_veh_st);
  param_registry.valueSearch("SINS_R_yaw_st",   &R_yaw_st);
  param_registry.valueSearch("SINS_R_yaw_mg",   &R_mag_yaw);

  param_registry.valueSearch("SINS_P_ned",      &P_ned);
  param_registry.valueSearch("SINS_P_acc_b",    &P_acc_b);
  param_registry.valueSearch("SINS_P_gyr_b",    &P_gyr_b);

  param_registry.valueSearch("SINS_B_acc_b",    &B_acc_b);
  param_registry.valueSearch("SINS_B_gyr_b",    &B_gyr_b);

  param_registry.valueSearch("SINS_init_lat",   &init_lat);
  param_registry.valueSearch("SINS_init_lon",   &init_lon);
  param_registry.valueSearch("SINS_init_alt",   &init_alt);
  param_registry.valueSearch("SINS_init_yaw",   &init_yaw);

  param_registry.valueSearch("SINS_Qm_acc",     &Qm_acc);
  param_registry.valueSearch("SINS_Qm_gyr",     &Qm_gyr);
  param_registry.valueSearch("SINS_Qm_acc_x",   &Qm_acc_x);
  param_registry.valueSearch("SINS_Qm_acc_y",   &Qm_acc_y);
  param_registry.valueSearch("SINS_Qm_acc_z",   &Qm_acc_z);
  param_registry.valueSearch("SINS_Qm_gyr_bias",&Qm_gyr_bias);

  param_registry.valueSearch("SINS_eu_vh_roll", &eu_vh_roll);
  param_registry.valueSearch("SINS_eu_vh_pitch",&eu_vh_pitch);
  param_registry.valueSearch("SINS_eu_vh_yaw",  &eu_vh_yaw);

  param_registry.valueSearch("SINS_acc_bias_x", &acc_bias_x);
  param_registry.valueSearch("SINS_acc_bias_y", &acc_bias_y);
  param_registry.valueSearch("SINS_acc_bias_z", &acc_bias_z);

  param_registry.valueSearch("SINS_gyr_bias_x", &gyr_bias_x);
  param_registry.valueSearch("SINS_gyr_bias_y", &gyr_bias_y);
  param_registry.valueSearch("SINS_gyr_bias_z", &gyr_bias_z);

  param_registry.valueSearch("SINS_acc_scale_x",&acc_scale_x);
  param_registry.valueSearch("SINS_acc_scale_y",&acc_scale_y);
  param_registry.valueSearch("SINS_acc_scale_z",&acc_scale_z);

  param_registry.valueSearch("SINS_gyr_scale_x",&gyr_scale_x);
  param_registry.valueSearch("SINS_gyr_scale_y",&gyr_scale_y);
  param_registry.valueSearch("SINS_gyr_scale_z",&gyr_scale_z);

  param_registry.valueSearch("SINS_acc_nort_0", &acc_nort_0);
  param_registry.valueSearch("SINS_acc_nort_1", &acc_nort_1);
  param_registry.valueSearch("SINS_acc_nort_2", &acc_nort_2);
  param_registry.valueSearch("SINS_acc_nort_3", &acc_nort_3);
  param_registry.valueSearch("SINS_acc_nort_4", &acc_nort_4);
  param_registry.valueSearch("SINS_acc_nort_5", &acc_nort_5);

  param_registry.valueSearch("SINS_gyr_nort_0", &acc_nort_0);
  param_registry.valueSearch("SINS_gyr_nort_1", &acc_nort_1);
  param_registry.valueSearch("SINS_gyr_nort_2", &acc_nort_2);
  param_registry.valueSearch("SINS_gyr_nort_3", &acc_nort_3);
  param_registry.valueSearch("SINS_gyr_nort_4", &acc_nort_4);
  param_registry.valueSearch("SINS_gyr_nort_5", &acc_nort_5);

  param_registry.valueSearch("SINS_eu_vh_roll", &eu_vh_roll);
  param_registry.valueSearch("SINS_eu_vh_pitch",&eu_vh_pitch);
  param_registry.valueSearch("SINS_eu_vh_yaw",  &eu_vh_yaw);

  param_registry.valueSearch("GLRT_acc_sigma",  &acc_sigma);
  param_registry.valueSearch("GLRT_gyr_sigma",  &gyr_sigma);
  param_registry.valueSearch("GLRT_gamma",      &gamma);
  param_registry.valueSearch("GLRT_samples",    &samples);

  param_registry.valueSearch("SINS_restart",    &restart);
}

/**
 *
 */
void Navi6dWrapper::sins_cold_start(void) {

  CommandType<klmnfp> cmd;

  cmd.command = *restart;

  cmd.param[0][0] = deg2rad(*init_lat);
  cmd.param[1][0] = deg2rad(*init_lon);
  cmd.param[2][0] = *init_alt;
  cmd.param[8][0] = deg2rad(*init_yaw);

  nav_sins.command_executor(cmd);
}
/**
 *
 */
void Navi6dWrapper::prepare_data_gnss(gnss::gnss_data_t &gnss_data) {

  if ((*en_gnss == 1) && (gnss_data.fresh) && (gnss_data.fix > 0)) {
    nav_sins.sensor_data.r_sns[0][0] = deg2rad(gnss_data.latitude);
    nav_sins.sensor_data.r_sns[1][0] = deg2rad(gnss_data.longitude);
    nav_sins.sensor_data.r_sns[2][0] = gnss_data.altitude;
    nav_sins.sensor_flags.sns_r_en = true;
    nav_sins.sensor_flags.sns_h_en = true;

    switch(gnss_data.speed_type) {
    case gnss::speed_t::SPEED_COURSE:
      nav_sins.sensor_data.v_sns[0][0] = gnss_data.speed * cos(deg2rad(gnss_data.course));
      nav_sins.sensor_data.v_sns[1][0] = gnss_data.speed * sin(deg2rad(gnss_data.course));
      nav_sins.sensor_data.v_sns[2][0] = gnss_data.course;
      nav_sins.sensor_flags.sns_v_n_en = true;
      nav_sins.sensor_flags.sns_v_e_en = true;
      nav_sins.sensor_flags.sns_v_d_en = false;
      break;
    case gnss::speed_t::VECTOR_3D:
    case gnss::speed_t::BOTH:
      for (size_t i=0; i<3; i++) {
        nav_sins.sensor_data.v_sns[i][0] = gnss_data.v[i];
      }
      nav_sins.sensor_flags.sns_v_n_en = true;
      nav_sins.sensor_flags.sns_v_e_en = true;
      nav_sins.sensor_flags.sns_v_d_en = true;
      break;
    default:
      nav_sins.sensor_flags.sns_v_n_en = false;
      nav_sins.sensor_flags.sns_v_e_en = false;
      nav_sins.sensor_flags.sns_v_d_en = false;
      break;
    }
  }

  // Important! Must be set to false after data processing
  if (gnss_data.fresh) {
    gnss_data.fresh = false;
  }
}

/*
 *
 */
void Navi6dWrapper::prepare_data(const baro_data_t &baro,
                                 const odometer_data_t &odo,
                                 const marg_data_t &marg) {

  nav_sins.ctrl_params.use_baro_alt = *en_baro;
  nav_sins.ctrl_params.use_odo = *en_odo;
  nav_sins.ctrl_params.use_nonhol_y = *en_nhl_y;
  nav_sins.ctrl_params.use_nonhol_z = *en_nhl_z;
  nav_sins.ctrl_params.use_roll = *en_roll;
  nav_sins.ctrl_params.use_pitch = *en_pitch;
  nav_sins.ctrl_params.use_yaw = *en_yaw;
  nav_sins.ctrl_params.use_mag = *en_mg_v;
  nav_sins.ctrl_params.use_mag_course = *en_mg_yaw;


  nav_sins.sensor_flags.odo_en = odo.fresh;

  nav_sins.sensor_flags.mag_en = true;
  nav_sins.sensor_flags.alt_b_en = true;

  nav_sins.sensor_data.alt_b[0][0] = baro.alt;
  nav_sins.sensor_data.v_odo[0][0] = odo.speed;

  for(size_t i=0; i<3; i++) {
    nav_sins.sensor_data.fb[i][0] = marg.acc[i];
    nav_sins.sensor_data.wb[i][0] = marg.gyr[i];
    nav_sins.sensor_data.mb[i][0] = marg.mag[i];
  }
}
