#include "main.h"

#include "mission_executor.hpp"
#include "maneuver/maneuver_infinity.hpp"
#include "waypoint_db.hpp"
#include "mav_dbg_print.hpp"
//#include "navigator_types.hpp"
#include "time_keeper.hpp"
#include "param_registry.hpp"
#include "mav_logger.hpp"
#include "mav_postman.hpp"
#include "mav_dbg_sender.hpp"
#include "geometry.hpp"
#include "maneuver/maneuver_list.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

// some convenient aliases
#define WP_RADIUS           param2

#define JUMP_REPEAT         param2
#define JUMP_SEQ            param1

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_system_t                 mavlink_system_struct;
extern mavlink_mission_current_t        mavlink_out_mission_current_struct;
extern mavlink_mission_item_reached_t   mavlink_out_mission_item_reached_struct;
extern mavlink_nav_controller_output_t  mavlink_out_nav_controller_output_struct;

extern EvtSource event_mission_reached;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
/**
 *
 */
void MissionExecutor::broadcast_mission_current(uint16_t seq) {

  mavlink_out_mission_current_struct.seq = seq;
}

/**
 *
 */
void MissionExecutor::broadcast_mission_item_reached(uint16_t seq) {

  event_mission_reached.broadcastFlags(EVMSK_MISSION_REACHED | (seq << 16));
  mavlink_out_mission_item_reached_struct.seq = seq;
}

/**
 *
 */
uint16_t MissionExecutor::jump_to_handler(uint16_t next) {

  /* protection from idiots */
  if (0 == round(third.JUMP_REPEAT))
    return next+1;

  /**/
  if (! jumpctx.active) {
    jumpctx.active = true;
    jumpctx.remain = round(third.JUMP_REPEAT);
  }
  else {
    jumpctx.remain--;
    if (0 == jumpctx.remain) {
      jumpctx.active = false;
      return next+1;
    }
  }

  return round(third.JUMP_SEQ);
}

/**
 *
 */
bool MissionExecutor::load_next_mission_item(void) {

  bool load_status = OSAL_FAILED;
  uint16_t next;

  prev = trgt;
  trgt = third;

  if (wpdb.getCount() == (third.seq + 1)) {
    /* if we fall here than last mission was not 'land'. System do not know
     * what to do so start unlimited loitering */
    third = trgt;
    third.x += 0.0001; /* singularity prevention */
    third.y += 0.0001; /* singularity prevention */
    third.command = MAV_CMD_NAV_LOITER_UNLIM;
    third.seq += 1;
    broadcast_mission_current(trgt.seq);
  }
  else if (wpdb.getCount() < (third.seq + 1)) {
    state = MissionState::completed;
    load_status = OSAL_SUCCESS;
  }
  else {
    next = third.seq + 1;
    LOAD_JUMP:
    load_status = wpdb.read(&third, next);
    if (OSAL_SUCCESS == load_status) {
      /* handle 'jump_to' command */
      if (MAV_CMD_DO_JUMP == third.command) {
        next = jump_to_handler(next);
        goto LOAD_JUMP;
      }
      else {
        state = MissionState::navigate;
        broadcast_mission_current(trgt.seq);
      }
    }
    else {
      state = MissionState::error;
    }
  }

  return load_status;
}

void MissionExecutor::analyze_execout(void) {

  if (out.crossed && out.final) {
    if (MissionComponent::landingAlignment != component) {
    maneuver_completed = true;
    part_number = 0;
    }
    else {
      double delta_alt = fabs(
              acs_in.ch[ACS_INPUT_alt]
            - acs_in.ch[ACS_INPUT_trgt_alt]);
      double dz_treshold = fabs(*aligment_dz_treshold);
      double dh_treshold = fabs(*aligment_dh_treshold);
      double dyaw_treshold = deg2rad<float>(fabs(*aligment_dyaw_treshold));

      if (   (   static_cast<double>(0.0) == dz_treshold
              || fabs(out.dz) <= dz_treshold)
          && (   static_cast<double>(0.0) == dh_treshold
              || delta_alt <= dh_treshold)
          && (   static_cast<double>(0.0) == dyaw_treshold
              || fabs(acs_in.ch[ACS_INPUT_dYaw]) <= dyaw_treshold)) {
        maneuver_completed = true;
        part_number = 0;
      }
      else {
        maneuver_completed = false;
        part_number = static_cast<uint32_t>(
            maneuver::ApproachToInfinity::count);
      }
    }
//    else {
//      maneuver_completed = true;
//      part_number = 0;
//    }
  }
  else if (out.crossed) {
    maneuver_completed = false;
    part_number++;
  }
  else {
    maneuver_completed = false;
  }
}

void MissionExecutor::execout2acsin() {
  acs_in.ch[ACS_INPUT_dZm]  = out.dz;
  acs_in.ch[ACS_INPUT_dYaw] = wrap_pi(
      acs_in.ch[ACS_INPUT_yaw]
    - static_cast<double>(out.crs));
  acs_in.ch[ACS_INPUT_trgt_alt] = out.alt;
}

void MissionExecutor::execout2mavlink() {

  mavlink_out_nav_controller_output_struct.wp_dist =
      static_cast<uint16_t>(round(fabs(out.dist)));

  mavlink_out_nav_controller_output_struct.xtrack_error =
      static_cast<float>(out.dz);

  mavlink_out_nav_controller_output_struct.target_bearing =
      rad2deg(out.crs);

  mavlink_out_nav_controller_output_struct.nav_bearing =
      rad2deg(acs_in.ch[ACS_INPUT_dYaw]);

  mavlink_out_nav_controller_output_struct.alt_error =
      acs_in.ch[ACS_INPUT_alt] - static_cast<double>(out.alt);

//  mavlink_out_nav_controller_output_struct.alt_error = out.alt;
  /* TODO: change constant values of target roll and pitch to real */
  mavlink_out_nav_controller_output_struct.nav_roll = 0;
  mavlink_out_nav_controller_output_struct.nav_pitch = 0;
  /* TODO: change odo_speed to air_speed */
  mavlink_out_nav_controller_output_struct.aspd_error =
      acs_in.ch[ACS_INPUT_trgt_speed] - acs_in.ch[ACS_INPUT_odo_speed];
}

void MissionExecutor::debug2mavlink(void) {

  mav_dbg_sender.send(
      "maneuver",
      static_cast<float>(part_number),
      static_cast<float>(acs_in.ch[ACS_INPUT_dYaw]),
      static_cast<float>(acs_in.ch[ACS_INPUT_dZm]),
      TimeKeeper::utc());
}

/*
 * Specifies current mission component depending on
 * command in previous, target and next (third) waypoints.
 */
void MissionExecutor::ident_component(void) {
  if (MAV_CMD_NAV_TAKEOFF == trgt.command) {
    /* ground phase of take-off */
    component = MissionComponent::takeoffGround;
  }
  else if (MAV_CMD_NAV_TAKEOFF == prev.command) {
    /* slope phase of take-off */
    component = MissionComponent::takeoffSlope;
  }
  else if (MAV_CMD_NAV_LAND == third.command) {
    /* alignment phase of landing */
    component = MissionComponent::landingAlignment;
  }
  else if (MAV_CMD_NAV_LAND == trgt.command) {
    /* slope phase of landing */
    component = MissionComponent::landingSlope;
  }
  else if (MAV_CMD_NAV_LAND == prev.command) {
    /* ground phase of landing */
    component = MissionComponent::landingGround;
  }
  else {
    /* mission maneuvers */
    switch (trgt.command) {
      case MAV_CMD_NAV_WAYPOINT:
        if (0.0f != trgt.param2) /* smoothing radius */
          component = MissionComponent::threePoints;
        else
          component = MissionComponent::line;
        break;

      case MAV_CMD_NAV_LOITER_TURNS:
        component = MissionComponent::circle;
        break;

      case MAV_CMD_NAV_STADIUM:
        component = MissionComponent::stadium;
        break;

      case MAV_CMD_NAV_INFINITY:
        component = MissionComponent::infinity;
        break;

      default:
        break;
    }
  }
}

void MissionExecutor::parse_component(void) {

  mnrfp localPrev[2][1];
  mnrfp localTrgt[2][1];
  double curr_wgs84[3][1] = {
        {acs_in.ch[ACS_INPUT_lat]},
        {acs_in.ch[ACS_INPUT_lon]},
        {acs_in.ch[ACS_INPUT_alt]}};

  maneuver::missionItemWGS84ToLocalNE(
      localPrev,
      curr_wgs84,
      prev);
  maneuver::missionItemWGS84ToLocalNE(
      localTrgt,
      curr_wgs84,
      trgt);

  // set target altitude from target waypoint
  part.fillAlt(trgt.z);

  switch (component) {
    case MissionComponent::line:
      maneuver::lineManeuver(part,localPrev,localTrgt);
      break;

    case MissionComponent::threePoints: {
      /* third waypoint use only here.  */
      mnrfp localThird[2][1];
      maneuver::missionItemWGS84ToLocalNE(
          localThird,
          curr_wgs84,
          third);
      maneuver::threePointsManeuver(
          part,
          part_number,
          trgt.param2,                  /* smoothing radius */
          localPrev,
          localTrgt,
          localThird);
      break;
    }

    case MissionComponent::circle:
      maneuver::circleManeuver(
          part,
          part_number,
          trgt.param1,                  /* number of repetitions */
          trgt.param3,                  /* radius of turn */
          localPrev,
          localTrgt);
      break;

    case MissionComponent::infinity:
      maneuver::infinityManeuver(
          part,
          part_number,
          trgt.param1,                  /* number of repetitions */
          trgt.param2,                  /* infinity width */
          trgt.param3,                  /* infinity height */
          deg2rad<mnrfp>(trgt.param4),  /* angle of rotation */
          localPrev,
          localTrgt);
      break;

    case MissionComponent::stadium:
      maneuver::stadiumManeuver(
          part,
          part_number,
          trgt.param1,                  /* number of repetitions */
          trgt.param2,                  /* stadium width */
          trgt.param3,                  /* stadium height */
          deg2rad<mnrfp>(trgt.param4),  /* angle of rotation */
          *default_radius,              /* radius of turn */
          localPrev,
          localTrgt);
      break;

    case MissionComponent::takeoffGround:
      maneuver::lineManeuver(part, localPrev, localTrgt);
      break;

    case MissionComponent::takeoffSlope:
      maneuver::lineSlopeManeuver(
          part,
          localPrev,
          localTrgt,
          static_cast<mnrfp>(prev.z),
          static_cast<mnrfp>(trgt.z));
      break;

    case MissionComponent::landingAlignment:
      maneuver::landingAlignmentManeuver(
          part,
          part_number,
          *aligment_width,
          *aligment_height,
          localPrev,
          localTrgt,
          static_cast<mnrfp>(prev.z),
          static_cast<mnrfp>(trgt.z));
      break;

    case MissionComponent::landingSlope:
      maneuver::lineSlopeManeuver(
          part,
          localPrev,
          localTrgt,
          static_cast<mnrfp>(prev.z),
          static_cast<mnrfp>(trgt.z));
      break;

    case MissionComponent::landingGround:
      maneuver::lineManeuver(part, localPrev, localTrgt);
      break;

    default:
      part.fillUnknown();
      break;
  }
}

/**
 *
 */
void MissionExecutor::navigate(float dT) {
  (void)dT;
//  double curr_wgs84[3][1] = {
//      {acs_in.ch[ACS_INPUT_lat]},
//      {acs_in.ch[ACS_INPUT_lon]},
//      {acs_in.ch[ACS_INPUT_alt]}};
//
//  MissionComponent component =
//      maneuver::indetifyTargetMissonItem(prev, trgt, third);
//
//  maneuver::parseMissionComponent(
//      part,
//      component,
//      part_number,
//      prev,
//      trgt,
//      third,
//      curr_wgs84);

  ident_component();
  parse_component();
  part.execute(out);
  execout2acsin();
  execout2mavlink();
  debug2mavlink();
  analyze_execout();

  if (maneuver_completed) {
    broadcast_mission_item_reached(trgt.seq);
    load_next_mission_item();
  }

}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
MissionExecutor::MissionExecutor(ACSInput &acs_in) :
  state(MissionState::uninit),
  acs_in(acs_in),
  component(MissionComponent::unknown) {

  /* fill home point with invalid data. This denotes it uninitialized. */
  memset(&home, 0xFF, sizeof(home));
  home.seq = -1;
}

/**
 *
 */
void MissionExecutor::start(void) {
  chTMObjectInit(&this->tmp_nav);

  state = MissionState::idle;

  param_registry.valueSearch("ACS_lnd_dz_trsh", &aligment_dz_treshold);
  param_registry.valueSearch("ACS_lnd_dh_trsh", &aligment_dh_treshold);
  param_registry.valueSearch("ACS_lnd_dy_trsh", &aligment_dyaw_treshold);
  param_registry.valueSearch("ACS_algmn_width", &aligment_width);
  param_registry.valueSearch("ACS_algmn_height",&aligment_height);
  param_registry.valueSearch("ACS_dflt_radius", &default_radius);

}

/**
 *
 */
void MissionExecutor::stop(void) {
  state = MissionState::uninit;
}

/**
 *
 */
void MissionExecutor::artificial_takeoff_point(void) {

  memset(&prev, 0, sizeof(prev));

  prev.x = rad2deg(acs_in.ch[ACS_INPUT_lat]);
  prev.y = rad2deg(acs_in.ch[ACS_INPUT_lon]);
  prev.z = acs_in.ch[ACS_INPUT_alt];
//  prev.command = MAV_CMD_NAV_TAKEOFF;
  prev.command = MAV_CMD_NAV_WAYPOINT;
  prev.frame = MAV_FRAME_GLOBAL;
  prev.seq = -1;

  /* store launch point */
  launch = prev;

  /* set launch point as home if it was not set yet from ground */
  if (uint16_t(-1) == home.seq) {
    home = launch;
    home.command = MAV_CMD_NAV_WAYPOINT;
  }
}


/**
 * @brief   Samples current coordinates as prev WP than append
 *          first 2 points from mission.
 */
bool MissionExecutor::takeoff(void) {
  bool read_status1 = OSAL_FAILED;
  bool read_status2 = OSAL_FAILED;

  if (wpdb.getCount() < 2) {
    mavlink_dbg_print(
        MAV_SEVERITY_INFO,
        "ACS: mission must be at least 2 WP long",
        MAV_COMP_ID_SYSTEM_CONTROL);
    return OSAL_FAILED;
  }
  else {
    artificial_takeoff_point();
    read_status1 = wpdb.read(&this->trgt,  0);
    read_status2 = wpdb.read(&this->third, 1);
    if ((OSAL_SUCCESS != read_status1) || (OSAL_SUCCESS != read_status2))
      return OSAL_FAILED;
    else {
      state = MissionState::navigate;
      return OSAL_SUCCESS;
    }
  }
}

/**
 *
 */
MissionState MissionExecutor::update(float dT) {

  osalDbgCheck(MissionState::uninit != state);

  switch (state) {
  case MissionState::navigate:
    chTMStartMeasurementX(&this->tmp_nav);
    this->navigate(dT);
    chTMStopMeasurementX(&this->tmp_nav);
    break;

  default:
    break;
  }

  return this->state;
}

/**
 *
 */
void MissionExecutor::setHome(void) {

  setHome(rad2deg(acs_in.ch[ACS_INPUT_lat]),
          rad2deg(acs_in.ch[ACS_INPUT_lon]),
          acs_in.ch[ACS_INPUT_alt]);
}

/**
 * @brief   Accepts coordinates in radians
 */
void MissionExecutor::setHome(float lat, float lon, float alt) {

  memset(&home, 0, sizeof(home));

  home.x = rad2deg(lat);
  home.y = rad2deg(lon);
  home.z = alt;
  home.command = MAV_CMD_NAV_WAYPOINT;
  home.frame = MAV_FRAME_GLOBAL;
  home.seq = -1;
}

/**
 *
 */
void MissionExecutor::forceGoHome(void) {
  return;
}

/**
 *
 */
uint16_t MissionExecutor::getCurrentMission(void) const {
  return this->trgt.seq;
}

/**
 *
 */
void MissionExecutor::forceReturnToLaunch(void) {
  return;
}

/**
 *
 */
bool MissionExecutor::forceJumpTo(uint16_t seq) {
  (void)seq;

  return OSAL_FAILED;
}
