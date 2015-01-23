#ifndef CMD_EXECUTOR_H_
#define CMD_EXECUTOR_H_

#include "message.hpp"
#include "acs.hpp"
#include "attitude_unit.hpp"

class CmdExecutor : public BaseStaticThread<512>{
public:
  CmdExecutor(ACS &acs, AttitudeUnit &attitude_unit);
  msg_t main(void);

private:
  void executeCmd(mavlink_command_long_t *clp);
  enum MAV_RESULT cmd_calibration_handler(mavlink_command_long_t *cl);
  enum MAV_RESULT cmd_reboot_shutdown_handler(mavlink_command_long_t *cl);
  enum MAV_RESULT cmd_preflight_storage_handler(mavlink_command_long_t *cl);
  enum MAV_RESULT cmd_do_set_mode_handler(mavlink_command_long_t *cl);
  enum MAV_RESULT cmd_nav_roi_handler(mavlink_command_long_t *cl);
  mavlink_mission_item_t mi;  // cached mission item to reduce loading delay
  ACS &acs;
  AttitudeUnit &attitude_unit;
};

#endif /* CMD_EXECUTOR_H_ */
