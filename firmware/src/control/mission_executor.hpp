#ifndef MISSION_EXECUTOR_HPP_
#define MISSION_EXECUTOR_HPP_

#include "mavlink_local.hpp"
#include "stabilizer/stabilizer.hpp"

namespace control {

/**
 *
 */
enum class MissionState {
  uninit,
  idle,
  navigate,
  maneuver
};

/**
 *
 */
class MissionExecutor {
public:
  MissionExecutor(void);
  void start(void);
  void stop(void);
  void launch(void);
  void update(StabInput &stab, float dT);
  void setHome(void);
private:
  void maneuver(void);
  void navigate(void);
  mavlink_mission_item_t segment[3];
  MissionState state;
};

} /* namespace */

#endif /* MISSION_EXECUTOR_HPP_ */
