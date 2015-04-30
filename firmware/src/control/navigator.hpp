#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include "nav_sphere.hpp"
#include "acs_input.hpp"
#include "mavlink_local.hpp"

#define NAVIGATOR_SEGMENT_LEN     3

namespace control {

/**
 * @brief   Status of operation returned by ACS
 */
enum class NavigatorStatus {
  loiter,
  navigate,
  wp_reached,
  error
};

/**
 *
 */
class Navigator {
public:
  Navigator(void);
  NavigatorStatus update(ACSInput &acs_in);
  void start(void);
  void stop(void);
  void start_loiter(void);
  void load_segment(const mavlink_mission_item_t *segment);
private:
  bool ready = false;
  bool crsline_switch_ecef(void);
  bool crsline_switch_sphere(void);
  NavSphere<float> sphere;
  mavlink_mission_item_t segment[NAVIGATOR_SEGMENT_LEN];
};

}/* namespace */

#endif /* NAVIGATOR_HPP_ */
