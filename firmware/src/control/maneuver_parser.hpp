#ifndef MANEUVER_PARSER_HPP_
#define MANEUVER_PARSER_HPP_

#include "ld_navigator_types.hpp"
#include "maneuver_list.hpp"
#include "mavlink_local.hpp"

namespace control {

template <typename T>
class ManeuverParser {
public:
  ManeuverParser(const mavlink_mission_item_t &prev,
		 const mavlink_mission_item_t &trgt,
		 const mavlink_mission_item_t &third) :
		   prev(prev), trgt(trgt), third(third), mnrPartNumber(0) {;}

  ManeuverPart<T> update(T (&currWGS84)[3][1]) {
    ManeuverPart<T> ret;
    T prevNE[2][1];
    maneuver::missionItemWGS84toNE<T>(prevNE, currWGS84, prev);
    T trgtNE[2][1];
    maneuver::missionItemWGS84toNE<T>(trgtNE, currWGS84, trgt);
    //T thirdNE[2][1];
    //maneuver::missionItemWGS84toNE<T>(thirdNE, currWGS84, third);

    switch (trgt.command) {
      case MAV_CMD_NAV_WAYPOINT:
        maneuver::updateMnrLine<T>(ret,
                                   prevNE,
                                   trgtNE);
        break;
      case MAV_CMD_NAV_LOITER_TURNS:
        maneuver::updateMnrCircle<T>(ret,
                                     mnrPartNumber,
                                     trgt.param1,   /* turns count */
                                     trgt.param3,   /* turns radius */
                                     prevNE,
                                     trgtNE);
        break;
      default:
        maneuver::updateMnrUnknown<T>(ret);
        break;
    }
    return ret;
  }

  void loadNextPart() {mnrPartNumber++;}

  void resetPartCounter() {mnrPartNumber = 0;}

private:
  const mavlink_mission_item_t &prev;
  const mavlink_mission_item_t &trgt;
  const mavlink_mission_item_t &third;
  size_t mnrPartNumber;
};

} /* namespace control */

#endif /* MANEUVER_PARSER_HPP_ */
