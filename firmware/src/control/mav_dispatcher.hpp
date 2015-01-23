#ifndef MAV_DISPATCHER_H_
#define MAV_DISPATCHER_H_

#include "message.hpp"
#include "acs.hpp"

class MavDispatcher : public BaseStaticThread<512>{
public:
  MavDispatcher(ACS &acsp);
  msg_t main(void);

private:
  mavlink_mission_item_t mi;  // cached mission item to reduce loading delay
  ACS &acs;
};

#endif /* MAV_DISPATCHER_H_ */
