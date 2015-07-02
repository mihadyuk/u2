#ifndef MISSION_RECEIVER_HPP_
#define MISSION_RECEIVER_HPP_

class MissionReceiver : public chibios_rt::BaseStaticThread<640> {
public:
  void main(void);
  void stop(void);
};

#endif /* MISSION_RECEIVER_HPP_ */
