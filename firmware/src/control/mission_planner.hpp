#ifndef MISSION_PLANNER_H_
#define MISSION_PLANNER_H_

class MissionPlanner : public BaseStaticThread<768>{
public:
  MissionPlanner(EepromFile *p);
  msg_t main(void);

private:
  EepromFile *wpdb_file;
  msg_t main_impl(void);
};

#endif /* MISSION_PLANNER_H_ */
