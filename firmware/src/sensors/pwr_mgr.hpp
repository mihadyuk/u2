#ifndef PWR_MGR_H_
#define PWR_MGR_H_

/**
 *
 */
enum class main_battery_state {
  GOOD,
  LOW,
  CRITICAL
};

void PwrMgrInit(void);
main_battery_state PwrMgrUpdate(void);
main_battery_state PwrMgrMainBatteryStartCheck(void);
bool PwrMgr6vGood(void);

#endif /* PWR_MGR_H_ */
