#ifndef WAYPOINT_DB_HPP_
#define WAYPOINT_DB_HPP_

#include "nvram_local.hpp"

/**
 *
 */
class WpDB{
public:
  WpDB(void);
  uint16_t start(void);
  void stop(void);
  bool write(const mavlink_mission_item_t *wpp, uint16_t seq);
  bool read(mavlink_mission_item_t *wpp, uint16_t seq);
  bool reset(void);
  uint16_t getCount(void) const;
  uint16_t getCapacity(void) const;
  bool seal(void);

private:
  size_t calc_offset(uint16_t seq, size_t bank);
  nvram::File *dbfile = nullptr;
  size_t active_bank = 0;
  uint16_t shadow_count = 0; // number tracker for shadow bank loading procedure
  uint16_t capacity = 0;    // single bank capacity
  uint16_t count = 0;      // number of waypoints in active bank
};

extern WpDB wpdb;

#endif /* WAYPOINT_DB_HPP_ */
