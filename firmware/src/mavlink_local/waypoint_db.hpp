#ifndef WAYPOINT_DB_HPP_
#define WAYPOINT_DB_HPP_

#include "nvram_local.hpp"

#define HEADER_SIZE         (sizeof(uint16_t))
#define WAYPOINT_FOOTPRINT  (sizeof(mavlink_mission_item_t) + 1) /* waypoint + crc8 */

/*
 * TODO:
 *
 * База данных делится на 2 банка, которые переключаются в случае загрузки нового маршрута
 * Навигатор работает только с одной половиной
 * Старший бит заголовка означает нормер банка (0, 1)
 * Остальные биты заголовка содержат количество точек активного банка
 */

/*
 * DB structure:
 *
 * uint16_t                 waypoint count currently stored in DB
 * mavlink_mission_item_t[] array of fixed size chunks storing wp data.
 *                          Chunks must be stored subsequently in waypoints' order
 */
class WpDB{
public:
  WpDB(void);
  uint16_t connect(void);
  bool write(const mavlink_mission_item_t *wpp, uint16_t seq);
  bool read(mavlink_mission_item_t *wpp, uint16_t seq);
  bool reset(void);
  uint16_t getCount(void) const;
  uint16_t getCapacity(void) const;
  bool seal(void);

private:
  NvramFile *dbfile = nullptr;
  size_t bank_offset = 0; /* start of bank in bytes */
  uint16_t count = 0;
};

extern WpDB wpdb;

#endif /* WAYPOINT_DB_HPP_ */
