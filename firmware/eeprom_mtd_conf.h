#ifndef EEPROM_MTD_CONF_H_
#define EEPROM_MTD_CONF_H_

#include "pads.h"

#define EEPROM_MTD_USE_MUTUAL_EXCLUSION     TRUE

#define EEPROM_PAGE_SIZE        32          /* page size in bytes. Consult datasheet. */
#define EEPROM_PAGES            1024        /* total amount of memory in pages */
#define EEPROM_WRITE_TIME_MS    20          /* time to write one page in mS. Consult datasheet! */

#endif /* EEPROM_MTD_CONF_H_ */
