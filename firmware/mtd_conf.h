#ifndef MTD_CONF_H_
#define MTD_CONF_H_

#include "pads.h"

#define MTD_USE_MUTUAL_EXCLUSION            TRUE
#define MTD_WRITE_BUF_SIZE                  (64 + 2)

#define NVRAM_FS_MAX_FILE_NAME_LEN          8
#define NVRAM_FS_MAX_FILE_CNT               3

#define NVRAM_FS_USE_DELETE_AND_RESIZE      FALSE

#endif /* MTD_CONF_H_ */
