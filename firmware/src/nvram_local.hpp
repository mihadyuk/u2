#ifndef NVRAM_LOCAL_HPP_
#define NVRAM_LOCAL_HPP_

#include "nvram_fs.hpp"

#define FRAM_I2C_ADDR           0b1010000
#define FRAM_SIZE               (1024 * 32)

/**
 * @brief   This value will be used for create parameter file in case
 *          of totally empty nvram.
 */
#define BOOTSTRAP_PARAM_FILE_SIZE     (1024 * 8)
#define BOOTSTRAP_WPDB_FILE_SIZE      (1024 * 16)

void NvramInit(void);
void NvramTest(void);
nvram::File *NvramTryOpen(const char *name, size_t size);

extern nvram::Fs nvram_fs;

#endif /* NVRAM_LOCAL_HPP_ */
