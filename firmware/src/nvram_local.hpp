#ifndef NVRAM_LOCAL_HPP_
#define NVRAM_LOCAL_HPP_

#include "nvram_fs.hpp"

/**
 * @brief   This value will be used for create parameter file in case
 *          of totally empty nvram.
 */
#define BOOTSTRAP_PARAM_FILE_SIZE     (1024 * 6)

void NvramInit(void);

extern NvramFs nvram_fs;

#endif /* NVRAM_LOCAL_HPP_ */
