#include "main.h"

#include "nvram_local.hpp"
#include "fram_mtd.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

static const FramConfig fram_cfg = {
  FRAM_SIZE
};

static const MtdConfig mtd_cfg = {
  &FRAM_I2CD,
  FRAM_I2C_ADDR,
};

static FramMtd nvram_mtd(&mtd_cfg, &fram_cfg);

NvramFs nvram_fs(nvram_mtd);

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

void NvramInit(void){

  if (OSAL_SUCCESS != nvram_fs.mount()){
    nvram_fs.mkfs();
    if (OSAL_SUCCESS != nvram_fs.mount()){
      osalSysHalt("Storage broken");
    }
  }
}

/**
 * @brief     Try to open file.
 * @details   Functions creates file if it does not exists.
 *
 * @param[in] name    file name.
 * @param[in] size    size of file to be created if it does not exists.
 *
 * @return            pointer to file.
 */
NvramFile *NvramTryOpen(const char *name, size_t size) {

  /* try to open file */
   NvramFile *file = nvram_fs.open(name);

  if (nullptr == file) {
    /* boot strapping when first run */
    if (nvram_fs.df() < size)
      osalSysHalt("Not enough free space in nvram to create file");
    else{
      file = nvram_fs.create(name, size);
      osalDbgCheck(nullptr != file);
    }
  }

  return file;
}


