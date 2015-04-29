#include "main.h"

#include "nvram_local.hpp"
#include "mtd_fm24.hpp"
#include "bus_i2c.hpp"

using namespace chibios_rt;
using namespace nvram;

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

static const FM24Config fram_cfg = {
  FRAM_SIZE
};

static BusI2C fram_bus(&FRAM_I2CD, FRAM_I2C_ADDR);

static MtdFM24 nvram_mtd(fram_bus, &fram_cfg);

Fs nvram_fs(nvram_mtd);

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
/**
 *
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
File *NvramTryOpen(const char *name, size_t size) {

  /* try to open file */
   File *file = nvram_fs.open(name);

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

