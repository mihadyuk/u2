#include "main.h"
#include "pads.h"

#include "nvram_local.hpp"
#include "mtd24.hpp"
#include "nvram_test.hpp"

using namespace chibios_rt;
using namespace nvram;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MTD_WRITE_BUF_SIZE                  (64 + 2)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

static void mtd_led_on(Mtd *mtd) {
  (void)mtd;
  red_led_on();
}

static void mtd_led_off(Mtd *mtd) {
  (void)mtd;
  red_led_off();
}

static const MtdConfig fram_cfg = {
    0,
    1,
    FRAM_SIZE,
    2,
    mtd_led_on,
    mtd_led_off,
    nullptr,
    nullptr,
    mtd_led_on,
    mtd_led_off,
};

static uint8_t workbuf[MTD_WRITE_BUF_SIZE];

static Mtd24 nvram_mtd(fram_cfg, workbuf, MTD_WRITE_BUF_SIZE, &FRAM_I2CD, FRAM_I2C_ADDR);

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
void NvramInit(void) {

  nvramTest(nvram_mtd);




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

