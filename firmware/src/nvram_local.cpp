#include "main.h"
#include "pads.h"

#include "nvram_local.hpp"
#include "mtd24.hpp"
#include "mtd25.hpp"
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


//#define S25_PAGE_SIZE     256
//#define S25_SIZE_BYTES    (16*1024*1024)
//
//static const MtdConfig eeprom_cfg = {
//    MS2ST(500),
//    S2ST(330),
//    S25_SIZE_BYTES / S25_PAGE_SIZE,
//    S25_PAGE_SIZE,
//    3,
//    mtd_led_on,
//    mtd_led_off,
//    nullptr,
//    nullptr,
//    mtd_led_on,
//    mtd_led_off,
//};
//
//static const SPIConfig spicfg = {
//  NULL,
//  GPIOB,
//  GPIOB_SPI2_NSS_UEXT,
//  0//SPI_CR1_BR_1
//};

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

/**
 *
 */
static void fram_test(void) {
  //nvramTestSuite(nvram_mtd);
}

/**
 *
 */
static void eeprom_test(void) {
//  Mtd25 eeprom_mtd(eeprom_cfg, workbuf, MTD_WRITE_BUF_SIZE, &UEXT_SPI);
//
//  spiStart(&UEXT_SPI, &spicfg);
//  nvramTestSuite(eeprom_mtd);
//  spiStop(&UEXT_SPI);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void NvramTest(void) {
  fram_test();
  eeprom_test();
}

/**
 *
 */
void NvramInit(void) {
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

