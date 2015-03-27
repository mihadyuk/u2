#include "main.h"

void set_bor() {

  osalSysLock();
  FLASH->OPTKEYR = 0x08192A3B;
  FLASH->OPTKEYR = 0x4C5D6E7F;
  while(FLASH->SR & FLASH_SR_BSY) {;}
  FLASH->OPTCR &= ~FLASH_OPTCR_BOR_LEV;
  FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;
  while(FLASH->SR & FLASH_SR_BSY){;}
  osalSysUnlock();
}

