#include <string.h>
#include "hal.h"

/**
 * @brief Thread "safe" variant of memcpy function.
 *
 * @param[in] dest  destination pointer
 * @param[in] src   source pointer
 * @param[in] len   size of transaction
 * @param[in] try   number of trys, minimum 1
 *
 * @return  status of operation
 */
bool memcpy_try(void *dest, const void *src, size_t len, uint32_t retry){
  do{
    memcpy(dest, src, len);
  }while ((0 != memcmp(dest, src, len)) && ((retry--) > 0));

  if (retry > 0)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

