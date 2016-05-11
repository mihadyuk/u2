#ifndef PMU_HPP_
#define PMU_HPP_

#include "baro_data.hpp"

void PMUGet(const baro_abs_data_t &abs,
            const baro_diff_data_t &diff,
            baro_data_t &result);

#endif /* PMU_HPP_ */
