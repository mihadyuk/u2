/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * FSMC driver system settings.
 */
#define STM32_FSMC_USE_FSMC1                FALSE
#define STM32_FSMC_FSMC1_IRQ_PRIORITY       10

#define STM32_NAND_USE_FSMC_NAND1           TRUE
#define STM32_NAND_USE_EXT_INT              TRUE
#define STM32_NAND_DMA_STREAM               STM32_DMA_STREAM_ID(2, 7)
#define STM32_NAND_DMA_PRIORITY             0
#define STM32_NAND_DMA_ERROR_HOOK(nandp)    osalSysHalt("DMA failure")

/*
 * FSMC SRAM driver system settings.
 */
#define STM32_USE_FSMC_SRAM                 FALSE
#define STM32_SRAM_USE_FSMC_SRAM4           FALSE

/*
 * EICU driver system settings.
 */
#define STM32_EICU_USE_TIM1                 FALSE
#define STM32_EICU_USE_TIM2                 FALSE
#define STM32_EICU_USE_TIM3                 FALSE
#define STM32_EICU_USE_TIM4                 TRUE
#define STM32_EICU_USE_TIM5                 TRUE
#define STM32_EICU_USE_TIM8                 FALSE
#define STM32_EICU_USE_TIM9                 TRUE
#define STM32_EICU_USE_TIM10                FALSE
#define STM32_EICU_USE_TIM11                TRUE
#define STM32_EICU_USE_TIM12                FALSE
#define STM32_EICU_USE_TIM13                FALSE
#define STM32_EICU_USE_TIM14                FALSE
#define STM32_EICU_TIM1_IRQ_PRIORITY        7
#define STM32_EICU_TIM2_IRQ_PRIORITY        7
#define STM32_EICU_TIM3_IRQ_PRIORITY        7
#define STM32_EICU_TIM4_IRQ_PRIORITY        7
#define STM32_EICU_TIM5_IRQ_PRIORITY        7
#define STM32_EICU_TIM8_IRQ_PRIORITY        7
#define STM32_EICU_TIM9_IRQ_PRIORITY        7
#define STM32_EICU_TIM10_IRQ_PRIORITY       7
#define STM32_EICU_TIM11_IRQ_PRIORITY       7
#define STM32_EICU_TIM12_IRQ_PRIORITY       7
#define STM32_EICU_TIM13_IRQ_PRIORITY       7
#define STM32_EICU_TIM14_IRQ_PRIORITY       7

