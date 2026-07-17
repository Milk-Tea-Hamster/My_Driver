#include "delay.h"

/* 使用 DWT (Data Watchpoint and Trace) 周期计数器实现微秒延时
   STM32F407 主频 168MHz，每个时钟周期 ≈ 5.95ns，168 周期 ≈ 1μs */

static uint32_t dwt_cycles_per_us;

void Delay_Init(void)
{
    /* 使能 DWT 外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    dwt_cycles_per_us = SystemCoreClock / 1000000U;
}

void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * dwt_cycles_per_us;

    while ((DWT->CYCCNT - start) < ticks) {
        /* 忙等 */
    }
}

void Delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        Delay_us(1000);
    }
}
