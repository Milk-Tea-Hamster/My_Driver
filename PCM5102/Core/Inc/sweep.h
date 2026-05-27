#ifndef __SWEEP_H__
#define __SWEEP_H__

#include "main.h"

/* 扫频状态标志 (main 循环可轮询) */
extern volatile uint8_t  g_sweep_cycle_done;
extern volatile uint16_t g_sweep_current_freq;

void Sweep_Init(void);
void Sweep_Start(void);

#endif /* __SWEEP_H__ */
