#ifndef __SI5351_H
#define __SI5351_H

#include "main.h"

/* ══════════════════════════════════════════════════════════
 * SI5351B I2C 驱动 — 1 路 PLL + 8 路时钟输出
 *
 * 硬件: I2C1, PB6(SCL) / PB7(SDA)
 * 参考: Silicon Labs AN619
 * ══════════════════════════════════════════════════════════ */

/* ── I2C 地址 (7 位, 运行时可通过 si5351_set_addr 修改) ── */

/* ── 寄存器地址 ── */
#define SI5351_REG_OE            0x03   /* 输出使能 (bit=1 禁用) */
#define SI5351_REG_PLL_SRC       0x0F   /* PLL 输入源选择 */

#define SI5351_REG_CLK0_CTRL     16     /* CLK0 控制寄存器基地址 (16~23, 每通道 1 字节) */

#define SI5351_REG_PLLA_BASE     26     /* PLLA 反馈参数 (26~33, 8 字节) */

#define SI5351_REG_MS0_BASE      42     /* MultiSynth0 参数 (42~49, 8 字节) */
#define SI5351_REG_MS1_BASE      50     /* MultiSynth1 参数 (50~57) */
#define SI5351_REG_MS2_BASE      58     /* MultiSynth2 参数 (58~65) */
#define SI5351_REG_MS3_BASE      66     /* MultiSynth3 参数 (66~73) */
#define SI5351_REG_MS4_BASE      74     /* MultiSynth4 参数 (74~81) */
#define SI5351_REG_MS5_BASE      82     /* MultiSynth5 参数 (82~89) */
#define SI5351_REG_MS6           90     /* MultiSynth6 整数分频值 (1 字节) */
#define SI5351_REG_MS7           91     /* MultiSynth7 整数分频值 (1 字节) */
#define SI5351_REG_MS67_RDIV     92     /* CLK6/CLK7 R 分频值 */

#define SI5351_REG_PHOFF_BASE    165    /* 相位偏移基地址 (165~170, CLK0~5) */

#define SI5351_REG_PLL_RESET     177    /* PLL 软复位 */

/* ── 物理常量 ── */
#define SI5351_XTAL_HZ           25000000UL
#define SI5351_VCO_MIN_HZ        600000000UL
#define SI5351_VCO_MAX_HZ        900000000UL
#define SI5351_C_DENOM           1048575UL   /* 分数分母最大值 2^20-1 */

/* ── 输入源选择 ── */
#define SI5351_IN_XTAL           0
#define SI5351_IN_CLKIN          1

/* ── 输出驱动强度 ── */
typedef enum {
    SI5351_DRIVE_2MA  = 0,
    SI5351_DRIVE_4MA  = 1,
    SI5351_DRIVE_6MA  = 2,
    SI5351_DRIVE_8MA  = 3
} Si5351Drive;

/* ── R 分频值 (写入 MS 参数 reg+2 的 bits[7:4]) ── */
typedef enum {
    SI5351_RDIV_1   = 0x00,
    SI5351_RDIV_2   = 0x10,
    SI5351_RDIV_4   = 0x20,
    SI5351_RDIV_8   = 0x30,
    SI5351_RDIV_16  = 0x40,
    SI5351_RDIV_32  = 0x50,
    SI5351_RDIV_64  = 0x60,
    SI5351_RDIV_128 = 0x70
} Si5351RDiv;

/* ══════════════════════════════════════════════════════════
 * API
 * ══════════════════════════════════════════════════════════ */

/* 初始化 I2C 句柄并复位芯片至已知状态 */
void si5351_init(I2C_HandleTypeDef *hi2c);

/* 设置 I2C 从机地址 (7 位), 默认 0x60.
 * 若模块 A0 引脚接高电平则地址为 0x62 */
void si5351_set_addr(uint8_t addr);

/* 配置 PLL VCO 频率 (600~900MHz)，返回实际设定频率 Hz */
uint32_t si5351_pll_config(uint32_t freq_hz);

/* 配置一路时钟输出: clk=0~7, freq_hz=目标频率, drive=驱动强度 */
void si5351_clk_config(uint8_t clk, uint32_t freq_hz, Si5351Drive drive);

/* 使能/禁用单路时钟输出 */
void si5351_clk_enable(uint8_t clk);
void si5351_clk_disable(uint8_t clk);

/* 使能/禁用全部 8 路时钟输出 */
void si5351_output_all_on(void);
void si5351_output_all_off(void);

/* 修改已配置通道的驱动强度 */
void si5351_set_drive(uint8_t clk, Si5351Drive drive);

/* 复位 PLL — 修改任一 PLL 或 MS 参数后调用一次 */
void si5351_reset_plls(void);

/* 读取 PLL 锁定状态: bit0=1 表示 PLLA 已锁定 */
uint8_t si5351_pll_lock_status(void);

#endif /* __SI5351_H */
