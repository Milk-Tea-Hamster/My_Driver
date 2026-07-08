#include "si5351.h"

/* ══════════════════════════════════════════════════════════
 * SI5351B 驱动实现
 *
 * SI5351B 只有 1 个 PLL (PLLA)，8 路时钟输出 (CLK0~CLK7)
 *
 * 寄存器共享约束:
 *   PLLA 反馈分频器寄存器 26~33 仅用于 PLL 反馈,
 *   CLK0~CLK5 的 MultiSynth 参数在独立寄存器区域 (42~89),
 *   CLK6/CLK7 仅支持整数分频 (寄存器 90/91)
 * ══════════════════════════════════════════════════════════ */

static I2C_HandleTypeDef *_hi2c = NULL;
static uint8_t _oe_shadow    = 0xFF;          /* 输出使能影子寄存器 */
static uint32_t _pll_freq    = 800000000UL;   /* PLL 当前 VCO 频率 */
static uint8_t _input_src    = SI5351_IN_XTAL; /* 当前输入源 */
static uint8_t _i2c_addr     = 0x60;          /* I2C 从机地址 (7 位), 默认 0x60 */

/* ── MS 参数基地址查表 ── */
static const uint8_t _ms_base[8] = {
    SI5351_REG_MS0_BASE,  /* CLK0 = 42 */
    SI5351_REG_MS1_BASE,  /* CLK1 = 50 */
    SI5351_REG_MS2_BASE,  /* CLK2 = 58 */
    SI5351_REG_MS3_BASE,  /* CLK3 = 66 */
    SI5351_REG_MS4_BASE,  /* CLK4 = 74 */
    SI5351_REG_MS5_BASE,  /* CLK5 = 82 */
    0,                     /* CLK6 = 特殊处理 (reg 90, 1 字节) */
    0                      /* CLK7 = 特殊处理 (reg 91, 1 字节) */
};

/* ══════════════════════════════════════════════════════════
 * I2C 基础操作
 * ══════════════════════════════════════════════════════════ */

static uint8_t _rd(uint8_t reg) {
    uint8_t v = 0;
    HAL_I2C_Mem_Read(_hi2c, _i2c_addr << 1, reg,
                     I2C_MEMADD_SIZE_8BIT, &v, 1, HAL_MAX_DELAY);
    return v;
}

static void _wr(uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(_hi2c, _i2c_addr << 1, reg,
                      I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

/* ══════════════════════════════════════════════════════════
 * 分数化简 (P1/P2/P3 参数计算)
 * ══════════════════════════════════════════════════════════ */

/* 计算 f_vco / f_out = a + b/c 的 P1/P2/P3 参数 */
static void _calc_params(uint32_t f_vco, uint32_t f_out,
                         uint32_t *p1, uint32_t *p2, uint32_t *p3) {
    float ratio = (float)f_vco / (float)f_out;
    uint32_t a = (uint32_t)ratio;
    uint32_t b, c;

    if (ratio - (float)a < 1e-6f) {
        /* 整数分频 */
        b = 0;
        c = 1;
    } else {
        c = SI5351_C_DENOM;
        b = (uint32_t)((ratio - (float)a) * (float)c + 0.5f);
    }

    float frac = (c > 0) ? ((float)b / (float)c) : 0.0f;
    uint32_t flr = (uint32_t)(128.0f * frac);

    *p1 = 128UL * a + flr - 512UL;
    *p2 = 128UL * b - c * flr;
    *p3 = c;
}

/* ══════════════════════════════════════════════════════════
 * PLL 参数写入 (8 字节, AN619 格式)
 *
 * reg+0 (26): P3[15:8]
 * reg+1 (27): P3[7:0]
 * reg+2 (28): [1:0] = P1[17:16]
 * reg+3 (29): P1[15:8]
 * reg+4 (30): P1[7:0]
 * reg+5 (31): [7:4] = P3[19:16], [3:0] = P2[19:16]
 * reg+6 (32): P2[15:8]
 * reg+7 (33): P2[7:0]
 * ══════════════════════════════════════════════════════════ */
static void _write_pll_params(uint32_t p1, uint32_t p2, uint32_t p3) {
    uint8_t base = SI5351_REG_PLLA_BASE;

    _wr(base + 0, (p3 >> 8) & 0xFF);
    _wr(base + 1, p3 & 0xFF);
    _wr(base + 2, (p1 >> 16) & 0x03);
    _wr(base + 3, (p1 >> 8) & 0xFF);
    _wr(base + 4, p1 & 0xFF);
    _wr(base + 5, ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
    _wr(base + 6, (p2 >> 8) & 0xFF);
    _wr(base + 7, p2 & 0xFF);
}

/* ══════════════════════════════════════════════════════════
 * MultiSynth 参数写入 (8 字节, AN619 格式)
 *
 * ms_base+0: P3[15:8]
 * ms_base+1: P3[7:0]
 * ms_base+2: [7:4]=R_DIV, [1:0]=P1[17:16]
 * ms_base+3: P1[15:8]
 * ms_base+4: P1[7:0]
 * ms_base+5: [7:4]=P3[19:16], [3:0]=P2[19:16]
 * ms_base+6: P2[15:8]
 * ms_base+7: P2[7:0]
 * ══════════════════════════════════════════════════════════ */
static void _write_ms_params(uint8_t clk, uint32_t p1, uint32_t p2, uint32_t p3,
                             uint8_t rdiv_code) {
    uint8_t base = _ms_base[clk];

    _wr(base + 0, (p3 >> 8) & 0xFF);
    _wr(base + 1, p3 & 0xFF);
    _wr(base + 2, rdiv_code | ((p1 >> 16) & 0x03));
    _wr(base + 3, (p1 >> 8) & 0xFF);
    _wr(base + 4, p1 & 0xFF);
    _wr(base + 5, ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
    _wr(base + 6, (p2 >> 8) & 0xFF);
    _wr(base + 7, p2 & 0xFF);
}

/* ══════════════════════════════════════════════════════════
 * CLK 控制寄存器写入 (1 字节)
 *
 * Bit 7: CLKx_PDN   (0 = 上电)
 * Bit 6: MSx_INT    (0 = 分数模式, 1 = 整数模式)
 * Bit 5: MSx_SRC    (SI5351B 始终为 0 = PLLA)
 * Bit 4:3: CLKx_IDRV (驱动电流)
 * Bit 2: CLKx_INV   (0 = 正常, 1 = 反相)
 * Bit 1:0: CLKx_SRC (11 = PLLx)
 * ══════════════════════════════════════════════════════════ */
static void _write_clk_ctrl(uint8_t clk, uint8_t is_integer, Si5351Drive drive) {
    /* 与参考驱动完全一致的 CLK 控制寄存器值:
     * 0x0F = PDN=0 | IDRV=01(4mA) | INV=1 | SRC=11(PLL)
     * + 0x40 = MSx_INT (整数模式) */
    uint8_t val = 0x0F | (is_integer ? 0x40 : 0x00);

    _wr(SI5351_REG_CLK0_CTRL + clk, val);
    (void)drive; /* 驱动强度固定 4mA, 与参考驱动一致 */
}

/* ══════════════════════════════════════════════════════════
 * CLK6/CLK7 配置 (仅整数分频)
 *
 * CLK6: 寄存器 90 写入整数分频值 (≤127, 可配合 R 分频扩展)
 * CLK7: 寄存器 91
 * CLK6/CLK7 R 分频: 寄存器 92
 *   bits[3:0] = CLK6 R-div, bits[7:4] = CLK7 R-div
 * ══════════════════════════════════════════════════════════ */
static void _write_clk67(uint8_t clk, uint32_t freq_hz, Si5351Drive drive) {
    uint32_t div_val = _pll_freq / freq_hz;
    uint8_t  rdiv    = 0;

    /* CLK6 分频值 7 位 (max 127), CLK7 分频值 8 位 (max 255) */
    uint8_t max_div = (clk == 6) ? 127 : 255;

    /* 通过 R 分频将 divider 降到寄存器位宽以内 */
    while (div_val > max_div && rdiv < 7) {
        div_val >>= 1;
        rdiv++;
    }
    if (div_val > max_div) div_val = max_div;
    if (div_val < 1)       div_val = 1;

    /* 写整数分频值 */
    if (clk == 6) {
        _wr(SI5351_REG_MS6, (uint8_t)div_val);

        /* 更新寄存器 92: CLK6 R-div 在低 3 位 */
        uint8_t r67 = _rd(SI5351_REG_MS67_RDIV);
        r67 = (r67 & 0xF8) | (rdiv & 0x07);
        _wr(SI5351_REG_MS67_RDIV, r67);
    } else {
        _wr(SI5351_REG_MS7, (uint8_t)div_val);

        /* 更新寄存器 92: CLK7 R-div 在 bits[6:4] */
        uint8_t r67 = _rd(SI5351_REG_MS67_RDIV);
        r67 = (r67 & 0x8F) | ((rdiv & 0x07) << 4);
        _wr(SI5351_REG_MS67_RDIV, r67);
    }

    /* 写入 CLK 控制寄存器 */
    _write_clk_ctrl(clk, 1, drive);
}

/* ══════════════════════════════════════════════════════════
 * 初始化
 * ══════════════════════════════════════════════════════════ */

void si5351_init(I2C_HandleTypeDef *hi2c) {
    _hi2c      = hi2c;
    _oe_shadow = 0x00;

    /* 参考驱动不写 reg 0x03, 完全由 OEB 硬件引脚控制输出使能 */
    _input_src = SI5351_IN_XTAL;
}

void si5351_set_addr(uint8_t addr) {
    _i2c_addr = addr;
}

/* ══════════════════════════════════════════════════════════
 * PLL 配置
 * ══════════════════════════════════════════════════════════ */

uint32_t si5351_pll_config(uint32_t freq_hz) {
    /* 钳位到 VCO 范围 */
    if (freq_hz < SI5351_VCO_MIN_HZ) freq_hz = SI5351_VCO_MIN_HZ;
    if (freq_hz > SI5351_VCO_MAX_HZ) freq_hz = SI5351_VCO_MAX_HZ;

    /* PLL 复位 (AN619: 向 reg 177 写 0xA0) */
    _wr(SI5351_REG_PLL_RESET, 0xA0);

    /* 配置输入源: XTAL → reg 15 = 0x00 */
    _wr(SI5351_REG_PLL_SRC, 0x00);

    /* 计算 PLL 反馈分频参数: f_vco / 25MHz = a + b/c */
    uint32_t p1, p2, p3;
    _calc_params(freq_hz, SI5351_XTAL_HZ, &p1, &p2, &p3);

    /* 写入 PLL 反馈参数 (8 字节, AN619 格式) */
    _write_pll_params(p1, p2, p3);

    _pll_freq = freq_hz;
    return freq_hz;
}

/* ══════════════════════════════════════════════════════════
 * 时钟输出配置
 * ══════════════════════════════════════════════════════════ */

void si5351_clk_config(uint8_t clk, uint32_t freq_hz, Si5351Drive drive) {
    if (clk > 7) return;

    /* CLK6/CLK7 仅支持整数分频, 单独处理 */
    if (clk >= 6) {
        _write_clk67(clk, freq_hz, drive);
        return;
    }

    /* ── CLK0~CLK5: 分数/整数 MultiSynth ── */
    uint32_t f_vco = _pll_freq;

    /* 计算所需 R 分频 */
    float ms_div = (float)f_vco / (float)freq_hz;
    uint8_t  rdiv     = 0;
    uint32_t r_ratio  = 1;

    while (ms_div > 900.0f && rdiv < 7) {
        rdiv++;
        r_ratio <<= 1;
        ms_div = (float)f_vco / (float)(freq_hz * r_ratio);
    }

    /* MultiSynth 分频比范围: 4 ~ 900 */
    if (ms_div < 4.0f)   ms_div = 4.0f;
    if (ms_div > 900.0f) ms_div = 900.0f;

    /* 计算 P1/P2/P3 */
    uint32_t p1, p2, p3;
    uint32_t effective_freq = freq_hz * r_ratio;
    _calc_params(f_vco, effective_freq, &p1, &p2, &p3);

    uint8_t is_integer = (p2 == 0 && p3 == 1) ? 1 : 0;

    /* 写入 MultiSynth 参数 (8 字节, 包含 R 分频值) */
    uint8_t rdiv_code = (uint8_t)(rdiv << 4);  /* bits[7:4] */
    _write_ms_params(clk, p1, p2, p3, rdiv_code);

    /* 写入 CLK 控制寄存器 */
    _write_clk_ctrl(clk, is_integer, drive);

    /* 相位偏移 = 0 */
    _wr(SI5351_REG_PHOFF_BASE + clk, 0x00);
}

/* ══════════════════════════════════════════════════════════
 * 输出使能控制
 * ══════════════════════════════════════════════════════════ */

void si5351_clk_enable(uint8_t clk) {
    if (clk > 7) return;
    _oe_shadow &= ~(1 << clk);
    _wr(SI5351_REG_OE, _oe_shadow);
}

void si5351_clk_disable(uint8_t clk) {
    if (clk > 7) return;
    _oe_shadow |= (1 << clk);
    _wr(SI5351_REG_OE, _oe_shadow);
}

void si5351_output_all_on(void) {
    _oe_shadow = 0x00;
    _wr(SI5351_REG_OE, 0x00);
}

void si5351_output_all_off(void) {
    _oe_shadow = 0xFF;
    _wr(SI5351_REG_OE, 0xFF);
}

/* ══════════════════════════════════════════════════════════
 * 运行时修改
 * ══════════════════════════════════════════════════════════ */

void si5351_set_drive(uint8_t clk, Si5351Drive drive) {
    if (clk > 7) return;
    uint8_t reg = SI5351_REG_CLK0_CTRL + clk;
    uint8_t val = _rd(reg);
    /* CLKx_IDRV 在 bits[4:3] */
    val = (val & 0xE7) | ((drive & 0x03) << 3);
    _wr(reg, val);
}

/* ══════════════════════════════════════════════════════════
 * PLL 复位
 * ══════════════════════════════════════════════════════════ */

void si5351_reset_plls(void) {
    _wr(SI5351_REG_PLL_RESET, 0xA0);
}

/* ══════════════════════════════════════════════════════════
 * 状态查询
 * ══════════════════════════════════════════════════════════ */

uint8_t si5351_pll_lock_status(void) {
    /* 寄存器 0x00 bit0 = PLLA_LOL, 0 = 锁定, 1 = 失锁 */
    uint8_t st = _rd(0x00);
    /* 返回 bit0: 1 = 锁定, 0 = 失锁 (取反) */
    return (~st) & 0x01;
}
