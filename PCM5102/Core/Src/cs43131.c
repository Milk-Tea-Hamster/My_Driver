#include "cs43131.h"
#include "i2c.h"
#include <stdio.h>
#include "stdarg.h"
#include "usart.h"
/* ============================================================================
 * CS43131 寄存器地址 (24-bit 地址空间)
 * ============================================================================ */

/* ── 系统/ID (0x01xxxx) ── */
#define REG_DEVID_BASE          0x010000U
#define REG_MCLK_SRC            0x010006U
#define REG_SAMPLE_RATE         0x01000BU
#define REG_SAMPLE_BITS         0x01000CU
#define REG_ASP_CTL             0x01000DU
#define REG_POP_FREE            0x010010U

/* ── 电源控制 (0x02xxxx) ── */
#define REG_PWR_CTL             0x020000U
#define REG_XTAL_DRV            0x020052U

/* ── 时钟分频 (0x04xxxx) ── */
#define REG_ASP_MASTER_CFG      0x040018U
#define REG_ASP_LRCK_CFG        0x040019U

/* ── ASP Rx 通道 (0x05xxxx) ── */
#define REG_ASP_RX_CH1_LOC      0x050000U
#define REG_ASP_RX_CH2_LOC      0x050001U
#define REG_ASP_RX_CH1_CTL      0x05000AU
#define REG_ASP_RX_CH2_CTL      0x05000BU

/* ── 耳机输出 (0x08xxxx) ── */
#define REG_HP_OUT_CTL1         0x080000U
#define REG_HP_POP_FREE         0x080032U

/* ── PCM 通路 (0x09xxxx) ── */
#define REG_PCM_FILTER          0x090000U
#define REG_PCM_VOL_A           0x090001U
#define REG_PCM_VOL_B           0x090002U
#define REG_PCM_MUTE_CTL        0x090003U
#define REG_PCM_SIG_CTL2        0x090004U

/* ── HP 电压模式 (0x0Bxxxx) ── */
#define REG_HP_VOLT_MODE        0x0B0000U

/* ============================================================================
 * 私有: I2C 底层读写 (32-bit 寄存器地址 + 0x01 命令字节 + 1 字节数据)
 *
 * 写: [Addr23:16][Addr15:8][Addr7:0][0x01][Data] — 5 字节
 * 读: [Addr23:16][Addr15:8][Addr7:0][0x01] — 先发 4 字节地址，再读 1 字节
 * ============================================================================ */

static uint8_t g_cs43131_addr = 0;

static uint8_t I2C_WriteReg(uint32_t addr, uint8_t val)
{
    uint8_t buf[5];
    buf[0] = (uint8_t)((addr >> 16) & 0xFFU);
    buf[1] = (uint8_t)((addr >> 8) & 0xFFU);
    buf[2] = (uint8_t)(addr & 0xFFU);
    buf[3] = 0x01U;
    buf[4] = val;

    if (HAL_I2C_Master_Transmit(&CS43131_I2C_HANDLE,
                                 g_cs43131_addr << 1,
                                 buf, 5,
                                 CS43131_I2C_TIMEOUT) != HAL_OK)
        return 1;
    return 0;
}

static uint8_t I2C_ReadReg(uint32_t addr, uint8_t *val)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)((addr >> 16) & 0xFFU);
    buf[1] = (uint8_t)((addr >> 8) & 0xFFU);
    buf[2] = (uint8_t)(addr & 0xFFU);
    buf[3] = 0x01U;

    if (HAL_I2C_Master_Transmit(&CS43131_I2C_HANDLE,
                                 g_cs43131_addr << 1,
                                 buf, 4,
                                 CS43131_I2C_TIMEOUT) != HAL_OK)
        return 1;
    if (HAL_I2C_Master_Receive(&CS43131_I2C_HANDLE,
                                g_cs43131_addr << 1,
                                val, 1,
                                CS43131_I2C_TIMEOUT) != HAL_OK)
        return 1;
    return 0;
}

/* ============================================================================
 * 私有: 电源域控制 (读-改-写)
 * ============================================================================ */

static void PwrOn(uint8_t domain_mask)
{
    uint8_t tmp = 0xFFU;
    I2C_ReadReg(REG_PWR_CTL, &tmp);
    tmp &= ~domain_mask;
    I2C_WriteReg(REG_PWR_CTL, tmp);
}

static void PwrOff(uint8_t domain_mask)
{
    uint8_t tmp = 0xFFU;
    I2C_ReadReg(REG_PWR_CTL, &tmp);
    tmp |= domain_mask;
    I2C_WriteReg(REG_PWR_CTL, tmp);
}

/* ============================================================================
 * 私有: I2C 地址扫描
 * ============================================================================ */

static uint8_t ScanAddress(void)
{
    uint8_t first_device  = 0;
    uint8_t cs43131_found = 0;
    uint8_t device_count  = 0;

    UART2_Printf("[CS43131] 开始扫描 I2C 总线...\r\n");

    for (uint16_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&CS43131_I2C_HANDLE,
                                   (uint8_t)(addr << 1), 2,
                                   CS43131_I2C_TIMEOUT) == HAL_OK) {
            device_count++;
            UART2_Printf("[CS43131]   发现设备: 0x%02X\r\n", (uint8_t)addr);

            if (first_device == 0) {
                first_device = (uint8_t)addr;
            }
            if (addr >= 0x60 && addr <= 0x63) {
                cs43131_found = (uint8_t)addr;
            }
        }
    }

    if (device_count == 0) {
        UART2_Printf("[CS43131]   未发现任何设备\r\n");
        return 0;
    }

    UART2_Printf("[CS43131]   共发现 %d 个设备\r\n", device_count);

    /* 优先用标准地址 (0x60-0x63), 否则用总线上第一个设备 */
    if (cs43131_found != 0) {
        return cs43131_found;
    }
    return first_device;
}

/* ============================================================================
 * 公开 API
 * ============================================================================ */

/******************************************************************************
 * @brief   完整初始化 CS43131。
 *          上电序列: RST → I2C验证 → XTAL → ASP → PCM → HP → pop-free → 解除静音。
 *          MCLK = XTAL 直通 (24.576MHz), I2S = Slave 模式。
 *          默认: 96kHz, 16-bit, 快滚降滤波器, 初始音量 -12dB。
 * @return  器件 ID 首字节, 0 表示 I2C 通信失败。
 */
uint8_t CS43131_Init(void)
{
    uint8_t id = 0;

    /* ── 1. 硬件复位 ── */
#ifdef CS43131_RST_PIN
    HAL_GPIO_WritePin(CS43131_RST_PORT, CS43131_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(CS43131_RST_PORT, CS43131_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
#endif

    /* ── 2. 扫描 I2C 地址 ── */
    g_cs43131_addr = ScanAddress();
    if (g_cs43131_addr == 0) {
        UART2_Printf("[CS43131] 设备未找到, 跳过 DAC 初始化\r\n");
        return 0;
    }

    /* ── 3. 验证 I2C 通信: 读取器件 ID ── */
    if (I2C_ReadReg(REG_DEVID_BASE, &id) != 0) {
        UART2_Printf("[CS43131] 寄存器读取超时, 跳过 DAC 初始化\r\n");
        return 0;
    }
    UART2_Printf("[CS43131] DevID[0]=0x%02X\r\n", id);

    /* ── 4. 晶振配置 ── */
    I2C_WriteReg(REG_XTAL_DRV, 0x02U);

    /* MCLK_SRC = 0x02: RCO 模式, 声明 MCLK_INT=24.576MHz */
    I2C_WriteReg(REG_MCLK_SRC, 0x02U);

    PwrOn(CS43131_PWR_XTAL);
    HAL_Delay(2);

    /* MCLK_SRC = 0x00: 切至 XTAL 直通, 内部 MCLK=24.576MHz */
    I2C_WriteReg(REG_MCLK_SRC, 0x00U);
    HAL_Delay(1);

    /* ── 5. ASP (I2S) 接口配置 ── */
    CS43131_SetSampleRate(CS43131_SR_96K);
    CS43131_SetBitDepth(CS43131_BITS_16);

    /* ASP 主从 + SCLK 极性: Slave 模式, SCLK 反相 */
    I2C_WriteReg(REG_ASP_MASTER_CFG, 0x04U);

    /* ASP LRCK: 50/50 占空比, 帧起始延迟=1 */
    I2C_WriteReg(REG_ASP_LRCK_CFG, 0x0AU);

    /* ASP Rx 通道起始位置 (SCLK 第 0 拍) */
    I2C_WriteReg(REG_ASP_RX_CH1_LOC, 0x00U);
    I2C_WriteReg(REG_ASP_RX_CH2_LOC, 0x00U);

    /* ASP Rx 通道: 32-bit slot, 数据传播至内部通路 */
    I2C_WriteReg(REG_ASP_RX_CH1_CTL, 0x07U);
    I2C_WriteReg(REG_ASP_RX_CH2_CTL, 0x0FU);

    /* ASP 时钟控制: 非主模式, 串口时钟持续有效 */
    I2C_WriteReg(REG_ASP_CTL, 0x00U);

    /* ── 6. PCM 通路配置 ── */
    CS43131_SetFilter(0);
    CS43131_SetVolume(0x20);                 /* 初始音量 -12dB */

    /* 静音控制: SZC 软斜坡 | 双声道联动 | 解除静音 */
    I2C_WriteReg(REG_PCM_MUTE_CTL, 0x0FU);

    /* 极性/复制: 正常左右声道 */
    I2C_WriteReg(REG_PCM_SIG_CTL2, 0x00U);

    /* ── 7. ASP 上电 ── */
    PwrOn(CS43131_PWR_ASP);

    /* ── 8. 耳机放大器配置 ── */
    I2C_WriteReg(REG_HP_OUT_CTL1, 0x10U);    /* 满偏输出 1Vrms */
    I2C_WriteReg(REG_HP_VOLT_MODE, 0x00U);   /* 高压模式 0 */

    /* Pop-free 序列 */
    I2C_WriteReg(REG_POP_FREE,    0x99U);
    I2C_WriteReg(REG_HP_POP_FREE, 0x20U);

    PwrOn(CS43131_PWR_HP);
    HAL_Delay(2);

    I2C_WriteReg(REG_POP_FREE,    0x00U);
    I2C_WriteReg(REG_HP_POP_FREE, 0x00U);

    /* ── 9. 最终解除静音 ── */
    CS43131_Mute(0);

  UART2_Printf("[CS43131] 初始化完成 (96kHz/16bit/Slave/快滚降)\r\n");

    return id;
}

void CS43131_SetVolume(uint8_t vol)
{
    I2C_WriteReg(REG_PCM_VOL_A, vol);
    I2C_WriteReg(REG_PCM_VOL_B, vol);
}

void CS43131_Mute(uint8_t mute)
{
    uint8_t tmp = 0;
    I2C_ReadReg(REG_PCM_MUTE_CTL, &tmp);
    if (mute) {
        tmp |= 0x03U;
    } else {
        tmp &= ~0x03U;
    }
    I2C_WriteReg(REG_PCM_MUTE_CTL, tmp);
}

void CS43131_PowerDown(uint8_t pdn)
{
    if (pdn) {
        I2C_WriteReg(REG_PWR_CTL, 0xFEU);
    } else {
        PwrOn(CS43131_PWR_XTAL | CS43131_PWR_PLL | CS43131_PWR_ASP | CS43131_PWR_HP);
    }
}

void CS43131_SetSampleRate(CS43131_SampleRate sr)
{
    I2C_WriteReg(REG_SAMPLE_RATE, (uint8_t)sr);
}

void CS43131_SetBitDepth(CS43131_BitDepth bits)
{
    I2C_WriteReg(REG_SAMPLE_BITS, (uint8_t)bits);
}

void CS43131_SetFilter(uint8_t mode)
{
    I2C_WriteReg(REG_PCM_FILTER, mode);
}
