#include "OLED.h"
#include "OLED_Font.h"
#include "delay.h"
#include <string.h>

/* ── 软件 I2C 引脚: PD9=SCL, PD10=SDA ── */
#define OLED_SCL_PIN    GPIO_PIN_9
#define OLED_SDA_PIN    GPIO_PIN_10
#define OLED_GPIO_PORT  GPIOD

#define OLED_SCL(x)  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN, (GPIO_PinState)(x))
#define OLED_SDA(x)  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SDA_PIN, (GPIO_PinState)(x))

/* ── 调试用 ── */
volatile int oled_init_ok = 0;  /* 1 = 初始化完成 */

/* ── 软件 I2C 时序（100kHz 标准模式） ── */
static void I2C_Start(void)
{
    OLED_SDA(1);
    OLED_SCL(1);
    Delay_us(5);
    OLED_SDA(0);
    Delay_us(5);
    OLED_SCL(0);
}

static void I2C_Stop(void)
{
    OLED_SDA(0);
    OLED_SCL(1);
    Delay_us(5);
    OLED_SDA(1);
    Delay_us(5);
}

static void I2C_SendByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_SDA(!!(byte & (0x80 >> i)));
        Delay_us(5);
        OLED_SCL(1);
        Delay_us(5);
        OLED_SCL(0);
        Delay_us(5);
    }
    /* 第 9 个时钟：释放 SDA 读取 ACK（不检查） */
    OLED_SDA(1);
    Delay_us(5);
    OLED_SCL(1);
    Delay_us(5);
    OLED_SCL(0);
    Delay_us(5);
}

/* ── SSD1306 写操作 ── */
static void OLED_WriteCommand(uint8_t cmd)
{
    I2C_Start();
    I2C_SendByte(OLED_ADDR);   /* 0x78 */
    I2C_SendByte(0x00);        /* 控制字节: 命令 */
    I2C_SendByte(cmd);
    I2C_Stop();
}

static void OLED_WriteData(uint8_t data)
{
    I2C_Start();
    I2C_SendByte(OLED_ADDR);
    I2C_SendByte(0x40);        /* 控制字节: 数据 */
    I2C_SendByte(data);
    I2C_Stop();
}

/* 批量写数据: 一个 I2C 事务内连续发送多个数据字节 */
static void OLED_WriteMultiData(const uint8_t *data, uint8_t len)
{
    I2C_Start();
    I2C_SendByte(OLED_ADDR);
    I2C_SendByte(0x40);
    for (uint8_t i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
    }
    I2C_Stop();
}

/* ── 光标定位 ── */
static void OLED_SetCursor(uint8_t page, uint8_t col)
{
    OLED_WriteCommand(0xB0 | page);
    OLED_WriteCommand(0x10 | ((col & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (col & 0x0F));
}

/* ── 清屏 ── */
void OLED_Clear(void)
{
    static const uint8_t zeros[128] = {0};
    for (uint8_t page = 0; page < 8; page++) {
        OLED_SetCursor(page, 0);
        OLED_WriteMultiData(zeros, 128);
    }
}

/* ── 显示单个字符（8x16 字体） ── */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t page = (Line - 1) * 2;
    uint8_t col  = (Column - 1) * 8;

    OLED_SetCursor(page, col);
    OLED_WriteMultiData(&OLED_F8x16[Char - ' '][0], 8);

    OLED_SetCursor(page + 1, col);
    OLED_WriteMultiData(&OLED_F8x16[Char - ' '][8], 8);
}

/* ── 显示字符串 ── */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/* ── 次方函数 ── */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) { Result *= X; }
    return Result;
}

/* ── 显示十进制正数 ── */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i,
                      Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* ── 显示带符号十进制数 ── */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t n = (Number >= 0) ? Number : -Number;
    OLED_ShowChar(Line, Column, Number >= 0 ? '+' : '-');
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i + 1,
                      n / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* ── 显示十六进制数 ── */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++) {
        uint8_t n = Number / OLED_Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(Line, Column + i, n < 10 ? n + '0' : n - 10 + 'A');
    }
}

/* ── 显示二进制数 ── */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i,
                      Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* ── 显示浮点数 ── */
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t Decimals)
{
    if (Number < 0) {
        OLED_ShowChar(Line, Column, '-');
        Column++;
        Number = -Number;
    }

    uint32_t int_part = (uint32_t)Number;
    float    dec_part = Number - int_part;

    uint32_t factor = 1;
    for (uint8_t i = 0; i < Decimals; i++) { factor *= 10; }
    dec_part = dec_part * factor + 0.5f;
    uint32_t dec_digits = (uint32_t)dec_part;

    if (dec_digits >= factor) {
        int_part++;
        dec_digits = 0;
    }

    uint8_t buf[12];
    uint8_t idx = 0;
    if (int_part == 0) {
        buf[idx++] = '0';
    } else {
        uint32_t tmp = int_part;
        while (tmp > 0 && idx < sizeof(buf)) {
            buf[idx++] = '0' + (tmp % 10);
            tmp /= 10;
        }
        for (uint8_t i = 0; i < idx / 2; i++) {
            uint8_t t = buf[i];
            buf[i] = buf[idx - 1 - i];
            buf[idx - 1 - i] = t;
        }
    }
    for (uint8_t i = 0; i < idx; i++) {
        OLED_ShowChar(Line, Column + i, buf[i]);
    }
    Column += idx;

    OLED_ShowChar(Line, Column, '.');
    Column++;

    for (uint8_t i = 0; i < Decimals; i++) {
        uint8_t d = dec_digits / OLED_Pow(10, Decimals - i - 1) % 10;
        OLED_ShowChar(Line, Column + i, '0' + d);
    }
}

/* ── 初始化 PD9/PD10 为开漏输出 ── */
static void OLED_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_GPIO_PORT, &gpio);

    OLED_SCL(1);
    OLED_SDA(1);
}

/* ── OLED 初始化序列（SSD1306） ── */
void OLED_Init(void)
{
    OLED_GPIO_Init();
    Delay_ms(100);

    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5);  OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);  OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);  OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA);  OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);  OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);  OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);  OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D);  OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);

    OLED_Clear();
    oled_init_ok = 1;
}
