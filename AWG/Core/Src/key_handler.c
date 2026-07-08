#include "key_handler.h"
#include "multi_button.h"
#include "waveform.h"

/* ── MultiButton 库配置覆写 ── */
/* 按键按下为低电平（内部上拉，按下接地） */
#define BTN_ACTIVE_LEVEL  0

/* 长按连发节流：BTN_LONG_PRESS_HOLD 每 5ms 触发一次，
   太快无法操作，每 200ms (=40 tick) 执行一次动作 */
#define HOLD_THROTTLE     40

/* ── 按键实例 ── */
static Button btn_duty;  /* K1 */
static Button btn_freq;  /* K2 */
static Button btn_wave;  /* K3 */

/* ── 长按节流计数器 ── */
static uint8_t duty_throttle;
static uint8_t freq_throttle;

/* ── GPIO 读取回调 ── */
static uint8_t read_gpio_level(uint8_t button_id)
{
    switch (button_id) {
    case BTN_ID_DUTY: return (GPIOE->IDR & GPIO_PIN_9) ? 1 : 0;
    case BTN_ID_FREQ: return (GPIOE->IDR & GPIO_PIN_7) ? 1 : 0;
    case BTN_ID_WAVE: return (GPIOB->IDR & GPIO_PIN_0) ? 1 : 0;
    default:          return 0;
    }
}

/* ── K3 (波形切换) 回调: 短按循环切换 ── */
static void cb_wave_single_click(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    WaveType t = WaveGen_GetType();
    t = (WaveType)((t + 1) % 4); /* SINE→TRI→SAW→SQUARE→... */
    WaveGen_Start(t, WaveGen_GetFrequency());
}

/* K3 双击: 峰峰值增加 0.1V */
static void cb_wave_double_click(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    WaveGen_AdjustVpp(+VPP_STEP);
}

/* ── K2 (频率调节) 回调 ── */
static void cb_freq_single_click(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    int f = (int)WaveGen_GetFrequency() + 10;
    if (f > 2000) f = 2000;
    WaveGen_SetFrequency((uint16_t)f);
}

static void cb_freq_long_hold(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    if (++freq_throttle < HOLD_THROTTLE) return;
    freq_throttle = 0;
    int f = (int)WaveGen_GetFrequency() - 10;
    if (f < 500) f = 500;
    WaveGen_SetFrequency((uint16_t)f);
}

/* K2 双击: 峰峰值减小 0.1V */
static void cb_freq_double_click(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    WaveGen_AdjustVpp(-VPP_STEP);
}


/* ── K1 (占空比调节，仅方波) 回调 ── */
static void cb_duty_single_click(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    if (WaveGen_GetType() != WAVE_SQUARE) return;
    int d = (int)WaveGen_GetDuty() + 5;
    if (d > 100) d = 100;
    WaveGen_SetDuty((uint8_t)d);
}

static void cb_duty_long_hold(Button* btn, void* user_data)
{
    (void)btn;
    (void)user_data;
    if (WaveGen_GetType() != WAVE_SQUARE) return;
    if (++duty_throttle < HOLD_THROTTLE) return;
    duty_throttle = 0;
    int d = (int)WaveGen_GetDuty() - 5;
    if (d < 0) d = 0;
    WaveGen_SetDuty((uint8_t)d);
}

/* ── 初始化 ── */
void Key_Init(void)
{
    /* 按键 GPIO：在 gpio.c 的 USER CODE 中已覆写为 INPUT+上拉 */

    /* K1: 占空比 */
    button_init(&btn_duty, read_gpio_level, BTN_ACTIVE_LEVEL, BTN_ID_DUTY);
    button_attach(&btn_duty, BTN_SINGLE_CLICK,   cb_duty_single_click, NULL);
    button_attach(&btn_duty, BTN_LONG_PRESS_HOLD, cb_duty_long_hold,    NULL);
    button_start(&btn_duty);

    /* K2: 频率 + 双击减峰峰值 */
    button_init(&btn_freq, read_gpio_level, BTN_ACTIVE_LEVEL, BTN_ID_FREQ);
    button_attach(&btn_freq, BTN_SINGLE_CLICK,   cb_freq_single_click, NULL);
    button_attach(&btn_freq, BTN_LONG_PRESS_HOLD, cb_freq_long_hold,    NULL);
    button_attach(&btn_freq, BTN_DOUBLE_CLICK,   cb_freq_double_click, NULL);
    button_start(&btn_freq);

    /* K3: 波形切换 + 双击增峰峰值 */
    button_init(&btn_wave, read_gpio_level, BTN_ACTIVE_LEVEL, BTN_ID_WAVE);
    button_attach(&btn_wave, BTN_SINGLE_CLICK, cb_wave_single_click, NULL);
    button_attach(&btn_wave, BTN_DOUBLE_CLICK, cb_wave_double_click, NULL);
    button_start(&btn_wave);
}

void Key_Ticks(void)
{
    button_ticks();
}
