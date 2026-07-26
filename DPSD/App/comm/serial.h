#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32h7xx_hal.h"

#define RXBUFFERSIZE      512U//串口环形缓冲区大小

 void Serial_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);

HAL_StatusTypeDef Serial_RxInit(UART_HandleTypeDef *huart);
void Serial_Process(void);

void Serial_OnFrame(UART_HandleTypeDef *huart,
                    const uint8_t *data,
                    uint16_t length);

extern uint8_t uart1_rx_buf[RXBUFFERSIZE];
extern volatile uint16_t uart1_rx_len;
extern volatile uint8_t uart1_rx_flag;

extern uint8_t uart3_rx_buf[RXBUFFERSIZE];
extern volatile uint16_t uart3_rx_len;
extern volatile uint8_t uart3_rx_flag;

void HMI_SendStr(const char *ctl, const char *text);
void HMI_SendInt(const char *ctl, int num);
void HMI_SendFloat(const char *ctl, float num, int decimals);
void HMI_WaveAdd(const char *ctl, int ch, int val);
void HMI_WaveClear(const char *ctl, int ch);


#define HMI_WAVE_WIDTH   300U
#define HMI_WAVE_HEIGHT  200U//串口屏波形控件长与宽

//使用 addt 透明传输，一次性写入多个波形点。
uint8_t HMI_WaveAddBatch(const char *ctl, uint8_t ch,
                         const uint8_t *data, uint16_t count);

/* 绘制一条完整波形，data 固定为 300 个 0~200 的像素值。 */
uint8_t HMI_WaveDraw(const char *ctl, uint8_t ch, const uint8_t *data);

void UART1_Printf(const char *format, ...);
void UART3_Printf(const char *format, ...);

#endif /* __SERIAL_H */
