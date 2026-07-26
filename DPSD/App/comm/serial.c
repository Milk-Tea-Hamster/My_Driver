#include "serial.h"

#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

__attribute__((section(".bss.AXI_SRAM"), aligned(32)))
uint8_t uart1_rx_buf[RXBUFFERSIZE];

volatile uint16_t uart1_rx_len;
volatile uint8_t uart1_rx_flag;

__attribute__((section(".bss.AXI_SRAM"), aligned(32)))
uint8_t uart3_rx_buf[RXBUFFERSIZE];
volatile uint16_t uart3_rx_len;
volatile uint8_t uart3_rx_flag;

static int serial_format(char *buffer,
                         size_t buffer_size,
                         const char *format,
                         va_list args)
{
    int length;

    if ((buffer == NULL) || (buffer_size == 0U) || (format == NULL)) {
        return 0;
    }

    length = vsnprintf(buffer, buffer_size, format, args);
    if (length <= 0) {
        return 0;
    }
    if ((size_t)length >= buffer_size) {
        length = (int)buffer_size - 1;
    }
    return length;
}

static void serial_vprintf(UART_HandleTypeDef *huart,
                           const char *format,
                           va_list args)
{
    char buffer[256];
    int length = serial_format(buffer, sizeof(buffer), format, args);

    if ((huart == NULL) || (length <= 0)) {
        return;
    }
    (void)HAL_UART_Transmit(
        huart,
        (uint8_t *)buffer,
        (uint16_t)length,
        50U);
}

void Serial_Printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    serial_vprintf(huart, fmt, args);
    va_end(args);
}

static HAL_StatusTypeDef serial_start_rx(UART_HandleTypeDef *huart,
                                         uint8_t *buffer,
                                         DMA_HandleTypeDef *hdma)
{
    HAL_StatusTypeDef status;

    SCB_CleanInvalidateDCache_by_Addr(
        (uint32_t *)buffer,
        (int32_t)RXBUFFERSIZE);
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        buffer,
        RXBUFFERSIZE);
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
    }
    return status;
}

HAL_StatusTypeDef Serial_RxInit(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return HAL_ERROR;
    }

    if (huart->Instance == USART1) {
        uart1_rx_flag = 0U;
        uart1_rx_len = 0U;
        return serial_start_rx(
            huart,
            uart1_rx_buf,
            &hdma_usart1_rx);
    }
    if (huart->Instance == USART3) {
        uart3_rx_flag = 0U;
        uart3_rx_len = 0U;
        return serial_start_rx(
            huart,
            uart3_rx_buf,
            &hdma_usart3_rx);
    }
    return HAL_ERROR;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART1) {
        uart1_rx_len = (size <= RXBUFFERSIZE)
                           ? size
                           : RXBUFFERSIZE;
        uart1_rx_flag = 1U;
    } else if (huart->Instance == USART3) {
        uart3_rx_len = (size <= RXBUFFERSIZE)
                           ? size
                           : RXBUFFERSIZE;
        uart3_rx_flag = 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart->Instance == USART1) || (huart->Instance == USART3)) {
        (void)HAL_UART_AbortReceive(huart);
        (void)Serial_RxInit(huart);
    }
}

void Serial_Process(void)
{
    if (uart1_rx_flag) {
        const uint16_t length = uart1_rx_len;
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)uart1_rx_buf,
            (int32_t)RXBUFFERSIZE);
        Serial_OnFrame(&huart1, uart1_rx_buf, length);
        uart1_rx_flag = 0U;
        (void)Serial_RxInit(&huart1);
    }

    if (uart3_rx_flag) {
        const uint16_t length = uart3_rx_len;
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)uart3_rx_buf,
            (int32_t)RXBUFFERSIZE);
        Serial_OnFrame(&huart3, uart3_rx_buf, length);
        uart3_rx_flag = 0U;
        (void)Serial_RxInit(&huart3);
    }
}

__weak void Serial_OnFrame(UART_HandleTypeDef *huart,
                           const uint8_t *data,
                           uint16_t length)
{
    (void)huart;
    (void)data;
    (void)length;
}

static void HMI_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(
        buffer,
        sizeof(buffer) - 3U,
        format,
        args);
    va_end(args);

    if (length <= 0) {
        return;
    }
    if ((size_t)length >= sizeof(buffer) - 3U) {
        length = (int)sizeof(buffer) - 4;
    }

    buffer[length++] = (char)0xFF;
    buffer[length++] = (char)0xFF;
    buffer[length++] = (char)0xFF;
    (void)HAL_UART_Transmit(
        &huart3,
        (uint8_t *)buffer,
        (uint16_t)length,
        50U);
}

void HMI_SendStr(const char *ctl, const char *text)
{
    HMI_Printf("%s=\"%s\"", ctl, text);
}

void HMI_SendInt(const char *ctl, int num)
{
    HMI_Printf("%s=%d", ctl, num);
}

void HMI_SendFloat(const char *ctl, float num, int decimals)
{
    static const int multiplier[] = {1, 10, 100, 1000};
    const int digits = (decimals >= 0 && decimals <= 3) ? decimals : 2;
    const float scaled_float = num * (float)multiplier[digits];
    const long scaled = (long)(
        scaled_float + ((scaled_float >= 0.0f) ? 0.5f : -0.5f));

    HMI_Printf("%s=%ld", ctl, scaled);
}

void HMI_WaveAdd(const char *ctl, int ch, int val)
{
    HMI_Printf("add %s,%d,%d", ctl, ch, val);
}

void HMI_WaveClear(const char *ctl, int ch)
{
    HMI_Printf("cle %s,%d", ctl, ch);
}

static uint8_t HMI_WaitTransparentReply(uint8_t marker, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t valid = 0U;

    while (uart3_rx_flag == 0U) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            break;
        }
    }

    if (uart3_rx_flag != 0U) {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)uart3_rx_buf,
            (int32_t)RXBUFFERSIZE);

        for (uint16_t i = 0U; (i + 3U) < uart3_rx_len; i++) {
            if ((uart3_rx_buf[i] == marker) &&
                (uart3_rx_buf[i + 1U] == 0xFFU) &&
                (uart3_rx_buf[i + 2U] == 0xFFU) &&
                (uart3_rx_buf[i + 3U] == 0xFFU)) {
                valid = 1U;
                break;
            }
        }
    }

    (void)HAL_UART_AbortReceive(&huart3);
    uart3_rx_flag = 0U;
    uart3_rx_len = 0U;
    (void)Serial_RxInit(&huart3);

    return valid;
}

uint8_t HMI_WaveAddBatch(const char *ctl, uint8_t ch,
                         const uint8_t *data, uint16_t count)
{
    char command[32];
    int length;

    if ((ctl == NULL) || (data == NULL) ||
        (count == 0U) || (count > 1024U)) {
        return 0U;
    }

    length = snprintf(
        command,
        sizeof(command),
        "addt %s,%u,%u",
        ctl,
        (unsigned int)ch,
        (unsigned int)count);
    if ((length <= 0) || ((length + 3) > (int)sizeof(command))) {
        return 0U;
    }

    command[length++] = (char)0xFF;
    command[length++] = (char)0xFF;
    command[length++] = (char)0xFF;

    (void)HAL_UART_AbortReceive(&huart3);
    __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);
    uart3_rx_flag = 0U;
    uart3_rx_len = 0U;
    (void)Serial_RxInit(&huart3);

    if ((HAL_UART_Transmit(
             &huart3,
             (uint8_t *)command,
             (uint16_t)length,
             100U) != HAL_OK) ||
        (HMI_WaitTransparentReply(0xFEU, 500U) == 0U)) {
        return 0U;
    }

    if ((HAL_UART_Transmit(
             &huart3,
             (uint8_t *)data,
             count,
             HAL_MAX_DELAY) != HAL_OK) ||
        (HMI_WaitTransparentReply(0xFDU, 500U) == 0U)) {
        return 0U;
    }

    return 1U;
}

uint8_t HMI_WaveDraw(const char *ctl, uint8_t ch, const uint8_t *data)
{
    static uint8_t wave_data[HMI_WAVE_WIDTH];

    if ((ctl == NULL) || (data == NULL)) {
        return 0U;
    }

    for (uint32_t i = 0U; i < HMI_WAVE_WIDTH; i++) {
        uint8_t value = data[i];

        if (value > HMI_WAVE_HEIGHT) {
            value = HMI_WAVE_HEIGHT;
        }
        wave_data[i] = value;
    }

    HMI_WaveClear(ctl, ch);
    return HMI_WaveAddBatch(ctl, ch, wave_data, HMI_WAVE_WIDTH);
}

void UART1_Printf(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    serial_vprintf(&huart1, format, args);
    va_end(args);
}

void UART3_Printf(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    serial_vprintf(&huart3, format, args);
    va_end(args);
}
