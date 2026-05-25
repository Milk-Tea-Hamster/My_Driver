#include "audio_out.h"
#include "i2s.h"

static int16_t tx_buffer[AUDIO_BUF_SIZE];
static AudioOut_FillCallback fill_callback;

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2 && fill_callback) {
        fill_callback(tx_buffer, 0);
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2 && fill_callback) {
        fill_callback(tx_buffer, 1);
    }
}

void AudioOut_Init(void)
{
    MX_I2S2_Init();
    fill_callback = NULL;
}

void AudioOut_SetFillCallback(AudioOut_FillCallback cb)
{
    fill_callback = cb;
}

void AudioOut_Start(void)
{
    if (fill_callback) {
        fill_callback(tx_buffer, 0);
        fill_callback(tx_buffer, 1);
    }
    if (HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)tx_buffer, AUDIO_BUF_SIZE) != HAL_OK) {
        Error_Handler();
    }
}

void AudioOut_Stop(void)
{
    HAL_I2S_DMAStop(&hi2s2);
}
