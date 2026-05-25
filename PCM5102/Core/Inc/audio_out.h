#ifndef __AUDIO_OUT_H__
#define __AUDIO_OUT_H__

#include "main.h"

#define AUDIO_BUF_SIZE  1024
#define AUDIO_SAMPLE_RATE 48000U

typedef void (*AudioOut_FillCallback)(int16_t *buffer, uint32_t half);

void AudioOut_Init(void);
void AudioOut_Start(void);
void AudioOut_Stop(void);
void AudioOut_SetFillCallback(AudioOut_FillCallback cb);

#endif
