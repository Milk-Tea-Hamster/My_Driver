#include "voice.h"
#include "audio_out.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
   Voice parameters
   ======================================================================== */

#define VOICE_DURATION_PAIRS    (AUDIO_SAMPLE_RATE * 1)    /* 1 second */
#define VOICE_ATTACK_PAIRS      (AUDIO_SAMPLE_RATE * 5 / 100) /* 50ms */
#define VOICE_DECAY_PAIRS       (AUDIO_SAMPLE_RATE * 15 / 100) /* 150ms */
#define VOICE_SILENCE_PAIRS     (AUDIO_SAMPLE_RATE / 10)    /* 100ms trailing silence */

#define SINE_TABLE_SIZE  256U
#define NUM_PARTIALS     4

static int16_t  sine_table[SINE_TABLE_SIZE];
static uint32_t partial_phase[NUM_PARTIALS];
static uint32_t partial_step[NUM_PARTIALS];
static uint32_t voice_pair_count;

static const float partial_freq[NUM_PARTIALS] = {150.0f, 730.0f, 1090.0f, 2440.0f};
static const float partial_amp[NUM_PARTIALS]  = {0.45f,  0.28f,  0.18f,   0.09f};
static const double DDS_SCALE = 4294967296.0;

/* ========================================================================
   Private helpers
   ======================================================================== */

static void InitSineTable(void)
{
    for (uint32_t i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = (int16_t)(sin((double)i / SINE_TABLE_SIZE * 2.0 * M_PI) * 32767.0);
    }
}

static float Envelope(void)
{
    if (voice_pair_count < VOICE_ATTACK_PAIRS) {
        return (float)voice_pair_count / (float)VOICE_ATTACK_PAIRS;
    }
    uint32_t decay_start = VOICE_DURATION_PAIRS - VOICE_DECAY_PAIRS;
    if (voice_pair_count >= decay_start && voice_pair_count < VOICE_DURATION_PAIRS) {
        return 1.0f - (float)(voice_pair_count - decay_start) / (float)VOICE_DECAY_PAIRS;
    }
    if (voice_pair_count >= VOICE_DURATION_PAIRS) {
        return 0.0f;
    }
    return 1.0f;
}

static void Voice_FillBuffer(int16_t *buf, uint32_t half)
{
    uint32_t start = half * (AUDIO_BUF_SIZE / 2U);
    uint32_t end   = start + (AUDIO_BUF_SIZE / 2U);

    for (uint32_t i = start; i < end; i += 2U) {
        if (voice_pair_count >= VOICE_DURATION_PAIRS + VOICE_SILENCE_PAIRS) {
            AudioOut_Stop();
            buf[i]     = 0;
            buf[i + 1] = 0;
            continue;
        }

        float sample = 0.0f;
        for (int j = 0; j < NUM_PARTIALS; j++) {
            int32_t raw = sine_table[(partial_phase[j] >> 24) & 0xFFU];
            sample += (float)raw * partial_amp[j];
            partial_phase[j] += partial_step[j];
        }
        sample = sample / 32767.0f * Envelope();

        int16_t s = (int16_t)(sample * 16384.0f);
        buf[i]     = s;
        buf[i + 1] = s;
        voice_pair_count++;
    }
}

/* ========================================================================
   Public API
   ======================================================================== */

void Voice_Init(void)
{
    InitSineTable();
    for (int j = 0; j < NUM_PARTIALS; j++) {
        partial_phase[j] = 0;
        partial_step[j] = (uint32_t)((double)partial_freq[j] / (double)AUDIO_SAMPLE_RATE * DDS_SCALE);
    }
    voice_pair_count = 0;
}

void Voice_Play(void)
{
    AudioOut_Stop();
    Voice_Init();
    AudioOut_SetFillCallback(Voice_FillBuffer);
    AudioOut_Start();
}
