#include "sweep.h"
#include "audio_out.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
   Configurable parameters
   ======================================================================== */

#define SINE_TABLE_SIZE     256U
#define SAMPLES_PER_HALF    (AUDIO_BUF_SIZE / 4U)

#define SWEEP_START_FREQ    20.0f
#define SWEEP_END_FREQ      8000.0f
#define SWEEP_STEP_HZ       10.0f
#define SWEEP_DURATION_SEC  60.0f

/* ========================================================================
   Static data
   ======================================================================== */

static int16_t  sine_table[SINE_TABLE_SIZE];
static uint32_t dds_phase_acc;
static uint32_t dds_phase_step;

static float    sweep_freq;
static int32_t  sweep_step_index;
static int32_t  sweep_total_steps;
static uint32_t sweep_pairs_per_step;
static uint32_t sweep_pair_counter;

static const double DDS_SCALE = 4294967296.0;

/* ========================================================================
   Private helpers
   ======================================================================== */

static void InitSineTable(void)
{
    for (uint32_t i = 0; i < SINE_TABLE_SIZE; i++) {
        double phase = (double)i / (double)SINE_TABLE_SIZE * 2.0 * M_PI;
        sine_table[i] = (int16_t)(sin(phase) * 32767.0);
    }
}

static void SetSweepFrequency(float freq)
{
    sweep_freq = freq;
    dds_phase_step = (uint32_t)((double)sweep_freq / (double)AUDIO_SAMPLE_RATE * DDS_SCALE);
}

static void Sweep_FillBuffer(int16_t *buf, uint32_t half)
{
    uint32_t start = half * (AUDIO_BUF_SIZE / 2U);
    uint32_t end   = start + (AUDIO_BUF_SIZE / 2U);

    for (uint32_t i = start; i < end; i += 2U) {
        int16_t sample = sine_table[(dds_phase_acc >> 24) & 0xFFU];
        buf[i]     = sample;
        buf[i + 1] = sample;
        dds_phase_acc += dds_phase_step;
    }

    sweep_pair_counter += SAMPLES_PER_HALF;
    if (sweep_pair_counter >= sweep_pairs_per_step) {
        sweep_pair_counter -= sweep_pairs_per_step;
        sweep_step_index++;
        if (sweep_step_index > sweep_total_steps) {
            sweep_step_index = 0;
        }
        SetSweepFrequency(SWEEP_START_FREQ + (float)sweep_step_index * SWEEP_STEP_HZ);
    }
}

/* ========================================================================
   Public API
   ======================================================================== */

void Sweep_Init(void)
{
    InitSineTable();

    sweep_step_index  = 0;
    sweep_total_steps = (int32_t)((SWEEP_END_FREQ - SWEEP_START_FREQ) / SWEEP_STEP_HZ);
    sweep_pair_counter = 0;

    float total_pairs = (float)AUDIO_SAMPLE_RATE * SWEEP_DURATION_SEC;
    sweep_pairs_per_step = (uint32_t)(total_pairs / (float)sweep_total_steps);

    dds_phase_acc = 0U;
    SetSweepFrequency(SWEEP_START_FREQ);
}

void Sweep_Start(void)
{
    AudioOut_SetFillCallback(Sweep_FillBuffer);
    AudioOut_Start();
}
