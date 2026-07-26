#ifndef __FFT_ANALYZER_H
#define __FFT_ANALYZER_H

#include "adc_app.h"
#include "app_types.h"
#include "arm_math.h"
#include <stdint.h>

#define FFT_N       ADC_LEN
#define FFT_2N      (FFT_N * 2U)
#define FFT_N_2     (FFT_N / 2U)



#define FFT_DEFAULT_DC_CUTOFF_HZ       80.0f
#define FFT_PEAK_MIN_SEPARATION_BINS   3U
#define FFT_MIN_VALID_MAG              1.0e-6f


#define WAVE_SINE_H3_MAX_RATIO      0.06f//正弦波识别阈值
#define WAVE_TRIANGLE_H3_MAX_RATIO  0.22f//

typedef struct {
    float cmp[FFT_2N];
} fftin_t;

typedef struct {
    float phase[FFT_N_2];
    float mag[FFT_N_2];
} fftout_t;

typedef struct {
    uint16_t index[3];
    uint8_t count;
} peak3_t;

typedef struct {
    float axis[FFT_N_2];
} freqaxis_t;

typedef struct {
    float sample_rate_hz;
    float adc_vref;
    uint16_t adc_max_code;
    float dc_cutoff_hz;
} FFT_Config;

typedef struct {
    float fundamental_hz;
    float fundamental_mag;
    float phase_rad;
    float vpp;
    float third_harmonic_ratio;
    uint16_t fundamental_bin;
    WaveType_t wave_type;
} FFT_Result;

float FFT_BinToHz(float bin);


void fft_prepare(const uint16_t *adc_data, fftin_t *out);

void fft_window(fftin_t *data, const float *window_func);

void fft_process(fftin_t *data, fftout_t *output);

void fft_normalize(fftout_t *result, float norm_val);

freqaxis_t *fft_get_axis(void);

void fft_find_peaks(const fftout_t *spectrum, peak3_t *peaks);

float fft_peak_frequency(const fftout_t *spectrum, uint16_t peak_bin);

float fft_round_freq(float raw_freq);

WaveType_t fft_rec_wavetype(const fftout_t *spectrum, uint16_t base_idx);

float find_vpp(const fftin_t *input);

void fft_phase_atan(const float *complex_data, uint32_t index, float *phase);

float fft_max_harmonic(const float *mag,
                       uint16_t base_idx,
                       uint8_t harmonic_n);

#endif /* __FFT_ANALYZER_H */
