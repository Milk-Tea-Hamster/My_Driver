#include "fft_analyzer.h"


#include "arm_common_tables_extra.h"
#include "arm_const_structs_extra.h"
#include "extra_ffts.h"
#include <math.h>
#include <string.h>

#define FFT_PI 3.14159265358979323846f

static FFT_Config s_config = {
    .sample_rate_hz = 0.0f,
    .adc_vref = ADC_VREF,
    .adc_max_code = ADC_MAX_CODE,
    .dc_cutoff_hz = FFT_DEFAULT_DC_CUTOFF_HZ
};
static uint8_t s_initialized;
static float s_window_coherent_gain = 1.0f;
__attribute__((section(".bss.DTCM_RAM"), aligned(32)))
static fftin_t s_fft_input;
__attribute__((section(".bss.DTCM_RAM"), aligned(32)))
static fftout_t s_fft_output;
__attribute__((section(".bss.DTCM_RAM"), aligned(32)))
static float s_hamming_window[FFT_N];
static freqaxis_t s_frequency_axis;

static const arm_cfft_instance_f32_extra *fft_get_instance(uint16_t fft_n)
{
    switch (fft_n) {
        case 16:    return &arm_cfft_sR_f32_len16_extra;
        case 32:    return &arm_cfft_sR_f32_len32_extra;
        case 64:    return &arm_cfft_sR_f32_len64_extra;
        case 128:   return &arm_cfft_sR_f32_len128_extra;
        case 256:   return &arm_cfft_sR_f32_len256_extra;
        case 512:   return &arm_cfft_sR_f32_len512_extra;
        case 1024:  return &arm_cfft_sR_f32_len1024_extra;
        case 2048:  return &arm_cfft_sR_f32_len2048_extra;
        case 4096:  return &arm_cfft_sR_f32_len4096_extra;
        case 8192:  return &arm_cfft_sR_f32_len8192_extra;
        case 16384: return &arm_cfft_sR_f32_len16384_extra;
        default:    return NULL;
    }
}


static float fft_parabolic_delta(float left, float center, float right)
{
    const float denominator = left - (2.0f * center) + right;
    float delta;

    if (fabsf(denominator) < 1.0e-12f) {
        return 0.0f;
    }

    delta = 0.5f * (left - right) / denominator;
    if (delta > 0.5f) delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;
    return delta;
}

float FFT_BinToHz(float bin)
{
    return bin * s_config.sample_rate_hz / (float)FFT_N;
}

void fft_prepare(const uint16_t *adc_data, fftin_t *out)
{
    float dc_offset = 0.0f;
    float voltage_scale;

    if ((adc_data == NULL) || (out == NULL) ||
        (s_config.adc_max_code == 0U)) {
        return;
    }

    for (uint32_t i = 0U; i < FFT_N; ++i) {
        dc_offset += (float)adc_data[i];
    }
    dc_offset /= (float)FFT_N;
    voltage_scale = s_config.adc_vref / (float)s_config.adc_max_code;

    for (uint32_t i = 0U; i < FFT_N; ++i) {
        out->cmp[2U * i] =
            ((float)adc_data[i] - dc_offset) * voltage_scale;
        out->cmp[2U * i + 1U] = 0.0f;
    }
    s_window_coherent_gain = 1.0f;
}

void fft_window(fftin_t *data, const float *window_func)
{
    float gain_sum = 0.0f;

    if ((data == NULL) || (window_func == NULL)) {
        return;
    }

    for (uint32_t i = 0U; i < FFT_N; ++i) {
        data->cmp[2U * i] *= window_func[i];
        gain_sum += window_func[i];
    }

    s_window_coherent_gain = gain_sum / (float)FFT_N;
    if (s_window_coherent_gain < FFT_MIN_VALID_MAG) {
        s_window_coherent_gain = 1.0f;
    }
}

void fft_process(fftin_t *data, fftout_t *output)
{
    const arm_cfft_instance_f32_extra *instance;
    const float magnitude_scale =
        2.0f / ((float)FFT_N * s_window_coherent_gain);

    if ((data == NULL) || (output == NULL)) {
        return;
    }

    instance = fft_get_instance((uint16_t)FFT_N);
    if (instance == NULL) {
        memset(output, 0, sizeof(*output));
        return;
    }

    arm_cfft_f32_extra(
        (arm_cfft_instance_f32_extra *)instance,
        data->cmp,
        0U,
        1U);

    for (uint32_t i = 0U; i < FFT_N_2; ++i) {
        const float real = data->cmp[2U * i];
        const float imag = data->cmp[2U * i + 1U];
        float magnitude;

        (void)arm_sqrt_f32((real * real) + (imag * imag), &magnitude);
        output->mag[i] = magnitude * magnitude_scale;
        output->phase[i] = atan2f(imag, real);

        if (FFT_BinToHz((float)i) <= s_config.dc_cutoff_hz) {
            output->mag[i] = 0.0f;
        }
    }
}

void fft_normalize(fftout_t *result, float norm_val)
{
    float max_magnitude = 0.0f;

    if (result == NULL) {
        return;
    }

    for (uint32_t i = 0U; i < FFT_N_2; ++i) {
        if (result->mag[i] > max_magnitude) {
            max_magnitude = result->mag[i];
        }
    }
    if (max_magnitude < FFT_MIN_VALID_MAG) {
        return;
    }
    for (uint32_t i = 0U; i < FFT_N_2; ++i) {
        result->mag[i] = result->mag[i] / max_magnitude * norm_val;
    }
}

freqaxis_t *fft_get_axis(void)
{
    for (uint32_t i = 0U; i < FFT_N_2; ++i) {
        s_frequency_axis.axis[i] = FFT_BinToHz((float)i);
    }
    return &s_frequency_axis;
}

void fft_find_peaks(const fftout_t *spectrum, peak3_t *peaks)
{
    if ((spectrum == NULL) || (peaks == NULL)) {
        return;
    }

    memset(peaks, 0, sizeof(*peaks));
    for (uint32_t rank = 0U; rank < 3U; ++rank) {
        float best_magnitude = 0.0f;
        uint16_t best_index = 0U;

        for (uint32_t i = 1U; i < FFT_N_2 - 1U; ++i) {
            uint8_t separated = 1U;

            if ((spectrum->mag[i] <= spectrum->mag[i - 1U]) ||
                (spectrum->mag[i] < spectrum->mag[i + 1U]) ||
                (spectrum->mag[i] <= best_magnitude)) {
                continue;
            }

            for (uint32_t p = 0U; p < peaks->count; ++p) {
                const uint32_t selected = peaks->index[p];
                const uint32_t distance =
                    (i > selected) ? (i - selected) : (selected - i);
                if (distance <= FFT_PEAK_MIN_SEPARATION_BINS) {
                    separated = 0U;
                    break;
                }
            }

            if (separated) {
                best_magnitude = spectrum->mag[i];
                best_index = (uint16_t)i;
            }
        }

        if (best_magnitude < FFT_MIN_VALID_MAG) {
            break;
        }
        peaks->index[peaks->count++] = best_index;
    }
}

float fft_peak_frequency(const fftout_t *spectrum, uint16_t peak_bin)
{
    float interpolated_bin;

    if ((spectrum == NULL) ||
        (peak_bin == 0U) ||
        (peak_bin >= FFT_N_2 - 1U)) {
        return FFT_BinToHz((float)peak_bin);
    }

    interpolated_bin = (float)peak_bin + fft_parabolic_delta(
        spectrum->mag[peak_bin - 1U],
        spectrum->mag[peak_bin],
        spectrum->mag[peak_bin + 1U]);
    return FFT_BinToHz(interpolated_bin);
}

float fft_round_freq(float raw_freq)
{
    if (raw_freq <= 0.0f) {
        return 0.0f;
    }
    return floorf(raw_freq + 0.5f);
}

float find_vpp(const fftin_t *input)
{
    float minimum;
    float maximum;

    if (input == NULL) {
        return 0.0f;
    }

    minimum = input->cmp[0];
    maximum = input->cmp[0];
    for (uint32_t i = 1U; i < FFT_N; ++i) {
        const float sample = input->cmp[2U * i];
        if (sample < minimum) minimum = sample;
        if (sample > maximum) maximum = sample;
    }
    return maximum - minimum;
}

void fft_phase_atan(const float *complex_data,
                    uint32_t index,
                    float *phase)
{
    if ((complex_data == NULL) || (phase == NULL) || (index >= FFT_N)) {
        return;
    }
    *phase = atan2f(
        complex_data[2U * index + 1U],
        complex_data[2U * index]);
}

float fft_max_harmonic(const float *mag,
                       uint16_t base_idx,
                       uint8_t harmonic_n)
{
    const uint32_t target = (uint32_t)harmonic_n * base_idx;
    float maximum = 0.0f;

    if ((mag == NULL) || (target >= FFT_N_2)) {
        return 0.0f;
    }

    for (int32_t offset = -2; offset <= 2; ++offset) {
        const int32_t index = (int32_t)target + offset;
        if ((index >= 0) &&
            (index < (int32_t)FFT_N_2) &&
            (mag[index] > maximum)) {
            maximum = mag[index];
        }
    }
    return maximum;
}

WaveType_t fft_rec_wavetype(const fftout_t *spectrum, uint16_t base_idx)
{
    float third_ratio;
    const float base_magnitude =
        (spectrum != NULL && base_idx < FFT_N_2)
            ? spectrum->mag[base_idx]
            : 0.0f;

    if ((base_idx == 0U) || (base_magnitude < FFT_MIN_VALID_MAG)) {
        return WAVE_UNKNOWN;
    }

    third_ratio =
        fft_max_harmonic(spectrum->mag, base_idx, 3U) / base_magnitude;

    if (third_ratio < WAVE_SINE_H3_MAX_RATIO) {
        return WAVE_SINE;
    }
    if (third_ratio < WAVE_TRIANGLE_H3_MAX_RATIO) {
        return WAVE_TRIANGLE;
    }
    return WAVE_SQUARE;
}

uint8_t FFT_Analyze(const uint16_t *adc_data, FFT_Result *result)
{
    peak3_t peaks;

    if (!s_initialized || (adc_data == NULL) || (result == NULL)) {
        return 0U;
    }

    memset(result, 0, sizeof(*result));
    result->wave_type = WAVE_UNKNOWN;

    fft_prepare(adc_data, &s_fft_input);
    result->vpp = find_vpp(&s_fft_input);
    fft_window(&s_fft_input, s_hamming_window);
    fft_process(&s_fft_input, &s_fft_output);
    fft_find_peaks(&s_fft_output, &peaks);

    if (peaks.count == 0U) {
        return 0U;
    }

    result->fundamental_bin = peaks.index[0];
    result->fundamental_hz =
        fft_peak_frequency(&s_fft_output, peaks.index[0]);
    result->fundamental_mag = s_fft_output.mag[peaks.index[0]];
    result->phase_rad = s_fft_output.phase[peaks.index[0]];
    result->third_harmonic_ratio =
        fft_max_harmonic(s_fft_output.mag, peaks.index[0], 3U) /
        result->fundamental_mag;
    result->wave_type =
        fft_rec_wavetype(&s_fft_output, peaks.index[0]);
    return 1U;
}
