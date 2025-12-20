#ifndef VENUS_EFFECT_H
#define VENUS_EFFECT_H

#include <cmath>
#include <algorithm>
#include "VenusHelpers.h"
#include "shy_fft.h"
#include "wave.h"

// Define constants from original code
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// FFT Configuration
// N = 4096 and laps = 4 (higher frequency resolution, greater latency)
const size_t VENUS_ORDER = 12;
const size_t VENUS_N = (1 << VENUS_ORDER);
const size_t VENUS_LAPS = 4;
const size_t VENUS_BUFFSIZE = 2 * VENUS_LAPS * VENUS_N;

using namespace soundmath;

class VenusEffect {
public:
    VenusEffect();
    ~VenusEffect();

    // Initialize the effect with sample rate
    void Init(float sample_rate);

    // Process a block of audio
    // in_left, in_right: input buffers
    // out_left, out_right: output buffers
    // size: number of samples in the buffer
    void Process(const float* in_left, const float* in_right, float* out_left, float* out_right, size_t size);

    // Parameter Setters
    void SetDecay(float value);       // 0.0 - 1.0
    void SetMix(float value);         // 0.0 - 1.0
    void SetDamp(float value);        // 0.0 - 1.0
    void SetShimmer(float value);     // 0.0 - 1.0
    void SetShimmerTone(float value); // 0.0 - 1.0
    void SetDetune(float value);      // -1.0 to 1.0 (Mapped from original ranges)

    // Switch/Mode Setters
    void SetShimmerMode(int mode);    // 0=Down, 1=Up, 2=Both
    void SetReverbMode(int mode);     // 0=Less Lofi, 1=Normal, 2=More Lofi
    void SetDriftMode(int mode);      // 0=Slower, 1=None, 2=Faster
    void SetFreeze(bool freeze);
    void SetBypass(bool bypass);

private:
    float sample_rate_;
    bool bypass_;
    bool freeze_;

    // Parameters
    float vdecay, vmix, vdamp, vshimmer, vshimmer_tone, vdetune;

    // Internal State for Shimmer/Detune logic
    float octave_up_rate_persecond, octave_up_rate_perinterval;
    float shimmer_double, shimmer_triple, shimmer_remainder;
    float detune_rate_persecond, detune_rate_perinterval, detune_double, detune_remainder;

    float window_samples;
    float interval_samples;

    int shimmer_mode;
    int detune_mode;
    int detune_multiplier;

    int drift_mode;
    float drift_multiplier, drift_multiplier2, drift_multiplier3, drift_multiplier4;

    // DSP Modules
    SimpleSampleRateReducer samplerateReducer;
    SimpleLowPass lowpass;
    int reverb_mode;

    // LFOs for Drift
    SimpleOscillator drift_osc;
    SimpleOscillator drift_osc2;
    SimpleOscillator drift_osc3;
    SimpleOscillator drift_osc4;

    // FFT / STFT Buffers and Objects
    float* in_buffer;
    float* middle_buffer;
    float* out_buffer;
    float* reverb_energy;

    ShyFFT<float, VENUS_N, RotationPhasor>* fft;
    // Fourier logic is embedded, so no member needed

    Wave<float>* hann_window;

    // Helper method to update internal calculations when parameters change
    void UpdateInternalParameters();

    // Fourier State Variables (from Fourier.h)
    size_t stride;
    int* writepoints;
    int* readpoints;
    bool* reading;
    bool* writing;
    int current_lap;

    // Internal methods implementing the STFT flow
    void STFT_Write(float x);
    float STFT_Read();
    void STFT_Forward(size_t i);
    void STFT_Backward(size_t i);
    void SpectralProcess(const float* in_spectral, float* out_spectral);

};

#endif // VENUS_EFFECT_H
