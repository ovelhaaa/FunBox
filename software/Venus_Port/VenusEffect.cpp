#include "VenusEffect.h"
#include <cstring>
#include <cstdlib> // for rand()

VenusEffect::VenusEffect() : fft(nullptr), hann_window(nullptr), in_buffer(nullptr), middle_buffer(nullptr), out_buffer(nullptr), reverb_energy(nullptr), writepoints(nullptr), readpoints(nullptr), reading(nullptr), writing(nullptr) {
}

VenusEffect::~VenusEffect() {
    if (fft) delete fft;
    if (hann_window) delete hann_window;

    if (in_buffer) delete[] in_buffer;
    if (middle_buffer) delete[] middle_buffer;
    if (out_buffer) delete[] out_buffer;
    if (reverb_energy) delete[] reverb_energy;

    if (writepoints) delete[] writepoints;
    if (readpoints) delete[] readpoints;
    if (reading) delete[] reading;
    if (writing) delete[] writing;
}

void VenusEffect::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    bypass_ = false;
    freeze_ = false;

    // Allocate Buffers
    in_buffer = new float[VENUS_BUFFSIZE];
    middle_buffer = new float[VENUS_BUFFSIZE];
    out_buffer = new float[VENUS_BUFFSIZE];
    reverb_energy = new float[VENUS_N / 2];

    // Clear buffers
    memset(in_buffer, 0, sizeof(float) * VENUS_BUFFSIZE);
    memset(middle_buffer, 0, sizeof(float) * VENUS_BUFFSIZE);
    memset(out_buffer, 0, sizeof(float) * VENUS_BUFFSIZE);
    memset(reverb_energy, 0, sizeof(float) * (VENUS_N / 2));

    // Initialize Parameters
    vdecay = 10.0f;
    vmix = 0.5f;
    vdamp = 0.1f;
    vshimmer = 0.0f;
    vshimmer_tone = 0.0f;
    vdetune = 0.0f;

    shimmer_mode = 0;
    detune_mode = 1;
    detune_multiplier = 1;
    drift_mode = 1;
    reverb_mode = 1;

    window_samples = (float)VENUS_N; // Fixed bug: was VENUS_BUFFSIZE
    interval_samples = std::ceil(window_samples / VENUS_LAPS);

    // Initialize DSP Modules
    samplerateReducer.Init();
    samplerateReducer.SetFreq(0.3f);

    lowpass.Init(sample_rate);
    lowpass.SetFreq(8000.0f);

    drift_osc.Init(sample_rate);
    drift_osc.SetAmp(1.0f);
    drift_osc2.Init(sample_rate);
    drift_osc2.SetAmp(1.0f);
    drift_osc3.Init(sample_rate);
    drift_osc3.SetAmp(1.0f);
    drift_osc4.Init(sample_rate);
    drift_osc4.SetAmp(1.0f);

    // Initialize FFT
    fft = new ShyFFT<float, VENUS_N, RotationPhasor>();
    fft->Init();

    // Initialize Window (Hann)
    // Wave<float> hann([] (float phase) -> float { return 0.5 * (1 - cos(2 * PI * phase)); });
    hann_window = new Wave<float>([](float phase) -> float { return 0.5f * (1.0f - std::cos(2.0f * PI * phase)); });

    // Initialize STFT State
    stride = VENUS_N / VENUS_LAPS;
    writepoints = new int[VENUS_LAPS * 2];
    readpoints = new int[VENUS_LAPS * 2];
    reading = new bool[VENUS_LAPS * 2];
    writing = new bool[VENUS_LAPS * 2];

    memset(writepoints, 0, sizeof(int) * VENUS_LAPS * 2);
    memset(readpoints, 0, sizeof(int) * VENUS_LAPS * 2);
    memset(reading, false, sizeof(bool) * VENUS_LAPS * 2);
    memset(writing, true, sizeof(bool) * VENUS_LAPS * 2);

    for (int i = 0; i < 2 * (int)VENUS_LAPS; i++) {
        writepoints[i] = -i * (int)stride;
    }

    UpdateInternalParameters();
}

void VenusEffect::UpdateInternalParameters() {
    octave_up_rate_persecond = std::pow(8.0f, vshimmer) - 1.0f;
    octave_up_rate_perinterval = std::min(0.75f, octave_up_rate_persecond / sample_rate_ * interval_samples);

    float octave_up_rate_persecond2 = std::pow(8.0f, vshimmer_tone) - 1.0f;
    float octave_up_rate_perinterval2 = std::min(0.75f, octave_up_rate_persecond2 / sample_rate_ * interval_samples);

    shimmer_double = octave_up_rate_perinterval * (1.0f - vshimmer_tone / 1.58f);
    shimmer_triple = (octave_up_rate_perinterval2 / 1.58f) * vshimmer_tone;
    shimmer_remainder = (1.0f - shimmer_double - shimmer_triple);

    detune_rate_persecond = std::pow(8.0f, vdetune) - 1.0f;
    detune_rate_perinterval = std::min(0.75f, detune_rate_persecond / sample_rate_ * interval_samples);
    detune_double = detune_rate_perinterval;
    detune_remainder = 1.0f - detune_double;
}

void VenusEffect::Process(const float* in_left, const float* in_right, float* out_left, float* out_right, size_t size) {
    for (size_t i = 0; i < size; i++) {
        // Drift Processing
        drift_multiplier = drift_osc.Process();
        drift_multiplier2 = drift_osc2.Process();
        drift_multiplier3 = drift_osc3.Process();
        drift_multiplier4 = drift_osc4.Process();

        if (bypass_) {
            out_left[i] = in_left[i];
            out_right[i] = in_right[i];
            continue;
        }

        STFT_Write(in_left[i]); // Mono in (Left)

        float wet = 0.0f;
        float stft_out = STFT_Read();

        if (reverb_mode == 0) { // less lofi
            wet = lowpass.Process(samplerateReducer.Process(stft_out));
        } else if (reverb_mode == 1) { // normal
            wet = stft_out;
        } else if (reverb_mode == 2) { // more lofi
            wet = samplerateReducer.Process(stft_out);
        }

        float mix_val = vmix;
        // Mix dry/wet
        out_left[i] = wet * mix_val + in_left[i] * (1.0f - mix_val);
        // Corrected Stereo Mixing: Right Channel uses Right Dry Signal
        out_right[i] = wet * mix_val + in_right[i] * (1.0f - mix_val);
    }
}

// STFT Implementation
void VenusEffect::STFT_Write(float x) {
    for (size_t i = 0; i < VENUS_LAPS * 2; i++) {
        if (writing[i]) {
            if (writepoints[i] >= 0) {
                float amp = (*hann_window)((float)writepoints[i] / VENUS_N);
                in_buffer[writepoints[i] + VENUS_N * i] = amp * x;
            }
            writepoints[i]++;

            if (writepoints[i] == (int)VENUS_N) {
                writing[i] = false;
                reading[i] = true;
                readpoints[i] = 0;

                STFT_Forward(i);
                SpectralProcess(middle_buffer + i * VENUS_N, out_buffer + i * VENUS_N);
                STFT_Backward(i);

                current_lap = i;
            }
        }
    }
}

float VenusEffect::STFT_Read() {
    float accum = 0.0f;
    for (size_t i = 0; i < VENUS_LAPS * 2; i++) {
        if (reading[i]) {
            float amp = (*hann_window)((float)readpoints[i] / VENUS_N);
            accum += amp * in_buffer[readpoints[i] + VENUS_N * i]; // Note: `in_buffer` here is used as output buffer in `Fourier` impl?
            // In Fourier.h: `backward` calls `fft->Inverse(out..., in...)`.
            // So `in` buffer gets the IFFT result.
            // Yes.

            readpoints[i]++;

            if (readpoints[i] == (int)VENUS_N) {
                writing[i] = true;
                reading[i] = false;
                writepoints[i] = 0;
            }
        }
    }
    accum /= (float)VENUS_N * VENUS_LAPS / 2.0f;
    return accum;
}

void VenusEffect::STFT_Forward(size_t i) {
    fft->Direct((in_buffer + i * VENUS_N), (middle_buffer + i * VENUS_N));
}

void VenusEffect::STFT_Backward(size_t i) {
    fft->Inverse((out_buffer + i * VENUS_N), (in_buffer + i * VENUS_N));
}


// The Core Reverb / Shimmer Algorithm
void VenusEffect::SpectralProcess(const float* in, float* out) {
    static const size_t offset = VENUS_N / 2;
    float fft_size = VENUS_N / 2;
    float half_fft_size = fft_size / 2;

    for (size_t i = 0; i < VENUS_N / 2; i++) {
        float fft_bin = i + 1;

        float real = in[i];
        float imag = in[i + offset];

        float energy = real * real + imag * imag;

        // Amplitude from energy
        float reverb_amp = std::sqrt(reverb_energy[i]);

        // Damping
        // Uses `vdamp` (which might be modulated)
        // Original code: if (fft_bin / fft_size > vdamp) ...
        // Note: vdamp is 0.0-1.0.
        // If vdamp is low, high frequencies are attenuated more?
        // Condition: `freq_ratio > damp_thresh` -> attenuate.
        if (fft_bin / fft_size > vdamp) {
            reverb_amp *= vdamp * fft_size / fft_bin;
        }

        // Add random phase
        float random_phase = ((float)rand() / RAND_MAX) * 2.0f * PI;
        real = reverb_amp * std::cos(random_phase);
        imag = reverb_amp * std::sin(random_phase);

        if (!freeze_) {
            reverb_energy[i] += energy / VENUS_LAPS;

            float reverb_decay_factor = 1.0f / vdecay;
            reverb_energy[i] *= (1.0f - reverb_decay_factor);

            float current = reverb_energy[i];

            if (i > 0 && i < half_fft_size - 2) {
                // Morph reverb up by octaves
                if (shimmer_mode == 1 || shimmer_mode == 2) { // Up
                     reverb_energy[2*i - 1] += 0.123f * shimmer_double * current;
                     reverb_energy[2*i]     += 0.25f  * shimmer_double * current;
                     reverb_energy[2*i + 1] += 0.123f * shimmer_double * current;
                } else if ((shimmer_mode == 0 || shimmer_mode == 2) && i > 1 && !(i % 2)) { // Down
                     reverb_energy[i/2 - 1] += 0.75f * shimmer_double * current;
                     reverb_energy[i/2]     += 1.5f  * shimmer_double * current;
                     reverb_energy[i/2 + 1] += 0.75f * shimmer_double * current;
                }

                // Morph reverb by octave+5th
                if (3*i + 1 < half_fft_size) {
                    reverb_energy[3*i - 2] += 0.055f * shimmer_triple * current;
                    reverb_energy[3*i - 1] += 0.11f  * shimmer_triple * current;
                    reverb_energy[3*i]     += 0.17f  * shimmer_triple * current;
                    reverb_energy[3*i + 1] += 0.11f  * shimmer_triple * current;
                    reverb_energy[3*i + 2] += 0.105f * shimmer_triple * current;
                }

                // Detune
                if (i > 2 && i < half_fft_size - 2 && detune_mode != 1) {
                    int idx = i + (3 * detune_multiplier);
                    if (idx >= 0 && idx < VENUS_N/2) reverb_energy[idx] += 0.123f * detune_double * current;

                    idx = i + (2 * detune_multiplier);
                    if (idx >= 0 && idx < VENUS_N/2) reverb_energy[idx] += 0.25f * detune_double * current;

                    idx = i + (1 * detune_multiplier);
                    if (idx >= 0 && idx < VENUS_N/2) reverb_energy[idx] += 0.123f * detune_double * current;
                }
            }

            if (detune_mode == 1) detune_remainder = 1.0f;
            reverb_energy[i] = detune_remainder * shimmer_remainder * current;
        }

        out[i] = real;
        out[i + offset] = imag;
    }
}


// Parameter Setters
void VenusEffect::SetDecay(float value) { vdecay = value * 99.0f + 1.0f; }
void VenusEffect::SetMix(float value) { vmix = value; }
void VenusEffect::SetDamp(float value) { vdamp = value; }
void VenusEffect::SetShimmer(float value) { vshimmer = value; UpdateInternalParameters(); }
void VenusEffect::SetShimmerTone(float value) { vshimmer_tone = value; UpdateInternalParameters(); }

void VenusEffect::SetDetune(float value) {
    // Logic from ProcessControls
    float vdetune_temp = value;
    vdetune = std::abs(vdetune_temp);

    if (vdetune > 0.03f) {
        vdetune = vdetune - 0.029f;
        if (vdetune_temp >= 0) {
            detune_mode = 2; // up
            detune_multiplier = 1;
        } else {
            detune_mode = 0; // down
            detune_multiplier = -1;
        }
    } else {
        detune_mode = 1;
    }
    UpdateInternalParameters();
}

void VenusEffect::SetShimmerMode(int mode) { shimmer_mode = mode; }
void VenusEffect::SetReverbMode(int mode) {
    reverb_mode = mode;
    if (reverb_mode == 0) {
         samplerateReducer.SetFreq(0.3f);
         lowpass.SetFreq(8000.0f);
    } else if (reverb_mode == 2) {
         samplerateReducer.SetFreq(0.2f);
    }
}
void VenusEffect::SetDriftMode(int mode) {
    drift_mode = mode;
    if (drift_mode == 0) {
        drift_osc.SetFreq(0.009f); drift_osc.SetWaveform(SimpleOscillator::WAVE_SIN);
        drift_osc2.SetFreq(0.01f); drift_osc2.SetWaveform(SimpleOscillator::WAVE_SIN);
        drift_osc3.SetFreq(0.011f); drift_osc3.SetWaveform(SimpleOscillator::WAVE_SIN);
        drift_osc4.SetFreq(0.012f); drift_osc4.SetWaveform(SimpleOscillator::WAVE_SIN);
    } else if (drift_mode == 2) {
        drift_osc.SetFreq(0.020f); drift_osc.SetWaveform(SimpleOscillator::WAVE_TRI);
        drift_osc2.SetFreq(0.025f); drift_osc2.SetWaveform(SimpleOscillator::WAVE_TRI);
        drift_osc3.SetFreq(0.03f); drift_osc3.SetWaveform(SimpleOscillator::WAVE_TRI);
        drift_osc4.SetFreq(0.035f); drift_osc4.SetWaveform(SimpleOscillator::WAVE_TRI);
    }
}

void VenusEffect::SetFreeze(bool freeze) { freeze_ = freeze; }
void VenusEffect::SetBypass(bool bypass) { bypass_ = bypass; }
