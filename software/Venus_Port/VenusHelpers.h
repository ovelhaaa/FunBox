#ifndef VENUS_HELPERS_H
#define VENUS_HELPERS_H

#include <cmath>
#include <cstdlib>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Simple Naive Oscillator for LFO use
class SimpleOscillator {
public:
    enum Waveform {
        WAVE_SIN = 0,
        WAVE_TRI = 1,
        WAVE_SAW = 2,
        WAVE_RAMP = 3,
        WAVE_SQUARE = 4
    };

    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        phase_ = 0.0f;
        freq_ = 1.0f;
        amp_ = 1.0f;
        waveform_ = WAVE_SIN;
    }

    void SetFreq(float freq) {
        freq_ = freq;
    }

    void SetAmp(float amp) {
        amp_ = amp;
    }

    void SetWaveform(int waveform) {
        waveform_ = (Waveform)waveform;
    }

    float Process() {
        float out = 0.0f;
        float inc = freq_ / sample_rate_;
        phase_ += inc;
        if (phase_ > 1.0f) phase_ -= 1.0f;

        switch (waveform_) {
            case WAVE_SIN:
                out = sinf(phase_ * 2.0f * PI);
                break;
            case WAVE_TRI:
                out = phase_ < 0.5f ? (4.0f * phase_ - 1.0f) : (3.0f - 4.0f * phase_);
                break;
            case WAVE_SAW:
                out = 2.0f * phase_ - 1.0f;
                break;
            case WAVE_RAMP:
                out = 1.0f - 2.0f * phase_;
                break;
            case WAVE_SQUARE:
                out = phase_ < 0.5f ? 1.0f : -1.0f;
                break;
            default:
                out = 0.0f;
                break;
        }
        return out * amp_;
    }

private:
    float sample_rate_;
    float phase_;
    float freq_;
    float amp_;
    Waveform waveform_;
};

// Simple One-Pole Low Pass Filter (Tone)
class SimpleLowPass {
public:
    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        out_ = 0.0f;
        SetFreq(1000.0f);
    }

    void SetFreq(float freq) {
        // Simple coefficient calculation for one-pole lowpass
        // y[n] = y[n-1] + a * (x[n] - y[n-1])
        // a = 1 - e^(-2*pi*f/sr)  ~= 2*pi*f/sr for low freqs
        // Using the bilinear transform approximation or similar for stability usually better,
        // but for "Tone" control simple RC approx is often used.
        // DaisySP Tone uses: val = ((in - val) * coef) + val; where coef = (2.0 * PI * freq) / sr

        float c = (2.0f * PI * freq) / sample_rate_;
        if (c > 1.0f) c = 1.0f;
        coeff_ = c;
    }

    float Process(float in) {
        out_ += coeff_ * (in - out_);
        return out_;
    }

private:
    float sample_rate_;
    float coeff_;
    float out_;
};

// Simple Sample Rate Reducer (Decimator)
class SimpleSampleRateReducer {
public:
    void Init() {
        phase_ = 0.0f;
        frequency_ = 0.1f; // Default normalized frequency
        last_val_ = 0.0f;
    }

    // Freq is normalized (0.0 to 1.0) relative to Nyquist or SampleRate?
    // DaisySP SampleRateReducer SetFreq takes range 0.0 to 1.0 (ratio of sample rate).
    void SetFreq(float freq) {
        frequency_ = freq;
    }

    float Process(float in) {
        phase_ += frequency_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
            last_val_ = in;
        }
        return last_val_;
    }

private:
    float phase_;
    float frequency_;
    float last_val_;
};

#endif // VENUS_HELPERS_H
