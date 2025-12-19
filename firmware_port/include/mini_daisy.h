#ifndef MINI_DAISY_H
#define MINI_DAISY_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define PI 3.1415926535897932384626433832795f
#define TWO_PI 6.283185307179586476925286766559f

// --- Utility Functions ---

inline void fonepole(float &out, float target, float coeff)
{
    out += coeff * (target - out);
}

inline float mtof(float m)
{
    return 440.0f * powf(2.0f, (m - 69.0f) / 12.0f);
}

// --- DelayLine Class ---

template <typename T, size_t MAX_DELAY>
class DelayLine
{
public:
    DelayLine() {}
    ~DelayLine() {}

    void Init()
    {
        Reset();
    }

    void Reset()
    {
        memset(buffer_, 0, sizeof(T) * MAX_DELAY);
        write_ptr_ = 0;
        delay_ = 0.0f;
    }

    void SetDelay(float delay)
    {
        if (delay < 0.0f) delay = 0.0f;
        if (delay >= MAX_DELAY - 1) delay = MAX_DELAY - 1.0f;
        delay_ = delay;
    }

    void Write(const T val)
    {
        buffer_[write_ptr_] = val;
        write_ptr_++;
        if (write_ptr_ >= MAX_DELAY)
        {
            write_ptr_ = 0;
        }
    }

    T Read() const
    {
        // Linear interpolation
        float read_pos = (float)write_ptr_ - delay_;
        while (read_pos < 0.0f) read_pos += MAX_DELAY;
        while (read_pos >= MAX_DELAY) read_pos -= MAX_DELAY;

        int32_t idx_a = (int32_t)read_pos;
        int32_t idx_b = idx_a + 1;
        if (idx_b >= MAX_DELAY) idx_b = 0;

        float frac = read_pos - idx_a;
        return buffer_[idx_a] + frac * (buffer_[idx_b] - buffer_[idx_a]);
    }

    // For simple non-interpolated read if needed, or Hermite if quality required.
    // Linear is usually fine for this spectral delay application.

private:
    T buffer_[MAX_DELAY];
    size_t write_ptr_;
    float delay_;
};

// --- Oscillator Class ---

class Oscillator
{
public:
    enum Waveforms {
        WAVE_SIN,
        WAVE_TRI,
        WAVE_SAW,
        WAVE_RAMP,
        WAVE_SQUARE,
        WAVE_POLYBLEP_TRI,
        WAVE_POLYBLEP_SAW,
        WAVE_POLYBLEP_SQUARE
    };

    Oscillator() {}
    ~Oscillator() {}

    void Init(float sample_rate)
    {
        sr_ = sample_rate;
        sr_recip_ = 1.0f / sample_rate;
        freq_ = 100.0f;
        amp_ = 0.5f;
        phase_ = 0.0f;
        wave_ = WAVE_SIN;
    }

    void SetFreq(float freq)
    {
        freq_ = freq;
        inc_ = freq_ * sr_recip_;
    }

    void SetAmp(float amp)
    {
        amp_ = amp;
    }

    void SetWaveform(int waveform)
    {
        wave_ = waveform;
    }

    float Process()
    {
        float out = 0.0f;
        switch (wave_)
        {
        case WAVE_SIN:
            out = sinf(phase_ * TWO_PI);
            break;
        case WAVE_TRI:
            out = -1.0f + (2.0f * phase_);
            out = 2.0f * (fabsf(out) - 0.5f);
            break;
        case WAVE_SAW:
            out = -1.0f + (2.0f * phase_);
            break;
        case WAVE_RAMP:
             out = 1.0f - (2.0f * phase_);
             break;
        case WAVE_SQUARE:
            out = phase_ < 0.5f ? 1.0f : -1.0f;
            break;
        default:
            out = 0.0f;
            break;
        }

        phase_ += inc_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;

        return out * amp_;
    }

private:
    float sr_, sr_recip_;
    float freq_, amp_, phase_, inc_;
    int wave_;
};

#endif
