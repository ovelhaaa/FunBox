#ifndef VENUS_MIDI_H
#define VENUS_MIDI_H

#include "VenusEffect.h"
#include <cstdint>

class VenusMidi {
public:
    VenusMidi(VenusEffect* effect) : effect_(effect) {}

    void HandleControlChange(uint8_t channel, uint8_t cc, uint8_t value) {
        if (!effect_) return;

        float norm_val = (float)value / 127.0f;

        switch (cc) {
            case 14: // Decay
                effect_->SetDecay(norm_val);
                break;
            case 15: // Mix
                effect_->SetMix(norm_val);
                break;
            case 16: // Damp
                effect_->SetDamp(norm_val);
                break;
            case 17: // Shimmer
                effect_->SetShimmer(norm_val); // Original maps this * 0.1 internally in knob logic? Check logic.
                // The class VenusEffect expects 0.0-1.0 and scales internally if needed?
                // Checking VenusEffect.cpp: SetShimmer calls UpdateInternalParameters directly.
                // Original `venus.cpp` did: `vshimmer = newExpressionValues[3];` where `newExpressionValues` came from `knobValues` which was `shimmer.Process()`.
                // `shimmer.Init(..., 0.0f, 0.1f, ...)`
                // So original range was 0.0 to 0.1.
                // My VenusEffect::SetShimmer takes `value`.
                // If I pass 0.0-1.0 here, `vshimmer` becomes 0-1.
                // In `UpdateInternalParameters`: `pow(8.0f, vshimmer) - 1.0f`.
                // If vshimmer is 1.0, rate is 7.0. If vshimmer is 0.1, rate is small.
                // I should probably preserve the original range scaling here or in VenusEffect.
                // VenusEffect.cpp SetShimmer just assigns `vshimmer = value`.
                // So I should scale it here to match original knob range if I want faithful sound.
                // Original: `shimmer.Init(..., 0.0f, 0.1f, ...)`
                // So I will scale input 0-1 to 0-0.1.
                effect_->SetShimmer(norm_val * 0.1f);
                break;
            case 18: // Shimmer Tone
                // Original: `shimmer_tone.Init(..., 0.0f, 0.3f, ...)`
                effect_->SetShimmerTone(norm_val * 0.3f);
                break;
            case 19: // Detune
                // Original: `detune.Init(..., -.15f, 0.15f, ...)`
                // My VenusEffect::SetDetune handles the sign logic if I pass the raw range?
                // No, SetDetune in my port takes `value` and does `abs(value)`.
                // It expects the value to be in the "knob range" (e.g. -0.15 to 0.15).
                // So I must map MIDI 0-127 to -0.15 to 0.15.
                {
                    float mapped = (norm_val * 0.3f) - 0.15f;
                    effect_->SetDetune(mapped);
                }
                break;
            case 20: // Shimmer Mode
                // 3-way switch
                if (value < 43) effect_->SetShimmerMode(0);
                else if (value < 85) effect_->SetShimmerMode(1);
                else effect_->SetShimmerMode(2);
                break;
            case 21: // Reverb Mode
                if (value < 43) effect_->SetReverbMode(0);
                else if (value < 85) effect_->SetReverbMode(1);
                else effect_->SetReverbMode(2);
                break;
            case 22: // Drift Mode
                if (value < 43) effect_->SetDriftMode(0);
                else if (value < 85) effect_->SetDriftMode(1);
                else effect_->SetDriftMode(2);
                break;
            case 23: // Freeze
                effect_->SetFreeze(value > 64);
                break;
            case 24: // Bypass
                effect_->SetBypass(value > 64);
                break;
            default:
                break;
        }
    }

private:
    VenusEffect* effect_;
};

#endif // VENUS_MIDI_H
