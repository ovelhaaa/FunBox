#ifndef MINI_DAISY_H
#define MINI_DAISY_H

#include <Arduino.h>
#include <stm32h7xx_hal.h>
#include "daisysp.h"
#include "funbox.h" // Use the existing header

// Define Audio Block Size
#define AUDIO_BLOCK_SIZE 48
#define SAMPLE_RATE 48000.0f

// Emulate Daisy Parameter class
class Parameter {
public:
    enum Curve { LINEAR, LOGARITHMIC, EXPONENTIAL };
    void Init(float& knob_input, float min, float max, Curve curve) {
        val_ = &knob_input;
        min_ = min;
        max_ = max;
        curve_ = curve;
    }
    float Process() {
        return min_ + (*val_ * (max_ - min_));
    }
private:
    float* val_;
    float min_, max_;
    Curve curve_;
};

// Emulate Daisy Switch class
class Switch {
public:
    void Init() { state_ = false; last_state_ = false; time_held_ = 0; }
    void Debounce() {}
    bool Pressed() { return state_; }
    bool RisingEdge() { return state_ && !last_state_; }
    bool FallingEdge() { return !state_ && last_state_; }
    float TimeHeldMs() { return state_ ? (millis() - time_held_) : 0; }

    void UpdateState(bool new_state) {
        last_state_ = state_;
        state_ = new_state;
        if (RisingEdge()) time_held_ = millis();
    }
private:
    bool state_;
    bool last_state_;
    unsigned long time_held_;
};

// Audio Handle for Callback
class AudioHandle {
public:
    typedef float** InputBuffer;
    typedef float** OutputBuffer;
};

// Renamed typedef to avoid collision with function name in main.cpp
typedef void (*DaisyAudioCallback)(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

// DaisyPetal / MiniDaisy Class
class MiniDaisy {
public:
    MiniDaisy();
    void Init(bool boost = true);
    float AudioSampleRate();
    void SetAudioBlockSize(size_t size);
    void StartAdc(); // Mocked
    void StartAudio(DaisyAudioCallback callback);
    void ProcessAnalogControls();
    void ProcessDigitalControls();

    // Controls
    float knob[6];
    Switch switches[12];
    // LED emulation
    struct Led {
        void Init(int pin, bool invert) {}
        void Set(float brightness) { val = brightness; }
        void Update() {}
        float val;
    } seed;

    struct MockPin { int pin; };
    MockPin GetPin(int p) { return {p}; }

    // MIDI
    struct MidiEvent {
        enum Type { NoteOn, NoteOff, ControlChange };
        Type type;
        union {
            struct { int note; int velocity; } note_on;
            struct { int note; int velocity; } note_off;
            struct { int control_number; int value; } control_change;
        };
        struct NoteOnEvent { int note; int velocity; };
        struct NoteOffEvent { int note; int velocity; };
        struct ControlChangeEvent { int control_number; int value; };

        NoteOnEvent AsNoteOn() { return {note_on.note, note_on.velocity}; }
        NoteOffEvent AsNoteOff() { return {note_off.note, note_off.velocity}; }
        ControlChangeEvent AsControlChange() { return {control_change.control_number, control_change.value}; }
    };

    class MidiHandler {
    public:
        void StartReceive();
        void Listen();
        bool HasEvents();
        MidiEvent PopEvent();
    private:
        static const int QUEUE_SIZE = 16;
        MidiEvent queue[QUEUE_SIZE];
        int head = 0;
        int tail = 0;
        friend class MiniDaisy;
    };
    MidiHandler midi;
    void InitMidi();

    void SetKnobValue(int index, float val);
    void SetSwitchState(int index, bool state);

private:
    DaisyAudioCallback callback_;
    friend void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
    friend void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
};

extern MiniDaisy hw;

#endif
