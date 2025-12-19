#include <Arduino.h>
#include <I2S.h>
#include "mini_daisy.h"
#include "shy_fft.h"
#include "fourier.h"
#include "wave.h"

// --- Hardware & Configuration ---
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 256

// --- Memory Management ---
// Place large buffers in AXI SRAM (.ram_d1) to avoid DTCM overflow (128KB).
// The WeAct H7 linker script typically defines .ram_d1 for AXI SRAM.
// If not standard, .axi_sram or .ram_d1 is common in STM32Duino.
#define D1_RAM __attribute__((section(".ram_d1")))
#define DMA_BUFFER_ALIGN __attribute__((aligned(32)))

// FFT Parameters
#define FFT_ORDER 10
#define N (1 << FFT_ORDER) // 1024
#define LAPS 4
#define BUFFSIZE (2 * LAPS * N)
#define OFFSET (N / 2)

#define DELAY_BLOCKS 150
#define DELAY_BINS 120

using namespace soundmath;

// --- Global Objects (in AXI SRAM) ---

// FFT Buffers
D1_RAM float in_buf[BUFFSIZE];
D1_RAM float middle_buf[BUFFSIZE];
D1_RAM float out_buf[BUFFSIZE];

D1_RAM float in2_buf[BUFFSIZE];
D1_RAM float middle2_buf[BUFFSIZE];
D1_RAM float out2_buf[BUFFSIZE];

// FFT Objects
ShyFFT<float, N, RotationPhasor> fft;
Fourier<float, N>* stft;

ShyFFT<float, N, RotationPhasor> fft2;
Fourier<float, N>* stft2;

// Delay Lines
D1_RAM DelayLine<float, DELAY_BLOCKS> delayLine_real[DELAY_BINS];
D1_RAM DelayLine<float, DELAY_BLOCKS> delayLine_imag[DELAY_BINS];
D1_RAM DelayLine<float, DELAY_BLOCKS> delayLine_real2[DELAY_BINS];
D1_RAM DelayLine<float, DELAY_BLOCKS> delayLine_imag2[DELAY_BINS];

// Helper structs
struct BinDelay {
    DelayLine<float, DELAY_BLOCKS>* del;
    float currentDelay;
    float delayTarget;
    float feedback;

    float Process(float in) {
        fonepole(currentDelay, delayTarget, 0.0002f);
        del->SetDelay(currentDelay);
        float read = del->Read();
        del->Write((feedback * read) + in);
        return read;
    }
};

BinDelay bin_delays_real[DELAY_BINS];
BinDelay bin_delays_imag[DELAY_BINS];
BinDelay bin_delays_real2[DELAY_BINS];
BinDelay bin_delays_imag2[DELAY_BINS];

// --- Parameters ---
float p_mix = 0.5f;
float p_filter = 0.5f;
float p_time = 0.5f;
float p_feedback = 0.0f;
float p_mod = 0.0f;

// Serial Control Values (0-100)
int val_time = 50;
int val_feedback = 0;
int val_mix = 50;
int val_mod = 0;
int val_filter = 50;

// State
Oscillator drift_osc;
Wave<float> hann([] (float phase) -> float { return 0.5f * (1.0f - cosf(2.0f * PI * phase)); });

bool bypass = false;
int filter_mode = 2; // 0=LP, 1=HP, 2=OFF
int filter_bin = 0;

// --- Processing Functions ---

inline void spectraldelay_L(const float* in, float* out)
{
    for (size_t i = 0; i < N / 2; i++)
    {
        float real = 0.0f;
        float imag = 0.0f;

        if (i < DELAY_BINS && ((filter_mode == 0 && i < filter_bin) || (filter_mode == 1 && i > filter_bin) || filter_mode == 2))
        {
            real = in[i];
            imag = in[i + OFFSET];

            real = bin_delays_real[i].Process(real);
            imag = bin_delays_imag[i].Process(imag);
        }

        out[i] = real;
        out[i + OFFSET] = imag;
    }
}

inline void spectraldelay_R(const float* in, float* out)
{
    for (size_t i = 0; i < N / 2; i++)
    {
        float real = 0.0f;
        float imag = 0.0f;

        if (i < DELAY_BINS && ((filter_mode == 0 && i < filter_bin) || (filter_mode == 1 && i > filter_bin) || filter_mode == 2))
        {
            real = in[i];
            imag = in[i + OFFSET];

            real = bin_delays_real2[i].Process(real);
            imag = bin_delays_imag2[i].Process(imag);
        }

        out[i] = real;
        out[i + OFFSET] = imag;
    }
}

// --- Audio I/O ---
// Place buffers in standard memory (DTCM or RAM) but ensure cache maintenance.
// DMA buffers usually need alignment.
DMA_BUFFER_ALIGN int16_t tx_buffer[BUFFER_SIZE * 2]; // Stereo
DMA_BUFFER_ALIGN int16_t rx_buffer[BUFFER_SIZE * 2];

void i2s_callback()
{
    // 1. Invalidate Cache before reading from DMA-filled buffer (rx_buffer)
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rx_buffer, sizeof(rx_buffer));

    // 2. Read
    I2S.read(rx_buffer, sizeof(rx_buffer));

    // 3. Process
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        int16_t in_l_int = rx_buffer[i * 2];
        int16_t in_r_int = rx_buffer[i * 2 + 1];

        float in_l = (float)in_l_int / 32768.0f;
        float in_r = (float)in_r_int / 32768.0f;

        float out_l, out_r;

        if (bypass)
        {
            out_l = in_l;
            out_r = in_r;
        }
        else
        {
            float drift = drift_osc.Process() + 0.5f;

            stft->write(in_l);
            float processed_l = stft->read();

            stft2->write(in_r);
            float processed_r = stft2->read();

            float delaygain = 3.0f;
            out_l = processed_l * p_mix * delaygain + in_l * (1.0f - p_mix);
            out_r = processed_r * p_mix * delaygain + in_r * (1.0f - p_mix);
        }

        if (out_l > 1.0f) out_l = 1.0f; else if (out_l < -1.0f) out_l = -1.0f;
        if (out_r > 1.0f) out_r = 1.0f; else if (out_r < -1.0f) out_r = -1.0f;

        tx_buffer[i * 2] = (int16_t)(out_l * 32767.0f);
        tx_buffer[i * 2 + 1] = (int16_t)(out_r * 32767.0f);
    }

    // 4. Flush Cache before DMA writes to peripheral (tx_buffer)
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)tx_buffer, sizeof(tx_buffer));

    // 5. Write
    I2S.write(tx_buffer, sizeof(tx_buffer));
}

void update_parameters()
{
    p_time = (float)val_time / 100.0f;
    p_feedback = (float)val_feedback / 100.0f;
    p_mix = (float)val_mix / 100.0f;
    p_mod = (float)val_mod / 100.0f;
    p_filter = (float)val_filter / 100.0f;

    // Filter Logic
    if (p_filter < 0.45f) { // LP
        filter_mode = 0;
        filter_bin = floor(p_filter * 2.22f * (DELAY_BINS - 5) + 4);
    } else if (p_filter > 0.55f) { // HP
        filter_mode = 1;
        filter_bin = floor((p_filter - 0.55f) * 0.5f * DELAY_BINS);
    } else {
        filter_mode = 2;
    }

    // Delay Time Logic
    for(int i=0; i<DELAY_BINS; i++) {
        float d_time_samples = p_time * (float)DELAY_BLOCKS * (float)i / (float)DELAY_BINS;

        bin_delays_real[i].delayTarget = bin_delays_imag[i].delayTarget =
        bin_delays_real2[i].delayTarget = bin_delays_imag2[i].delayTarget = d_time_samples;

        bin_delays_real[i].feedback = bin_delays_imag[i].feedback =
        bin_delays_real2[i].feedback = bin_delays_imag2[i].feedback = p_feedback;
    }

    drift_osc.SetFreq(p_mod * 5.0f);
}

void parse_serial()
{
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) return;

        int spaceIdx = cmd.indexOf(' ');
        String command = spaceIdx == -1 ? cmd : cmd.substring(0, spaceIdx);
        String valStr = spaceIdx == -1 ? "" : cmd.substring(spaceIdx + 1);
        int val = valStr.toInt();

        if (command == "time") {
            val_time = constrain(val, 0, 100);
            Serial.printf("Time set to %d\n", val_time);
        } else if (command == "feedback") {
            val_feedback = constrain(val, 0, 100);
            Serial.printf("Feedback set to %d\n", val_feedback);
        } else if (command == "mix") {
            val_mix = constrain(val, 0, 100);
            Serial.printf("Mix set to %d\n", val_mix);
        } else if (command == "mod") {
            val_mod = constrain(val, 0, 100);
            Serial.printf("Mod set to %d\n", val_mod);
        } else if (command == "filter") {
            val_filter = constrain(val, 0, 100);
            Serial.printf("Filter set to %d\n", val_filter);
        } else if (command == "list") {
            Serial.println("--- Current Settings ---");
            Serial.printf("Time:     %d (%.2f)\n", val_time, (float)val_time/100.0);
            Serial.printf("Feedback: %d (%.2f)\n", val_feedback, (float)val_feedback/100.0);
            Serial.printf("Mix:      %d (%.2f)\n", val_mix, (float)val_mix/100.0);
            Serial.printf("Mod:      %d (%.2f)\n", val_mod, (float)val_mod/100.0);
            Serial.printf("Filter:   %d (%.2f)\n", val_filter, (float)val_filter/100.0);
        } else {
            Serial.println("Unknown command. Available: time, feedback, mix, mod, filter, list");
        }
        update_parameters();
    }
}

void check_ram()
{
    Serial.println("--- RAM Estimation ---");
    size_t delay_mem = 4 * DELAY_BINS * DELAY_BLOCKS * sizeof(float) * 4; // 4 arrays
    size_t fft_mem = 6 * BUFFSIZE * sizeof(float);
    Serial.printf("Delay Arrays: %d bytes\n", delay_mem);
    Serial.printf("FFT Buffers:  %d bytes\n", fft_mem);
    Serial.printf("Total Large Arrays: %d bytes\n", delay_mem + fft_mem);

    Serial.printf("Address of delayLine_real: %p\n", delayLine_real);
    Serial.printf("Address of rx_buffer: %p\n", rx_buffer);
}

void setup() {
    Serial.begin(115200);
    // Wait for Serial? No, proceed.

    check_ram();

    // Init FFT & STFT
    fft.Init();
    stft = new Fourier<float, N>(spectraldelay_L, &fft, &hann, LAPS, in_buf, middle_buf, out_buf);

    fft2.Init();
    stft2 = new Fourier<float, N>(spectraldelay_R, &fft2, &hann, LAPS, in2_buf, middle2_buf, out2_buf);

    // Init Delays
    for(int i=0; i<DELAY_BINS; i++) {
        delayLine_real[i].Init();
        bin_delays_real[i].del = &delayLine_real[i];
        bin_delays_real[i].delayTarget = 10.0f;
        bin_delays_real[i].feedback = 0.0f;

        delayLine_imag[i].Init();
        bin_delays_imag[i].del = &delayLine_imag[i];
        bin_delays_imag[i].delayTarget = 10.0f;
        bin_delays_imag[i].feedback = 0.0f;

        delayLine_real2[i].Init();
        bin_delays_real2[i].del = &delayLine_real2[i];
        bin_delays_real2[i].delayTarget = 10.0f;
        bin_delays_real2[i].feedback = 0.0f;

        delayLine_imag2[i].Init();
        bin_delays_imag2[i].del = &delayLine_imag2[i];
        bin_delays_imag2[i].delayTarget = 10.0f;
        bin_delays_imag2[i].feedback = 0.0f;
    }

    drift_osc.Init(SAMPLE_RATE);
    drift_osc.SetFreq(0.1f);
    drift_osc.SetAmp(0.5f);

    // I2S Setup
    // Pins: PB12(WS), PB13(CK), PB15(SD), PC2(DIN from PCM1808)
    I2S.setFS(PB12);
    I2S.setSCLK(PB13);
    I2S.setDataOutputPin(PB15);
    I2S.setDataInputPin(PC2);

    I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16);
}

void loop() {
    parse_serial();
    i2s_callback();
}
