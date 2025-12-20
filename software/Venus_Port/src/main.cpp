#include <Arduino.h>
#include <I2S.h>

// Use built-in MIDIUSB if available for this core/configuration
#include <MIDIUSB.h>

// Include Venus headers
#include "VenusEffect.h"
#include "VenusMidi.h"

// --- Hardware Configuration ---
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 256

// --- Memory Management ---
// Place large buffers in AXI SRAM (.ram_d1 or .ram_d2) if needed
// VenusEffect allocates large arrays on the heap (new float[]).

// DMA Buffers for I2S must be in D2 or D3 and aligned
#define DMA_BUFFER_ALIGN __attribute__((aligned(32)))
#define D2_RAM __attribute__((section(".ram_d2")))

DMA_BUFFER_ALIGN D2_RAM int16_t rx_buffer[BUFFER_SIZE * 2];
DMA_BUFFER_ALIGN D2_RAM int16_t tx_buffer[BUFFER_SIZE * 2];

// --- Objects ---
VenusEffect effect;
VenusMidi midiHandler(&effect);

// Processing Buffers (static to avoid stack overflow)
static float in_l[BUFFER_SIZE];
static float in_r[BUFFER_SIZE];
static float out_l[BUFFER_SIZE];
static float out_r[BUFFER_SIZE];

void audio_callback() {
    // 1. Invalidate Cache (RX Buffer)
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rx_buffer, sizeof(rx_buffer));

    // 2. Read from I2S (DMA)
    // In blocking mode, this waits for the buffer to be filled.
    I2S.read(rx_buffer, sizeof(rx_buffer));

    // 3. Process
    // Convert int16 -> float
    for (int i = 0; i < BUFFER_SIZE; i++) {
        in_l[i] = (float)rx_buffer[i * 2] / 32768.0f;
        in_r[i] = (float)rx_buffer[i * 2 + 1] / 32768.0f;
    }

    // Effect Processing
    effect.Process(in_l, in_r, out_l, out_r, BUFFER_SIZE);

    // Convert float -> int16
    for (int i = 0; i < BUFFER_SIZE; i++) {
        // Simple clip
        if (out_l[i] > 1.0f) out_l[i] = 1.0f; else if (out_l[i] < -1.0f) out_l[i] = -1.0f;
        if (out_r[i] > 1.0f) out_r[i] = 1.0f; else if (out_r[i] < -1.0f) out_r[i] = -1.0f;

        tx_buffer[i * 2] = (int16_t)(out_l[i] * 32767.0f);
        tx_buffer[i * 2 + 1] = (int16_t)(out_r[i] * 32767.0f);
    }

    // 4. Clean Cache (TX Buffer)
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)tx_buffer, sizeof(tx_buffer));

    // 5. Write to I2S (DMA)
    I2S.write(tx_buffer, sizeof(tx_buffer));
}

void setup() {
    // Initialize Effect
    effect.Init(SAMPLE_RATE);

    // I2S Setup
    // Pins: PB12(WS), PB13(CK), PB15(SD), PC2(DIN from PCM1808)
    I2S.setFS(PB12);
    I2S.setSCLK(PB13);
    I2S.setDataOutputPin(PB15);
    I2S.setDataInputPin(PC2);

    I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16);
}

void loop() {
    // 1. Handle Audio
    audio_callback();

    // 2. Handle MIDI
    // Poll USB MIDI
    midiEventPacket_t rx;
    do {
        rx = MidiUSB.read();
        if (rx.header != 0) {
            // Check for Control Change (0xB0 for channel 1)
            // Allow any channel for now, or check channel 1 (0xB0 - 0xBF)
            if ((rx.byte1 & 0xF0) == 0xB0) {
                uint8_t channel = rx.byte1 & 0x0F;
                uint8_t cc = rx.byte2;
                uint8_t value = rx.byte3;
                midiHandler.HandleControlChange(channel, cc, value);
            }
        }
    } while (rx.header != 0);
}
