#include "MiniDaisy.h"

// Global instance
MiniDaisy hw;

// Buffers for I2S DMA
// Using I2S2 (TX) and I2S3 (RX)
// Audio Block Size = 48 frames (96 samples stereo)
// Buffer size = 2 * Block Size (Double Buffer) * 2 (Stereo) = 192 samples
#define I2S_BUFFER_SIZE (AUDIO_BLOCK_SIZE * 2 * 2)

// 32-byte alignment for Cache
int32_t __attribute__((section(".ram_d2"), aligned(32))) rx_buffer[I2S_BUFFER_SIZE];
int32_t __attribute__((section(".ram_d2"), aligned(32))) tx_buffer[I2S_BUFFER_SIZE];

float in_buffer[2][AUDIO_BLOCK_SIZE];
float out_buffer[2][AUDIO_BLOCK_SIZE];
float* in_ptrs[2] = {in_buffer[0], in_buffer[1]};
float* out_ptrs[2] = {out_buffer[0], out_buffer[1]};


// HAL Handles
I2S_HandleTypeDef hi2s2; // TX
I2S_HandleTypeDef hi2s3; // RX
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

MiniDaisy::MiniDaisy() {}

void MiniDaisy::Init(bool boost) {
    Serial.begin(115200);
}

float MiniDaisy::AudioSampleRate() {
    return SAMPLE_RATE;
}

void MiniDaisy::SetAudioBlockSize(size_t size) {}

void MiniDaisy::StartAdc() {}

void MiniDaisy::ProcessAnalogControls() {}

void MiniDaisy::ProcessDigitalControls() {}

void MiniDaisy::SetKnobValue(int index, float val) {
    if(index >= 0 && index < 6) {
        knob[index] = val;
    }
}

void MiniDaisy::SetSwitchState(int index, bool state) {
    if(index >= 0 && index < 12) {
        switches[index].UpdateState(state);
    }
}

// MIDI Implementation via Serial (USB CDC)
void MiniDaisy::InitMidi() {
    // Serial already started
}

void MiniDaisy::MidiHandler::StartReceive() {}

void MiniDaisy::MidiHandler::Listen() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        static int state = 0;
        static uint8_t status = 0;
        static uint8_t data1 = 0;

        if (byte & 0x80) {
            status = byte;
            state = 1;
        } else if (state == 1) {
            data1 = byte;
            if ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) {
                 state = 0;
            } else {
                 state = 2;
            }
        } else if (state == 2) {
            uint8_t data2 = byte;
            state = 0;

            int next_head = (head + 1) % QUEUE_SIZE;
            if(next_head != tail) {
                MidiEvent& e = queue[head];
                if ((status & 0xF0) == 0x90) {
                     if (data2 == 0) {
                        e.type = MidiEvent::NoteOff;
                        e.note_off = {data1, data2};
                     } else {
                        e.type = MidiEvent::NoteOn;
                        e.note_on = {data1, data2};
                     }
                     head = next_head;
                }
                else if ((status & 0xF0) == 0x80) {
                     e.type = MidiEvent::NoteOff;
                     e.note_off = {data1, data2};
                     head = next_head;
                }
                else if ((status & 0xF0) == 0xB0) {
                     e.type = MidiEvent::ControlChange;
                     e.control_change = {data1, data2};
                     head = next_head;
                }
            }
        }
    }
}

bool MiniDaisy::MidiHandler::HasEvents() {
    return head != tail;
}

MiniDaisy::MidiEvent MiniDaisy::MidiHandler::PopEvent() {
    MidiEvent e = queue[tail];
    tail = (tail + 1) % QUEUE_SIZE;
    return e;
}


// I2S DMA Callbacks
extern "C" void DMA1_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

extern "C" void DMA1_Stream1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

// We drive the loop with TX Half/Full Complete, assuming RX is in sync
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI2) {
        // Process first half (0 to 95)
        // We read from RX buffer (first half) and write to TX buffer (first half)

        SCB_InvalidateDCache_by_Addr((uint32_t*)&rx_buffer[0], (I2S_BUFFER_SIZE/2) * 4);

        for(int i=0; i<AUDIO_BLOCK_SIZE; i++) {
            in_buffer[0][i] = (float)rx_buffer[2*i] / 2147483648.0f;
            in_buffer[1][i] = (float)rx_buffer[2*i+1] / 2147483648.0f;
        }

        if(hw.callback_) {
            hw.callback_(in_ptrs, out_ptrs, AUDIO_BLOCK_SIZE);
        }

        for(int i=0; i<AUDIO_BLOCK_SIZE; i++) {
            tx_buffer[2*i]   = (int32_t)(out_buffer[0][i] * 2147483647.0f);
            tx_buffer[2*i+1] = (int32_t)(out_buffer[1][i] * 2147483647.0f);
        }

        SCB_CleanDCache_by_Addr((uint32_t*)&tx_buffer[0], (I2S_BUFFER_SIZE/2) * 4);
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI2) {
        // Process second half (96 to 191)
        int offset = AUDIO_BLOCK_SIZE * 2; // 96

        SCB_InvalidateDCache_by_Addr((uint32_t*)&rx_buffer[offset], (I2S_BUFFER_SIZE/2) * 4);

        for(int i=0; i<AUDIO_BLOCK_SIZE; i++) {
            in_buffer[0][i] = (float)rx_buffer[offset + 2*i] / 2147483648.0f;
            in_buffer[1][i] = (float)rx_buffer[offset + 2*i+1] / 2147483648.0f;
        }

        if(hw.callback_) {
            hw.callback_(in_ptrs, out_ptrs, AUDIO_BLOCK_SIZE);
        }

        for(int i=0; i<AUDIO_BLOCK_SIZE; i++) {
            tx_buffer[offset + 2*i]   = (int32_t)(out_buffer[0][i] * 2147483647.0f);
            tx_buffer[offset + 2*i+1] = (int32_t)(out_buffer[1][i] * 2147483647.0f);
        }

        SCB_CleanDCache_by_Addr((uint32_t*)&tx_buffer[offset], (I2S_BUFFER_SIZE/2) * 4);
    }
}

void DaisyAudioErrorHandler() {
    while(1);
}

void MiniDaisy::StartAudio(DaisyAudioCallback callback) {
    callback_ = callback;

    // Enable Clocks
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // I2S2 TX Pins (PB12=WS, PB13=CK, PB15=SDO)
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // I2S3 RX Pin (PC11=SDI) - Hardware change required! PC2 is not available for I2S3 RX.
    // Also we assume I2S3 clocks are wired externally to I2S2 clocks if using Slave Mode.
    // I2S3_WS (PA4/PA15) and I2S3_CK (PC10/PB3).
    // Let's assume user wires: PB12->PA4 (I2S3_WS), PB13->PC10 (I2S3_CK), ADC->PC11 (I2S3_SD)

    // Configure I2S3_SD (PC11)
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure I2S3_WS (PA4) and I2S3_CK (PC10) as Inputs (AF)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    // DMA Init
    // SPI3_RX on DMA1 Stream 0
    hdma_spi3_rx.Instance = DMA1_Stream0;
    hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_spi3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) DaisyAudioErrorHandler();
    __HAL_LINKDMA(&hi2s3, hdmarx, hdma_spi3_rx);

    // SPI2_TX on DMA1 Stream 1
    hdma_spi2_tx.Instance = DMA1_Stream1;
    hdma_spi2_tx.Init.Request = DMA_REQUEST_SPI2_TX;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) DaisyAudioErrorHandler();
    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_spi2_tx);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    // I2S2 Init (Master TX)
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
    hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
    hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
    hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK) DaisyAudioErrorHandler();

    // I2S3 Init (Slave RX)
    hi2s3.Instance = SPI3;
    hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_32B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.FirstBit = I2S_FIRSTBIT_MSB;
    hi2s3.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
    hi2s3.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
    hi2s3.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK) DaisyAudioErrorHandler();

    // Start I2S3 (RX) first to be ready
    if (HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)rx_buffer, I2S_BUFFER_SIZE) != HAL_OK) { // Pass full size 192
        DaisyAudioErrorHandler();
    }

    // Start I2S2 (TX)
    if (HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)tx_buffer, I2S_BUFFER_SIZE) != HAL_OK) {
        DaisyAudioErrorHandler();
    }
}
