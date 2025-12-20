# Venus Effect Port (STM32H7)

This directory contains the "Venus" Spectral Reverb effect ported to the STM32H7 platform (specifically the WeAct MiniSTM32H743VIT6 board).

## Overview

The Venus effect creates shimmery, ethereal soundscapes using a Fast Fourier Transform (FFT) based spectral processing engine. It supports various parameters like Decay, Mix, Damp, Shimmer, and Detune, controllable via a WebMIDI interface.

## Hardware Requirements

*   **Microcontroller:** WeAct MiniSTM32H743VIT6 (STM32H743VIT6)
*   **Audio ADC:** PCM1808 (or compatible I2S ADC)
*   **Audio DAC:** PCM5102 (or compatible I2S DAC)
*   **USB Connection:** USB-C cable for power, programming, and MIDI control.

### Pinout Configuration

The firmware is configured for the following I2S pin mapping (Standard I2S2):

*   **WS (Word Select / LRCLK):** PB12
*   **CK (Bit Clock / BCLK):** PB13
*   **SD (Data Output / DOUT):** PB15
*   **DIN (Data Input):** PC2

Ensure your audio codec hardware is wired to these pins.

## Software Prerequisites

To build and upload this project, you need:

1.  **Visual Studio Code (VSCode):** [Download Here](https://code.visualstudio.com/)
2.  **PlatformIO IDE Extension:** Install this extension from the VSCode Marketplace.

## Setup & Build Instructions

1.  **Open the Project:**
    *   Open VSCode.
    *   Click on the PlatformIO Alien icon in the sidebar.
    *   Select "Open Project" and navigate to this folder: `software/Venus_Port`.

2.  **Initialize Dependencies:**
    *   PlatformIO should automatically detect the `platformio.ini` file and start downloading the necessary toolchains (STM32 platform, Arduino framework) and libraries (STM32I2S).
    *   Wait for the initialization to complete.

3.  **Build the Firmware:**
    *   Click the **Build** icon (checkmark `✓`) in the bottom status bar of VSCode.
    *   Alternatively, run the command `pio run` in the VSCode terminal.
    *   Verify that the build completes successfully with a `[SUCCESS]` message.

## Upload Instructions

### Method 1: DFU Mode (Recommended)

The WeAct MiniSTM32H7 board has a built-in USB bootloader (DFU).

1.  **Enter DFU Mode:**
    *   Connect the board to your computer via USB.
    *   Press and hold the **BOOT0** button.
    *   Press and release the **NRST** (Reset) button.
    *   Release the **BOOT0** button.
    *   The board should now be in DFU mode.

2.  **Upload:**
    *   Click the **Upload** icon (arrow `→`) in the PlatformIO status bar.
    *   PlatformIO will detect the DFU device and upload the firmware.

### Method 2: ST-Link

If you have an ST-Link V2 or V3 programmer:

1.  Connect the ST-Link to the SWD pins (GND, 3.3V, SWCLK, SWDIO) on the board.
2.  Change the `upload_protocol` in `platformio.ini` to `stlink` (optional, usually auto-detected or specified in CLI).
3.  Click **Upload**.

## Web Interface & MIDI Control

This firmware implements a USB MIDI Class device. It is designed to be controlled via the included Web Interface.

1.  **Connect:** Ensure the board is connected via USB and running the Venus firmware.
2.  **Open Interface:** Open the `web_interface/index.html` file in a WebMIDI-supported browser (Chrome, Edge, Opera).
3.  **Select Device:** In the web interface, look for a "STM32" or "Venus" MIDI device in the dropdown menu.
4.  **Control:** Use the on-screen sliders and buttons to control the effect parameters in real-time.

## Project Structure

*   `src/`: Source files (`.cpp`).
    *   `main.cpp`: Main entry point, hardware setup, audio callback, and MIDI handling.
    *   `VenusEffect.cpp`: Core DSP logic for the spectral reverb.
*   `include/`: Header files (`.h`).
*   `platformio.ini`: PlatformIO project configuration file.
*   `web_interface/`: HTML/JS controller for the effect.
