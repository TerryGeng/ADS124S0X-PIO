# ADS124S0X PIO Driver on RP2040

## Introduction

This is a RP2040 driver for TI ADS124S0X series ADCs. It features a PIO-based
SPI state machine that reads the samples of multiple channels in a completely
unattended way, fully leveraging the DMA.

To be more specific:
 - The state machine is fed with set of instructions, including a set of
   channel settings, by the DMA.
 - It writes the channel settings, waits for the DRDY pin to be driven high,
   then performs a read operation, before it moves on the next channel.

To re-trigger the transfer of the instructions to the state machine, the
transfer is supervised by three DMA channels:
 - `tx_ctrl1`: Reset `tx_ctrl2` and trigger it.
 - `tx_ctrl2`: Program `tx` and trigger it.
 - `tx`: The channel that actually writes instructions to the state machine.

To help user read out the samples, two receiving channels `rx1` and `rx2` are
chained together and write to two different sample buffers. The user can read
out one buffer while the working RX channel is writing to the other buffer.

## Board configuration

The default wiring is
 - `SCK`: GPIO 2,
 - `TX`: GPIO 3,
 - `RX`: GPIO 4,
 - `CS`: GPIO 5,
 - `DRDY`: GPIO 6.

By default, this program will read channel 0 and 1, 2 and 3, 4 and 5,
differentially in a cyclic manner.

## Build

Please refer to the documentation of Pico SDK.

 1. Set up Pico SDK first (the environment variable `PICO_SDK_PATH`).
 2. `cmake -B build/`
 3. `cd build`
 4. `make && pyocd flash --target rp2040 pio_adc.elf`, or drag and drop the uf2
 file.

## Usage

This program implements a simple shell interface. A list of usable command:
 - `start`: Start acquisition.
 - `stop`: Stop acquisition.
 - `set_slow`: Set data rate to 4 (20 SPS).
 - `set_fast`: Set data rate to 11 (1000 SPS).
 - `set_verbose`: Set verbose output mode (show conversion to float).
 - `unset_verbose`: Unset verbose output mode (output samples in integer).

For characterizing performance, a logger script `logger.py` is included:
```
usage: logger.py [-h] [-f] COM LEN OUTPUT

Serial logger for pio_adc program

positional arguments:
  COM         COM Port connecting to the MCU.
  LEN         Number of measurements to log
  OUTPUT      Output file name

options:
  -h, --help  show this help message and exit
  -f, --fast  Fast sampling mode (1 kHz)
```

