# EspMount
This project's goal is to create a controller for two stepper-motor mount (one for each axis).

The controller communicates with the pc through UART, so in theory almost any programming
language can be used to communicate with it. Currently the easiest way to control it
is using `esp-mount-ctrl` library for python (available [here](https://github.com/tomvitek/esp-mount-ctrl)).

## Project goals
- simple communication protocol
- easy to deploy
- allow for fast slew speeds and precise tracking (for satellites)

## Status
While the current version is capable of tracking satellites (at least at radio precision),
it is still far from complete, so you can encounter some errors.

## Installation
1) Clone the repo to your pc
2) Update pin configuration according to your build. Currently it is a bit spread-out, uart pins are defined in `main/comm/comm-task.c`, motor pins in `main/motors/motor-task.h`, led pin in `main.c`. The steps per axis revolution are in `main/config.h`. (update to this chaos hopefully coming soon)
3) Flash the code onto your `espressif32` device (guide from the official docs: [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#introduction))
   1) In vs-code, install ESP-IDF plugin
   2) Perform initial configuration and installation, leave settings at default
   3) Connect your ESP to the computer and select correct port in bottom-left corner of vs-code window
   4) Flash it using the small lightning symbol, or with shortcut `CTRL+E CTRL+D` (press one, than the other)
4) Have fun :)