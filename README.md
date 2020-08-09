# ghetto-ballistic-chrono
A MCU-based low-cost minimalist implementation of ballistic chrono

Details here: https://dannyelectronics.wordpress.com/2020/08/05/ghetto-ballistic-chrono-proof-of-concept/

Design goal: to produce a minimalist ballistic chrono.
Key features:
1. Minimal part count.
2. Support a variety of MCUs and LED display types: 3-digit or 4-digit, common cathode or common anode
3. Flexible wiring connection: user configuration

Currently the code supports the following devices:
1. PIC16F19xx and compatible devices (including PIC16F88x, PIC18Fxxk20 and PIC18Fxxk22)
2. Arduino / ATMega328 (and compatible devices)
3. ATtiny2313
3. STM32F103 / BluePill
