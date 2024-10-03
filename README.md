# STM32G431 GPSDO

## Purpose

Others have built [a GPSDO on the STM32 platform](https://github.com/AndrewBCN/STM32-GPSDO), but generally use an external DAC with an STM32F411. I think the STM32G431 is the optimal chip for this use case, with its very capable timers and internal DAC.

This is still a very early draft. Right now it's a proof of concept - it counts a 10MHz clock with a 1PPS pin, but doesn't yet adjust. Therefore, there are no instructions on build or pinout until it's in a more final state.
