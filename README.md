# Eris
A Dynamic Stochastic Synthesizer implemented in C++ for the Teensy Platform.

Eris features two GENDYN oscillators, a filter and a AR Envelope generator.

The second oscillator can be also used as an LFO for modulating the first one and/or to modulate the filter cutoff frequency.

The GENDYN algorithm (Iannis Xenakis) implemented here is inspired by the Supercollider version by Nick Collins, see:
https://github.com/supercollider/supercollider/blob/develop/server/plugins/GendynUGens.cpp

Note:
- in this version, only the amplitudes are modulated by the stochastic 
process, not the durations.

- The 5 distributions are interpolated by mean of the "Dist" parameter

# PCB
The code in this repository is designed to run on a Teensy 4.0 board, equipped with a DAC such as PCM5102 or the Teensy Audio Shield.
A schematic diagram is provided here, as well as gerber files and a BOM sheet for ordering a PCB.


![3drender](https://github.com/spareknobs/Eris/blob/main/pcb/Eris_v_1_1_pcb_render_3d.png)
