// pots.hpp -- ADS1115 ADC driver + front-panel pot scanning.
//
// Two ADS1115 chips share the I2C bus with the OLED:
//   #1 at 0x48 -- EQ pots (currently A3 -> presence)
//   #2 at 0x49 -- drive/warmth/vibe/volume (currently A3 -> master volume)
//
// pots_init() configures both ADCs for continuous A3 conversion and
// takes initial readings so the audio path starts at the wiper-correct
// values (no flash persistence; analog pots remember mechanically).
//
// pots_poll_tick() is called from a BTstack timer at 20 Hz; it reads
// both pots, EMA-smooths, maps to Q15, applies an audio-taper curve to
// volume, and pushes the values into the DSP module + the OLED popup.

#pragma once

void pots_init();
void pots_poll_tick();
