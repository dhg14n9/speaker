# Adaptive Resonance Speaker

Firmware for the Adaptive Resonance Speaker. It runs on an ESP32 and moves the tube to the resonant length for whatever frequency is playing, then uses the mic to fine-tune the position.

## Features
- Sine tone output, 60–1500 Hz (I2S)
- Stepper-driven tube length, 660–1100 mm, with limit-switch homing
- Resonant length calculated from frequency, harmonic (auto or manual) and air temperature
- Auto-scan ±40 mm around the target to find the loudest position (Goertzel filter on the mic)

## Libraries
- [U8g2](https://github.com/olikraus/u8g2)
- [FastAccelStepper](https://github.com/gin66/FastAccelStepper)
- ESP32 Arduino core 3.x

## Controls
Turn the encoder to change the selected field. Hold the button and release:

| Hold | Action |
|---|---|
| < 0.7 s | Next field |
| 0.7–2 s | Speaker on/off |
| 2–5 s | Fine-tune scan |
| > 5 s | Re-home |
