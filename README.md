# Workshop Computer VCV Bridge

Bidirectional USB bridge between the [Music Thing Modular Workshop Computer](https://www.musicthing.co.uk/workshopsystem/) and [VCV Rack](https://vcvrack.com/).

Streams **audio, CV and gates** in both directions over USB CDC Serial — no USB Audio Class, no driver installs, works cross-platform.

---

## What's in this repo

```
card/        RP2040 firmware (Pico SDK + TinyUSB)
plugin/      VCV Rack plugin (standalone, separate from MTMWorkshopComputer)
```

---

## Signals

### From Hardware → VCV (outputs on the plugin module)

| Port | Signal | Rate |
|---|---|---|
| Audio 1 / 2 | I2S ADC at full quality | 48 kHz, int16 |
| CV 1 / 2 | DC-coupled, hardware lowpass filtered | ~750 Hz |
| Pulse 1 / 2 | Gate / trigger | 48 kHz bit-accurate |
| Knob Main / X / Y | Knob position as CV (0–10 V) | ~750 Hz |
| Switch | Switch position (0 / 5 / 10 V) | ~750 Hz |

### VCV → Hardware (inputs on the plugin module)

| Port | Destination |
|---|---|
| Audio 1 / 2 | I2S DAC |
| CV 1 / 2 | 12-bit PWM DAC |
| Pulse 1 / 2 | Digital output |

---

## Knob → VCV Parameter Mapping

Right-click the **VCV Bridge** module in Rack → select *"Map Knob [Main/X/Y] to parameter"* → click any VCV parameter to bind it. The physical knob then directly drives that parameter. Mappings are saved with the patch.

---

## Transport

- **USB CDC ACM** (virtual COM port) — appears automatically on all platforms:
  - macOS: `/dev/cu.usbmodem*`
  - Linux: `/dev/ttyACM0` (or similar)
  - Windows: `COMx` (no driver needed on Windows 10+)
- **Bandwidth**: ~430 KB/s total (less than half USB Full Speed capacity)
- **Block size**: 64 samples = 1.33 ms latency blocks
- **Round-trip latency**: ~5–6 ms (USB + block buffering)

---

## Building the Firmware

```bash
cd card
mkdir build && cd build
cmake .. -DCOMPUTERCARD_DIR=/path/to/ComputerCard
make -j4
# Flash vcv_bridge.uf2 to the Workshop Computer
```

Requires: [Pico SDK](https://github.com/raspberrypi/pico-sdk), [ComputerCard](https://github.com/TomWhitwell/ComputerCard)

## Building the VCV Plugin

```bash
cd plugin
# Set RACK_DIR to your VCV Rack SDK
make RACK_DIR=/path/to/Rack-SDK
make install-dev   # installs directly to VCV Rack plugins folder (macOS arm64)
```

---

## Protocol

See [`card/BridgeProtocol.hpp`](card/BridgeProtocol.hpp) — shared between firmware and plugin.

Frame format (per 64-sample block):

| Field | Bytes | Rate |
|---|---|---|
| Sync + block index | 4 | — |
| Audio 1 & 2 (int16 × 64) | 256 | 48 kHz |
| CV 1 & 2 (int16) | 4 | ~750 Hz |
| Pulse 1 & 2 (bit-packed) | 16 | 48 kHz |
| Knobs + Switch | 7 | ~750 Hz |
| CRC-16 | 2 | — |

---

## License

MIT
