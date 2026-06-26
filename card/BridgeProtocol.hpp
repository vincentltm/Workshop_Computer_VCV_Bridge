/**
 * BridgeProtocol.hpp
 * Shared between the RP2040 firmware card and the VCV Rack plugin.
 * No platform-specific includes — header-only, pure C++ / C99.
 *
 * Signal rates:
 *   Audio 1 & 2  : 48 kHz, int16 per sample (full rate)
 *   CV 1 & 2     : ~750 Hz (once per block) — hardware lowpass filter makes this plenty
 *   Pulse 1 & 2  : 48 kHz, 1 bit per sample packed — preserves edge timing
 *   Knob M/X/Y   : ~750 Hz (once per block), uint16 0–4095
 *   Switch Z      : ~750 Hz (once per block), uint8 0=Down 1=Mid 2=Up
 */

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ── Protocol constants ────────────────────────────────────────────────────────

#define BRIDGE_BLOCK_SIZE   64          // samples per USB frame
#define BRIDGE_SAMPLE_RATE  48000       // RP2040 fixed sample rate (Hz)
#define BRIDGE_PULSE_BYTES  (BRIDGE_BLOCK_SIZE / 8)  // 8 bytes

// Sync words
#define BRIDGE_D2H_SYNC_0   0xBE       // Device → Host frame start
#define BRIDGE_D2H_SYNC_1   0xEF
#define BRIDGE_H2D_SYNC_0   0xDE       // Host → Device frame start
#define BRIDGE_H2D_SYNC_1   0xAD

// Frame sizes (for serialisation / buffer sizing)
#define BRIDGE_D2H_SIZE     289
#define BRIDGE_H2D_SIZE     282

// ── Cross-platform packed struct ──────────────────────────────────────────────

#ifdef _MSC_VER
  #pragma pack(push, 1)
  #define BRIDGE_PACKED
#else
  #define BRIDGE_PACKED __attribute__((packed))
#endif

// ── Device → Host (RP2040 → VCV Rack) ────────────────────────────────────────
// 289 bytes per block (750 blocks/sec = 217 KB/s)

struct BRIDGE_PACKED D2H_Frame {
    uint8_t  sync[2];                       // {0xBE, 0xEF}
    uint16_t block_idx;                     // rolling counter

    // Audio: full 48kHz, int16 (-2048..+2047)
    int16_t  audio1[BRIDGE_BLOCK_SIZE];     // 128 bytes
    int16_t  audio2[BRIDGE_BLOCK_SIZE];     // 128 bytes

    // CV: once per block (~750 Hz), int16 (-2048..+2047)
    int16_t  cv1;
    int16_t  cv2;

    // Pulse: 1 bit per sample, LSB = sample 0
    uint8_t  pulse1[BRIDGE_PULSE_BYTES];    // 8 bytes
    uint8_t  pulse2[BRIDGE_PULSE_BYTES];    // 8 bytes

    // Controls: once per block
    uint16_t knob_main;                     // 0–4095
    uint16_t knob_x;                        // 0–4095
    uint16_t knob_y;                        // 0–4095
    uint8_t  switch_z;                      // 0=Down 1=Mid 2=Up

    uint16_t crc;                           // CRC-16/CCITT over all bytes before crc
};
static_assert(sizeof(D2H_Frame) == BRIDGE_D2H_SIZE, "D2H_Frame layout error");

// ── Host → Device (VCV Rack → RP2040) ────────────────────────────────────────
// 282 bytes per block (750 blocks/sec = 212 KB/s)

struct BRIDGE_PACKED H2D_Frame {
    uint8_t  sync[2];                       // {0xDE, 0xAD}
    uint16_t block_idx;

    // Audio: full 48kHz, int16
    int16_t  audio1[BRIDGE_BLOCK_SIZE];     // 128 bytes
    int16_t  audio2[BRIDGE_BLOCK_SIZE];     // 128 bytes

    // CV: once per block
    int16_t  cv1;
    int16_t  cv2;

    // Pulse: 1 bit per sample
    uint8_t  pulse1[BRIDGE_PULSE_BYTES];    // 8 bytes
    uint8_t  pulse2[BRIDGE_PULSE_BYTES];    // 8 bytes

    uint16_t crc;
};
static_assert(sizeof(H2D_Frame) == BRIDGE_H2D_SIZE, "H2D_Frame layout error");

#ifdef _MSC_VER
  #pragma pack(pop)
#endif

// ── CRC-16/CCITT (polynomial 0x1021, init 0xFFFF) ────────────────────────────

inline uint16_t bridge_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

template<typename Frame>
inline uint16_t bridge_frame_crc(const Frame& f) {
    return bridge_crc16(
        reinterpret_cast<const uint8_t*>(&f),
        sizeof(f) - sizeof(uint16_t)   // exclude trailing crc field
    );
}

// ── Pulse bit helpers ─────────────────────────────────────────────────────────

inline void bridge_pulse_set(uint8_t* arr, int sample_idx, bool val) {
    uint8_t mask = (uint8_t)(1u << (sample_idx & 7));
    if (val) arr[sample_idx >> 3] |=  mask;
    else     arr[sample_idx >> 3] &= (uint8_t)(~mask);
}

inline bool bridge_pulse_get(const uint8_t* arr, int sample_idx) {
    return (arr[sample_idx >> 3] >> (sample_idx & 7)) & 1;
}

inline void bridge_pulse_clear(uint8_t* arr) {
    memset(arr, 0, BRIDGE_PULSE_BYTES);
}

// ── Voltage conversion helpers (VCV plugin side) ──────────────────────────────

// int16 (-2048..+2047) <-> VCV voltage (-6V..+6V)
inline float bridge_to_volts(int16_t v)  { return v * (6.0f / 2048.0f); }
inline int16_t bridge_from_volts(float v) {
    int32_t r = (int32_t)(v * (2048.0f / 6.0f));
    if (r < -2048) r = -2048;
    if (r >  2047) r =  2047;
    return (int16_t)r;
}

// Knob uint16 (0..4095) -> VCV voltage (0..10V)
inline float bridge_knob_to_volts(uint16_t k) { return k * (10.0f / 4095.0f); }

// Switch uint8 (0=Down 1=Mid 2=Up) -> VCV voltage (0 / 5 / 10 V)
inline float bridge_switch_to_volts(uint8_t s) { return s * 5.0f; }
