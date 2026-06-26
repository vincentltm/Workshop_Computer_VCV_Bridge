/**
 * BridgeProtocol.hpp
 * Shared between the RP2040 firmware card and the VCV Rack plugin.
 * No platform-specific includes — header-only, pure C++ / C99.
 *
 * Signal rates:
 *   Audio 1 & 2  : 48 kHz, int16 per sample (full rate)
 *   CV 1 & 2     : ~3 kHz (4 values per 64-sample block, every 16 samples)
 *                  Captures fast envelopes & LFOs; matches hardware lowpass cutoff
 *   Pulse 1 & 2  : 48 kHz, 1 bit per sample packed — preserves edge timing to 20 µs
 *   Knob M/X/Y   : ~750 Hz (1 per block), uint16 0–4095 (human-controlled, slow)
 *   Switch Z      : ~750 Hz (1 per block), uint8 0=Down 1=Mid 2=Up
 *
 * CV sub-block rate: BRIDGE_CV_STRIDE = 16 samples → 48000/16 = 3000 Hz
 *   D2H bandwidth: 301 bytes/block × 750 = 226 KB/s
 *   H2D bandwidth: 294 bytes/block × 750 = 221 KB/s
 *   Total: ~447 KB/s — well under USB FS capacity (1.1 MB/s)
 */

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ── Protocol constants ────────────────────────────────────────────────────────

#define BRIDGE_BLOCK_SIZE    64                           // samples per USB frame
#define BRIDGE_SAMPLE_RATE   48000                        // RP2040 fixed sample rate (Hz)
#define BRIDGE_PULSE_BYTES   (BRIDGE_BLOCK_SIZE / 8)     // 8 bytes per channel
#define BRIDGE_CV_STRIDE     16                           // CV sample every N audio samples
#define BRIDGE_CV_COUNT      (BRIDGE_BLOCK_SIZE / BRIDGE_CV_STRIDE)  // 4 CV values per block
#define BRIDGE_CV_RATE       (BRIDGE_SAMPLE_RATE / BRIDGE_CV_STRIDE) // ~3000 Hz

// Sync words
#define BRIDGE_D2H_SYNC_0    0xBE   // Device → Host
#define BRIDGE_D2H_SYNC_1    0xEF
#define BRIDGE_H2D_SYNC_0    0xDE   // Host → Device
#define BRIDGE_H2D_SYNC_1    0xAD

// Frame sizes (bytes)
#define BRIDGE_D2H_SIZE      301
#define BRIDGE_H2D_SIZE      294

// ── Cross-platform packed struct ──────────────────────────────────────────────

#ifdef _MSC_VER
  #pragma pack(push, 1)
  #define BRIDGE_PACKED
#else
  #define BRIDGE_PACKED __attribute__((packed))
#endif

// ── Device → Host (RP2040 → VCV Rack): 301 bytes ─────────────────────────────
//
//  Field                           Bytes   Notes
//  sync[2]                           2     0xBE 0xEF
//  block_idx uint16                  2     rolling counter
//  audio1[64] int16                128     full 48kHz
//  audio2[64] int16                128     full 48kHz
//  cv1[4] int16                      8     every 16 samples → 3kHz
//  cv2[4] int16                      8
//  pulse1[8] uint8                   8     1 bit/sample, 48kHz edge timing
//  pulse2[8] uint8                   8
//  knob_main uint16                  2     once/block, 0–4095
//  knob_x    uint16                  2
//  knob_y    uint16                  2
//  switch_z  uint8                   1     0=Down 1=Mid 2=Up
//  crc       uint16                  2     CRC-16/CCITT
//  TOTAL                           301

struct BRIDGE_PACKED D2H_Frame {
    uint8_t  sync[2];
    uint16_t block_idx;

    int16_t  audio1[BRIDGE_BLOCK_SIZE];        // 128 bytes, full 48kHz
    int16_t  audio2[BRIDGE_BLOCK_SIZE];        // 128 bytes

    int16_t  cv1[BRIDGE_CV_COUNT];             // 8 bytes, ~3kHz
    int16_t  cv2[BRIDGE_CV_COUNT];             // 8 bytes

    uint8_t  pulse1[BRIDGE_PULSE_BYTES];       // 8 bytes, 1 bit/sample
    uint8_t  pulse2[BRIDGE_PULSE_BYTES];       // 8 bytes

    uint16_t knob_main;
    uint16_t knob_x;
    uint16_t knob_y;
    uint8_t  switch_z;

    uint16_t crc;
};
static_assert(sizeof(D2H_Frame) == BRIDGE_D2H_SIZE, "D2H_Frame size mismatch");

// ── Host → Device (VCV Rack → RP2040): 294 bytes ─────────────────────────────

struct BRIDGE_PACKED H2D_Frame {
    uint8_t  sync[2];
    uint16_t block_idx;

    int16_t  audio1[BRIDGE_BLOCK_SIZE];
    int16_t  audio2[BRIDGE_BLOCK_SIZE];

    int16_t  cv1[BRIDGE_CV_COUNT];             // ~3kHz — smoothed by hardware lowpass
    int16_t  cv2[BRIDGE_CV_COUNT];

    uint8_t  pulse1[BRIDGE_PULSE_BYTES];
    uint8_t  pulse2[BRIDGE_PULSE_BYTES];

    uint16_t crc;
};
static_assert(sizeof(H2D_Frame) == BRIDGE_H2D_SIZE, "H2D_Frame size mismatch");

#ifdef _MSC_VER
  #pragma pack(pop)
#endif

// ── CRC-16/CCITT (poly 0x1021, init 0xFFFF) ──────────────────────────────────

inline uint16_t bridge_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
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

// ── Voltage conversion helpers ────────────────────────────────────────────────

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
