/**
 * vcv_bridge.cpp  —  Workshop Computer VCV Bridge Card
 *
 * Streams all Workshop Computer I/O bidirectionally to VCV Rack
 * over USB CDC Serial. No USB Audio Class involved.
 *
 * Core 0: main() → TinyUSB task + frame TX/RX via CDC
 * Core 1: ComputerCard AudioWorker → ProcessSample() at 48 kHz
 *
 * Protocol: see BridgeProtocol.hpp
 *   Audio:  48 kHz int16 per sample
 *   CV:     3 kHz (every BRIDGE_CV_STRIDE=16 samples) — covers hardware lowpass
 *   Pulse:  48 kHz bit-packed (1 bit/sample) — 20 µs edge resolution
 *   Knobs:  750 Hz (once per block)
 */

#include "ComputerCard.h"
#include "BridgeProtocol.hpp"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"
#include <cmath>
#include <algorithm>

// ── Ping-pong double-buffers (Core 1 writes TX, Core 0 sends; vice versa for RX)

static constexpr int BLOCK   = BRIDGE_BLOCK_SIZE;   // 64
static constexpr int PBYTES  = BRIDGE_PULSE_BYTES;  // 8
static constexpr int CV_STEP = BRIDGE_CV_STRIDE;    // 16
static constexpr int CV_N    = BRIDGE_CV_COUNT;     // 4

// TX: Core 1 builds, Core 0 sends
static D2H_Frame     tx_frames[2];
static volatile int  tx_write_buf  = 0;
static volatile bool tx_ready[2]   = {false, false};

// RX: Core 0 fills from USB, Core 1 consumes
static H2D_Frame     rx_frames[2];
static volatile int  rx_read_buf   = 0;
static volatile bool rx_ready[2]   = {false, false};

static volatile int      tx_idx          = 0;
static volatile int      rx_idx          = 0;
static volatile uint16_t tx_block_counter = 0;

// ── VU peak state (updated once per 10 ms in ProcessSample)
static int vu_a1 = 0, vu_a2 = 0, vu_c1 = 0, vu_c2 = 0;
static int     vu_tick = 0;

// ── VCV Bridge Card ───────────────────────────────────────────────────────────

class VCVBridgeCard : public ComputerCard {
public:
    VCVBridgeCard() {}

    void ProcessSample() override {
        // ── Read hardware ─────────────────────────────────────────────────
        int16_t a1 = AudioIn1();
        int16_t a2 = AudioIn2();
        bool    p1 = PulseIn1();
        bool    p2 = PulseIn2();

        // ── Fill TX frame ─────────────────────────────────────────────────
        D2H_Frame& tf = tx_frames[tx_write_buf];
        tf.audio1[tx_idx] = a1;
        tf.audio2[tx_idx] = a2;
        bridge_pulse_set(tf.pulse1, tx_idx, p1);
        bridge_pulse_set(tf.pulse2, tx_idx, p2);

        // CV every CV_STEP samples → 3 kHz
        if ((tx_idx & (CV_STEP - 1)) == 0) {
            int ci = tx_idx >> 4;  // tx_idx / 16
            tf.cv1[ci] = CVIn1();
            tf.cv2[ci] = CVIn2();
        }

        tx_idx++;
        if (tx_idx == BLOCK) {
            tf.sync[0]    = BRIDGE_D2H_SYNC_0;
            tf.sync[1]    = BRIDGE_D2H_SYNC_1;
            tf.block_idx  = tx_block_counter++;
            tf.knob_main  = (uint16_t)KnobVal(Main);
            tf.knob_x     = (uint16_t)KnobVal(X);
            tf.knob_y     = (uint16_t)KnobVal(Y);
            tf.switch_z   = (uint8_t)SwitchVal();
            tf.crc        = bridge_frame_crc(tf);

            tx_ready[tx_write_buf] = true;
            tx_write_buf ^= 1;
            tx_idx = 0;

            // Clear pulse arrays of incoming write buffer
            D2H_Frame& next = tx_frames[tx_write_buf];
            bridge_pulse_clear(next.pulse1);
            bridge_pulse_clear(next.pulse2);
        }

        // ── Consume RX frame → hardware outputs ───────────────────────────
        if (rx_ready[rx_read_buf]) {
            const H2D_Frame& rf = rx_frames[rx_read_buf];

            AudioOut1(rf.audio1[rx_idx]);
            AudioOut2(rf.audio2[rx_idx]);

            // CV every CV_STEP samples
            if ((rx_idx & (CV_STEP - 1)) == 0) {
                int ci = rx_idx >> 4;
                CVOut1(rf.cv1[ci]);
                CVOut2(rf.cv2[ci]);
            }

            PulseOut1(bridge_pulse_get(rf.pulse1, rx_idx));
            PulseOut2(bridge_pulse_get(rf.pulse2, rx_idx));

            rx_idx++;
            if (rx_idx == BLOCK) {
                rx_ready[rx_read_buf] = false;
                rx_read_buf ^= 1;
                rx_idx = 0;
            }
        } else {
            AudioOut1(0);
            AudioOut2(0);
        }

        // ── LED VU (update every ~10 ms = 480 samples) ───────────────────
        int aa1 = std::abs((int)a1);
        int aa2 = std::abs((int)a2);
        int cc1 = std::abs((int)CVIn1());
        int cc2 = std::abs((int)CVIn2());
        if (aa1 > vu_a1) vu_a1 = aa1;
        if (aa2 > vu_a2) vu_a2 = aa2;
        if (cc1 > vu_c1) vu_c1 = cc1;
        if (cc2 > vu_c2) vu_c2 = cc2;

        if (++vu_tick >= 480) {
            vu_tick = 0;
            // Scale audio (0..32767 -> 0..4095) and CV (0..2048 -> 0..4095)
            LedBrightness(0, (uint16_t)std::min(4095, vu_a1 / 8));
            LedBrightness(1, (uint16_t)std::min(4095, vu_a2 / 8));
            LedBrightness(2, (uint16_t)std::min(4095, vu_c1 * 2));
            LedBrightness(3, (uint16_t)std::min(4095, vu_c2 * 2));
            LedOn(4, PulseIn1());
            LedOn(5, PulseIn2());
            vu_a1 = vu_a2 = vu_c1 = vu_c2 = 0;
        }
    }
};

// ── USB CDC RX frame parser ───────────────────────────────────────────────────

static uint8_t  rx_parse_buf[BRIDGE_H2D_SIZE];
static int      rx_parse_pos   = 0;
static bool     rx_sync_found  = false;
static int      rx_write_usb   = 0;  // which RX ping-pong Core 0 is filling

static void usb_process_byte(uint8_t b) {
    if (!rx_sync_found) {
        if (rx_parse_pos == 0 && b == BRIDGE_H2D_SYNC_0) {
            rx_parse_buf[rx_parse_pos++] = b;
        } else if (rx_parse_pos == 1 && b == BRIDGE_H2D_SYNC_1) {
            rx_parse_buf[rx_parse_pos++] = b;
            rx_sync_found = true;
        } else {
            rx_parse_pos = 0;
        }
        return;
    }

    rx_parse_buf[rx_parse_pos++] = b;
    if (rx_parse_pos == BRIDGE_H2D_SIZE) {
        const H2D_Frame* f = reinterpret_cast<const H2D_Frame*>(rx_parse_buf);
        if (f->crc == bridge_frame_crc(*f)) {
            int wb = rx_write_usb;
            if (!rx_ready[wb]) {
                rx_frames[wb] = *f;
                rx_ready[wb]  = true;
                rx_write_usb ^= 1;
            }
            // else: drop — Core 1 hasn't consumed yet
        }
        rx_parse_pos  = 0;
        rx_sync_found = false;
    }
}

// ── Core 0: USB task + frame pump ────────────────────────────────────────────

static void core1_entry() {
    VCVBridgeCard card;
    card.Run();   // never returns
}

int main() {
    stdio_init_all();
    tusb_init();
    multicore_launch_core1(core1_entry);

    while (true) {
        tud_task();

        // Send completed TX blocks to host
        int read_buf = tx_write_buf ^ 1;
        if (tx_ready[read_buf] && tud_cdc_connected()) {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(&tx_frames[read_buf]);
            uint32_t written = 0;
            while (written < BRIDGE_D2H_SIZE) {
                uint32_t avail = tud_cdc_write_available();
                if (avail == 0) { tud_task(); continue; }
                uint32_t chunk = BRIDGE_D2H_SIZE - written;
                if (chunk > avail) chunk = avail;
                tud_cdc_write(data + written, chunk);
                written += chunk;
            }
            tud_cdc_write_flush();
            tx_ready[read_buf] = false;
        }

        // Receive bytes from host
        while (tud_cdc_available()) {
            uint8_t b;
            tud_cdc_read(&b, 1);
            usb_process_byte(b);
        }
    }
}
