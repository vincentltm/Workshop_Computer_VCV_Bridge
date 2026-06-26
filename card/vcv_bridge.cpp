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
 *
 * Build: Pico SDK + TinyUSB (pico_stdio_usb NOT used — raw CDC endpoint)
 */

#include "ComputerCard.h"
#include "BridgeProtocol.hpp"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "tusb.h"

// ── Config ────────────────────────────────────────────────────────────────────

static constexpr int BLOCK  = BRIDGE_BLOCK_SIZE;    // 64
static constexpr int PBYTES = BRIDGE_PULSE_BYTES;   // 8

// Number of ping-pong buffer pairs (must be 2 for double-buffering)
static constexpr int N_BUFS = 2;

// ── Ping-pong buffers  (Core 1 writes TX, Core 0 reads; Core 0 writes RX, Core 1 reads)

// TX: Core 1 builds blocks, Core 0 sends them
static D2H_Frame tx_frames[N_BUFS];
static volatile int  tx_write_buf  = 0;     // Core 1 writes into this index
static volatile bool tx_ready[N_BUFS] = {}; // Core 0 reads when true

// RX: Core 0 fills blocks from USB, Core 1 consumes
static H2D_Frame rx_frames[N_BUFS];
static volatile int  rx_read_buf   = 0;     // Core 1 reads from this index
static volatile bool rx_ready[N_BUFS] = {}; // Core 1 reads when true

// Current sample index within the block being built / consumed
static volatile int tx_idx = 0;
static volatile int rx_idx = 0;
static volatile uint16_t tx_block_counter = 0;

// ── LED VU peak detectors  ────────────────────────────────────────────────────
static int16_t vu_a1 = 0, vu_a2 = 0, vu_c1 = 0, vu_c2 = 0;
static int     vu_decay = 0;

// ── VCV Bridge Card ───────────────────────────────────────────────────────────

class VCVBridgeCard : public ComputerCard {
public:
    VCVBridgeCard() {}

    void ProcessSample() override {
        // ── Read hardware inputs ──────────────────────────────────────────
        int16_t a1 = AudioIn1();
        int16_t a2 = AudioIn2();
        int16_t c1 = CVIn1();    // hardware lowpass-filtered
        int16_t c2 = CVIn2();
        bool    p1 = PulseIn1();
        bool    p2 = PulseIn2();

        // ── Fill TX frame ─────────────────────────────────────────────────
        D2H_Frame& tf = tx_frames[tx_write_buf];
        tf.audio1[tx_idx] = a1;
        tf.audio2[tx_idx] = a2;
        bridge_pulse_set(tf.pulse1, tx_idx, p1);
        bridge_pulse_set(tf.pulse2, tx_idx, p2);

        tx_idx++;
        if (tx_idx == BLOCK) {
            // CV and controls: one snapshot per block is enough
            tf.sync[0]    = BRIDGE_D2H_SYNC_0;
            tf.sync[1]    = BRIDGE_D2H_SYNC_1;
            tf.block_idx  = tx_block_counter++;
            tf.cv1        = c1;
            tf.cv2        = c2;
            tf.knob_main  = (uint16_t)KnobVal(Main);
            tf.knob_x     = (uint16_t)KnobVal(X);
            tf.knob_y     = (uint16_t)KnobVal(Y);
            tf.switch_z   = (uint8_t)SwitchVal();
            tf.crc        = bridge_frame_crc(tf);

            tx_ready[tx_write_buf] = true;
            tx_write_buf ^= 1;      // flip ping-pong
            tx_idx = 0;

            // Prepare new TX buffer
            D2H_Frame& next = tx_frames[tx_write_buf];
            bridge_pulse_clear(next.pulse1);
            bridge_pulse_clear(next.pulse2);
        }

        // ── Consume RX frame → drive hardware outputs ─────────────────────
        const H2D_Frame& rf = rx_frames[rx_read_buf];
        if (rx_ready[rx_read_buf]) {
            AudioOut1(rf.audio1[rx_idx]);
            AudioOut2(rf.audio2[rx_idx]);

            // CV: apply once at block start
            if (rx_idx == 0) {
                CVOut1(rf.cv1);
                CVOut2(rf.cv2);
            }

            PulseOut1(bridge_pulse_get(rf.pulse1, rx_idx));
            PulseOut2(bridge_pulse_get(rf.pulse2, rx_idx));

            rx_idx++;
            if (rx_idx == BLOCK) {
                rx_ready[rx_read_buf] = false;
                rx_read_buf ^= 1;   // flip to next buffer
                rx_idx = 0;
            }
        } else {
            // No data yet — silence / hold
            AudioOut1(0);
            AudioOut2(0);
        }

        // ── LED VU meters ─────────────────────────────────────────────────
        if (a1 < 0) a1 = -a1;
        if (a2 < 0) a2 = -a2;
        if (c1 < 0) c1 = -c1;
        if (c2 < 0) c2 = -c2;

        if (a1 > vu_a1) vu_a1 = a1;
        if (a2 > vu_a2) vu_a2 = a2;
        if (c1 > vu_c1) vu_c1 = c1;
        if (c2 > vu_c2) vu_c2 = c2;

        if (++vu_decay >= 480) {    // decay ~every 10ms
            vu_decay = 0;
            LedBrightness(0, (uint16_t)(vu_a1 >> 1));  // Audio 1 (max 4095)
            LedBrightness(1, (uint16_t)(vu_a2 >> 1));  // Audio 2
            LedBrightness(2, (uint16_t)(vu_c1 >> 1));  // CV 1
            LedBrightness(3, (uint16_t)(vu_c2 >> 1));  // CV 2
            LedOn(4, PulseIn1());                        // Pulse 1 gate
            LedOn(5, PulseIn2());                        // Pulse 2 gate
            vu_a1 = vu_a2 = vu_c1 = vu_c2 = 0;
        }
    }
};

// ── USB CDC frame receiver state ──────────────────────────────────────────────

static uint8_t  rx_parse_buf[BRIDGE_H2D_SIZE];
static int      rx_parse_pos = 0;
static bool     rx_sync_found = false;
static int      rx_write_buf_usb = 0;  // which RX ping-pong Core 0 is filling

static void usb_process_byte(uint8_t b) {
    if (!rx_sync_found) {
        // Slide sync window
        if (rx_parse_pos == 0 && b == BRIDGE_H2D_SYNC_0) {
            rx_parse_buf[rx_parse_pos++] = b;
        } else if (rx_parse_pos == 1 && b == BRIDGE_H2D_SYNC_1) {
            rx_parse_buf[rx_parse_pos++] = b;
            rx_sync_found = true;
        } else {
            rx_parse_pos = 0; // reset
        }
        return;
    }

    rx_parse_buf[rx_parse_pos++] = b;
    if (rx_parse_pos == BRIDGE_H2D_SIZE) {
        // Full frame — validate CRC
        const H2D_Frame* f = reinterpret_cast<const H2D_Frame*>(rx_parse_buf);
        uint16_t expected = bridge_frame_crc(*f);
        if (f->crc == expected) {
            // Copy into the non-active RX buffer
            int wb = rx_write_buf_usb;
            if (!rx_ready[wb]) {
                rx_frames[wb] = *f;
                rx_ready[wb]  = true;
                rx_write_buf_usb ^= 1;
            }
            // else: drop frame (Core 1 hasn't consumed yet)
        }
        rx_parse_pos   = 0;
        rx_sync_found  = false;
    }
}

// ── Core 0 main: TinyUSB + frame pump ────────────────────────────────────────

static void core1_entry() {
    VCVBridgeCard card;
    card.Run();   // never returns — drives 48kHz DSP via DMA IRQ
}

int main() {
    // Overclock for USB + DSP headroom
    // set_sys_clock_khz(250000, true);  // uncomment if needed

    stdio_init_all();

    // Init TinyUSB as CDC device
    tusb_init();

    // Launch Core 1 (audio DSP)
    multicore_launch_core1(core1_entry);

    // Core 0: USB task + TX pump
    while (true) {
        tud_task();

        // ── Send TX frames to host ────────────────────────────────────────
        int read_buf = tx_write_buf ^ 1;    // the buffer Core 1 just finished
        if (tx_ready[read_buf] && tud_cdc_connected()) {
            // Write frame in chunks if CDC TX buffer is limited
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

        // ── Receive RX frames from host ───────────────────────────────────
        while (tud_cdc_available()) {
            uint8_t b;
            tud_cdc_read(&b, 1);
            usb_process_byte(b);
        }
    }
}
