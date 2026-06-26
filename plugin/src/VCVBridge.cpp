/**
 * VCVBridge.cpp  —  MTM VCV Bridge module
 *
 * Bidirectional USB CDC bridge between Workshop Computer and VCV Rack.
 *
 * Ports exposed:
 *   FROM HARDWARE (outputs):
 *     Audio 1/2     — full 48kHz int16, SRC to VCV rate
 *     CV 1/2        — 3kHz effective (every 16 samples), int16
 *     Pulse 1/2     — 48kHz bit-accurate, reconstructed at VCV rate
 *     Knob Main/X/Y — 0–10 V, ~750 Hz
 *     Switch Z      — 0 / 5 / 10 V
 *
 *   TO HARDWARE (inputs):
 *     Audio 1/2, CV 1/2, Pulse 1/2
 *
 * Knob parameter mapping:
 *   Right-click → "Map Knob [Main/X/Y] to parameter" → click a VCV knob.
 *   Physical hardware knob drives that VCV parameter directly.
 *
 * Sample rate:
 *   RP2040 is clock master at 48 kHz.
 *   VCV adapts via linear interpolation (phase accumulator SRC).
 *   Audio only: no anti-aliasing filter (add later if needed at high VCV rates).
 *
 * Threading:
 *   Serial thread — non-audio background thread, reads/writes USB CDC.
 *   VCV audio thread — process(), phase-accumulator SRC, ring buffer exchange.
 */

#include "plugin.hpp"
#include "SerialPort.hpp"
#include "BridgeProtocol.hpp"

#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>

// ── Lock-free single-producer / single-consumer ring buffer ──────────────────

template<typename T, size_t N>
struct RingBuf {
    static_assert((N & (N-1)) == 0, "N must be a power of 2");
    std::atomic<size_t> head{0}, tail{0};
    T buf[N];

    bool push(const T& v) {
        size_t t    = tail.load(std::memory_order_relaxed);
        size_t next = (t + 1) & (N - 1);
        if (next == head.load(std::memory_order_acquire)) return false;
        buf[t] = v;
        tail.store(next, std::memory_order_release);
        return true;
    }
    bool pop(T& v) {
        size_t h = head.load(std::memory_order_relaxed);
        if (h == tail.load(std::memory_order_acquire)) return false;
        v = buf[h];
        head.store((h + 1) & (N - 1), std::memory_order_release);
        return true;
    }
    size_t size() const {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_acquire);
        return (t - h) & (N - 1);
    }
    void clear() { head.store(0, std::memory_order_relaxed); tail.store(0, std::memory_order_relaxed); }
};

static constexpr size_t RING_DEPTH = 8;  // ~10 ms jitter absorption

struct RxBlock { D2H_Frame frame; };
struct TxBlock { H2D_Frame frame; };

// ── Port IDs ──────────────────────────────────────────────────────────────────

enum ParamIds  { NUM_PARAMS };

enum InputIds  {
    AUDIO1_INPUT, AUDIO2_INPUT,
    CV1_INPUT,    CV2_INPUT,
    PULSE1_INPUT, PULSE2_INPUT,
    NUM_INPUTS
};

enum OutputIds {
    AUDIO1_OUTPUT, AUDIO2_OUTPUT,
    CV1_OUTPUT,    CV2_OUTPUT,
    PULSE1_OUTPUT, PULSE2_OUTPUT,
    KNOB_MAIN_OUTPUT, KNOB_X_OUTPUT, KNOB_Y_OUTPUT,
    SWITCH_OUTPUT,
    NUM_OUTPUTS
};

// GreenRedLight occupies TWO consecutive IDs.
// RX and TX lights follow after.
enum LightIds {
    STATUS_LIGHT_G,   // 0 — green component of the status GreenRedLight
    STATUS_LIGHT_R,   // 1 — red component
    RX_LIGHT,         // 2 — blinks on receive
    TX_LIGHT,         // 3 — blinks on transmit
    // Input level LEDs (6)
    IN_LED_1, IN_LED_2, IN_LED_3, IN_LED_4, IN_LED_5, IN_LED_6,
    // Output level LEDs (6)
    OUT_LED_1, OUT_LED_2, OUT_LED_3, OUT_LED_4, OUT_LED_5, OUT_LED_6,
    NUM_LIGHTS
};

// ── VCVBridge module ──────────────────────────────────────────────────────────

struct VCVBridgeModule : rack::Module {

    VCVBridgeModule() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configInput(AUDIO1_INPUT, "Audio 1 → Hardware");
        configInput(AUDIO2_INPUT, "Audio 2 → Hardware");
        configInput(CV1_INPUT,    "CV 1 → Hardware (–6 to +6 V)");
        configInput(CV2_INPUT,    "CV 2 → Hardware (–6 to +6 V)");
        configInput(PULSE1_INPUT, "Pulse 1 → Hardware (≥1 V = high)");
        configInput(PULSE2_INPUT, "Pulse 2 → Hardware (≥1 V = high)");

        configOutput(AUDIO1_OUTPUT, "Audio 1 from Hardware");
        configOutput(AUDIO2_OUTPUT, "Audio 2 from Hardware");
        configOutput(CV1_OUTPUT,    "CV 1 from Hardware (–6 to +6 V, ~3kHz)");
        configOutput(CV2_OUTPUT,    "CV 2 from Hardware (–6 to +6 V, ~3kHz)");
        configOutput(PULSE1_OUTPUT, "Pulse 1 from Hardware (0 / 5 V)");
        configOutput(PULSE2_OUTPUT, "Pulse 2 from Hardware (0 / 5 V)");
        configOutput(KNOB_MAIN_OUTPUT, "Knob Main position (0–10 V)");
        configOutput(KNOB_X_OUTPUT,    "Knob X position (0–10 V)");
        configOutput(KNOB_Y_OUTPUT,    "Knob Y position (0–10 V)");
        configOutput(SWITCH_OUTPUT,    "Switch position (0 / 5 / 10 V)");

        // Initialise atomics (no brace-init for array of atomics)
        for (int i = 0; i < 3; i++) {
            hw_knob[i].store(0);
            knob_handles[i].text = knob_names[i];
            APP->engine->addParamHandle(&knob_handles[i]);
        }
        hw_switch_val.store(1);
    }

    ~VCVBridgeModule() override {
        stop_serial();
        for (int i = 0; i < 3; i++)
            APP->engine->removeParamHandle(&knob_handles[i]);
    }

    // ── Serial state
    std::string port_path;
    std::atomic<bool> connected{false};
    std::atomic<bool> run_thread{false};

    // ── Ring buffers
    RingBuf<RxBlock, RING_DEPTH> rx_ring;
    RingBuf<TxBlock, RING_DEPTH> tx_ring;

    // ── HW→VCV SRC state
    double   hw_phase       = 0.0;
    float    prev_rx_a1     = 0.f, curr_rx_a1 = 0.f;
    float    prev_rx_a2     = 0.f, curr_rx_a2 = 0.f;
    bool     curr_rx_p1     = false, curr_rx_p2 = false;
    float    curr_rx_cv1    = 0.f,  curr_rx_cv2 = 0.f;
    float    curr_rx_km     = 0.f,  curr_rx_kx  = 0.f;
    float    curr_rx_ky     = 0.f,  curr_rx_sw  = 0.f;

    D2H_Frame rx_cur_frame   = {};
    int       rx_frame_sample = BRIDGE_BLOCK_SIZE;  // triggers first fetch

    // ── VCV→HW SRC state
    H2D_Frame tx_cur_frame   = {};
    int       tx_frame_sample = 0;
    uint16_t  tx_block_idx   = 0;
    double    vcv_src_phase  = 0.0;

    // ── Serial thread
    std::thread serial_thread;

    // ── Knob mapping
    static constexpr const char* knob_names[3] = {"Knob Main", "Knob X", "Knob Y"};
    rack::engine::ParamHandle knob_handles[3];
    int knob_learn_idx = -1;

    // Latest knob values from hardware (atomic — written by serial thread, read by audio thread)
    std::atomic<uint16_t> hw_knob[3];    // initialised in constructor body
    std::atomic<uint8_t>  hw_switch_val;

    // Light blink counters
    int rx_blink = 0, tx_blink = 0;

    // VU level meters
    float in_level[6] = {};
    float out_level[6] = {};

    // ── Serial thread management
    void start_serial(const std::string& path) {
        stop_serial();
        port_path  = path;
        run_thread = true;
        rx_ring.clear();
        tx_ring.clear();
        hw_phase = 0.0;
        vcv_src_phase = 0.0;
        rx_frame_sample = BRIDGE_BLOCK_SIZE;
        tx_frame_sample = 0;
        serial_thread = std::thread([this]() { serial_worker(); });
    }

    void stop_serial() {
        run_thread = false;
        connected  = false;
        if (serial_thread.joinable()) serial_thread.join();
    }

    // ── Serial worker (background thread) ────────────────────────────────────
    void serial_worker() {
        SerialPort port;
        uint8_t parse_buf[BRIDGE_D2H_SIZE];
        int     parse_pos   = 0;
        bool    sync_found  = false;

        auto try_connect = [&]() -> bool {
            port.close();
            if (!port.open(port_path)) return false;
            parse_pos  = 0;
            sync_found = false;
            return true;
        };

        while (run_thread.load(std::memory_order_relaxed)) {
            if (!port.is_open()) {
                connected = false;
                if (!try_connect()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                connected = true;
            }

            // TX: drain pending blocks
            TxBlock tb;
            while (tx_ring.pop(tb)) {
                const uint8_t* d = reinterpret_cast<const uint8_t*>(&tb.frame);
                if (!port.write(d, BRIDGE_H2D_SIZE)) {
                    port.close(); connected = false; break;
                }
                tx_blink = 4;
            }
            if (!port.is_open()) continue;

            // RX: read incoming bytes, parse frames
            uint8_t buf[256];
            int n = port.read(buf, sizeof(buf));
            if (n < 0) { port.close(); connected = false; continue; }

            for (int i = 0; i < n; i++) {
                uint8_t b = buf[i];
                if (!sync_found) {
                    if (parse_pos == 0 && b == BRIDGE_D2H_SYNC_0)      { parse_buf[parse_pos++] = b; }
                    else if (parse_pos == 1 && b == BRIDGE_D2H_SYNC_1) { parse_buf[parse_pos++] = b; sync_found = true; }
                    else                                                 { parse_pos = 0; }
                    continue;
                }
                parse_buf[parse_pos++] = b;
                if (parse_pos == BRIDGE_D2H_SIZE) {
                    const D2H_Frame* f = reinterpret_cast<const D2H_Frame*>(parse_buf);
                    if (f->crc == bridge_frame_crc(*f)) {
                        // Update knob/switch atomics (audio thread reads these)
                        hw_knob[0]     = f->knob_main;
                        hw_knob[1]     = f->knob_x;
                        hw_knob[2]     = f->knob_y;
                        hw_switch_val  = f->switch_z;
                        RxBlock rb; rb.frame = *f;
                        rx_ring.push(rb);
                        rx_blink = 4;
                    }
                    parse_pos  = 0;
                    sync_found = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }

        port.close();
        connected = false;
    }

    // ── process() — VCV audio thread ─────────────────────────────────────────
    void process(const ProcessArgs& args) override {
        const double hw_rate  = (double)BRIDGE_SAMPLE_RATE;  // 48000
        const double vcv_rate = args.sampleRate;
        const double step     = hw_rate / vcv_rate;

        // ── HW → VCV (RP2040 is master, VCV interpolates) ────────────────
        hw_phase += step;
        while (hw_phase >= 1.0) {
            hw_phase -= 1.0;

            rx_frame_sample++;
            if (rx_frame_sample >= BRIDGE_BLOCK_SIZE) {
                RxBlock rb;
                if (rx_ring.pop(rb)) {
                    rx_cur_frame = rb.frame;
                }
                rx_frame_sample = 0;
            }

            int si = rx_frame_sample;
            prev_rx_a1 = curr_rx_a1;
            prev_rx_a2 = curr_rx_a2;
            curr_rx_a1 = bridge_to_volts(rx_cur_frame.audio1[si]);
            curr_rx_a2 = bridge_to_volts(rx_cur_frame.audio2[si]);
            curr_rx_p1 = bridge_pulse_get(rx_cur_frame.pulse1, si);
            curr_rx_p2 = bridge_pulse_get(rx_cur_frame.pulse2, si);

            // CV: update every BRIDGE_CV_STRIDE samples
            if ((si & (BRIDGE_CV_STRIDE - 1)) == 0) {
                int ci      = si >> 4;  // si / 16
                curr_rx_cv1 = bridge_to_volts(rx_cur_frame.cv1[ci]);
                curr_rx_cv2 = bridge_to_volts(rx_cur_frame.cv2[ci]);
            }
            // Knobs/switch: update at block start
            if (si == 0) {
                curr_rx_km = bridge_knob_to_volts(rx_cur_frame.knob_main);
                curr_rx_kx = bridge_knob_to_volts(rx_cur_frame.knob_x);
                curr_rx_ky = bridge_knob_to_volts(rx_cur_frame.knob_y);
                curr_rx_sw = bridge_switch_to_volts(rx_cur_frame.switch_z);
            }
        }

        // Linear interpolation between adjacent HW samples
        float frac  = (float)hw_phase;
        float a1out = prev_rx_a1 + frac * (curr_rx_a1 - prev_rx_a1);
        float a2out = prev_rx_a2 + frac * (curr_rx_a2 - prev_rx_a2);

        outputs[AUDIO1_OUTPUT].setVoltage(a1out);
        outputs[AUDIO2_OUTPUT].setVoltage(a2out);
        outputs[CV1_OUTPUT].setVoltage(curr_rx_cv1);
        outputs[CV2_OUTPUT].setVoltage(curr_rx_cv2);
        outputs[PULSE1_OUTPUT].setVoltage(curr_rx_p1 ? 5.f : 0.f);
        outputs[PULSE2_OUTPUT].setVoltage(curr_rx_p2 ? 5.f : 0.f);
        outputs[KNOB_MAIN_OUTPUT].setVoltage(curr_rx_km);
        outputs[KNOB_X_OUTPUT].setVoltage(curr_rx_kx);
        outputs[KNOB_Y_OUTPUT].setVoltage(curr_rx_ky);
        outputs[SWITCH_OUTPUT].setVoltage(curr_rx_sw);

        // ── Knob → VCV parameter mapping ─────────────────────────────────
        for (int i = 0; i < 3; i++) {
            rack::engine::ParamHandle& h = knob_handles[i];
            if (h.moduleId < 0) continue;
            rack::Module* m = APP->engine->getModule(h.moduleId);
            if (!m) continue;
            int pid = h.paramId;
            if (pid < 0 || pid >= (int)m->paramQuantities.size()) continue;
            rack::engine::ParamQuantity* pq = m->paramQuantities[pid];
            if (!pq) continue;
            float norm = hw_knob[i].load() / 4095.f;
            pq->setScaledValue(norm);
        }

        // ── VCV → HW ─────────────────────────────────────────────────────
        vcv_src_phase += step;
        while (vcv_src_phase >= 1.0) {
            vcv_src_phase -= 1.0;

            int si = tx_frame_sample;
            tx_cur_frame.audio1[si] = bridge_from_volts(inputs[AUDIO1_INPUT].getVoltage());
            tx_cur_frame.audio2[si] = bridge_from_volts(inputs[AUDIO2_INPUT].getVoltage());
            bridge_pulse_set(tx_cur_frame.pulse1, si, inputs[PULSE1_INPUT].getVoltage() >= 1.f);
            bridge_pulse_set(tx_cur_frame.pulse2, si, inputs[PULSE2_INPUT].getVoltage() >= 1.f);

            // CV every CV_STRIDE samples
            if ((si & (BRIDGE_CV_STRIDE - 1)) == 0) {
                int ci = si >> 4;
                tx_cur_frame.cv1[ci] = bridge_from_volts(inputs[CV1_INPUT].getVoltage());
                tx_cur_frame.cv2[ci] = bridge_from_volts(inputs[CV2_INPUT].getVoltage());
            }

            tx_frame_sample++;
            if (tx_frame_sample == BRIDGE_BLOCK_SIZE) {
                tx_cur_frame.sync[0]   = BRIDGE_H2D_SYNC_0;
                tx_cur_frame.sync[1]   = BRIDGE_H2D_SYNC_1;
                tx_cur_frame.block_idx = tx_block_idx++;
                tx_cur_frame.crc       = bridge_frame_crc(tx_cur_frame);

                TxBlock tb; tb.frame = tx_cur_frame;
                tx_ring.push(tb);

                tx_frame_sample = 0;
                memset(&tx_cur_frame, 0, sizeof(tx_cur_frame));
                bridge_pulse_clear(tx_cur_frame.pulse1);
                bridge_pulse_clear(tx_cur_frame.pulse2);
                tx_blink = 4;
            }
        }

        // ── VU level meters ──────────────────────────────────────────────
        auto update_vu = [](float& current, float target, float attack = 0.2f, float decay = 0.005f) {
            if (target > current) {
                current += attack * (target - current);
            } else {
                current += decay * (target - current);
            }
        };

        // Inputs (VCV to Hardware)
        update_vu(in_level[0], std::max(0.f, std::min(1.f, std::abs(inputs[AUDIO1_INPUT].getVoltage()) / 5.f)));
        update_vu(in_level[1], std::max(0.f, std::min(1.f, std::abs(inputs[AUDIO2_INPUT].getVoltage()) / 5.f)));
        update_vu(in_level[2], std::max(0.f, std::min(1.f, std::abs(inputs[CV1_INPUT].getVoltage()) / 5.f)));
        update_vu(in_level[3], std::max(0.f, std::min(1.f, std::abs(inputs[CV2_INPUT].getVoltage()) / 5.f)));
        update_vu(in_level[4], (inputs[PULSE1_INPUT].getVoltage() >= 1.f) ? 1.f : 0.f, 0.5f, 0.05f);
        update_vu(in_level[5], (inputs[PULSE2_INPUT].getVoltage() >= 1.f) ? 1.f : 0.f, 0.5f, 0.05f);

        // Outputs (Hardware to VCV)
        update_vu(out_level[0], std::max(0.f, std::min(1.f, std::abs(outputs[AUDIO1_OUTPUT].getVoltage()) / 5.f)));
        update_vu(out_level[1], std::max(0.f, std::min(1.f, std::abs(outputs[AUDIO2_OUTPUT].getVoltage()) / 5.f)));
        update_vu(out_level[2], std::max(0.f, std::min(1.f, std::abs(outputs[CV1_OUTPUT].getVoltage()) / 5.f)));
        update_vu(out_level[3], std::max(0.f, std::min(1.f, std::abs(outputs[CV2_OUTPUT].getVoltage()) / 5.f)));
        update_vu(out_level[4], (outputs[PULSE1_OUTPUT].getVoltage() >= 1.f) ? 1.f : 0.f, 0.5f, 0.05f);
        update_vu(out_level[5], (outputs[PULSE2_OUTPUT].getVoltage() >= 1.f) ? 1.f : 0.f, 0.5f, 0.05f);

        // Update level lights
        for (int i = 0; i < 6; i++) {
            lights[IN_LED_1 + i].setBrightness(in_level[i]);
            lights[OUT_LED_1 + i].setBrightness(out_level[i]);
        }

        // ── Lights ───────────────────────────────────────────────────────
        bool conn = connected.load(std::memory_order_relaxed);
        lights[STATUS_LIGHT_G].setBrightness(conn  ? 1.f : 0.f);
        lights[STATUS_LIGHT_R].setBrightness(!conn ? 0.6f : 0.f);

        if (rx_blink > 0) { lights[RX_LIGHT].setBrightness(1.f); rx_blink--; }
        else                { lights[RX_LIGHT].setBrightness(0.f); }
        if (tx_blink > 0) { lights[TX_LIGHT].setBrightness(1.f); tx_blink--; }
        else                { lights[TX_LIGHT].setBrightness(0.f); }
    }

    // ── JSON persistence ──────────────────────────────────────────────────────
    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "port_path", json_string(port_path.c_str()));
        json_t* maps = json_array();
        for (int i = 0; i < 3; i++) {
            json_t* m = json_object();
            json_object_set_new(m, "moduleId", json_integer(knob_handles[i].moduleId));
            json_object_set_new(m, "paramId",  json_integer(knob_handles[i].paramId));
            json_array_append_new(maps, m);
        }
        json_object_set_new(root, "knob_maps", maps);
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* pp = json_object_get(root, "port_path");
        if (pp && json_is_string(pp)) port_path = json_string_value(pp);

        json_t* maps = json_object_get(root, "knob_maps");
        if (maps) {
            for (size_t i = 0; i < std::min((size_t)3, json_array_size(maps)); i++) {
                json_t* m   = json_array_get(maps, i);
                int64_t mid = json_integer_value(json_object_get(m, "moduleId"));
                int     pid = (int)json_integer_value(json_object_get(m, "paramId"));
                if (mid >= 0 && pid >= 0)
                    APP->engine->updateParamHandle(&knob_handles[i], mid, pid, false);
            }
        }

        if (!port_path.empty()) start_serial(port_path);
    }
};

// Note: knob_names is constexpr static inline in C++17 — no out-of-class definition needed

// ── Widget ────────────────────────────────────────────────────────────────────

struct VCVBridgeWidget : rack::ModuleWidget {
    explicit VCVBridgeWidget(VCVBridgeModule* module) {
        setModule(module);
        setPanel(rack::createPanel(rack::asset::plugin(pluginInstance, "res/VCVBridge.svg")));

        // Screws — black, matching Workshop Computer style
        addChild(createWidget<ScrewBlack>(Vec(15.f,   0.f)));
        addChild(createWidget<ScrewBlack>(Vec(30.f, 365.f)));

        // Status / RX / TX lights  (raw px coords, viewBox 0 0 60 380)
        addChild(createLightCentered<SmallLight<GreenRedLight>>(
            Vec(20.f, 20.f), module, STATUS_LIGHT_G));
        addChild(createLightCentered<TinyLight<GreenLight>>(
            Vec(30.f, 20.f), module, RX_LIGHT));
        addChild(createLightCentered<TinyLight<YellowLight>>(
            Vec(42.f, 20.f), module, TX_LIGHT));

        // ── CONTROL outputs (knobs and switch outputs)
        // Row 1: Knob Main (left), Knob X (right)
        addOutput(createOutputCentered<PJ301MPort>(Vec(15.f, 50.f), module, KNOB_MAIN_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(45.f, 50.f), module, KNOB_X_OUTPUT));
        // Row 2: Knob Y (left), Switch Z (right)
        addOutput(createOutputCentered<PJ301MPort>(Vec(15.f, 86.f), module, KNOB_Y_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(45.f, 86.f), module, SWITCH_OUTPUT));

        // ── TO HARDWARE inputs
        // Row 3: Audio 1/2 Inputs
        addInput(createInputCentered<PJ301MPort>(Vec(15.f, 122.f), module, AUDIO1_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(45.f, 122.f), module, AUDIO2_INPUT));
        // Row 4: CV 1/2 Inputs
        addInput(createInputCentered<PJ301MPort>(Vec(15.f, 158.f), module, CV1_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(45.f, 158.f), module, CV2_INPUT));
        // Row 5: Pulse 1/2 Inputs
        addInput(createInputCentered<PJ301MPort>(Vec(15.f, 194.f), module, PULSE1_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(45.f, 194.f), module, PULSE2_INPUT));

        // ── FROM HARDWARE outputs (Moved up: rows 6, 7, 8)
        // Row 6: Audio 1/2 Outputs
        addOutput(createOutputCentered<PJ301MPort>(Vec(15.f, 230.f), module, AUDIO1_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(45.f, 230.f), module, AUDIO2_OUTPUT));
        // Row 7: CV 1/2 Outputs
        addOutput(createOutputCentered<PJ301MPort>(Vec(15.f, 266.f), module, CV1_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(45.f, 266.f), module, CV2_OUTPUT));
        // Row 8: Pulse 1/2 Outputs
        addOutput(createOutputCentered<PJ301MPort>(Vec(15.f, 302.f), module, PULSE1_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(45.f, 302.f), module, PULSE2_OUTPUT));

        // ── VU level indicators (12 LEDs at the bottom in two clusters of 3x2)
        // Left cluster (Inputs) - Columns: 11.5, 23.5. Rows: 324, 338, 352
        addChild(createLightCentered<TinyLight<GreenLight>>(Vec(11.5f, 324.f), module, IN_LED_1));
        addChild(createLightCentered<TinyLight<GreenLight>>(Vec(23.5f, 324.f), module, IN_LED_2));
        addChild(createLightCentered<TinyLight<GreenLight>>(Vec(11.5f, 338.f), module, IN_LED_3));
        addChild(createLightCentered<TinyLight<GreenLight>>(Vec(23.5f, 338.f), module, IN_LED_4));
        addChild(createLightCentered<TinyLight<GreenLight>>(Vec(11.5f, 352.f), module, IN_LED_5));
        addChild(createLightCentered<TinyLight<GreenLight>>(Vec(23.5f, 352.f), module, IN_LED_6));

        // Right cluster (Outputs) - Columns: 36.5, 48.5. Rows: 324, 338, 352
        addChild(createLightCentered<TinyLight<YellowLight>>(Vec(36.5f, 324.f), module, OUT_LED_1));
        addChild(createLightCentered<TinyLight<YellowLight>>(Vec(48.5f, 324.f), module, OUT_LED_2));
        addChild(createLightCentered<TinyLight<YellowLight>>(Vec(36.5f, 338.f), module, OUT_LED_3));
        addChild(createLightCentered<TinyLight<YellowLight>>(Vec(48.5f, 338.f), module, OUT_LED_4));
        addChild(createLightCentered<TinyLight<YellowLight>>(Vec(36.5f, 352.f), module, OUT_LED_5));
        addChild(createLightCentered<TinyLight<YellowLight>>(Vec(48.5f, 352.f), module, OUT_LED_6));
    }

    void appendContextMenu(Menu* menu) override {
        auto* m = dynamic_cast<VCVBridgeModule*>(module);
        if (!m) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("USB Serial Port"));

        auto ports = SerialPort::enumerate();
        if (ports.empty()) {
            menu->addChild(createMenuLabel("  (no ports found — plug in device)"));
        }
        for (const auto& p : ports) {
            bool active = (p == m->port_path && m->connected.load());
            menu->addChild(createMenuItem(
                (active ? "✓ " : "  ") + p, "",
                [m, p]() { m->start_serial(p); }
            ));
        }
        menu->addChild(createMenuItem("Disconnect", "", [m]() { m->stop_serial(); }));

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Knob → VCV Parameter Map"));

        for (int i = 0; i < 3; i++) {
            rack::engine::ParamHandle& h = m->knob_handles[i];
            std::string label = std::string(VCVBridgeModule::knob_names[i]) + ": ";
            label += (h.moduleId >= 0) ? h.text : "(unassigned — click to learn)";

            menu->addChild(createMenuItem(label, "", [m, i]() {
                // Clear old mapping and enter learn mode
                APP->engine->updateParamHandle(&m->knob_handles[i], -1, 0, false);
                m->knob_learn_idx = i;
            }));
            if (h.moduleId >= 0) {
                menu->addChild(createMenuItem(
                    std::string("  Clear ") + VCVBridgeModule::knob_names[i], "",
                    [m, i]() { APP->engine->updateParamHandle(&m->knob_handles[i], -1, 0, false); }
                ));
            }
        }
    }

    void onHoverKey(const HoverKeyEvent& e) override {
        auto* m = dynamic_cast<VCVBridgeModule*>(module);
        if (m && m->knob_learn_idx >= 0 && e.key == GLFW_KEY_ESCAPE && e.action == GLFW_PRESS) {
            m->knob_learn_idx = -1;
            e.consume(this);
        }
        rack::ModuleWidget::onHoverKey(e);
    }
};

// ── Registration ──────────────────────────────────────────────────────────────
Model* modelVCVBridge = createModel<VCVBridgeModule, VCVBridgeWidget>("VCVBridge");
