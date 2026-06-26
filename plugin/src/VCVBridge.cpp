/**
 * VCVBridge.cpp  —  MTM VCV Bridge module
 *
 * Bidirectional USB CDC bridge between Workshop Computer and VCV Rack.
 *
 * Ports exposed:
 *   FROM HARDWARE (outputs):
 *     Audio 1/2, CV 1/2, Pulse 1/2
 *     Knob Main/X/Y (0–10 V), Switch (0/5/10 V)
 *   TO HARDWARE (inputs):
 *     Audio 1/2, CV 1/2, Pulse 1/2
 *
 * Knob parameter mapping:
 *   Right-click the module → "Map Knob [Main/X/Y] to parameter"
 *   Then click any VCV parameter to bind it. The physical knob will
 *   directly drive that parameter. Mappings saved in patch JSON.
 *
 * Sample rate:
 *   RP2040 is clock master at 48 kHz.
 *   VCV adapts via linear interpolation (phase accumulator).
 *
 * Threading:
 *   Serial thread: reads/writes USB CDC, fills/drains ring buffers.
 *   VCV audio thread: process() — SRC + ring buffer exchange.
 */

#include "plugin.hpp"
#include "SerialPort.hpp"
#include "BridgeProtocol.hpp"

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <cassert>

using namespace rack;

// ── Ring buffer (lock-free single-producer / single-consumer) ─────────────────

template<typename T, size_t N>
struct RingBuf {
    static_assert((N & (N-1)) == 0, "N must be power of 2");
    std::atomic<size_t> head{0}, tail{0};
    T buf[N];

    bool push(const T& v) {
        size_t t = tail.load(std::memory_order_relaxed);
        size_t next = (t + 1) & (N - 1);
        if (next == head.load(std::memory_order_acquire)) return false; // full
        buf[t] = v;
        tail.store(next, std::memory_order_release);
        return true;
    }
    bool pop(T& v) {
        size_t h = head.load(std::memory_order_relaxed);
        if (h == tail.load(std::memory_order_acquire)) return false; // empty
        v = buf[h];
        head.store((h + 1) & (N - 1), std::memory_order_release);
        return true;
    }
    size_t size() const {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_acquire);
        return (t - h) & (N - 1);
    }
    void clear() { head.store(0); tail.store(0); }
};

// ── Per-block ring buffers (8 blocks deep ≈ 10 ms jitter absorption) ──────────
static constexpr size_t RING_DEPTH = 8;

struct RxBlock { D2H_Frame frame; };
struct TxBlock { H2D_Frame frame; };

// ── Param IDs ─────────────────────────────────────────────────────────────────

enum ParamIds {
    // No panel params — all interaction via ports or context menu
    NUM_PARAMS
};

enum InputIds {
    // TO HARDWARE
    AUDIO1_INPUT, AUDIO2_INPUT,
    CV1_INPUT,    CV2_INPUT,
    PULSE1_INPUT, PULSE2_INPUT,
    NUM_INPUTS
};

enum OutputIds {
    // FROM HARDWARE
    AUDIO1_OUTPUT, AUDIO2_OUTPUT,
    CV1_OUTPUT,    CV2_OUTPUT,
    PULSE1_OUTPUT, PULSE2_OUTPUT,
    KNOB_MAIN_OUTPUT, KNOB_X_OUTPUT, KNOB_Y_OUTPUT,
    SWITCH_OUTPUT,
    NUM_OUTPUTS
};

enum LightIds {
    STATUS_LIGHT,    // green = connected, red = error
    RX_LIGHT,        // blinks on RX activity
    TX_LIGHT,        // blinks on TX activity
    NUM_LIGHTS
};

// ── VCVBridge module ──────────────────────────────────────────────────────────

struct VCVBridgeModule : Module {
    // ── Port/param definitions
    VCVBridgeModule() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configInput(AUDIO1_INPUT, "Audio 1 → Hardware");
        configInput(AUDIO2_INPUT, "Audio 2 → Hardware");
        configInput(CV1_INPUT,    "CV 1 → Hardware");
        configInput(CV2_INPUT,    "CV 2 → Hardware");
        configInput(PULSE1_INPUT, "Pulse 1 → Hardware");
        configInput(PULSE2_INPUT, "Pulse 2 → Hardware");

        configOutput(AUDIO1_OUTPUT, "Audio 1 from Hardware");
        configOutput(AUDIO2_OUTPUT, "Audio 2 from Hardware");
        configOutput(CV1_OUTPUT,    "CV 1 from Hardware");
        configOutput(CV2_OUTPUT,    "CV 2 from Hardware");
        configOutput(PULSE1_OUTPUT, "Pulse 1 from Hardware (gate)");
        configOutput(PULSE2_OUTPUT, "Pulse 2 from Hardware (gate)");
        configOutput(KNOB_MAIN_OUTPUT, "Knob Main position (0–10 V)");
        configOutput(KNOB_X_OUTPUT,    "Knob X position (0–10 V)");
        configOutput(KNOB_Y_OUTPUT,    "Knob Y position (0–10 V)");
        configOutput(SWITCH_OUTPUT,    "Switch Z position (0 / 5 / 10 V)");

        // Knob parameter mapping handles
        for (int i = 0; i < 3; i++) {
            knob_handles[i].text = knob_names[i];
            APP->engine->addParamHandle(&knob_handles[i]);
        }
    }

    ~VCVBridgeModule() override {
        stop_serial();
        for (int i = 0; i < 3; i++)
            APP->engine->removeParamHandle(&knob_handles[i]);
    }

    // ── Serial config (set via context menu / saved to JSON)
    std::string port_path;
    std::atomic<bool> connected{false};
    std::atomic<bool> run_thread{false};

    // ── Ring buffers (serial thread ↔ audio thread)
    RingBuf<RxBlock, RING_DEPTH> rx_ring;
    RingBuf<TxBlock, RING_DEPTH> tx_ring;

    // ── SRC state (RP2040 master at 48 kHz, VCV follows)
    double hw_phase  = 0.0;     // phase accumulator for HW→VCV resampling
    double vcv_phase = 0.0;     // phase accumulator for VCV→HW resampling

    // Current and previous HW samples for interpolation
    float prev_rx_a1 = 0.f, curr_rx_a1 = 0.f;
    float prev_rx_a2 = 0.f, curr_rx_a2 = 0.f;
    bool  curr_rx_p1 = false,  curr_rx_p2 = false;
    float curr_rx_cv1= 0.f,   curr_rx_cv2 = 0.f;
    float curr_rx_km = 0.f,   curr_rx_kx  = 0.f;
    float curr_rx_ky = 0.f,   curr_rx_sw  = 0.f;

    // Current HW block being consumed, and sample index within it
    D2H_Frame rx_cur_frame = {};
    int rx_frame_sample = BRIDGE_BLOCK_SIZE;   // force first fetch

    // TX accumulation (VCV samples → HW block)
    H2D_Frame tx_cur_frame = {};
    int tx_frame_sample = 0;
    uint16_t tx_block_idx = 0;
    double vcv_src_phase = 0.0;   // resampling phase for VCV→HW

    // ── Serial thread
    std::thread serial_thread;

    // ── Knob parameter mapping
    static constexpr const char* knob_names[3] = {"Knob Main", "Knob X", "Knob Y"};
    ParamHandle knob_handles[3];
    int knob_learn_idx = -1;   // which knob is in learn mode (-1 = none)

    // Latest knob values from hardware (0..4095)
    std::atomic<uint16_t> hw_knob[3]{0, 0, 0};
    std::atomic<uint8_t>  hw_switch{1};

    // ── Light blink timers
    int rx_blink = 0, tx_blink = 0;

    // ── Start/stop serial thread
    void start_serial(const std::string& path) {
        stop_serial();
        port_path  = path;
        run_thread = true;
        rx_ring.clear();
        tx_ring.clear();
        serial_thread = std::thread([this]() { serial_worker(); });
    }

    void stop_serial() {
        run_thread = false;
        connected  = false;
        if (serial_thread.joinable()) serial_thread.join();
    }

    // ── Serial worker thread ──────────────────────────────────────────────────
    void serial_worker() {
        SerialPort port;
        uint8_t parse_buf[BRIDGE_D2H_SIZE];
        int     parse_pos    = 0;
        bool    sync_found   = false;

        auto try_connect = [&]() -> bool {
            if (port.is_open()) port.close();
            if (!port.open(port_path)) return false;
            parse_pos  = 0;
            sync_found = false;
            return true;
        };

        while (run_thread) {
            if (!port.is_open()) {
                connected = false;
                if (!try_connect()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                connected = true;
            }

            // ── TX: send pending blocks to device ────────────────────────
            TxBlock tb;
            while (tx_ring.pop(tb)) {
                const uint8_t* d = reinterpret_cast<const uint8_t*>(&tb.frame);
                if (!port.write(d, BRIDGE_H2D_SIZE)) {
                    port.close();
                    connected = false;
                    break;
                }
                tx_blink = 4;   // signal activity
            }

            // ── RX: receive bytes from device ────────────────────────────
            uint8_t buf[256];
            int n = port.read(buf, sizeof(buf));
            if (n < 0) {
                port.close();
                connected = false;
                continue;
            }

            for (int i = 0; i < n; i++) {
                uint8_t b = buf[i];

                if (!sync_found) {
                    if (parse_pos == 0 && b == BRIDGE_D2H_SYNC_0) {
                        parse_buf[parse_pos++] = b;
                    } else if (parse_pos == 1 && b == BRIDGE_D2H_SYNC_1) {
                        parse_buf[parse_pos++] = b;
                        sync_found = true;
                    } else {
                        parse_pos = 0;
                    }
                    continue;
                }

                parse_buf[parse_pos++] = b;
                if (parse_pos == BRIDGE_D2H_SIZE) {
                    const D2H_Frame* f = reinterpret_cast<const D2H_Frame*>(parse_buf);
                    if (f->crc == bridge_frame_crc(*f)) {
                        RxBlock rb;
                        rb.frame = *f;
                        // Update hw_knob/switch atomically for parameter mapping
                        hw_knob[0] = f->knob_main;
                        hw_knob[1] = f->knob_x;
                        hw_knob[2] = f->knob_y;
                        hw_switch  = f->switch_z;
                        rx_ring.push(rb);   // drops if ring full (audio thread slow)
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
        const double hw_rate  = BRIDGE_SAMPLE_RATE;   // 48000
        const double vcv_rate = args.sampleRate;
        // Step size: how many HW samples per VCV sample
        const double step = hw_rate / vcv_rate;

        // ── HW → VCV resampling ───────────────────────────────────────────
        hw_phase += step;
        while (hw_phase >= 1.0) {
            hw_phase -= 1.0;

            // Advance to next hardware sample
            rx_frame_sample++;
            if (rx_frame_sample >= BRIDGE_BLOCK_SIZE) {
                // Need next block from ring buffer
                RxBlock rb;
                if (rx_ring.pop(rb)) {
                    rx_cur_frame    = rb.frame;
                    rx_blink        = 4;
                }
                rx_frame_sample = 0;
            }

            int si = rx_frame_sample;
            prev_rx_a1 = curr_rx_a1;
            prev_rx_a2 = curr_rx_a2;
            curr_rx_a1  = bridge_to_volts(rx_cur_frame.audio1[si]);
            curr_rx_a2  = bridge_to_volts(rx_cur_frame.audio2[si]);
            curr_rx_p1  = bridge_pulse_get(rx_cur_frame.pulse1, si);
            curr_rx_p2  = bridge_pulse_get(rx_cur_frame.pulse2, si);
            // CV/knobs: update once per block (si==0)
            if (si == 0) {
                curr_rx_cv1 = bridge_to_volts(rx_cur_frame.cv1);
                curr_rx_cv2 = bridge_to_volts(rx_cur_frame.cv2);
                curr_rx_km  = bridge_knob_to_volts(rx_cur_frame.knob_main);
                curr_rx_kx  = bridge_knob_to_volts(rx_cur_frame.knob_x);
                curr_rx_ky  = bridge_knob_to_volts(rx_cur_frame.knob_y);
                curr_rx_sw  = bridge_switch_to_volts(rx_cur_frame.switch_z);
            }
        }

        // Linear interpolation between previous and current HW samples
        float frac = (float)hw_phase;   // 0..1
        float a1_out = prev_rx_a1 + frac * (curr_rx_a1 - prev_rx_a1);
        float a2_out = prev_rx_a2 + frac * (curr_rx_a2 - prev_rx_a2);

        outputs[AUDIO1_OUTPUT].setVoltage(a1_out);
        outputs[AUDIO2_OUTPUT].setVoltage(a2_out);
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
            ParamHandle& h = knob_handles[i];
            if (h.moduleId < 0) continue;
            Module* m = APP->engine->getModule(h.moduleId);
            if (!m) continue;
            int pid = h.paramId;
            if (pid < 0 || pid >= (int)m->params.size()) continue;
            ParamQuantity* pq = m->paramQuantities[pid];
            if (!pq) continue;
            float norm = hw_knob[i] / 4095.f;
            float val  = pq->minValue + norm * (pq->maxValue - pq->minValue);
            m->params[pid].setValue(val);
        }

        // ── VCV → HW resampling ───────────────────────────────────────────
        vcv_src_phase += step;
        while (vcv_src_phase >= 1.0) {
            vcv_src_phase -= 1.0;

            // Read one VCV sample → HW block slot
            int si = tx_frame_sample;
            tx_cur_frame.audio1[si] = bridge_from_volts(inputs[AUDIO1_INPUT].getVoltage());
            tx_cur_frame.audio2[si] = bridge_from_volts(inputs[AUDIO2_INPUT].getVoltage());
            bridge_pulse_set(tx_cur_frame.pulse1, si,
                inputs[PULSE1_INPUT].getVoltage() >= 1.f);
            bridge_pulse_set(tx_cur_frame.pulse2, si,
                inputs[PULSE2_INPUT].getVoltage() >= 1.f);

            tx_frame_sample++;
            if (tx_frame_sample == BRIDGE_BLOCK_SIZE) {
                // Finalise block — CV once per block
                tx_cur_frame.sync[0]   = BRIDGE_H2D_SYNC_0;
                tx_cur_frame.sync[1]   = BRIDGE_H2D_SYNC_1;
                tx_cur_frame.block_idx = tx_block_idx++;
                tx_cur_frame.cv1 = bridge_from_volts(inputs[CV1_INPUT].getVoltage());
                tx_cur_frame.cv2 = bridge_from_volts(inputs[CV2_INPUT].getVoltage());
                tx_cur_frame.crc = bridge_frame_crc(tx_cur_frame);

                TxBlock tb;
                tb.frame = tx_cur_frame;
                tx_ring.push(tb);   // serial thread picks this up

                // Reset for next block
                tx_frame_sample = 0;
                memset(&tx_cur_frame, 0, sizeof(tx_cur_frame));
                bridge_pulse_clear(tx_cur_frame.pulse1);
                bridge_pulse_clear(tx_cur_frame.pulse2);
                tx_blink = 4;
            }
        }

        // ── Status lights ─────────────────────────────────────────────────
        lights[STATUS_LIGHT + 0].setBrightness(connected ?  1.f : 0.f);   // green
        lights[STATUS_LIGHT + 1].setBrightness(!connected ? 0.5f : 0.f);  // red (dim)

        if (rx_blink > 0) { lights[RX_LIGHT].setBrightness(1.f); rx_blink--; }
        else               { lights[RX_LIGHT].setBrightness(0.f); }

        if (tx_blink > 0) { lights[TX_LIGHT].setBrightness(1.f); tx_blink--; }
        else               { lights[TX_LIGHT].setBrightness(0.f); }
    }

    // ── JSON serialisation ────────────────────────────────────────────────────
    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "port_path", json_string(port_path.c_str()));

        // Knob mappings
        json_t* maps = json_array();
        for (int i = 0; i < 3; i++) {
            json_t* m = json_object();
            json_object_set_new(m, "moduleId",  json_integer(knob_handles[i].moduleId));
            json_object_set_new(m, "paramId",   json_integer(knob_handles[i].paramId));
            json_array_append_new(maps, m);
        }
        json_object_set_new(root, "knob_maps", maps);
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* pp = json_object_get(root, "port_path");
        if (pp) port_path = json_string_value(pp);

        json_t* maps = json_object_get(root, "knob_maps");
        if (maps) {
            for (int i = 0; i < 3 && i < (int)json_array_size(maps); i++) {
                json_t* m  = json_array_get(maps, i);
                int64_t mid = json_integer_value(json_object_get(m, "moduleId"));
                int64_t pid = json_integer_value(json_object_get(m, "paramId"));
                if (mid >= 0 && pid >= 0) {
                    APP->engine->updateParamHandle(&knob_handles[i], (int64_t)mid, (int)pid, false);
                }
            }
        }

        // Auto-connect if a port was saved
        if (!port_path.empty()) {
            start_serial(port_path);
        }
    }
};

// ── Widget ────────────────────────────────────────────────────────────────────

struct VCVBridgeWidget : ModuleWidget {
    VCVBridgeWidget(VCVBridgeModule* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/VCVBridge.svg")));

        // Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Layout uses mm2px for consistent positioning
        // Panel is 14HP = 71.12mm wide, 128.5mm tall

        // Status lights (top area)
        addChild(createLightCentered<SmallLight<GreenRedLight>>(
            mm2px(Vec(8.f, 14.f)), module, VCVBridgeModule::STATUS_LIGHT));
        addChild(createLightCentered<TinyLight<GreenLight>>(
            mm2px(Vec(14.f, 14.f)), module, VCVBridgeModule::RX_LIGHT));
        addChild(createLightCentered<TinyLight<YellowLight>>(
            mm2px(Vec(19.f, 14.f)), module, VCVBridgeModule::TX_LIGHT));

        // ── FROM HARDWARE outputs ─────────────────────────────────────────
        // Row 1: Audio 1 & 2
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.f, 34.f)), module, VCVBridgeModule::AUDIO1_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.f, 34.f)), module, VCVBridgeModule::AUDIO2_OUTPUT));
        // Row 2: CV 1 & 2
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.f, 48.f)), module, VCVBridgeModule::CV1_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.f, 48.f)), module, VCVBridgeModule::CV2_OUTPUT));
        // Row 3: Pulse 1 & 2
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.f, 62.f)), module, VCVBridgeModule::PULSE1_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.f, 62.f)), module, VCVBridgeModule::PULSE2_OUTPUT));
        // Row 4: Knobs & Switch
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec( 8.f, 76.f)), module, VCVBridgeModule::KNOB_MAIN_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.f, 76.f)), module, VCVBridgeModule::KNOB_X_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(28.f, 76.f)), module, VCVBridgeModule::KNOB_Y_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(38.f, 76.f)), module, VCVBridgeModule::SWITCH_OUTPUT));

        // ── TO HARDWARE inputs ────────────────────────────────────────────
        // Row 5: Audio 1 & 2
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.f, 96.f)),  module, VCVBridgeModule::AUDIO1_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(22.f, 96.f)),  module, VCVBridgeModule::AUDIO2_INPUT));
        // Row 6: CV 1 & 2
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.f, 110.f)), module, VCVBridgeModule::CV1_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(22.f, 110.f)), module, VCVBridgeModule::CV2_INPUT));
        // Row 7: Pulse 1 & 2
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.f, 124.f)), module, VCVBridgeModule::PULSE1_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(22.f, 124.f)), module, VCVBridgeModule::PULSE2_INPUT));
    }

    // ── Context menu ─────────────────────────────────────────────────────────
    void appendContextMenu(Menu* menu) override {
        auto* m = dynamic_cast<VCVBridgeModule*>(module);
        if (!m) return;

        menu->addChild(new MenuSeparator);

        // Serial port selector
        menu->addChild(createMenuLabel("USB Serial Port"));

        auto ports = SerialPort::enumerate();
        if (ports.empty()) {
            menu->addChild(createMenuLabel("  (no ports found)"));
        }
        for (const auto& p : ports) {
            bool active = (p == m->port_path && m->connected);
            menu->addChild(createMenuItem(
                (active ? "✓ " : "  ") + p,
                "",
                [m, p]() { m->start_serial(p); }
            ));
        }

        menu->addChild(createMenuItem("Disconnect", "",
            [m]() { m->stop_serial(); }));

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Knob → VCV Parameter Map"));

        for (int i = 0; i < 3; i++) {
            ParamHandle& h = m->knob_handles[i];
            std::string label = std::string(VCVBridgeModule::knob_names[i]) + ": ";
            if (h.moduleId >= 0) {
                label += h.text.empty() ? "mapped" : h.text;
            } else {
                label += "(unassigned)";
            }

            menu->addChild(createMenuItem(label, "click to learn",
                [m, i]() {
                    // Enter learn mode — next param interaction binds this knob
                    m->knob_learn_idx = i;
                    APP->engine->updateParamHandle(&m->knob_handles[i], -1, 0, false);
                }
            ));

            if (h.moduleId >= 0) {
                menu->addChild(createMenuItem("  Clear mapping", "",
                    [m, i]() {
                        APP->engine->updateParamHandle(&m->knob_handles[i], -1, 0, false);
                    }
                ));
            }
        }
    }

    // Intercept param touch for learn mode
    void onHoverKey(const HoverKeyEvent& e) override {
        auto* m = dynamic_cast<VCVBridgeModule*>(module);
        if (m && m->knob_learn_idx >= 0 && e.key == GLFW_KEY_ESCAPE) {
            m->knob_learn_idx = -1;
            e.consume(this);
        }
        ModuleWidget::onHoverKey(e);
    }
};

// ── Registration ─────────────────────────────────────────────────────────────
Model* modelVCVBridge = createModel<VCVBridgeModule, VCVBridgeWidget>("VCVBridge");
