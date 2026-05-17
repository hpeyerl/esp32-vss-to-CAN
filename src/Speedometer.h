#pragma once

/**
 * Speedometer.h
 *
 * Vehicle speed from a pulse train sensor → filtered MPH.
 *
 * Compile-time output modes (set via build_flags in platformio.ini):
 *
 *   -DSPEEDOMETER_OUTPUT_CAN
 *       Transmits CAN frame ID 0x257 (uint16_t big-endian, MPH*100) via
 *       M5Stack CAN Unit (TJA1051T/3, GROVE port or direct wire).
 *       Calls begin()/update() from main loop — no internal task created.
 *
 *   -DSPEEDOMETER_EMBEDDED
 *       No CAN. Creates an internal FreeRTOS task pinned to VSS_TASK_CORE.
 *       Dashboard reads speed with getMPH() from any task/core — thread-safe.
 *
 * Usage (CAN standalone):
 *   Speedometer vss;
 *   void setup() { vss.begin(); }
 *   void loop()  { vss.update(); }   // handles TX interval internally
 *
 * Usage (embedded):
 *   Speedometer vss;
 *   void setup() { vss.begin(); }
 *   // from ui_task:
 *   float mph = vss.getMPH();
 */

#include <Arduino.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#if defined(SPEEDOMETER_OUTPUT_CAN)
  #include "driver/twai.h"
#endif

// ──────────────────────────────────────────────────────────────────────────────
// VEHICLE CONFIGURATION — edit these for your application
// ──────────────────────────────────────────────────────────────────────────────

#ifndef TIRE_CIRCUMFERENCE_INCHES
  #define TIRE_CIRCUMFERENCE_INCHES   103.67f   // π × 33" tire
#endif

#ifndef DIFF_RATIO
  #define DIFF_RATIO                  4.10f
#endif

#ifndef PULSES_PER_REV
  #define PULSES_PER_REV              4
#endif

// ──────────────────────────────────────────────────────────────────────────────
// PIN CONFIGURATION
// ──────────────────────────────────────────────────────────────────────────────

#ifndef VSS_PULSE_PIN
  #define VSS_PULSE_PIN               GPIO_NUM_34
#endif

// M5Stack CAN Unit default pins (GROVE connector wired to TJA1051T/3)
// Override in platformio.ini build_flags if using direct wire or different port
#ifndef CAN_TX_PIN
  #define CAN_TX_PIN                  GPIO_NUM_5
#endif
#ifndef CAN_RX_PIN
  #define CAN_RX_PIN                  GPIO_NUM_4
#endif

// ──────────────────────────────────────────────────────────────────────────────
// DAMPENING CONFIGURATION
// ──────────────────────────────────────────────────────────────────────────────

#ifndef EMA_ALPHA_SLOW
  #define EMA_ALPHA_SLOW              0.08f
#endif
#ifndef EMA_ALPHA_FAST
  #define EMA_ALPHA_FAST              0.40f
#endif
#ifndef EMA_FAST_THRESHOLD_MPH
  #define EMA_FAST_THRESHOLD_MPH      2.0f
#endif
#ifndef PERIOD_AVG_DEPTH
  #define PERIOD_AVG_DEPTH            4
#endif

// ──────────────────────────────────────────────────────────────────────────────
// TIMING
// ──────────────────────────────────────────────────────────────────────────────

#ifndef CAN_TX_INTERVAL_MS
  #define CAN_TX_INTERVAL_MS          100       // 10 Hz CAN transmit rate
#endif
#ifndef VSS_STALE_TIMEOUT_US
  #define VSS_STALE_TIMEOUT_US        2000000ULL
#endif
#ifndef VSS_MAX_PLAUSIBLE_MPH
  #define VSS_MAX_PLAUSIBLE_MPH       160.0f
#endif

// ──────────────────────────────────────────────────────────────────────────────
// DERIVED CONSTANTS (compile-time)
// ──────────────────────────────────────────────────────────────────────────────

#define _PULSES_PER_MILE  ((float)((63360.0f / TIRE_CIRCUMFERENCE_INCHES) * DIFF_RATIO * PULSES_PER_REV))
#define _USEC_PER_PULSE_AT_1MPH  (3600000000.0f / _PULSES_PER_MILE)

// ──────────────────────────────────────────────────────────────────────────────

class Speedometer {
public:
    Speedometer() = default;

    /**
     * begin() — call once from setup().
     *
     * CAN mode:      initialises TWAI peripheral, attaches ISR.
     * Embedded mode: attaches ISR, spawns internal FreeRTOS task on VSS_TASK_CORE.
     *
     * Returns false if TWAI init fails (CAN mode only).
     */
    bool begin();

    /**
     * update() — CAN mode only. Call from loop() or a dedicated task.
     * Drains timestamp buffer, updates EMA, transmits CAN frame on interval.
     * No-op in embedded mode (internal task handles this).
     */
    void update();

    /**
     * getMPH() — embedded mode primary API. Thread-safe, non-blocking.
     * Also valid in CAN mode if you want to read locally.
     */
    float getMPH() const;

    /**
     * getPulsesPerMile() / getUSecPerPulseAt1MPH() — diagnostic / tuning.
     */
    float getPulsesPerMile()      const { return _PULSES_PER_MILE; }
    float getUSecPerPulseAt1MPH() const { return _USEC_PER_PULSE_AT_1MPH; }

private:
    // ── ISR timestamp ring buffer ─────────────────────────────────────────────
    static constexpr uint32_t TS_BUF_MASK = 15u;   // 16 entries, power-of-2
    volatile uint64_t _ts_buf[TS_BUF_MASK + 1];
    volatile uint32_t _ts_write = 0;
    volatile uint32_t _ts_read  = 0;

    // ── Period averager ───────────────────────────────────────────────────────
    uint64_t _periods[PERIOD_AVG_DEPTH] = {};
    uint32_t _period_idx   = 0;
    uint32_t _period_count = 0;
    uint64_t _last_ts      = 0;

    // ── EMA ───────────────────────────────────────────────────────────────────
    float _ema      = 0.0f;
    bool  _ema_init = false;

    // ── Shared output (embedded mode: written by VSS task, read by UI task) ──
    QueueHandle_t _speed_queue = nullptr;   // single-slot mailbox, float

    // ── Internal methods ──────────────────────────────────────────────────────
    void     _drainBuffer();
    float    _processTimestamp(uint64_t ts);
    float    _emaUpdate(float sample);
    void     _resetState();

#if defined(SPEEDOMETER_OUTPUT_CAN)
    bool     _twaiInit();
    void     _canSend(float mph);
    uint32_t _last_tx_ms = 0;
#endif

#if defined(SPEEDOMETER_EMBEDDED)
    static void _vssTask(void* arg);
#endif

    // ISR trampoline — attachInterrupt needs a plain function pointer,
    // so we store the instance pointer in a static and bounce through it.
    static Speedometer* _instance;
    static void IRAM_ATTR _isrTrampoline();
};
