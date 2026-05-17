#include "Speedometer.h"

// Static instance pointer for ISR trampoline
Speedometer* Speedometer::_instance = nullptr;

// ── ISR ───────────────────────────────────────────────────────────────────────

void IRAM_ATTR Speedometer::_isrTrampoline() {
    Speedometer* self = _instance;
    if (!self) return;
    uint64_t t = (uint64_t)esp_timer_get_time();
    uint32_t next = (self->_ts_write + 1) & TS_BUF_MASK;
    if (next != self->_ts_read) {
        self->_ts_buf[self->_ts_write] = t;
        self->_ts_write = next;
    }
}

// ── begin() ───────────────────────────────────────────────────────────────────

bool Speedometer::begin() {
    _instance = this;

    // Single-slot speed mailbox — xQueueOverwrite never blocks writer,
    // xQueuePeek never blocks reader, always returns latest value.
    _speed_queue = xQueueCreate(1, sizeof(float));

    // Seed queue with 0.0 so getMPH() never reads uninitialised data
    float zero = 0.0f;
    xQueueOverwrite(_speed_queue, &zero);

    Serial.printf("[VSS] Pulses/mile:      %.2f\n", getPulsesPerMile());
    Serial.printf("[VSS] µs/pulse @ 1 mph: %.1f\n", getUSecPerPulseAt1MPH());
    Serial.printf("[VSS] EMA: slow=%.2f  fast=%.2f  threshold=%.1f mph\n",
                  EMA_ALPHA_SLOW, EMA_ALPHA_FAST, (float)EMA_FAST_THRESHOLD_MPH);

    pinMode((uint8_t)VSS_PULSE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(VSS_PULSE_PIN), _isrTrampoline, RISING);

#if defined(SPEEDOMETER_OUTPUT_CAN)
    if (!_twaiInit()) {
        Serial.println("[VSS] TWAI init FAILED");
        return false;
    }
    Serial.println("[VSS] TWAI OK");

#elif defined(SPEEDOMETER_EMBEDDED)
    // Spawn internal task pinned to VSS_TASK_CORE (default core 0)
    // so LVGL/UI on core 1 is never touched by speed processing.
    xTaskCreatePinnedToCore(
        _vssTask,
        "vss",
        VSS_TASK_STACK,
        this,
        VSS_TASK_PRIORITY,
        nullptr,
        VSS_TASK_CORE
    );
    Serial.println("[VSS] Task started");
#endif

    return true;
}

// ── update() — CAN mode, called from loop() ───────────────────────────────────

void Speedometer::update() {
#if defined(SPEEDOMETER_OUTPUT_CAN)
    _drainBuffer();

    // Stale check
    if (_last_ts > 0) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        if ((now_us - _last_ts) > VSS_STALE_TIMEOUT_US) {
            _resetState();
        }
    }

    uint32_t now_ms = (uint32_t)millis();
    if ((now_ms - _last_tx_ms) >= CAN_TX_INTERVAL_MS) {
        _last_tx_ms = now_ms;
        float mph = getMPH();
        _canSend(mph);
        Serial.printf("[VSS] %.2f mph\n", mph);
    }
#endif
}

// ── getMPH() ──────────────────────────────────────────────────────────────────

float Speedometer::getMPH() const {
    float mph = 0.0f;
    if (_speed_queue) {
        xQueuePeek(_speed_queue, &mph, 0);
    }
    return mph;
}

// ── Internal: drain timestamp ring buffer ─────────────────────────────────────

void Speedometer::_drainBuffer() {
    while (_ts_read != _ts_write) {
        uint64_t ts = _ts_buf[_ts_read];
        _ts_read = (_ts_read + 1) & TS_BUF_MASK;
        float spd = _processTimestamp(ts);
        if (spd > 0.0f) {
            float ema = _emaUpdate(spd);
            if (_speed_queue) {
                xQueueOverwrite(_speed_queue, &ema);
            }
        }
    }
}

// ── Internal: period averager → instantaneous speed ──────────────────────────

float Speedometer::_processTimestamp(uint64_t ts) {
    if (_last_ts == 0) {
        _last_ts = ts;
        return -1.0f;
    }

    uint64_t period_us = ts - _last_ts;
    _last_ts = ts;

    float instant = _USEC_PER_PULSE_AT_1MPH / (float)period_us;
    if (instant > VSS_MAX_PLAUSIBLE_MPH || instant < 0.1f) {
        return -1.0f;   // glitch — discard
    }

    _periods[_period_idx] = period_us;
    _period_idx = (_period_idx + 1) % PERIOD_AVG_DEPTH;
    if (_period_count < PERIOD_AVG_DEPTH) _period_count++;

    // Arithmetic mean of periods → correct average speed
    uint64_t sum = 0;
    for (uint32_t i = 0; i < _period_count; i++) sum += _periods[i];
    float avg_period = (float)(sum / _period_count);

    return _USEC_PER_PULSE_AT_1MPH / avg_period;
}

// ── Internal: dual-rate EMA ───────────────────────────────────────────────────

float Speedometer::_emaUpdate(float sample) {
    if (!_ema_init) {
        _ema      = sample;
        _ema_init = true;
        return _ema;
    }
    float err   = sample - _ema;
    float alpha = (fabsf(err) > EMA_FAST_THRESHOLD_MPH) ? EMA_ALPHA_FAST : EMA_ALPHA_SLOW;
    _ema += alpha * err;
    return _ema;
}

// ── Internal: reset all state (called on stale timeout) ───────────────────────

void Speedometer::_resetState() {
    _ema          = 0.0f;
    _ema_init     = false;
    _last_ts      = 0;
    _period_count = 0;
    _period_idx   = 0;
    if (_speed_queue) {
        float zero = 0.0f;
        xQueueOverwrite(_speed_queue, &zero);
    }
}

// ── Embedded mode: internal FreeRTOS task ────────────────────────────────────

#if defined(SPEEDOMETER_EMBEDDED)
void Speedometer::_vssTask(void* arg) {
    Speedometer* self = static_cast<Speedometer*>(arg);
    TickType_t   last_wake = xTaskGetTickCount();

    for (;;) {
        self->_drainBuffer();

        // Stale timeout check
        if (self->_last_ts > 0) {
            uint64_t now_us = (uint64_t)esp_timer_get_time();
            if ((now_us - self->_last_ts) > VSS_STALE_TIMEOUT_US) {
                self->_resetState();
            }
        }

        // 10ms tick — fast enough to keep the period averager current
        // without burning core 0. UI reads at its own 66ms render rate.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
#endif

// ── CAN mode: TWAI init + transmit ───────────────────────────────────────────

#if defined(SPEEDOMETER_OUTPUT_CAN)
bool Speedometer::_twaiInit() {
    // M5Stack CAN Unit uses TJA1051T/3 (3.3V compatible, no S-pin needed)
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    return (twai_driver_install(&g_cfg, &t_cfg, &f_cfg) == ESP_OK) &&
           (twai_start() == ESP_OK);
}

void Speedometer::_canSend(float mph) {
    uint16_t enc = (uint16_t)(fmaxf(mph, 0.0f) * 100.0f + 0.5f);
    twai_message_t msg  = {};
    msg.identifier       = 0x257;
    msg.data_length_code = 2;
    msg.data[0]          = (enc >> 8) & 0xFF;
    msg.data[1]          =  enc       & 0xFF;
    twai_transmit(&msg, 0);
}
#endif
