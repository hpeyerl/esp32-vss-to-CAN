#include <Arduino.h>
#include "Speedometer.h"

Speedometer vss;

// ══════════════════════════════════════════════════════════════════════════════
// CAN STANDALONE MODE  (-DSPEEDOMETER_OUTPUT_CAN)
//
// Drop this main.cpp into a bare ESP32 project.
// vss.update() in loop() handles buffer draining, EMA, and CAN TX.
// ══════════════════════════════════════════════════════════════════════════════

#if defined(SPEEDOMETER_OUTPUT_CAN)

void setup() {
    Serial.begin(115200);
    if (!vss.begin()) {
        Serial.println("VSS init failed — halting");
        while (true) delay(1000);
    }
}

void loop() {
    vss.update();   // drains ISR buffer, updates EMA, transmits CAN on interval
}

// ══════════════════════════════════════════════════════════════════════════════
// EMBEDDED MODE  (-DSPEEDOMETER_EMBEDDED)
//
// In your Tab5 dashboard project, call vss.begin() from setup() and
// vss.getMPH() from ui_task (or wherever you render the speedo gauge).
// The internal VSS task runs on core 0; your UI stays on core 1.
//
// This main.cpp is a minimal stub — in the real Tab5 project, setup() and
// the ui_task already exist. Just add vss.begin() to setup() and
// vss.getMPH() wherever you update the gauge widget.
// ══════════════════════════════════════════════════════════════════════════════

#elif defined(SPEEDOMETER_EMBEDDED)

// Example: how the dashboard project consumes speed
static void ui_task(void* arg) {
    for (;;) {
        float mph = vss.getMPH();

        // Replace with your actual LVGL gauge update, e.g.:
        // lv_arc_set_value(ui_SpeedArc, (int)mph);
        // lv_label_set_text_fmt(ui_SpeedLabel, "%.0f", mph);
        Serial.printf("[UI] Speed: %.2f mph\n", mph);

        vTaskDelay(pdMS_TO_TICKS(66));   // ~15 Hz UI refresh
    }
}

void setup() {
    Serial.begin(115200);
    vss.begin();   // spawns VSS task on core 0

    // UI task on core 1 — matches Tab5 dashboard task structure
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, nullptr, 4, nullptr, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));   // loop() unused; tasks own everything
}

#else
  #error "Define either SPEEDOMETER_OUTPUT_CAN or SPEEDOMETER_EMBEDDED in build_flags"
#endif
