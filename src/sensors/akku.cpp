/** ---------------------------------------------------------------------------
 *
 * @file akku.cpp
 * @brief Implementierung des Akku-Überwachungs-Moduls
 *
 * ---------------------------------------------------------------------------
 *
 * Änderungen:
 * ---------------------------------------------------------------------------
 * 2026-06-07 08:15 - Fix "Pin is not configured as analog channel":
 *                    analogSetPinAttenuation() braucht auf ESP32-C6 / neuere
 *                    Arduino-Cores einen vorherigen analogRead*() um den Pin
 *                    intern als ADC-Channel zu registrieren. Daher: erst
 *                    Dummy-Read, dann Attenuation setzen.
 *
 * 2026-06-06 21:03 - Farbige Log-Tags (log_colors.h) eingebunden
 *
 * 2026-06-06 20:58 - pinMode(ANALOG) auch in akku_measure() (Core kann Pin-State zuruecksetzen)
 *
 * 2026-06-06 20:27 - pinMode(ANALOG) + ADC_ATTENDB_MAX für ESP32-C6 / IDF 5.x
 *
 * ---------------------------------------------------------------------------
 *
 * Hardware:
 *   18650 Li-Ion (2000 mAh, Vmax=4.2 V, Vmin=3.0 V)
 *   → ADC-Pin (GP4) hinter Spannungsteiler 1:2 (V_adc = V_akku / 2)
 *
 * ADC-Konfiguration:
 *   - 12-Bit-Auflösung (0-4095)
 *   - 11-dB-Attenuation (auf ESP32-C6: ~0…3.1 V Messbereich)
 *     → V_akku_max=4.2V / 2 = 2.1V passt komfortabel hinein
 *   - analogReadMilliVolts() statt analogRead()
 *     (kalibriert per eFuse, auf ESP32-C6 zuverlässiger - siehe Espressif
 *     Issue #11324)
 *
 * Messstrategie:
 *   1. 16 ADC-Samples in kurzem Abstand sammeln
 *   2. Mittelwert dieser Samples bilden
 *   3. Werte, die mehr als OUTLIER_THRESHOLD_MV vom Mittelwert
 *      abweichen, als Ausreißer verwerfen
 *   4. Mittelwert der verbleibenden Werte als Endergebnis
 *
 * --------------------------------------------------------------------------- */

#include "akku.h"

#include <Arduino.h>
#include <math.h>
#include "../cpu/pinout.h"
#include "../cpu/log_colors.h"
#include "../services/gxLog.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "akku";


// ---------------------------------------------------------------------------
// Entladekurve 18650 Li-Ion (anonymous namespace = file-private)
// ---------------------------------------------------------------------------

namespace {

typedef struct {
    uint16_t voltage;
    uint8_t percent;
} soc_point_t;

/**
 * Lookup-Tabelle: Leerlaufspannung → State of Charge.
 *
 * Werte gemittelt aus den Datenblättern gängiger 18650-Zellen
 * (Samsung INR18650-25R, LG MJ1, Panasonic NCR18650B) bei Entladung
 * mit <0.5C - das passt zu unserer Anwendung (Sensor-Node,
 * Durchschnittsstrom <50 mA).
 *
 * Wichtig: Im Bereich 3.7-3.9 V ist die Kurve sehr flach (typisch für
 * Li-Ion). Hier ist die SOC-Schätzung am ungenauesten - das ist
 * physikalisch bedingt, nicht durch die Tabelle.
 */
// constexpr soc_point_t SOC_CURVE[] = {
//     { 4.20f, 100 },
//     { 4.10f,  90 },
//     { 4.00f,  80 },
//     { 3.95f,  70 },
//     { 3.90f,  60 },
//     { 3.85f,  50 },
//     { 3.80f,  40 },
//     { 3.75f,  30 },
//     { 3.70f,  22 },
//     { 3.65f,  15 },
//     { 3.60f,  10 },
//     { 3.50f,   5 },
//     { 3.40f,   3 },
//     { 3.30f,   2 },
//     { 3.20f,   1 },
//     { 3.00f,   0 },
// };

constexpr soc_point_t SOC_CURVE[] = {
    { 3000,   0 },
    { 3300,  10 },
    { 3450,  20 },
    { 3550,  40 },
    { 3650,  60 },
    { 3750,  70 },
    { 3850,  80 },
    { 4100,  90 },
    { 4200, 100 },
};

constexpr size_t SOC_CURVE_SIZE = sizeof(SOC_CURVE) / sizeof(SOC_CURVE[0]);

/** Mikrosekunden Pause zwischen einzelnen ADC-Samples */
constexpr uint32_t SAMPLE_DELAY_US = 200;

/**
 * Schwelle für Ausreißererkennung (in mV, bezogen auf ADC-Wert).
 *
 * Das ESP32-C6 ADC-Rauschen liegt bei typ. 5-15 mV. 30 mV ist eine
 * großzügige Schwelle, die echtes Rauschen toleriert, aber grobe
 * Spikes (z. B. durch BLE-/WiFi-Sendeaktivität) wegfiltert.
 */


}  // anonymous namespace

// ---------------------------------------------------------------------------
// Modul-interner Zustand
// ---------------------------------------------------------------------------

static bool s_initialized = false;

// ---------------------------------------------------------------------------
// Hilfsfunktion: Spannung → Ladezustand (lineare Interpolation)  mit Temperatur-Kompensation
// ---------------------------------------------------------------------------

uint8_t akku_voltage_to_percent_temp_comp(uint16_t mv, int16_t temp_c)
{
    const unsigned n = sizeof(SOC_CURVE) / sizeof(SOC_CURVE[0]);
    // const int16_t ref_c = 25;
    // const int16_t comp_mv_per_c = 2;   // ca. 2 mV/K pro Zelle als grobe Korrektur
    const int16_t ref_c = 20;
    const int16_t comp_mv_per_c = 1;   // ca. 1 mV/K pro Zelle als grobe Korrektur

    int32_t mv_comp = (int32_t)mv + (int32_t)(ref_c - temp_c) * comp_mv_per_c;

    if (mv_comp <= SOC_CURVE[0].voltage) {
        return( SOC_CURVE[0].percent );
    }
    else if (mv_comp >= SOC_CURVE[n - 1].voltage) {
        return( mv_comp = SOC_CURVE[n - 1].percent );
    }

    for (unsigned i = 0; i < n - 1; ++i) {
        uint16_t x0 = SOC_CURVE[i].voltage;
        uint16_t y0 = SOC_CURVE[i].percent;
        uint16_t x1 = SOC_CURVE[i + 1].voltage;
        uint16_t y1 = SOC_CURVE[i + 1].percent;

        if (mv_comp >= x0 && mv_comp <= x1) {
            uint32_t num = (uint32_t)(mv_comp - x0) * (uint32_t)(y1 - y0);
            uint32_t den = (uint32_t)(x1 - x0);
            return (uint8_t)(y0 + (num + den / 2) / den);
        }
    }

    return 0;
}

// ---------------------------------------------------------------------------
// Hilfsfunktion: Spannung → Ladezustand (lineare Interpolation)
// ---------------------------------------------------------------------------
// static uint8_t voltage_to_percent(float v)
// {
//     // Über Maximum: 100 %
//     if (v >= SOC_CURVE[0].voltage) return 100;

//     // Unter Minimum: 0 %
//     if (v <= SOC_CURVE[SOC_CURVE_SIZE - 1].voltage) return 0;

//     // Passendes Intervall finden und linear interpolieren
//     for (size_t i = 0; i < SOC_CURVE_SIZE - 1; i++) {
//         float v_hi = SOC_CURVE[i].voltage;
//         float v_lo = SOC_CURVE[i + 1].voltage;
//         if (v >= v_lo) {
//             float p_hi  = static_cast<float>(SOC_CURVE[i].percent);
//             float p_lo  = static_cast<float>(SOC_CURVE[i + 1].percent);
//             float ratio = (v - v_lo) / (v_hi - v_lo);
//             float pct   = p_lo + ratio * (p_hi - p_lo);
//             return static_cast<uint8_t>(pct + 0.5f);
//         }
//     }

//     return 0;  // unerreichbar, beruhigt den Compiler
// }

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

bool akku_init(void)
{
    gxLog_setLevel(tag, LOG_LVL_AKKU);
    gxLog_setColor(tag, ANSI_ESC_FG_YELLOW);

    // Pin explizit als Analog-Eingang konfigurieren.
    pinMode(pinout::BATTERY_ADC, ANALOG);

    // ADC-Resolution explizit setzen (Default ist 12).
    analogReadResolution(12);

    // WICHTIG (ESP32-C6 / neuer Arduino-Core):
    // analogSetPinAttenuation() erwartet, dass der Pin bereits intern als
    // ADC-Channel registriert ist. Das passiert erst beim ERSTEN analogRead*()
    // auf diesem Pin. Ohne diesen Dummy-Read erscheint
    // "[E] Pin is not configured as analog channel" im Log.
    (void) analogRead(pinout::BATTERY_ADC);

    // Maximale Attenuation: ~0…3.1 V Messbereich.
    // V_adc_max = 4.2V / 2 = 2.1V → passt sicher in den Bereich.
    analogSetPinAttenuation(pinout::BATTERY_ADC, ADC_ATTENDB_MAX);

    s_initialized = true;

    gxLog_I(tag, "Init OK | ADC=GP%u, %u Samples, Teiler=1:%u, Ausreißer-Schwelle=%.0fmV",
                  pinout::BATTERY_ADC,
                  pinout::BATTERY_SAMPLES,
                  static_cast<unsigned>(1.0f / BATTERY_DIVIDER_RATIO + 0.5f),
                  OUTLIER_THRESHOLD_MV);
    return true;
}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------
bool akku_is_initialized()
{
    return(s_initialized);
}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------
bool akku_measure(sensor_meas_item_t *out)
{
    out->value_f = 0.0f;
    out->value_u16 = 0;
    out->is_valid = false;

    if (!s_initialized) {
        gxLog_E(tag, "FEHLER: akku_measure() vor akku_init()");
        return false;
    }

    // Pin erneut als ANALOG setzen - der ESP32-Core kann den Pin-State nach
    // anderen Peripherie-Inits (z.B. Wire/I2C) intern zuruecksetzen.
    // Dummy-Read sorgt fuer Re-Registrierung als ADC-Channel (analoge
    // Begruendung wie in akku_init()).
    pinMode(pinout::BATTERY_ADC, ANALOG);
    (void) analogRead(pinout::BATTERY_ADC);

    float value_mv = static_cast<float>(analogReadMilliVolts(pinout::BATTERY_ADC));

    // printf("adc1: %.1f\r\n", value_mv);

    // --- 4. ADC-Spannung in Akku-Spannung umrechnen ---
    out->value_f = value_mv / BATTERY_DIVIDER_RATIO;

    // printf("adc2: %.1f\r\n", value_mv);

    // out->percentage = voltage_to_percent(v_akku);
    // out->value_u16 = voltage_to_percent_temp_comp(v_akku, temp);

    out->is_valid = true;

    return true;
}
// ---------------------------------------------------------------------------
// Messung mit Ausreißerfilterung
// ---------------------------------------------------------------------------
// bool akku_measure(sensor_meas_item_t *out, int16_t temp)
// {
//     if (out == nullptr) {
//         gxLog_E(tag, "FEHLER: akku_measure() mit NULL-Pointer");
//         return false;
//     }
//     if (!s_initialized) {
//         gxLog_E(tag, "FEHLER: akku_measure() vor akku_init()");
//         return false;
//     }

//     constexpr size_t N = pinout::BATTERY_SAMPLES;

//     // Pin erneut als ANALOG setzen - der ESP32-Core kann den Pin-State nach
//     // anderen Peripherie-Inits (z.B. Wire/I2C) intern zuruecksetzen.
//     // Dummy-Read sorgt fuer Re-Registrierung als ADC-Channel (analoge
//     // Begruendung wie in akku_init()).
//     pinMode(pinout::BATTERY_ADC, ANALOG);
//     (void) analogRead(pinout::BATTERY_ADC);

//     // --- 1. Alle Samples in mV in ein Array sammeln ---
//     float samples_mv[N];
//     float sum_mv = 0.0f;
//     for (size_t i = 0; i < N; i++) {
//         samples_mv[i] = static_cast<float>(
//             analogReadMilliVolts(pinout::BATTERY_ADC));
//         sum_mv += samples_mv[i];
//         delayMicroseconds(SAMPLE_DELAY_US);
//     }

//     // --- 2. Erster Mittelwert ---
//     float mean_mv = sum_mv / static_cast<float>(N);

//     // --- 3. Ausreißer verwerfen, neuer Mittelwert ---
//     float filtered_sum = 0.0f;
//     size_t filtered_n  = 0;
//     size_t outliers    = 0;
//     for (size_t i = 0; i < N; i++) {
//         if (fabsf(samples_mv[i] - mean_mv) <= OUTLIER_THRESHOLD_MV) {
//             filtered_sum += samples_mv[i];
//             filtered_n++;
//         } else {
//             outliers++;
//         }
//     }

//     // Fallback: falls alle als Ausreißer markiert wurden (sehr unwahrscheinlich),
//     // verwenden wir den ursprünglichen Mittelwert.
//     float final_mv = (filtered_n > 0) ? (filtered_sum / filtered_n) : mean_mv;

//     if (outliers > 0) {
//         gxLog_D(tag, "%u Ausreißer gefiltert (von %u Samples)", static_cast<unsigned>(outliers), static_cast<unsigned>(N));
//     }

//     // --- 4. ADC-Spannung in Akku-Spannung umrechnen ---
//     uint16_t v_akku = (uint16_t)final_mv / pinout::BATTERY_DIVIDER_RATIO;

//     out->value_f    = v_akku;
//     // out->percentage = voltage_to_percent(v_akku);
//     out->value_u16 = voltage_to_percent_temp_comp(v_akku, temp);

//     return true;
// }

// ---------------------------------------------------------------------------
// Convenience-Funktionen
// -----------------------

