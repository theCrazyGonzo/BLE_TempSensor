#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

/**
 * @file akku.h
 * @brief Treiber für die Akku-Überwachung (18650 Li-Ion, 2000 mAh)
 *
 * Misst die Akkuspannung am ADC-Pin (GP1) hinter dem 1:2-Spannungsteiler
 * und schätzt den Ladezustand (SOC, State of Charge) anhand der
 * typischen Entladekurve einer 18650-Standardzelle bei niedriger Last.
 *
 * Genauigkeitshinweis:
 *   Die SOC-Schätzung über die Leerlaufspannung ist im flachen mittleren
 *   Bereich der Li-Ion-Kennlinie (3.7-3.9 V) ungenau (±15 %). Für eine
 *   bessere Schätzung wäre Coulomb-Counting nötig - das übersteigt aber
 *   den Rahmen dieser Anwendung. Für eine grobe Ampel-Anzeige
 *   "voll / mittel / leer" reicht die Lookup-Tabelle gut aus.
 *
 * Vorbedingungen:
 *   1. Spannungsteiler ist am ADC-Pin korrekt verschaltet (Faktor 1:2)
 *   2. akku_init() wurde aufgerufen
 *
 * Typischer Programmablauf:
 *     akku_init();
 *     akku_state_t s;
 *     if (akku_measure(&s)) {
 *         printf("%.3f V (%u %%)\n", s.voltage, s.percentage);
 *     }
 */

// ---------------------------------------------------------------------------
// Datenstruktur für Messergebnis
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t voltage;       /**< Akkuspannung in Volt (typ. 3000 … 4200) */
    uint8_t  percentage;    /**< Ladezustand in Prozent (0 … 100) */
} akku_state_t;

constexpr float OUTLIER_THRESHOLD_MV = 30.0f;

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den ADC für die Akku-Spannungsmessung.
 *
 * Setzt 12-Bit-Auflösung und 11-dB-Attenuation auf dem konfigurierten
 * Pin (siehe cpu/pinout.h).
 *
 * @return true   Erfolg (immer)
 * @return false  Falls die ADC-Konfiguration fehlschlägt
 */
bool akku_init(void);

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

/**
 * @brief Misst Spannung und Ladezustand in einem Schritt.
 *
 * Führt intern mehrere ADC-Samples durch (BATTERY_SAMPLES aus pinout.h),
 * mittelt diese, rechnet den Spannungsteiler heraus und ermittelt den
 * SOC durch lineare Interpolation der Lookup-Tabelle.
 *
 * @param out  Ergebnispuffer (wird nur bei Erfolg geschrieben)
 *
 * @return true   Messung erfolgreich
 * @return false  Fehler (z. B. akku_init() nicht aufgerufen)
 */
bool akku_measure(sensor_meas_item_t *out);

/**
 * @brief Convenience: gibt nur die Spannung zurück.
 *
 * Führt eine vollständige Messung durch und gibt nur die Spannung
 * zurück. Bei Bedarf nach Spannung UND Ladezustand bitte
 * akku_measure() verwenden (effizienter).
 *
 * @return Akkuspannung in V, 0.0 bei Fehler
 */
// float akku_get_voltage(void);

/**
 * @brief Convenience: gibt nur den Ladezustand zurück.
 *
 * Führt eine vollständige Messung durch und gibt nur den SOC zurück.
 *
 * @return Ladezustand 0 … 100, 0 bei Fehler
 */
// uint8_t akku_get_percentage(void);

/**
 * @brief Berechnet Akku-Level
 *
 * Berechnet den Akku-Level und führt eine Temperatur-Kompensation aus
 *
 * @param mv  Akkuspannung in mV
 * @param temp_c  Temperatur in °C
 *
 * @return Ladezustand 0 … 100, 0 bei Fehler
 */
uint8_t akku_voltage_to_percent_temp_comp(uint16_t mv, int16_t temp_c);

/**
 * @brief Prüfen, ob ADC schon initialsiert wurde
 *
 * Prüfen, ob ADC schon initialsiert wurde
 *
  * @return wenn schon initialisiert: true, wenn nicht initialisiert: false
 */
bool akku_is_initialized();
