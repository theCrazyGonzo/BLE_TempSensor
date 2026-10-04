#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

#include "settings.h"

#ifdef USE_SENSOR_HTU21

/**
 * @file htu21.h
 * @brief Schlanker Treiber für den HTU21D (kompatibel: SHT21, SHT20, SHT25)
 *
 * Misst Temperatur (°C) und relative Luftfeuchte (%RH) per I²C.
 * Verwendet ausschließlich den cpu/i2c_driver - keine externen Libraries.
 *
 * Mess-Modus:
 *   No-Hold-Master, 12-Bit RH / 14-Bit Temperatur (Default nach Power-On)
 *   → Temp-Messdauer max. 50 ms (HTU21D Datenblatt)
 *   → RH-Messdauer   max. 16 ms
 *
 * Vorbedingungen:
 *   1. Sensor-Power-Pin ist HIGH (Stromversorgung anliegend)
 *   2. SENSOR_BOOT_MS gewartet (siehe cpu/pinout.h)
 *   3. i2c_init() wurde erfolgreich aufgerufen
 *
 * Typischer Programmablauf:
 *     htu21_init();
 *     htu21_measurement_t m;
 *     if (htu21_measure(&m)) {
 *         printf("%.2f °C  %.2f %%RH\n", m.temperature_c, m.humidity_pct);
 *     }
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 *
 * 2026-06-09 08:28 - Umbenannt von sht21.h → htu21.h (verbauter Sensor ist HTU21D)
 *
 * ---------------------------------------------------------------------------
 */

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/** Fest verdrahtete I²C-Adresse des HTU21D (nicht änderbar) */
constexpr uint8_t HTU21_I2C_ADDR = 0x40;

// ---------------------------------------------------------------------------
// Datenstruktur für Messergebnis
// ---------------------------------------------------------------------------

// typedef struct {
//     float temperature_c;   /**< Temperatur in °C, Bereich -40 … +125 */
//     float humidity_pct;    /**< Relative Luftfeuchte in %, Bereich 0 … 100 */
// } htu21_measurement_t;

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den Sensor: Soft-Reset + User-Register-Read als Health-Check.
 *
 * Nach dem Soft-Reset wartet der Treiber die vom Datenblatt vorgeschriebenen
 * 15 ms, bevor das User-Register gelesen wird.
 *
 * @return true   Sensor erreichbar, User-Register enthält plausiblen Wert
 * @return false  Sensor antwortet nicht oder I²C nicht initialisiert
 */
bool htu21_init(void);

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

/**
 * @brief Führt eine Temperatur- und Feuchtemessung durch (blockierend, ~80 ms).
 *
 * Schritte intern:
 *   1. No-Hold-Master Temperatur-Messbefehl (0xF3) senden
 *   2. Festes Delay (55 ms) abwarten
 *   3. 3 Bytes lesen (MSB, LSB, CRC), CRC prüfen, Rohwert umrechnen
 *   4. No-Hold-Master Feuchte-Messbefehl (0xF5) senden
 *   5. Festes Delay (20 ms) abwarten
 *   6. 3 Bytes lesen (MSB, LSB, CRC), CRC prüfen, Rohwert umrechnen
 *
 * @param out  Ergebnispuffer (wird nur bei Erfolg geschrieben)
 *
 * @return true   Messung erfolgreich, Werte in *out gültig
 * @return false  Fehler bei Kommunikation oder CRC-Prüfung
 */
bool htu21_measure(sensor_meas_item_t *temp, sensor_meas_item_t *hum);

#endif