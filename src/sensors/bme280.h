#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

#include "settings.h"

#ifdef USE_SENSOR_BME280


/**
 * @file bme280.h
 * @brief Schlanker Treiber für den BME280 (Temperatur, Luftfeuchte, Luftdruck)
 *
 * Misst Temperatur (°C), relative Luftfeuchte (%RH) und Luftdruck (hPa) per I²C.
 * Verwendet ausschließlich den cpu/i2c_driver - keine externen Libraries.
 *
 * Mess-Modus:
 *   Forced Mode (Einzelmessung auf Befehl, danach automatisch Sleep)
 *   → ideal für Deep-Sleep-Anwendungen
 *
 * Kalibrierung:
 *   Kompensationsformeln nach BME280 Datenblatt BST-BME280-DS002 §4.2.3
 *
 * Vorbedingungen:
 *   1. Sensor-Power-Pin ist HIGH (Stromversorgung anliegend)
 *   2. SENSOR_BOOT_MS gewartet (siehe cpu/pinout.h)
 *   3. i2c_init() wurde erfolgreich aufgerufen
 *
 * Typischer Programmablauf:
 *     bme280_init();
 *     bme280_measurement_t m;
 *     if (bme280_measure(&m)) {
 *         printf("%.2f °C  %.2f %%RH  %.2f hPa\n",
 *                m.temperature_c, m.humidity_pct, m.pressure_hpa);
 *     }
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 * ---------------------------------------------------------------------------
 * 2026-06-12 20:18 - Datei erstellt: BME280-Treiber (ersetzt SHT3x in main.cpp,
 *                    SHT3x-Treiber bleibt als Referenz erhalten)
 *
 * ---------------------------------------------------------------------------
 */

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/** Standard-I²C-Adresse: SDO-Pin auf GND */
constexpr uint8_t BME280_ADDR_LOW  = 0x76;

/** Alternative I²C-Adresse: SDO-Pin auf VDD */
constexpr uint8_t BME280_ADDR_HIGH = 0x77;

/** Chip-ID des BME280 (Register 0xD0, Erwartungswert) */
constexpr uint8_t BME280_CHIP_ID   = 0x60;

// ---------------------------------------------------------------------------
// Datenstruktur für Messergebnis
// ---------------------------------------------------------------------------

// typedef struct {
//     float temperature_c;   /**< Temperatur in °C, Bereich -40 … +85 */
//     float humidity_pct;    /**< Relative Luftfeuchte in %, Bereich 0 … 100 */
//     float pressure_hpa;    /**< Luftdruck in hPa, Bereich 300 … 1100 */
// } bme280_measurement_t;

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

/**
 * @brief Setzt die zu verwendende I²C-Adresse.
 *
 * Muss vor bme280_init() aufgerufen werden, wenn nicht die Default-Adresse
 * (0x76) verwendet werden soll.
 *
 * @param addr  BME280_ADDR_LOW (0x76) oder BME280_ADDR_HIGH (0x77)
 */
void bme280_set_address(uint8_t addr);

/**
 * @brief Liefert die aktuell konfigurierte I²C-Adresse.
 */
uint8_t bme280_get_address(void);

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den Sensor: Chip-ID prüfen, Kalibrierungsdaten lesen,
 *        Überabtastung konfigurieren.
 *
 * @return true   Sensor erreichbar, Chip-ID korrekt (0x60)
 * @return false  Sensor nicht erreichbar oder I²C nicht initialisiert
 */
bool bme280_init(void);

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

/**
 * @brief Führt eine Forced-Mode-Messung durch (blockierend, ~10 ms).
 *
 * Schritte intern:
 *   1. Forced-Mode starten (Sensor misst einmalig, geht dann in Sleep)
 *   2. Auf Messende warten (Status-Register polling, max. 15 ms)
 *   3. Rohdaten lesen (8 Bytes Druck+Temp + 3 Bytes Feuchte)
 *   4. Kompensationsformeln nach Datenblatt §4.2.3 anwenden
 *
 * @param out  Ergebnispuffer (wird nur bei Erfolg geschrieben)
 *
 * @return true   Messung erfolgreich, Werte in *out gültig
 * @return false  Fehler bei Kommunikation oder Sensor nicht initialisiert
 */
bool bme280_measure(sensor_meas_item_t *temp, sensor_meas_item_t *hum, sensor_meas_item_t *pres);

#endif