#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

#include "settings.h"

#ifdef USE_SENSOR_SHT3X


/**
 * @file sht3x.h
 * @brief Schlanker Treiber für die SHT3x-Familie (SHT30/SHT31/SHT35)
 *
 * Misst Temperatur (°C) und relative Luftfeuchte (%RH) per I²C.
 * Verwendet ausschließlich den cpu/i2c_driver - keine externen Libraries.
 *
 * Mess-Modus:
 *   Single-Shot, High-Repeatability, ohne Clock-Stretching (Befehl 0x2400)
 *   → typ. 12,5 ms Messdauer, max. 15 ms (siehe Datenblatt §4.4)
 *
 * Vorbedingungen:
 *   1. Sensor-Power-Pin ist HIGH (Stromversorgung anliegend)
 *   2. SENSOR_BOOT_MS gewartet (siehe cpu/pinout.h)
 *   3. i2c_init() wurde erfolgreich aufgerufen
 *
 * Typischer Programmablauf:
 *     sht3x_set_address(SHT3X_DEFAULT_ADDR);   // optional
 *     sht3x_init();
 *     sht3x_measurement_t m;
 *     if (sht3x_measure(&m)) {
 *         printf("%.2f °C  %.2f %%RH\n", m.temperature_c, m.humidity_pct);
 *     }
 */

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/** Standard-I²C-Adresse: ADDR-Pin auf GND */
constexpr uint8_t SHT3X_DEFAULT_ADDR = 0x44;

/** Alternative I²C-Adresse: ADDR-Pin auf VDD */
constexpr uint8_t SHT3X_ALT_ADDR     = 0x45;

// ---------------------------------------------------------------------------
// Datenstruktur für Messergebnis
// ---------------------------------------------------------------------------

// typedef struct {
//     float temperature_c;   /**< Temperatur in °C, Bereich -45 … +130 */
//     float humidity_pct;    /**< Relative Luftfeuchte in %, Bereich 0 … 100 */
// } sht3x_measurement_t;

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

/**
 * @brief Setzt die zu verwendende I²C-Adresse.
 *
 * Muss vor sht3x_init() aufgerufen werden, wenn nicht die Default-Adresse
 * (0x44) verwendet werden soll. Kann später erneut geändert werden.
 *
 * @param addr  7-Bit-Adresse, typischerweise SHT3X_DEFAULT_ADDR (0x44)
 *              oder SHT3X_ALT_ADDR (0x45)
 */
void sht3x_set_address(uint8_t addr);

/**
 * @brief Liefert die aktuell konfigurierte I²C-Adresse.
 */
uint8_t sht3x_get_address(void);

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den Sensor: Soft-Reset + Status-Read als Health-Check.
 *
 * @return true   Sensor erreichbar, CRC der Status-Antwort korrekt
 * @return false  Sensor nicht antwortet oder I²C nicht initialisiert
 */
bool sht3x_init(void);

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

/**
 * @brief Führt eine Single-Shot-Messung durch (blockierend, ~15 ms).
 *
 * Schritte intern:
 *   1. Befehl 0x2400 senden (High-Repeatability, kein Clock-Stretching)
 *   2. 15 ms warten
 *   3. 6 Datenbytes lesen (Temp+CRC, Hum+CRC)
 *   4. Beide CRCs validieren
 *   5. Rohwerte in physikalische Größen umrechnen
 *
 * @param out  Ergebnispuffer (wird nur bei Erfolg geschrieben)
 *
 * @return true   Messung erfolgreich, Werte in *out gültig
 * @return false  Fehler bei Kommunikation oder CRC-Prüfung
 */
bool sht3x_measure(sensor_meas_item_t *temperature, sensor_meas_item_t *humidity);

#endif