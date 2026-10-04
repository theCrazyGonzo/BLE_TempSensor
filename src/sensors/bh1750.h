#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

#include "settings.h"

#ifdef USE_SENSOR_BH1750

/**
 * @file bh1750.h
 * @brief Schlanker Treiber für den BH1750(FVI)-Umgebungslichtsensor
 *
 * Misst die Beleuchtungsstärke in Lux per I²C.
 * Verwendet ausschließlich den cpu/i2c_driver - keine externen Libraries.
 *
 * Mess-Modus:
 *   One-Time High-Resolution (Befehl 0x20)
 *   → 1 lx Auflösung, max. 180 ms Messdauer
 *   → Sensor schaltet sich nach der Messung AUTOMATISCH in Power-Down
 *     (ideal für unsere Deep-Sleep-Anwendung)
 *
 * Vorbedingungen:
 *   1. Sensor-Power-Pin ist HIGH (Stromversorgung anliegend)
 *   2. SENSOR_BOOT_MS gewartet (siehe cpu/pinout.h)
 *   3. i2c_init() wurde erfolgreich aufgerufen
 *
 * Typischer Programmablauf:
 *     bh1750_set_address(BH1750_DEFAULT_ADDR);   // optional
 *     bh1750_init();
 *     bh1750_measurement_t m;
 *     if (bh1750_measure(&m)) {
 *         printf("%.1f lx\n", m.lux);
 *     }
 */

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/** Standard-I²C-Adresse: ADDR-Pin auf GND (oder offen, da intern auf LOW) */
constexpr uint8_t BH1750_DEFAULT_ADDR = 0x23;

/** Alternative I²C-Adresse: ADDR-Pin auf VDD */
constexpr uint8_t BH1750_ALT_ADDR     = 0x5C;

// ---------------------------------------------------------------------------
// Datenstruktur für Messergebnis
// ---------------------------------------------------------------------------

typedef struct {
    float lux;   /**< Beleuchtungsstärke in Lux, Bereich 0 … ~54612 */
} bh1750_measurement_t;

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

/**
 * @brief Setzt die zu verwendende I²C-Adresse.
 *
 * Muss vor bh1750_init() aufgerufen werden, wenn nicht die Default-Adresse
 * (0x23) verwendet werden soll. Kann später erneut geändert werden.
 *
 * @param addr  7-Bit-Adresse, typischerweise BH1750_DEFAULT_ADDR (0x23)
 *              oder BH1750_ALT_ADDR (0x5C)
 */
void bh1750_set_address(uint8_t addr);

/**
 * @brief Liefert die aktuell konfigurierte I²C-Adresse.
 */
uint8_t bh1750_get_address(void);

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den Sensor: sendet Power-On als Health-Check.
 *
 * Im One-Time-Modus reicht ein einzelnes Power-On, da der Sensor sich
 * nach jeder Messung selbsttätig wieder in Power-Down begibt.
 *
 * @return true   Sensor antwortet (ACK auf Power-On)
 * @return false  Sensor nicht erreichbar oder I²C nicht initialisiert
 */
bool bh1750_init(void);

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

/**
 * @brief Führt eine Einzelmessung durch (blockierend, ~180 ms).
 *
 * Schritte intern:
 *   1. Befehl 0x20 senden (One-Time, High-Resolution)
 *   2. 180 ms warten (Datenblatt: max. Messdauer)
 *   3. 2 Datenbytes lesen
 *   4. Lux-Wert berechnen: lux = rohwert / 1.2
 *
 * Nach Abschluss befindet sich der Sensor automatisch wieder im Power-Down.
 *
 * @param out  Ergebnispuffer (wird nur bei Erfolg geschrieben)
 *
 * @return true   Messung erfolgreich, Wert in out->lux gültig
 * @return false  Fehler bei der Kommunikation
 */
bool bh1750_measure(sensor_meas_item_t *out);

#endif