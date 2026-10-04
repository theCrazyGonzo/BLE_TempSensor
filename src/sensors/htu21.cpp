/**
 * @file htu21.cpp
 * @brief Implementierung des HTU21D-Treibers
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 *
 * 2026-06-09 08:28 - Umbenannt von sht21.cpp → htu21.cpp (verbauter Sensor ist HTU21D)
 *
 * ---------------------------------------------------------------------------
 * 2026-06-09 08:25 - CRC-Berechnung korrigiert: keine Maskierung der Status-Bits
 *                    (HTU21D berechnet CRC über Rohbytes, SHT21-Maskierung war falsch)
 *
 * ---------------------------------------------------------------------------
 * 2026-06-09 08:21 - No-Hold-Modus: Poll-Loop durch festes Delay ersetzt
 *                    (HTU21D kompatibel: INVALID_STATE-Fehler bei sofortigem Read behoben)
 *
 * ---------------------------------------------------------------------------
 * 2026-06-06 21:03 - Farbige Log-Tags (log_colors.h) eingebunden
 *
 * ---------------------------------------------------------------------------
 * 2026-06-06 19:30 - Datei erstellt (neuer Sensor HTU21D ersetzt SHT3x)
 *
 * ---------------------------------------------------------------------------
 * Datenblatt-Referenz:
 *   MEAS HTU21D(F), HPC199_3 (10/2013)
 *   §CRC  Checksum Calculation
 *   §Hold/No-Hold  Measurement Commands
 *   §Conversion  Signal Output
 */

#include "settings.h"

#ifdef USE_SENSOR_HTU21

#include <Arduino.h>
#include "../cpu/i2c_driver.h"
#include "../cpu/log_colors.h"
#include "../services/gxLog.h"
#include "htu21.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "htu21";

// ---------------------------------------------------------------------------
// HTU21D-Befehle
// ---------------------------------------------------------------------------

namespace {

// Soft-Reset
constexpr uint8_t CMD_SOFT_RESET            = 0xFE;

// No-Hold-Master Messbefehle
// Im No-Hold-Modus wird der I²C-Bus zwischen Befehl und Lesen freigegeben;
// der Sensor NACK't solange die Messung noch läuft.
constexpr uint8_t CMD_TRIGGER_TEMP_NO_HOLD  = 0xF3;
constexpr uint8_t CMD_TRIGGER_RH_NO_HOLD    = 0xF5;

// User-Register lesen/schreiben
constexpr uint8_t CMD_READ_USER_REG         = 0xE7;

// Wartezeiten (HTU21D Datenblatt - Max-Werte + Sicherheitsmarge)
// Festes Delay statt Poll-Loop: sofortiger Read führt zu ESP32 INVALID_STATE
constexpr uint32_t SOFT_RESET_DELAY_MS      = 15;   // Datenblatt: max. 15 ms
constexpr uint32_t TEMP_MEASURE_DELAY_MS    = 55;   // HTU21D 14-Bit: max. 50 ms
constexpr uint32_t RH_MEASURE_DELAY_MS      = 20;   // HTU21D 12-Bit: max. 16 ms

// CRC-8 Parameter
// Polynom: x^8 + x^5 + x^4 + 1  (0x131 → gekürzt 0x31)
constexpr uint8_t CRC8_POLYNOMIAL           = 0x31;
constexpr uint8_t CRC8_INIT                 = 0x00;

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Modul-interner Zustand
// ---------------------------------------------------------------------------

static bool s_initialized = false;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

// ------------------------------------------------------------
/**
 * CRC-8 nach HTU21D-Spezifikation.
 *   Polynom: 0x131 (x^8+x^5+x^4+1), Init: 0x00, kein Final-XOR
 *   CRC wird über Rohbytes berechnet (Status-Bits NICHT maskieren!)
 *
 * Verifiziert gegen Datenblatt-Beispiele:
 *   crc({0x4E, 0x85}) == 0x6B  (32.3 %RH)
 *   crc({0x68, 0x3A}) == 0x7C  (24.7 °C)
 */
static uint8_t htu21_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = CRC8_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ CRC8_POLYNOMIAL);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;

}

// ------------------------------------------------------------
/**
 * Sendet einen 1-Byte-Befehl, wartet ein festes Delay und liest
 * dann 3 Bytes (MSB, LSB, CRC). HTU21D-kompatibel: kein Poll-Loop,
 * da der ESP32-I2C-Treiber bei NACK in INVALID_STATE gerät.
 *
 * @param cmd       Messbefehl (0xF3 oder 0xF5)
 * @param delay_ms  Feste Wartezeit nach Befehl (sensor-spezifisch)
 * @param raw_out   Empfangspuffer (mind. 3 Bytes)
 * @return true bei Erfolg, false bei Read-Fehler oder CRC-Fehler
 */
static bool htu21_trigger_and_read(uint8_t cmd, uint32_t delay_ms,
                                   uint8_t *raw_out)
{
    // 1. Messbefehl senden
    if (!i2c_write(HTU21_I2C_ADDR, &cmd, 1)) {
        gxLog_E(tag, "Befehl 0x%02X senden fehlgeschlagen", cmd);
        return false;
    }

    // 2. Festes Delay abwarten (HTU21D: kein Poll, sofortiger Read → INVALID_STATE)
    delay(delay_ms);

    // 3. Messergebnis lesen
    if (!i2c_read(HTU21_I2C_ADDR, raw_out, 3)) {
        gxLog_E(tag, "Read fehlgeschlagen nach %lu ms (Befehl 0x%02X)", (unsigned long)delay_ms, cmd);
        return false;
    }

    // 4. CRC über MSB + LSB prüfen (Rohbytes, keine Maskierung der Status-Bits)
    // HTU21D Datenblatt §CRC: CRC wird über die unveränderten Datenbytes berechnet.
    // Beispiel: 0x4E85 → CRC 0x6B (Bit1=1 bleibt erhalten, nicht maskieren!)
    uint8_t crc_expected = htu21_crc8(raw_out, 2);
    if (crc_expected != raw_out[2]) {
        gxLog_E(tag, "CRC ungültig für Befehl 0x%02X (empf. 0x%02X, erw. 0x%02X)", cmd, raw_out[2], crc_expected);
        return false;
    }

    return true;

}

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

// ------------------------------------------------------------
bool htu21_init(void)
{
    s_initialized = false;

    gxLog_setLevel(tag, LOG_LVL_SENSORS);
    gxLog_setColor(tag, ANSI_ESC_FG_MAGENTA);

    if (!i2c_is_initialized()) {
        gxLog_E(tag, "I²C nicht initialisiert (zuerst i2c_init() aufrufen)");
        return false;
    }

    // 1. Soft-Reset senden
    const uint8_t reset_cmd = CMD_SOFT_RESET;
    if (!i2c_write(HTU21_I2C_ADDR, &reset_cmd, 1)) {
        gxLog_E(tag, "Soft-Reset @ 0x%02X fehlgeschlagen (Sensor nicht erreichbar?)", HTU21_I2C_ADDR);
        return false;
    }
    delay(SOFT_RESET_DELAY_MS);

    // 2. User-Register lesen (Health-Check - prüft ob Sensor antwortet)
    const uint8_t user_cmd = CMD_READ_USER_REG;
    uint8_t       user_reg = 0;
    if (!i2c_write_read(HTU21_I2C_ADDR, &user_cmd, 1, &user_reg, 1)) {
        gxLog_E(tag, "User-Register-Read @ 0x%02X fehlgeschlagen", HTU21_I2C_ADDR);
        return false;
    }

    s_initialized = true;

    gxLog_I(tag, "Init OK @ 0x%02X | User-Reg: 0x%02X", HTU21_I2C_ADDR, user_reg);
    return true;

}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

// ------------------------------------------------------------
bool htu21_measure(sensor_meas_item_t *temp, sensor_meas_item_t *hum)
{
    temp->value_f = 0.0f;
    temp->value_u16  = 0;
    temp->is_valid = false;

    hum->value_f = 0.0f;
    hum->value_u16  = 0;
    hum->is_valid = false;

    if (!s_initialized) {
        gxLog_E(tag, "htu21_measure() vor htu21_init()");
        return false;
    }

    uint8_t raw[3];

    // --- Temperatur ---
    if (!htu21_trigger_and_read(CMD_TRIGGER_TEMP_NO_HOLD, TEMP_MEASURE_DELAY_MS, raw)) {
        return false;
    }
    // Rohwert: MSB[15:8] LSB[7:2] | Bits 1:0 = Status (für Physik-Formel maskieren)
    uint16_t t_raw = (static_cast<uint16_t>(raw[0]) << 8)
                   | (static_cast<uint16_t>(raw[1]) & 0xFC);
    // T = -46.85 + 175.72 * S_T / 2^16  (HTU21D Datenblatt §Conversion)
    temp->value_f = -46.85f + 175.72f * (static_cast<float>(t_raw) / 65536.0f);

    // --- Relative Luftfeuchte ---
    if (!htu21_trigger_and_read(CMD_TRIGGER_RH_NO_HOLD, RH_MEASURE_DELAY_MS, raw)) {
        return false;
    }
    uint16_t rh_raw = (static_cast<uint16_t>(raw[0]) << 8)
                    | (static_cast<uint16_t>(raw[1]) & 0xFC);
    // RH = -6 + 125 * S_RH / 2^16  (HTU21D Datenblatt §Conversion)
    float rh = -6.0f + 125.0f * (static_cast<float>(rh_raw) / 65536.0f);
    // Auf physikalisch gültigen Bereich begrenzen
    if (rh < 0.0f)   rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;
    hum->value_f = rh;

    temp->is_valid = true;
    hum->is_valid = true;

    return true;

}

#endif