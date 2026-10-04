/**
 * @file sht3x.cpp
 * @brief Implementierung des SHT3x-Treibers
 *
 * Datenblatt-Referenz:
 *   Sensirion SHT3x-DIS, Version 7 (12/2022)
 *   §4.3  Measurement Commands for Single Shot Data Acquisition
 *   §4.11 Status Register
 *   §4.13 Conversion of Signal Output
 *   §4.4  Checksum Calculation
 */

#include "sht3x.h"

#include "settings.h"

#ifdef USE_SENSOR_SHT3X

#include <Arduino.h>
#include "../cpu/i2c_driver.h"
#include "../services/gxLog.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "sht3x";

// ---------------------------------------------------------------------------
// SHT3x-Befehle (16-Bit, MSB zuerst übertragen)
// ---------------------------------------------------------------------------

namespace {

// Soft-Reset (§4.9)
constexpr uint8_t CMD_SOFT_RESET_MSB    = 0x30;
constexpr uint8_t CMD_SOFT_RESET_LSB    = 0xA2;

// Read Status Register (§4.11)
constexpr uint8_t CMD_READ_STATUS_MSB   = 0xF3;
constexpr uint8_t CMD_READ_STATUS_LSB   = 0x2D;

// Single-Shot, High-Repeatability, no clock stretching (§4.3, Tab. 9)
constexpr uint8_t CMD_MEASURE_HIGH_MSB  = 0x24;
constexpr uint8_t CMD_MEASURE_HIGH_LSB  = 0x00;

// Wartezeiten (Datenblatt, mit Sicherheitsmarge)
constexpr uint32_t SOFT_RESET_DELAY_MS  = 2;    // typ. 0,5 ms, max 1 ms
constexpr uint32_t MEASURE_DELAY_MS     = 16;   // typ. 12,5 ms, max 15 ms

// CRC-8 Parameter (§4.4, Tab. 20)
constexpr uint8_t CRC8_POLYNOMIAL       = 0x31;
constexpr uint8_t CRC8_INIT             = 0xFF;

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Modul-interner Zustand
// ---------------------------------------------------------------------------

static uint8_t s_address     = SHT3X_DEFAULT_ADDR;
static bool    s_initialized = false;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

/**
 * CRC-8 nach Sensirion-Spezifikation.
 *   Polynom: 0x31  (x^8 + x^5 + x^4 + 1)
 *   Init:    0xFF
 *   kein Bit-Reflect, kein Final-XOR
 *
 * Verifiziert gegen Datenblatt-Beispiel: crc({0xBE, 0xEF}) == 0x92
 */
static uint8_t sht3x_crc8(const uint8_t *data, size_t len)
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

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

void sht3x_set_address(uint8_t addr)
{
    if (addr != SHT3X_DEFAULT_ADDR && addr != SHT3X_ALT_ADDR) {
        Serial.printf("[SHT3X] WARNUNG: ungewöhnliche Adresse 0x%02X "
                      "(typ. 0x44 oder 0x45)\n", addr);
    }
    s_address     = addr;
    s_initialized = false;   // neue Adresse → erneute Initialisierung nötig
}

uint8_t sht3x_get_address(void)
{
    return s_address;
}

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

bool sht3x_init(void)
{
    gxLog_setLevel(tag, LOG_LVL_SENSORS);
    gxLog_setColor(tag, ANSI_ESC_FG_MAGENTA);

    if (!i2c_is_initialized()) {
        Serial.println("[SHT3X] FEHLER: I²C nicht initialisiert "
                       "(zuerst i2c_init() aufrufen)");
        return false;
    }

    // --- 1. Soft-Reset senden ---
    const uint8_t reset_cmd[2] = { CMD_SOFT_RESET_MSB, CMD_SOFT_RESET_LSB };
    if (!i2c_write(s_address, reset_cmd, sizeof(reset_cmd))) {
        Serial.printf("[SHT3X] FEHLER: Soft-Reset @ 0x%02X fehlgeschlagen "
                      "(Sensor nicht erreichbar?)\n", s_address);
        return false;
    }
    delay(SOFT_RESET_DELAY_MS);

    // --- 2. Status-Register lesen (Health-Check) ---
    const uint8_t status_cmd[2] = { CMD_READ_STATUS_MSB, CMD_READ_STATUS_LSB };
    uint8_t status_resp[3];    // 2 Datenbytes + 1 CRC
    if (!i2c_write_read(s_address, status_cmd, sizeof(status_cmd),
                                   status_resp, sizeof(status_resp))) {
        Serial.printf("[SHT3X] FEHLER: Status-Read @ 0x%02X fehlgeschlagen\n",
                      s_address);
        return false;
    }

    // --- 3. CRC der Status-Antwort prüfen ---
    uint8_t expected = sht3x_crc8(status_resp, 2);
    if (expected != status_resp[2]) {
        Serial.printf("[SHT3X] FEHLER: Status-CRC ungültig "
                      "(empf. 0x%02X, erw. 0x%02X)\n",
                      status_resp[2], expected);
        return false;
    }

    uint16_t status = (static_cast<uint16_t>(status_resp[0]) << 8) | status_resp[1];

    s_initialized = true;

    gxLog_I(tag, "[SHT3X] Init OK @ 0x%02X | Status: 0x%04X\n",
                  s_address, status);
    return true;
}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

bool sht3x_measure(sensor_meas_item_t *temperature, sensor_meas_item_t *humidity)
{
    temperature->value_f = 0.0f;
    temperature->value_u16 = 0;
    temperature->is_valid = false;

    humidity->value_f  = 0.0f;
    humidity->value_u16 = 0;
    humidity->is_valid = false;

    // if (out == nullptr) {
    //     Serial.println("[SHT3X] FEHLER: sht3x_measure() mit NULL-Pointer");
    //     return false;
    // }
    if (!s_initialized) {
        Serial.println("[SHT3X] FEHLER: sht3x_measure() vor sht3x_init()");
        return false;
    }

    // --- 1. Single-Shot-Befehl senden ---
    const uint8_t meas_cmd[2] = { CMD_MEASURE_HIGH_MSB, CMD_MEASURE_HIGH_LSB };
    if (!i2c_write(s_address, meas_cmd, sizeof(meas_cmd))) {
        Serial.printf("[SHT3X] FEHLER: Mess-Befehl @ 0x%02X fehlgeschlagen\n",
                      s_address);
        return false;
    }

    // --- 2. Auf Messung warten ---
    delay(MEASURE_DELAY_MS);

    // --- 3. 6 Bytes lesen: T_MSB T_LSB T_CRC RH_MSB RH_LSB RH_CRC ---
    uint8_t raw[6];
    if (!i2c_read(s_address, raw, sizeof(raw))) {
        Serial.printf("[SHT3X] FEHLER: Datenbytes-Read @ 0x%02X fehlgeschlagen\n",
                      s_address);
        return false;
    }

    // --- 4. Beide CRCs prüfen ---
    uint8_t crc_temp_expected = sht3x_crc8(&raw[0], 2);
    if (crc_temp_expected != raw[2]) {
        Serial.printf("[SHT3X] FEHLER: Temperatur-CRC ungültig "
                      "(empf. 0x%02X, erw. 0x%02X)\n",
                      raw[2], crc_temp_expected);
        return false;
    }
    uint8_t crc_humi_expected = sht3x_crc8(&raw[3], 2);
    if (crc_humi_expected != raw[5]) {
        Serial.printf("[SHT3X] FEHLER: Luftfeuchte-CRC ungültig "
                      "(empf. 0x%02X, erw. 0x%02X)\n",
                      raw[5], crc_humi_expected);
        return false;
    }

    // --- 5. Rohwerte zu physikalischen Größen (Datenblatt §4.13) ---
    uint16_t t_raw  = (static_cast<uint16_t>(raw[0]) << 8) | raw[1];
    uint16_t rh_raw = (static_cast<uint16_t>(raw[3]) << 8) | raw[4];

    // T  =  -45 + 175 * S_T  / (2^16 - 1)
    // RH =  100 * S_RH / (2^16 - 1)
    temperature->value_f = -45.0f + 175.0f * (static_cast<float>(t_raw)  / 65535.0f);
    humidity->value_f    = 100.0f * (static_cast<float>(rh_raw) / 65535.0f);
    temperature->is_valid = true;
    humidity->is_valid = true;

    return true;
}

#endif