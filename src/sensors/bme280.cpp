/**
 * @file bme280.cpp
 * @brief Implementierung des BME280-Treibers
 *
 * Datenblatt-Referenz:
 *   Bosch Sensortec BME280, BST-BME280-DS002-15 (05/2015)
 *   §4.2.3  Compensation formulas in double precision floating point
 *   §4.3.4  Forced mode
 *   §5      Register description
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 * ---------------------------------------------------------------------------
 * 2026-06-12 20:18 - Datei erstellt: BME280-Treiber für Temperatur, Luftfeuchte
 *                    und Luftdruck. Ersetzt SHT3x in main.cpp (SHT3x-Treiber
 *                    bleibt erhalten). Forced-Mode, Kalibrierung nach DB §4.2.3.
 *
 * ---------------------------------------------------------------------------
 */

#include "settings.h"

#ifdef USE_SENSOR_BME280

#include <Arduino.h>
#include "../cpu/i2c_driver.h"
#include "../cpu/log_colors.h"
#include "../services/gxLog.h"
#include "bme280.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "bme280";

// ---------------------------------------------------------------------------
// Register-Adressen (BME280 Datenblatt §5)
// ---------------------------------------------------------------------------

namespace {

constexpr uint8_t REG_CHIP_ID       = 0xD0;
constexpr uint8_t REG_RESET         = 0xE0;
constexpr uint8_t REG_CTRL_HUM      = 0xF2;
constexpr uint8_t REG_STATUS        = 0xF3;
constexpr uint8_t REG_CTRL_MEAS     = 0xF4;
constexpr uint8_t REG_CONFIG        = 0xF5;
constexpr uint8_t REG_PRESS_MSB     = 0xF7;   // Startadresse Burst-Read (Druck+Temp+Feuchte)
constexpr uint8_t REG_CALIB_00      = 0x88;   // Kalibrierung Teil 1: 26 Bytes (Druck+Temp)
constexpr uint8_t REG_CALIB_26      = 0xE1;   // Kalibrierung Teil 2: 7 Bytes (Feuchte)

constexpr uint8_t RESET_VALUE       = 0xB6;   // Soft-Reset-Wert

// Überabtastung ×1 für alle drei Sensoren (ausreichend für unsere Anwendung)
// ctrl_hum:  osrs_h = 001 → ×1
// ctrl_meas: osrs_t = 001, osrs_p = 001, mode = 01 (Forced)
constexpr uint8_t OSRS_HUM_X1       = 0x01;
constexpr uint8_t OSRS_TEMP_X1      = (1 << 5);
constexpr uint8_t OSRS_PRESS_X1     = (1 << 2);
constexpr uint8_t MODE_FORCED       = 0x01;
constexpr uint8_t CTRL_MEAS_FORCED  = OSRS_TEMP_X1 | OSRS_PRESS_X1 | MODE_FORCED;

// Maximale Wartezeit auf Messende: 2 + 2.3×osrs_T + (2.3×osrs_P + 0.575) + (2.3×osrs_H + 0.575)
// Bei ×1 überall: typ. ~10 ms, max. ~15 ms → wir nehmen 20 ms als sicheren Wert
constexpr uint32_t MEASURE_WAIT_MS  = 20;
constexpr uint32_t BOOT_DELAY_MS    = 2;   // nach Power-On / Reset

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Kalibrierungsdaten-Struktur (Datenblatt §4.2.2)
// ---------------------------------------------------------------------------

struct bme280_calib_t {
    // Temperatur
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    // Druck
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    // Luftfeuchte
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
};

// ---------------------------------------------------------------------------
// Modul-interner Zustand
// ---------------------------------------------------------------------------

static uint8_t        s_address     = BME280_ADDR_LOW;
static bool           s_initialized = false;
static bme280_calib_t s_calib;

// ---------------------------------------------------------------------------
// Hilfsfunktion: Register schreiben (1 Byte)
// ---------------------------------------------------------------------------

static bool reg_write(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = { reg, value };
    return i2c_write(s_address, buf, 2);

}

// ---------------------------------------------------------------------------
// Hilfsfunktion: Register lesen (n Bytes, Burst-Read)
// ---------------------------------------------------------------------------

static bool reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read(s_address, &reg, 1, buf, len);

}

// ---------------------------------------------------------------------------
// Kalibrierungsdaten laden
// ---------------------------------------------------------------------------

static bool load_calibration(void)
{
    uint8_t raw[26];

    // --- Teil 1: 0x88..0xA1 (26 Bytes: Temperatur + Druck) ---
    if (!reg_read(REG_CALIB_00, raw, 26)) {
        gxLog_E(tag, "Kalibrierung Teil 1 lesen fehlgeschlagen");
        return false;
    }

    s_calib.dig_T1 = (uint16_t)(raw[1]  << 8 | raw[0]);
    s_calib.dig_T2 = (int16_t) (raw[3]  << 8 | raw[2]);
    s_calib.dig_T3 = (int16_t) (raw[5]  << 8 | raw[4]);

    s_calib.dig_P1 = (uint16_t)(raw[7]  << 8 | raw[6]);
    s_calib.dig_P2 = (int16_t) (raw[9]  << 8 | raw[8]);
    s_calib.dig_P3 = (int16_t) (raw[11] << 8 | raw[10]);
    s_calib.dig_P4 = (int16_t) (raw[13] << 8 | raw[12]);
    s_calib.dig_P5 = (int16_t) (raw[15] << 8 | raw[14]);
    s_calib.dig_P6 = (int16_t) (raw[17] << 8 | raw[16]);
    s_calib.dig_P7 = (int16_t) (raw[19] << 8 | raw[18]);
    s_calib.dig_P8 = (int16_t) (raw[21] << 8 | raw[20]);
    s_calib.dig_P9 = (int16_t) (raw[23] << 8 | raw[22]);
    s_calib.dig_H1 = raw[25];   // 0xA1

    // --- Teil 2: 0xE1..0xE7 (7 Bytes: Luftfeuchte) ---
    uint8_t h[7];
    if (!reg_read(REG_CALIB_26, h, 7)) {
        gxLog_E(tag, "Kalibrierung Teil 2 lesen fehlgeschlagen");
        return false;
    }

    s_calib.dig_H2 = (int16_t) (h[1] << 8 | h[0]);
    s_calib.dig_H3 = h[2];
    // dig_H4 und dig_H5 sind über Byte-Grenzen hinweg gepackt (DB §4.2.2)
    s_calib.dig_H4 = (int16_t) ((int16_t)h[3] << 4 | (h[4] & 0x0F));
    s_calib.dig_H5 = (int16_t) (h[5] << 4 | (h[4] >> 4));
    s_calib.dig_H6 = (int8_t)  h[6];

    return true;

}

// ---------------------------------------------------------------------------
// Kompensationsformeln (Datenblatt §4.2.3, floating point)
// t_fine wird intern zwischen Temperatur- und den anderen Berechnungen
// weitergegeben.
// ---------------------------------------------------------------------------

static float compensate_temperature(int32_t adc_T, int32_t *t_fine)
{
    float var1 = ((float)adc_T / 16384.0f - (float)s_calib.dig_T1 / 1024.0f) * (float)s_calib.dig_T2;
    float var2 = ((float)adc_T / 131072.0f - (float)s_calib.dig_T1 / 8192.0f)
               * ((float)adc_T / 131072.0f - (float)s_calib.dig_T1 / 8192.0f)
               * (float)s_calib.dig_T3;
    *t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;

}

// ------------------------------------------------------------
static float compensate_pressure(int32_t adc_P, int32_t t_fine)
{
    float var1 = (float)t_fine / 2.0f - 64000.0f;
    float var2 = var1 * var1 * (float)s_calib.dig_P6 / 32768.0f;
    var2 = var2 + var1 * (float)s_calib.dig_P5 * 2.0f;
    var2 = var2 / 4.0f + (float)s_calib.dig_P4 * 65536.0f;
    var1 = ((float)s_calib.dig_P3 * var1 * var1 / 524288.0f + (float)s_calib.dig_P2 * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)s_calib.dig_P1;
    if (var1 == 0.0f) return 0.0f;   // Division durch Null verhindern
    float p = 1048576.0f - (float)adc_P;
    p = (p - var2 / 4096.0f) * 6250.0f / var1;
    var1 = (float)s_calib.dig_P9 * p * p / 2147483648.0f;
    var2 = p * (float)s_calib.dig_P8 / 32768.0f;
    p = p + (var1 + var2 + (float)s_calib.dig_P7) / 16.0f;
    return p / 100.0f;   // Pa → hPa

}

// ------------------------------------------------------------
static float compensate_humidity(int32_t adc_H, int32_t t_fine)
{
    float x = (float)t_fine - 76800.0f;
    if (x == 0.0f) return 0.0f;
    x = ((float)adc_H - ((float)s_calib.dig_H4 * 64.0f + (float)s_calib.dig_H5 / 16384.0f * x))
      * ((float)s_calib.dig_H2 / 65536.0f
         * (1.0f + (float)s_calib.dig_H6 / 67108864.0f * x
            * (1.0f + (float)s_calib.dig_H3 / 67108864.0f * x)));
    x = x * (1.0f - (float)s_calib.dig_H1 * x / 524288.0f);
    if (x > 100.0f) x = 100.0f;
    if (x < 0.0f)   x = 0.0f;
    return x;

}

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

void bme280_set_address(uint8_t addr)
{
    if (addr != BME280_ADDR_LOW && addr != BME280_ADDR_HIGH) {
        gxLog_W(tag, "ungewöhnliche Adresse 0x%02X (typ. 0x76 oder 0x77)", addr);
    }
    s_address     = addr;
    s_initialized = false;

}

// ------------------------------------------------------------
uint8_t bme280_get_address(void)
{
    return s_address;

}

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

bool bme280_init(void)
{
    s_initialized = false;

    gxLog_setLevel(tag, LOG_LVL_SENSORS);
    gxLog_setColor(tag, ANSI_ESC_FG_MAGENTA);

    if (!i2c_is_initialized()) {
        gxLog_E(tag, "I²C nicht initialisiert (zuerst i2c_init() aufrufen)");
        return false;
    }

    // 1. Soft-Reset
    if (!reg_write(REG_RESET, RESET_VALUE)) {
        gxLog_E(tag, "Soft-Reset @ 0x%02X fehlgeschlagen (Sensor nicht erreichbar?)", s_address);
        return false;
    }
    delay(BOOT_DELAY_MS);

    // 2. Chip-ID prüfen
    uint8_t chip_id = 0;
    if (!reg_read(REG_CHIP_ID, &chip_id, 1)) {
        gxLog_E(tag, "Chip-ID lesen fehlgeschlagen");
        return false;
    }
    if (chip_id != BME280_CHIP_ID) {
        gxLog_E(tag, "Unbekannte Chip-ID: 0x%02X (erwartet 0x%02X)", chip_id, BME280_CHIP_ID);
        return false;
    }

    // 3. Kalibrierungsdaten laden
    if (!load_calibration()) return false;

    // 4. Feuchte-Überabtastung konfigurieren (muss VOR ctrl_meas gesetzt werden)
    if (!reg_write(REG_CTRL_HUM, OSRS_HUM_X1)) {
        gxLog_E(tag, "ctrl_hum schreiben fehlgeschlagen");
        return false;
    }

    // config-Register: kein IIR-Filter, kein Standby (Forced-Mode braucht das nicht)
    if (!reg_write(REG_CONFIG, 0x00)) {
        gxLog_E(tag, "config schreiben fehlgeschlagen");
        return false;
    }

    s_initialized = true;

    gxLog_I(tag, "Init OK @ 0x%02X | Chip-ID: 0x%02X | Forced-Mode, ×1 Oversample", s_address, chip_id);
    return true;

}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

bool bme280_measure(sensor_meas_item_t *temp, sensor_meas_item_t *hum, sensor_meas_item_t *pres)
{
    temp->value_f = 0.0f;
    temp->value_u16  = 0;
    temp->is_valid = false;

    hum->value_f = 0.0f;
    hum->value_u16  = 0;
    hum->is_valid = false;

    pres->value_f = 0.0f;
    pres->value_u16  = 0;
    pres->is_valid = false;

    if (!s_initialized) {
        gxLog_E(tag, "bme280_measure() vor bme280_init()");
        return false;
    }

    // 1. Forced-Mode starten (Sensor misst einmalig, geht dann in Sleep)
    if (!reg_write(REG_CTRL_MEAS, CTRL_MEAS_FORCED)) {
        gxLog_E(tag, "Forced-Mode starten fehlgeschlagen");
        return false;
    }

    // 2. Auf Messende warten (Status-Bit measuring = 0)
    const uint32_t t0 = millis();
    while (millis() - t0 < MEASURE_WAIT_MS) {
        uint8_t status = 0;
        if (reg_read(REG_STATUS, &status, 1) && (status & 0x08) == 0) break;
        delay(2);
    }

    // 3. Rohdaten lesen: 0xF7..0xFE (8 Bytes Druck+Temp) + 0xFD..0xFE (3 Bytes Feuchte)
    // Burst-Read ab 0xF7 liefert: press[7:0], temp[7:0], hum[7:0] zusammenhängend
    uint8_t raw[8];
    if (!reg_read(REG_PRESS_MSB, raw, 8)) {
        gxLog_E(tag, "Rohdaten lesen fehlgeschlagen");
        return false;
    }

    // Rohdaten zusammensetzen (20-Bit-Werte, linksbündig in 3 Bytes)
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  |  (int32_t)raw[7];

    // 4. Kompensation anwenden (Reihenfolge wichtig: Temp zuerst wegen t_fine)
    int32_t t_fine = 0;
    temp->value_f = compensate_temperature(adc_T, &t_fine);
    pres->value_f = compensate_pressure(adc_P, t_fine);
    hum->value_f  = compensate_humidity(adc_H, t_fine);

    temp->is_valid = true;
    pres->is_valid = true;
    hum->is_valid = true;

    gxLog_D(tag, "%.2f °C  %.1f %%RH  %.2f hPa", temp->value_f, hum->value_f, pres->value_f);
    return true;

}

#endif