/**
 * @file bh1750.cpp
 * @brief Implementierung des BH1750(FVI)-Treibers
 *
 * Datenblatt-Referenz:
 *   ROHM Semiconductor BH1750FVI, Rev. D (Nov. 2011)
 *   Tab. "Instruction Set Architecture"
 *   §  "Measurement Time"
 *   §  "Measurement accuracy"
 *
 * Hinweis: Der BH1750 hat keine CRC-Absicherung. Die Werte sind
 * unverschlüsselt 16-Bit-Big-Endian.
 */

#include "settings.h"

#ifdef USE_SENSOR_BH1750

#include <Arduino.h>
#include "../cpu/i2c_driver.h"
#include "../cpu/log_colors.h"
#include "../services/gxLog.h"
#include "bh1750.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "bh1750";

// ---------------------------------------------------------------------------
// BH1750-Befehle (Single-Byte)
// ---------------------------------------------------------------------------

namespace {

constexpr uint8_t CMD_POWER_DOWN          = 0x00;
constexpr uint8_t CMD_POWER_ON            = 0x01;
constexpr uint8_t CMD_RESET               = 0x07;   // setzt Datenregister auf 0,
                                                    // nur in Power-On-Zustand

// One-Time Modes: nach der Messung geht der Sensor automatisch in Power-Down.
constexpr uint8_t CMD_ONESHOT_H_RES       = 0x20;   // 1   lx Auflösung, 120/180 ms
constexpr uint8_t CMD_ONESHOT_H_RES_2     = 0x21;   // 0,5 lx Auflösung, 120/180 ms
constexpr uint8_t CMD_ONESHOT_L_RES       = 0x23;   // 4   lx Auflösung,  16/ 24 ms

// Wartezeit nach Mess-Befehl im H-Resolution-Modus.
// Datenblatt: typ. 120 ms, max. 180 ms → wir nehmen den Maximalwert + Marge.
constexpr uint32_t MEASURE_DELAY_MS       = 180;

/**
 * Lux-Umrechnungsfaktor (siehe Datenblatt "Measurement accuracy").
 *
 *   Lux = rohwert / 1.2
 *
 * Gilt für H-Resolution mit Default-MTReg (= 69). Der Faktor 1.2 ist
 * herstellerseitig fest und nicht konfigurierbar.
 */
constexpr float    LUX_DIVIDER            = 1.2f;

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Modul-interner Zustand
// ---------------------------------------------------------------------------

static uint8_t s_address     = BH1750_DEFAULT_ADDR;
static bool    s_initialized = false;
static bool    s_has_error   = false;

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

void bh1750_set_address(uint8_t addr)
{
    if (addr != BH1750_DEFAULT_ADDR && addr != BH1750_ALT_ADDR) {
        gxLog_W(tag, "ungewöhnliche Adresse 0x%02X (typ. 0x23 oder 0x5C)", addr);
    }
    s_address     = addr;
    s_initialized = false;   // neue Adresse → erneute Initialisierung nötig
}

uint8_t bh1750_get_address(void)
{
    return s_address;
}

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

bool bh1750_init(void)
{
    gxLog_setLevel(tag, LOG_LVL_BH1750);
    gxLog_setColor(tag, ANSI_ESC_FG_MAGENTA);

    if (!i2c_is_initialized()) {
        gxLog_E(tag, "I²C nicht initialisiert. (zuerst i2c_init() aufrufen)");
        s_has_error = true;
        return false;
    }

    // Single-byte command: Power-On
    const uint8_t cmd = CMD_POWER_ON;
    if (!i2c_write(s_address, &cmd, 1)) {
        gxLog_E(tag, "Power-On @ 0x%02X fehlgeschlagen (Sensor nicht erreichbar?)", s_address);
        s_has_error = true;
        return false;
    }

    s_initialized = true;
    s_has_error = false;

    gxLog_I(tag, "Init OK @ 0x%02X | Modus: One-Time H-Res (180 ms / 1 lx)", s_address);
    return true;
}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

bool bh1750_measure(sensor_meas_item_t *out)
{
    out->value_f = 0;
    out->value_u16 = 0;
    out->is_valid = false;

    if( s_has_error )
        return false;

    if (out == nullptr) {
        gxLog_E(tag, "bh1750_measure() mit NULL-Pointer");
        return false;
    }
    if (!s_initialized) {
        gxLog_E(tag, "bh1750_measure() vor bh1750_init()");
        return false;
    }

    // --- 1. One-Time-Mess-Befehl senden ---
    const uint8_t cmd = CMD_ONESHOT_H_RES;
    if (!i2c_write(s_address, &cmd, 1)) {
        gxLog_E(tag, "Mess-Befehl @ 0x%02X fehlgeschlagen", s_address);
        return false;
    }

    // --- 2. Auf Messung warten (max. Wartezeit aus Datenblatt) ---
    delay(MEASURE_DELAY_MS);

    // --- 3. 2 Bytes lesen: MSB, LSB (Big-Endian, keine CRC) ---
    uint8_t raw[2];
    if (!i2c_read(s_address, raw, sizeof(raw))) {
        gxLog_E(tag, "Datenbytes-Read @ 0x%02X fehlgeschlagen", s_address);
        return false;
    }

    // --- 4. Lux berechnen (siehe Datenblatt-Formel) ---
    uint16_t raw_value = (static_cast<uint16_t>(raw[0]) << 8) | raw[1];
    out->value_f = static_cast<float>(raw_value) / LUX_DIVIDER;
    out->is_valid = true;

    // Sensor ist nun automatisch wieder im Power-Down (One-Time-Modus)
    return true;
}

#endif