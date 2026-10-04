/**
 * @file i2c_driver.cpp
 * @brief Implementierung des schlanken I²C-Treibers
 *
 * Backend: Arduino `Wire`-Library (Bestandteil des ESP32-Arduino-Cores,
 * keine externe Abhängigkeit).
 *
 * Fehler-Codes von Wire.endTransmission():
 *   0 = success
 *   1 = data too long for transmit buffer
 *   2 = NACK on address transmit  (Gerät nicht da)
 *   3 = NACK on data transmit
 *   4 = other error
 *   5 = timeout
 */

#include "i2c_driver.h"

#include <Arduino.h>
#include <Wire.h>
#include "log_colors.h"
#include "../services/gxLog.h"
#include "settings.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "i2c";


// ---------------------------------------------------------------------------
// Modul-interner Zustand
// ---------------------------------------------------------------------------

static bool s_initialized = false;

/** Übersetzt Wire.endTransmission()-Returncodes in Klartext. */
static const char *wire_error_str(uint8_t code)
{
    switch (code) {
        case 0: return "OK";
        case 1: return "Buffer-Overflow";
        case 2: return "NACK (Adresse) - Gerät nicht erreichbar";
        case 3: return "NACK (Daten)";
        case 4: return "anderer Fehler";
        case 5: return "Timeout";
        default: return "unbekannter Fehler";
    }
}

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

bool i2c_init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency_hz)
{
    gxLog_setLevel(tag, LOG_LVL_I2C);
    gxLog_setColor(tag, ANSI_ESC_FG_CYAN);

    if (s_initialized) {
        gxLog_I(tag, "Bereits initialisiert - überspringe");
        return true;
    }

    // Wire.begin() der ESP32-Arduino-Variante:
    //   bool begin(int sda, int scl, uint32_t frequency = 0)
    bool ok = Wire.begin(static_cast<int>(sda_pin),
                         static_cast<int>(scl_pin),
                         frequency_hz);

    if (!ok) {
        gxLog_E(tag, "Wire.begin(%u, %u, %lu Hz) fehlgeschlagen", sda_pin, scl_pin, (unsigned long)frequency_hz);
        return false;
    }

    s_initialized = true;
    gxLog_I(tag, "Initialisiert: SDA=GP%u, SCL=GP%u, %lu Hz", sda_pin, scl_pin, (unsigned long)frequency_hz);
    return true;
}

void i2c_deinit(void)
{
    if (!s_initialized) return;

    Wire.end();
    s_initialized = false;
    gxLog_I(tag, "Bus deinitialisiert");
}

bool i2c_is_initialized(void)
{
    return s_initialized;
}

// ---------------------------------------------------------------------------
// Datenübertragung
// ---------------------------------------------------------------------------

bool i2c_write(uint8_t slave_addr, const uint8_t *data, size_t length)
{
    if (!s_initialized) {
        gxLog_E(tag, "i2c_write() vor i2c_init()");
        return false;
    }
    if (data == nullptr || length == 0) {
        gxLog_E(tag, "i2c_write() mit NULL oder length=0");
        return false;
    }

    Wire.beginTransmission(slave_addr);

    size_t written = Wire.write(data, length);
    if (written != length) {
        gxLog_E(tag, "nur %zu von %zu Bytes in Puffer geschrieben", written, length);
        Wire.endTransmission(true);   // Bus freigeben
        return false;
    }

    uint8_t result = Wire.endTransmission(true);  // mit STOP
    if (result != 0) {
        gxLog_E(tag, "write@0x%02X: %s (Code %u)", slave_addr, wire_error_str(result), result);
        return false;
    }

    return true;
}

bool i2c_read(uint8_t slave_addr, uint8_t *buffer, size_t length)
{
    if (!s_initialized) {
        gxLog_E(tag, "i2c_read() vor i2c_init()");
        return false;
    }
    if (buffer == nullptr || length == 0) {
        gxLog_E(tag, "i2c_read() mit NULL oder length=0");
        return false;
    }

    size_t received = Wire.requestFrom(static_cast<uint16_t>(slave_addr),
                                       length,
                                       /*sendStop=*/ true);
    if (received != length) {
        gxLog_E(tag, "read@0x%02X: %zu von %zu Bytes erhalten", slave_addr, received, length);
        // Trotzdem den Puffer drainen, damit der nächste Aufruf sauber startet
        while (Wire.available()) (void)Wire.read();
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        if (!Wire.available()) {
            gxLog_E(tag, "read@0x%02X: vorzeitiges Bus-Ende bei Byte %zu", slave_addr, i);
            return false;
        }
        buffer[i] = static_cast<uint8_t>(Wire.read());
    }

    return true;
}

bool i2c_write_read(uint8_t slave_addr,
                    const uint8_t *write_data,  size_t write_length,
                    uint8_t       *read_buffer, size_t read_length)
{
    if (!s_initialized) {
        gxLog_E(tag, "i2c_write_read() vor i2c_init()");
        return false;
    }
    if (write_data == nullptr || write_length == 0 ||
        read_buffer == nullptr || read_length == 0) {
        gxLog_E(tag, "i2c_write_read() mit ungültigen Parametern");
        return false;
    }

    // --- Phase 1: Write mit REPEATED-START (kein STOP) ---
    Wire.beginTransmission(slave_addr);
    size_t written = Wire.write(write_data, write_length);
    if (written != write_length) {
        gxLog_E(tag, "write_read@0x%02X: Puffer-Overflow (%zu/%zu)", slave_addr, written, write_length);
        Wire.endTransmission(true);
        return false;
    }
    uint8_t result = Wire.endTransmission(/*sendStop=*/ false);
    if (result != 0) {
        gxLog_E(tag, "write_read@0x%02X: Write-Phase: %s (Code %u)", slave_addr, wire_error_str(result), result);
        return false;
    }

    // --- Phase 2: Read mit STOP ---
    size_t received = Wire.requestFrom(static_cast<uint16_t>(slave_addr),
                                       read_length,
                                       /*sendStop=*/ true);
    if (received != read_length) {
        gxLog_E(tag, "write_read@0x%02X: Read-Phase: %zu von %zu Bytes", slave_addr, received, read_length);
        while (Wire.available()) (void)Wire.read();
        return false;
    }
    for (size_t i = 0; i < read_length; i++) {
        if (!Wire.available()) {
            gxLog_E(tag, "write_read@0x%02X: vorzeitiges Bus-Ende bei Byte %zu", slave_addr, i);
            return false;
        }
        read_buffer[i] = static_cast<uint8_t>(Wire.read());
    }

    return true;
}

// ---------------------------------------------------------------------------
// Debug-Helfer
// ---------------------------------------------------------------------------

void i2c_scan(void)
{
    if (!s_initialized) {
        gxLog_E(tag, "i2c_scan() vor i2c_init()");
        return;
    }

    gxLog_I(tag, "Scanne Bus (0x03 - 0x77) …");

    uint8_t found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        uint8_t result = Wire.endTransmission(true);
        if (result == 0) {
            gxLog_I(tag, "       Gerät gefunden: 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        gxLog_I(tag, "Kein Gerät gefunden");
    } else {
        gxLog_I(tag, "Scan beendet: %u Gerät(e)", found);
    }
}
