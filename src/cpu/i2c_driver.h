#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @file i2c_driver.h
 * @brief Schlanker I²C-Treiber für den ESP32-C6 Sensor-Node
 *
 * Wrapper um die Arduino-`Wire`-Library mit einer Plain-C-API.
 * Wird von den Sensor-Modulen (sensors/sht3x, sensors/bh1750) verwendet,
 * damit diese keine direkte Wire-Abhängigkeit haben.
 *
 * Lebenszyklus:
 *   1. i2c_init(...)   - einmalig in setup() bzw. nach Power-On der Sensoren
 *   2. i2c_write / i2c_read / i2c_write_read   - nach Bedarf
 *   3. i2c_deinit()    - vor dem Eintritt in Deep-Sleep
 *
 * Thread-Safety:
 *   NICHT thread-safe. Vorgesehen für sequenzielle Nutzung im Haupt-Thread.
 *   (Auf dem ESP32-C6 mit unserem Programmfluss ist das gegeben.)
 *
 * Bus-Eigenschaften:
 *   - 7-Bit-Adressierung (Standard)
 *   - Max. ~128 Bytes pro Transfer (Wire-Buffer-Limit ESP32)
 */

// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den I²C-Bus auf den angegebenen GPIOs.
 *
 * @param sda_pin       GPIO-Nummer des SDA-Pins
 * @param scl_pin       GPIO-Nummer des SCL-Pins
 * @param frequency_hz  Bustakt in Hz (typ. 100000 oder 400000)
 *
 * @return true   Bus erfolgreich initialisiert
 * @return false  Initialisierung fehlgeschlagen
 */
bool i2c_init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency_hz);

/**
 * @brief Beendet den I²C-Bus und gibt die GPIOs frei.
 *
 * MUSS vor dem Eintritt in den Deep-Sleep aufgerufen werden, damit die
 * SDA/SCL-Pins keine Pull-Ups aktiv halten.
 */
void i2c_deinit(void);

/**
 * @brief Gibt zurück, ob i2c_init() erfolgreich ausgeführt wurde.
 */
bool i2c_is_initialized(void);

// ---------------------------------------------------------------------------
// Datenübertragung
// ---------------------------------------------------------------------------

/**
 * @brief Sendet eine Folge von Bytes an einen I²C-Slave.
 *
 * Erzeugt: START | (addr<<1|W) | data[0] | … | data[n-1] | STOP
 *
 * @param slave_addr  7-Bit-Slave-Adresse (z. B. 0x44 für SHT3x)
 * @param data        Zeiger auf zu sendende Bytes
 * @param length      Anzahl der Bytes
 *
 * @return true   ACK von allen Bytes erhalten
 * @return false  NACK, Bus-Fehler oder ungültige Parameter
 */
bool i2c_write(uint8_t slave_addr, const uint8_t *data, size_t length);

/**
 * @brief Liest eine Folge von Bytes von einem I²C-Slave.
 *
 * Erzeugt: START | (addr<<1|R) | <data n bytes> | STOP
 *
 * @param slave_addr  7-Bit-Slave-Adresse
 * @param buffer      Zielpuffer (muss ≥ length Bytes groß sein)
 * @param length      Anzahl zu lesender Bytes
 *
 * @return true   alle Bytes erhalten
 * @return false  Fehler oder weniger Bytes empfangen
 */
bool i2c_read(uint8_t slave_addr, uint8_t *buffer, size_t length);

/**
 * @brief Schreibt Bytes (Befehl/Register-Adresse), dann liest Bytes.
 *
 * Erzeugt: START | (addr<<1|W) | wdata | REPEATED-START | (addr<<1|R)
 *          | <rdata n bytes> | STOP
 *
 * Klassisches Pattern für „Register lesen" oder „Befehl senden, Antwort holen".
 *
 * @param slave_addr    7-Bit-Slave-Adresse
 * @param write_data    Zeiger auf zu sendende Bytes (z. B. Register-Adresse)
 * @param write_length  Anzahl zu sendender Bytes
 * @param read_buffer   Zielpuffer
 * @param read_length   Anzahl zu lesender Bytes
 *
 * @return true   Beide Phasen erfolgreich
 * @return false  Fehler in einer der beiden Phasen
 */
bool i2c_write_read(uint8_t slave_addr,
                    const uint8_t *write_data,  size_t write_length,
                    uint8_t       *read_buffer, size_t read_length);

// ---------------------------------------------------------------------------
// Debug-Helfer
// ---------------------------------------------------------------------------

/**
 * @brief Scannt den Bus (Adressbereich 0x03-0x77) und gibt gefundene
 *        Geräte auf der seriellen Konsole aus.
 *
 * Nützlich beim Hardware-Setup, um Adressen zu verifizieren.
 */
void i2c_scan(void);
