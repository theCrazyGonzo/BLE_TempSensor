#pragma once

#include <stdint.h>
#include <Arduino.h>

/**
 * @file pinout.h
 * @brief Zentrale Pin- und Hardware-Konfiguration für den ESP32-C6 Sensor-Node
 *
 * Diese Datei ist die EINZIGE Stelle, an der GPIO-Nummern,
 * Spannungsteiler-Faktoren und Sensor-Power-Konventionen definiert werden.
 * Alle anderen Module (sensors/, ble/, cpu/) verwenden ausschließlich
 * diese Konstanten.
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 * ---------------------------------------------------------------------------
 * 2026-06-12 20:04 - Pin-Belegung an tatsächliche Hardware angepasst:
 *                    I2C_SDA: GP1 → GP5
 *                    I2C_SCL: GP0 → GP4
 *                    BATTERY_ADC: GP4 → GP1
 *                    DEEPSLEEP_DISABLE_PIN: GP3 → GP20
 *
 * 2026-06-06 19:00 - GP3 als DEEPSLEEP_DISABLE_PIN ergänzt
 *
 * ---------------------------------------------------------------------------
 * Hardware-Übersicht: Super Mini ESP32-C6
 * ---------------------------------------------------------------------------
 *
 *      GP5 ───┬─── SDA (I²C)   ──► HTU21D, BH1750
 *      GP4 ───┴─── SCL (I²C)   ──► HTU21D, BH1750
 *
 *      GP3 ──────► VDD-Schalter (HIGH = Sensoren versorgt,
 *                                LOW  = Sensoren stromlos im Deep-Sleep)
 *
 *      GP20 ─────► DeepSleep-Disable (HIGH beim Boot = kein DeepSleep)
 *
 *      GP1 ──────► ADC ◄── Spannungsteiler 1:2 ◄── V_AKKU
 *                  (V_adc = V_akku / 2)
 *
 * ---------------------------------------------------------------------------
 */

namespace pinout {

constexpr uint8_t LED_PIN = 15;

/** Logischer Pegel: LED EIN */
constexpr uint8_t LED_ON  = HIGH;

/** Logischer Pegel: LED AUS */
constexpr uint8_t LED_OFF = LOW;

// ===========================================================================
// I²C-Bus (SHT3x + BH1750)
// ===========================================================================

/** SDA-Pin des I²C-Bus */
constexpr uint8_t I2C_SDA          = 5;

/** SCL-Pin des I²C-Bus */
constexpr uint8_t I2C_SCL          = 4;

/** I²C-Bustakt in Hz (Standard-Mode: 100 kHz, Fast-Mode: 400 kHz) */
constexpr uint32_t I2C_FREQUENCY   = 400000UL;

// ===========================================================================
// Sensor-Stromversorgung (Power-Switch)
// ===========================================================================

/**
 * GPIO, das die Versorgungsspannung der I²C-Sensoren schaltet.
 *
 * HIGH = Sensoren EIN  (vor jeder Messung setzen, danach kurz warten,
 *                       bis die Sensoren bereit sind - siehe SENSOR_BOOT_MS)
 * LOW  = Sensoren AUS  (zwingend vor dem Eintritt in Deep-Sleep,
 *                       sonst dauerhaft Stromverbrauch)
 */
constexpr uint8_t SENSOR_POWER     = 3;

/** Logischer Pegel: Sensoren EIN */
constexpr uint8_t SENSOR_POWER_ON  = HIGH;

/** Logischer Pegel: Sensoren AUS */
constexpr uint8_t SENSOR_POWER_OFF = LOW;

/**
 * Wartezeit (ms) nach dem Einschalten, bis die Sensoren bereit sind.
 *
 * - SHT3x: 1 ms Soft-Reset-Boot, max. ~15 ms nach Power-On
 * - BH1750: ~10 ms Power-On bis erste Messung möglich
 * → 50 ms ist ein sicherer, gemeinsamer Wert
 */
constexpr uint32_t SENSOR_BOOT_MS  = 50UL;

// ===========================================================================
// DeepSleep-Disable-Pin
// ===========================================================================

/**
 * GPIO, das beim Boot gelesen wird, um den DeepSleep zu deaktivieren.
 *
 * HIGH beim Start = DeepSleep für diese Laufzeit deaktiviert.
 * LOW  beim Start = normaler DeepSleep-Betrieb.
 *
 * Interner Pull-Down wird aktiviert; der Pin muss also aktiv auf HIGH
 * gezogen werden (z.B. Jumper nach 3V3).
 */
constexpr uint8_t DEEPSLEEP_DISABLE_PIN = 20;

// ===========================================================================
// Akkuspannungs-Messung
// ===========================================================================

/**
 * ADC-Pin, an dem die geteilte Akkuspannung anliegt.
 * Der ESP32-C6 unterstützt ADC1 auf den GPIOs 0-6 (GP4 ist ADC1_CH4).
 */
constexpr uint8_t BATTERY_ADC      = 1;

/**
 * Spannungsteiler-Faktor.
 * V_adc = V_akku × (R2 / (R1 + R2))  →  hier R1 = R2  →  Faktor = 0.5
 *
 * Zur Umrechnung der gemessenen Spannung in die echte Akkuspannung:
 *   V_akku = V_adc / BATTERY_DIVIDER_RATIO
 *          = V_adc * 2.0
 */
//constexpr float    BATTERY_DIVIDER_RATIO = 0.5f;

/**
 * Anzahl ADC-Messungen, über die für ein stabiles Ergebnis gemittelt wird.
 * (Mehrfachsamplung gleicht ADC-Rauschen des ESP32-C6 aus.)
 */
constexpr uint8_t  BATTERY_SAMPLES = 16;

}  // namespace pinout
