#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file settings.h
 * @brief
 *
 */

// ===========================================================================
// Konfiguration
// ===========================================================================

#define VERSION "V1.3 (22.07.2026)"

#define SENSOR_00

/** Gerätename im BLE-Advertising */
#if defined(SENSOR_00)
  #define BLE_DEVICE_NAME     "ESP32-BLE-Sensor00"

  #define DEEPSLEEP_DURATION_MS  (600UL * 1000UL)

  // #define USE_SENSOR_HTU21
  #define USE_SENSOR_BME280
  // #define USE_SENSOR_BH1750
  #define USE_BATTERY
  #define USE_LED_ON_USB_PLUG

  #define BATTERY_DIVIDER_RATIO   0.485f

  #define BLE_DEVICE_TX_POWER  15

  #define LOG_LVL_MAIN    GXLOG_LVL_INFO
  #define LOG_LVL_SENSORS GXLOG_LVL_ERROR
  #define LOG_LVL_BH1750  GXLOG_LVL_ERROR
  #define LOG_LVL_AKKU    GXLOG_LVL_ERROR
  #define LOG_LVL_I2C     GXLOG_LVL_ERROR
  #define LOG_LVL_BLE     GXLOG_LVL_INFO

#elif defined(SENSOR_01)
  #define BLE_DEVICE_NAME     "ESP32-BLE-Sensor01"

  // #define DEEPSLEEP_DISABLE

  #define DEEPSLEEP_DURATION_MS  (600UL * 1000UL)

  #define USE_SENSOR_SHT3X
  // #define USE_SENSOR_HTU21
  // #define USE_SENSOR_BME280
  #define USE_BATTERY
  #define USE_LED_ON_USB_PLUG

  constexpr float    BATTERY_DIVIDER_RATIO = 0.5f;

  #define BLE_DEVICE_TX_POWER  15

  #define LOG_LVL_MAIN    GXLOG_LVL_INFO
  #define LOG_LVL_SENSORS GXLOG_LVL_ERROR
  #define LOG_LVL_BH1750  GXLOG_LVL_ERROR
  #define LOG_LVL_AKKU    GXLOG_LVL_ERROR
  #define LOG_LVL_I2C     GXLOG_LVL_ERROR
  #define LOG_LVL_BLE     GXLOG_LVL_INFO

#else
    #error "Kein Board definiert !"
#endif

// ----------------------------------------------------------------------------------------

/** für debug measure-intervall im No-DeepSleep-Modus [ms]. Default: 30 Sekunden */
#define NO_DEEPSLEEP_DURATION_MS    (14UL * 1000UL)

/** Timeout für BLE-Verbindungswartezeit im Deep-Sleep-Modus [ms] */
#define BLE_CONNECT_TIMEOUT_MS      (30UL * 1000UL)


/** ACK-Wartezeit pro Sendeversuch [ms] - typischer ACK kommt in <100ms */
#define BLE_ACK_TIMEOUT_MS          (500UL)

/** Anzahl Sendeversuche (Original + Wiederholungen) bis ACK eintrifft */
#define BLE_SEND_MAX_RETRIES        (5U)

/**
 * Harte Obergrenze fuer die gesamte Sendephase (Warten auf Connect/Subscribe
 * inkl. aller Sendeversuche) [ms]. Wird diese Zeit ueberschritten, wird der
 * Sendevorgang abgebrochen und der Controller geht in den Deep-Sleep zurueck.
 * Verhindert die gelegentlich beobachtete elapsed_time von ~18000 ms.
 */
#define BLE_SEND_DEADLINE_MS        (3UL * 1000UL)

/**
 * DeepSleep per GP3-Pin (DEEPSLEEP_DISABLE_PIN) deaktivierbar.
 * Auskommentieren = Pin wird ignoriert, kein pinMode/digitalRead.
 * Default: aktiviert
*/

#define _ON_  true
#define _OFF_ false

/**
 * DeepSleep per Serial-'x' deaktivierbar.
 * Auskommentieren = kein Warten auf 'x', kein extra Delay.
 * Default: deaktiviert
 */
// #define DEEPSLEEP_SERIAL_DISABLE_ENABLE

/** Serielle Baudrate */
#define SERIAL_BAUD (115200UL)


// ---------------------------------------------------------------------------
// Aktiver Umgebungssensor: genau EINES der folgenden Defines aktiv lassen.
// SHT3x  = Temperatur + Luftfeuchte (kein Druck)
// HTU21  = Temperatur + Luftfeuchte (kein Druck, kompatibel: SHT21/SHT20/SHT25)
// BME280 = Temperatur + Luftfeuchte + Luftdruck

