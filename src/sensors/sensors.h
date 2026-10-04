#pragma once

/**
 * @file sensors.h
 * @brief
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "settings.h"

// ---------------------------------------------------------------------------
// Datenstruktur für Messergebnis
// ---------------------------------------------------------------------------


namespace SENSOR
{
    typedef enum
    {
        NA = -1,
        SHT3X = 0,
        HTU21 = 1,
        BME280 = 2
    } Type;

    typedef enum
    {
        NO_ERROR    = 0x0000,
        TEMPERATURE = 0x0001,
        HUMIDITY    = 0x0002,
        PRESSURE    = 0x0004,
        ILLUMINANCE = 0x0008,
        BATTERY     = 0x0010,
        I2C         = 0x0100,
        SEND_ERROR  = 0x8000
    } ErrorCode;
}

typedef struct {
    float value_f;
    uint16_t value_u16;
    bool is_valid;
    bool is_enabled;
}sensor_meas_item_t;

// typedef struct {
//     sensor_meas_item_t temperature;
//     sensor_meas_item_t humidity;
//     sensor_meas_item_t pressure;
//     sensor_meas_item_t illuminance;
// } sensor_measurement_t;

// typedef struct {
//     float temperature_c;    /**< Temperatur in °C, Bereich -45 … +130 */
//     float humidity_pct;     /**< Relative Luftfeuchte in %, Bereich 0 … 100 */
//     float pressure_hpa;     /**< Luftdruck in hPa, Bereich 300 … 1100 */
//     float illuminance_lx;  /**< Beleuchtungsstärke in Lux, Bereich 0 … ~54612 */
//     bool temperature_valid;
//     bool humidity_valid;
//     bool pressure_valid;
//     bool illuminance_valid;
// } sensor_measurement_t;

// ===========================================================================
// Strukturhalter für Messwerte
// ===========================================================================

typedef struct {
    SENSOR::Type sensor_type;
    uint16_t error_code;
    #if defined(USE_SENSOR_SHT3X) || defined(USE_SENSOR_HTU21)
    sensor_meas_item_t temperature;
    sensor_meas_item_t humidity;

    #elif defined(USE_SENSOR_BME280)
    sensor_meas_item_t temperature;
    sensor_meas_item_t humidity;
    sensor_meas_item_t pressure;
    #endif

    #if defined(USE_SENSOR_BH1750)
    sensor_meas_item_t illuminance;
    #endif

    #if defined(USE_BATTERY)
    sensor_meas_item_t battery;
    #endif

}env_result_t;


bool sensor_init(env_result_t *out);

void sensor_measure(env_result_t *out);

// SENSOR::Type sensor_type();
