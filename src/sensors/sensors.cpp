/**
 * @file sensors.cpp
 * @brief
 */

#include <Arduino.h>
#include "../cpu/i2c_driver.h"
#include "../cpu/log_colors.h"
#include "../services/gxLog.h"
#include "settings.h"
#include "sensors.h"

#ifdef USE_SENSOR_BH1750
    #include "sensors/bh1750.h"
#endif

#if defined(USE_SENSOR_SHT3X)
  #include "sht3x.h"
#elif defined(USE_SENSOR_HTU21)
  #include "htu21.h"
#elif defined(USE_SENSOR_BME280)
  #include "bme280.h"
#else
#  error "Keinen Sensor aktiviert - genau eines von USE_SENSOR_SHT3X / USE_SENSOR_HTU21 / USE_SENSOR_BME280 definieren"
#endif

// ---------------------------------------------------------------------------
static constexpr char tag[] = "sensors";

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------

// namespace {


// ---------------------------------------------------------------------------
// Lebenszyklus
// ---------------------------------------------------------------------------

// ------------------------------------------------------------
bool sensor_init(env_result_t *out)
{
    gxLog_setLevel(tag, LOG_LVL_SENSORS);
    gxLog_setColor(tag, ANSI_ESC_FG_BRIGHT_BLUE);

    gxLog_I(tag, "sensor_init");

    #if defined(USE_SENSOR_SHT3X)
        out->sensor_type = SENSOR::SHT3X;
        if( sht3x_init() ) {
            out->temperature.is_valid = true;
            out->humidity.is_valid    = true;
        }
        else {
            out->error_code = out->error_code | SENSOR::TEMPERATURE;
            out->error_code = out->error_code | SENSOR::HUMIDITY;
            gxLog_E(tag, "sensor_init (temp) failed");
        }
    #elif defined(USE_SENSOR_HTU21)
        out->sensor_type = SENSOR::HTU21;
        if( htu21_init() ) {
            out->temperature.is_valid = true;
            out->humidity.is_valid    = true;
        }
        else {
            out->error_code = out->error_code | SENSOR::TEMPERATURE;
            out->error_code = out->error_code | SENSOR::HUMIDITY;
            gxLog_E(tag, "sensor_init (temp) failed");
        }
    #elif defined(USE_SENSOR_BME280)
        out->sensor_type = SENSOR::BME280;
        if( bme280_init() ) {
            out->temperature.is_valid = true;
            out->humidity.is_valid    = true;
            out->pressure.is_valid    = true;
        }
        else {
            out->error_code = out->error_code | SENSOR::TEMPERATURE;
            out->error_code = out->error_code | SENSOR::HUMIDITY;
            out->error_code = out->error_code | SENSOR::PRESSURE;
            gxLog_E(tag, "sensor_init (temp) failed");
        }
    #endif

    // #ifdef USE_SENSOR_BH1750
    //     if( bh1750_init() ) {
    //         out->illuminance.is_valid = true;
    //     }
    //     else {
    //         out->error_code = out->error_code | SENSOR::ILLUMINANCE;
    //         gxLog_E(tag, "sensor_init (illuminance) failed");
    //     }

    // #endif

    return(true);
}

// ---------------------------------------------------------------------------
// Messung
// ---------------------------------------------------------------------------

// ------------------------------------------------------------
void sensor_measure(env_result_t *out)
{
    gxLog_I(tag, "sensor_measure");

    #if defined(USE_SENSOR_SHT3X)
        sht3x_measure(&out->temperature, &out->humidity);

    #elif defined(USE_SENSOR_HTU21)
        htu21_measure(&out->temperature, &out->humidity);

    #elif defined(USE_SENSOR_BME280)
        bme280_measure(&out->temperature, &out->humidity, &out->pressure);

    #endif

    // #ifdef USE_SENSOR_BH1750
    //     bh1750_measure(&out->illuminance);
    // #endif
}


// }
