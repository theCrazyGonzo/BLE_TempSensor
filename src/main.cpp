/** ---------------------------------------------------------------------------
 *
 * @file main.cpp
 * @brief Hauptprogramm des ESP32-C6 Sensor-Nodes mit Deep-Sleep
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 * ---------------------------------------------------------------------------
 * 2026-07-18 - send_ble(): Harte Sende-Deadline BLE_SEND_DEADLINE_MS (3 s) ab
 *                    Boot. Warten auf Connect/Subscribe und alle Sendeversuche
 *                    werden spaetestens nach 3 s abgebrochen, dann Deep-Sleep.
 *                    Behebt gelegentliche elapsed_time von ~18000 ms. Ein
 *                    abgebrochener/unbestaetigter Versuch setzt g_pending_send_error
 *                    (RTC-RAM); die naechste gesendete Nachricht fuehrt dann das
 *                    Bit SENSOR::SEND_ERROR im error_code.
 *
 * 2026-06-28 10:34 - enter_deep_sleep(): Wachzeit (millis()-g_start_ms) von der
 *                    Soll-Schlafdauer abgezogen, damit der Gesamt-Zyklus (Wach +
 *                    Schlaf) ungefaehr DEEPSLEEP_DURATION entspricht. Vorher war
 *                    der reale Intervall = DEEPSLEEP_DURATION + Wachzeit (zu lang).
 *                    Untergrenze MIN_DEEPSLEEP_US falls Wachzeit >= Soll.
 *
 * 2026-06-27 16:16 - TIMING-Ausgaben wieder entfernt (waren temporaer zur
 *                    Connect-Latenz-Diagnose): Test-Konfig-Zeile und Phasen-
 *                    Marker (setup-start, advertising aktiv) raus.
 *
 * 2026-06-13 07:36 - HTU21 als dritten wählbaren Sensor ergänzt (USE_SENSOR_HTU21);
 *                    Compile-Guard, Include, env_result_t, measure(),
 *                    build_json() und Logs auf drei Sensoren erweitert.
 *
 * 2026-06-13 07:01 - Sensor-Auswahl per Define: USE_SENSOR_SHT3X (Default)
 *                    oder USE_SENSOR_BME280 in Konfigurationsblock wählbar.
 *                    Compile-Error bei keiner oder doppelter Auswahl.
 *                    env_result_t, measure(), build_json() und Logs
 *                    per #ifdef umgeschaltet.
 *
 * 2026-06-12 20:18 - SHT3x durch BME280 ersetzt: htu21 → bme280, env_result_t
 *                    um pressure_hpa + bme280_ok erweitert, build_json() und
 *                    Logs angepasst. SHT3x-Treiber bleibt erhalten.
 *
 * 2026-06-10 08:15 - BLE_ACK_TIMEOUT_MS 200ms → 500ms: 200ms war zu kurz,
 *                    Python-ACK-Write über BLE braucht 30-150ms; ESP sendete
 *                    alle 5 Retries bevor der erste ACK ankam.
 *
 * 2026-06-09 08:28 - sht21 → htu21 umbenannt (verbauter Sensor ist HTU21D)
 *
 * ---------------------------------------------------------------------------
 * 2026-06-07 20:06 - Energie-Optimierungen Wachzeit:
 *                    (1) delay(1500) fuer USB-CDC entfernt im DeepSleep-Modus
 *                        (nur noch im NoDeepSleep-Modus aktiv, dort hilfreich
 *                         fuer Debugging).
 *                    (3) delay(200) nach ACK entfernt (ACK ist bereits die
 *                        Bestaetigung dass Daten angekommen sind).
 *                    (6) BLE_ACK_TIMEOUT_MS von 500ms auf 200ms gesenkt
 *                        (typischer ACK kommt in <100ms).
 *
 * 2026-06-07 10:48 - Application-Layer-ACK eingebaut: nach ble_send_json() wird
 *                    auf BLE_ACK_BYTE vom Peer gewartet (ble_wait_for_ack).
 *                    Bei Timeout bis zu BLE_SEND_MAX_RETRIES Wiederholungen.
 *                    Erst NACH ACK (oder allen Retries) wird Deep-Sleep
 *                    eingeleitet. Schliesst die Notify-Verlust-Luecke.
 *
 * 2026-06-07 08:29 - DeepSleep-Setup wartet jetzt auf ble_is_subscribed()
 *                    statt nur auf ble_is_connected(). Bisher wurde direkt
 *                    nach Connect gesendet - der Peer hatte aber noch nicht
 *                    Notify aktiviert -> Frame verworfen, Empfaenger erhielt
 *                    nichts. Subscribe braucht typ. 100-500 ms nach Connect.
 *
 * 2026-06-07 08:15 - Wartezeit nach Senden vor Deep-Sleep von 500ms auf 1500ms
 *                    erhoeht, damit Notify-PDU + ACK den Peer erreichen, bevor
 *                    ble_stop() den Disconnect einleitet (verhinderte unsaubere
 *                    Verbindungs-Beendigung -> GATT UNLIKELY_ERROR bei bleak).
 *
 * 2026-06-06 21:03 - Farbige Log-Tags (log_colors.h) eingebunden
 *
 * 2026-06-06 20:56 - IDF-Log fuer i2c.master auf NONE (SHT21 NACK-Polling-Spam)
 *
 * 2026-06-06 20:28 - JSON-Keys fehlerhafter Sensoren werden weggelassen
 *
 * 2026-06-06 19:45 - Elapsed-Zeit (ms seit Boot) in BLE-JSON eingefügt
 *
 * 2026-06-06 19:30 - SHT3x durch SHT21 ersetzt (neuer Sensor)
 *
 * 2026-06-06 19:20 - DEEPSLEEP_DISABLE Define ergänzt (Default: aktiviert)
 *
 * 2026-06-06 19:15 - DEEPSLEEP_SERIAL_DISABLE_ENABLE Define ergänzt (Default: deaktiviert)
 *
 * 2026-06-06 19:00 - Deep-Sleep implementiert:
 *                    - Aufwachen -> Messung parallel zu BLE-Verbindungswartezeit
 *                    - Daten senden wenn verbunden -> Deep-Sleep
 *                    - DeepSleep deaktivierbar via GP3 (HIGH) oder 'x' auf Serial
 *                    - Im Nicht-DeepSleep-Modus: Messung+Senden alle NO_DEEPSLEEP_DURATION_MS
 *
 * ---------------------------------------------------------------------------
 *
 * Ablauf im Deep-Sleep-Modus (Default):
 *   Boot → Pin GP3 & Serial 'x' prüfen → Messen (parallel zum BLE-Start)
 *   → Auf BLE-Client warten (max. BLE_CONNECT_TIMEOUT_MS)
 *   → Wenn verbunden: Daten senden → Deep-Sleep für DEEPSLEEP_DURATION_MS
 *   → Wenn nicht verbunden innerhalb Timeout: direkt Deep-Sleep
 *
 * Ablauf im No-DeepSleep-Modus (GP3=HIGH oder 'x' empfangen):
 *   Normaler Loop, alle NO_DEEPSLEEP_DURATION_MS messen + senden (wenn verbunden)
 *   DeepSleep wird in dieser Laufzeit nie aktiviert.
 *
 * --------------------------------------------------------------------------- */

// TODO: evtl. read_sensors und wait_for_client mit FreeRTOS als jeweils eigene Task

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "services/gxLog.h"

#include "ble/ble_device.h"
#include "cpu/pinout.h"
#include "cpu/i2c_driver.h"
#include "cpu/log_colors.h"
#include "settings.h"
#include "sensors.h"
#include "bh1750.h"
#include "akku.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "main";


// ===========================================================================
// Globale Startzeit für Elapsed-Messung
// ===========================================================================

void measure_send_and_sleep();

/** Zeitstempel des Programmstarts (gesetzt am Anfang von setup()) */
static uint32_t g_start_ms = 0;

/**
 * Merker fuer einen abgebrochenen Sendevorgang. Ueberlebt den Deep-Sleep im
 * RTC-RAM, damit die naechste gesendete Nachricht das Bit SENSOR::SEND_ERROR
 * im error_code fuehrt. Wird nach dem erfolgreichen Einbau in error_code
 * wieder zurueckgesetzt.
 */
RTC_DATA_ATTR static bool g_pending_send_error = false;

/** Anzahl Messungen pro I²C-Sensor pro Zyklus */
static constexpr uint8_t  SENSOR_SAMPLES        = 3;

/** Pause zwischen den 3 Sensor-Einzelmessungen */
static constexpr uint32_t SENSOR_SAMPLE_GAP_MS  = 10;

// --- Ausreißerschwellen für die 3×-Messung der I²C-Sensoren ---
static constexpr float    THRESHOLD_TEMP_C      = 0.5f;
static constexpr float    THRESHOLD_HUMI_PCT    = 3.0f;
static constexpr float    THRESHOLD_LUX         = 50.0f;

// ===========================================================================
// Globaler Laufzeitstatus (kein RTC-RAM - wird bei jedem Boot neu ermittelt)
// ===========================================================================

/** true = DeepSleep für diese Laufzeit deaktiviert */
static bool g_deepsleep_disabled = false;

// ===========================================================================
// Hilfsfunktionen: Median und Ausreißerfilter (für 3 Werte)
// ===========================================================================

/* Vergleichsfunktion für qsort (aufsteigend) */
static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;

    if (fa < fb) return -1;
    if (fa > fb) return  1;
    return 0;
}

// ------------------------------------------------------------
static float median(float *value, int len)
{
    if (value == NULL || len <= 0) {
        return NAN;
    }

    qsort(value, (size_t)len, sizeof(float), cmp_float);

    if (len % 2 == 1) {
        return value[len / 2];
    }
    return (value[len / 2 - 1] + value[len / 2]) / 2.0f;
}

// ------------------------------------------------------------
// static float median(float *v, int len)
// {
//     if( len != 3 ) {
//         gxLog_E(tag, "median only for 3 samples !");
//         return(0.0f);
//     }

//     for (int i = 0; i < 2; i++) {
//         for (int j = i + 1; j < 3; j++) {
//             if (v[i] > v[j]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
//         }
//     }
//     return v[1];
// }

// ------------------------------------------------------------
static float trimmed_mean(float *v, float threshold, int len )
{
    float med;
    float sum = 0.0f;
    int   n   = 0;

    med = median(v, len);
    if (isnan(med)) {
        return( NAN );
    }

    for(int n=0; n<len;n++) {
        if (fabsf(v[n] - med) <= threshold) { sum += v[n]; n++; }
    }

    return (n > 0) ? (sum / static_cast<float>(n)) : med;
}

// ===========================================================================
// Sensor-Power-Management
// ===========================================================================

// ------------------------------------------------------------
static inline void sensors_power(bool on)
{
    if( on )
        digitalWrite(pinout::SENSOR_POWER, pinout::SENSOR_POWER_ON);
    else
        digitalWrite(pinout::SENSOR_POWER, pinout::SENSOR_POWER_OFF);
}

// ------------------------------------------------------------
static inline void led(bool on)
{
    #ifdef USE_LED_ON_USB_PLUG
        if( on )
            digitalWrite(pinout::LED_PIN, pinout::LED_ON);
        else
            digitalWrite(pinout::LED_PIN, pinout::LED_OFF);
    #endif
}


// ===========================================================================
// Kompletter Messzyklus für BME280 + BH1750
// ===========================================================================

// ===========================================================================
// Kompletter Messzyklus
// ===========================================================================
#ifdef DEBUG_FAKE_VALUES
float m_temp = 20.0;
static bool measure(env_result_t *out)
{
    m_temp = m_temp + 0.5;
    if(m_temp >= 30) m_temp = 20.0;

    out->bh1750_ok = true;
    out->sensors_ok = true;
    out->humidity_pct = 51.2;
    out->temperature_c = m_temp;
    out->lux = 1234.5;
    out->valid_samples = 5;

    return(true);
}

#else
// ------------------------------------------------------------
static bool measure(env_result_t *env)
{
    env->error_code = SENSOR::NO_ERROR;

    // 1. Sensor-Power AN + Boot-Zeit
    sensors_power(true);
    delay(pinout::SENSOR_BOOT_MS);

    // 2. I²C + Sensor-Init
    if (!i2c_init(pinout::I2C_SDA, pinout::I2C_SCL, pinout::I2C_FREQUENCY)) {
        sensors_power(false);
        return false;
    }

    //--- init sensors
    sensor_init(env);

    #if defined(USE_SENSOR_BH1750)
        if( bh1750_init() == false ) {
            env->error_code |= SENSOR::HUMIDITY;
        }
    #endif

    #if defined(USE_BATTERY)
        if( akku_is_initialized() == false ) {
            if( akku_init() == false ) {
                    env->error_code |= SENSOR::BATTERY;
                }
            }
    #endif

    // 3. SENSOR_SAMPLES Einzelmessungen
    float temps[SENSOR_SAMPLES];
    float humis[SENSOR_SAMPLES];
    float press[SENSOR_SAMPLES];
    float luxes[SENSOR_SAMPLES];
    float akku[SENSOR_SAMPLES];
    env_result_t meas;

    for (uint8_t i = 0; i < SENSOR_SAMPLES; i++) {

        #if defined(USE_SENSOR_SHT3X) || defined(USE_SENSOR_HTU21) || defined(USE_SENSOR_BME280)

            sensor_measure(&meas);

            temps[i] = meas.temperature.value_f;
            if( !meas.temperature.is_valid ) {
                env->error_code |= SENSOR::TEMPERATURE;
            }

            humis[i] = meas.humidity.value_f;
            if( !meas.humidity.is_valid ) {
                env->error_code |= SENSOR::HUMIDITY;
            }

            #if defined(USE_SENSOR_BME280)
                press[i] = meas.pressure.value_f;
                if( !meas.pressure.is_valid ) {
                    env->error_code |= SENSOR::PRESSURE;
                }
            #endif

        #endif

        #if defined(USE_SENSOR_BH1750)

            bh1750_measure(&meas.illuminance);

            luxes[i] = meas.illuminance.value_f;

                if( !meas.illuminance.is_valid ) {
                    env->error_code |= SENSOR::ILLUMINANCE;
                }
        #endif

        #if defined(USE_BATTERY)
            akku_measure(&meas.battery);
            akku[i] = meas.battery.value_f;
            if( !meas.temperature.is_valid )
                env->error_code |= SENSOR::BATTERY;
        #endif

        if (i < SENSOR_SAMPLES - 1)
            delay(SENSOR_SAMPLE_GAP_MS);
    }

    // 4. Sensoren stromlos schalten
    i2c_deinit();
    sensors_power(false);

    // memset(&env, 0, sizeof(env_result_t));
    // env->error_code = meas.error_code;

    // 5. Ausreißergefilterten Mittelwert bilden
    #if defined(USE_SENSOR_SHT3X) || defined(USE_SENSOR_HTU21) || defined(USE_SENSOR_BME280)

        if( (env->error_code & SENSOR::TEMPERATURE) == false ) {
            env->temperature.value_f = trimmed_mean(temps, THRESHOLD_TEMP_C, SENSOR_SAMPLES);
            if (isnan(env->temperature.value_f)) {
                env->error_code = env->error_code | SENSOR::TEMPERATURE;
            }
        }
        if( (env->error_code & SENSOR::HUMIDITY) == false ) {
            env->humidity.value_f  = trimmed_mean(humis, THRESHOLD_HUMI_PCT, SENSOR_SAMPLES);
            if (isnan(env->humidity.value_f)) {
                env->error_code = env->error_code | SENSOR::HUMIDITY;
            }
        }

        #if defined(USE_SENSOR_BME280)
            if( (env->error_code & SENSOR::PRESSURE) == false ) {
                env->pressure.value_f  = trimmed_mean(press, 1.0f, SENSOR_SAMPLES);
                if (isnan(env->pressure.value_f)) {
                    env->error_code = env->error_code | SENSOR::PRESSURE; }
            }
        #endif
    #endif

    #if defined(USE_BATTERY)
        if( (env->error_code & SENSOR::BATTERY) == false ) {
            env->battery.value_f = trimmed_mean(akku, OUTLIER_THRESHOLD_MV, SENSOR_SAMPLES);
            if (isnan(env->battery.value_f)) {
                env->error_code = env->error_code | SENSOR::BATTERY; }
        }

        //---- calc akku-level
        if( (env->error_code & SENSOR::BATTERY) == false ) {
            if( (env->error_code & SENSOR::TEMPERATURE) == false ) {
                env->battery.value_u16 = akku_voltage_to_percent_temp_comp((uint16_t)env->battery.value_f, (uint16_t)env->temperature.value_f);
            }
            else {
                env->battery.value_u16 = akku_voltage_to_percent_temp_comp((uint16_t)env->battery.value_f, 25);
            }
        }
        else
            env->battery.value_u16 = 0;
    #endif

    #ifdef USE_SENSOR_BH1750
    if( (env->error_code & SENSOR::ILLUMINANCE) == false ) {
        env->illuminance.value_f = trimmed_mean(luxes, THRESHOLD_LUX, SENSOR_SAMPLES);
        if (isnan(env->illuminance.value_f)) {
            env->error_code = env->error_code | SENSOR::ILLUMINANCE; }
    }
    #endif

    return true;
}
#endif

// ===========================================================================
// JSON-Nachricht zusammenbauen (nur gültige Sensor-Keys)
// ===========================================================================

// ------------------------------------------------------------
/**
 * Baut den JSON-String aus den verfügbaren Messwerten.
 * Keys fehlerhafter Sensoren werden weggelassen.
 *
 * @param buf        Zielpuffer
 * @param buf_size   Größe des Puffers
 * @param env        Umgebungsmesswerte (Flags sht3x_ok/bme280_ok + bh1750_ok entscheiden)
 * @param batt       Akkumesswerte
 * @param batt_ok    true = Akkuwerte gültig
 * @param elapsed_ms Verstrichene Zeit seit Boot in ms
 */
static void build_json(char *buf, size_t buf_size, const env_result_t &env, uint32_t elapsed_ms)
{
    // Puffer mit öffnender Klammer initialisieren
    size_t pos = 0;
    buf[pos++] = '{';
    buf[pos]   = '\0';

    bool need_comma = false;

    // Hilfsmakro: hängt ein formatiertes Key-Value-Paar an
    #define JSON_APPEND(fmt, ...) do { \
        if (need_comma) { buf[pos++] = ','; buf[pos] = '\0'; } \
        pos += snprintf(buf + pos, buf_size - pos, fmt, ##__VA_ARGS__); \
        need_comma = true; \
    } while (0)

    #if defined(USE_SENSOR_SHT3X) || defined(USE_SENSOR_HTU21) || defined(USE_SENSOR_BME280)
        if( (env.error_code & SENSOR::TEMPERATURE) == false )
            JSON_APPEND("\"Temperature\":%.1f", env.temperature.value_f);
        if( (env.error_code & SENSOR::HUMIDITY) == false )
            JSON_APPEND("\"Humidity\":%d",      (int)env.humidity.value_f);

        #if defined(USE_SENSOR_BME280)
            if( (env.error_code & SENSOR::PRESSURE) == false )
                JSON_APPEND("\"Pressure\":%.1f",    env.pressure.value_f);
        #endif
    #endif

    #if defined(USE_SENSOR_BH1750)
        if( (env.error_code & SENSOR::ILLUMINANCE) == false )
            JSON_APPEND("\"Illuminance\":%.0f", env.illuminance.value_f);
    #endif

    #if defined(USE_BATTERY)
        if( (env.error_code & SENSOR::BATTERY) == false ) {
            // JSON_APPEND("\"Battery_Voltage\":%d",  (int)(batt.voltage * 1000.0f));
            // JSON_APPEND("\"Battery_Percent\":%u",  batt.percentage);
            JSON_APPEND("\"Battery\":%d", env.battery.value_u16);
            JSON_APPEND("\"Battery_mv\":%d", (uint16_t)env.battery.value_f);
            JSON_APPEND("\"elapsed_ms\":%d", (unsigned long)elapsed_ms);
            // JSON_APPEND("\"Debug\":{\"Battery_mV\":%d, \"elapsed_ms\":%lu}",  (uint16_t)env.battery.value_f, (unsigned long)elapsed_ms);
        }
    #else

        JSON_APPEND("\"Debug\":{\"elapsed_ms\":%lu}", (unsigned long)elapsed_ms);

    #endif

    JSON_APPEND("\"Version\":\"%s\"", VERSION);
    JSON_APPEND("\"Error\":\"0x%04x\"", env.error_code);


    #undef JSON_APPEND

    if (pos < buf_size - 1) {
        buf[pos++] = '}';
        buf[pos]   = '\0';
    }

}

// ===========================================================================
// Einen vollständigen Mess-Zyklus durchführen und per BLE senden
// ===========================================================================

// ------------------------------------------------------------
// static void measure_and_send(void)
// {
//     gxLog_I(tag, "---------- Neuer Messzyklus ----------");

//     //-------- read sensors and battery

//     //--- read Sensors
//     env_result_t env;
//     memset(&env, 0, sizeof(env_result_t));

//     measure(&env);


//     uint32_t elapsed_ms = millis() - g_start_ms;

//     #if defined(USE_SENSOR_BME280)
//         if( env.temperature.is_valid && env.humidity.is_valid && env.pressure.is_valid)
//             gxLog_I(tag, "->  Temp: %.2f °C  RH: %.2f %%  Druck: %.2f hPa", env.temperature.value_f, env.humidity.value_f, env.pressure.value_f);
//         else
//             gxLog_I(tag, "->  Temp, RH, Druck: not valid");
//     #endif
//     #if defined(USE_SENSOR_SHT3X) || defined(USE_SENSOR_HTU21)
//         if( env.temperature.is_valid && env.humidity.is_valid )
//             gxLog_I(tag, "->  Temp: %.2f °C  RH: %.2f %%", env.temperature.value_f, env.humidity.value_f);
//         else
//             gxLog_I(tag, "->  Temp, RH: not valid");
//     #endif

//     #if defined(USE_SENSOR_BH1750)
//         if( env.illuminance.is_valid )
//             gxLog_I(tag, "->  Licht: %.1f lx", env.illuminance.value_f);
//         else
//             gxLog_I(tag, "->  Licht: not valid");
//     #endif

//     #if defined(USE_BATTERY)
//         gxLog_I(tag, "->  Akku: %d V  (%u %%)", (uint16_t)env.battery.value_f,  env.battery.value_u16);
//     #endif

//     gxLog_I(tag, "->  Elapsed: %lu ms", (unsigned long)elapsed_ms);

//     char json[BLE_JSON_MAX_LEN];
//     build_json(json, sizeof(json), env, elapsed_ms);

//     if (!ble_send_json(json)) {
//         gxLog_E(tag, "BLE-Versand fehlgeschlagen");
//     }
// }

// ===========================================================================
// Deep-Sleep einleiten
// ===========================================================================

// ------------------------------------------------------------
static void enter_deep_sleep(void)
{
    gxLog_I(tag, "Deep-Sleep für %lu s ...", (unsigned long)(DEEPSLEEP_DURATION_MS / 1000UL));
    // Serial.flush();

    // BLE sauber beenden, damit der Stack beim nächsten Boot korrekt startet
    ble_stop();

    // Sensor-Power sicherstellen: AUS
    sensors_power(false);

    Serial.flush();
    delay(100);   // ausgehende HCI-/UART-Pakete rausschreiben lassen

    // Wachzeit von der Soll-Schlafdauer abziehen, damit der Gesamt-Zyklus
    // (Wachzeit + Schlaf) ungefaehr DEEPSLEEP_DURATION entspricht.
    static const uint64_t min_deepsleep_us = 1000000ULL;   // min. 1 s schlafen
    uint64_t awake_us  = (uint64_t)(millis() - g_start_ms) * 1000ULL;
    uint64_t target_us = (uint64_t)DEEPSLEEP_DURATION_MS  * 1000ULL;
    uint64_t sleep_us  = (awake_us < target_us) ? (target_us - awake_us) : min_deepsleep_us;

    gxLog_I(tag, "Wach %lu ms -> Deep-Sleep %lu s ...",
                  (unsigned long)(awake_us / 1000ULL), (unsigned long)(sleep_us / 1000000ULL));
    // Serial.flush();

    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
    // ab hier wird nichts mehr ausgeführt

}

// ===========================================================================
// DeepSleep-Disable prüfen (Pin GP3 und Serial 'x')
// Gibt true zurück wenn DeepSleep deaktiviert werden soll.
// ===========================================================================

// ------------------------------------------------------------
static bool check_deepsleep_disable(void)
{
#ifndef DEEPSLEEP_DISABLE
    // Pin GP3: interner Pull-Down, HIGH = DeepSleep deaktivieren
    // pinMode(pinout::DEEPSLEEP_DISABLE_PIN, INPUT_PULLDOWN);
    delay(5);  // kurz warten bis Pull-Down stabil
    if (digitalRead(pinout::DEEPSLEEP_DISABLE_PIN) == HIGH) {
        // gxLog_I(tag, "GP3=HIGH erkannt - DeepSleep deaktiviert");
        return true;
    }
    else {
        return false;
    }
#else
    return true;
#endif
}

// ===========================================================================
// Ausgabe der Messergebnisse
// ===========================================================================
void print_measurement(const env_result_t &env)
{
        #if defined(USE_BATTERY)
            if( env.error_code & SENSOR::BATTERY )
                gxLog_I(tag, "battery measurement failed !");
            else
                gxLog_I(tag, "Akku:   %d V (%u %%)", (uint16_t)env.battery.value_f, env.battery.value_u16);
        #endif

        #if defined(USE_SENSOR_SHT3X) || defined(USE_SENSOR_HTU21)
            if( (env.error_code & SENSOR::TEMPERATURE) ||  (env.error_code & SENSOR::HUMIDITY) )
                gxLog_I(tag, "sensor temperature, humidity measurement failed !");
            else
                gxLog_I(tag, "%.2f °C  %.2f %%RH", env.temperature.value_f, env.humidity.value_f);
        #endif

        #if defined(USE_SENSOR_BME280)
            if( env.error_code & SENSOR::TEMPERATURE & SENSOR::HUMIDITY & SENSOR::PRESSURE)
                gxLog_I(tag, "sensor temperature, humidity, pressure failed !");
            else
                gxLog_I(tag, "%.2f °C  %.2f %%RH  %.2f hPa", env.temperature.value_f, env.humidity.value_f, env.pressure.value_f);
        #endif

        #ifdef USE_SENSOR_BH1750
            if( env.error_code & SENSOR::ILLUMINANCE)
                gxLog_I(tag, "sensor illuminance failed !");
            else
                gxLog_I(tag, "%.1f lx", env.illuminance.value_f);
        #endif
}

// ===========================================================================
//  Senden
// ===========================================================================
void send_ble(const env_result_t &env)
{
        // Harte Obergrenze fuer die gesamte Sendephase. Wird sie erreicht,
        // wird abgebrochen und der Controller geht in den Deep-Sleep zurueck.
        // Deadline relativ zum Boot (g_start_ms), damit die bereits vor dem
        // Senden verstrichene Wachzeit mitzaehlt.
        const uint32_t send_deadline = g_start_ms + BLE_SEND_DEADLINE_MS;

        // Auf BLE-Client warten (Connect + Subscribe).
        // Beide noetig: Notify ohne Subscribe wird vom Stack verworfen.
        gxLog_I(tag,  "Warte auf BLE-Client (Connect + Subscribe) ...");
        uint32_t t0 = millis();
        // Effektive Connect-Wartezeit auf die verbleibende Deadline begrenzen.
        const uint32_t connect_timeout = (BLE_SEND_DEADLINE_MS < BLE_CONNECT_TIMEOUT_MS)
                                            ? BLE_SEND_DEADLINE_MS : BLE_CONNECT_TIMEOUT_MS;
        bool connected = false;
        bool subscribed = false;
        bool timed_out = false;
        while (millis() - t0 < connect_timeout) {
            if ((int32_t)(millis() - send_deadline) >= 0) {
                timed_out = true;
                break;
            }
            if (ble_is_subscribed()) {
                connected  = true;
                subscribed = true;
                break;
            }
            delay(50);
        }
        // Falls Connect war aber kein Subscribe: trotzdem als 'connected' loggen
        if (!subscribed && ble_is_connected()) {
            connected = true;
        }

        if (subscribed) {
            uint32_t elapsed_ms = millis() - g_start_ms;
            gxLog_D(tag, "Elapsed vor Senden: %lu ms", (unsigned long)elapsed_ms);

            // Wurde der vorige Sendevorgang abgebrochen, das SEND_ERROR-Bit in
            // den error_code dieser Nachricht uebernehmen und Merker loeschen.
            env_result_t env_tx = env;
            if (g_pending_send_error) {
                env_tx.error_code |= SENSOR::SEND_ERROR;
                g_pending_send_error = false;
                gxLog_I(tag, "SEND_ERROR aus vorigem Zyklus in error_code gesetzt");
            }

            // JSON bauen (nur gueltige Keys)
            char json[BLE_JSON_MAX_LEN] = { 0 };
            build_json(json, sizeof(json), env_tx, elapsed_ms);

            // Senden mit Retry bis Application-ACK eintrifft oder Versuche
            // aufgebraucht sind. Schliesst die Notify-Verlust-Luecke (BLE-
            // Notify hat selbst kein End-to-End-ACK).
            bool ack_ok = false;
            for (uint8_t attempt = 1; attempt <= BLE_SEND_MAX_RETRIES; attempt++) {
                // Deadline pruefen, bevor ein weiterer Versuch begonnen wird.
                if ((int32_t)(millis() - send_deadline) >= 0) {
                    gxLog_W(tag, "Sende-Deadline (%lums) erreicht - Abbruch", (unsigned long)BLE_SEND_DEADLINE_MS);
                    timed_out = true;
                    break;
                }
                ble_clear_ack();
                if (!ble_send_json(json)) {
                    gxLog_W(tag, "Sendeversuch %u/%u fehlgeschlagen", attempt, BLE_SEND_MAX_RETRIES);
                    delay(100);
                    continue;
                }
                if (ble_wait_for_ack(BLE_ACK_TIMEOUT_MS)) {
                    gxLog_I(tag, "ACK empfangen nach Versuch %u/%u", attempt, BLE_SEND_MAX_RETRIES);
                    ack_ok = true;
                    break;
                }
                gxLog_W(tag, "Kein ACK in %lums (Versuch %u/%u)", (unsigned long)BLE_ACK_TIMEOUT_MS, attempt, BLE_SEND_MAX_RETRIES);
                // Verbindung verloren? -> keine weiteren Versuche
                if (!ble_is_subscribed()) {
                    gxLog_W(tag, "Peer nicht mehr subscribed - Retry abgebrochen");
                    break;
                }
            }
            if (!ack_ok) {
                gxLog_E(tag, "Keine Bestaetigung nach %u Versuchen - Daten evtl. verloren", BLE_SEND_MAX_RETRIES);
                // Nicht bestaetigt -> naechste Nachricht mit SEND_ERROR markieren.
                g_pending_send_error = true;
            }
            else {
                gxLog_I(tag, "%s", json );
            }
            // delay(200) entfernt: ACK ist bereits die Bestaetigung dass die
            // Daten angekommen sind - kein weiterer Puffer noetig.
        } else if (timed_out) {
            gxLog_W(tag, "Sende-Deadline (%lums) erreicht - kein Client, Abbruch", (unsigned long)BLE_SEND_DEADLINE_MS);
            // Sendevorgang abgebrochen -> naechste Nachricht mit SEND_ERROR markieren.
            g_pending_send_error = true;
        } else if (!connected) {
            gxLog_W(tag, "BLE-Timeout - kein Client verbunden");
            g_pending_send_error = true;
        } else if (!subscribed) {
            gxLog_W(tag, "Client verbunden aber nicht subscribed - nichts gesendet");
            g_pending_send_error = true;
        } else {
            gxLog_W(tag, "Keine gueltigen Messwerte - nichts gesendet");
            g_pending_send_error = true;
        }
}

// ===========================================================================
// MAC-Adresse ausgeben
// ===========================================================================

// ------------------------------------------------------------
static void print_mac(void)
{
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_BT);   // BLE-MAC (wie im Advertising sichtbar)
    gxLog_I(tag, "MAC-Adresse: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ===========================================================================
// Setup
// ===========================================================================

void setup(void)
{
    g_start_ms = millis();   // Startzeit für Elapsed-Messung

    pinMode(pinout::DEEPSLEEP_DISABLE_PIN, INPUT_PULLDOWN);

    Serial.begin(SERIAL_BAUD);

    gxLog_setOptions( GXLOG_OPT_ITEM_COLOR | GXLOG_OPT_TIMESTAMP );

    gxLog_setLevel(tag, LOG_LVL_MAIN);
    gxLog_setColor(tag, ANSI_ESC_FG_BRIGHT_GREEN);

    // HTU21D No-Hold-Master sendet waehrend der Messung bewusst NACKs (lt. Datenblatt).
    // Der ESP32-IDF-Core loggt jeden NACK als ERROR -> hier unterdrücken.
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    // LED-Pin als Output + initial AUS
    pinMode(pinout::LED_PIN, OUTPUT);
    digitalWrite(pinout::LED_PIN, pinout::LED_OFF);

    // Sensor-Power-Pin als Output + initial AUS
    pinMode(pinout::SENSOR_POWER, OUTPUT);
    sensors_power(false);

    // ---------- DeepSleep-Disable-Check ----------
    g_deepsleep_disabled = check_deepsleep_disable();

    //---------------------------------------------
    //--- messungen durchführen, auswerten und deepsleep
    //---------------------------------------------
    if (!g_deepsleep_disabled) {

        // gxLog_I(tag, "Deep-Sleep-Modus aktiv (Dauer: %lu s, BLE-Timeout: %lu s)",(unsigned long)(DEEPSLEEP_DURATION_MS / 1000UL), (unsigned long)(BLE_CONNECT_TIMEOUT_MS / 1000UL));

        // BLE-Stack starten
        ble_init();

        env_result_t env;
        memset(&env, 0, sizeof(env_result_t));

        measure(&env);

        print_measurement(env);

        send_ble(env);

        // In den Deep-Sleep
        enter_deep_sleep();
        // ab hier wird setup() nicht weiter ausgefuehrt
    }

    //---------------------------------------------
    //--- kein deepsleep
    //---------------------------------------------
    // LED-Pin als Output
    led(true);
    delay(1500); //wegen Serial-Out
    gxLog_I(tag, "Deep-Sleep-Modus deaktiviert");
    // gxLog_I(tag, "ESP32-C6 Sensor-Node");
    gxLog_I(tag, "Device-Name: %s", BLE_DEVICE_NAME);
    print_mac();

    gxLog_I(tag, "Messung alle %lu s", (unsigned long)(NO_DEEPSLEEP_DURATION_MS / 1000UL));
}

// ===========================================================================
// Loop  (nur im No-DeepSleep-Modus aktiv)
// ===========================================================================

void loop(void)
{
    static uint32_t last_measure_ms = 0;
    static env_result_t env;
    const uint32_t now = millis();

    // Bei Bedarf messen + senden
    if (now - last_measure_ms >= NO_DEEPSLEEP_DURATION_MS) {
        last_measure_ms = now;
        led(false);

        measure(&env);
        print_measurement(env);

        led(true);
    }

    delay(50);
}
