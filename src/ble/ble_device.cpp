/**
 * @file ble_device.cpp
 * @brief NimBLE GATT-Server Implementierung für den ESP32-C6 Sensor-Node
 *
 * ---------------------------------------------------------------------------
 * Änderungen:
 *
 * 2026-06-27 16:16 - TIMING-Marker in onConnect/onSubscribe wieder entfernt
 *                    (waren temporaer zur Connect-Latenz-Diagnose).
 *
 * 2026-06-26 20:52 - Explizites schnelles Advertising-Intervall (20-30 ms) in
 *                    ble_init() gesetzt (setMinInterval/setMaxInterval). Ohne
 *                    explizites Intervall nutzt NimBLE einen langsameren Default
 *                    -> Empfaenger findet den ESP nach dem Aufwachen spaeter,
 *                    was die elapsed_ms (Connect-Latenz) hochtreibt.
 *
 * 2026-06-25 19:00 - s_subscribed_count++ / -- ersetzt durch
 *                    s_subscribed_count = s_subscribed_count + 1 / - 1:
 *                    C++20 deprecates ++/-- auf volatile-Typen (Warning -Wvolatile).
 *
 * 2026-06-07 20:06 - TX-Leistung von 3 dBm auf 9 dBm erhoeht - Verbindung
 *                    war zu schwach. ESP32-C6 unterstuetzt bis +21 dBm,
 *                    9 dBm ist ein vernuenftiger Wert mit guter Reichweite
 *                    bei moderatem Stromverbrauch waehrend des Sendens.
 *
 * 2026-06-07 10:48 - Application-Layer-ACK eingefuehrt: zweite Characteristic
 *                    (ACK_CHAR_UUID, Write+Read) mit eigenem Callback. Beim
 *                    Empfang von BLE_ACK_BYTE wird s_ack_received gesetzt.
 *                    Neue Funktionen: ble_clear_ack(), ble_wait_for_ack().
 *                    Damit kann main.cpp jede Notify retransmitten bis das
 *                    Application-ACK ankommt - schliesst die 25%-Verlustluecke.
 *
 * 2026-06-07 08:29 - ble_is_subscribed() ergaenzt; Subscribe-Zaehler
 *                    s_subscribed_count wird in onSubscribe() gepflegt,
 *                    in onDisconnect() bei Verbindungsverlust auf 0
 *                    zurueckgesetzt. Loest das Kernproblem "Daten kommen
 *                    nicht beim Empfaenger an": ESP wartete bisher nur auf
 *                    Connect, sendete dann Notify - der Client war aber
 *                    noch nicht subscribed -> Notify-Frame verworfen.
 *
 * 2026-06-07 08:15 - ble_stop() sauberer Shutdown: erst disconnect() mit
 *                    Reason 0x13 (REMOTE_USER_TERM_CONN), dann warten bis
 *                    Verbindung wirklich getrennt ist, ERST DANN deinit().
 *                    Behebt "ble_hs_stop: failed to terminate connection rc=2"
 *                    und damit den nachfolgenden BlueZ-Fehler GATT_UNLIKELY.
 *
 * 2026-06-06 21:03 - Farbige Log-Tags (log_colors.h) eingebunden
 *
 * ---------------------------------------------------------------------------
 * 2026-06-06 20:28 - MAC-Adresse wird nach ble_init() auf Serial ausgegeben
 *
 * Verwendet das Arduino-NimBLE Framework.
 * Der Server exponiert einen Custom-Service mit einer Notify-Charakteristik.
 * Sensordaten werden als JSON-String gepusht.
 */

#include "ble_device.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLECharacteristic.h>
#include <NimBLEAdvertising.h>
#include "../cpu/log_colors.h"
#include "../services/gxLog.h"

// ---------------------------------------------------------------------------
static constexpr char tag[] = "ble";

// ---------------------------------------------------------------------------
// UUIDs  (zufällig generiert - projektspezifisch)
// ---------------------------------------------------------------------------

/** UUID des primären Sensor-Service */
static const char *SERVICE_UUID        = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";

/** UUID der Sensor-Daten-Charakteristik (Notify + Read) */
static const char *SENSOR_CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

/** UUID der ACK-Charakteristik (Write + Read) - Peer schreibt BLE_ACK_BYTE */
static const char *ACK_CHAR_UUID       = "beb5483e-36e1-4688-b7f5-ea07361b26a9";

// ---------------------------------------------------------------------------
// Modul-interne Variablen
// ---------------------------------------------------------------------------

static NimBLEServer         *s_pServer          = nullptr;
static NimBLECharacteristic *s_pSensorChar      = nullptr;
static NimBLECharacteristic *s_pAckChar         = nullptr;
static bool                  s_initialized      = false;
static volatile uint8_t      s_subscribed_count = 0;
static volatile bool         s_ack_received     = false;

// ---------------------------------------------------------------------------
// Callback: Server-Ereignisse (Connect / Disconnect)
// ---------------------------------------------------------------------------

class SensorServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
    {
        gxLog_I(tag, "Client verbunden  | Handle: %d | Adresse: %s",
                      connInfo.getConnHandle(),
                      connInfo.getAddress().toString().c_str());

        // Verbindungsparameter optimieren: kürzeres Intervall für niedrige Latenz
        pServer->updateConnParams(connInfo.getConnHandle(),
                                  /*minInterval=*/ 6,   // 7.5 ms
                                  /*maxInterval=*/ 12,  // 15 ms
                                  /*latency=*/     0,
                                  /*timeout=*/     200);

        // Advertising weiter aktiv lassen → mehrere Clients möglich
        NimBLEDevice::startAdvertising();
    }

    void onDisconnect(NimBLEServer *pServer,
                      NimBLEConnInfo &connInfo,
                      int reason) override
    {
        gxLog_I(tag, "Client getrennt   | Handle: %d | Grund: 0x%02X",
                      connInfo.getConnHandle(), reason);

        // Subscribe-Status zuruecksetzen wenn keine Clients mehr verbunden sind
        if (pServer->getConnectedCount() == 0) {
            s_subscribed_count = 0;
        }

        // Advertising neu starten, damit neue Clients finden können
        NimBLEDevice::startAdvertising();
        gxLog_I(tag, "Advertising neu gestartet");
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
    {
        gxLog_D(tag, "MTU geändert      | Handle: %d | MTU: %d", connInfo.getConnHandle(), MTU);
    }
};

// ---------------------------------------------------------------------------
// Callback: Charakteristik-Ereignisse (Subscribe / Unsubscribe)
// ---------------------------------------------------------------------------

class SensorCharCallbacks : public NimBLECharacteristicCallbacks
{
    void onSubscribe(NimBLECharacteristic *pChar,
                     NimBLEConnInfo       &connInfo,
                     uint16_t              subValue) override
    {
        if (subValue == 1) {
            if (s_subscribed_count < 255) s_subscribed_count = s_subscribed_count + 1;
            gxLog_I(tag, "Client subscribed (Notify)  | Handle: %d", connInfo.getConnHandle());
        } else if (subValue == 0) {
            if (s_subscribed_count > 0) s_subscribed_count = s_subscribed_count - 1;
            gxLog_I(tag, "Client unsubscribed         | Handle: %d", connInfo.getConnHandle());
        }
    }
};


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Callback: ACK-Characteristic (Peer schreibt BLE_ACK_BYTE nach Empfang)
// ---------------------------------------------------------------------------

class AckCharCallbacks : public NimBLECharacteristicCallbacks
{
    // ------------------------------------------------------------
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
    {
        NimBLEAttValue val = pChar->getValue();
        if (val.length() >= 1 && val.data()[0] == BLE_ACK_BYTE) {
            s_ack_received = true;
            gxLog_D(tag, "ACK empfangen | Handle: %d", connInfo.getConnHandle());
        } else {
            gxLog_W(tag, "Unbekanntes Byte auf ACK-Char (len=%u, byte0=0x%02X)",
                    (unsigned) val.length(), val.length() ? val.data()[0] : 0);
        }
    }
};

// ---------------------------------------------------------------------------
// Öffentliche Funktionen
// ---------------------------------------------------------------------------

void ble_init(void)
{
    gxLog_setLevel(tag, LOG_LVL_BLE);
    gxLog_setColor(tag, ANSI_ESC_FG_BRIGHT_BLUE);

    if (s_initialized) {
        gxLog_E(tag, "Bereits initialisiert - überspringe");
        return;
    }

    gxLog_I(tag, "Initialisierung gestartet …");

    // --- NimBLE-Stack initialisieren ---
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(BLE_DEVICE_TX_POWER); // ESP32-C6 koennte bis +21 dBm

    // --- GATT-Server anlegen ---
    s_pServer = NimBLEDevice::createServer();
    s_pServer->setCallbacks(new SensorServerCallbacks());

    // --- Service anlegen ---
    NimBLEService *pService = s_pServer->createService(SERVICE_UUID);

    // --- Charakteristik: Notify + Read ---
    s_pSensorChar = pService->createCharacteristic(
        SENSOR_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );
    s_pSensorChar->setCallbacks(new SensorCharCallbacks());

    // Initialen Platzhalterwert setzen
    s_pSensorChar->setValue("{}");

    // --- ACK-Charakteristik: Write + Read ---
    // Peer (Python) schreibt nach Empfang einer Nachricht BLE_ACK_BYTE darauf.
    s_pAckChar = pService->createCharacteristic(
        ACK_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::READ
    );
    s_pAckChar->setCallbacks(new AckCharCallbacks());
    s_pAckChar->setValue((uint8_t)0x00);

    // --- GATT-Server starten (startet implizit alle Services) ---
    s_pServer->start();

    // --- Advertising konfigurieren ---
    //
    // Hauptpaket (31 Bytes max) → Flags + vollständiger Gerätename
    //   ▸ Linux/BlueZ liest bei passiven Scans nur das Hauptpaket. Damit der
    //     Name garantiert auch dort sichtbar ist, gehört er hier rein.
    //
    // Scan-Response-Paket → 128-bit Service-UUID (16 Bytes + Header)
    //   ▸ Wird nur bei aktiven Scans angefragt - das macht bleak/nrf connect.
    //
    // Flags 0x06 = LE General Discoverable | BR/EDR not supported
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();

    NimBLEAdvertisementData advData;
    advData.setFlags(0x06);
    advData.setName(BLE_DEVICE_NAME);
    pAdvertising->setAdvertisementData(advData);

    NimBLEAdvertisementData scanData;
    scanData.setCompleteServices(NimBLEUUID(SERVICE_UUID));
    pAdvertising->setScanResponseData(scanData);

    // Schnelles Advertising-Intervall: Einheit 0.625 ms.
    // 0x20 = 20 ms, 0x30 = 30 ms -> Empfaenger findet den ESP nach dem
    // Aufwachen sehr schnell, minimiert die Connect-Latenz (elapsed_ms).
    pAdvertising->setMinInterval(0x20);   // 20 ms
    pAdvertising->setMaxInterval(0x30);   // 30 ms

    NimBLEDevice::startAdvertising();

    s_initialized = true;

    gxLog_I(tag, "Initialisierung abgeschlossen");
    gxLog_I(tag, "Gerätename  : %s",  BLE_DEVICE_NAME);
    gxLog_I(tag, "MAC-Adresse : %s",  NimBLEDevice::getAddress().toString().c_str());
    gxLog_I(tag, "Service UUID: %s",  SERVICE_UUID);
    gxLog_I(tag, "Char UUID   : %s",  SENSOR_CHAR_UUID);
    gxLog_I(tag, "ACK  UUID   : %s",  ACK_CHAR_UUID);
    gxLog_I(tag, "Advertising aktiv - warte auf Clients …");
}

// ---------------------------------------------------------------------------

bool ble_send_json(const char *json)
{
    if (!s_initialized || s_pSensorChar == nullptr) {
        gxLog_E(tag, "ble_send_json() vor ble_init() aufgerufen");
        return false;
    }

    if (json == nullptr) {
        gxLog_E(tag, "json-Zeiger ist NULL");
        return false;
    }

    size_t len = strlen(json);
    if (len == 0 || len >= BLE_JSON_MAX_LEN) {
        gxLog_E(tag, "ungültige JSON-Länge (%zu Zeichen)", len);
        return false;
    }

    // Wert setzen und Notify an alle subscribed Clients senden
    s_pSensorChar->setValue(reinterpret_cast<const uint8_t *>(json), len);
    s_pSensorChar->notify();

    uint8_t count = ble_get_connection_count();
    gxLog_I(tag, "Gesendet (%d Client(s)): %s", count, json);

    return (count > 0);
}

// ---------------------------------------------------------------------------

void ble_clear_ack(void)
{
    s_ack_received = false;
}

// ---------------------------------------------------------------------------

bool ble_wait_for_ack(uint32_t timeout_ms)
{
    const uint32_t t0 = millis();
    while (!s_ack_received && (millis() - t0) < timeout_ms) {
        delay(5);
    }
    return s_ack_received;
}

// ---------------------------------------------------------------------------

bool ble_is_connected(void)
{
    if (!s_initialized || s_pServer == nullptr) return false;
    return (s_pServer->getConnectedCount() > 0);
}

// ---------------------------------------------------------------------------

bool ble_is_subscribed(void)
{
    if (!s_initialized) return false;
    return (s_subscribed_count > 0);
}

// ---------------------------------------------------------------------------

uint8_t ble_get_connection_count(void)
{
    if (!s_initialized || s_pServer == nullptr) return 0;
    return static_cast<uint8_t>(s_pServer->getConnectedCount());
}

// ---------------------------------------------------------------------------

void ble_stop(void)
{
    if (!s_initialized) return;

    gxLog_I(tag, "Stoppe BLE …");

    // 1) Advertising stoppen
    NimBLEDevice::stopAdvertising();

    // 2) Alle Verbindungen aktiv trennen MIT Reason-Code, damit der Peer
    //    (BlueZ/bleak) eine saubere Disconnect-PDU erhält. Reason 0x13 =
    //    REMOTE_USER_TERMINATED_CONNECTION - das ist der Standard-Code wenn
    //    eine Seite die Verbindung absichtlich beendet (vs. Link-Loss).
    if (s_pServer != nullptr) {
        const auto handles = s_pServer->getPeerDevices();
        for (const uint16_t connHandle : handles) {
            s_pServer->disconnect(connHandle, 0x13);
        }

        // 3) Warten, bis Stack die Disconnect-Events verarbeitet hat
        //    (max. 500 ms - in der Praxis < 100 ms).
        const uint32_t t0 = millis();
        while (s_pServer->getConnectedCount() > 0 && millis() - t0 < 500UL) {
            delay(10);
        }
    }

    // 4) Kurzer zusätzlicher Puffer, damit ausgehende HCI-Pakete
    //    (Disconnect-Complete) den Controller verlassen können.
    delay(100);

    // 5) Erst jetzt Stack herunterfahren - kein "failed to terminate" mehr
    NimBLEDevice::deinit(true);

    s_initialized      = false;
    s_pServer          = nullptr;
    s_pSensorChar      = nullptr;
    s_pAckChar         = nullptr;
    s_subscribed_count = 0;
    s_ack_received     = false;

    gxLog_I(tag, "BLE gestoppt");
}
