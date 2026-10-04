#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../settings.h"

/**
 * @file ble_device.h
 * @brief BLE-Kommunikationsmodul für den ESP32-C6 Sensor-Node
 *
 * Stellt einen BLE GATT-Server bereit, der Sensordaten als JSON-String
 * per Notify an verbundene Clients (Ubuntu-Server) sendet.
 *
 * Service UUID  : 4fafc201-1fb5-459e-8fcc-c5c9c331914b
 * Characteristic: beb5483e-36e1-4688-b7f5-ea07361b26a8
 */

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

/** Sendeleistung in dBm */
// #define BLE_DEVICE_TX_POWER  9

/** Maximale Länge des JSON-Payloads (inkl. Null-Terminator) */
// #define BLE_JSON_MAX_LEN    160
#define BLE_JSON_MAX_LEN    512

/** ACK-Byte das die Python-App nach Empfang einer Nachricht zurückschreibt */
#define BLE_ACK_BYTE        0x06    /* ASCII ACK */

// ---------------------------------------------------------------------------
// Öffentliche Funktionen
// ---------------------------------------------------------------------------

/**
 * @brief Initialisiert den NimBLE-Stack, GATT-Server und startet Advertising.
 *
 * Muss einmalig in setup() aufgerufen werden.
 * Blockiert NICHT - Advertising läuft im Hintergrund.
 */
void ble_init(void);

/**
 * @brief Sendet einen JSON-String an alle verbundenen und subscribed Clients.
 *
 * @param json  Null-terminierter JSON-String, max. BLE_JSON_MAX_LEN Zeichen.
 * @return true  Mindestens ein Client hat die Nachricht erhalten.
 * @return false Kein Client verbunden oder kein Notify-Abo aktiv.
 */
bool ble_send_json(const char *json);

/**
 * @brief Gibt an, ob mindestens ein BLE-Client verbunden ist.
 *
 * @return true  Verbindung aktiv.
 * @return false Kein Client verbunden.
 */
bool ble_is_connected(void);

/**
 * @brief Gibt an, ob mindestens ein verbundener Client Notify subscribed hat.
 *
 * Solange das nicht der Fall ist, gehen ble_send_json()-Aufrufe ins Leere
 * (Notify-Frames werden vom Stack verworfen wenn kein CCC-Subscription).
 *
 * @return true  Mindestens ein Client hat Notify aktiviert.
 * @return false Niemand subscribed.
 */
bool ble_is_subscribed(void);

/**
 * @brief Gibt die Anzahl aktuell verbundener Clients zurück.
 *
 * @return uint8_t Anzahl verbundener Clients (0-n).
 */
uint8_t ble_get_connection_count(void);

/**
 * @brief Wartet auf das ACK-Byte vom Peer (Write auf ACK-Characteristic).
 *
 * Der Peer (Python-App) bestätigt jeden empfangenen Notify durch einen
 * Write von BLE_ACK_BYTE auf die ACK-Characteristic. Das ist die einzige
 * zuverlässige Möglichkeit, Notify-Verluste auf Application-Layer zu
 * erkennen (BLE-Notify hat selbst kein End-to-End-ACK).
 *
 * @param timeout_ms  Maximale Wartezeit in Millisekunden.
 * @return true   ACK empfangen.
 * @return false  Timeout abgelaufen ohne ACK.
 */
bool ble_wait_for_ack(uint32_t timeout_ms);

/**
 * @brief Setzt den internen ACK-Zustand zurueck (vor neuem Send aufrufen).
 */
void ble_clear_ack(void);

/**
 * @brief Trennt alle aktiven BLE-Verbindungen und stoppt Advertising.
 *
 * Muss vor dem Eintritt in den Deep-Sleep aufgerufen werden.
 */
void ble_stop(void);
