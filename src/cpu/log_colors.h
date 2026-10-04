#pragma once

/**
 * @file log_colors.h
 * @brief ANSI-Farbcodes und farbige Log-Tags fuer die serielle Ausgabe
 *
 * ---------------------------------------------------------------------------
 * Erstellt:
 * ---------------------------------------------------------------------------
 * 2026-06-07 08:29 - Farbe wird auf die GANZE Zeile angewendet:
 *                    ANSI_RESET steht jetzt am ANFANG (alte Farbe schliessen)
 *                    und nicht mehr direkt nach dem Tag. Da jede Log-Zeile
 *                    mit einem Tag beginnt, faerbt sich der komplette Text
 *                    bis zum naechsten Tag oder Zeilenende ein.
 *
 * 2026-06-06 21:02 - Datei erstellt: farbige Tags, alle gleich lang (8 Zeichen)
 *
 * ---------------------------------------------------------------------------
 *
 * Alle Tags sind auf die Breite des laengsten Tags [BH1750] normiert:
 *   "[BH1750]" = 8 Zeichen inkl. Klammern
 *
 * Verwendung:
 *   Serial.println(TAG_MAIN "Hallo Welt");
 *   Serial.printf( TAG_BLE  "Verbunden mit %s\n", addr);
 */

// ---------------------------------------------------------------------------
// ANSI Reset
// ---------------------------------------------------------------------------

#define ANSI_RESET    "\033[0m"

// ---------------------------------------------------------------------------
// ANSI Farben (Vordergrund)
// ---------------------------------------------------------------------------

#define ANSI_RED      "\033[31m"
#define ANSI_GREEN    "\033[32m"
#define ANSI_YELLOW   "\033[33m"
#define ANSI_BLUE     "\033[34m"
#define ANSI_MAGENTA  "\033[35m"
#define ANSI_CYAN     "\033[36m"
#define ANSI_WHITE    "\033[37m"

#define ANSI_BRIGHT_RED     "\033[91m"
#define ANSI_BRIGHT_GREEN   "\033[92m"
#define ANSI_BRIGHT_YELLOW  "\033[93m"
#define ANSI_BRIGHT_BLUE    "\033[94m"
#define ANSI_BRIGHT_MAGENTA "\033[95m"
#define ANSI_BRIGHT_CYAN    "\033[96m"
#define ANSI_BRIGHT_WHITE   "\033[97m"

// ---------------------------------------------------------------------------
// Log-Tags  (alle 8 Zeichen breit inkl. Klammern)
//
// Aufbau:  ANSI_RESET + ANSI_FARBE + "[NAME  ] "
//   - ANSI_RESET am Anfang: schliesst die Farbe der vorherigen Zeile sauber.
//   - ANSI_FARBE direkt vor dem Tag: faerbt Tag + Rest der Zeile ein.
//   - KEIN Reset am Ende: damit die Farbe ueber die ganze Zeile gilt.
//
// Hinweis: IDF/HAL-eigene Logs (z. B. [E][...]) haben eigene Farbcodes -
// die ueberschreiben unsere Farbe automatisch.
// ---------------------------------------------------------------------------

/** Hauptprogramm          - Hellgruen  */
#define TAG_MAIN   ANSI_RESET ANSI_BRIGHT_GREEN   "[MAIN  ] "

/** BLE-Stack              - Hellblau   */
#define TAG_BLE    ANSI_RESET ANSI_BRIGHT_BLUE    "[BLE   ] "

/** I2C-Treiber            - Cyan       */
#define TAG_I2C    ANSI_RESET ANSI_CYAN           "[I2C   ] "

/** Akku-Ueberwachung      - Hellgelb   */
#define TAG_AKKU   ANSI_RESET ANSI_BRIGHT_YELLOW  "[AKKU  ] "

/** SHT21-Sensor           - Magenta    */
#define TAG_SHT21  ANSI_RESET ANSI_MAGENTA        "[SHT21 ] "

/** SHT3x-Sensor (Legacy)  - Hellmagenta */
#define TAG_SHT3X  ANSI_RESET ANSI_BRIGHT_MAGENTA "[SHT3X ] "

/** BH1750-Lichtsensor     - Hellcyan   */
#define TAG_BH1750 ANSI_RESET ANSI_BRIGHT_CYAN    "[BH1750] "
