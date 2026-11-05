/* Dekodierung des lokalen 0x21-Berichts (90 Bytes) — Port von report21.py,
 * report_clock.py und der Maske aus pico.py.
 *
 * Reine Logik ohne ESP-Abhängigkeiten: derselbe Quelltext läuft im Gerät und
 * im Host-Test (tests/aldes_host_tests.c). Der Test vergleicht jede Ausgabe
 * mit den Python-Referenzen, damit ein Port nicht stillschweigend abweicht.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALDES_REPORT21_LEN 90
#define ALDES_WRITE10_LEN 88
#define ALDES_TX_MAX 512

typedef struct {
  bool present;            /* 90 Bytes, Praefix FFFD5AFF21, 0xFE, Summe 0     */
  bool has_setpoint;       /* LE16 bei 13+2i != 0xFFFF                        */
  bool has_temperature;    /* LE16 bei 53+2i != 0xFFFF                        */
  int16_t setpoint_hundredths;     /* LE16 (ohne Vorzeichen), 1/100 °C        */
  int16_t temperature_hundredths;  /* signed LE16, 1/100 °C, Layout-Kandidat   */
} aldes_zone21_t;

typedef struct {
  bool present;
  uint8_t raw;             /* 0xFF = Mode unbekannt / nicht gesetzt           */
  const char *label;       /* AIR/ECS-Text wie report21.AIR/ECS                */
} aldes_mode21_t;

typedef struct {
  bool present;
  bool vacation_present;
  uint32_t vacation_departure;  /* DOS/FAT 32 Bit, little endian              */
  uint32_t vacation_return;
  aldes_zone21_t zone[4];
  aldes_mode21_t air;
  aldes_mode21_t ecs;
  bool clock_valid;
  uint16_t clock_year;     /* 1980+y (bis 2107, passt nicht in uint8_t)        */
  uint8_t clock_month;
  uint8_t clock_day;
  uint8_t clock_hour;
  uint8_t clock_minute;
  uint8_t clock_second;    /* immer gerade (Bit 0 des Feldes)                   */
} aldes_report21_t;

/* Liefert NULL wenn der Frame kein vollstaendiger gueltiger lokaler 0x21 ist
 * (gleiche Ablehnung wie decode21()). */
const char *aldes_decode21(const uint8_t *data, size_t len, aldes_report21_t *out);

/* report21.decode_date: "YYYY-MM-DDTHH:MM:SS", "Date invalide" oder NULL. */
const char *aldes_decode_date(uint32_t value, char *buf, size_t buf_len);

/* Labeltabellen, identisch zu pico.AIR_MODES / pico.ECS_MODES. */
const char *aldes_air_label(unsigned mode);
const char *aldes_ecs_label(unsigned mode);
#define ALDES_AIR_MODE_COUNT 9
#define ALDES_ECS_MODE_COUNT 3

/* Erzeugt die naechste Sendekennung aus der letzten: gleiche Feldreihenfolge
 * wie `int(s['last']) + 1` in pico.Workbench.transact. 0 bleibt unbenutzt. */
uint32_t aldes_next_tx_id(uint32_t last);

/* Baut die 88-Byte-Schreibframes aus pico.py nach (TEMPLATE, _blank, _seal).
 * Rueckgabe false bei Zone/Mode/Temperatur ausserhalb der geprueften Bereiche.
 * Die Ergebnisse muessen byte-genau frame_*() von pico.py entsprechen. */
bool aldes_frame_setpoint(unsigned zone, float temperature, uint8_t out[ALDES_WRITE10_LEN]);
bool aldes_frame_air(unsigned mode, uint8_t out[ALDES_WRITE10_LEN]);
bool aldes_frame_ecs(unsigned mode, uint8_t out[ALDES_WRITE10_LEN]);
bool aldes_frame_vacation_clear(uint8_t out[ALDES_WRITE10_LEN]);

/* pico.TEMPLATE (88 Bytes, Summe 0) und abschliessende Summenpruefung. */
size_t aldes_frame_template(uint8_t out[ALDES_WRITE10_LEN]);

#ifdef __cplusplus
}
#endif
