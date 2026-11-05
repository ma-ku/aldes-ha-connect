/* Dekodierung des lokalen 0x21-Berichts (90 Bytes) — Port von report21.py,
 * report_clock.py und der Maske aus pico.py.
 *
 * Reine Logik ohne ESP-Abhaengigkeiten: derselbe Quelltext laeuft im Geraet und
 * im Host-Test (tests/run_esphome.sh). Der Test vergleicht jede Ausgabe mit den
 * Python-Referenzen, damit ein Port nicht stillschweigend abweicht.
 */

#include <stdio.h>
#include <string.h>

#include "aldes_report21.h"

/* pico.TEMPLATE: am 11. Januar 2026 aus einer echten ConnectBox gelesen
 * (88 Bytes, Summe 0). Felder 5-12 = Urlaub (hier 00 00 00 00 6C 07 00 00),
 * 13-20 = Sollwerte K1-K4 (FF), 21-52 = FF, 53-72 = Nutzlast (00, hier immer
 * maskiert), 73-80 = FF, 79/80 = Air/ECS in _blank auf FF, 86 = FE,
 * 87 = Summe. */
static const uint8_t k_template[ALDES_WRITE10_LEN] = {
    0xFD, 0xFA, 0x58, 0xFF, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x6C, 0x07, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x6C,
};

#define OFF_ZONE_SETPOINT 13u
#define OFF_ZONE_TEMP 53u
#define OFF_VACATION_DEPARTURE 5u
#define OFF_VACATION_RETURN 9u
#define OFF_CLOCK 77u
#define OFF_AIR 81u
#define OFF_ECS 82u
#define OFF_TRAILER 86u

#define SETPOINT_MIN 16.0f
#define SETPOINT_MAX 30.0f

/* ---------------------------------------------------------------- Tabellen */

/* pico.AIR_MODES — Text woertlich, damit Home Assistant dieselben Verteilungen
 * anzeigt wie die Python-Oberflaeche. */
static const char *const k_air_labels[ALDES_AIR_MODE_COUNT] = {
    "Arrêt",                  /* 0 */
    "Chauffage · Confort",    /* 1 */
    "Chauffage · Éco",        /* 2 */
    "Chauffage · Programme A",/* 3 */
    "Chauffage · Programme B",/* 4 */
    "Clim · Confort",         /* 5 */
    "Clim · Boost",           /* 6 */
    "Clim · Programme C",     /* 7 */
    "Clim · Programme D",     /* 8 */
};

static const char *const k_ecs_labels[ALDES_ECS_MODE_COUNT] = {
    "Arrêt",  /* 0 */
    "Marche", /* 1 */
    "Boost",  /* 2 */
};

#define UNKNOWN_LABEL "Mode non validé"

const char *aldes_air_label(unsigned mode) {
  return mode < ALDES_AIR_MODE_COUNT ? k_air_labels[mode] : UNKNOWN_LABEL;
}

const char *aldes_ecs_label(unsigned mode) {
  return mode < ALDES_ECS_MODE_COUNT ? k_ecs_labels[mode] : UNKNOWN_LABEL;
}

/* ---------------------------------------------------------------- Helfer */

static uint16_t le16(const uint8_t *p) { return (uint16_t) (p[0] | (p[1] << 8)); }

static uint32_t le32(const uint8_t *p) {
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

/* report21.decode_date — Format uebernommen, samt der Ausnahmen, die Python
 * wirft: datetime.datetime() lehnt Monat 0/13 und Tag 0 ab; Stunde und Minute
 * koennen 31/63 erreichen und werden von Python als ValueError abgelehnt. */
const char *aldes_decode_date(uint32_t value, char *buf, size_t buf_len) {
  const unsigned day_of_month = (value >> 16) & 31u;
  const unsigned month = (value >> 21) & 15u;
  const unsigned hour = (value >> 11) & 31u;
  const unsigned minute = (value >> 5) & 63u;

  if (!value) return NULL;
  if (month < 1 || month > 12 || day_of_month < 1 || hour > 23 || minute > 59) return "Date invalide";

  snprintf(buf, buf_len, "%04u-%02u-%02uT%02u:%02u:%02u", (unsigned) (1980u + (value >> 25)), month, day_of_month,
           hour, minute, (unsigned) ((value & 31u) * 2u));
  return buf;
}

/* ---------------------------------------------------------------- decode */

const char *aldes_decode21(const uint8_t *data, size_t len, aldes_report21_t *out) {
  memset(out, 0, sizeof(*out));

  if (data == NULL || len != ALDES_REPORT21_LEN) return "laenge";
  static const uint8_t prefix[5] = {0xFF, 0xFD, 0x5A, 0xFF, 0x21};
  if (memcmp(data, prefix, sizeof(prefix)) != 0) return "praefix";
  if (data[len - 2] != 0xFE) return "cadrage";

  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) sum = (uint8_t) (sum + data[i]);
  if (sum != 0) return "somme";

  out->present = true;

  for (unsigned i = 0; i < 4; i++) {
    aldes_zone21_t *zone = &out->zone[i];
    const uint16_t setpoint = le16(&data[OFF_ZONE_SETPOINT + 2 * i]);
    const uint16_t temperature = le16(&data[OFF_ZONE_TEMP + 2 * i]);
    zone->has_setpoint = setpoint != 0xFFFF;
    zone->setpoint_hundredths = (int16_t) setpoint;
    zone->has_temperature = temperature != 0xFFFF;
    zone->temperature_hundredths = (int16_t) temperature;
  }

  out->air.raw = data[OFF_AIR];
  out->air.label = out->air.raw == 0xFF ? UNKNOWN_LABEL : aldes_air_label(out->air.raw);
  out->ecs.raw = data[OFF_ECS];
  out->ecs.label = out->ecs.raw == 0xFF ? UNKNOWN_LABEL : aldes_ecs_label(out->ecs.raw);

  out->vacation_departure = le32(&data[OFF_VACATION_DEPARTURE]);
  out->vacation_return = le32(&data[OFF_VACATION_RETURN]);
  bool any = false;
  for (unsigned i = OFF_VACATION_DEPARTURE; i < OFF_ZONE_SETPOINT; i++) any = any || data[i] != 0;
  out->vacation_present = any;

  const uint32_t clock = le32(&data[OFF_CLOCK]);
  const unsigned month = (clock >> 21) & 15u;
  const unsigned day = (clock >> 16) & 31u;
  const unsigned hour = (clock >> 11) & 31u;
  const unsigned minute = (clock >> 5) & 63u;
  out->clock_year = (uint16_t) (1980u + (clock >> 25));
  out->clock_month = (uint8_t) month;
  out->clock_day = (uint8_t) day;
  out->clock_hour = (uint8_t) hour;
  out->clock_minute = (uint8_t) minute;
  out->clock_second = (uint8_t) ((clock & 31u) * 2u);
  out->clock_valid = month >= 1 && month <= 12 && day >= 1 && hour <= 23 && minute <= 59;

  return NULL;
}

/* ---------------------------------------------------------------- frames */

size_t aldes_frame_template(uint8_t out[ALDES_WRITE10_LEN]) {
  memcpy(out, k_template, ALDES_WRITE10_LEN);
  return ALDES_WRITE10_LEN;
}

/* facade[-1] = -sum(facade[:-1]) — pico._seal. */
static void seal(uint8_t frame[ALDES_WRITE10_LEN]) {
  unsigned sum = 0;
  for (unsigned i = 0; i + 1 < ALDES_WRITE10_LEN; i++) sum += frame[i];
  frame[ALDES_WRITE10_LEN - 1] = (uint8_t) ((0u - sum) & 0xFFu);
}

/* pico._blank: Sollwerte K1-K4, Air und ECS auf FF; der Rest des Templates
 * bleibt (Urlaub 5-12 und Nutzlast 53-72 sind schon maskiert). */
static void blank(uint8_t frame[ALDES_WRITE10_LEN]) {
  memcpy(frame, k_template, ALDES_WRITE10_LEN);
  for (unsigned i = OFF_ZONE_SETPOINT; i < OFF_ZONE_SETPOINT + 8; i++) frame[i] = 0xFF;
  frame[OFF_AIR] = 0xFF;
  frame[OFF_ECS] = 0xFF;
}

/* round(x) wie Python: kaufmaennisch, 0.5 von der Null weg. */
static long py_round(double value) {
  return value >= 0.0 ? (long) (value + 0.5) : -(long) (0.5 - value);
}

/* round(float(temperature) * 2) / 2 — pico.frame_setpoint. Temperaturen aus
 * Home Assistant sind Vielfache von 0.5 °C (siehe Zahl-Entitaeten); der Test
 * deckt genau dieses Raster plus die Grenzen ab. */
bool aldes_frame_setpoint(unsigned zone, float temperature, uint8_t out[ALDES_WRITE10_LEN]) {
  if (zone < 1 || zone > 4) return false;
  if (!(temperature >= SETPOINT_MIN && temperature <= SETPOINT_MAX)) return false; /* auch NaN */

  const long halves = py_round((double) temperature * 2.0);
  const long hundredths = py_round((double) halves * 50.0);
  if (hundredths < 1600 || hundredths > 3000) return false;

  blank(out);
  const unsigned off = OFF_ZONE_SETPOINT + 2 * (zone - 1);
  out[off] = (uint8_t) (hundredths & 0xFF);
  out[off + 1] = (uint8_t) ((hundredths >> 8) & 0xFF);
  seal(out);
  return true;
}

bool aldes_frame_air(unsigned mode, uint8_t out[ALDES_WRITE10_LEN]) {
  if (mode >= ALDES_AIR_MODE_COUNT) return false;
  blank(out);
  out[OFF_AIR] = (uint8_t) mode;
  seal(out);
  return true;
}

bool aldes_frame_ecs(unsigned mode, uint8_t out[ALDES_WRITE10_LEN]) {
  if (mode >= ALDES_ECS_MODE_COUNT) return false;
  blank(out);
  out[OFF_ECS] = (uint8_t) mode;
  seal(out);
  return true;
}

/* pico.frame_vacation_clear: 5-12 auf 00; _blank laesst 13-20 und 81/82 auf FF. */
bool aldes_frame_vacation_clear(uint8_t out[ALDES_WRITE10_LEN]) {
  blank(out);
  for (unsigned i = OFF_VACATION_DEPARTURE; i < OFF_ZONE_SETPOINT; i++) out[i] = 0x00;
  seal(out);
  return true;
}

uint32_t aldes_next_tx_id(uint32_t last) {
  uint32_t id = last + 1u;
  if (id == 0) id = 1; /* SEND 0 wird abgelehnt, die 0 nicht als ID verwenden */
  return id;
}
