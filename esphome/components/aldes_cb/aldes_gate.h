/* Klon der AldesConnect Box — Kontrollfilter.
 *
 * Uebersetzung von firmware/src/control_protocol.c und der Freigabelogik aus
 * pico.Pilot.send(): eine Freigabe (ARM) autorisiert genau einen Versuch
 * innerhalb von 30 s; die Kennung muss strikt groesser sein als die letzte
 * akzeptierte; der 45-s-Erinnerungsschutz (Beobachtungsfenster nach der
 * letzten bestaetigten Aenderung) sitzt im Baustein, weil dort die Uhr lebt.
 *
 * Reine Logik, im Host-Test mit denselben Vektoren wie firmware/tests/
 * test_protocol.c geprueft.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware-Konstante: ein Versuch, 30 s Fenster. */
#define ALDES_ARM_WINDOW_MS 30000u
/* Aus pico.Pilot(cooldown=45): erst nach 45 s ohne bestaetigte Aenderung
 * darf die naechste Aenderung starten (Beobachtungszeit des 0x21-Berichts). */
#define ALDES_COOLDOWN_MS 45000u
/* Aus pico.Workbench(timeout=90). */
#define ALDES_TX_TIMEOUT_MS 90000u

typedef struct {
  bool armed;
  uint32_t armed_at;
  uint32_t last_id;
} aldes_gate_t;

void aldes_gate_disarm(aldes_gate_t *gate);
void aldes_gate_arm(aldes_gate_t *gate, uint32_t now_ms);
/* Verbraucht die Freigabe immer; akzeptiert nur wenn scharf, im Fenster und
 * id > last_id. Entspricht gate_take() der Firmware. */
bool aldes_gate_take(aldes_gate_t *gate, uint32_t now_ms, uint32_t id);

/* Ruft int(s['last']) + 1 aus pico.Workbench, aber ohne die 0 als Kennung. */
uint32_t aldes_gate_next_id(const aldes_gate_t *gate);

/* 45-s-Sperre, gilt fuer ARM und SEND gleichermassen. */
uint32_t aldes_cooldown_remaining(uint32_t now_ms, uint32_t last_done_ms);

/* Ein Statusabzug, wie ihn der Baustein nach aussen gibt (Pico-STATUS-Zeilen
 * und pico.Workbench.state() benutzen dieselben Feldnamen). */
typedef struct {
  bool usb_initialized;
  bool mounted;
  bool armed;        /* Freigabe aktuell scharf (Fenster noch offen)      */
  bool tx_busy;
  uint32_t last_id;
  uint32_t tx_id;
  const char *tx_state;
} aldes_gate_status_t;

/* SEND-Befehl im Pico-Format "SEND <id> <hex>" (control_protocol.parse_send). */
bool aldes_parse_send(const char *line, uint32_t *id, uint8_t *out, uint16_t *len, uint16_t capacity);
/* Hexdekodierung fuer Rohbefehle ("FDFA..." ohne SEND), z. B. aus HA-Lambda. */
bool aldes_parse_hex(const char *text, uint8_t *out, uint16_t *len, uint16_t capacity);
/* Binarframe in Hex umwandeln ("FDFA..." gross, wie frame.hex() in Python). */
void aldes_hex_encode(const uint8_t *data, uint16_t len, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
