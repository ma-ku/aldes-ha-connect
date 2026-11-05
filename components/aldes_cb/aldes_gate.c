/* Klon der AldesConnect Box — Kontrollfilter (Logik; siehe aldes_gate.h). */

#include <string.h>

#include "aldes_gate.h"

#include "aldes_report21.h"

void aldes_gate_disarm(aldes_gate_t *gate) { gate->armed = false; }

void aldes_gate_arm(aldes_gate_t *gate, uint32_t now_ms) {
  gate->armed = true;
  gate->armed_at = now_ms;
}

bool aldes_gate_take(aldes_gate_t *gate, uint32_t now_ms, uint32_t id) {
  const bool ok = gate->armed && (uint32_t) (now_ms - gate->armed_at) < ALDES_ARM_WINDOW_MS && id > gate->last_id;
  gate->armed = false;
  if (ok) gate->last_id = id;
  return ok;
}

uint32_t aldes_gate_next_id(const aldes_gate_t *gate) {
  const uint32_t id = gate->last_id + 1u;
  return id == 0 ? 1u : id;
}

uint32_t aldes_cooldown_remaining(uint32_t now_ms, uint32_t last_done_ms) {
  const uint32_t elapsed = now_ms - last_done_ms;
  return elapsed >= ALDES_COOLDOWN_MS ? 0u : ALDES_COOLDOWN_MS - elapsed;
}

/* ------------------------------------------------------------ SEND-Parser */

static int nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool aldes_parse_hex(const char *text, uint8_t *out, uint16_t *len, uint16_t capacity) {
  const size_t size = strlen(text);
  if (!size || (size % 2) != 0 || size / 2 > capacity) return false;
  for (size_t i = 0; i < size; i += 2) {
    const int hi = nibble(text[i]);
    const int lo = nibble(text[i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i / 2] = (uint8_t) (hi * 16 + lo);
  }
  *len = (uint16_t) (size / 2);
  return true;
}

/* Streng wie control_protocol.parse_send: "SEND " + Dezimalzahl (ohne
 * Vorzeichen, ueberlaufsicher, != 0) + " " + gerade Anzahl Hexziffern. Der
 * Pico begrenzt auf TX_MAX*2 Ziffern; wir zusaetzlich auf `capacity`. */
bool aldes_parse_send(const char *line, uint32_t *id, uint8_t *out, uint16_t *len, uint16_t capacity) {
  if (strncmp(line, "SEND ", 5) != 0) return false;
  const char *p = line + 5;

  uint32_t n = 0;
  unsigned digits = 0;
  while (*p >= '0' && *p <= '9') {
    const unsigned v = (unsigned) (*p++ - '0');
    if (n > (UINT32_MAX - v) / 10u) return false;
    n = n * 10u + v;
    digits++;
  }
  if (!digits || !n || *p++ != ' ') return false;

  const size_t size = strlen(p);
  if (!size || (size % 2) != 0 || size > (size_t) ALDES_TX_MAX * 2u || size / 2 > capacity) return false;
  for (size_t i = 0; i < size; i += 2) {
    const int hi = nibble(p[i]);
    const int lo = nibble(p[i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i / 2] = (uint8_t) (hi * 16 + lo);
  }
  *len = (uint16_t) (size / 2);
  *id = n;
  return true;
}

void aldes_hex_encode(const uint8_t *data, uint16_t len, char *out, size_t out_len) {
  static const char k_hex[] = "0123456789ABCDEF";
  size_t o = 0;
  for (uint16_t i = 0; i < len && o + 3 <= out_len; i++) {
    out[o++] = k_hex[data[i] >> 4];
    out[o++] = k_hex[data[i] & 0x0F];
  }
  if (out_len) out[o < out_len ? o : out_len - 1] = '\0';
}
