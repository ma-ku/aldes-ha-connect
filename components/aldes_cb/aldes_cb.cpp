/* Klon der AldesConnect Box — Klassentreiber, Framer, Dekodierung und
 * Sende-/Freigabelogik.
 *
 * Port von firmware/src/cb_cdc_driver.c. Bewusste Abweichungen, alle durch den
 * Transport begruendet (ESP32-S31 = High-Speed-DWC2 statt Full-Speed-Sie805):
 *
 *  1. Kein eingebauter CDC-Treiber (CFG_TUD_CDC=0); dieser Treiber kommt ueber
 *     usbd_app_driver_get_cb(), damit GET_LINE_CODING mit 00 x7 beantwortet
 *     und JEDE Kontrollanfrage protokolliert wird.
 *  2. Auf HS liefert ein 64-Byte-ep ein Bulk-OUT-Paket nicht mehr als
 *     "kurzes Paket" ab; der Framer schliesst erst bei naechster Uebertragung
 *     oder nach INACTIVITY_MS Stille. Der Text "FRAME tronquee" bleibt.
 *  3. Der RX-Puffer ist wegen des DMA-Modus das USB-RAM; zwischen EPAuslesen
 *     und Neuarmierung gibt es daher ein kurzes Fenster. Der Zaehler
 *     rx_rearm_failed (Pico 0) bleibt sichtbar, falls es je zuschlaegt.
 */

#include "aldes_cb.h"

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tinyusb.h"
#include "tusb.h"

#include "device/usbd_pvt.h"

#include "aldes_desc.h"

/* Die CDC-Konstanten kommen aus TinyUSBs class/cdc/cdc.h, das bei
 * CFG_TUD_CDC=0 nicht eingebunden wird. Werte aus USB CDC 1.2 / PSTN 1.2 —
 * dieselben, die die Box auf dem Draht zeigt. */
#ifndef CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL
#define CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL 0x02
#endif
#ifndef CDC_REQUEST_SET_LINE_CODING
#define CDC_REQUEST_SET_LINE_CODING 0x20
#endif
#ifndef CDC_REQUEST_GET_LINE_CODING
#define CDC_REQUEST_GET_LINE_CODING 0x21
#endif
#ifndef CDC_REQUEST_SET_CONTROL_LINE_STATE
#define CDC_REQUEST_SET_CONTROL_LINE_STATE 0x22
#endif
#ifndef CDC_REQUEST_SEND_BREAK
#define CDC_REQUEST_SEND_BREAK 0x23
#endif

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#ifdef USE_API_USER_DEFINED_ACTIONS
#include "esphome/components/api/custom_api_device.h"
#endif

namespace esphome::aldes_cb {

extern "C" esp_err_t aldes_usb_install(void);
extern "C" esp_err_t aldes_usb_uninstall(void);
extern "C" const char *aldes_variant_name(void);

/* TinyUSB 0.21 (Espressif-Fork) kennt den ISR-Parameter; die Calls aus dem
 * pm-Callback laufen im TinyUSB-Task, nicht im Interrupt. */
constexpr bool IS_ISR = false;

namespace {

static const char *const TAG = "aldes_cb";

/* cb_tx.h: 90 s bis zum Abbruch des einen Versuchs. */
constexpr uint32_t CB_TX_TIMEOUT_MS = ALDES_TX_TIMEOUT_MS;
constexpr uint32_t TX_MAX = ALDES_TX_MAX;
/* Stille zwischen zwei OUT-Uebertragungen, danach gilt eine offene Teil-Trame
 * als abgebrochen. Der PAC sendet zusammenhaengend; 20 ms liegt weit ueber
 * einem SOF (1 ms) und weit unter der Berichtskadenz (~20 s). */
constexpr uint32_t INACTIVITY_MS = 20;
constexpr size_t FRAME_MAX = 300;

/* Variable der ersten Pico-Experimentreihe: Antwort auf GET_LINE_CODING.
 *   1 -> 00 00 00 00 00 00 00  (was die echte ConnectBox liefert)
 *   0 -> 00 C2 01 00 00 00 08  (115200 8N1, TinyUSB-Default = Kontrollzweig) */
#ifndef CB_LINE_CODING_ZERO
#define CB_LINE_CODING_ZERO 1
#endif

#if CB_LINE_CODING_ZERO
uint8_t s_line_coding[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const char *const s_variant = "B (GET_LINE_CODING = 00 x7, comme la vraie CB)";
#else
uint8_t s_line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};
const char *const s_variant = "A (GET_LINE_CODING = 115200 8N1, temoin TinyUSB)";
#endif

uint8_t s_set_lc_buf[8];

uint8_t s_ep_out = 0;
uint8_t s_ep_in = 0;
uint8_t s_ep_notif = 0;

/* DMA-faehiger Puffer: im HS/DMA-Modus muss das Ziel im USB-RAM liegen. */
CFG_TUSB_MEM_SECTION uint8_t s_out_buf[64] CFG_TUSB_MEM_ALIGN;

/* Nur vom Main-Loop befuellt (tx_start), nie aus TCP/IRQ-Callbacks. */
CFG_TUSB_MEM_SECTION uint8_t tx_buf[TX_MAX] CFG_TUSB_MEM_ALIGN;
bool tx_busy = false;
bool tx_zlp = false;
uint32_t tx_started = 0;
uint32_t tx_id = 0;
uint16_t tx_len = 0;
const char *tx_state = "IDLE";

/* Diagnose, nur Zaehler und Logzeilen — keine Endpunkt-Schreibzugriffe. */
uint32_t lc_diag_queued = 0, lc_diag_failed = 0, lc_diag_data = 0, lc_diag_ack = 0, lc_diag_report_ms = 0;
uint32_t diag_loops = 0, diag_usb_max_us = 0, diag_control_max_us = 0;
uint32_t diag_last_report = 0, diag_last_loop = 0, diag_loop_gap_max_us = 0;

/* Zaehler — periodisch in der STATUS-Zeile veroeffentlicht (main.c). */
uint32_t cb_n_get_line_coding = 0;
uint32_t cb_n_set_line_coding = 0;
uint32_t cb_n_set_control_line_state = 0;
uint32_t cb_n_send_break = 0;
uint32_t cb_n_ctrl_other = 0;
uint32_t cb_n_out_xfer = 0;
uint32_t cb_n_out_bytes = 0;
uint32_t cb_n_out_errors = 0;
uint32_t cb_n_out_zlp = 0;
uint32_t cb_n_out_rearm_failed = 0;
uint32_t cb_n_frames = 0;
uint32_t cb_n_frames_phase = 0; /* 0x22 gueltig: Phasenwechsel belegt      */
uint32_t cb_n_frames_live = 0;  /* 0x21 gueltig mit lebenden Werten        */
uint32_t cb_n_frames_bad = 0;   /* Summe oder Rahmung ungueltig            */
uint32_t cb_n_truncated = 0;

/* ---------------------------------------------------------------- Zustand */

/* Alles, was TinyUSB-Task und Main-Loop teilen, steht hier (Name `Internals`
 * ist der Freund von AldesCB). Geschuetzt wird es vom rekursiven Mutex. */
struct Shared {
  SemaphoreHandle_t lock{nullptr};
  AldesCB *owner{nullptr};

  /* Freigabe/Sperre (Main-Loop, Mutex) */
  aldes_gate_t gate{};
  bool have_done = false;
  uint32_t last_done_ms = 0;

  /* Empfangsseite (TinyUSB-Task -> Main-Loop) */
  bool report_pending = false;
  aldes_report21_t report{};
  aldes_report21_t last_good{};
  uint32_t report_ms = 0;
  uint32_t last_good_ms = 0;
  std::string frame_line;
  bool frame_event_pending = false;
  std::string frame_event;

  /* Framer-Zeitstempel fuer die HS-Inaktivenregel */
  uint32_t last_rx_ms = 0;
};

struct Internals : Shared {};

Internals s_shared;

inline void lock() {
  if (s_shared.lock != nullptr) xSemaphoreTakeRecursive(s_shared.lock, portMAX_DELAY);
}
inline void unlock() {
  if (s_shared.lock != nullptr) xSemaphoreGiveRecursive(s_shared.lock);
}

uint32_t now_ms() { return (uint32_t) (esp_timer_get_time() / 1000); }

void log_context(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_context(const char *fmt, ...) {
  char line[480];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  aldes_log_printf("%s", line);
  if (s_shared.owner != nullptr) s_shared.owner->log_sink()->publish_state(std::string(line));
}

/* Pico-Format der Rohhex-Zeilen: prefix + " +NNN:" + 32 Bytes je Zeile. */
void log_hex(const char *prefix, const uint8_t *data, uint32_t len) {
  char line[480];
  uint32_t i = 0;
  while (i < len) {
    uint32_t chunk = len - i;
    if (chunk > 32) chunk = 32;
    int n = snprintf(line, sizeof(line), "%s +%03lu:", prefix, (unsigned long) i);
    for (uint32_t j = 0; j < chunk && n > 0 && (uint32_t) n < sizeof(line) - 4; j++) {
      n += snprintf(line + n, sizeof(line) - (uint32_t) n, " %02X", data[i + j]);
    }
    log_context("%s", line);
    i += chunk;
  }
}

void tx_diag(const char *event) {
  log_context(
      "TXDIAG %s id=%lu ep=%u pending=%u age_ms=%lu buf=%08lx loops=%lu usb_max_us=%lu ctl_max_us=%lu gap_max_us=%lu",
      event, (unsigned long) tx_id, s_ep_in, tx_busy ? 1u : 0u, (unsigned long) (now_ms() - tx_started), 0ul,
      (unsigned long) diag_loops, (unsigned long) diag_usb_max_us, (unsigned long) diag_control_max_us,
      (unsigned long) diag_loop_gap_max_us);
}

/* --------------------------------------------------- Frame-Reassemblierung */
/* Rahmung: FA FD [LEN] FF [Opcode] ... FE [Summe], LEN = Gesamtlaenge
 * (0x29=41 fuer Hello, 0x70=112, 0xAF=175). */

uint8_t s_frame[FRAME_MAX];
uint16_t s_frame_idx = 0;
uint16_t s_frame_len = 0;
uint8_t s_frame_state = 0;

void frame_reset() {
  s_frame_idx = 0;
  s_frame_len = 0;
  s_frame_state = 0;
}

/* Pruefsumme. Der Ordner nannte eine CRC-8 DARC (Init 0xB8, reflektiert
 * 0x9C, xor 0xFF ueber frame[4:-1]); sie passt auf den 41-Byte-Hello, liefert
 * aber 0xDA statt 0x17 auf der oeffentlichen 112-Byte-Trame. Die 8-Bit-Summe
 * passt auf beide: das letzte Byte ist das Zweierkomplement der Summe der
 * vorherigen, also Summe(Trame) == 0 mod 256. Beide werden je Trame
 * protokolliert, damit jeder Lauf die Frage kostenlos entscheidet. */
uint8_t crc8_darc(const uint8_t *d, uint16_t n) {
  uint8_t crc = 0xB8;
  for (uint16_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 1) ? (uint8_t) ((crc >> 1) ^ 0x9C) : (uint8_t) (crc >> 1);
  }
  return crc ^ 0xFF;
}

void frame_emit() {
  const uint8_t opcode = (s_frame_len >= 5) ? s_frame[4] : 0xFF;

  uint8_t sum = 0;
  for (uint16_t i = 0; i < s_frame_len; i++) sum = (uint8_t) (sum + s_frame[i]);
  const bool sum_ok = (sum == 0);
  const bool framing_ok = (s_frame[3] == 0xFF) && (s_frame[s_frame_len - 2] == 0xFE);
  const uint8_t crc = crc8_darc(&s_frame[4], (uint16_t) (s_frame_len - 5));
  const bool crc_ok = (crc == s_frame[s_frame_len - 1]);
  const bool valid = sum_ok && framing_ok;

  /* Zwei getrennte Orakel, nicht eins.
   *
   * Die oeffentliche 112-Byte-0x22, als LE16 gelesen, ergibt 16,00 / 24,00 /
   * 22,00 / 31,00 Grad und dann Timings und Schwellen: das sind SOLLWERT-
   * GRENZEN, keine Messungen. Ein 0x22 belegt also einen Phasenwechsel, keine
   * Live-Anzeige.
   *
   * Die lebenden Werte stehen im 0x21: Sollwerte K1-K4 bei D0 07 und Messungen
   * BD 07 D0 07 E9 07 46 08, also LE16 zwischen 1000 und 3500 (10,00 bis
   * 35,00 Grad).
   *
   * Der Nullkopf der oeffentlichen T.One wird NICHT verlangt: der versionierte
   * Unterdatensatz FF 21 25 81 08 des Status steht in einer 0x25 und kommt
   * hier nie als eigene Trame an; ihn zu verlangen schuetzte vor nichts und
   * verfaelschte einen echten 0x21 im versionierten Dialekt. Der Kopf wird
   * protokolliert, nicht gefiltert. */
  const bool phase = valid && (opcode == 0x22);

  bool live = false;
  uint8_t plausibles = 0;
  if (valid && opcode == 0x21 && s_frame_len >= 24) {
    for (uint16_t i = 5; i + 1 < (uint16_t) (s_frame_len - 2); i += 2) {
      const uint16_t v = (uint16_t) (s_frame[i] | (s_frame[i + 1] << 8));
      if (v >= 1000 && v <= 3500) plausibles++;
    }
    live = (plausibles >= 4);
  }

  cb_n_frames++;
  if (!valid) cb_n_frames_bad++;
  if (phase) cb_n_frames_phase++;
  if (live) cb_n_frames_live++;

  char line[400];
  snprintf(line, sizeof(line), "FRAME #%lu opcode=0x%02X len=%u somme=%s cadrage=%s crc8darc=0x%02X(%s)%s%s",
           (unsigned long) cb_n_frames, opcode, s_frame_len, sum_ok ? "OK" : "KO", framing_ok ? "OK" : "KO", crc,
           crc_ok ? "OK" : "KO", phase ? "  <<<< 0x22 : CHANGEMENT DE PHASE >>>>" : "",
           live ? "  <<<< 0x21 : VALEURS VIVANTES >>>>" : "");
  log_context("%s", line);
  if (opcode == 0x21 && s_frame_len >= 9) {
    log_context("      en-tete apres FF 21 : %02X %02X %02X %02X, LE16 en 1000-3500 : %u", s_frame[5], s_frame[6],
                s_frame[7], s_frame[8], plausibles);
  }
  log_hex("      ", s_frame, s_frame_len);

  /* Home-Assistant-Ereignis: dieselbe Zeile mit den Rohbytes, damit eine
   * Auswertung in HA ohne USB-Zugriff moeglich bleibt (Ersatz fuer den
   * RAW-Log des Pico). Es liegt immer nur die juengste Trame bereit. */
  {
    std::string event = "{\"line\":\"";
    event += line;
    event += "\",\"opcode\":\"";
    char op[8];
    snprintf(op, sizeof(op), "0x%02X", opcode);
    event += op;
    event += "\",\"hex\":\"";
    for (uint16_t i = 0; i < s_frame_len; i++) {
      char b[3];
      snprintf(b, sizeof(b), "%02X", s_frame[i]);
      event += b;
    }
    event += "\"}";
    lock();
    s_shared.frame_line = line;
    s_shared.frame_event = std::move(event);
    s_shared.frame_event_pending = true;
    unlock();
  }

  /* Lokaler 0x21: dekodieren und fuer den Main-Loop vormerken. */
  if (valid && opcode == 0x21 && s_frame_len == ALDES_REPORT21_LEN) {
    aldes_report21_t decoded;
    if (aldes_decode21(s_frame, s_frame_len, &decoded) != nullptr) {
      lock();
      s_shared.report = decoded;
      s_shared.report_ms = now_ms();
      s_shared.report_pending = true;
      unlock();
    }
  }
  frame_reset();
}

void frame_feed(const uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    const uint8_t b = data[i];
    switch (s_frame_state) {
      case 0:
        if (b == 0xFA) {
          s_frame[0] = b;
          s_frame_idx = 1;
          s_frame_state = 1;
        }
        break;
      case 1:
        if (b == 0xFD) {
          s_frame[1] = b;
          s_frame_idx = 2;
          s_frame_state = 2;
        } else {
          frame_reset();
          if (b == 0xFA) { /* sofortige Resynchronisation auf ein neues FA */
            s_frame[0] = b;
            s_frame_idx = 1;
            s_frame_state = 1;
          }
        }
        break;
      case 2:
        s_frame[2] = b;
        s_frame_len = b;
        s_frame_idx = 3;
        if (s_frame_len < 6 || s_frame_len > FRAME_MAX) {
          log_context("FRAME longueur aberrante 0x%02X, resynchro", b);
          frame_reset();
        } else {
          s_frame_state = 3;
        }
        break;
      case 3:
        s_frame[s_frame_idx++] = b;
        if (s_frame_idx >= s_frame_len) frame_emit();
        break;
      default:
        frame_reset();
        break;
    }
  }
}

void frame_flush_inactivity() {
  if (s_frame_state == 0) return;
  cb_n_truncated++;
  log_context("FRAME tronquee (silence de %u ms a %u/%u octets) -> resynchro", (unsigned) INACTIVITY_MS, s_frame_idx,
              s_frame_len);
  frame_reset();
}

/* -------------------------------------------------- Senden (Main-Loop) */

bool tx_start(const uint8_t *data, uint16_t len, uint32_t id) {
  if (!len || len > TX_MAX || tx_busy || !tud_inited() || !tud_mounted() || !s_ep_in) return false;
  if (!usbd_edpt_claim(0, s_ep_in)) {
    tx_diag("CLAIM_FAILED");
    return false;
  }
  memcpy(tx_buf, data, len);
  tx_len = len;
  tx_id = id;
  tx_zlp = false;
  tx_busy = true;
  tx_started = now_ms();
  tx_state = "PENDING";
  log_context("TX id=%lu SUBMIT len=%u (not proof of PAC action)", (unsigned long) id, len);
  log_hex("TX raw", tx_buf, len);
  tx_diag("BEFORE_ARM");
  if (!usbd_edpt_xfer(0, s_ep_in, tx_buf, len, IS_ISR)) {
    usbd_edpt_release(0, s_ep_in);
    tx_busy = false;
    tx_state = "SUBMIT_FAILED";
    tx_diag("ARM_FAILED");
    return false;
  }
  tx_diag("ARM_ACCEPTED");
  return true;
}

/* Wird beim Timeout und bei STOP gerufen: vollstaendige Deinstallation des
 * Controllers entfernt ausstehende DPRAM-Pakete. Bis zum expliziten USBON
 * bleibt das Geraet abgemeldet — die abgebrochene Anfrage wird nie wiederholt. */
void tx_cancel(const char *why) {
  if (!tx_busy) return;
  aldes_usb_uninstall();
  tx_busy = false;
  tx_state = "ABORTED_USB_OFF";
  lock();
  aldes_gate_disarm(&s_shared.gate);
  unlock();
  log_context("TX id=%lu ABORTED_USB_OFF; partial delivery possible; no retry (%s)", (unsigned long) tx_id, why);
}

void tx_service() {
  if (tx_busy && (uint32_t) (now_ms() - tx_started) >= CB_TX_TIMEOUT_MS) tx_cancel("delai 90 s");
}

/* ------------------------------------------------------ class driver */

void cb_init() {
  s_ep_out = s_ep_in = s_ep_notif = 0;
  frame_reset();
}

bool cb_deinit() { return true; }

void cb_reset(uint8_t rhport) {
  if (tx_busy) {
    tx_busy = false;
    tx_state = "RESET_DELIVERY_UNKNOWN";
  }
  lock();
  aldes_gate_disarm(&s_shared.gate);
  unlock();
  (void) rhport;
  s_ep_out = s_ep_in = s_ep_notif = 0;
  frame_reset();
  log_context("USB bus reset");
}

/* usbd ruft open() interface-weise auf. Ohne IAD und mit CFG_TUD_CDC=0 ist
 * die Kopplung (assoc_itf_count=2) abgeschaltet; wuerden wir beide Interfaces
 * auf einmal konsumieren, bliebe itf2drv[1] unbesetzt und jede Anfrage an
 * Interface 1 ginge in einen stillen STALL — ohne Log. Also je Interface. */
uint16_t cb_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  uint8_t const *p_desc = (uint8_t const *) itf_desc;

  if (TUSB_CLASS_CDC_DATA == itf_desc->bInterfaceClass) {
    uint16_t drv_len = tu_desc_len(p_desc);
    p_desc = tu_desc_next(p_desc);
    TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, 2, TUSB_XFER_BULK, &s_ep_out, &s_ep_in), 0);
    drv_len = (uint16_t) (drv_len + 2 * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(drv_len <= max_len, 0);
    log_context("OPEN itf data : out=0x%02X in=0x%02X (drv_len=%u)", s_ep_out, s_ep_in, drv_len);
    /* Empfang sofort; Bulk IN bleibt ohne ausdrueckliche Anfrage stumm. */
    if (s_ep_out) {
      TU_ASSERT(usbd_edpt_xfer(rhport, s_ep_out, s_out_buf, sizeof(s_out_buf), IS_ISR), 0);
    }
    return drv_len;
  }

  TU_VERIFY(TUSB_CLASS_CDC == itf_desc->bInterfaceClass &&
                CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL == itf_desc->bInterfaceSubClass,
            0);

  uint16_t drv_len = tu_desc_len(p_desc);
  p_desc = tu_desc_next(p_desc);
  while (TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len) {
    drv_len = (uint16_t) (drv_len + tu_desc_len(p_desc));
    p_desc = tu_desc_next(p_desc);
  }
  if (TUSB_DESC_ENDPOINT == tu_desc_type(p_desc)) {
    tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p_desc;
    TU_ASSERT(usbd_edpt_open(rhport, ep), 0);
    s_ep_notif = ep->bEndpointAddress;
    drv_len = (uint16_t) (drv_len + tu_desc_len(p_desc));
    p_desc = tu_desc_next(p_desc);
  }
  TU_VERIFY(drv_len <= max_len, 0);
  log_context("OPEN itf ctrl : notif=0x%02X (drv_len=%u)", s_ep_notif, drv_len);
  return drv_len;
}

bool cb_control_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  if (request->bRequest == CDC_REQUEST_GET_LINE_CODING && request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
      stage != CONTROL_STAGE_SETUP) {
    if (stage == CONTROL_STAGE_DATA) lc_diag_data++;
    if (stage == CONTROL_STAGE_ACK) lc_diag_ack++;
    log_context("LCDIAG phase=%u", (unsigned) stage);
  }
  if (stage == CONTROL_STAGE_SETUP) {
    log_context("CTRL bmRequestType=0x%02X bRequest=0x%02X wValue=0x%04X wIndex=0x%04X wLength=%u",
                request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength);
  }

  /* Nur Klassenanfragen; der Standardkram (GET/SET_INTERFACE) geht zurueck an
   * usbd, das ihn selbst beantwortet. */
  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;

  switch (request->bRequest) {
    case CDC_REQUEST_GET_LINE_CODING:
      if (stage == CONTROL_STAGE_SETUP) {
        cb_n_get_line_coding++;
        log_context("  GET_LINE_CODING #%lu -> reponse %02X %02X %02X %02X %02X %02X %02X",
                    (unsigned long) cb_n_get_line_coding, s_line_coding[0], s_line_coding[1], s_line_coding[2],
                    s_line_coding[3], s_line_coding[4], s_line_coding[5], s_line_coding[6]);
        const bool queued = tud_control_xfer(rhport, request, s_line_coding, 7);
        if (queued) lc_diag_queued++;
        else lc_diag_failed++;
        log_context("LCDIAG queued=%u", (unsigned) queued);
      }
      break;

    case CDC_REQUEST_SET_LINE_CODING:
      if (stage == CONTROL_STAGE_SETUP) {
        cb_n_set_line_coding++;
        tud_control_xfer(rhport, request, s_set_lc_buf, 7);
      } else if (stage == CONTROL_STAGE_DATA) {
        log_context("  SET_LINE_CODING #%lu = %02X %02X %02X %02X %02X %02X %02X",
                    (unsigned long) cb_n_set_line_coding, s_set_lc_buf[0], s_set_lc_buf[1], s_set_lc_buf[2],
                    s_set_lc_buf[3], s_set_lc_buf[4], s_set_lc_buf[5], s_set_lc_buf[6]);
      }
      break;

    case CDC_REQUEST_SET_CONTROL_LINE_STATE:
      if (stage == CONTROL_STAGE_SETUP) {
        cb_n_set_control_line_state++;
        log_context("  SET_CONTROL_LINE_STATE #%lu DTR=%u RTS=%u", (unsigned long) cb_n_set_control_line_state,
                    request->wValue & 1u, (request->wValue >> 1) & 1u);
        tud_control_status(rhport, request);
      }
      break;

    case CDC_REQUEST_SEND_BREAK:
      if (stage == CONTROL_STAGE_SETUP) {
        cb_n_send_break++;
        log_context("  SEND_BREAK #%lu", (unsigned long) cb_n_send_break);
        tud_control_status(rhport, request);
      }
      break;

    default:
      if (stage == CONTROL_STAGE_SETUP) {
        cb_n_ctrl_other++;
        log_context("  requete de classe INCONNUE 0x%02X -> stall", request->bRequest);
      }
      return false;
  }
  return true;
}

bool cb_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  if (ep_addr == s_ep_out) {
    if (result != XFER_RESULT_SUCCESS) {
      cb_n_out_errors++;
      log_context("OUT ERROR result=%u bytes=%lu count=%lu", (unsigned) result, (unsigned long) xferred_bytes,
                  (unsigned long) cb_n_out_errors);
    } else if (!xferred_bytes) {
      cb_n_out_zlp++;
      log_context("OUT ZLP count=%lu", (unsigned long) cb_n_out_zlp);
    }
    if (result == XFER_RESULT_SUCCESS && xferred_bytes) {
      lock();
      s_shared.last_rx_ms = now_ms();
      unlock();
      cb_n_out_xfer++;
      cb_n_out_bytes += xferred_bytes;
      log_context("OUT xfer #%lu, %lu octets", (unsigned long) cb_n_out_xfer, (unsigned long) xferred_bytes);
      log_hex("   raw", s_out_buf, xferred_bytes);
      frame_feed(s_out_buf, xferred_bytes);
      /* HS: ein 64-Byte-Paket ist kein "kurzes Paket". Die uebrige Trame
       * schliesst frame_flush_inactivity() im Main-Loop, wenn Stille eintritt. */
    }
    /* Sofortige Neuarmierung: ein laengeres NAK verschoebe das beobachtete
     * Timing (Pakete des PAC gingen verloren). */
    if (!usbd_edpt_xfer(rhport, s_ep_out, s_out_buf, sizeof(s_out_buf), IS_ISR)) {
      cb_n_out_rearm_failed++;
      log_context("OUT REARM_FAILED count=%lu", (unsigned long) cb_n_out_rearm_failed);
    }
  } else if (ep_addr == s_ep_in) {
    tx_diag("CALLBACK");
    if (tx_busy) {
      if (result != XFER_RESULT_SUCCESS || xferred_bytes != (tx_zlp ? 0u : tx_len)) {
        tx_busy = false;
        tx_state = "TRANSFER_FAILED";
      } else if (!tx_zlp && tx_len % 64u == 0) {
        tx_zlp = true;
        if (!usbd_edpt_xfer(rhport, s_ep_in, NULL, 0, IS_ISR)) {
          tx_busy = false;
          tx_state = "ZLP_FAILED";
        }
      } else {
        tx_busy = false;
        tx_state = "USB_DONE";
      }
      if (!tx_busy) {
        lock();
        s_shared.last_done_ms = now_ms();
        s_shared.have_done = true;
        unlock();
      }
      log_context("TX id=%lu state=%s result=%u bytes=%lu (USB receipt only)", (unsigned long) tx_id, tx_state,
                  (unsigned) result, (unsigned long) xferred_bytes);
    }
  }
  return true;
}

usbd_class_driver_t const s_cb_driver[] = {{
    "CBCLONE", cb_init, cb_deinit, cb_reset, cb_open, cb_control_xfer, cb_xfer_cb, nullptr, nullptr,
}};

/* esp_tinyusb-Meldehaken: mount/unmount ergaenzen den Pico-Log. */
void tinyusb_event(tinyusb_event_t *event, void *arg) {
  (void) arg;
  if (event->id == TINYUSB_EVENT_ATTACHED) {
    aldes_desc_mount_event();
  } else if (event->id == TINYUSB_EVENT_DETACHED) {
    if (tx_busy) {
      tx_busy = false;
      tx_state = "RESET_DELIVERY_UNKNOWN";
    }
    aldes_desc_umount_event();
  }
}

/* Einmalige Installation. Nach einem Abbruch (STOP/Timeout/Reset) ruft der
 * Baustein dieselbe Prozedur ueber USBON noch einmal; esp_tinyusb verlangt
 * dafuer den vorherigen Uninstall. */
esp_err_t install_tinyusb() {
  const tusb_desc_device_t *device = nullptr;
  const tusb_desc_device_qualifier_t *qualifier = nullptr;
  const char **strings = nullptr;
  int string_count = 0;
  aldes_desc_get_tinyusb_config(&device, &qualifier, &strings, &string_count);

  tinyusb_config_t config = {};
#if SOC_USB_OTG_PERIPH_NUM > 1
  config.port = TINYUSB_PORT_HIGH_SPEED_0;
#else
  config.port = TINYUSB_PORT_FULL_SPEED_0; /* der einzige Port des S31 ist HS */
#endif
  config.phy = {
      .skip_setup = false,
      .self_powered = true,
      /* Dauerhaft auf GND, s. esphome/README.md: Auf dem CoreBoard ist kein
       * VBUS-GPIO verdrahtet; ohne konfigurierten Monitor meldet esp_tinyusb
       * 2.2.1 den High-Speed-Port aber nie als angeschlossen. Ein konstantes
       * LOW gilt als "kein Abziehen" (Interrupt nur auf fallende Flanke). */
      .vbus_monitor_io = 0,
  };
  config.task = {
      .size = 4096,
      .priority = 5,
      .xCoreID = 1,
  };
  config.descriptor = {
      .device = device,
      .qualifier = qualifier,
      .string = strings,
      .string_count = string_count,
      .full_speed_config = aldes_desc_fs_configuration(),
      .high_speed_config = aldes_desc_hs_configuration(),
  };
  config.event_cb = tinyusb_event;
  config.event_arg = nullptr;

  const esp_err_t err = tinyusb_driver_install(&config);
  aldes_log_printf("USB install port=%d -> %s", (int) config.port, err == ESP_OK ? "OK" : "ECHEC");
  return err;
}

}  // namespace

/* ------------------------------------------------------------------ C-API */

extern "C" {

/* Wird von TinyUSB bei tud_init() gerufen: Anwendungstreiber stehen VOR den
 * eingebauten (TinyUSB 0.21, usbd.c:get_driver). */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return s_cb_driver;
}

const char *aldes_variant_name(void) { return s_variant; }

bool aldes_usb_ready() { return tud_inited() && tud_mounted(); }
bool aldes_usb_initialized() { return tud_inited(); }
bool aldes_usb_mounted() { return tud_mounted(); }
bool aldes_tx_busy() { return tx_busy; }
const char *aldes_tx_state() { return tx_state; }
uint32_t aldes_tx_id() { return tx_id; }

esp_err_t aldes_usb_install() { return install_tinyusb(); }
esp_err_t aldes_usb_uninstall() { return tinyusb_driver_uninstall(); }

void aldes_tx_service() { tx_service(); }

void aldes_frames_flush_inactivity() {
  if (s_frame_state == 0) return;
  uint32_t idle = 0;
  lock();
  idle = now_ms() - s_shared.last_rx_ms;
  unlock();
  if (idle >= INACTIVITY_MS) frame_flush_inactivity();
}

bool aldes_tx_start_public(const uint8_t *data, uint16_t len, uint32_t id) { return tx_start(data, len, id); }

void aldes_diag_loop(uint32_t usb_us, uint32_t control_us) {
  const uint32_t now = now_ms();
  if (diag_last_loop && now - diag_last_loop > diag_loop_gap_max_us) diag_loop_gap_max_us = now - diag_last_loop;
  diag_last_loop = now;
  diag_loops++;
  if (usb_us > diag_usb_max_us) diag_usb_max_us = usb_us;
  if (control_us > diag_control_max_us) diag_control_max_us = control_us;
  if (now - lc_diag_report_ms >= 10000u) {
    lc_diag_report_ms = now;
    log_context("LCDIAG cumulative queued=%lu failed=%lu data=%lu ack=%lu", (unsigned long) lc_diag_queued,
                (unsigned long) lc_diag_failed, (unsigned long) lc_diag_data, (unsigned long) lc_diag_ack);
  }
  if (tx_busy && now - diag_last_report >= 1000u) {
    diag_last_report = now;
    tx_diag("PENDING");
  }
}

/* Testhaken: Kanal-Einspeisung ohne USB (Host-Test; auch von HA-Lambdas aus
 * zur Diagnose mit bekannten Frames nutzbar — rein lesend, kein Senden). */
void aldes_test_feed(const uint8_t *data, uint32_t len) {
  lock();
  s_shared.last_rx_ms = now_ms();
  unlock();
  cb_n_out_xfer++;
  cb_n_out_bytes += len;
  frame_feed(data, len);
}

/* Testhaken: den zuletzt dekodierten 0x21 abholen (einmalig). */
bool aldes_test_last_report(aldes_report21_t *out) {
  lock();
  const bool pending = s_shared.report_pending;
  if (pending) *out = s_shared.report;
  unlock();
  return pending;
}

}  // extern "C"

/* ------------------------------------------------------ Implementierung AldesCB */

bool AldesCB::is_ready() const { return aldes_usb_ready(); }

/* ------------------------------------------------------------------ Sinks */

void TextSink::flush() {
  if (!pending_ || line_.empty()) return;
  pending_ = false;
  for (auto &cb : callbacks_) cb(line_);
  /* Ohne verbundenen TextSensor bleibt die Zeile im Pico-Log sichtbar. */
  if (parent_ != nullptr && callbacks_.empty()) parent_->log_line_("%s", line_.c_str());
}

void FloatSink::flush() {
  if (!pending_) return;
  pending_ = false;
  if (parent_ != nullptr && callbacks_.empty()) {
    if (std::isnan(value_)) parent_->log_line_("VALEUR indisponible");
  }
  for (auto &cb : callbacks_) cb(value_);
}

void IndexSink::control(size_t index) {
  if (names_.empty()) {
    if (index >= options_) return;
  } else if (index >= names_.size()) {
    return;
  }
  pending_controls_.push_back(index);
}

void IndexSink::control_name(const std::string &name) {
  for (size_t i = 0; i < names_.size(); i++) {
    if (names_[i] == name) {
      pending_controls_.push_back(i);
      return;
    }
  }
  if (parent_ != nullptr) parent_->log_line_("MODE inconnu : %s", name.c_str());
}

void IndexSink::flush() {
  if (pending_) {
    pending_ = false;
    const float value = value_;
    if (parent_ != nullptr && callbacks_.empty() && !std::isnan(value)) {
      parent_->log_line_("MODE valeur=%u", (unsigned) value);
    }
    for (auto &cb : callbacks_) cb(value);
  }
  if (!pending_controls_.empty()) {
    auto pending = std::move(pending_controls_);
    pending_controls_.clear();
    for (size_t idx : pending) {
      for (auto &cb : on_control_) cb(idx);
    }
  }
}

/* ------------------------------------------------------------------ USB */

void AldesCB::init_usb() {
  s_shared.owner = this;
  if (s_shared.lock == nullptr) s_shared.lock = xSemaphoreCreateRecursiveMutex();
  if (install_tinyusb() != ESP_OK) this->mark_failed();
}

void AldesCB::install_usb() {
  if (tud_inited()) return;
  if (install_tinyusb() != ESP_OK) ESP_LOGE(TAG, "USBON: Installation fehlgeschlagen");
}

void AldesCB::set_report_sensor(uint8_t slot, FloatSink *sink) {
  if (slot < SLOT_COUNT) report_sinks_[slot] = sink;
}

void AldesCB::set_setpoint_number(uint8_t zone, NumberSink *number) {
  if (zone >= 1 && zone <= 4) setpoint_numbers_[zone - 1] = number;
}

float AldesCB::get_setup_priority() const { return setup_priority::LATE; }

void AldesCB::setup() {
  s_shared.owner = this;
  if (s_shared.lock == nullptr) s_shared.lock = xSemaphoreCreateRecursiveMutex();
  if (log_sink_ == nullptr) log_sink_ = new TextSink(this);

  if (btn_send_setpoints_ != nullptr) btn_send_setpoints_->add_on_press_callback([this]() { this->send_setpoints(); });
  if (btn_send_air_ != nullptr) btn_send_air_->add_on_press_callback([this]() { this->send_air(); });
  if (btn_send_ecs_ != nullptr) btn_send_ecs_->add_on_press_callback([this]() { this->send_ecs(); });
  if (btn_vacation_clear_ != nullptr)
    btn_vacation_clear_->add_on_press_callback([this]() { this->vacation_clear(); });
  if (btn_usb_on_ != nullptr) btn_usb_on_->add_on_press_callback([this]() { this->usb_on_from_ui(); });
  if (btn_arm_ != nullptr) btn_arm_->add_on_press_callback([this]() { this->arm_from_ui(); });
  if (btn_stop_ != nullptr) btn_stop_->add_on_press_callback([this]() { this->stop_from_ui(); });

  /* Ein Wert aus der HA-Auswahlliste gilt ab dem Moment als gewaehlt. */
  if (air_select_ != nullptr) air_select_->add_on_control_callback([this](size_t idx) { this->air_selected(idx); });
  if (ecs_select_ != nullptr) ecs_select_->add_on_control_callback([this](size_t idx) { this->ecs_selected(idx); });

  if (services_enabled_) {
    services_ = new AldesCBServices(this);
    services_->setup();
  }

  /* USB einmalig aufbauen, wie setup() in main.c: erst hier, damit alle
   * Sensoren haengen und die Mount-Zeile im Log-Sink ankommt. */
  this->init_usb();
}

void AldesCB::on_shutdown() {
  /* Der TinyUSB-Task haelt keinen Endpunkt mehr, bevor der Chip neu startet. */
  if (tud_inited()) tinyusb_driver_uninstall();
}

void AldesCB::loop() {
  const uint32_t now = now_ms();
  aldes_tx_service();
  aldes_frames_flush_inactivity();
  aldes_diag_loop(0, 0);
  publish_(now);

  if (status_text_ != nullptr && (first_status_ || (uint32_t) (now - last_status_ms_) >= status_period_ms_)) {
    first_status_ = false;
    last_status_ms_ = now;
    status_text_->publish_state(status_line());
  }

  std::string frame_line;
  std::string frame_event;
  bool have_frame = false;
  lock();
  if (s_shared.frame_event_pending) {
    s_shared.frame_event_pending = false;
    frame_line = std::move(s_shared.frame_line);
    frame_event = std::move(s_shared.frame_event);
    have_frame = true;
  }
  unlock();
  if (have_frame) {
    if (frame_text_ != nullptr) frame_text_->publish_state(frame_line);
    /* Das Ereignis traegt Zeile, Opcode und Rohhex. Der API-Aufruf selbst steht
     * im YAML (on_value-Lambda auf einem text_sensor mit
     * fire_homeassistant_event), weil nur die generierte API-Schicht weiss, ob
     * Events freigeschaltet sind. */
    if (frame_event_text_ != nullptr) frame_event_text_->publish_state(frame_event);
  }

  /* Alle Zwischenklassen am Ende des Durchlaufs schieben: Sensoren duerfen nur
   * hier (Main-Loop) publiziert werden. */
  if (log_sink_ != nullptr) log_sink_->flush();
  for (auto *sink : report_sinks_) {
    if (sink != nullptr) sink->flush();
  }
  if (air_select_ != nullptr) air_select_->flush();
  if (ecs_select_ != nullptr) ecs_select_->flush();
  if (status_text_ != nullptr) status_text_->flush();
  if (frame_text_ != nullptr) frame_text_->flush();
  if (frame_event_text_ != nullptr) frame_event_text_->flush();
}

float AldesCB::report_value_(uint8_t slot, const aldes_report21_t &r) const {
  switch (slot) {
    case SLOT_SETPOINT_K1:
    case SLOT_SETPOINT_K2:
    case SLOT_SETPOINT_K3:
    case SLOT_SETPOINT_K4: {
      const aldes_zone21_t &z = r.zone[slot - SLOT_SETPOINT_K1];
      return z.has_setpoint ? z.setpoint_hundredths / 100.0f : ALDES_NAN;
    }
    case SLOT_TEMPERATURE_K1:
    case SLOT_TEMPERATURE_K2:
    case SLOT_TEMPERATURE_K3:
    case SLOT_TEMPERATURE_K4: {
      const aldes_zone21_t &z = r.zone[slot - SLOT_TEMPERATURE_K1];
      return z.has_temperature ? z.temperature_hundredths / 100.0f : ALDES_NAN;
    }
    case SLOT_AIR:
      return r.air.raw != 0xFF ? (float) r.air.raw : ALDES_NAN;
    case SLOT_ECS:
      return r.ecs.raw != 0xFF ? (float) r.ecs.raw : ALDES_NAN;
    case SLOT_CLOCK:
      return r.clock_valid ? (float) pac_clock_to_epoch_(r) : ALDES_NAN;
    default:
      return ALDES_NAN;
  }
}

void AldesCB::publish_(uint32_t now) {
  aldes_report21_t report{};
  bool fresh = false;
  bool live = false;
  bool published = false;
  lock();
  if (s_shared.report_pending) {
    s_shared.report_pending = false;
    s_shared.last_good = s_shared.report;
    s_shared.last_good_ms = s_shared.report_ms;
    fresh = true;
    published_good_ = true;
  }
  report = s_shared.last_good;
  live = s_shared.last_good.present && (uint32_t) (now - s_shared.last_good_ms) <= report_window_s_ * 1000u;
  published = published_good_;
  unlock();

  /* Wie reader.snapshot(): ein Bericht gilt nur eine Weile (Pico-Grenze 25 s,
   * Kadenz ~20 s). Danach werden Sollwerte ungueltig, statt still zu veralten. */
  if (!fresh && !published) return;
  for (uint8_t slot = 0; slot < SLOT_COUNT; slot++) {
    FloatSink *sink = report_sinks_[slot];
    if (sink == nullptr) continue;
    sink->publish_state(live ? report_value_(slot, report) : ALDES_NAN);
  }
  if (fresh && live) {
    if (air_select_ != nullptr && report.air.raw != 0xFF) air_select_->publish_state((float) report.air.raw);
    if (ecs_select_ != nullptr && report.ecs.raw != 0xFF) ecs_select_->publish_state((float) report.ecs.raw);
  } else if (!live) {
    if (air_select_ != nullptr) air_select_->publish_state(ALDES_NAN);
    if (ecs_select_ != nullptr) ecs_select_->publish_state(ALDES_NAN);
  }
}

time_t AldesCB::pac_clock_to_epoch_(const aldes_report21_t &r) const {
  /* DOS/FAT-Zeit ist lokale PAC-Zeit ohne Zeitzone. Wir bilden daraus einen
   * UTC-Epochenwert ueber days_from_civil(); die Grenze ist dokumentiert. */
  int64_t y = r.clock_year;
  const int64_t m = r.clock_month;
  const int64_t era_y = y - (m <= 2 ? 1 : 0);
  const int64_t era = (era_y >= 0 ? era_y : era_y - 399) / 400;
  const int64_t yoe = era_y - era * 400;
  const int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + r.clock_day - 1;
  const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = era * 146097 + doe - 719468;
  return (time_t) (days * 86400 + (int64_t) r.clock_hour * 3600 + (int64_t) r.clock_minute * 60 + r.clock_second);
}

/* ---------------------------------------------------------- Befehle */

void AldesCB::log_line_(const char *fmt, ...) {
  char line[480];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  aldes_log_printf("%s", line);
  if (log_sink_ != nullptr) log_sink_->publish_state(std::string(line));
}

std::string AldesCB::status_line() {
  char line[400];
  snprintf(
      line, sizeof(line),
      "STATUS mounted=%d | GET_LC=%lu SET_LC=%lu SET_CLS=%lu BREAK=%lu autres=%lu | OUT "
      "xfer=%lu octets=%lu | trames=%lu dont phase(0x22)=%lu live(0x21)=%lu invalides=%lu tronquees=%lu "
      "| log perdus=0 | rx_errors=%lu rx_zlp=%lu rx_rearm_failed=%lu",
      aldes_usb_mounted() ? 1 : 0, (unsigned long) cb_n_get_line_coding, (unsigned long) cb_n_set_line_coding,
      (unsigned long) cb_n_set_control_line_state, (unsigned long) cb_n_send_break, (unsigned long) cb_n_ctrl_other,
      (unsigned long) cb_n_out_xfer, (unsigned long) cb_n_out_bytes, (unsigned long) cb_n_frames,
      (unsigned long) cb_n_frames_phase, (unsigned long) cb_n_frames_live, (unsigned long) cb_n_frames_bad,
      (unsigned long) cb_n_truncated, (unsigned long) cb_n_out_errors, (unsigned long) cb_n_out_zlp,
      (unsigned long) cb_n_out_rearm_failed);
  return std::string(line);
}

bool AldesCB::send_allowed_(std::string *why) {
  const uint32_t now = now_ms();
  if (!aldes_usb_ready()) {
    if (why != nullptr) *why = "USB_NOT_READY";
    return false;
  }
  if (aldes_tx_busy()) {
    if (why != nullptr) *why = "BUSY";
    return false;
  }
  lock();
  const bool have_done = s_shared.have_done;
  const uint32_t last_done = s_shared.last_done_ms;
  unlock();
  if (have_done && aldes_cooldown_remaining(now, last_done) > 0) {
    if (why != nullptr) *why = "COOLDOWN";
    return false;
  }
  return true;
}

void AldesCB::submit_frame_(const uint8_t *frame, size_t len, const char *action) {
  const uint32_t now = now_ms();
  std::string why;
  if (!send_allowed_(&why)) {
    this->log_line_("SEND %s refus (%s)", action, why.c_str());
    return;
  }
  uint32_t id = 0;
  lock();
  id = aldes_gate_next_id(&s_shared.gate);
  aldes_gate_arm(&s_shared.gate, now);
  /* Freigabe sofort verbrauchen: genau ein Versuch. */
  aldes_gate_take(&s_shared.gate, now, id);
  unlock();
  char hex[ALDES_WRITE10_LEN * 2 + 1];
  aldes_hex_encode(frame, (uint16_t) len, hex, sizeof(hex));
  this->log_line_("ACTION %s id=%lu frame=%s", action, (unsigned long) id, hex);
  if (!aldes_tx_start_public(frame, (uint16_t) len, id)) {
    this->log_line_("ACTION %s id=%lu ERR NOT_SUBMITTED ID_CONSUMED", action, (unsigned long) id);
  }
}

void AldesCB::request_send_(uint8_t kind, float value, uint8_t zone) {
  uint8_t frame[ALDES_WRITE10_LEN];
  bool ok = false;
  const char *action = "?";
  switch (kind) {
    case 0:
      ok = aldes_frame_setpoint(zone, value, frame);
      action = "SETPOINT";
      break;
    case 1:
      ok = aldes_frame_air((unsigned) value, frame);
      action = "AIR";
      break;
    case 2:
      ok = aldes_frame_ecs((unsigned) value, frame);
      action = "ECS";
      break;
    case 3:
      ok = aldes_frame_vacation_clear(frame);
      action = "VACATION_CLEAR";
      break;
    default:
      break;
  }
  if (!ok) {
    this->log_line_("SEND kind=%u valeur=%.2f zone=%u hors plage", kind, value, zone);
    return;
  }
  this->submit_frame_(frame, sizeof(frame), action);
}

void AldesCB::send_setpoints() {
  /* Vier Frames = vier Versuche. Die Pico-Workbench haette sie einzeln mit je
   * frischer Freigabe gesendet; hier laufen sie direkt nacheinander, mit dem
   * 45-s-Riegel nur vor dem ersten (die Sperre startet erst nach "done"). */
  for (uint8_t zone = 1; zone <= 4; zone++) {
    NumberSink *number = setpoint_numbers_[zone - 1];
    if (number == nullptr || !number->has()) {
      this->log_line_("SEND SETPOINT K%u non envoye (valeur non choisie)", zone);
      continue;
    }
    this->request_send_(0, number->state(), zone);
  }
}

void AldesCB::send_air() {
  if (!air_choice_set_) {
    this->log_line_("SEND AIR non envoye (mode non choisi)");
    return;
  }
  this->request_send_(1, (float) air_choice_, 0);
}

void AldesCB::send_ecs() {
  if (!ecs_choice_set_) {
    this->log_line_("SEND ECS non envoye (mode non choisi)");
    return;
  }
  this->request_send_(2, (float) ecs_choice_, 0);
}

void AldesCB::vacation_clear() { this->request_send_(3, 0.0f, 0); }

void AldesCB::arm_from_ui() {
  std::string why;
  if (!send_allowed_(&why)) {
    this->log_line_("ARM refus (%s)", why.c_str());
    return;
  }
  lock();
  aldes_gate_arm(&s_shared.gate, now_ms());
  unlock();
  this->log_line_("ARM OK ONE_ATTEMPT 30000ms");
}

void AldesCB::stop_from_ui() {
  lock();
  aldes_gate_disarm(&s_shared.gate);
  unlock();
  if (aldes_tx_busy()) {
    tx_cancel("STOP");
  } else {
    this->log_line_("STOP OK (aucune emission)");
  }
}

void AldesCB::usb_on_from_ui() {
  lock();
  aldes_gate_disarm(&s_shared.gate);
  unlock();
  if (aldes_tx_busy()) {
    this->log_line_("USBON ERR BUSY");
    return;
  }
  if (!aldes_usb_initialized()) this->install_usb();
  this->log_line_("USBON %s NO_REPLAY", aldes_usb_initialized() ? "OK" : "ERR USB_INIT");
}

/* ------------------------------------------------- WORKBENCH-1-Kommandos */

std::string AldesCB::exec_workbench_line(const std::string &line) {
  char buf[96];
  if (line == "STATUS") {
    const uint32_t now = now_ms();
    bool armed = false;
    uint32_t last = 0;
    lock();
    armed = s_shared.gate.armed && (uint32_t) (now - s_shared.gate.armed_at) < ALDES_ARM_WINDOW_MS;
    last = s_shared.gate.last_id;
    unlock();
    snprintf(buf, sizeof(buf), "STATUS mounted=%u usb=%u armed=%u last=%lu txid=%lu tx=%s max=%u tx_timeout_ms=%u\n",
             aldes_usb_mounted() ? 1u : 0u, aldes_usb_initialized() ? 1u : 0u, armed ? 1u : 0u, (unsigned long) last,
             (unsigned long) aldes_tx_id(), aldes_tx_state(), (unsigned) TX_MAX, (unsigned) CB_TX_TIMEOUT_MS);
    return std::string(buf);
  }
  if (line == "ARM") {
    lock();
    aldes_gate_arm(&s_shared.gate, now_ms());
    unlock();
    return "OK ARM ONE_ATTEMPT 30000ms\n";
  }
  if (line == "STOP") {
    this->stop_from_ui();
    return "OK STOP\n";
  }
  if (line == "USBON") {
    lock();
    aldes_gate_disarm(&s_shared.gate);
    unlock();
    if (aldes_tx_busy()) return "ERR BUSY\n";
    if (!aldes_usb_initialized()) this->install_usb();
    return aldes_usb_initialized() ? "OK USBON NO_REPLAY\n" : "ERR USB_INIT\n";
  }

  uint32_t id = 0;
  uint16_t len = 0;
  static uint8_t bytes[TX_MAX];
  if (!aldes_parse_send(line.c_str(), &id, bytes, &len, sizeof(bytes))) {
    lock();
    aldes_gate_disarm(&s_shared.gate);
    unlock();
    return "ERR COMMAND_OR_HEX\n";
  }
  lock();
  const bool taken = aldes_gate_take(&s_shared.gate, now_ms(), id);
  unlock();
  if (!taken) return "ERR NOT_ARMED_EXPIRED_OR_ID\n";
  if (!aldes_tx_start_public(bytes, len, id)) return "ERR NOT_SUBMITTED ID_CONSUMED\n";
  return "OK SUBMITTED\n";
}

/* ------------------------------------------------------------ HA-Dienste */

void AldesCBServices::setup() {
#ifdef USE_API_USER_DEFINED_ACTIONS
  /* Vier Dienste wie die Workbench der Pico-Variante. `send` akzeptiert die
   * Aktionsnamen der Frames; alles andere wird abgewiesen, bevor gesendet
   * wird. */
  using SendService = api::CustomAPIDeviceService<AldesCBServices, std::string, float, int32_t>;
  using ArmService = api::CustomAPIDeviceService<AldesCBServices>;
  api::global_api_server->register_user_service(
      new SendService("send", std::array<std::string, 3>{"action", "value", "zone"}, this,
                      &AldesCBServices::on_send));
  api::global_api_server->register_user_service(
      new ArmService("arm", std::array<std::string, 0>{}, this, &AldesCBServices::on_arm));
  api::global_api_server->register_user_service(
      new ArmService("stop", std::array<std::string, 0>{}, this, &AldesCBServices::on_stop));
  api::global_api_server->register_user_service(
      new ArmService("usb_on", std::array<std::string, 0>{}, this, &AldesCBServices::on_usb_on));
#endif
}

void AldesCBServices::on_send(std::string action, float value, int32_t zone) {
  if (parent_ == nullptr) return;
  if (action == "setpoint") {
    parent_->request_send_(0, value, (uint8_t) zone);
  } else if (action == "air") {
    parent_->request_send_(1, value, 0);
  } else if (action == "ecs") {
    parent_->request_send_(2, value, 0);
  } else if (action == "vacation_clear") {
    parent_->request_send_(3, 0.0f, 0);
  } else {
    parent_->log_line_("SEND %s refus (ACTION_INCONNUE)", action.c_str());
  }
}

void AldesCBServices::on_arm() {
  if (parent_ != nullptr) parent_->arm_from_ui();
}

void AldesCBServices::on_stop() {
  if (parent_ != nullptr) parent_->stop_from_ui();
}

void AldesCBServices::on_usb_on() {
  if (parent_ != nullptr) parent_->usb_on_from_ui();
}

void AldesCB::dump_config() {
  ESP_LOGCONFIG(TAG, "AldesConnect-Box-Klon (ESP32-S31, HS):");
  ESP_LOGCONFIG(TAG, "  Variante: %s", aldes_variant_name());
  ESP_LOGCONFIG(TAG, "  0x21-Fenster: %u s, STATUS-Kadenz: %u ms", (unsigned) report_window_s_,
                (unsigned) status_period_ms_);
  ESP_LOGCONFIG(TAG, "  45-s-Riegel: %u ms, ARM-Fenster: %u ms, TX-Timeout: %u ms", (unsigned) ALDES_COOLDOWN_MS,
                (unsigned) ALDES_ARM_WINDOW_MS, (unsigned) ALDES_TX_TIMEOUT_MS);
}

}  // namespace esphome::aldes_cb
