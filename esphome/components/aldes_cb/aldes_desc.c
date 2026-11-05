/* Klon der AldesConnect Box — USB-Deskriptoren, byte-genau.
 *
 * Quelle der Bytes: öffentliche HACF-#505-Capture (1. Januar 2026), reproduziert
 * in FABLE_FRESH_USB_REVIEW.md. Die Tabellen unten sind wörtlich aus dieser
 * Capture übernommen — nicht "aufräumen", nicht mit den TUD_CDC_DESCRIPTOR-
 * Makros neu erzeugen: Das TinyUSB-Makro fügt eine IAD und eine Device-Klasse
 * 0xEF ein, was die echte ConnectBox nicht tut.
 *
 * Unterschied zur Pico-Firmware nur im Transport, nie in den Bytes:
 *  - esp_tinyusb 2.2.1 nimmt `device` als Zeiger auf `const tusb_desc_device_t`
 *    und reicht ihn an tud_descriptor_device_cb() weiter. Die byte-exakte
 *    Tabelle wird deshalb als C-Struktur gespiegelt (gleiche Werte, gleiches
 *    Little-Endian-Layout); der Host-Test vergleicht beide Serialisierungen.
 *  - Der S31 ist HS-only; die HS-Konfiguration ist wörtlich dieselbe
 *    64/8-Bytes-Tabelle (kein 512-Bump wie im Espressif-Default).
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "aldes_desc.h"

static const char *const TAG = "aldes_cb";

/* -------------------------------------------------------------------- Zähler */
uint32_t aldes_desc_count_device;
uint32_t aldes_desc_count_config;
uint32_t aldes_desc_count_string;
uint32_t aldes_desc_count_vendor;

void aldes_log_printf(const char *fmt, ...) {
  char line[480];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  /* Ursprungsformat jeder Zeile: "[%8lu] " = Boot-Zeit in Millisekunden. */
  ESP_LOGI(TAG, "[%8llu] %s", (unsigned long long) esp_log_timestamp(), line);
}

/* ------------------------------------------------------------------ device */
/* 12 01 00 02 02 02 00 40 83 04 40 57 00 02 01 02 03 01 */
static uint8_t const desc_device_bytes[18] = {
    0x12,       /* bLength                                                       */
    0x01,       /* bDescriptorType = DEVICE                                      */
    0x00, 0x02, /* bcdUSB 2.00                                                   */
    0x02,       /* bDeviceClass    = CDC                                         */
    0x02,       /* bDeviceSubClass = ACM                                         */
    0x00,       /* bDeviceProtocol                                               */
    0x40,       /* bMaxPacketSize0 = 64                                          */
    0x83, 0x04, /* idVendor  0x0483 STMicroelectronics                           */
    0x40, 0x57, /* idProduct 0x5740 STM32 Virtual ComPort                        */
    0x00, 0x02, /* bcdDevice 2.00                                                */
    0x01,       /* iManufacturer                                                 */
    0x02,       /* iProduct                                                      */
    0x03,       /* iSerialNumber                                                 */
    0x01        /* bNumConfigurations                                            */
};

/* Struktur-Spiegel der Tabelle: die Little-Endian-Felder dekodieren zu
 * 0x0200 / 0x0200 / 0x0483 / 0x5740 und serialisieren wieder zu den Bytes oben
 * (geprüft von aldes_test_run, Test "descriptors"). */
static tusb_desc_device_t const desc_device = {
    .bLength = 18,
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x02,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

/* ----------------------------------------------------------- configuration */
/* 67 Bytes, ohne IAD, self-powered 0xC0, 100 mA */
static uint8_t const desc_configuration[67] = {
    /* Configuration, 2 Interfaces, gesamt 0x0043 = 67                           */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32,
    /* Interface 0: CDC-Control, 1 EP, Klasse 02 / Sub 02 / Proto 01             */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* CDC Header, bcdCDC 1.10                                                   */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* CDC Call Management, capabilities 0x00, Data-Interface 1                  */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* CDC ACM, capabilities 0x02                                                */
    0x04, 0x24, 0x02, 0x02,
    /* CDC Union, Master 0, Slave 1                                              */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* EP 0x82 IN interrupt, 8 Bytes, bInterval 0x10                             */
    0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x10,
    /* Interface 1: CDC-Data, 2 EP, Klasse 0x0A                                  */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* EP 0x01 OUT bulk 64                                                       */
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
    /* EP 0x81 IN bulk 64                                                        */
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00};

/* Device-Qualifier (10 Bytes, USB 2.0 §9.6.2): ein High-Speed-fähiges Gerät
 * muss ihn beantworten. Die ConnectBox (Full-Speed-only STM32-CDC) liefert ihn
 * auf dem FS-Bus nicht — auf dem HS-Bus des S31 fragt der Host ihn zwingend
 * ab. Klassenfelder und bMaxPacketSize0 folgen dem Device-Deskriptor. */
static tusb_desc_device_qualifier_t const desc_qualifier = {
    .bLength = 10,
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x02,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00,
};

/* Nur für den Host-Test: Serialisierung des Qualifiers wie über EP0. */
static void qualifier_bytes(uint8_t *out) {
  const tusb_desc_device_qualifier_t *q = &desc_qualifier;
  out[0] = q->bLength;
  out[1] = q->bDescriptorType;
  out[2] = (uint8_t) (q->bcdUSB & 0xFF);
  out[3] = (uint8_t) (q->bcdUSB >> 8);
  out[4] = q->bDeviceClass;
  out[5] = q->bDeviceSubClass;
  out[6] = q->bDeviceProtocol;
  out[7] = q->bMaxPacketSize0;
  out[8] = q->bNumConfigurations;
  out[9] = q->bReserved;
}

/* ----------------------------------------------------------------- strings */
static char const *const string_desc_arr[] = {
    NULL,                    /* 0: langid, gesondert behandelt                  */
    "STMicroelectronics",    /* 1: iManufacturer                               */
    "STM32 Virtual ComPort", /* 2: iProduct                                    */
    "00000000001A",          /* 3: iSerialNumber                               */
};

#define ALDES_STRING_COUNT (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))

/* esp_tinyusb und TinyUSB 0.21 lesen string[0] als Zeiger auf
 * {LANGID_LO, LANGID_HI} und kopieren die zwei Bytes wörtlich; 0x0409 =
 * Englisch (USA), wie bei der ConnectBox. */
static const char langid_desc[2] = {(char) 0x09, (char) 0x04};

static char const *const tinyusb_string_desc_arr[] = {
    langid_desc,
    string_desc_arr[1],
    string_desc_arr[2],
    string_desc_arr[3],
};

#define ALDES_TINYUSB_STRING_COUNT (sizeof(tinyusb_string_desc_arr) / sizeof(tinyusb_string_desc_arr[0]))

static uint16_t _desc_str[32];

uint16_t const *aldes_desc_string_cb(uint8_t index, uint16_t langid) {
  uint8_t chr_count;

  aldes_desc_count_string++;
  aldes_log_printf("ENUM GET_DESCRIPTOR string index=%u langid=0x%04X", index, langid);

  if (index == 0) {
    _desc_str[1] = 0x0409; /* English (United States) */
    chr_count = 1;
  } else {
    if (index >= ALDES_STRING_COUNT) return NULL;

    char const *str = string_desc_arr[index];
    chr_count = (uint8_t) strlen(str);
    if (chr_count > 31) chr_count = 31;

    for (uint8_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = (uint16_t) str[i];
    }
  }

  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}

void aldes_desc_get_tinyusb_config(const tusb_desc_device_t **device, const tusb_desc_device_qualifier_t **qualifier,
                                   const char ***strings, int *string_count) {
  *device = &desc_device;
  *qualifier = &desc_qualifier;
  *strings = (const char **) tinyusb_string_desc_arr;
  *string_count = (int) ALDES_TINYUSB_STRING_COUNT;
}

/* Journalisierung auf Enumerationsebene. esp_tinyusb 2.2.1 liefert Device- und
 * Konfigurationsdeskriptor aus seiner eigenen Callback-Schicht, ohne Hook; der
 * einzige abfragbare Zähler dort ist die POST-ENUMERATION (tud_mount_cb() =
 * "ENUM SET_CONFIGURATION : device monte"), dieselbe Zeile wie auf dem Pico.
 * GET_DESCRIPTOR device/config bleibt deshalb in der ESPHome-Variante nicht
 * einzeln protokolliert — Unterschied dokumentiert in esphome/README.md. */
void aldes_desc_note_mount(void) {
  aldes_desc_count_device++;
  aldes_desc_count_config++;
}

/* ------------------------------------------------------ Debug-/Test-Hooks */
const uint8_t *aldes_desc_device_bytes(size_t *len) {
  *len = sizeof(desc_device_bytes);
  return desc_device_bytes;
}

size_t aldes_desc_configuration_len(void) { return sizeof(desc_configuration); }

uint8_t const *aldes_desc_configuration(void) { return desc_configuration; }

uint8_t const *aldes_desc_hs_configuration(void) { return desc_configuration; }

uint8_t const *aldes_desc_fs_configuration(void) { return desc_configuration; }

size_t aldes_desc_qualifier_len(void) { return 10; }

uint8_t const *aldes_desc_qualifier(void) {
  static uint8_t serialized[10];
  qualifier_bytes(serialized);
  return serialized;
}

/* Serialisiert die Struktur so, wie TinyUSB sie über EP0 sendet (18 Bytes,
 * Little-Endian), damit der Host-Test beide Formen vergleichen kann. */
void aldes_desc_device_struct_bytes(uint8_t *out) {
  const tusb_desc_device_t *d = &desc_device;
  out[0] = d->bLength;
  out[1] = d->bDescriptorType;
  out[2] = (uint8_t) (d->bcdUSB & 0xFF);
  out[3] = (uint8_t) (d->bcdUSB >> 8);
  out[4] = d->bDeviceClass;
  out[5] = d->bDeviceSubClass;
  out[6] = d->bDeviceProtocol;
  out[7] = d->bMaxPacketSize0;
  out[8] = (uint8_t) (d->idVendor & 0xFF);
  out[9] = (uint8_t) (d->idVendor >> 8);
  out[10] = (uint8_t) (d->idProduct & 0xFF);
  out[11] = (uint8_t) (d->idProduct >> 8);
  out[12] = (uint8_t) (d->bcdDevice & 0xFF);
  out[13] = (uint8_t) (d->bcdDevice >> 8);
  out[14] = d->iManufacturer;
  out[15] = d->iProduct;
  out[16] = d->iSerialNumber;
  out[17] = d->bNumConfigurations;
}

const char *const *aldes_desc_strings(size_t *count) {
  *count = ALDES_STRING_COUNT;
  return string_desc_arr;
}

/* ------------------------------------------------------------------ events */

/* esp_tinyusb 2.2.1 besitzt tud_mount_cb()/tud_umount_cb() selbst. Eigene
 * starke Symbole würden den Link brechen; das Deskriptor-Modul stellt deshalb
 * diese Haken bereit, die der Baustein aus seinem esp_tinyusb-Event-Callback
 * (TINYUSB_EVENT_ATTACHED/DETACHED) ruft. */
void aldes_desc_mount_event(void) {
  aldes_desc_note_mount();
  aldes_log_printf("ENUM SET_CONFIGURATION : device monte");
}

void aldes_desc_umount_event(void) { aldes_log_printf("ENUM device demonte"); }

/* esp_tinyusb ersetzt tud_suspend_cb()/tud_resume_cb() nur, wenn
 * CONFIG_TINYUSB_SUSPEND_CALLBACK/RESUME_CALLBACK gesetzt ist. Wir lassen die
 * Kconfigs aus, damit diese starken Definitionen greifen und wir BUS suspend/
 * resume genau wie auf dem Pico sehen. */
void tud_suspend_cb(bool remote_wakeup_en) {
  aldes_log_printf("BUS suspend (remote_wakeup_en=%d)", remote_wakeup_en ? 1 : 0);
}

void tud_resume_cb(void) { aldes_log_printf("BUS resume"); }

/* Juni hatte keine Vendor-Requests gesehen — seine Instrumentierung konnte sie
 * aber auch nicht sehen. Hier loggen wir sie, bevor wir sie stallen. */
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  (void) rhport;
  if (stage == CONTROL_STAGE_SETUP) {
    aldes_desc_count_vendor++;
    aldes_log_printf(
        "CTRL VENDOR bmRequestType=0x%02X bRequest=0x%02X wValue=0x%04X wIndex=0x%04X wLength=%u "
        "-> stall",
        request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength);
  }
  return false;
}
