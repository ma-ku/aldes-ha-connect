/* Klon der AldesConnect Box — USB-Deskriptoren (Port der Pico-Firmware).
 *
 * Dieses Header ist die einzige Schnittstelle zwischen dem Deskriptor-Modul und
 * dem Rest des Bausteins. Die Bytes selbst stehen in aldes_desc.c und werden
 * dort Zeile für Zeile aus der öffentlichen Capture übernommen.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tusb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Zähler wie auf dem Pico (dort aus netlog-Sicht "ENUM GET_DESCRIPTOR ...").
 * esp_tinyusb 2.2.1 liefert Device-/Konfigurationsdeskriptoren aus seiner
 * eigenen Callback-Schicht ohne Hook; gezählt werden die Ereignisse, die es
 * gibt: mount, String-Deskriptoren, Vendor-Requests. */
extern uint32_t aldes_desc_count_device;
extern uint32_t aldes_desc_count_config;
extern uint32_t aldes_desc_count_string;
extern uint32_t aldes_desc_count_vendor;

/* Zentrale Log-Funktion des Bausteins — dieselben Zeilenformate wie auf dem
 * Pico ("[%8lu] ..."), nur über den ESPHome-Logger statt den TCP-Ring. */
void aldes_log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Rohtabellen (Referenz und Host-Test-Grundlage). */
uint8_t const *aldes_desc_device_bytes(size_t *len);
void aldes_desc_device_struct_bytes(uint8_t *out /* 18 Bytes */);
size_t aldes_desc_configuration_len(void);
uint8_t const *aldes_desc_configuration(void);
uint8_t const *aldes_desc_hs_configuration(void);
uint8_t const *aldes_desc_fs_configuration(void);
size_t aldes_desc_qualifier_len(void);
uint8_t const *aldes_desc_qualifier(void);
const char *const *aldes_desc_strings(size_t *count);

/* Gefüllte esp_tinyusb-Deskriptorkonfiguration (device, qualifier, strings,
 * full_speed_config, high_speed_config). Zigartige Puffer sind statisch, der
 * Zeiger bleibt für die Lebensdauer des Stacks gültig. */
void aldes_desc_get_tinyusb_config(const tusb_desc_device_t **device, const tusb_desc_device_qualifier_t **qualifier,
                                   const char ***strings, int *string_count);

/* String-Deskriptor im Pico-Format (inkl. der Zeile "ENUM GET_DESCRIPTOR
 * string index=…"). Der Stack benutzt die esp_tinyusb-Variante; dieser
 * Callback dient dem Host-Test und ist bereit, falls ein späterer esp_tinyusb
 * einen Hook anbietet. */
uint16_t const *aldes_desc_string_cb(uint8_t index, uint16_t langid);

/* Wird von tud_mount_cb() gerufen: zählt Mounts und schreibt die Pico-Zeile
 * "ENUM SET_CONFIGURATION : device monte". */
void aldes_desc_note_mount(void);

/* ------------------------------------------------------------------ events */
/* Diese Callbacks liefert der Baustein selbst (starke Symbole):
 *  - tud_mount_cb / tud_umount_cb gehören esp_tinyusb 2.2.1 (starke Symbole).
 *    aldes_desc_mount_event()/aldes_desc_umount_event() schreiben dieselben
 *    Zeilen und werden aus dem esp_tinyusb-Event-Callback des Bausteins
 *    gerufen (TINYUSB_EVENT_ATTACHED/DETACHED).
 *  - tud_suspend_cb / tud_resume_cb: schwach in TinyUSB, stark hier, solange
 *    CONFIG_TINYUSB_SUSPEND_CALLBACK/RESUME_CALLBACK aus bleiben.
 *  - tud_vendor_control_xfer_cb: Vendor-Requests loggen und stallen. */
void aldes_desc_mount_event(void);
void aldes_desc_umount_event(void);

#ifdef __cplusplus
}
#endif
