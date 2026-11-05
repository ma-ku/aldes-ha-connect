"""AldesConnect-Box-Klon auf ESP32-S31 (ESPHome-nativ, ohne Python-Bruecke).

Der Baustein ersetzt firmware/src (Pico W) durch einen eigenen TinyUSB-Klassen-
treiber auf esp_tinyusb 2.2.1. Er stellt dieselbe USB-Schnittstelle bereit
(VID 0x0483 / PID 0x5740, GET_LINE_CODING = 00 x7, stumm ohne SEND) und
uebersetzt das Workbench-Protokoll der Pico-Firmware in ESPHome-Entitaeten:

  sensor       setpoint_k1..4, temperature_k1..4, air_mode, ecs_mode, clock
  text_sensor  log, status, frame, frame_event
  select       air_mode_set, ecs_mode_set
  number       setpoint_k1..4 (16..30 °C, 0,5-°C-Raster)
  button       send_setpoints, send_air, send_ecs, vacation_clear, arm, stop,
               usb_on
  aldes_cb_services   API-Dienste aldes_cb.send/.arm/.stop/.usb_on

YAML:
  * `esp32: variant: ESP32S31`, `framework: type: esp-idf`
  * `logger: hardware_uart: UART0` (der S31-Default USB_SERIAL_JTAG bleibt frei)
  * `api: custom_services: true` fuer die Dienste (services.py)
  * `api: homeassistant_services: true` fuers Frame-Ereignis

Jede Entitaet haengt mit `aldes_cb_id: <ID des aldes_cb-Bausteins>` an genau
einem Baustein.
"""

import esphome.codegen as cg
from esphome.components import esp32
from esphome.components.esp32 import (
    VARIANT_ESP32S3,
    VARIANT_ESP32S31,
    add_idf_component,
    add_idf_sdkconfig_option,
)
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@kikisarem"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor", "text_sensor", "select", "number", "button"]

CONF_ALDES_CB_ID = "aldes_cb_id"
CONF_REPORT_WINDOW = "report_window"
CONF_STATUS_PERIOD = "status_period"

aldes_cb_ns = cg.esphome_ns.namespace("aldes_cb")
AldesCB = aldes_cb_ns.class_("AldesCB", cg.Component)
TextSink = aldes_cb_ns.class_("TextSink")
FloatSink = aldes_cb_ns.class_("FloatSink")
IndexSink = aldes_cb_ns.class_("IndexSink")
NumberSink = aldes_cb_ns.class_("NumberSink")
ButtonSink = aldes_cb_ns.class_("ButtonSink")

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AldesCB),
            # reader.snapshot(): ein 0x21 gilt 25 s (Berichtskadenz ~20 s).
            cv.Optional(CONF_REPORT_WINDOW, default="25s"): cv.positive_time_period_seconds,
            # main.c: STATUS-Zeile alle 10 s.
            cv.Optional(CONF_STATUS_PERIOD, default="10s"): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    esp32.only_on_variant(
        supported=[VARIANT_ESP32S3, VARIANT_ESP32S31],
        msg_prefix="The Aldes ConnectBox Clone",
    ),
)


def _require_esp_idf(config):
    if CORE.using_arduino:
        raise cv.Invalid("aldes_cb needs esp_tinyusb and ultimately framework: type: esp-idf")
    return config


FINAL_VALIDATE_SCHEMA = _require_esp_idf

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_report_window_seconds(int(config[CONF_REPORT_WINDOW].total_seconds)))
    cg.add(var.set_status_period_ms(int(config[CONF_STATUS_PERIOD].total_milliseconds)))

    # esp_tinyusb 2.2.1 mit eigenem Klassentreiber (usbd_app_driver_get_cb).
    # Kein eingebauter CDC-Treiber: CFG_TUD_CDC=0, sonst legt Espressif seine
    # eigenen Deskriptoren an und GET_LINE_CODING liefe ueber deren Code.
    add_idf_component(name="espressif/esp_tinyusb", ref="2.2.1")
    add_idf_sdkconfig_option("CONFIG_TINYUSB_CDC_ENABLED", False)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_CDC_COUNT", 0)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_MSC_ENABLED", False)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_HID_COUNT", 0)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_MIDI_COUNT", 0)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_VENDOR_COUNT", 0)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_DFU_MODE_NONE", True)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_NET_MODE_NONE", True)
    # tud_mount_cb/tud_umount_cb besitzt esp_tinyusb; suspend/resume liefert
    # der Baustein selbst (aldes_desc.c), solange die Kconfigs aus bleiben.
    add_idf_sdkconfig_option("CONFIG_TINYUSB_SUSPEND_CALLBACK", False)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_RESUME_CALLBACK", False)


# ---------------------------------------------------------------- Bausteine

# Basis-Schema der Plattform-Bausteine: sie erben den Baustein NICHT (das wuerde
# die Entitaeten im YAML verschachteln), sondern verweisen mit aldes_cb_id darauf.
PLATFORM_SCHEMA_BASE = cv.Schema(
    {
        cv.Required(CONF_ALDES_CB_ID): cv.use_id(AldesCB),
    }
)


CONF_SINK_ID = "sink_id"


async def new_sink(cls, parent, config):
    """Sink anlegen: `new Cls(<AldesCB-Objekt>)`.

    Der Sink ist ein reines C++-Hilfsobjekt hinter der Entitaet; er bekommt
    seine eigene, automatisch vergebene ID (`sink_id`), nicht die der
    Entitaet — sonst kaeme sich `new_sensor(id)` mit dem Sink in die Quere.
    """
    return cg.new_Pvariable(config[CONF_SINK_ID], parent)


def sink_schema(cls):
    """Pflicht-ID fuer den Sink, mit automatischem Namen je Entitaet."""
    return {cv.GenerateID(CONF_SINK_ID): cv.declare_id(cls)}
