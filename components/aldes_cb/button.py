"""button: die Knopfleiste der Pico-Weboberflaeche.

  send_setpoints   vier Sollwert-Frames aus den number-Entitaeten
  send_air         Air-Modus aus `select.air_mode_set`
  send_ecs         ECS-Modus aus `select.ecs_mode_set`
  vacation_clear   Urlaubsfelder 5..12 auf 00 (frame_vacation_clear)
  arm              Freigabe fuer genau einen Versuch, 30 s
  stop             laufende Uebertragung abbrechen (USB abgemeldet)
  usb_on           Controller neu anmelden (kein Replay des Abbruchs)

Jeder Knopf ist ein `aldes_cb::ButtonSink`; der eigentliche Aufruf haengt am
`press()` des Sinks und laeuft im Main-Loop, nicht im API-Task.
"""

import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import (
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    ENTITY_CATEGORY_CONFIG,
)

from . import CONF_ALDES_CB_ID, PLATFORM_SCHEMA_BASE, AldesCB, ButtonSink, new_sink, sink_schema

DEPENDENCIES = ["aldes_cb"]

CONF_SEND_SETPOINTS = "send_setpoints"
CONF_SEND_AIR = "send_air"
CONF_SEND_ECS = "send_ecs"
CONF_VACATION_CLEAR = "vacation_clear"
CONF_ARM = "arm"
CONF_STOP = "stop"
CONF_USB_ON = "usb_on"

aldes_cb_ns = cg.esphome_ns.namespace("aldes_cb")

# ControlButton: beim Druecken laeuft press_action() -> Sink des Bausteins.
CONTROL_BUTTON = aldes_cb_ns.class_("ControlButton", button.Button)

# key -> (Setter am Baustein, Vorschlag Icon)
BUTTONS = {
    CONF_SEND_SETPOINTS: ("set_send_setpoints_button", "mdi:thermometer-check"),
    CONF_SEND_AIR: ("set_send_air_button", "mdi:air-conditioner"),
    CONF_SEND_ECS: ("set_send_ecs_button", "mdi:water-boiler"),
    CONF_VACATION_CLEAR: ("set_vacation_clear_button", "mdi:palmtree"),
    CONF_ARM: ("set_arm_button", "mdi:lock-open-variant"),
    CONF_STOP: ("set_stop_button", "mdi:stop"),
    CONF_USB_ON: ("set_usb_on_button", "mdi:usb"),
}


def _schema(icon):
    return (
        button.button_schema(CONTROL_BUTTON, entity_category=ENTITY_CATEGORY_CONFIG)
        .extend(sink_schema(ButtonSink))
        .extend({cv.Optional(CONF_ICON, default=icon): cv.icon})
    )


# Die Plattform selbst ist kein Geraet: die ID bleibt leer, die Entitaeten
# tragen ihre eigenen IDs (wie ld2410/sensor.py).
PLATFORM_SCHEMA = PLATFORM_SCHEMA_BASE.extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase)}
)

CONFIG_SCHEMA = PLATFORM_SCHEMA.extend(
    {cv.Optional(key): _schema(icon) for key, (_setter, icon) in BUTTONS.items()}
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ALDES_CB_ID])
    for key, (setter, _icon) in BUTTONS.items():
        if key not in config:
            continue
        sub = config[key]
        sink = await new_sink(ButtonSink, parent, sub)
        var = await button.new_button(sub)
        cg.add(
            var.set_press_callback(
                cg.RawExpression(f"[{sink}]() {{ {sink}->press(); }}")
            )
        )
        cg.add(getattr(parent, setter)(sink))
