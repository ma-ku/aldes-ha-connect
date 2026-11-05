"""text_sensor: log, status, frame, frame_event.

  log         jede Pico-Logzeile ("[       0] OUT xfer #1, 64 octets", ...)
  status      die 10-s-Zeile "STATUS mounted=1 | GET_LC=... | ..." wie main.c
  frame       die letzte Zeile "FRAME #n opcode=0x21 len=90 somme=OK ..."
  frame_event die letzte Trame als JSON {"line":..,"opcode":..,"hex":..}

Das Frame-Ereignis geht in Home Assistant zusaetzlich als Ereignis
`esphome.aldes_frame` ein — mit `api: homeassistant_services: true`:

    text_sensor:
      - platform: aldes_cb
        aldes_cb_id: cb
        frame_event:
          name: "Aldes trame"
          on_value:
            - homeassistant.event:
                event: esphome.aldes_frame
                data:
                  value: !lambda "return x;"

Ohne text_sensor bleiben die Zeilen im ESPHome-Log sichtbar (TextSink haengt
sie dann an log_line_()).
"""

import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CONF_ALDES_CB_ID, PLATFORM_SCHEMA_BASE, AldesCB, TextSink, new_sink, sink_schema

DEPENDENCIES = ["aldes_cb"]

CONF_LOG = "log"
CONF_STATUS = "status"
CONF_FRAME = "frame"
CONF_FRAME_EVENT = "frame_event"

# key -> (Setter am Baustein, Icon, Vorgabe entity_category)
TEXTS = {
    CONF_LOG: ("set_log_sink", "mdi:text-box-outline", ENTITY_CATEGORY_DIAGNOSTIC),
    CONF_STATUS: (
        "set_status_text",
        "mdi:information-outline",
        ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_FRAME: ("set_frame_text", "mdi:code-braces", ENTITY_CATEGORY_DIAGNOSTIC),
    # Frames sollen im HA-Verlauf auftauchen, also keine Diagnose-Entitaet.
    CONF_FRAME_EVENT: ("set_frame_event_text", "mdi:code-json", None),
}


def _schema(icon, category):
    extra = {cv.Optional(CONF_ICON, default=icon): cv.icon}
    if category is not None:
        extra[cv.Optional(CONF_ENTITY_CATEGORY, default=category)] = cv.entity_category
    return text_sensor.text_sensor_schema(
        text_sensor.text_sensor_ns.class_("TextSensor")
    ).extend(sink_schema(TextSink)).extend(extra)


# Die Plattform selbst ist kein Geraet: die ID bleibt leer, die Entitaeten
# tragen ihre eigenen IDs (wie ld2410/sensor.py).
PLATFORM_SCHEMA = PLATFORM_SCHEMA_BASE.extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase)}
)

CONFIG_SCHEMA = PLATFORM_SCHEMA.extend(
    {
        cv.Optional(key): _schema(icon, category)
        for key, (_setter, icon, category) in TEXTS.items()
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ALDES_CB_ID])
    for key, (setter, _icon, _category) in TEXTS.items():
        if key not in config:
            continue
        sub = config[key]
        sink = await new_sink(TextSink, parent, sub)
        cg.add(getattr(parent, setter)(sink))
        var = await text_sensor.new_text_sensor(sub)
        cg.add(
            sink.add_on_state_callback(
                cg.RawExpression(
                    f"[{var}](std::string value) {{ {var}->publish_state(value); }}"
                )
            )
        )
