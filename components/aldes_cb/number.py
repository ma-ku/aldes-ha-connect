"""number: setpoint_k1..4 — Bedienelemente wie in der Pico-Weboberflaeche.

Der gewaehlte Wert wird nur gemerkt; gesendet wird er mit dem Knopf
"Send setpoints" (vier Frames, ein Versuch je Frame). Bereich und Raster
stammen aus pico.SETPOINT_MIN/MAX und frame_setpoint(): 16..30 °C in 0,5 °C.

`aldes_cb::ControlNumber` ist die Nummer, deren `control(value)` den Wert direkt
im Sink des Bausteins merkt — ohne Textumweg und ohne Zusatz-Lambda im YAML.
"""

import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CELSIUS,
)

from . import CONF_ALDES_CB_ID, PLATFORM_SCHEMA_BASE, AldesCB, NumberSink, new_sink, sink_schema

DEPENDENCIES = ["aldes_cb"]

CONF_SETPOINT_K1 = "setpoint_k1"
CONF_SETPOINT_K2 = "setpoint_k2"
CONF_SETPOINT_K3 = "setpoint_k3"
CONF_SETPOINT_K4 = "setpoint_k4"

SETPOINT_MIN = 16.0
SETPOINT_MAX = 30.0
SETPOINT_STEP = 0.5

NUMBERS = {
    CONF_SETPOINT_K1: 1,
    CONF_SETPOINT_K2: 2,
    CONF_SETPOINT_K3: 3,
    CONF_SETPOINT_K4: 4,
}

aldes_cb_ns = cg.esphome_ns.namespace("aldes_cb")
CONTROL_NUMBER = aldes_cb_ns.class_("ControlNumber", number.Number)

# Die Plattform selbst ist kein Geraet: die ID bleibt leer, die Entitaeten
# tragen ihre eigenen IDs (wie ld2410/sensor.py).
PLATFORM_SCHEMA = PLATFORM_SCHEMA_BASE.extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase)}
)

CONFIG_SCHEMA = PLATFORM_SCHEMA.extend(
    {
        cv.Optional(key): number.number_schema(
            CONTROL_NUMBER,
            unit_of_measurement=UNIT_CELSIUS,
            device_class=DEVICE_CLASS_TEMPERATURE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:thermostat",
        ).extend(sink_schema(NumberSink))
        for key in NUMBERS
    }
)


async def _wire(config, key, zone):
    parent = await cg.get_variable(config[CONF_ALDES_CB_ID])
    sub = config[key]
    sink = await new_sink(NumberSink, parent, sub)
    var = await number.new_number(
        sub,
        min_value=SETPOINT_MIN,
        max_value=SETPOINT_MAX,
        step=SETPOINT_STEP,
    )
    cg.add(
        var.set_number_control_callback(
            cg.RawExpression(f"[{sink}](float value) {{ {sink}->control(value); }}")
        )
    )
    cg.add(parent.set_setpoint_number(zone, sink))


async def to_code(config):
    for key, zone in NUMBERS.items():
        if key in config:
            await _wire(config, key, zone)
