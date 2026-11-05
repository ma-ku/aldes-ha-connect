"""generierte Sensoren: setpoint_k1..4, temperature_k1..4, air_mode, ecs_mode,
clock (Unix-Zeit der PAC-Uhr).

Werte kommen aus dem lokalen 0x21-Bericht. Nach `report_window` (Pico:
Snapshot-Grenze 25 s) verfaellt der Wert; Home Assistant sieht dann
`unavailable`, statt einen alten Messwert weiterzufuehren — wie
reader.snapshot() in der Python-Variante.

Jeder Eintrag ist ein vollwertiger ESPHome-Sensor (`sensor_schema()`): `name`,
`id`, `internal`, `entity_category` usw. gelten wie ueberall. Nur die
Vorgaben je Slot (Einheit, Genauigkeit, Icon) kommen hier dazu.
"""

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_DEVICE_CLASS,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_STATE_CLASS,
    CONF_ID,
    CONF_UNIT_OF_MEASUREMENT,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_SECOND,
)

from . import CONF_ALDES_CB_ID, PLATFORM_SCHEMA_BASE, AldesCB, FloatSink, new_sink, sink_schema

DEPENDENCIES = ["aldes_cb"]

CONF_SETPOINT_K1 = "setpoint_k1"
CONF_SETPOINT_K2 = "setpoint_k2"
CONF_SETPOINT_K3 = "setpoint_k3"
CONF_SETPOINT_K4 = "setpoint_k4"
CONF_TEMPERATURE_K1 = "temperature_k1"
CONF_TEMPERATURE_K2 = "temperature_k2"
CONF_TEMPERATURE_K3 = "temperature_k3"
CONF_TEMPERATURE_K4 = "temperature_k4"
CONF_AIR_MODE = "air_mode"
CONF_ECS_MODE = "ecs_mode"
CONF_CLOCK = "clock"

# Schluessel -> (AldesCB-Slot, Vorschlag Icon)
SENSORS = {
    CONF_SETPOINT_K1: (0, "mdi:thermostat"),
    CONF_SETPOINT_K2: (1, "mdi:thermostat"),
    CONF_SETPOINT_K3: (2, "mdi:thermostat"),
    CONF_SETPOINT_K4: (3, "mdi:thermostat"),
    CONF_TEMPERATURE_K1: (4, "mdi:thermometer"),
    CONF_TEMPERATURE_K2: (5, "mdi:thermometer"),
    CONF_TEMPERATURE_K3: (6, "mdi:thermometer"),
    CONF_TEMPERATURE_K4: (7, "mdi:thermometer"),
    CONF_AIR_MODE: (8, "mdi:air-conditioner"),
    CONF_ECS_MODE: (9, "mdi:water-boiler"),
    CONF_CLOCK: (10, "mdi:clock-outline"),
}


def _schema(slot, icon):
    # sensor_schema() bringt name/id/internal/... mit; die Vorgaben hier
    # ersetzen nur Genauigkeit, Einheit und Icon.
    defaults = {
        CONF_ICON: icon,
        CONF_ACCURACY_DECIMALS: 0 if slot >= 8 else 2,
    }
    if slot < 8:
        defaults.update(
            {
                CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
                CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
                # Enum-Wert statt des YAML-Strings: er landet woertlich im
                # C++, und set_state_class() will den Typ.
                #valider Wert direkt, validate_state_class() ist idempotent
                CONF_STATE_CLASS: "measurement",
            }
        )
    elif slot == 10:
        defaults[CONF_UNIT_OF_MEASUREMENT] = UNIT_SECOND
    else:
        defaults[CONF_UNIT_OF_MEASUREMENT] = UNIT_EMPTY

    extra = {cv.Optional(key, default=value): _VALIDATORS[key] for key, value in defaults.items()}
    # Rohmodus/Ecs-Modus sind Diagnose, Temperaturen normale Entitaeten.
    if slot >= 8:
        extra[cv.Optional(CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_DIAGNOSTIC)] = (
            cv.entity_category
        )
    return sensor.sensor_schema(
        sensor.sensor_ns.class_("Sensor")
    ).extend(sink_schema(FloatSink)).extend(extra)


# Validatoren der Vorgaben: cv.string reicht, weil die Werte unten schon die
# endgueltigen Texte sind; state_class behaelt seinen Enum-Wert.
_VALIDATORS = {
    CONF_ICON: cv.icon,
    CONF_ACCURACY_DECIMALS: cv.int_,
    CONF_UNIT_OF_MEASUREMENT: cv.string,
    CONF_DEVICE_CLASS: cv.string,
    CONF_STATE_CLASS: sensor.validate_state_class,
}

# Die Plattform selbst ist kein Geraet: die ID bleibt leer, die Entitaeten
# tragen ihre eigenen IDs (wie ld2410/sensor.py).
PLATFORM_SCHEMA = PLATFORM_SCHEMA_BASE.extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase)}
)

CONFIG_SCHEMA = PLATFORM_SCHEMA.extend(
    {cv.Optional(key): _schema(slot, icon) for key, (slot, icon) in SENSORS.items()}
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ALDES_CB_ID])
    for key, (slot, _icon) in SENSORS.items():
        if key not in config:
            continue
        sub = config[key]
        sink = await new_sink(FloatSink, parent, sub)
        # Slot sofort belegen: der Baustein publiziert auch ohne Sensor.
        cg.add(parent.set_report_sensor(slot, sink))
        var = await sensor.new_sensor(sub)
        cg.add(
            sink.add_on_state_callback(
                cg.RawExpression(
                    f"[{var}](float value) {{ {var}->publish_state(value); }}"
                )
            )
        )
