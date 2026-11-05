"""select: air_mode_set, ecs_mode_set — worauf "Air senden"/"ECS senden" zugreift.

ESPHome-select-Komponenten liefern Optionsnamen; der Baustein arbeitet mit dem
Rohwert (Index). Beide Richtungen laufen im Main-Loop:

  PAC -> select   IndexSink-Callback -> ControlSelect::publish_index()
  HA  -> PAC      ControlSelect::control(size_t) -> sink.control(index)

`aldes_cb::ControlSelect` haengt an `control(size_t)` den Sink an, damit die
Umlaute der Optionsliste nicht ueber einen Textvergleich zurueckgefunden werden
muessen. Gesendet wird erst mit dem Knopf "Send air"/"Send ECS"; bis dahin
merkt sich der Baustein die Auswahl.
"""

import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG
from esphome.helpers import cpp_string_escape

from . import CONF_ALDES_CB_ID, PLATFORM_SCHEMA_BASE, AldesCB, IndexSink, new_sink, sink_schema

DEPENDENCIES = ["aldes_cb"]

CONF_AIR_MODE = "air_mode_set"
CONF_ECS_MODE = "ecs_mode_set"

# Texte wie pico.AIR_MODES / pico.ECS_MODES, Reihenfolge = Rohwert.
AIR_MODES = [
    "Arrêt",
    "Chauffage · Confort",
    "Chauffage · Éco",
    "Chauffage · Programme A",
    "Chauffage · Programme B",
    "Clim · Confort",
    "Clim · Boost",
    "Clim · Programme C",
    "Clim · Programme D",
]
ECS_MODES = ["Arrêt", "Marche", "Boost"]

aldes_cb_ns = cg.esphome_ns.namespace("aldes_cb")
CONTROL_SELECT = aldes_cb_ns.class_("ControlSelect", select.Select)

# Die Plattform selbst ist kein Geraet: die ID bleibt leer, die Entitaeten
# tragen ihre eigenen IDs (wie ld2410/sensor.py).
PLATFORM_SCHEMA = PLATFORM_SCHEMA_BASE.extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase)}
)

CONFIG_SCHEMA = PLATFORM_SCHEMA.extend(
    {
        cv.Optional(CONF_AIR_MODE): select.select_schema(
            CONTROL_SELECT, entity_category=ENTITY_CATEGORY_CONFIG
        ).extend(sink_schema(IndexSink)),
        cv.Optional(CONF_ECS_MODE): select.select_schema(
            CONTROL_SELECT, entity_category=ENTITY_CATEGORY_CONFIG
        ).extend(sink_schema(IndexSink)),
    }
)


async def _wire(config, key, setter, names):
    parent = await cg.get_variable(config[CONF_ALDES_CB_ID])
    sub = config[key]
    sink = await new_sink(IndexSink, parent, sub)
    var = await select.new_select(sub, options=names)
    cg.add(
        sink.set_options(
            cg.RawExpression(
                "std::vector<std::string>{"
                + ", ".join(cpp_string_escape(n) for n in names)
                + "}"
            )
        )
    )
    # PAC -> HA: Index (NAN = unbekannt) auf den statischen Text abbilden.
    cg.add(
        sink.add_on_state_callback(
            cg.RawExpression(f"[{var}](float value) {{ {var}->publish_index(value); }}")
        )
    )
    # HA -> PAC (control(size_t) der Unterklasse) landet als Rohwert im Sink.
    cg.add(
        var.set_sink_index_callback(
            cg.RawExpression(f"[{sink}](size_t index) {{ {sink}->control(index); }}")
        )
    )
    cg.add(getattr(parent, setter)(sink))


async def to_code(config):
    if CONF_AIR_MODE in config:
        await _wire(config, CONF_AIR_MODE, "set_air_select", AIR_MODES)
    if CONF_ECS_MODE in config:
        await _wire(config, CONF_ECS_MODE, "set_ecs_select", ECS_MODES)
