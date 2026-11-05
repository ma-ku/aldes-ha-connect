"""API-Services of the Aldes ConnectBox clone (aldes_cb.send/.arm/.stop/.usb_on).

A component of its onw as ESPHome services names need their own entry format in the 
YAML file. It links to exactly one `aldes_cb` instance and registers its services in 
the native API (Requires: `api: custom_services: true`).
"""

import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components.aldes_cb import AldesCB

CODEOWNERS = ["@kikisarem"]
DEPENDENCIES = ["aldes_cb"]

CONF_ALDES_CB_ID = "aldes_cb_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ALDES_CB_ID): cv.use_id(AldesCB),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ALDES_CB_ID])
    cg.add(parent.enable_services())
