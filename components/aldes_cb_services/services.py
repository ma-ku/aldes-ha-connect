
import esphome.codegen as cg
import esphome.config_validation as cv

from components.aldes_cb import AldesCB

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
