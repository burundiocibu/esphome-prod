import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["openthread"]

openthread_rssi_ns = cg.esphome_ns.namespace("openthread_rssi")
OpenThreadNeighborLog = openthread_rssi_ns.class_(
    "OpenThreadNeighborLog", cg.PollingComponent
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OpenThreadNeighborLog),
    }
).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
