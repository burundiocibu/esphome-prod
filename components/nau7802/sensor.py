import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_VOLTAGE,
    CONF_GAIN,
    CONF_ID,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
)
nau7802_ns = cg.esphome_ns.namespace("nau7802")

DEPENDENCIES = ["i2c"]

NAU7802Gain = nau7802_ns.enum("NAU7802Gain")
NAU7802_GAIN = {
    "128": NAU7802Gain.NAU7802_GAIN_128,
    "64": NAU7802Gain.NAU7802_GAIN_64,
    "32": NAU7802Gain.NAU7802_GAIN_32,
    "16": NAU7802Gain.NAU7802_GAIN_16,
    "8": NAU7802Gain.NAU7802_GAIN_8,
    "4": NAU7802Gain.NAU7802_GAIN_4,
    "2": NAU7802Gain.NAU7802_GAIN_2,
    "1": NAU7802Gain.NAU7802_GAIN_1,
}

def validate_gain(value):
    if isinstance(value, float):
        value = f"{value:0.03f}"
    elif not isinstance(value, str):
        raise cv.Invalid(f'invalid gain "{value}"')

    return cv.enum(GAIN)(value)

NAU7802Component = nau7802_ns.class_(
    "NAU7802Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NAU7802Component),
            cv.Required(CONF_GAIN): cv.enum(NAU7802_GAIN, upper=True),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x2A))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    if CONF_VOLTAGE in config:
        conf = config[CONF_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_voltage_sensor(sens))
