import esphome.codegen as cg
from esphome.components import i2c, sensor
from esphome.components.nau7802 import sensor as base_sensor
import esphome.config_validation as cv
from esphome.const import CONF_GAIN, CONF_ID, ICON_SCALE, STATE_CLASS_MEASUREMENT

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["nau7802"]

CONF_GAIN_CALIBRATION = "gain_calibration"
CONF_OFFSET_CALIBRATION = "offset_calibration"
CONF_LDO_VOLTAGE = "ldo_voltage"
CONF_SAMPLES_PER_SECOND = "samples_per_second"

nau7802_local_ns = cg.esphome_ns.namespace("nau7802_local")
NAU7802LocalSensor = nau7802_local_ns.class_(
    "NAU7802LocalSensor", base_sensor.NAU7802Sensor
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        NAU7802LocalSensor,
        icon=ICON_SCALE,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_LDO_VOLTAGE, default="3.0V"): cv.enum(
                base_sensor.LDO, upper=True
            ),
            cv.Optional(CONF_SAMPLES_PER_SECOND, default=10): cv.enum(
                base_sensor.SAMPLES_PER_SECOND, int=True
            ),
            cv.Optional(CONF_GAIN, default=128): cv.enum(base_sensor.GAINS, int=True),
            cv.Optional(CONF_OFFSET_CALIBRATION, default=0): cv.int_range(
                min=-8388608, max=8388607
            ),
            cv.Optional(CONF_GAIN_CALIBRATION, default=1.0): cv.float_range(
                min=0, max=511.9999998807907
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
    await sensor.register_sensor(var, config)

    cg.add(var.set_samples_per_second(config[CONF_SAMPLES_PER_SECOND]))
    cg.add(var.set_ldo_voltage(config[CONF_LDO_VOLTAGE]))
    cg.add(var.set_gain(config[CONF_GAIN]))
    cg.add(var.set_gain_calibration(config[CONF_GAIN_CALIBRATION]))
    cg.add(var.set_offset_calibration(config[CONF_OFFSET_CALIBRATION]))
