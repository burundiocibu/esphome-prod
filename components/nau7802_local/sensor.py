import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_GAIN,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_SCALE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]

CONF_GAIN_CALIBRATION = "gain_calibration"
CONF_OFFSET_CALIBRATION = "offset_calibration"
CONF_LDO_VOLTAGE = "ldo_voltage"
CONF_SAMPLES_PER_SECOND = "samples_per_second"

nau7802_local_ns = cg.esphome_ns.namespace("nau7802_local")
NAU7802Base = nau7802_local_ns.class_(
    "NAU7802Base", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)
NAU7802LocalSensor = nau7802_local_ns.class_("NAU7802LocalSensor", NAU7802Base)

NAU7802Gain = nau7802_local_ns.enum("NAU7802Gain")
GAINS = {
    128: NAU7802Gain.NAU7802_GAIN_128,
    64: NAU7802Gain.NAU7802_GAIN_64,
    32: NAU7802Gain.NAU7802_GAIN_32,
    16: NAU7802Gain.NAU7802_GAIN_16,
    8: NAU7802Gain.NAU7802_GAIN_8,
    4: NAU7802Gain.NAU7802_GAIN_4,
    2: NAU7802Gain.NAU7802_GAIN_2,
    1: NAU7802Gain.NAU7802_GAIN_1,
}

NAU7802SPS = nau7802_local_ns.enum("NAU7802SPS")
SAMPLES_PER_SECOND = {
    320: NAU7802SPS.NAU7802_SPS_320,
    80: NAU7802SPS.NAU7802_SPS_80,
    40: NAU7802SPS.NAU7802_SPS_40,
    20: NAU7802SPS.NAU7802_SPS_20,
    10: NAU7802SPS.NAU7802_SPS_10,
}

NAU7802LDO = nau7802_local_ns.enum("NAU7802LDO")
LDO = {
    "2.4V": NAU7802LDO.NAU7802_LDO_2V4,
    "2.7V": NAU7802LDO.NAU7802_LDO_2V7,
    "3.0V": NAU7802LDO.NAU7802_LDO_3V0,
    "3.3V": NAU7802LDO.NAU7802_LDO_3V3,
    "3.6V": NAU7802LDO.NAU7802_LDO_3V6,
    "3.9V": NAU7802LDO.NAU7802_LDO_3V9,
    "4.2V": NAU7802LDO.NAU7802_LDO_4V2,
    "4.5V": NAU7802LDO.NAU7802_LDO_4V5,
    "EXTERNAL": NAU7802LDO.NAU7802_LDO_EXTERNAL,
    "EXT": NAU7802LDO.NAU7802_LDO_EXTERNAL,
}

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
                LDO, upper=True
            ),
            cv.Optional(CONF_SAMPLES_PER_SECOND, default=10): cv.enum(
                SAMPLES_PER_SECOND, int=True
            ),
            cv.Optional(CONF_GAIN, default=128): cv.enum(GAINS, int=True),
            cv.Optional(CONF_OFFSET_CALIBRATION, default=0): cv.int_range(
                min=-8388608, max=8388607
            ),
            cv.Optional(CONF_GAIN_CALIBRATION, default=1.0): cv.float_range(
                min=0, max=511.9999998807907
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                device_class=DEVICE_CLASS_TEMPERATURE,
                accuracy_decimals=1,
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
    await sensor.register_sensor(var, config)

    cg.add(var.set_samples_per_second(config[CONF_SAMPLES_PER_SECOND]))
    cg.add(var.set_ldo_voltage(config[CONF_LDO_VOLTAGE]))
    cg.add(var.set_gain(config[CONF_GAIN]))
    cg.add(var.set_gain_calibration(config[CONF_GAIN_CALIBRATION]))
    cg.add(var.set_offset_calibration(config[CONF_OFFSET_CALIBRATION]))

    if temp_config := config.get(CONF_TEMPERATURE):
        temp_sens = await sensor.new_sensor(temp_config)
        cg.add(var.set_temperature_sensor(temp_sens))
