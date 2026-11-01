import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, sensor, uart
from esphome.const import (
    DEVICE_CLASS_RUNNING,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

CODEOWNERS = ["@adin"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor"]

portable_ac_ns = cg.esphome_ns.namespace("portable_ac")
PortableAC = portable_ac_ns.class_("PortableAC", climate.Climate, cg.Component)

CONF_MAINBOARD_UART_ID = "mainboard_uart_id"
CONF_MAINBOARD_RX_UART_ID = "mainboard_rx_uart_id"
CONF_PANEL_UART_ID = "panel_uart_id"
CONF_PANEL_TX_UART_ID = "panel_tx_uart_id"
CONF_HOLD_OVERRIDE = "hold_override"
CONF_START_IN_PASSTHROUGH = "start_in_passthrough"
CONF_CURRENT_TEMPERATURE_SENSOR = "current_temperature_sensor"
CONF_INVERTER_LEVEL = "inverter_level"
CONF_COMPRESSOR_RUNNING = "compressor_running"
CONF_ANTI_FREEZE = "anti_freeze"
CONF_OVERRIDE_ACTIVE = "override_active"

# Raw status bytes from the main board frame (byte offset in the 28-byte frame).
# Scales are not decoded yet, so these are published as raw 0-255 values.
RAW_SENSORS = {
    "sensor_11_raw": 11,  # rises while cooling (condenser / exhaust?)
    "coil_temp_raw": 12,  # falls while cooling (evaporator coil)
    "sensor_18_raw": 18,
    "sensor_21_raw": 21,
    "sensor_22_raw": 22,
    "status_23_raw": 23,  # ramps with load, winds down after power-off
    "status_24_raw": 24,
}

RAW_SENSOR_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

CONFIG_SCHEMA = (
    climate.climate_schema(PortableAC)
    .extend(
        {
            cv.Required(CONF_MAINBOARD_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_PANEL_UART_ID): cv.use_id(uart.UARTComponent),
            # Optional separate buses, e.g. hardware TX on one pin and software RX on another
            cv.Optional(CONF_MAINBOARD_RX_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_PANEL_TX_UART_ID): cv.use_id(uart.UARTComponent),
            # true  = keep rewriting panel frames until someone presses a panel button
            # false = send the command once, then go back to pure pass-through
            cv.Optional(CONF_HOLD_OVERRIDE, default=True): cv.boolean,
            cv.Optional(CONF_START_IN_PASSTHROUGH, default=True): cv.boolean,
            cv.Optional(CONF_CURRENT_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_INVERTER_LEVEL): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                icon="mdi:sine-wave",
            ),
            cv.Optional(CONF_COMPRESSOR_RUNNING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING,
                icon="mdi:heat-pump",
            ),
            cv.Optional(CONF_ANTI_FREEZE): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:snowflake-alert",
            ),
            cv.Optional(CONF_OVERRIDE_ACTIVE): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:swap-horizontal",
            ),
            **{cv.Optional(k): RAW_SENSOR_SCHEMA for k in RAW_SENSORS},
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    cg.add(var.set_mainboard_uart(await cg.get_variable(config[CONF_MAINBOARD_UART_ID])))
    cg.add(var.set_panel_uart(await cg.get_variable(config[CONF_PANEL_UART_ID])))
    if CONF_MAINBOARD_RX_UART_ID in config:
        cg.add(var.set_mainboard_rx_uart(await cg.get_variable(config[CONF_MAINBOARD_RX_UART_ID])))
    if CONF_PANEL_TX_UART_ID in config:
        cg.add(var.set_panel_tx_uart(await cg.get_variable(config[CONF_PANEL_TX_UART_ID])))
    cg.add(var.set_hold_override(config[CONF_HOLD_OVERRIDE]))
    cg.add(var.set_passthrough(config[CONF_START_IN_PASSTHROUGH]))

    if CONF_CURRENT_TEMPERATURE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_CURRENT_TEMPERATURE_SENSOR])
        cg.add(var.set_current_temperature_sensor(sens))
    if CONF_INVERTER_LEVEL in config:
        cg.add(var.set_inverter_level_sensor(await sensor.new_sensor(config[CONF_INVERTER_LEVEL])))
    if CONF_COMPRESSOR_RUNNING in config:
        cg.add(var.set_compressor_sensor(
            await binary_sensor.new_binary_sensor(config[CONF_COMPRESSOR_RUNNING])))
    if CONF_ANTI_FREEZE in config:
        cg.add(var.set_anti_freeze_sensor(
            await binary_sensor.new_binary_sensor(config[CONF_ANTI_FREEZE])))
    if CONF_OVERRIDE_ACTIVE in config:
        cg.add(var.set_override_active_sensor(
            await binary_sensor.new_binary_sensor(config[CONF_OVERRIDE_ACTIVE])))
    for key, offset in RAW_SENSORS.items():
        if key in config:
            cg.add(var.set_raw_sensor(offset, await sensor.new_sensor(config[key])))
