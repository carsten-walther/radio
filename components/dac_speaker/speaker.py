import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import esp32, speaker
from esphome.components.esp32.const import VARIANT_ESP32
from esphome.const import CONF_ID, CONF_PIN

CODEOWNERS = ["@carsten-walther"]
AUTO_LOAD = ["audio", "ring_buffer"]
DEPENDENCIES = ["esp32"]

CONF_BUFFER_DURATION = "buffer_duration"

dac_speaker_ns = cg.esphome_ns.namespace("dac_speaker")
DacSpeaker = dac_speaker_ns.class_("DacSpeaker", speaker.Speaker, cg.Component)


def _only_classic_esp32(config):
    variant = esp32.get_esp32_variant()
    if variant != VARIANT_ESP32:
        raise cv.Invalid(
            f"The internal DAC exists only on the classic ESP32, not on {variant}"
        )
    return config


CONFIG_SCHEMA = cv.All(
    speaker.SPEAKER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(DacSpeaker),
            # The two DAC pins are fixed in silicon: GPIO25 is channel 0,
            # GPIO26 is channel 1. Anything else cannot reach the DAC.
            cv.Required(CONF_PIN): cv.All(
                pins.internal_gpio_output_pin_number, cv.one_of(25, 26, int=True)
            ),
            cv.Optional(
                CONF_BUFFER_DURATION, default="100ms"
            ): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _only_classic_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    cg.add(var.set_dac_channel(1 if config[CONF_PIN] == 26 else 0))
    cg.add(var.set_buffer_duration(config[CONF_BUFFER_DURATION]))
