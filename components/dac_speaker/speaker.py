import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import audio, esp32, speaker
from esphome.components.esp32.const import VARIANT_ESP32
from esphome.const import CONF_ID, CONF_PIN

CODEOWNERS = ["@carsten-walther"]
AUTO_LOAD = ["audio", "ring_buffer"]
DEPENDENCIES = ["esp32"]

CONF_BUFFER_DURATION = "buffer_duration"

dac_speaker_ns = cg.esphome_ns.namespace("dac_speaker")
DacSpeaker = dac_speaker_ns.class_("DacSpeaker", speaker.Speaker, cg.Component)


def _declare_stream_limits(config):
    """Tell the pipeline what this speaker accepts.

    Note what this does NOT do: it does not make anything convert. The speaker
    media_player's pipeline has no resampler - these limits are validated
    against the pipeline's declared preferred format and nothing more, so a
    stereo MP3 still arrives here as stereo and the speaker task downmixes it
    itself. Declaring 1..1 channels would therefore be a claim the runtime
    quietly disproves; 1..2 is the truth, and the task handles both.

    16 bits both ways because that is what the pipeline produces; the cut to
    the DAC's 8 is done in the speaker task, not asked for here.

    The sample-rate floor is the DAC-DMA's own: below 19.6 kHz the driver
    rejects the frequency outright with the default digital clock.
    """
    # set_stream_limits' validator mutates the config in place and returns
    # None, so the result must not be passed on as this validator's own.
    audio.set_stream_limits(
        min_bits_per_sample=16,
        max_bits_per_sample=16,
        min_channels=1,
        max_channels=2,
        min_sample_rate=19600,
        max_sample_rate=48000,
    )(config)
    return config


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
    _declare_stream_limits,
    _only_classic_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    cg.add(var.set_dac_channel(1 if config[CONF_PIN] == 26 else 0))
    cg.add(var.set_buffer_duration(config[CONF_BUFFER_DURATION]))
