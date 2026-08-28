"""icy_info - stream title and bitrate from SHOUTcast/Icecast ICY metadata.

ESPHome's audio reader plays the stream but never asks for metadata: nothing
in the tree sends `Icy-MetaData: 1` or parses the in-band title blocks, so
the "what is playing" question has no built-in answer.

This component answers it with its own short-lived HTTP connection. On each
poll it requests the stream WITH metadata, reads the response headers
(`icy-br` is the bitrate, `icy-metaint` the audio-bytes-between-metadata
interval), discards one interval's worth of audio, parses the first
`StreamTitle='...';` block and hangs up. At a typical metaint of 8-16 kB that
is one to two seconds of stream data per poll - a deliberate trade against
holding a second permanent connection open on a board with ~100 kB of free
heap.

Plain http:// only. TLS would cost ~40 kB of heap per handshake, which this
board does not have while the audio pipeline runs; an https URL is skipped
with a log line rather than attempted.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_EMPTY,
)

CODEOWNERS = ["@carsten-walther"]
AUTO_LOAD = ["sensor", "text_sensor"]
DEPENDENCIES = ["esp32", "network"]

CONF_STREAM_TITLE = "stream_title"
CONF_BITRATE = "bitrate"
CONF_STATION_NAME = "station_name"

icy_info_ns = cg.esphome_ns.namespace("icy_info")
IcyInfo = icy_info_ns.class_("IcyInfo", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(IcyInfo),
        cv.Optional(CONF_STREAM_TITLE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_STATION_NAME): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_BITRATE): sensor.sensor_schema(
            unit_of_measurement="kbit/s",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if title_config := config.get(CONF_STREAM_TITLE):
        sens = await text_sensor.new_text_sensor(title_config)
        cg.add(var.set_title_sensor(sens))

    if name_config := config.get(CONF_STATION_NAME):
        sens = await text_sensor.new_text_sensor(name_config)
        cg.add(var.set_name_sensor(sens))

    if bitrate_config := config.get(CONF_BITRATE):
        sens = await sensor.new_sensor(bitrate_config)
        cg.add(var.set_bitrate_sensor(sens))
