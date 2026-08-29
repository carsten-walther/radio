from esphome import automation
import esphome.codegen as cg
from esphome.components import media_player
import esphome.config_validation as cv

CODEOWNERS = ["@carsten-walther"]

CONF_TARGET = "target"
CONF_ON_MEDIA_URL = "on_media_url"

radio_proxy_ns = cg.esphome_ns.namespace("radio_proxy")
RadioProxyMediaPlayer = radio_proxy_ns.class_(
    "RadioProxyMediaPlayer",
    media_player.MediaPlayer,
    cg.Component,
)

CONFIG_SCHEMA = media_player.media_player_schema(RadioProxyMediaPlayer).extend(
    {
        # Any media_player, not this platform's own: the point is to sit in
        # front of one that actually plays.
        cv.Required(CONF_TARGET): cv.use_id(media_player.MediaPlayer),
        cv.Optional(CONF_ON_MEDIA_URL): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await media_player.new_media_player(config)
    await cg.register_component(var, config)

    target = await cg.get_variable(config[CONF_TARGET])
    cg.add(var.set_target(target))

    if on_media_url := config.get(CONF_ON_MEDIA_URL):
        await automation.build_automation(
            var.get_media_url_trigger(),
            [(cg.std_string, "x")],
            on_media_url,
        )
