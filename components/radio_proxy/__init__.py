"""radio_proxy - a media_player that forwards to another one and publishes the URL it was given.

ESPHome never hands the stream URL back out. `MediaPlayerCall::get_media_url()`
exists, but the call is a temporary: the API connection builds one, calls
perform(), and it is gone (api_connection.cpp, on_media_player_command_request).
`SpeakerMediaPlayer` does not keep the URL anywhere readable - and it is
`final`, so control() cannot be overridden by subclassing it either. The only
place above `ESP_LOGV` where the URL exists at all is inside that one call.

So this platform puts an entity of its own in front: Home Assistant talks to
the proxy, the proxy reads the URL out of the call it receives, fires
`on_media_url`, and forwards the call unchanged to the real player. That real
player stays exactly as configured and becomes `internal: true`, so there is
still one radio in Home Assistant.

Everything the panel does keeps addressing the real player directly, so
`on_media_url` fires for outside commands only - which is the case that had no
station name, title or bitrate, because nothing had told icy_info what to poll.
"""

CODEOWNERS = ["@carsten-walther"]
