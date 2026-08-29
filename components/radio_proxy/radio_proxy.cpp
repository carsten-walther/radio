#include "radio_proxy.h"

#include "esphome/core/log.h"

namespace esphome {
namespace radio_proxy {

static const char *const TAG = "radio_proxy";

void RadioProxyMediaPlayer::setup() {
  // The mirror. Everything this entity reports is the target's - state,
  // volume and mute - because the target is what is actually playing, and two
  // entities disagreeing about whether the radio is on would be worse than
  // having only one.
  //
  // is_muted() reads through on every call, so it needs nothing here.
  this->target_->add_on_state_callback([this](media_player::MediaPlayerState state) {
    this->state = state;
    this->volume = this->target_->volume;
    this->publish_state();
  });

  // The target has already restored its volume by now (LATE priority above),
  // so this is a real value rather than the 1.0f the base class starts with.
  this->state = this->target_->state;
  this->volume = this->target_->volume;
  this->publish_state();
}

void RadioProxyMediaPlayer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Radio Proxy Media Player:\n"
                "  Target: %s",
                this->target_->get_name().c_str());
}

void RadioProxyMediaPlayer::control(const media_player::MediaPlayerCall &call) {
  const auto &media_url = call.get_media_url();

  // Read from the CALL, not from the player. radio_player reports ANNOUNCING
  // for everything, because it has only an announcement pipeline - but the
  // caller's own flag still says which of the two this was, and it is the
  // only place that distinction survives.
  const bool announcement = call.get_announcement().value_or(false);

  // Rebuilt rather than passed on: MediaPlayerCall holds a pointer to the
  // player it was made for, and control() is what that pointer selects. The
  // fields are copied one by one because an unset field must stay unset -
  // set_volume(0) and "no volume in this call" mean different things.
  auto forwarded = this->target_->make_call();
  if (call.get_command().has_value()) {
    forwarded.set_command(*call.get_command());
  }
  if (media_url.has_value()) {
    forwarded.set_media_url(*media_url);
  }
  if (call.get_volume().has_value()) {
    forwarded.set_volume(*call.get_volume());
  }
  if (call.get_announcement().has_value()) {
    forwarded.set_announcement(*call.get_announcement());
  }
  forwarded.perform();

  // Audio first, metadata second. The trigger is where the ICY poller and the
  // station label are hung, and both can wait the microseconds it takes to
  // hand the URL to the pipeline - which is the allocation that fails first
  // if anything else is competing for the heap.
  //
  // Announcements are skipped on purpose. A TTS message from Home Assistant
  // arrives as a media_url like any stream, and pointing the poller and the
  // station label at it would trade the running station's name for the name
  // of a host that serves one sentence and hangs up - and would leave it
  // there, because nothing restores the old name when the announcement ends.
  if (media_url.has_value() && !announcement) {
    ESP_LOGD(TAG, "Media URL from outside: %s", media_url->c_str());
    this->media_url_trigger_.trigger(*media_url);
  }
}

}  // namespace radio_proxy
}  // namespace esphome
