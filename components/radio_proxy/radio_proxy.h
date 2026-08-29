#pragma once

#include "esphome/components/media_player/media_player.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <string>

namespace esphome {
namespace radio_proxy {

//
// A media_player that plays nothing itself - it forwards every command to
// another one and, on the way past, hands out the URL. See media_player.py
// for why that is the only way to get at it.
//
// The entity Home Assistant sees is THIS one; the player that owns the audio
// pipeline sits behind it as internal:. Everything on the panel keeps talking
// to the real player directly, so a station change from a button never goes
// through here - only what arrives from outside does, which is exactly the
// case that had no metadata.
//
class RadioProxyMediaPlayer : public Component, public media_player::MediaPlayer {
 public:
  // After the target: it restores its volume in its own setup(), and the
  // mirror below is only worth anything once that has happened.
  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override;
  void dump_config() override;

  // Both delegated rather than tracked. The traits decide which buttons Home
  // Assistant offers, and a copy taken once at setup would go stale the
  // moment the target's own traits depend on something later than that.
  media_player::MediaPlayerTraits get_traits() override { return this->target_->get_traits(); }
  bool is_muted() const override { return this->target_->is_muted(); }

  void set_target(media_player::MediaPlayer *target) { this->target_ = target; }

  Trigger<std::string> *get_media_url_trigger() { return &this->media_url_trigger_; }

 protected:
  void control(const media_player::MediaPlayerCall &call) override;

  media_player::MediaPlayer *target_{nullptr};
  Trigger<std::string> media_url_trigger_;
};

}  // namespace radio_proxy
}  // namespace esphome
