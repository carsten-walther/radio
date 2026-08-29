#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32_VARIANT_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <driver/dac_continuous.h>

#include <atomic>
#include <memory>

#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"

namespace esphome {
namespace dac_speaker {

//
// A speaker on the ESP32's internal 8-bit DAC, via IDF 5's dac_continuous
// DMA driver - the replacement for the legacy I2S built-in DAC mode that
// ESPHome's i2s_audio speaker no longer carries.
//
// The shape follows I2SAudioSpeakerBase: play() feeds a ring buffer, a
// FreeRTOS task drains it into the driver, stop() and finish() signal the
// task through an event group. The task owns the DAC handle and the ring
// buffer for exactly as long as it runs, so a stopped speaker holds no
// audio memory at all.
//
// What the task adds over the I2S version is conversion: the pipeline
// delivers 16-bit signed frames in whatever channel count it was configured
// for, and the DAC wants 8-bit unsigned mono. Downmix, software volume and
// the bit-depth cut all happen in one pass per chunk.
//
class DacSpeaker : public speaker::Speaker, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  /// 0 = GPIO25, 1 = GPIO26. Fixed by silicon, validated in speaker.py.
  void set_dac_channel(uint8_t channel) { this->dac_channel_ = channel; }
  void set_buffer_duration(uint32_t buffer_duration_ms) { this->buffer_duration_ms_ = buffer_duration_ms; }

  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  size_t play(const uint8_t *data, size_t length) override { return this->play(data, length, 0); }

  //
  // The meter tap. Returns the loudest chunk RMS seen on one channel since
  // the last call, 0.0f..1.0f of full scale, and clears it - so a caller on a
  // 100ms interval gets the loudest of the ~8 chunks that fell in between
  // rather than whichever one happened to be last.
  //
  // Measured BEFORE the volume multiply on purpose. This is the level of what
  // the station is sending, not of what the amplifier is doing with it: a
  // meter that empties when the knob goes down says nothing about the
  // programme, and at half volume it would never reach past half scale.
  //
  // channel: 0 = left, 1 = right. Mono input reports the same value on both.
  //
  float pop_level(uint8_t channel);

  void start() override;
  void stop() override;
  void finish() override;

  bool has_buffered_data() const override;

 protected:
  static void speaker_task(void *params);
  /// Keeps the louder of this chunk's RMS and whatever has not been read yet.
  void report_level_(uint8_t channel, uint64_t sum_squares, size_t frames);
  void run_();
  /// Creates the DAC on first use and keeps it across stop/start, rebuilding
  /// only when the sample rate changes.
  esp_err_t ensure_dac_(uint32_t sample_rate);
  void release_dac_();

  uint8_t dac_channel_{1};
  uint32_t buffer_duration_ms_{100};

  EventGroupHandle_t event_group_{nullptr};
  TaskHandle_t task_handle_{nullptr};

  /// Kept across stop/start; see ensure_dac_(). Only the task touches it.
  dac_continuous_handle_t dac_handle_{nullptr};
  uint32_t dac_rate_{0};

  //
  // All allocated once in setup() and never released. play() writes into the
  // ring buffer from the decoder task while the speaker task reads it - a
  // RingBuffer is safe for that single-producer/single-consumer pair, and
  // neither side can now find it missing halfway through a station change.
  //
  std::unique_ptr<uint8_t[]> in_;
  std::unique_ptr<uint8_t[]> out_;
  std::unique_ptr<ring_buffer::RingBuffer> ring_buffer_;

  //
  // Written by the speaker task, cleared by whoever reads them. Relaxed
  // ordering is enough: these are two independent 16-bit values that exist to
  // be looked at, and nothing else is ordered against them.
  //
  // The one race left is benign and deliberate: a reader clearing between the
  // task's load and store drops a single chunk's peak - 12ms of meter, once,
  // and only when both happen in the same microsecond.
  //
  std::atomic<uint16_t> level_[2]{};
};

}  // namespace dac_speaker
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32
