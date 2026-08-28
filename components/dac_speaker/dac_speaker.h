#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32_VARIANT_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <driver/dac_continuous.h>

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

  void start() override;
  void stop() override;
  void finish() override;

  bool has_buffered_data() const override;

 protected:
  static void speaker_task(void *params);
  void run_();

  uint8_t dac_channel_{1};
  uint32_t buffer_duration_ms_{100};

  EventGroupHandle_t event_group_{nullptr};
  TaskHandle_t task_handle_{nullptr};

  /// Earliest millis() at which start() may try again after a failure.
  ///
  /// Without it a failed start is retried on every play(), which the decoder
  /// calls every ~20ms - so one exhausted heap produced a permanent error
  /// storm that burned the CPU and log bandwidth needed to recover from it.
  uint32_t retry_after_ms_{0};

  /// Owned by the task while running; play() and has_buffered_data() take
  /// copies. Same arrangement as the i2s_audio speaker.
  std::shared_ptr<ring_buffer::RingBuffer> ring_buffer_;
};

}  // namespace dac_speaker
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32
