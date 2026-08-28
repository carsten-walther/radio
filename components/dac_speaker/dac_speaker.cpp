#include "dac_speaker.h"

#ifdef USE_ESP32_VARIANT_ESP32

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dac_speaker {

static const char *const TAG = "dac_speaker";

static const uint32_t COMMAND_STOP = 1 << 0;
static const uint32_t COMMAND_STOP_GRACEFULLY = 1 << 1;

// Bytes of INPUT audio handled per task iteration. At 44100 Hz stereo 16-bit
// this is ~6 ms of audio, so the task wakes often enough to notice a stop
// quickly and rarely enough not to matter.
static const size_t CHUNK_BYTES = 1024;

// How long start() stays quiet after a failure. Long enough that a device out
// of memory logs once a second instead of fifty times, short enough that a
// transient shortage recovers on its own.
static const uint32_t RETRY_BACKOFF_MS = 1000;

// The DAC's DMA ring. These buffers come from MALLOC_CAP_INTERNAL |
// MALLOC_CAP_DMA - a far smaller pool than the general heap - so this is sized
// to be frugal rather than comfortable: 3 x 1024 is 3kB, where the 4 x 2048
// this started with wanted 8kB and failed with ESP_ERR_NO_MEM the moment the
// audio pipeline had taken its share.
//
// At 44100Hz mono 8-bit each buffer is 23ms of audio, so the DMA holds ~70ms
// and its interrupt fires every 23ms. Raise desc_num first if playback ever
// crackles under load; the driver requires at least 2.
static const uint32_t DAC_DESC_NUM = 3;
static const size_t DAC_BUF_SIZE = 1024;

void DacSpeaker::setup() {
  this->event_group_ = xEventGroupCreate();
  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
  }
}

void DacSpeaker::dump_config() {
  ESP_LOGCONFIG(TAG, "DAC Speaker:");
  ESP_LOGCONFIG(TAG, "  Pin: GPIO%d (DAC channel %d)", this->dac_channel_ == 1 ? 26 : 25, this->dac_channel_);
  ESP_LOGCONFIG(TAG, "  Buffer duration: %" PRIu32 "ms", this->buffer_duration_ms_);
}

void DacSpeaker::start() {
  if (this->is_failed() || this->state_ == speaker::STATE_RUNNING || this->state_ == speaker::STATE_STARTING)
    return;

  // Silent while backing off. play() calls start() on every chunk it cannot
  // place, so without this one failure becomes an unbounded error storm.
  if (millis() < this->retry_after_ms_)
    return;

  // STARTING before the task exists, so a caller polling is_stopped() between
  // here and the task's first instruction cannot conclude the speaker is idle.
  this->state_ = speaker::STATE_STARTING;
  xEventGroupClearBits(this->event_group_, COMMAND_STOP | COMMAND_STOP_GRACEFULLY);

  xTaskCreate(DacSpeaker::speaker_task, "dac_spk", 4096, this, 19, &this->task_handle_);
  if (this->task_handle_ == nullptr) {
    ESP_LOGE(TAG, "Out of memory creating the speaker task; retrying in %" PRIu32 "ms", RETRY_BACKOFF_MS);
    this->retry_after_ms_ = millis() + RETRY_BACKOFF_MS;
    this->state_ = speaker::STATE_STOPPED;
  }
}

void DacSpeaker::stop() {
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  xEventGroupSetBits(this->event_group_, COMMAND_STOP);
}

void DacSpeaker::finish() {
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  xEventGroupSetBits(this->event_group_, COMMAND_STOP_GRACEFULLY);
}

size_t DacSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  if (this->is_failed())
    return 0;

  if (this->state_ != speaker::STATE_RUNNING && this->state_ != speaker::STATE_STARTING) {
    this->start();
  }

  if (this->state_ != speaker::STATE_RUNNING) {
    // The task is still bringing the driver up. Spend the caller's patience
    // waiting for it rather than returning 0 immediately.
    vTaskDelay(ticks_to_wait);
    ticks_to_wait = 0;
  }

  size_t bytes_written = 0;
  if (this->state_ == speaker::STATE_RUNNING) {
    auto rb = this->ring_buffer_;
    if (rb != nullptr) {
      bytes_written = rb->write_without_replacement(data, length, ticks_to_wait);
    }
  }
  return bytes_written;
}

bool DacSpeaker::has_buffered_data() const {
  auto rb = this->ring_buffer_;
  return rb != nullptr && rb->available() > 0;
}

void DacSpeaker::speaker_task(void *params) {
  auto *this_speaker = static_cast<DacSpeaker *>(params);
  this_speaker->run_();

  this_speaker->task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

void DacSpeaker::run_() {
  // The pipeline sets the stream info before the first play(), and play() is
  // what starts this task - so by the time this runs, the format is real.
  const audio::AudioStreamInfo info = this->audio_stream_info_;
  const uint32_t sample_rate = info.get_sample_rate();
  const uint8_t channels = info.get_channels();
  const size_t frame_bytes = 2 * channels;  // pipeline audio is 16-bit

  // With the default digital clock the ESP32's DAC-DMA floor is 19.6 kHz.
  // Below that the driver would reject the config at runtime with a range
  // error that does not name the cause, so name it here instead.
  if (sample_rate < 19600) {
    ESP_LOGE(TAG, "Sample rate %" PRIu32 " is below the DAC's 19.6 kHz floor", sample_rate);
    this->state_ = speaker::STATE_STOPPED;
    return;
  }

  dac_continuous_config_t cfg = {};
  cfg.chan_mask = this->dac_channel_ == 1 ? DAC_CHANNEL_MASK_CH1 : DAC_CHANNEL_MASK_CH0;
  cfg.desc_num = DAC_DESC_NUM;
  cfg.buf_size = DAC_BUF_SIZE;
  cfg.freq_hz = sample_rate;
  cfg.offset = 0;
  cfg.clk_src = DAC_DIGI_CLK_SRC_DEFAULT;
  cfg.chan_mode = DAC_CHANNEL_MODE_SIMUL;

  dac_continuous_handle_t handle = nullptr;
  esp_err_t err = dac_continuous_new_channels(&cfg, &handle);
  if (err == ESP_OK) {
    err = dac_continuous_enable(handle);
    if (err != ESP_OK) {
      dac_continuous_del_channels(handle);
      handle = nullptr;
    }
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "DAC driver setup failed: %s", esp_err_to_name(err));
    this->retry_after_ms_ = millis() + RETRY_BACKOFF_MS;
    this->state_ = speaker::STATE_STOPPED;
    return;
  }

  // Input chunk plus the mono 8-bit output it shrinks to. Heap rather than
  // stack: 1.25 kB of task stack is real money at a 4 kB stack size.
  auto in = std::make_unique<uint8_t[]>(CHUNK_BYTES);
  auto out = std::make_unique<uint8_t[]>(CHUNK_BYTES / frame_bytes);
  auto rb = ring_buffer::RingBuffer::create(info.ms_to_bytes(this->buffer_duration_ms_));

  if (in == nullptr || out == nullptr || rb == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate audio buffers");
    dac_continuous_disable(handle);
    dac_continuous_del_channels(handle);
    this->retry_after_ms_ = millis() + RETRY_BACKOFF_MS;
    this->state_ = speaker::STATE_STOPPED;
    return;
  }

  this->ring_buffer_ = std::move(rb);
  this->retry_after_ms_ = 0;
  this->state_ = speaker::STATE_RUNNING;

  // Bytes at the front of `in` carried over from the previous iteration
  // because the ring buffer handed over a partial frame. Without this a read
  // that splits a 16-bit sample would shift every later sample by one byte -
  // full-scale noise, not audio.
  size_t carry = 0;

  while (true) {
    const uint32_t bits = xEventGroupGetBits(this->event_group_);
    if (bits & COMMAND_STOP)
      break;

    const size_t got = this->ring_buffer_->read(in.get() + carry, CHUNK_BYTES - carry, pdMS_TO_TICKS(20));
    const size_t have = carry + got;

    if (have < frame_bytes) {
      carry = have;
      // Graceful stop: done once the pipeline stopped feeding and the ring
      // buffer has drained past the last whole frame.
      if ((bits & COMMAND_STOP_GRACEFULLY) && got == 0)
        break;
      continue;
    }

    const size_t frames = have / frame_bytes;
    const size_t used = frames * frame_bytes;

    // 16-bit signed -> 8-bit unsigned, downmixed, volume applied. Q8 keeps
    // the multiply in integers; mute is just volume zero, which parks the
    // output at the DAC's midpoint instead of slamming it to ground.
    const auto *samples = reinterpret_cast<const int16_t *>(in.get());
    const int32_t vol_q8 = this->mute_state_ ? 0 : static_cast<int32_t>(this->volume_ * 256.0f);
    for (size_t f = 0; f < frames; f++) {
      int32_t s = channels == 2 ? (static_cast<int32_t>(samples[2 * f]) + samples[2 * f + 1]) / 2 : samples[f];
      s = (s * vol_q8) >> 8;
      out[f] = static_cast<uint8_t>(static_cast<uint32_t>(s + 32768) >> 8);
    }

    size_t written = 0;
    while (written < frames) {
      size_t loaded = 0;
      if (dac_continuous_write(handle, out.get() + written, frames - written, &loaded, 100) != ESP_OK)
        break;
      written += loaded;
      if (xEventGroupGetBits(this->event_group_) & COMMAND_STOP)
        break;
    }

    carry = have - used;
    if (carry > 0) {
      std::memmove(in.get(), in.get() + used, carry);
    }
  }

  dac_continuous_disable(handle);
  dac_continuous_del_channels(handle);

  // STOPPED before releasing the buffer: play() only touches the buffer
  // while the state reads RUNNING.
  this->state_ = speaker::STATE_STOPPED;
  this->ring_buffer_.reset();
}

}  // namespace dac_speaker
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32
