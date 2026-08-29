#include "dac_speaker.h"

#ifdef USE_ESP32_VARIANT_ESP32

#include <cmath>
#include <cstring>
#include <new>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dac_speaker {

static const char *const TAG = "dac_speaker";

static const uint32_t COMMAND_START = 1 << 0;
static const uint32_t COMMAND_STOP = 1 << 1;
static const uint32_t COMMAND_STOP_GRACEFULLY = 1 << 2;

// Bytes of INPUT audio handled per task iteration. At 44100 Hz stereo 16-bit
// this is ~12 ms of audio, so the task wakes often enough to notice a stop
// quickly and rarely enough not to matter.
//
// It was 1024, and the size is not free-choice: it sets the size of the write
// handed to the driver, and that decides whether the driver splits it. See
// DAC_BUF_SIZE.
static const size_t CHUNK_BYTES = 2048;

// The DAC's DMA ring. These buffers come from MALLOC_CAP_INTERNAL |
// MALLOC_CAP_DMA - a far smaller pool than the general heap.
//
// THESE NUMBERS ARE WHY THE LOG READ "Get available descriptor timeout"
// followed by "dac_continuous: Load data failed", every 100ms, forever.
//
// buf_size is in BYTES OF DMA MEMORY, and one output sample does not cost one
// byte there. The ESP32's DAC borrows I2S0's DMA, which is a 16-bit interface,
// so with CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=y (the default, and set in this
// build) the driver expands every 8-bit sample into the high half of a 16-bit
// slot: s_dac_load_data_into_buf() writes dest[i+1] = src[i/2]. A descriptor
// of buf_size bytes therefore holds buf_size/2 SAMPLES. The old comment here
// claimed 512 bytes matched a 512-sample write; it never did - it matched 256.
//
// On top of that the driver splits any write it considers small
// (s_dac_wait_to_load_dma_data):
//
//     if (buf_size * 2 < 2 * handle->cfg.buf_size)   // i.e. samples < buf_size
//         buf_size >>= 1;                            // every other call
//
// The old pair was DESC_NUM 4 / BUF_SIZE 512 - sized for MONO input, where one
// 1024-byte chunk yields 512 samples. The stream is STEREO ("Decoded audio has
// 2 channels" in the log), and the pipeline has no resampler, so a chunk
// yields 256 samples instead. 256 < 512, so every single write was split into
// two 128-sample descriptors, and the whole ring held 4 x 128 = 512 samples =
// 11.6ms of audio at 44.1kHz. A radio that has to survive a WiFi task at
// priority 23 preempting this one cannot run on 11.6ms.
//
// Now: 12 descriptors of 256 bytes = 128 samples each, so the ring holds 1536
// samples, ~35ms - three times the runway on 1kB more DMA memory. And a stereo
// chunk yields 512 samples, well over the 128-sample split threshold, so the
// driver fills descriptors whole instead of halving them. Mono yields 1024,
// also fine.
static const uint32_t DAC_DESC_NUM = 12;
static const size_t DAC_BUF_SIZE = 256;

// How long the ring above takes to play out: DESC_NUM * (BUF_SIZE / 2) samples
// at the lowest rate this speaker accepts, rounded up. Used to let the DMA run
// itself dry before the driver is disabled - see the end of run_().
static const uint32_t DAC_DRAIN_MS = 80;

// What the ring buffer is sized for, since the real format is not known until
// the first play(). The media_player's pipeline has no resampler, so a stereo
// MP3 arrives as stereo - this is the widest case, and a narrower stream
// simply buffers for longer than buffer_duration rather than overflowing.
static const uint32_t SIZING_SAMPLE_RATE = 44100;
static const uint8_t SIZING_CHANNELS = 2;

//
// EVERYTHING IS ALLOCATED ONCE, in setup(), and the task runs for the lifetime
// of the device.
//
// This began the other way round - task, ring buffer and DAC created on
// start() and released on stop() - which is tidier and wrong on this board.
// Measured during playback the internal 8-bit heap sits between 10 and 40 kB,
// and a station change asks the old pipeline to release while the new one is
// already allocating. The speaker lost that race regularly: "Out of memory
// creating the speaker task" with 2.7 kB free, then silence until the next
// press. Holding ~15 kB permanently takes the speaker out of the race
// entirely - starting a stream now costs it no allocation at all.
//
void DacSpeaker::setup() {
  this->event_group_ = xEventGroupCreate();
  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  // new (std::nothrow), NOT make_unique: make_unique uses plain operator new[],
  // which THROWS std::bad_alloc - and with exceptions disabled that reaches
  // abort() and reboots the device. A null check after make_unique can never
  // run. This cost a crash loop to learn.
  this->in_.reset(new (std::nothrow) uint8_t[CHUNK_BYTES]);
  this->out_.reset(new (std::nothrow) uint8_t[CHUNK_BYTES / 2]);

  const size_t ring_bytes = (size_t) SIZING_SAMPLE_RATE * SIZING_CHANNELS * 2 * this->buffer_duration_ms_ / 1000;
  this->ring_buffer_ = ring_buffer::RingBuffer::create(ring_bytes);

  if (this->in_ == nullptr || this->out_ == nullptr || this->ring_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate audio buffers");
    this->mark_failed();
    return;
  }

  // Build the DAC here too, at the rate almost every stream uses. Its DMA
  // buffers come from the DMA-capable pool, which is the scarcest of the lot,
  // and reserving them at boot - when the heap is empty - is the difference
  // between a radio that always plays and one that plays when it can.
  //
  // A stream at another rate rebuilds it on first use; that is the only case
  // where this allocation happens with audio already running.
  if (this->ensure_dac_(SIZING_SAMPLE_RATE) != ESP_OK) {
    ESP_LOGW(TAG, "Could not reserve the DAC at boot; it will be built on first use");
  }

  xTaskCreate(DacSpeaker::speaker_task, "dac_spk", 4096, this, 19, &this->task_handle_);
  if (this->task_handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create the speaker task");
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

  this->state_ = speaker::STATE_STARTING;
  xEventGroupClearBits(this->event_group_, COMMAND_STOP | COMMAND_STOP_GRACEFULLY);
  xEventGroupSetBits(this->event_group_, COMMAND_START);
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
    // The task is still bringing the DAC up. Spend the caller's patience
    // waiting for it rather than returning 0 immediately.
    vTaskDelay(ticks_to_wait);
    ticks_to_wait = 0;
  }

  if (this->state_ != speaker::STATE_RUNNING)
    return 0;

  return this->ring_buffer_->write_without_replacement(data, length, ticks_to_wait);
}

bool DacSpeaker::has_buffered_data() const {
  return this->ring_buffer_ != nullptr && this->ring_buffer_->available() > 0;
}

void DacSpeaker::speaker_task(void *params) { static_cast<DacSpeaker *>(params)->run_(); }

//
// Creates the DAC on first use and KEEPS it across stop/start, rebuilding only
// when the sample rate actually changes. Its DMA buffers come from the scarce
// DMA-capable pool, so churning them on every station change is exactly the
// allocation this component must not make.
//
esp_err_t DacSpeaker::ensure_dac_(uint32_t sample_rate) {
  if (this->dac_handle_ != nullptr && this->dac_rate_ == sample_rate)
    return ESP_OK;

  this->release_dac_();

  dac_continuous_config_t cfg = {};
  cfg.chan_mask = this->dac_channel_ == 1 ? DAC_CHANNEL_MASK_CH1 : DAC_CHANNEL_MASK_CH0;
  cfg.desc_num = DAC_DESC_NUM;
  cfg.buf_size = DAC_BUF_SIZE;
  cfg.freq_hz = sample_rate;
  cfg.offset = 0;
  cfg.clk_src = DAC_DIGI_CLK_SRC_DEFAULT;
  cfg.chan_mode = DAC_CHANNEL_MODE_SIMUL;

  dac_continuous_handle_t handle = nullptr;
  const esp_err_t err = dac_continuous_new_channels(&cfg, &handle);
  if (err != ESP_OK)
    return err;

  this->dac_handle_ = handle;
  this->dac_rate_ = sample_rate;
  return ESP_OK;
}

void DacSpeaker::release_dac_() {
  if (this->dac_handle_ == nullptr)
    return;
  dac_continuous_disable(this->dac_handle_);
  dac_continuous_del_channels(this->dac_handle_);
  this->dac_handle_ = nullptr;
  this->dac_rate_ = 0;
}

//
// One chunk's RMS, kept if it is louder than what is already waiting. sqrt of
// a mean of squares - integer until the very end, and the only floating point
// in the audio path, once per ~12ms chunk rather than per sample.
//
void DacSpeaker::report_level_(uint8_t channel, uint64_t sum_squares, size_t frames) {
  if (frames == 0)
    return;
  const auto rms = static_cast<uint16_t>(std::sqrt(static_cast<float>(sum_squares / frames)));
  if (rms > this->level_[channel].load(std::memory_order_relaxed))
    this->level_[channel].store(rms, std::memory_order_relaxed);
}

float DacSpeaker::pop_level(uint8_t channel) {
  if (channel > 1)
    return 0.0f;
  // exchange, not load-then-store: the read has to leave the slot empty, or a
  // loud moment would keep being reported long after it passed.
  const uint16_t rms = this->level_[channel].exchange(0, std::memory_order_relaxed);
  return static_cast<float>(rms) / 32768.0f;
}

void DacSpeaker::run_() {
  while (true) {
    // Idle until something wants to play. Nothing is allocated or freed here,
    // which is the whole point of this arrangement.
    const EventBits_t bits =
        xEventGroupWaitBits(this->event_group_, COMMAND_START, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
    if (!(bits & COMMAND_START))
      continue;

    const audio::AudioStreamInfo info = this->audio_stream_info_;
    const uint32_t sample_rate = info.get_sample_rate();
    const uint8_t channels = info.get_channels();
    const size_t frame_bytes = 2 * channels;  // pipeline audio is 16-bit

    // With the default digital clock the ESP32's DAC-DMA floor is 19.6 kHz.
    // Below that the driver rejects the frequency with a range error that does
    // not name the cause, so name it here instead.
    if (sample_rate < 19600) {
      ESP_LOGE(TAG, "Sample rate %" PRIu32 " is below the DAC's 19.6 kHz floor", sample_rate);
      this->state_ = speaker::STATE_STOPPED;
      continue;
    }

    esp_err_t err = this->ensure_dac_(sample_rate);
    if (err == ESP_OK)
      err = dac_continuous_enable(this->dac_handle_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "DAC driver setup failed: %s", esp_err_to_name(err));
      this->release_dac_();
      this->state_ = speaker::STATE_STOPPED;
      // Stand back before accepting another start. play() calls start() on
      // every chunk it cannot place, so without this one failure becomes a
      // tight loop of identical errors - which is exactly what the rewrite
      // reintroduced after the earlier version had already fixed it.
      vTaskDelay(pdMS_TO_TICKS(1000));
      xEventGroupClearBits(this->event_group_, COMMAND_START);
      continue;
    }

    this->ring_buffer_->reset();
    this->state_ = speaker::STATE_RUNNING;

    // Bytes at the front of in_ carried over because the ring buffer handed
    // over a partial frame. Without this a read that splits a 16-bit sample
    // shifts every later sample by one byte - full-scale noise, not audio.
    size_t carry = 0;
    uint8_t consecutive_write_failures = 0;
    bool stalled = false;

    while (true) {
      const EventBits_t now_bits = xEventGroupGetBits(this->event_group_);
      if (now_bits & COMMAND_STOP)
        break;

      const size_t got = this->ring_buffer_->read(this->in_.get() + carry, CHUNK_BYTES - carry, pdMS_TO_TICKS(20));
      const size_t have = carry + got;

      if (have < frame_bytes) {
        carry = have;
        // Graceful stop: done once the pipeline stopped feeding and the ring
        // buffer has drained past the last whole frame.
        if ((now_bits & COMMAND_STOP_GRACEFULLY) && got == 0)
          break;
        continue;
      }

      const size_t frames = have / frame_bytes;
      const size_t used = frames * frame_bytes;

      // 16-bit signed -> 8-bit unsigned, downmixed, volume applied. Q8 keeps
      // the multiply in integers; mute is volume zero, which parks the output
      // at the DAC's midpoint instead of slamming it to ground.
      //
      // The meter's sums are taken in the same pass, from the samples as they
      // arrive - one traversal, and the levels are pre-volume because that is
      // what pop_level() promises. Squaring a full-scale pair costs two
      // multiplies per frame; at 44.1kHz stereo that is 88k of them a second,
      // which this core does not notice.
      const auto *samples = reinterpret_cast<const int16_t *>(this->in_.get());
      const int32_t vol_q8 = this->mute_state_ ? 0 : static_cast<int32_t>(this->volume_ * 256.0f);
      uint64_t sum_l = 0;
      uint64_t sum_r = 0;
      for (size_t f = 0; f < frames; f++) {
        const int32_t l = channels == 2 ? samples[2 * f] : samples[f];
        const int32_t r = channels == 2 ? samples[2 * f + 1] : l;
        sum_l += static_cast<uint32_t>(l * l);
        sum_r += static_cast<uint32_t>(r * r);

        int32_t s = channels == 2 ? (l + r) / 2 : l;
        s = (s * vol_q8) >> 8;
        this->out_[f] = static_cast<uint8_t>(static_cast<uint32_t>(s + 32768) >> 8);
      }
      this->report_level_(0, sum_l, frames);
      this->report_level_(1, sum_r, frames);

      size_t written = 0;
      bool write_failed = false;
      while (written < frames) {
        size_t loaded = 0;
        if (dac_continuous_write(this->dac_handle_, this->out_.get() + written, frames - written, &loaded, 100) !=
            ESP_OK) {
          write_failed = true;
          break;
        }
        written += loaded;
        if (xEventGroupGetBits(this->event_group_) & COMMAND_STOP)
          break;
      }

      // A DMA that has drained to a stop does not always come back on its own:
      // its descriptors sit in the pending chain rather than the pool, so every
      // later write waits out its timeout. Drop this DAC instance - the next
      // start rebuilds it, which is the one case where rebuilding is right.
      if (write_failed) {
        if (++consecutive_write_failures >= 5) {
          ESP_LOGW(TAG, "DAC DMA stalled; rebuilding it on the next start");
          this->release_dac_();
          stalled = true;
          break;
        }
      } else {
        consecutive_write_failures = 0;
      }

      carry = have - used;
      if (carry > 0) {
        std::memmove(this->in_.get(), this->in_.get() + used, carry);
      }
    }

    // Disabled, NOT deleted - the handle and its DMA buffers stay, so the next
    // station change costs no allocation at all.
    //
    // The delay is not politeness, it is correctness. The driver keeps its
    // in-flight descriptors on handle->head, and NOTHING clears that list:
    // dac_continuous_disable() does not, and dac_continuous_enable() only
    // refills the descriptor POOL (xQueueReset + xQueueSend for each desc) and
    // leaves the chain alone. Disable with descriptors still in flight and the
    // list stays populated across the stop. On the next start
    // dac_continuous_write() then finds STAILQ_FIRST(&head) != NULL, concludes
    // the DMA is still running, skips the branch that loads the ring and calls
    // dac_dma_periph_dma_trans_start(), and instead restarts from a descriptor
    // the hardware already consumed. No EOF interrupt ever follows, so nothing
    // is ever returned to the pool, and every write from then on waits out its
    // full 100ms timeout - which is exactly the log this fixes.
    //
    // Sleeping DAC_DRAIN_MS lets the hardware run the chain to its end, where
    // the EOF interrupts empty handle->head and the total-EOF sets is_running
    // false. Then disable() has nothing left to strand. The cost is that an
    // abrupt stop keeps playing for up to 80ms of already-buffered audio.
    if (this->dac_handle_ != nullptr) {
      if (!stalled) {
        vTaskDelay(pdMS_TO_TICKS(DAC_DRAIN_MS));
      }
      dac_continuous_disable(this->dac_handle_);
    }
    this->state_ = speaker::STATE_STOPPED;

    // Same stand-back as the setup-failure path above, and for the same
    // reason: play() calls start() on every chunk it cannot place, so a DAC
    // that keeps stalling would otherwise retry as fast as the decoder feeds.
    if (stalled) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      xEventGroupClearBits(this->event_group_, COMMAND_START);
    }
  }
}

}  // namespace dac_speaker
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32
