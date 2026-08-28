#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <string>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace icy_info {

/// How long request_now() stands back from a station change before polling.
static const uint32_t REQUEST_DELAY_MS = 5000;

//
// Polls a SHOUTcast/Icecast stream for its ICY metadata - see __init__.py for
// why this exists and what it costs.
//
// Threading model: update() spawns a short-lived FreeRTOS task, because
// reading one metadata interval means downloading it, and stream servers feed
// at little more than playback speed - one to two seconds the main loop must
// not spend blocked. The task writes its findings into members under a mutex
// and exits; loop() publishes them from the main loop, where publish_state()
// is safe to call.
//
class IcyInfo : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  void set_title_sensor(text_sensor::TextSensor *sensor) { this->title_sensor_ = sensor; }
  void set_bitrate_sensor(sensor::Sensor *sensor) { this->bitrate_sensor_ = sensor; }
  void set_name_sensor(text_sensor::TextSensor *sensor) { this->name_sensor_ = sensor; }

  /// The stream to poll. An empty string stops polling and clears both
  /// sensors - the YAML calls this on every station change and on stop.
  void set_url(const std::string &url);

  /// Poll soon rather than at the next interval - called on station change so
  /// the title does not lag the music by half a minute.
  ///
  /// Soon, not now: a station change is the one moment the audio pipeline
  /// allocates everything it needs, and this poll wants a 4kB task stack and
  /// an HTTP client at the same instant. Standing back a few seconds keeps
  /// the metadata prompt without competing for the heap that has to carry the
  /// audio.
  void request_now() { this->request_after_ms_ = millis() + REQUEST_DELAY_MS; }

 protected:
  static void fetch_task(void *params);
  void run_fetch_();
  /// One attempt against one URL; true if anything usable came back.
  bool try_fetch_(const std::string &url);

  text_sensor::TextSensor *title_sensor_{nullptr};
  sensor::Sensor *bitrate_sensor_{nullptr};
  text_sensor::TextSensor *name_sensor_{nullptr};

  SemaphoreHandle_t mutex_{nullptr};
  TaskHandle_t task_handle_{nullptr};

  // All guarded by mutex_: url_ is read by the task at start, the results are
  // written by the task and consumed by loop().
  std::string url_;
  std::string fetched_title_;
  std::string fetched_name_;
  int fetched_bitrate_{-1};
  bool dirty_{false};

  bool clear_pending_{false};
  /// millis() at which a deferred poll becomes due; 0 means none pending.
  uint32_t request_after_ms_{0};
};

}  // namespace icy_info
}  // namespace esphome

#endif  // USE_ESP32
