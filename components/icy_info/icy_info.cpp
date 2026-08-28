#include "icy_info.h"

#ifdef USE_ESP32

#include <esp_http_client.h>

#include <cstdlib>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace icy_info {

static const char *const TAG = "icy_info";

// Servers commonly use 8192 or 16000. Anything far beyond that is either a
// misconfigured server or not ICY at all - refuse rather than download it.
static const int MAX_METAINT = 65536;

namespace {
struct HeaderCapture {
  int metaint = -1;
  int bitrate = -1;
};

esp_err_t http_event_handler(esp_http_client_event_t *evt) {
  // esp_http_client_get_header() reads REQUEST headers, not response ones -
  // response headers are only reachable through this event.
  if (evt->event_id == HTTP_EVENT_ON_HEADER) {
    auto *capture = static_cast<HeaderCapture *>(evt->user_data);
    if (strcasecmp(evt->header_key, "icy-metaint") == 0) {
      capture->metaint = atoi(evt->header_value);
    } else if (strcasecmp(evt->header_key, "icy-br") == 0) {
      capture->bitrate = atoi(evt->header_value);
    }
  }
  return ESP_OK;
}
}  // namespace

void IcyInfo::setup() {
  this->mutex_ = xSemaphoreCreateMutex();
  if (this->mutex_ == nullptr) {
    this->mark_failed();
  }
}

void IcyInfo::dump_config() { ESP_LOGCONFIG(TAG, "ICY stream info, polling every %" PRIu32 "ms", this->get_update_interval()); }

void IcyInfo::set_url(const std::string &url) {
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  this->url_ = url;
  xSemaphoreGive(this->mutex_);
  if (url.empty()) {
    this->clear_pending_ = true;
  }
}

void IcyInfo::update() {
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  const bool have_url = !this->url_.empty();
  xSemaphoreGive(this->mutex_);

  // One task at a time: a poll that is still downloading when the next
  // interval fires simply keeps the older task, rather than stacking a second
  // connection on a heap that cannot afford it.
  if (!have_url || this->task_handle_ != nullptr)
    return;

  xTaskCreate(IcyInfo::fetch_task, "icy_fetch", 6144, this, 5, &this->task_handle_);
}

void IcyInfo::loop() {
  if (this->request_now_) {
    this->request_now_ = false;
    this->update();
  }

  if (this->clear_pending_) {
    this->clear_pending_ = false;
    if (this->title_sensor_ != nullptr)
      this->title_sensor_->publish_state("");
    if (this->bitrate_sensor_ != nullptr)
      this->bitrate_sensor_->publish_state(NAN);
  }

  // publish_state() belongs to the main loop; the task only leaves results.
  bool publish = false;
  std::string title;
  int bitrate = -1;
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  if (this->dirty_) {
    this->dirty_ = false;
    publish = true;
    title = this->fetched_title_;
    bitrate = this->fetched_bitrate_;
  }
  xSemaphoreGive(this->mutex_);

  if (publish) {
    if (this->title_sensor_ != nullptr && !title.empty() && this->title_sensor_->state != title)
      this->title_sensor_->publish_state(title);
    if (this->bitrate_sensor_ != nullptr && bitrate > 0 && this->bitrate_sensor_->state != (float) bitrate)
      this->bitrate_sensor_->publish_state((float) bitrate);
  }
}

void IcyInfo::fetch_task(void *params) {
  auto *this_info = static_cast<IcyInfo *>(params);
  this_info->run_fetch_();
  this_info->task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

void IcyInfo::run_fetch_() {
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  const std::string url = this->url_;
  xSemaphoreGive(this->mutex_);

  if (url.empty())
    return;

  // Plain http only - see __init__.py. A TLS handshake costs ~40kB of heap,
  // which does not exist next to a running audio pipeline on this board.
  if (url.rfind("http://", 0) != 0) {
    ESP_LOGW(TAG, "Only http:// streams can be polled for metadata: %s", url.c_str());
    return;
  }

  HeaderCapture capture;

  esp_http_client_config_t config = {};
  config.url = url.c_str();
  config.event_handler = http_event_handler;
  config.user_data = &capture;
  config.timeout_ms = 8000;
  config.buffer_size = 1024;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr)
    return;

  esp_http_client_set_header(client, "Icy-MetaData", "1");

  std::string title;
  esp_err_t err = esp_http_client_open(client, 0);
  if (err == ESP_OK) {
    esp_http_client_fetch_headers(client);

    if (capture.metaint > 0 && capture.metaint <= MAX_METAINT) {
      char buf[512];
      // The first metadata block usually carries the current title, but a
      // server may send an empty one; read up to two intervals before giving
      // up rather than looping on a server that never talks.
      for (int block = 0; block < 2 && title.empty(); block++) {
        // Discard one interval of audio.
        int remaining = capture.metaint;
        while (remaining > 0) {
          const int n = esp_http_client_read(client, buf, std::min(remaining, (int) sizeof(buf)));
          if (n <= 0) {
            remaining = -1;
            break;
          }
          remaining -= n;
        }
        if (remaining != 0)
          break;

        // One length byte, then length*16 bytes of metadata.
        char len_byte;
        if (esp_http_client_read(client, &len_byte, 1) != 1)
          break;
        int meta_len = (uint8_t) len_byte * 16;
        std::string meta;
        while (meta_len > 0) {
          const int n = esp_http_client_read(client, buf, std::min(meta_len, (int) sizeof(buf)));
          if (n <= 0)
            break;
          meta.append(buf, n);
          meta_len -= n;
        }

        const size_t start = meta.find("StreamTitle='");
        if (start != std::string::npos) {
          const size_t from = start + strlen("StreamTitle='");
          const size_t end = meta.find("';", from);
          if (end != std::string::npos && end > from) {
            title = meta.substr(from, end - from);
          }
        }
      }
    }
  } else {
    ESP_LOGW(TAG, "Connection to %s failed: %s", url.c_str(), esp_err_to_name(err));
  }

  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  // Only report for the station that was asked about: if the URL changed
  // while this task was downloading, its findings describe the old stream.
  if (this->url_ == url && (!title.empty() || capture.bitrate > 0)) {
    this->fetched_title_ = title;
    this->fetched_bitrate_ = capture.bitrate;
    this->dirty_ = true;
  }
  xSemaphoreGive(this->mutex_);
}

}  // namespace icy_info
}  // namespace esphome

#endif  // USE_ESP32
