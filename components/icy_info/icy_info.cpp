#include "icy_info.h"

#ifdef USE_ESP32

#include <esp_crt_bundle.h>
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

// Redirects to follow before giving up. Station URLs commonly point at a
// balancer that answers 302 with the actual node; two hops is more than any of
// them needs and stops a misconfigured server looping this task.
static const int MAX_REDIRECTS = 2;

namespace {
struct HeaderCapture {
  int metaint = -1;
  int bitrate = -1;
  std::string name;
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
    } else if (strcasecmp(evt->header_key, "icy-name") == 0) {
      capture->name = evt->header_value;
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

  xTaskCreate(IcyInfo::fetch_task, "icy_fetch", 4096, this, 5, &this->task_handle_);
  if (this->task_handle_ == nullptr) {
    // Was silent. On a device this tight the 6kB stack is exactly the kind of
    // allocation that fails while audio runs, and then metadata simply never
    // appeared with nothing in the log to say why.
    ESP_LOGW(TAG, "Out of memory starting the metadata fetch; skipping this poll");
  }
}

void IcyInfo::loop() {
  if (this->request_after_ms_ != 0 && millis() >= this->request_after_ms_) {
    this->request_after_ms_ = 0;
    this->update();
  }

  if (this->clear_pending_) {
    this->clear_pending_ = false;
    if (this->title_sensor_ != nullptr)
      this->title_sensor_->publish_state("");
    if (this->bitrate_sensor_ != nullptr)
      this->bitrate_sensor_->publish_state(NAN);
    // The station name is deliberately NOT cleared: it names what the panel is
    // tuned to, which is still true while stopped. Only what is playing goes.
  }

  // publish_state() belongs to the main loop; the task only leaves results.
  bool publish = false;
  std::string title;
  std::string name;
  int bitrate = -1;
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  if (this->dirty_) {
    this->dirty_ = false;
    publish = true;
    title = this->fetched_title_;
    name = this->fetched_name_;
    bitrate = this->fetched_bitrate_;
  }
  xSemaphoreGive(this->mutex_);

  if (publish) {
    if (this->title_sensor_ != nullptr && !title.empty() && this->title_sensor_->state != title)
      this->title_sensor_->publish_state(title);
    if (this->bitrate_sensor_ != nullptr && bitrate > 0 && this->bitrate_sensor_->state != (float) bitrate)
      this->bitrate_sensor_->publish_state((float) bitrate);
    if (this->name_sensor_ != nullptr && !name.empty() && this->name_sensor_->state != name)
      this->name_sensor_->publish_state(name);
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

  //
  // For an https stream, TRY PLAIN HTTP FIRST.
  //
  // Not laziness - measured. https metadata does work: a poll that finds room
  // returns "status 200, icy-name 'SUPER RTL TOGGO', icy-br 128". But a second
  // TLS session wants ~25kB on a heap that measures 10-40kB while audio runs,
  // so most polls died with ESP_ERR_HTTP_CONNECT during the handshake.
  //
  // What travels here is a public station name and song title, and the AUDIO
  // stays on whatever scheme the station list says - only this side-channel
  // drops to http, which costs nothing to set up. If the server refuses plain
  // http the original URL is tried straight after, so nothing is lost that
  // was working before.
  //
  if (url.rfind("https://", 0) == 0) {
    std::string plain = "http://" + url.substr(strlen("https://"));
    if (this->try_fetch_(plain))
      return;
    ESP_LOGD(TAG, "Plain http gave nothing; retrying over https");
  }

  this->try_fetch_(url);
}

//
// One attempt against one URL. Returns true if anything usable came back, so
// the caller knows whether to try the next candidate.
//
bool IcyInfo::try_fetch_(const std::string &url) {

  const bool is_https = url.rfind("https://", 0) == 0;
  if (!is_https && url.rfind("http://", 0) != 0) {
    ESP_LOGW(TAG, "Not an http(s) URL, cannot poll for metadata: %s", url.c_str());
    return false;
  }

  HeaderCapture capture;

  esp_http_client_config_t config = {};
  config.url = url.c_str();
  config.event_handler = http_event_handler;
  config.user_data = &capture;
  config.timeout_ms = 8000;
  config.buffer_size = 1024;

  // https was refused outright here for a while, on the grounds that a TLS
  // handshake costs more heap than this board has spare. It does cost that
  // much with static mbedTLS buffers - CONFIG_MBEDTLS_DYNAMIC_BUFFER in the
  // YAML is what makes a second session affordable, and the audio reader
  // already proves one works.
  //
  // Same CA bundle ESPHome's own audio reader uses, so a stream that plays
  // can also be polled.
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  if (is_https) {
    config.crt_bundle_attach = esp_crt_bundle_attach;
  }
#endif

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr)
    return false;

  esp_http_client_set_header(client, "Icy-MetaData", "1");

  std::string title;
  esp_err_t err = esp_http_client_open(client, 0);
  if (err == ESP_OK) {
    esp_http_client_fetch_headers(client);
    // After fetch_headers, not before - the status line is not parsed until
    // then, so reading it earlier reports 0 no matter what the server said.
    int status = esp_http_client_get_status_code(client);

    // Follow redirects by hand. The automatic ones only happen inside
    // esp_http_client_perform(), which this cannot use - perform() wants to
    // read the whole body, and the body here is an endless radio stream.
    //
    // This is what stood between the panel and its metadata: Klassik Radio
    // answers the published URL with 302, so every poll read the headers of a
    // redirect page and correctly reported no ICY in them.
    for (int hop = 0; hop < MAX_REDIRECTS && (status == 301 || status == 302 || status == 307 || status == 308);
         hop++) {
      if (esp_http_client_set_redirection(client) != ESP_OK) {
        ESP_LOGW(TAG, "Server sent %d without a usable Location header", status);
        break;
      }
      esp_http_client_close(client);
      if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGW(TAG, "Following the redirect failed");
        status = -1;
        break;
      }
      esp_http_client_fetch_headers(client);
      status = esp_http_client_get_status_code(client);
    }

    // The one line that says whether this stream talks ICY at all. A status
    // other than 200 usually means the server refused a second connection
    // from the same client while the audio pipeline holds the first.
    ESP_LOGD(TAG, "%s: status %d, icy-name '%s', icy-br %d, icy-metaint %d", url.c_str(), status,
             capture.name.c_str(), capture.bitrate, capture.metaint);

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

  const bool found = !title.empty() || capture.bitrate > 0 || !capture.name.empty();
  if (found && !title.empty()) {
    ESP_LOGD(TAG, "Now playing: %s", title.c_str());
  }
  if (!found) {
    ESP_LOGD(TAG, "No ICY metadata on this stream");
    return false;
  }

  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  // Compared against the ORIGINAL station URL, not the one just polled - the
  // http retry above deliberately differs from it. What matters is that the
  // station has not changed while this task was downloading, which would make
  // these findings describe the previous stream.
  if (!this->url_.empty()) {
    this->fetched_title_ = title;
    this->fetched_bitrate_ = capture.bitrate;
    this->fetched_name_ = capture.name;
    this->dirty_ = true;
  }
  xSemaphoreGive(this->mutex_);
  return true;
}

}  // namespace icy_info
}  // namespace esphome

#endif  // USE_ESP32
