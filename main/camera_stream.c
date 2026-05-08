#include "camera_stream.h"

#include <stdio.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "camera";

// XIAO ESP32-S3 Sense camera pin map (OV2640)
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   10
#define CAM_PIN_SIOD   40
#define CAM_PIN_SIOC   39
#define CAM_PIN_D7     48
#define CAM_PIN_D6     11
#define CAM_PIN_D5     12
#define CAM_PIN_D4     14
#define CAM_PIN_D3     16
#define CAM_PIN_D2     18
#define CAM_PIN_D1     17
#define CAM_PIN_D0     15
#define CAM_PIN_VSYNC  38
#define CAM_PIN_HREF   47
#define CAM_PIN_PCLK   13

#define BOUNDARY      "frame"
#define STREAM_TYPE   "multipart/x-mixed-replace; boundary=" BOUNDARY
#define PART_HEADER   "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

esp_err_t camera_stream_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn      = CAM_PIN_PWDN,
        .pin_reset     = CAM_PIN_RESET,
        .pin_xclk      = CAM_PIN_XCLK,
        .pin_sccb_sda  = CAM_PIN_SIOD,
        .pin_sccb_scl  = CAM_PIN_SIOC,
        .pin_d7        = CAM_PIN_D7,
        .pin_d6        = CAM_PIN_D6,
        .pin_d5        = CAM_PIN_D5,
        .pin_d4        = CAM_PIN_D4,
        .pin_d3        = CAM_PIN_D3,
        .pin_d2        = CAM_PIN_D2,
        .pin_d1        = CAM_PIN_D1,
        .pin_d0        = CAM_PIN_D0,
        .pin_vsync     = CAM_PIN_VSYNC,
        .pin_href      = CAM_PIN_HREF,
        .pin_pclk      = CAM_PIN_PCLK,
        .xclk_freq_hz  = 20 * 1000 * 1000,
        .ledc_timer    = LEDC_TIMER_0,
        .ledc_channel  = LEDC_CHANNEL_0,
        .pixel_format  = PIXFORMAT_JPEG,
        .frame_size    = FRAMESIZE_VGA,    // 640x480
        .jpeg_quality  = 12,                // 0 (best) – 63 (worst)
        .fb_count      = 2,
        .fb_location   = CAMERA_FB_IN_PSRAM,
        .grab_mode     = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "camera ready (VGA JPEG)");
    return ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t err = httpd_resp_set_type(req, STREAM_TYPE);
    if (err != ESP_OK) return err;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char header[80];
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "camera_fb_get returned NULL");
            return ESP_FAIL;
        }

        int hlen = snprintf(header, sizeof(header), PART_HEADER, (unsigned)fb->len);
        err = httpd_resp_send_chunk(req, header, hlen);
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        esp_camera_fb_return(fb);

        if (err != ESP_OK) {
            // Client disconnected
            return ESP_OK;
        }
    }
}

static const httpd_uri_t stream_uri = {
    .uri     = "/stream",
    .method  = HTTP_GET,
    .handler = stream_handler,
};

static httpd_handle_t s_stream_server;

esp_err_t camera_stream_start_server(uint16_t port)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.ctrl_port  = port + 1024;  // separate internal control socket

    esp_err_t err = httpd_start(&s_stream_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start (stream) failed: %s", esp_err_to_name(err));
        return err;
    }
    err = httpd_register_uri_handler(s_stream_server, &stream_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register /stream failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "stream server up on port %u", port);
    return ESP_OK;
}
