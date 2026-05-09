#include "camera_stream.h"

#include <errno.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_credentials.h"

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

#define MAX_PAYLOAD    1400          // < typical WiFi MTU minus IP+UDP headers
#define HEADER_SIZE    8             // u32 frame_id, u16 idx, u16 total

static int                s_sock = -1;
static struct sockaddr_in s_dst;

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
        .jpeg_quality  = 12,
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

static void send_jpeg(const uint8_t *jpeg, size_t len)
{
    static uint32_t frame_id = 0;
    frame_id++;

    uint16_t total = (uint16_t)((len + MAX_PAYLOAD - 1) / MAX_PAYLOAD);
    if (total == 0) return;

    uint8_t pkt[HEADER_SIZE + MAX_PAYLOAD];

    for (uint16_t idx = 0; idx < total; idx++) {
        size_t off   = (size_t)idx * MAX_PAYLOAD;
        size_t chunk = (len - off > MAX_PAYLOAD) ? MAX_PAYLOAD : (len - off);

        memcpy(pkt + 0, &frame_id, sizeof(frame_id));
        memcpy(pkt + 4, &idx,      sizeof(idx));
        memcpy(pkt + 6, &total,    sizeof(total));
        memcpy(pkt + HEADER_SIZE, jpeg + off, chunk);

        if (sendto(s_sock, pkt, HEADER_SIZE + chunk, 0,
                   (struct sockaddr *)&s_dst, sizeof(s_dst)) < 0) {
            // WiFi TX queue full or client gone — drop the rest of this
            // frame; the next frame is ~50 ms away.
            return;
        }
    }
}

static void stream_task(void *arg)
{
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "camera_fb_get returned NULL");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        send_jpeg(fb->buf, fb->len);
        esp_camera_fb_return(fb);
    }
}

esp_err_t camera_stream_start(void)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }

    memset(&s_dst, 0, sizeof(s_dst));
    s_dst.sin_family = AF_INET;
    s_dst.sin_port   = htons(VIDEO_PORT);
    if (inet_pton(AF_INET, CLIENT_IP, &s_dst.sin_addr) != 1) {
        ESP_LOGE(TAG, "bad CLIENT_IP '%s'", CLIENT_IP);
        return ESP_ERR_INVALID_ARG;
    }

    // Pin to Core 0 — camera capture itself runs on Core 1
    // (CONFIG_CAMERA_CORE1=y), so the sender lives next to WiFi/LWIP.
    if (xTaskCreatePinnedToCore(stream_task, "video", 4096, NULL, 4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "video stream → %s:%u", CLIENT_IP, VIDEO_PORT);
    return ESP_OK;
}
