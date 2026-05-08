#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// Initializes the OV2640 (XIAO ESP32-S3 Sense pin map) at VGA / JPEG.
esp_err_t camera_stream_init(void);

// Starts a dedicated httpd on the given port, exposing GET /stream as
// multipart/x-mixed-replace JPEG frames at the camera's natural rate.
// A separate server is required because the streaming handler never returns
// and would otherwise block sibling endpoints on the main HTTP server.
esp_err_t camera_stream_start_server(uint16_t port);
