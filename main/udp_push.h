#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// Starts the registration HTTP endpoint (POST /subscribe) and prepares the
// UDP push socket. Subscribers refresh every few seconds; expired subscribers
// stop receiving packets.
esp_err_t udp_push_start(void);

// Sends a quaternion to all current subscribers as a 16-byte little-endian
// payload: float32 w, x, y, z. Safe to call from any task.
void udp_push_send_quat(float w, float x, float y, float z);

// Returns the underlying HTTP server handle so other modules (e.g. the camera
// stream) can register additional URI handlers on the same server.
httpd_handle_t udp_push_get_httpd(void);
