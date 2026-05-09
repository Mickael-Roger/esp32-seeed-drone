#pragma once

#include "esp_err.h"

// Initialises the UDP socket used to push quaternions to the (statically
// known) ground client at CLIENT_IP:ORIENTATION_PORT.
esp_err_t udp_push_start(void);

// Pushes a quaternion as a 16-byte little-endian payload [w, x, y, z].
// Safe to call from any task. Drops silently if the socket failed to open.
void udp_push_send_quat(float w, float x, float y, float z);
