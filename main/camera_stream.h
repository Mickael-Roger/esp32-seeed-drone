#pragma once

#include "esp_err.h"

// Initializes the OV2640 (XIAO ESP32-S3 Sense pin map) at VGA / JPEG.
esp_err_t camera_stream_init(void);

// Starts a background task that captures JPEG frames and pushes them to
// CLIENT_IP:VIDEO_PORT as fragmented UDP datagrams. Each fragment carries
// an 8-byte header: <u32 frame_id LE><u16 packet_idx LE><u16 packet_total LE>
// followed by up to 1400 bytes of JPEG. The client reassembles by frame_id.
esp_err_t camera_stream_start(void);
