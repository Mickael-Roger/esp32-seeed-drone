#pragma once

#include "esp_err.h"

// Brings up the ESP32 as a WiFi access point on the SSID/PSK/channel from
// wifi_credentials.h, with a static AP IP and a single-slot DHCP pool that
// always hands out CLIENT_IP. Returns once the AP is started.
esp_err_t wifi_ap_start(void);
