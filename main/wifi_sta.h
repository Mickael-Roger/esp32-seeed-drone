#pragma once

#include "esp_err.h"

// Brings up WiFi in STA mode using credentials from wifi_credentials.h.
// Blocks until associated and an IP is obtained, or returns ESP_FAIL after retries.
esp_err_t wifi_sta_start(void);
