#pragma once

#include "esp_err.h"

// Brings up UART1 to the flight controller (TX on GPIO 43, 19200 8N1) and a
// UDP server on FLIGHT_PORT (from wifi_credentials.h). Each datagram is
// forwarded to UART verbatim after stripping its first byte (a 1-byte framing
// prefix used by the ground client).
esp_err_t flight_control_start(void);
