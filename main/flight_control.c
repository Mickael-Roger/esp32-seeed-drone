#include "flight_control.h"

#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_credentials.h"

#define UART_PORT       UART_NUM_1
#define UART_TX_GPIO    43            // XIAO D6 — freed by routing console to USB-JTAG
#define UART_BAUD_RATE  19200

#define RX_BUF_SIZE     128

static const char *TAG = "flight";

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART1 up on TX=GPIO%d @ %d bps", UART_TX_GPIO, UART_BAUD_RATE);
}

static void udp_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(FLIGHT_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "UDP listening on port %u", FLIGHT_PORT);

    uint8_t buf[RX_BUF_SIZE];
    while (1) {
        int len = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (len <= 1) continue;

        // Skip the 1-byte framing prefix; forward the rest to the flight
        // controller verbatim.
        uart_write_bytes(UART_PORT, (const char *)&buf[1], len - 1);
    }
}

esp_err_t flight_control_start(void)
{
    uart_init();

    if (xTaskCreate(udp_task, "fc_udp", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
