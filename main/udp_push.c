#include "udp_push.h"

#include <errno.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "esp_log.h"

#include "wifi_credentials.h"

static const char *TAG = "udp_push";

static int                s_sock = -1;
static struct sockaddr_in s_target;

esp_err_t udp_push_start(void)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }

    memset(&s_target, 0, sizeof(s_target));
    s_target.sin_family = AF_INET;
    s_target.sin_port   = htons(ORIENTATION_PORT);
    if (inet_pton(AF_INET, CLIENT_IP, &s_target.sin_addr) != 1) {
        ESP_LOGE(TAG, "bad CLIENT_IP '%s'", CLIENT_IP);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "pushing quaternions to %s:%u", CLIENT_IP, ORIENTATION_PORT);
    return ESP_OK;
}

void udp_push_send_quat(float w, float x, float y, float z)
{
    if (s_sock < 0) return;

    float buf[4] = { w, x, y, z };
    sendto(s_sock, buf, sizeof(buf), 0,
           (struct sockaddr *)&s_target, sizeof(s_target));
}
