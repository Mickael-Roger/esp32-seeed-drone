#include "udp_push.h"

#include <errno.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define MAX_SUBSCRIBERS    4
#define SUBSCRIBE_TTL_US   (5LL * 1000 * 1000)

static const char *TAG = "udp_push";

typedef struct {
    in_addr_t ip;          // network byte order
    uint16_t  port;        // host byte order
    int64_t   expires_us;  // 0 = empty slot
} subscriber_t;

static subscriber_t       s_subs[MAX_SUBSCRIBERS];
static SemaphoreHandle_t  s_subs_lock;
static int                s_udp_sock = -1;
static httpd_handle_t     s_httpd;

static void add_or_refresh(in_addr_t ip, uint16_t port)
{
    int64_t now     = esp_timer_get_time();
    int64_t expires = now + SUBSCRIBE_TTL_US;

    xSemaphoreTake(s_subs_lock, portMAX_DELAY);

    int free_slot = -1;
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (s_subs[i].expires_us > now &&
            s_subs[i].ip == ip && s_subs[i].port == port) {
            s_subs[i].expires_us = expires;
            xSemaphoreGive(s_subs_lock);
            return;
        }
        if (s_subs[i].expires_us <= now && free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot >= 0) {
        s_subs[free_slot].ip         = ip;
        s_subs[free_slot].port       = port;
        s_subs[free_slot].expires_us = expires;
    }
    xSemaphoreGive(s_subs_lock);
}

static esp_err_t subscribe_handler(httpd_req_t *req)
{
    char body[64];
    int  len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing body");
        return ESP_FAIL;
    }
    body[len] = 0;

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_FAIL;
    }
    cJSON *port_node = cJSON_GetObjectItemCaseSensitive(root, "port");
    if (!cJSON_IsNumber(port_node) ||
        port_node->valueint < 1 || port_node->valueint > 65535) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing or invalid port");
        return ESP_FAIL;
    }
    uint16_t port = (uint16_t)port_node->valueint;
    cJSON_Delete(root);

    int sockfd = httpd_req_to_sockfd(req);
    struct sockaddr_in6 peer;
    socklen_t plen = sizeof(peer);
    if (getpeername(sockfd, (struct sockaddr *)&peer, &plen) < 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "getpeername");
        return ESP_FAIL;
    }

    in_addr_t ip;
    if (peer.sin6_family == AF_INET) {
        ip = ((struct sockaddr_in *)&peer)->sin_addr.s_addr;
    } else {
        // IPv4-mapped IPv6 — the v4 address is the last 4 bytes
        memcpy(&ip, &peer.sin6_addr.s6_addr[12], 4);
    }

    add_or_refresh(ip, port);

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &ip, addr_str, sizeof(addr_str));
    ESP_LOGI(TAG, "subscriber %s:%u (TTL %lld ms)",
             addr_str, port, SUBSCRIBE_TTL_US / 1000);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static const httpd_uri_t subscribe_uri = {
    .uri     = "/subscribe",
    .method  = HTTP_POST,
    .handler = subscribe_handler,
};

esp_err_t udp_push_start(void)
{
    s_subs_lock = xSemaphoreCreateMutex();
    if (!s_subs_lock) return ESP_ERR_NO_MEM;

    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "udp socket: errno %d", errno);
        return ESP_FAIL;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_httpd, &cfg);
    if (err != ESP_OK) return err;

    err = httpd_register_uri_handler(s_httpd, &subscribe_uri);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "ready: POST /subscribe {\"port\":N} → UDP push of [w,x,y,z] float32 LE");
    return ESP_OK;
}

httpd_handle_t udp_push_get_httpd(void)
{
    return s_httpd;
}

void udp_push_send_quat(float w, float x, float y, float z)
{
    if (s_udp_sock < 0) return;

    float buf[4] = { w, x, y, z };
    int64_t now = esp_timer_get_time();

    xSemaphoreTake(s_subs_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (s_subs[i].expires_us <= now) continue;
        struct sockaddr_in dst = {
            .sin_family = AF_INET,
            .sin_port   = htons(s_subs[i].port),
            .sin_addr   = { .s_addr = s_subs[i].ip },
        };
        sendto(s_udp_sock, buf, sizeof(buf), 0,
               (struct sockaddr *)&dst, sizeof(dst));
    }
    xSemaphoreGive(s_subs_lock);
}
