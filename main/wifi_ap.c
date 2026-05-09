#include "wifi_ap.h"

#include <string.h>

#include <arpa/inet.h>

#include "dhcpserver/dhcpserver.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"

#include "wifi_credentials.h"

static const char *TAG = "wifi_ap";

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "client joined " MACSTR " (aid=%d)", MAC2STR(e->mac), e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGW(TAG, "client left " MACSTR " (aid=%d)", MAC2STR(e->mac), e->aid);
    }
}

esp_err_t wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Stop DHCP, set our static AP address, restrict the lease pool to a
    // single IP (CLIENT_IP), then restart DHCP. The single connecting client
    // always gets CLIENT_IP this way — no handshake needed.
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    esp_netif_ip_info_t ip;
    ip.ip.addr      = ipaddr_addr(ESP_IP);
    ip.netmask.addr = ipaddr_addr(ESP_NETMASK);
    ip.gw.addr      = ipaddr_addr(ESP_IP);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip));

    // The DHCP server requires start_ip < end_ip, so we expose a 2-IP pool
    // starting at CLIENT_IP. With max_connection=1, the (only) client always
    // grabs start_ip — i.e. CLIENT_IP — so we still have a stable assignment.
    uint32_t start_n = ipaddr_addr(CLIENT_IP);
    uint32_t end_n   = htonl(ntohl(start_n) + 1);

    dhcps_lease_t lease;
    memset(&lease, 0, sizeof(lease));
    lease.enable        = true;
    lease.start_ip.addr = start_n;
    lease.end_ip.addr   = end_n;
    ESP_ERROR_CHECK(esp_netif_dhcps_option(ap_netif, ESP_NETIF_OP_SET,
                                           ESP_NETIF_REQUESTED_IP_ADDRESS,
                                           &lease, sizeof(lease)));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_event, NULL, NULL));

    wifi_config_t wcfg = {
        .ap = {
            .channel        = WIFI_CHANNEL,
            .max_connection = 1,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg        = { .required = false },
        },
    };
    strlcpy((char *)wcfg.ap.ssid,     WIFI_SSID,     sizeof(wcfg.ap.ssid));
    strlcpy((char *)wcfg.ap.password, WIFI_PASSWORD, sizeof(wcfg.ap.password));
    wcfg.ap.ssid_len = strlen(WIFI_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "AP '%s' up on channel %d, ESP=%s, client=%s",
             WIFI_SSID, WIFI_CHANNEL, ESP_IP, CLIENT_IP);
    return ESP_OK;
}
