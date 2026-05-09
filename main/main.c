#include <math.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#include "camera_stream.h"
#include "flight_control.h"
#include "udp_push.h"
#include "wifi_ap.h"
#include "wifi_credentials.h"

#define I2C_SDA_GPIO    5
#define I2C_SCL_GPIO    6
#define I2C_PORT        I2C_NUM_0
#define I2C_CLOCK_HZ    400000

static const char *TAG = "main";

static i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_GPIO,
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_io_num = I2C_SCL_GPIO,
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .master.clk_speed = I2C_CLOCK_HZ,
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
};

static icm0948_config_i2c_t icm_cfg = {
    .i2c_port = I2C_PORT,
    .i2c_addr = ICM_20948_I2C_ADDR_AD0,
};

static void enable_dmp_quat9(icm20948_device_t *icm)
{
    bool ok = true;
    ok &= (icm20948_init_dmp_sensor_with_defaults(icm) == ICM_20948_STAT_OK);
    ok &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_STAT_OK);
    ok &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat9, 0) == ICM_20948_STAT_OK);
    ok &= (icm20948_enable_fifo(icm, true) == ICM_20948_STAT_OK);
    ok &= (icm20948_enable_dmp(icm, true) == ICM_20948_STAT_OK);
    ok &= (icm20948_reset_dmp(icm) == ICM_20948_STAT_OK);
    ok &= (icm20948_reset_fifo(icm) == ICM_20948_STAT_OK);

    if (!ok) {
        ESP_LOGE(TAG, "DMP enable failed");
        abort();
    }
    ESP_LOGI(TAG, "DMP enabled (Quat9 / 9-axis)");
}

static void fusion_task(void *arg)
{
    icm20948_device_t *icm = (icm20948_device_t *)arg;

    int64_t next_log = esp_timer_get_time() + 1000000;
    int16_t last_accuracy = -1;

    while (1) {
        icm_20948_DMP_data_t data;
        icm20948_status_e st = inv_icm20948_read_dmp_data(icm, &data);

        if (st == ICM_20948_STAT_OK || st == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            if (data.header & DMP_header_bitmap_Quat9) {
                // Quaternion is scaled by 2^30; Q0 derived from Q1²+Q2²+Q3²+Q0²=1
                double q1 = data.Quat9.Data.Q1 / 1073741824.0;
                double q2 = data.Quat9.Data.Q2 / 1073741824.0;
                double q3 = data.Quat9.Data.Q3 / 1073741824.0;
                double s  = 1.0 - (q1 * q1 + q2 * q2 + q3 * q3);
                double q0 = s > 0.0 ? sqrt(s) : 0.0;

                udp_push_send_quat((float)q0, (float)q1, (float)q2, (float)q3);
                last_accuracy = data.Quat9.Data.Accuracy;
            }
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_log) {
            // Quat9 heading uncertainty in Q12 radians: lower = better.
            // Move IMU in figure-8 to drive it down; <100 (~1.4°) is well-calibrated.
            float deg = last_accuracy * (180.0f / 3.14159265f) / 4096.0f;
            ESP_LOGI(TAG, "heading uncertainty: raw=%d (~%.2f deg)",
                     last_accuracy, deg);
            next_log = now + 1000000;
        }

        if (st != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void imu_setup(icm20948_device_t *icm)
{
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, i2c_conf.mode, 0, 0, 0));

    icm20948_init_i2c(icm, &icm_cfg);

    while (icm20948_check_id(icm) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "ICM-20948 WHO_AM_I check failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "ICM-20948 detected");

    icm20948_sw_reset(icm);
    vTaskDelay(pdMS_TO_TICKS(250));

    icm20948_internal_sensor_id_bm sensors =
        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
    icm20948_set_sample_mode(icm, sensors, SAMPLE_MODE_CONTINUOUS);

    icm20948_fss_t fss = { .a = GPM_2, .g = DPS_250 };
    icm20948_set_full_scale(icm, sensors, fss);

    icm20948_dlpcfg_t dlp = { .a = ACC_D473BW_N499BW, .g = GYR_D361BW4_N376BW5 };
    icm20948_set_dlpf_cfg(icm, sensors, dlp);
    icm20948_enable_dlpf(icm, ICM_20948_INTERNAL_ACC, false);
    icm20948_enable_dlpf(icm, ICM_20948_INTERNAL_GYR, false);

    icm20948_sleep(icm, false);
    icm20948_low_power(icm, false);

    // The DMP defaults configure the chip's internal I2C master to read the
    // AK09916 magnetometer, but never actually enable that master. Without
    // this, no mag samples reach the DMP and Quat9 accuracy stays at 0.
    if (icm20948_i2c_master_passthrough(icm, false) != ICM_20948_STAT_OK ||
        icm20948_i2c_master_enable(icm, true)        != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "I2C master enable failed");
        abort();
    }

    enable_dmp_quat9(icm);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    static icm20948_device_t icm;
    imu_setup(&icm);

    xTaskCreate(fusion_task, "fusion", 8192, &icm, 5, NULL);

    ESP_ERROR_CHECK(wifi_ap_start());
    ESP_ERROR_CHECK(udp_push_start());
    ESP_ERROR_CHECK(flight_control_start());

    ESP_ERROR_CHECK(camera_stream_init());
    ESP_ERROR_CHECK(camera_stream_start());

    printf("AP '%s' ready (ESP=%s, client=%s)\n", WIFI_SSID, ESP_IP, CLIENT_IP);
    printf("Orientation:    UDP  %s -> %s:%u\n", ESP_IP, CLIENT_IP, ORIENTATION_PORT);
    printf("Video:          UDP  %s -> %s:%u\n", ESP_IP, CLIENT_IP, VIDEO_PORT);
    printf("Flight control: UDP  %s -> %s:%u\n", CLIENT_IP, ESP_IP, FLIGHT_PORT);
}
