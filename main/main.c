#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "esp_crc.h"
#include "esp_system.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define ADV_START_COMPLETE (1 << 0)
#define ADV_STOP_COMPLETE (1 << 1)

static const char *TAG = "BLUETALK";

static QueueHandle_t adv_queue;
static EventGroupHandle_t evt_handle;

static const esp_ble_scan_params_t scan_params_default = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x50,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static const esp_ble_adv_data_t adv_data_default = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = false,
    .min_interval = 0x0000,
    .max_interval = 0x0000,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static const esp_ble_adv_params_t adv_params_default = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x28,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define STDIN_BUF_LEN (2 + 31 * 2 + 2 + 2 + 1 + 1)
char stdin_buf[STDIN_BUF_LEN] = {};
char hex_char[] = "0123456789abcdef";
char bda_buf[13] = {0};
char mfr_buf[53] = {0};

char *u8_to_hex(uint8_t *data, size_t data_len, char *buf, size_t buf_len) {
    memset(buf, 0, buf_len);
    if (data_len == 0)
        return (char *)buf;
    for (int i = 0; i < data_len; i++) {
        buf[i * 2] = hex_char[data[i] / 16];
        buf[i * 2 + 1] = hex_char[data[i] % 16];
    }
    return (char *)buf;
}

bool hex_to_u8(char high, char low, uint8_t *u8) {
    if (low >= '0' && low <= '9') {
        *u8 = low - '0';
    } else if (low >= 'a' && low <= 'f') {
        *u8 = low - 'a' + 10;
    } else if (low >= 'A' && low <= 'F') {
        *u8 = low - 'A' + 10;
    } else {
        return false;
    }

    if (high >= '0' && high <= '9') {
        *u8 += 16 * (high - '0');
    } else if (high >= 'a' && high <= 'f') {
        *u8 += 16 * (high - 'a' + 10);
    } else if (high >= 'A' && high <= 'F') {
        *u8 += 16 * (high - 'A' + 10);
    } else {
        return false;
    }
    return true;
}

bool hex_str_to_u8s(char *str, uint8_t *data) {
    int l = strlen(str);
    if (l % 2 || l / 2 > 26)
        return false;

    for (int i = 0; i < l / 2; i++) {
        if (!hex_to_u8(str[i * 2], str[i * 2 + 1], &data[i + 1]))
            return false;
    }

    data[0] = l / 2;
    return true;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
    uint8_t *mfr_data;
    uint8_t mfr_data_len;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        ESP_LOGI(TAG, "starting ble scan (permanently)");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            mfr_data = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &mfr_data_len);

            if (mfr_data_len > 0) {
                // b01bca57
                if (mfr_data[0] != 0xb0 || mfr_data[1] != 0x1b ||
                    mfr_data[2] != 0xca || mfr_data[3] != 0x57)
                    return;

                char *bda_str = u8_to_hex(scan_result->scan_rst.bda, 6, bda_buf,
                                          sizeof(bda_buf));
                char *mfr_str =
                    u8_to_hex(mfr_data, mfr_data_len, mfr_buf, sizeof(mfr_buf));
                printf("BULBCAST %s %s\n", bda_str, mfr_str);
            }
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "ble scan started");
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        esp_ble_adv_params_t adv_params = adv_params_default;
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    } break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        xEventGroupSetBits(evt_handle, ADV_START_COMPLETE);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        xEventGroupSetBits(evt_handle, ADV_STOP_COMPLETE);
        break;
    default:
        ESP_LOGI(TAG, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

void ble_adv_scan(void *pvParams) {
    uint8_t mfr[27] = {0};
    evt_handle = xEventGroupCreate();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    esp_ble_scan_params_t scan_params = scan_params_default;
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));

    while (1) {
        if (xQueueReceive(adv_queue, mfr, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Queue receive error");
            continue;
        }

        esp_ble_adv_data_t adv_data = adv_data_default;
        adv_data.p_manufacturer_data = &mfr[1];
        adv_data.manufacturer_len = mfr[0];
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

        // wait advertising started
        xEventGroupWaitBits(evt_handle, ADV_START_COMPLETE, pdFALSE, pdFALSE,
                            0);
        xEventGroupClearBits(evt_handle, ADV_START_COMPLETE);
        ESP_LOGI(TAG, "advertising started");

        vTaskDelay(50 / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());

        // wait advertising started
        xEventGroupWaitBits(evt_handle, ADV_STOP_COMPLETE, pdFALSE, pdFALSE, 0);
        xEventGroupClearBits(evt_handle, ADV_STOP_COMPLETE);
        ESP_LOGI(TAG, "advertising stopped");
    }
}

void app_main(void) {
    adv_queue = xQueueCreate(40, 32);

    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, NULL, 6, NULL);

    // Initialize VFS & UART so we can use std::cout/cin
    setvbuf(stdin, NULL, _IONBF, 0);
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(
        (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CRLF);

    while (1) {
        // read a line from stdin
        char *str = fgets(stdin_buf, STDIN_BUF_LEN, stdin);
        if (str == NULL) {
            ESP_LOGE(TAG, "fgets returns NULL");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        if (str[strlen(str) - 1] != '\n') {
            ESP_LOGI(TAG, "from stdin: %s", str);
            ESP_LOGE(TAG, "string not ending with newline (too large?)");
            continue;
        }

        str[strlen(str) - 1] = '\0';
        ESP_LOGI(TAG, "from stdin: %s", str);

        uint8_t msg[32] = {0};
        if (!hex_str_to_u8s(str, msg)) {
            ESP_LOGE(TAG, "not valid hex string");
            continue;
        }

        esp_log_buffer_hex(TAG, msg, msg[0] + 1);
        xQueueSend(adv_queue, msg, portMAX_DELAY); 
    }
}
