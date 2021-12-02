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

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define BIT_0 (1 << 0)  // config_adv_data
#define BIT_1 (1 << 1)  // start advertising
#define BIT_2 (1 << 2)  // stop advertising

static const char *tag = "BLUETALK";

static EventGroupHandle_t eg_handle;
static StaticEventGroup_t eg_data;

static uint8_t out_mfr_data[32] = {};

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
    .min_interval = 0x0006, // 7.5ms
    .max_interval = 0x000c, // 15ms
    .appearance = 0x00,
    .manufacturer_len = 20,                  // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = &out_mfr_data[0], //&test_manufacturer[0],
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

/** prefix 2 bytes (0x)
    payload min 2 bytes, max 31 * 2 bytes,
    crc8, 2 bytes,
    new line, 1 byte,
    null-termination, 1 byte */

#define STDIN_BUF_LEN (2 + 31 * 2 + 2 + 2 + 1 + 1)
char stdin_buf[STDIN_BUF_LEN] = {};
uint8_t outgoing_buf[32] = {};

uint8_t hex_char[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

uint8_t bda_buf[16] = {};
uint8_t mfr_buf[64] = {};
uint8_t all_buf[128] = {};

char *u8_to_hex(uint8_t *buf, size_t buf_len, uint8_t *data, size_t data_len) {
    memset(buf, 0, buf_len);
    if (data_len == 0)
        return (char *)buf;
    for (int i = 0; i < data_len; i++) {
        buf[i * 2] = hex_char[data[i] / 16];
        buf[i * 2 + 1] = hex_char[data[i] % 16];
    }
    return (char *)buf;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
    uint8_t *mfr_data;
    uint8_t mfr_data_len;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        ESP_LOGI(tag, "starting ble scan (permanently)");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            mfr_data = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &mfr_data_len);

            if (mfr_data_len > 0) {
                char *bda_str = u8_to_hex(bda_buf, sizeof(bda_buf),
                                          scan_result->scan_rst.bda, 6);
                char *mfr_str =
                    u8_to_hex(mfr_buf, sizeof(mfr_buf), mfr_data, mfr_data_len);

                printf("BTRECV ADDR: %s, MFR: %s\n", bda_str, mfr_str);
            }
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(tag, "ble scan started");
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, BIT_0);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, BIT_1);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, BIT_2);
        break;
    default:
        ESP_LOGI(tag, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

bool hex_to_u8(char high, char low, uint8_t *u8) {
    if (low >= '0' && low <= '9') {
        *u8 = low - '0';
    } else if (low >= 'a' && low <= 'f') {
        *u8 = low - 'a' + 10;
    } else {
        return false;
    }

    if (high >= '0' && high <= '9') {
        *u8 += 16 * (high - '0');
    } else if (high >= 'a' && high <= 'f') {
        *u8 += 16 * (high - 'a' + 10);
    } else {
        return false;
    }
    return true;
}

bool hex_str_to_u8s(char *str, uint8_t *data, size_t *data_len) {
    int l = strlen(str);
    if (l % 2)
        return false;

    for (int i = 0; i < l / 2; i++) {
        if (!hex_to_u8(str[i * 2], str[i * 2 + 1], &data[i]))
            return false;
    }

    *data_len = l / 2;
    return true;
}

void app_main(void) {
    // init (a)synchronizer
    eg_handle = xEventGroupCreateStatic(&eg_data);

    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Initialize VFS & UART so we can use std::cout/cin
    setvbuf(stdin, NULL, _IONBF, 0);
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(
        (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CRLF);

    // this is not yet implemented for c3.
    // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // TODO add error checking
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bt_sleep_disable();

    esp_bluedroid_init();
    esp_bluedroid_enable();

    err = esp_ble_gap_register_callback(esp_gap_cb);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "error registerring gap callback: %s",
                 esp_err_to_name(err));
        vTaskDelay(1000);
        return;
    }

    esp_ble_scan_params_t scan_params = scan_params_default;
    esp_ble_gap_set_scan_params(&scan_params);

    while (1) {
        // read a line from stdin
        char *str = fgets(stdin_buf, STDIN_BUF_LEN, stdin);
        if (str == NULL) {
            ESP_LOGE(tag, "fgets returns NULL");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        if (str[strlen(str) - 1] != '\n') {
            ESP_LOGI(tag, "from stdin: %s", str);
            ESP_LOGE(tag, "string not ending with newline (too large?)");
            continue;
        }

        str[strlen(str) - 1] = '\0';
        ESP_LOGI(tag, "from stdin: %s", str);

        size_t data_len = 0;
        if (!hex_str_to_u8s(str, outgoing_buf, &data_len)) {
            ESP_LOGE(tag, "not valid hex string");
            continue;
        }

        esp_log_buffer_hex(tag, outgoing_buf, data_len);

        esp_ble_adv_data_t adv_data = adv_data_default;
        adv_data.p_manufacturer_data = &outgoing_buf[0];
        adv_data.manufacturer_len = data_len;
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(tag, "config adv data failed, error code = %x", ret);
            continue;
        }

        // wait config done
        xEventGroupWaitBits(eg_handle, BIT_0, pdFALSE, pdFALSE, 0);
        xEventGroupClearBits(eg_handle, BIT_0);

        esp_ble_adv_params_t adv_params = adv_params_default;
        err = esp_ble_gap_start_advertising(&adv_params);
        if (ret) {
            ESP_LOGE(tag, "error starting advertising: %s",
                     esp_err_to_name(err));
            continue;
        }

        // wait advertising started
        xEventGroupWaitBits(eg_handle, BIT_1, pdFALSE, pdFALSE, 0);
        xEventGroupClearBits(eg_handle, BIT_1);

        vTaskDelay(50 / portTICK_PERIOD_MS);

        err = esp_ble_gap_stop_advertising();
        if (err) {
            ESP_LOGE(tag, "error stopping advertising: %s",
                     esp_err_to_name(err));
            continue;
        }

        // wait advertising started
        xEventGroupWaitBits(eg_handle, BIT_2, pdFALSE, pdFALSE, 0);
        xEventGroupClearBits(eg_handle, BIT_2);
        ESP_LOGI(tag, "advertising stopped");
    }
}
