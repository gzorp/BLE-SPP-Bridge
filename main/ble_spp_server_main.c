/*
 * ble_spp_server_main.c
 * Goals:
 *  Get some practical hands-on experience with Git, Bluetooth, USB, UART, ESP-IDF framework, ESP32-S3.
 *  Brush-up C programming (It's been quite a while since I last wrote any C code)
 *  Send gcode data to Ender3 3D Printer via bluetooth without requiring hardware modification.
 *  Sub-Goal:
 *    Investigate\Confirm bandwidth capability of BLE->USB Serial using esp32-S3
 */

#include "ble_spp_server.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "tusb.h"
#include "tusb_cdc_acm.h"

static const char *TAG = "BLE_SPP_BRIDGE";

static uint16_t ble_spp_svc_gatt_read_val_handle;
QueueHandle_t spp_common_uart_queue = NULL;
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
void ForwardToUSBSerial(const struct os_mbuf *mbuf, int len, void *data);
static void ble_spp_server_advertise(void);
static uint8_t my_address_type;

static bool usb_serial_initialized = false;

void cdc_rx_callback(int itf, cdcacm_event_t *event) {
    // This callback is for handling incoming data
    ESP_LOGW(TAG, "cdc_rx_callback not implemented!");
}

void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event) {
    // This callback is for handling line state changes (e.g., DTR/RTS)
    ESP_LOGW(TAG, "cdc_line_state_changed_callback not implemented!");
}

// Callback for BLE GATT events (Read/Write)
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *context, void *arg) {
    switch (context->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "GATT operation BLE_GATT_ACCESS_OP_READ_CHR being performed. GATT attribute access operation: Read characteristic.");
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            ESP_LOGI(TAG, "GATT attribute access operation: Write characteristic");
            if (context->om) {  // Check if data was written
                uint8_t *data = (uint8_t *)malloc(context->om->om_len);
                os_mbuf_copydata(context->om, 0, context->om->om_len, data);
                ESP_LOGI(TAG, "BLE_GATT_ACCESS_OP_WRITE_CHR: Copied received data: %.*s\n", context->om->om_len, data);

                ForwardToUSBSerial(context->om, context->om->om_len, data);
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "Unhandled GATT operation: %d", context->op);
            break;
    }
    return 0;
}

// Function to forward data to USB serial
void forward_data_to_usb(const uint8_t *data, int len) {
    size_t written = 0;
    while (written < len) {
        size_t available = tud_cdc_write_available();
        if (available == 0) {
            tud_task();  // Handle any pending USB events
            continue;
        }
        size_t to_write = (len - written) < available ? (len - written) : available;
        size_t just_written = tud_cdc_write(data + written, to_write);
        written += just_written;
        tud_cdc_write_flush();
    }

    if (written != len) {
        ESP_LOGW(TAG, "Partial write to USB Serial: %d/%d bytes", written, len);
    } else {
        ESP_LOGI(TAG, "Successfully wrote %d bytes to USB Serial", written);
    }
}

void init_usb_serial() {
    ESP_LOGI(TAG, "Initializing USB Serial");
    // We currently reach at least here
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_LOGI(TAG, "Installing driver");
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {.usb_dev = TINYUSB_USBDEV_0,
                                       .cdc_port = TINYUSB_CDC_ACM_0,
                                       .rx_unread_buf_sz = 64,
                                       .callback_rx = &cdc_rx_callback,
                                       .callback_rx_wanted_char = NULL,
                                       .callback_line_state_changed = &cdc_line_state_changed_callback,
                                       .callback_line_coding_changed = NULL};

    ESP_LOGI(TAG, "Initializing CDC ACM");
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_LOGI(TAG, "CDC ACM Initialized");
    usb_serial_initialized = true;
    ESP_LOGI(TAG, "USB Serial initialized");
}

void ForwardToUSBSerial(const struct os_mbuf *mbuf, int len, void *data) {
    if (!usb_serial_initialized) {
        init_usb_serial();
    }

    if (!tud_cdc_connected()) {
        ESP_LOGW(TAG, "USB Serial is not connected");
        return;
    }
    ESP_LOGI(TAG, "Successfully connected to USB Serial");

    forward_data_to_usb((const uint8_t *)data, len);
}

/* Nordic UART service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        // Nordic UART Service Service UUID aka: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
        .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    // Nordic UART Service (NUS) RX Characteristic UUID aka: 6e400002-b5a3-f393-e0a9-e50e24dcca9e
                    .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E),
                    .access_cb = ble_svc_gatt_handler,
                    .val_handle = &ble_spp_svc_gatt_read_val_handle,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    // Nordic UART Service TX Characteristic UUID aka: 6e400003-b5a3-f393-e0a9-e50e24dcca9e
                    .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E),
                    .access_cb = ble_svc_gatt_handler,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                },
                {
                    0, /* No more characteristics in this service */
                }},
    },
    {
        0, /* No more services */
    },
};

static void gatt_svr_register_callback(struct ble_gatt_register_ctxt *context, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (context->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            printf("Registered service %s with handle = %d\n", ble_uuid_to_str(context->svc.svc_def->uuid, buf), context->svc.handle);
            break;

        case BLE_GATT_REGISTER_OP_CHR:
            printf("Registering characteristic %s with def_handle = %d val_handle=%d\n", ble_uuid_to_str(context->chr.chr_def->uuid, buf),
                   context->chr.def_handle, context->chr.val_handle);
            break;

        case BLE_GATT_REGISTER_OP_DSC:
            printf("Registering descriptor %s with handle=%d\n", ble_uuid_to_str(context->dsc.dsc_def->uuid, buf), context->dsc.handle);
            break;

        default:
            assert(0);
            break;
    }
}

int gatt_svr_init(void) {
    int ret = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ret = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (ret != 0) {
        return ret;
    }

    ret = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (ret != 0) {
        return ret;
    }

    return 0;
}

void ble_server_uart_task(void *pvParameters) {
    printf("ble_server_uart_task started\n");
    uart_event_t event;
    int ret = 0;

    for (;;) {
        // Waiting for UART event.
        if (xQueueReceive(spp_common_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    if (event.size) {
                        uint8_t *ntf;
                        ntf = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
                        memset(ntf, 0x00, event.size);

                        uart_read_bytes(UART_NUM_0, ntf, event.size, portMAX_DELAY);
                        printf("UART Received data: %.*s\n", event.size, ntf);

                        for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
                            // Check if client has subscribed to notifications
                            if (conn_handle_subs[i]) {
                                struct os_mbuf *txom;
                                txom = ble_hs_mbuf_from_flat(ntf, event.size);
                                ret = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle, txom);
                                if (ret == 0) {
                                    MODLOG_DFLT(INFO, "Notification sent successfully");
                                } else {
                                    MODLOG_DFLT(ERROR, "Error in sending notification ret = %d", ret);
                                }
                            }
                        }

                        free(ntf);
                    }
                    break;
                default:
                    break;
            }
        }
    }
    vTaskDelete(NULL);
    printf("ble_server_uart_task deleted\n");
}

static void ble_spp_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_common_uart_queue, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(ble_server_uart_task, "uTask", 4096, (void *)UART_NUM_0, 8, NULL);
}

// GAP event handler (modified to start advertising)
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE GAP EVENT CONNECT: status=%d conn_handle=%d\n", event->connect.status, event->connect.conn_handle);
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT: reason=%d conn_handle=%d\n", event->disconnect.reason, event->disconnect.conn.conn_handle);
            ble_spp_server_advertise();  // Restart advertising after disconnect
            break;

        default:
            break;
    }
    return 0;
}

static void ble_spp_server_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int ret;

    uint8_t *name = (uint8_t *)ble_svc_gap_device_name();  // API caps length at 31 chars
    uint8_t name_len = strlen((const char *)name);

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = name;
    fields.name_len = name_len;
    fields.name_is_complete = 1;
    fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    ret = ble_gap_adv_set_fields(&fields);

    if (ret != 0) {
        MODLOG_DFLT(ERROR, "Error setting advertisement data; ret = %d\n", ret);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ret = ble_gap_adv_start(my_address_type, NULL, BLE_HS_FOREVER, &adv_params, ble_spp_server_gap_event, NULL);
    if (ret != 0) {
        MODLOG_DFLT(ERROR, "Error enabling advertisement; ret = %d\n", ret);
        return;
    }
}

static void on_reset_callback(int reason) {
    // Prevent clang format from converting this to a one liner
    MODLOG_DFLT(INFO, "Resetting state; reason = %d\n", reason);
}

static void on_sync_callback(void) {
    int ret;

    ret = ble_hs_util_ensure_addr(0);
    assert(ret == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    ret = ble_hs_id_infer_auto(0, &my_address_type);
    if (ret != 0) {
        MODLOG_DFLT(ERROR, "Error determining address type; ret = %d\n", ret);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    ret = ble_hs_id_copy_addr(my_address_type, addr_val, NULL);

    printf("Device Address: ");
    print_addr(addr_val);
    printf("\n");
    /* Begin advertising. */
    ble_spp_server_advertise();
}

void ble_spp_server_host_task(void *param) {
    printf("BLE Host Task Started\n");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void app_main(void) {
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t esp_error = nvs_flash_init();
    int ret = 0;

    if (esp_error == ESP_ERR_NVS_NO_FREE_PAGES || esp_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        esp_error = nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_error);

    ret = nimble_port_init();

    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }

    /* Initialize connection_handle array */
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    ble_spp_uart_init();

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = on_reset_callback;
    ble_hs_cfg.sync_cb = on_sync_callback;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_callback;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = CONFIG_EXAMPLE_IO_TYPE;
    ble_hs_cfg.sm_sc = 0;

    /* Register custom service */
    ret = gatt_svr_init();
    assert(ret == 0);

    /* Set the default device name. */
    ret = ble_svc_gap_device_name_set("BLE_SPP_SVR");
    assert(ret == 0);

    nimble_port_freertos_init(ble_spp_server_host_task);
}