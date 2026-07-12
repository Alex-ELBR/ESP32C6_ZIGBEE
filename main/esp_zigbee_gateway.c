/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * ЧАСТЬ 1: БАЗА ДАННЫХ ДАТЧИКОВ TUYA И ВЫЧИСЛЕНИЕ МИНИМАЛЬНОЙ ТЕМПЕРАТУРЫ
 */
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"

static const char *TAG = "ESP_ZB_GATEWAY";

// Маска радиоканалов Zigbee (строго ограничиваем: 11, 15, 20, 25) под датчики Tuya
#define ESP_ZB_PRIMARY_CHANNEL_MASK      (1u << 11 | 1u << 15 | 1u << 20 | 1u << 25)
#define MAX_SENSORS 10

typedef struct {
    uint16_t short_addr; // Сетевой адрес датчика (0x0000, если слот пуст)
    float temp;          // Температура
    float hum;           // Влажность
    bool has_temp;       // Флаг получения данных
} zigbee_sensor_t;

static zigbee_sensor_t sensors[MAX_SENSORS];

// Функция перерасчета и вывода минимальной температуры в лог
void process_and_log_minimum(void)
{
    float min_temp = 99.0;
    uint16_t min_addr = 0;
    float associated_hum = 0.0;
    int active_sensors_count = 0;
    bool found_any_temp = false;

    for (int i = 0; i < MAX_SENSORS; i++) {
        if (sensors[i].short_addr != 0x0000) {
            active_sensors_count++;
            if (sensors[i].has_temp) {
                if (sensors[i].temp < min_temp) {
                    min_temp = sensors[i].temp;
                    min_addr = sensors[i].short_addr;
                    associated_hum = sensors[i].hum;
                    found_any_temp = true;
                }
            }
        }
    }

    ESP_LOGW("SYSTEM_MONITOR", "==================================================");
    ESP_LOGI("SYSTEM_MONITOR", "Датчиков на связи (Sensors online): %d", active_sensors_count);
    
    if (found_any_temp) {
        ESP_LOGE("SYSTEM_MONITOR", "РЕЗУЛЬТАТ -> МИНИМАЛЬНАЯ ТЕМПЕРАТУРА: %.2f C", min_temp);
        ESP_LOGE("SYSTEM_MONITOR", "Самый холодный датчик (Address): [0x%04X]", min_addr);
        ESP_LOGI("SYSTEM_MONITOR", "Влажность на этом датчике (Humidity): %.2f %%", associated_hum);
    } else {
        ESP_LOGI("SYSTEM_MONITOR", "Сеть пуста. Ожидание отчетов от Tuya ZBTH3...");
    }
    ESP_LOGW("SYSTEM_MONITOR", "==================================================");
}

// Приёмник атрибутов Zigbee сети
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
        esp_zb_zcl_report_attr_message_t *report_msg = (esp_zb_zcl_report_attr_message_t *)message;
        uint16_t src_addr = report_msg->src_address.u.short_addr;
        uint16_t cluster_id = report_msg->cluster;
        uint16_t attribute_id = report_msg->attribute.id;
        
        int slot = -1;
        for (int i = 0; i < MAX_SENSORS; i++) {
            if (sensors[i].short_addr == src_addr) { slot = i; break; }
        }
        if (slot == -1) {
            for (int i = 0; i < MAX_SENSORS; i++) {
                if (sensors[i].short_addr == 0x0000) { sensors[i].short_addr = src_addr; slot = i; break; }
            }
        }

        if (slot != -1) {
            // ВЛАЖНОСТЬ
            if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT && 
                attribute_id == ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID) {
                sensors[slot].hum = *(uint16_t *)report_msg->attribute.data.value / 100.0;
                ESP_LOGI(TAG, "[0x%04X] Принята влажность: %.2f %%", src_addr, sensors[slot].hum);
                process_and_log_minimum();
            } 
            // ТЕМПЕРАТУРА
            else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT && 
                     attribute_id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
                sensors[slot].temp = *(int16_t *)report_msg->attribute.data.value / 100.0;
                sensors[slot].has_temp = true;
                ESP_LOGI(TAG, "[0x%04X] Принята температура: %.2f C", src_addr, sensors[slot].temp);
                process_and_log_minimum();
            }
        }
    }
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}
/*
 * ЧАСТЬ 2: АВТОМАТ СЕТЕВЫХ СИГНАЛОВ С БЕЗОПАСНЫМ СБРОСОМ И СТАРТ СИСТЕМЫ
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Старт инициализации Zigbee...");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            // ИСПРАВЛЕННЫЙ УМНЫЙ СБРОС: Проверяем статус сети прямо внутри системного сигнала
            if (esp_zb_bdb_is_factory_new()) {
                // Если сеть чистая — запускаем её формирование на каналах Tuya
                ESP_LOGI(TAG, "Формирование новой чистой Zigbee сети...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                // Если стек пытается восстановить старую сеть — принудительно перехватываем это,
                // стираем кэш и уходим в одну перезагрузку. После ребута сеть станет factory_new!
                ESP_LOGE(TAG, "==================================================");
                ESP_LOGE(TAG, "Перехват старой сети из NVS! Выполняется сброс...");
                ESP_LOGE(TAG, "==================================================");
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_zb_factory_reset(); 
            }
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Сеть координатора успешно создана!");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGW(TAG, "ЭФИР ОТКРЫТ! Переведите ваши датчики Tuya ZBTH3 в режим сопряжения...");
            esp_zb_bdb_open_network(180); 
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGW(TAG, "!!! Обнаружен новый датчик! Адрес в сети: [0x%04hx]", dev_annce_params->device_short_addr);
        for (int i = 0; i < MAX_SENSORS; i++) {
            if (sensors[i].short_addr == dev_annce_params->device_short_addr) break;
            if (sensors[i].short_addr == 0x0000) {
                sensors[i].short_addr = dev_annce_params->device_short_addr;
                process_and_log_minimum();
                break;
            }
        }
        break;
    default:
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    memset(sensors, 0, sizeof(sensors));
    
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false 
    };
    esp_zb_init(&zb_nwk_cfg);
    
    // Принудительно меняем PAN ID на уникальный, чтобы датчик не путал его со старыми сетями
    esp_zb_set_pan_id(0x4A5B);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID,
    };

    esp_zb_attribute_list_t *basic_cluser = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "\x09\x45\x73\x70\x72\x65\x73\x73\x69\x66");
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, "\x0c\x45\x53\x50\x33\x32\x43\x36\x2d\x47\x57");
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    esp_zb_attribute_list_t *tuya_custom_cluster = esp_zb_zcl_attr_list_create(0xEF00);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, tuya_custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_add_gateway_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

    uint8_t tuya_link_key[] = {0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, 0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39};
    esp_zb_secur_ic_set(0, tuya_link_key);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&(usb_serial_jtag_driver_config_t)USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT()));
    usb_serial_jtag_vfs_use_driver();
#endif
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
