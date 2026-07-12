/*
 * Файл: main/tasks/zigbee_task.c
 * Изолированная задача для работы со стеком Zigbee (ESP-IDF v6.0)
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"       // Ядро Zigbee
#include "zcl/esp_zigbee_zcl_common.h"
#include "zigbee_task.h"

// Настройки параметров шлюза
#define ESP_ZB_GATEWAY_ENDPOINT          1
#define ESP_MANUFACTURER_NAME            "\x09\x45\x73\x70\x72\x65\x73\x73\x69\x66"
#define ESP_MODEL_IDENTIFIER             "\x0c\x45\x53\x50\x33\x32\x43\x36\x2d\x47\x57"

// Стандартная маска радиоканалов (Разрешаем каналы 11, 15, 20, 25 в диапазоне 2.4 ГГц)
#define ESP_ZB_PRIMARY_CHANNEL_MASK      (1u << 11 | 1u << 15 | 1u << 20 | 1u << 25)

static const char *TAG = "ZIGBEE_TASK";

// Приёмник данных из Zigbee сети
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
        esp_zb_zcl_report_attr_message_t *report_msg = (esp_zb_zcl_report_attr_message_t *)message;
        uint16_t src_addr = report_msg->src_address.u.short_addr;
        uint16_t cluster_id = report_msg->cluster;
        uint16_t attribute_id = report_msg->attribute.id;
        
        // ОБРАБОТКА ВЛАЖНОСТИ
        if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT) {
            if (attribute_id == ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID) {
                float humidity = *(uint16_t *)report_msg->attribute.data.value / 100.0;
                ESP_LOGW(TAG, "[Датчик 0x%04X] ВЛАЖНОСТЬ: %.2f %%", src_addr, humidity);
            }
        } 
        // ОБРАБОТКА ТЕМПЕРАТУРЫ
        else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
            if (attribute_id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
                float temp = *(int16_t *)report_msg->attribute.data.value / 100.0;
                ESP_LOGW(TAG, "[Датчик 0x%04X] ТЕМПЕРАТУРА: %.2f C", src_addr, temp);
            }
        }
    }
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Инициализация Zigbee стека...");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Запуск формирования сети Zigbee...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "Сеть восстановлена. Открываем эфир для сопряжения на 180 секунд...");
                esp_zb_bdb_open_network(180);
            }
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Сеть успешно сформирована!");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGW(TAG, "Поиск устройств запущен. Открываем сеть на 180 сек! Включите сопряжение на Tuya ZBTH3...");
            esp_zb_bdb_open_network(180); 
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGW(TAG, "!!! Датчик обнаружен и зашел в сеть! Короткий адрес: 0x%04hx", dev_annce_params->device_short_addr);
        break;
    default:
        break;
    }
}

static void v_zigbee_task_worker(void *pvParameters)
{
    // Настраиваем роль координатора напрямую через структуру конфигурации
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false
    };
    
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ESP_ZB_GATEWAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Подключаем два клиентских кластера для прослушивания параметров датчика климата
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    esp_zb_ep_list_add_gateway_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);
    
    esp_zb_core_action_handler_register(zb_action_handler);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void zigbee_task_start(void)
{
    xTaskCreate(v_zigbee_task_worker, "Zigbee_main", 8192, NULL, 5, NULL);
}
