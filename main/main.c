#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_zigbee_gateway.h"
// Прототип функции вашей Zigbee задачи (реализована в esp_zigbee_gateway.c)
extern void esp_zb_task(void *pvParameters);

// Пример вашей пользовательской задачи (например, опрос датчиков)
void my_sensor_task(void *pvParameters)
{
    while (1) {
        printf("Чтение данных с датчиков...\n");
        // Ваш код работы с периферией (I2C, SPI, GPIO)
        vTaskDelay(pdMS_TO_TICKS(2000)); // Задержка 2 секунды
    }
}

// Еще одна пользовательская задача (например, обработка логики шлюза)
void gateway_logic_task(void *pvParameters)
{
    while (1) {
        // Логика отправки данных в облако или обработки правил
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    // Инициализация платформы и базовых сервисов
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif

    // --- СОЗДАНИЕ ВАШИХ ПОЛЬЗОВАТЕЛЬСКИХ ЗАДАЧ ---
    xTaskCreate(my_sensor_task, "sensor_task", 4096, NULL, 3, NULL);
    
    // Задача для логики автоматизации
    xTaskCreate(gateway_logic_task, "logic_task", 4096, NULL, 3, NULL);

    // --- ЗАПУСК ГЛАВНОЙ ЗАДАЧИ ZIGBEE ---
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
