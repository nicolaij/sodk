#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "main.h"

void app_main()
{
    //printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //xTaskCreate(btn_task, "btn_task", 1024 * 2, NULL, 5, NULL);
    
	uicmd_queue = xQueueCreate(1, sizeof(cmd_t));
    adc1_queue = xQueueCreate(10, sizeof(int) * 2);

    xTaskCreate(dual_adc, "dual_adc", 1024 * 2, NULL, 5, NULL);

    xTaskCreate(ui_task, "ui_task", 1024 * 8, NULL, 5, NULL);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };

    for (int i = 10; i >= 0; i--)
    {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
