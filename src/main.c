#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "esp_sleep.h"

#include "main.h"
#include "nvs.h"
#include "nvs_flash.h"

RTC_DATA_ATTR int bootCount = 0;

TaskHandle_t xHandleLora = NULL;
TaskHandle_t xHandleUI = NULL;

menu_t menu[] = {
    {.id = "pulse", .name = "Импульс", .val = 100, .min = 1, .max = 1000},
    {.id = "kU", .name = "коэф. U", .val = 5555, .min = 1000, .max = 10000},
    {.id = "kR", .name = "коэф. R", .val = 575, .min = 1, .max = 1000},
    {.id = "offsU", .name = "смещ. U", .val = 22, .min = -500, .max = 500},
    {.id = "offsADC", .name = "смещ. adcR", .val = 145, .min = -500, .max = 500},
    {.id = "", .name = "WiFi", .val = 0, .min = 0, .max = 1},
    {.id = "", .name = "Выход  ", .val = 0, .min = 0, .max = 0},
};

int read_nvs_menu()
{
    // Open
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)
        {
            err = nvs_get_i32(my_handle, menu[i].id, &menu[i].val);
            switch (err)
            {
            case ESP_OK:
                printf("Read \"%s\" = %d\n", menu[i].name, menu[i].val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value  \"%s\" is not initialized yet!\n", menu[i].name);
                break;
            default:
                printf("Error (%s) reading!\n", esp_err_to_name(err));
            }
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}

void go_sleep(void)
{
    if (xHandleLora)
        xTaskNotify(xHandleLora, SLEEP_BIT, eSetValueWithOverwrite);
    if (xHandleUI)
        xTaskNotify(xHandleUI, SLEEP_BIT, eSetValueWithOverwrite);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    fflush(stdout);
    printf("Go sleep...");
#if CONFIG_IDF_TARGET_ESP32
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // 1 = High, 0 = Low
#endif

    esp_sleep_enable_timer_wakeup(60 * 1000000);
    esp_deep_sleep_start();
}

void app_main()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        printf("Wakeup caused by external signal using RTC_IO\n");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        printf("Wakeup caused by external signal using RTC_CNTL\n");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        printf("Wakeup caused by timer\n");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        printf("Wakeup caused by touchpad\n");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        printf("Wakeup caused by ULP program\n");
        break;
    default:
        printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }

    ++bootCount;
    //  "Количество загрузок: "
    printf("Boot number: %d\n", bootCount);

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

    // xTaskCreate(btn_task, "btn_task", 1024 * 2, NULL, 5, NULL);

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    read_nvs_menu();

    uicmd_queue = xQueueCreate(2, sizeof(cmd_t));
    adc1_queue = xQueueCreate(2, sizeof(result_t));

    send_queue = xQueueCreate(2, sizeof(result_t));

    ws_send_queue = xQueueCreate(2, 256);

    i2c_mux = xSemaphoreCreateMutex();

    ready_event_group = xEventGroupCreate();

    // set_lora_queue = xQueueCreate(2, sizeof(cmd_t));

    xTaskCreate(radio_task, "radio_task", 1024 * 4, NULL, 5, &xHandleLora);

#ifndef RECEIVER_ONLY
    xTaskCreate(dual_adc, "dual_adc", 1024 * 4, NULL, 6, NULL);
#endif

    cmd_t cmd;
    cmd.cmd = 3;
    cmd.power = 255;
    xQueueSend(uicmd_queue, &cmd, (portTickType)0);

    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
    {
//        xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, NULL);
#if CONFIG_IDF_TARGET_ESP32
        xTaskCreate(ui_task, "ui_task", 1024 * 8, NULL, 5, &xHandleUI);
        xTaskCreate(clock_task, "clock_task", 1024 * 2, NULL, 5, NULL);
#endif
    }
    else
    {

        xEventGroupWaitBits(
            ready_event_group, /* The event group being tested. */
            BIT0 | BIT1,       /* The bits within the event group to wait for. */
            pdTRUE,            /* BIT_0 & BIT_1 should be cleared before returning. */
            pdTRUE,
            3000 / portTICK_PERIOD_MS);

        //засыпаем...
        go_sleep();
    }

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}
