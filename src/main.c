#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#include "main.h"
#include "nvs.h"
#include "nvs_flash.h"

#if CONFIG_IDF_TARGET_ESP32C3
#include "driver/temp_sensor.h"
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int BattLow = 0; //Признак разряда батареи

float tsens_out = 0;

TaskHandle_t xHandleLora = NULL;
TaskHandle_t xHandleUI = NULL;
TaskHandle_t xHandleWifi = NULL;

menu_t menu[] = {
    {.id = "pulse", .name = "Импульс", .val = 100, .min = 0, .max = 10000},
    {.id = "Volt", .name = "Ограничение U", .val = 500, .min = 0, .max = 1000},
    {.id = "kU", .name = "коэф. U", .val = 162, .min = 1, .max = 10000},
    {.id = "offsU", .name = "смещ. U", .val = -8000, .min = -100000, .max = 100000},
    {.id = "kR1", .name = "коэф. R (ch 1)", .val = 1562, .min = 1, .max = 10000},
    {.id = "offsAR1", .name = "смещ. R (ch 1)", .val = -30200, .min = -100000, .max = 100000},
    {.id = "kR2", .name = "коэф. R (ch 2)", .val = 139, .min = 1, .max = 100000},
    {.id = "offsAR2", .name = "смещ. R (ch 2)", .val = 4076, .min = -100000, .max = 100000},
    {.id = "kUbat", .name = "коэф. U bat", .val = 3970, .min = 1, .max = 10000},
    {.id = "offsUbat", .name = "смещ. U bat", .val = 0, .min = -100000, .max = 100000},
    {.id = "kU0", .name = "коэф. U петли", .val = 160, .min = 1, .max = 10000},
    {.id = "offsU0", .name = "смещ. U петли", .val = 0, .min = -100000, .max = 100000},
    {.id = "UbatLow", .name = "Нижн. U bat под нагр", .val = 0, .min = 0, .max = 12000},
    {.id = "UbatEnd", .name = "U bat отключения", .val = 0, .min = 0, .max = 12000},
    {.id = "Trepeat", .name = "Интервал измер.", .val = 60, .min = 1, .max = 1000000},
    {.id = "WiFitime", .name = "WiFi timeout", .val = 60, .min = 1, .max = 10000},
    {.id = "", .name = "WiFi", .val = 0, .min = 0, .max = 1},
    {.id = "", .name = "Выход ", .val = 0, .min = 0, .max = 0},
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

#if CONFIG_IDF_TARGET_ESP32
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // 1 = High, 0 = Low
#endif
    /*
    #if CONFIG_IDF_TARGET_ESP32C3
        const gpio_config_t config = {
            .pin_bit_mask = BIT64(GPIO_NUM_0),
            .mode = GPIO_MODE_INPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(GPIO_NUM_0), ESP_GPIO_WAKEUP_GPIO_HIGH));
    #endif
    */
    uint64_t time_in_us = menu[14].val * 1000000ULL;

    if (BattLow > 200)
    {
        time_in_us = 60 * 60 * 24 * 365 * 1000000ULL; // 365 days
    }
    else if (BattLow > 100)
    {
        time_in_us = 60 * 60 * 24 * 30 * 1000000ULL; // 30 days
    }

    printf("Go sleep...\n");
    fflush(stdout);

    esp_deep_sleep(time_in_us);
}

void app_main()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        printf("Wakeup caused by external signal using RTC_IO\n");
        break;
#if SOC_PM_SUPPORT_EXT_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
#endif
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_GPIO:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
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

#if CONFIG_IDF_TARGET_ESP32C3
    // ESP_LOGI(TAG, "Initializing Temperature sensor");

    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    // temp_sensor_get_config(&temp_sensor);
    // ESP_LOGI(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
    // temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    ESP_ERROR_CHECK(temp_sensor_set_config(temp_sensor));
    ESP_ERROR_CHECK(temp_sensor_start());
    ESP_ERROR_CHECK(temp_sensor_read_celsius(&tsens_out));
    ESP_LOGI("TempSensor", "Temperature out celsius %f°C", tsens_out);
    ESP_ERROR_CHECK(temp_sensor_stop());
#endif

    // go_sleep();
    //  Initialize NVS
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

    ws_send_queue = xQueueCreate(512, WS_BUF_LINE);

    i2c_mux = xSemaphoreCreateMutex();

    ready_event_group = xEventGroupCreate();

    // set_lora_queue = xQueueCreate(2, sizeof(cmd_t));

#ifdef RECEIVER_ONLY
    xTaskCreate(radio_task, "radio_task", 1024 * 4, NULL, 5, &xHandleLora);
    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
#else
    xTaskCreate(radio_task, "radio_task", 1024 * 4, NULL, 5, &xHandleLora);
    xTaskCreate(dual_adc, "dual_adc", 1024 * 4, NULL, 6, NULL);
#endif

    cmd_t cmd;
    cmd.cmd = 3;
    cmd.power = 255;
    xQueueSend(uicmd_queue, &cmd, (portTickType)0);

    // BIT0 - окончание измерения
    // BIT1 - окончание передачи
    // BIT2 - timeout работы wifi
    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_MEASURE,       /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        portMAX_DELAY);

    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
    {
        xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);
#if CONFIG_IDF_TARGET_ESP32
        // xTaskCreate(ui_task, "ui_task", 1024 * 8, NULL, 5, &xHandleUI);
        // xTaskCreate(clock_task, "clock_task", 1024 * 2, NULL, 5, NULL);
#endif

        xEventGroupWaitBits(
            ready_event_group, /* The event group being tested. */
            END_WIFI_TIMEOUT,  /* The bits within the event group to wait for. */
            pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
            pdTRUE,
            portMAX_DELAY);
    }

    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_TRANSMIT,      /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        portMAX_DELAY);

    xTaskNotify(xHandleLora, SLEEP_BIT, eSetValueWithOverwrite);

    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_LORA_SLEEP,    /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        10);

    if (xHandleUI)
    {
        xTaskNotify(xHandleUI, SLEEP_BIT, eSetValueWithOverwrite);

        xEventGroupWaitBits(
            ready_event_group, /* The event group being tested. */
            END_UI_SLEEP,      /* The bits within the event group to wait for. */
            pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
            pdTRUE,
            10);
    }

    //засыпаем...
    go_sleep();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}
