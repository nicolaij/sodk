#include "main.h"

#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#include "nvs.h"
#include "nvs_flash.h"

#if CONFIG_IDF_TARGET_ESP32C3
#include "driver/temp_sensor.h"
#endif

#if MULTICHAN
#include "pcf8575.h"
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int BattLow = 0; //Признак разряда батареи

float tsens_out = 0;

uint8_t mac[6];

TaskHandle_t xHandleRadio = NULL;
TaskHandle_t xHandleUI = NULL;
TaskHandle_t xHandleWifi = NULL;

int i2c_err = 1;

static i2c_dev_t pcf8575;

menu_t menu[] = {
    {.id = "pulse", .name = "Время импульса", .val = 100, .min = 0, .max = 10000},
    {.id = "Volt", .name = "Ограничение U", .val = 500, .min = 0, .max = 1000},
    {.id = "kU", .name = "коэф. U", .val = 1690, .min = 1, .max = 10000},
    {.id = "offsU", .name = "смещ. U", .val = -7794, .min = -100000, .max = 100000},
    {.id = "kR1", .name = "коэф. R (ch 1)", .val = 15000, .min = 1, .max = 100000},
    {.id = "offsAR1", .name = "смещ. R (ch 1)", .val = -7925, .min = -100000, .max = 100000},
    {.id = "kR2", .name = "коэф. R (ch 2)", .val = 319, .min = 1, .max = 100000},
    {.id = "offsAR2", .name = "смещ. R (ch 2)", .val = -1729, .min = -100000, .max = 100000},
    {.id = "kUbat", .name = "коэф. U bat", .val = 5005, .min = 1, .max = 10000},
    {.id = "offsUbat", .name = "смещ. U bat", .val = -4000, .min = -100000, .max = 100000},
    {.id = "kU0", .name = "коэф. U петли", .val = 1611, .min = 1, .max = 10000}, /*10*/
    {.id = "offsU0", .name = "смещ. U петли", .val = -539, .min = -100000, .max = 100000},
    {.id = "UbatLow", .name = "Нижн. U bat под нагр", .val = 0, .min = 0, .max = 12000},
    {.id = "UbatEnd", .name = "U bat отключения", .val = 0, .min = 0, .max = 12000},
    {.id = "Trepeat", .name = "Интервал измер.", .val = 60, .min = 1, .max = 1000000},
    {.id = "WiFitime", .name = "WiFi timeout", .val = 60, .min = 1, .max = 10000},
    {.id = "repeat", .name = "Кол-во повт. имп.", .val = 0, .min = 0, .max = 6},
    {.id = "avgcnt", .name = "Кол-во усред. рез.", .val = 20, .min = 1, .max = 1000}, /*17*/
    {.id = "chanord", .name = "Порядок опроса каналов", .val = 1234, .min = 0, .max = 9999},
    {.id = "test", .name = " ", .val = 1, .min = 0, .max = 1},
};

int read_nvs_menu()
{
    // Open
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
        {
            err = nvs_get_i32(my_handle, menu[i].id, &menu[i].val);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGD("NVS", "Read \"%s\" = %d", menu[i].name, menu[i].val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", menu[i].name);
                break;
            default:
                ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
            }
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}
/*
void power_on(int channel_mask)
{
    if (i2c_err)
        return;
    uint16_t port_val = ~(channel_mask | (1 << NB_PWR_BIT));
    // ESP_ERROR_CHECK(pcf8575_port_write(&pcf8575, port_val));
    pcf8575_port_write(&pcf8575, port_val);
}

void power_off(void)
{
    if (i2c_err)
        return;
    uint16_t port_val = ~((1 << POWER_BIT) | (1 << NB_PWR_BIT));
    // ESP_ERROR_CHECK(pcf8575_port_write(&pcf8575, port_val));
    pcf8575_port_write(&pcf8575, port_val);
}
*/
void pcf8575_set(uint16_t channel_mask)
{
    if (i2c_err)
        return;
    uint16_t port_val = ~(channel_mask);
    pcf8575_port_write(&pcf8575, port_val);
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

    ESP_LOGW("main", "Go sleep...");

    fflush(stdout);

    esp_deep_sleep(time_in_us);
}

void start_measure()
{
    cmd_t cmd;
    cmd.cmd = 3;
    cmd.channel = 1;

#if MULTICHAN
    int pos = 1000;
    while (pos > 0)
    {
        cmd.channel = (menu[18].val % (pos * 10)) / pos;
        if (cmd.channel > 0)
            xQueueSend(uicmd_queue, &cmd, (portTickType)0);

        pos /= 10;
    }
#else
    xQueueSend(uicmd_queue, &cmd, (portTickType)0);
#endif
}

void app_main()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        ESP_LOGI("main", "Wakeup caused by external signal using RTC_IO");
        break;
#if SOC_PM_SUPPORT_EXT_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGI("main", "Wake up from GPIO %d", pin);
        }
        else
        {
            ESP_LOGI("main", "Wake up from GPIO");
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
            ESP_LOGI("main", "Wake up from GPIO %d", pin);
        }
        else
        {
            ESP_LOGI("main", "Wake up from GPIO");
        }
        break;
    }
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI("main", "Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        ESP_LOGI("main", "Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        ESP_LOGI("main", "Wakeup caused by ULP program");
        break;
    default:
        ESP_LOGI("main", "Wakeup was not caused by deep sleep: %d", wakeup_reason);
        break;
    }

    ++bootCount;
    //  "Количество загрузок: "
    ESP_LOGI("main", "Boot number: %d", bootCount);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI("main", "This is ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d",
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             chip_info.revision);

    ESP_LOGI("main", "flash: %dMB %s", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    esp_efuse_mac_get_default(mac);
    ESP_LOGI("main", "mac: %02x-%02x-%02x-%02x-%02x-%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

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

#if MULTICHAN

    gpio_config_t io_conf = {};

    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << I2C_MASTER_SDA_PIN) | (1 << I2C_MASTER_SCL_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (gpio_get_level(I2C_MASTER_SDA_PIN) == 1 && gpio_get_level(I2C_MASTER_SCL_PIN) == 1)
    {
        i2c_err = 0;
    }
    else
    {
        ESP_LOGE("i2c", "pcf8575 not found!");
    }

    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    // Zero device descriptor
    memset(&pcf8575, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8575_init_desc(&pcf8575, PCF8575_I2C_ADDR_BASE, 0, I2C_MASTER_SDA_PIN, I2C_MASTER_SCL_PIN));

    //отключаем питание АЦП, включаем NBIOT
    pcf8575_set((1 << POWER_BIT) | (1 << NB_PWR_BIT));

#endif

    uicmd_queue = xQueueCreate(4, sizeof(cmd_t));
    adc1_queue = xQueueCreate(2, sizeof(result_t));

    send_queue = xQueueCreate(4, sizeof(result_t));

    ws_send_queue = xQueueCreate(8, WS_BUF_SIZE);

    // i2c_mux = xSemaphoreCreateMutex();

    ready_event_group = xEventGroupCreate();

#ifdef RECEIVER_ONLY
    xTaskCreate(radio_task, "radio_task", 1024 * 4, NULL, 5, &xHandleRadio);
    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
#else

    //ждем включения NBIOT модуля
    // vTaskDelay(600 / portTICK_PERIOD_MS);
    // pcf8575_set((1 << POWER_BIT) | (1 << NB_PWR_BIT));

    xTaskCreate(radio_task, "radio_task", 1024 * 6, NULL, 5, &xHandleRadio);

    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
    {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    };

    xTaskCreate(dual_adc, "dual_adc", 1024 * 4, NULL, 7, NULL);
#endif

    xTaskCreate(btn_task, "btn_task", 1024 * 2, NULL, 5, NULL);

    start_measure();

    // BIT0 - окончание измерения
    // BIT1 - окончание передачи
    // BIT2 - timeout работы wifi
    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_MEASURE,       /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        20000 / portTICK_PERIOD_MS);

    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
    {
        xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);

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
        120000 / portTICK_PERIOD_MS);

    xTaskNotify(xHandleRadio, SLEEP_BIT, eSetValueWithOverwrite);

    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_LORA_SLEEP,    /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        100 / portTICK_PERIOD_MS);

    //засыпаем...
    go_sleep();

    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    };
}
