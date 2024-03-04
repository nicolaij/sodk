#include "main.h"
#include <sys/time.h>

#include <stdio.h>
#include "esp_mac.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_timer.h"

#include "driver/i2c.h"
#define PCF8575_I2C_ADDR_BASE 0x20

#include <string.h>

#if ESP_IDF_VERSION_MAJOR == 5
#include "driver/temperature_sensor.h"
#endif

#if ESP_IDF_VERSION_MAJOR == 4
#include "driver/temp_sensor.h"
#endif

RTC_DATA_ATTR int bootCount = 0;

float tsens_out = 0;

uint8_t mac[6];

TaskHandle_t xHandleRadio = NULL;
TaskHandle_t xHandleUI = NULL;
TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandleADC = NULL;
TaskHandle_t xHandleBtn = NULL;

int i2c_err = 1;

// static i2c_dev_t pcf8575;

menu_t menu[] = {
    {.id = "pulse", .name = "Время импульса", .val = 500, .min = -1, .max = 10000},
    {.id = "Volt", .name = "Ограничение U", .val = 600, .min = 0, .max = 1000},
    {.id = "kU", .name = "коэф. U", .val = 1690, .min = 1, .max = 10000},
    {.id = "offsU", .name = "смещ. U", .val = -7794, .min = -100000, .max = 100000},
    {.id = "kUlv", .name = "коэф. U низ.", .val = 1690, .min = 1, .max = 10000},
    {.id = "offsUlv", .name = "смещ. U низ.", .val = -7794, .min = -100000, .max = 100000},
    {.id = "kR1", .name = "коэф. R (ch 1)", .val = 15000, .min = 1, .max = 100000},
    {.id = "offsAR1", .name = "смещ. R (ch 1)", .val = -7925, .min = -100000, .max = 100000},
    {.id = "kR2", .name = "коэф. R (ch 2)", .val = 319, .min = 1, .max = 100000},
    {.id = "offsAR2", .name = "смещ. R (ch 2)", .val = -1729, .min = -100000, .max = 100000},
    {.id = "kU0", .name = "коэф. U петли", .val = 1611, .min = 1, .max = 10000}, /*10*/
    {.id = "offsU0", .name = "смещ. U петли", .val = -539, .min = -100000, .max = 100000},
    {.id = "kU0lv", .name = "коэф. U петли низ.", .val = 1611, .min = 1, .max = 10000},
    {.id = "offsU0lv", .name = "смещ. U петли низ.", .val = -539, .min = -100000, .max = 100000},
    {.id = "kUbat", .name = "коэф. U bat", .val = 5005, .min = 1, .max = 100000},
    {.id = "offsUbat", .name = "смещ. U bat", .val = -4000, .min = -1000000, .max = 1000000},
    {.id = "UbatLow", .name = "Нижн. U bat под нагр", .val = 0, .min = 0, .max = 12000},
    {.id = "UbatEnd", .name = "U bat отключения", .val = 0, .min = 0, .max = 12000},
    {.id = "Trepeatlv", .name = "Интервал измерений", .val = 120, .min = 1, .max = 1000000},
    {.id = "Trepeathv", .name = "Интервал измер. выс.", .val = 3600, .min = 1, .max = 1000000},
    {.id = "chanord", .name = "Порядок опроса каналов", .val = 1234, .min = 0, .max = 999999999}, /*20*/
    {.id = "WiFitime", .name = "WiFi timeout", .val = 120, .min = 1, .max = 10000},
    {.id = "avgcomp", .name = "Кол-во совпад. сравн.", .val = 25, .min = 1, .max = 1000},
    {.id = "avgcnt", .name = "Кол-во усред. сравн.", .val = 25, .min = 1, .max = 1000},
    {.id = "blocks", .name = "График после результ.", .val = 100, .min = 0, .max = 2000},
    {.id = "Kfilter", .name = "Коэф. фильтрации АЦП", .val = 10, .min = 1, .max = 100},
    {.id = "percU0lv", .name = "\% U петли низ.", .val = 50, .min = 0, .max = 100}, /*26 Процент от Ubatt, ниже которого - обрыв 0 провода, > - цел. 100% - не проводим высоковольные измерения от изменения*/
    {.id = "percRlv", .name = "\% R низ.", .val = 50, .min = 0, .max = 100},        /*Процент изменения от предыдущего значения сопротивления, ниже которого считаем нормой, выше - проводим высоковольтные измерения. 100% - не проводим высоковольтные измерения по изменению сопротивления*/
};

QueueHandle_t uicmd_queue;
QueueHandle_t send_queue;
QueueHandle_t set_lora_queue;
RingbufHandle_t wsbuf_handle;

EventGroupHandle_t ready_event_group;

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
                ESP_LOGD("NVS", "Read \"%s\" = %ld", menu[i].name, menu[i].val);
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

int pcf8575_read(uint16_t bit)
{
    if (i2c_err)
        return 0;

    static uint16_t port_val = UINT16_MAX;

    ESP_ERROR_CHECK(i2c_master_read_from_device(0, PCF8575_I2C_ADDR_BASE, (uint8_t *)&port_val, 2, 1000 / portTICK_PERIOD_MS));

    if ((port_val & BIT(bit)) == 0)
        return 1;
    return 0;
}

// #define INVERT_NB_PWR 1 //без транзистора на входе (0 - сигнал вкл.)
//  POWER_BIT - и NB_PWR_BIT - инверсные
//  POWER_BIT - всегда, кроме когда каналы выбраны
void pcf8575_set(int channel_cmd)
{
    if (i2c_err)
        return;

    ESP_LOGD("pcf8575", "pcf8575 set (%d)", channel_cmd);

    static uint16_t current_mask = 0;
    uint16_t cmd_mask = 0;

    switch (channel_cmd)
    {
    case 0:
#if NB == 26
        current_mask = BIT(POWER_BIT) | BIT(NB_PWR_BIT);
#else
        current_mask = BIT(POWER_BIT);
#endif
        cmd_mask = current_mask;
        break;
    case NB_PWR_CMDON:
#if NB == 26
        current_mask &= ~BIT(NB_PWR_BIT);
#else
        current_mask |= BIT(NB_PWR_BIT);
#endif
        cmd_mask = current_mask;
        break;
    case NB_PWR_CMDOFF:
#if NB == 26
        current_mask |= BIT(NB_PWR_BIT);
#else
        current_mask &= ~BIT(NB_PWR_BIT);
#endif
        cmd_mask = current_mask;
        break;
    case LV_CMDON:
        current_mask |= BIT(LV_POWER_BIT);
        cmd_mask = current_mask;
        break;
    case LV_CMDOFF:
        current_mask &= ~BIT(LV_POWER_BIT);
        cmd_mask = current_mask;
        break;
    case NB_RESET_CMD:
        // cmd_mask = current_mask | BIT(NB_RESET_BIT);
        break;
    case POWER_CMDOFF:
        current_mask |= BIT(POWER_BIT); // Off HV power
        current_mask &= ~(0x60ff);      // очищаем каналы 0..7 bit + LV
        cmd_mask = current_mask;
        break;
    case 1:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(6);          // ADC R
        current_mask |= BIT(7);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 5:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(LV_BIT);     // LV
        current_mask |= BIT(6);          // ADC R
        current_mask |= BIT(7);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 2:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(4);          // ADC R
        current_mask |= BIT(5);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 6:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(LV_BIT);     // LV
        current_mask |= BIT(4);          // ADC R
        current_mask |= BIT(5);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 3:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(2);          // ADC R
        current_mask |= BIT(3);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 7:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(LV_BIT);     // LV
        current_mask |= BIT(2);          // ADC R
        current_mask |= BIT(3);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 4:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(0);          // ADC R
        current_mask |= BIT(1);          // Uout + U0
        cmd_mask = current_mask;
        break;
    case 8:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV
        current_mask |= BIT(LV_BIT);     // LV
        current_mask |= BIT(0);          // ADC R
        current_mask |= BIT(1);          // Uout + U0
        cmd_mask = current_mask;
        break;
    }

    uint16_t port_val = ~(cmd_mask);

    ESP_ERROR_CHECK(i2c_master_write_to_device(0, PCF8575_I2C_ADDR_BASE, (uint8_t *)&port_val, 2, 1000 / portTICK_PERIOD_MS));
    // ESP_LOGV("pcf8575", "pcf8575 set OK");
}

void go_sleep(void)
{

#if CONFIG_IDF_TARGET_ESP32
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // 1 = High, 0 = Low
#endif

    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(INT_PIN), ESP_GPIO_WAKEUP_GPIO_LOW));

    ESP_LOGW("main", "Go sleep...");
    fflush(stdout);

    uint64_t time_in_us = StoUS(menu[18].val);

    // коррекция на время работы
    time_in_us = time_in_us - (esp_timer_get_time() % StoUS(menu[18].val));

    esp_sleep_enable_timer_wakeup(time_in_us * BattLow);
    esp_deep_sleep_start();
}

void start_measure(int channel, int flag)
{
    cmd_t cmd;
    int pos;
    if (channel >= 1 && channel <= 8)
    {
        cmd.channel = channel;
        xQueueSend(uicmd_queue, &cmd, (TickType_t)0);
    }
    else
    {
        if (flag != 1)
        {
            pos = 100000000;
            while (pos > 0) // First LV
            {
                cmd.channel = (menu[20].val % (pos * 10)) / pos;
                if (cmd.channel > 0)
                {
                    if (cmd.channel <= 4)
                    {
                        cmd.channel += 4;
                    }
                    xQueueSend(uicmd_queue, &cmd, (TickType_t)0);
                }
                pos /= 10;
            }
        }

        if (flag != 2)
        {
            pos = 100000000;
            while (pos > 0) // HV
            {
                cmd.channel = (menu[20].val % (pos * 10)) / pos;
                if (cmd.channel > 0 && cmd.channel <= 4)
                {
                    xQueueSend(uicmd_queue, &cmd, (TickType_t)0);
                }
                pos /= 10;
            }
        }
    }
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

    bootCount++;
    //  "Количество загрузок: "
    ESP_LOGI("main", "Boot number: %d", bootCount);

    esp_efuse_mac_get_default(mac);
    ESP_LOGI("main", "mac: %02x-%02x-%02x-%02x-%02x-%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

#if CONFIG_IDF_TARGET_ESP32C3
    ESP_LOGD("main", "Initializing Temperature sensor");

#if ESP_IDF_VERSION_MAJOR == 5 // esp-idf v5
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_out));

    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_sensor));
#endif
#if ESP_IDF_VERSION_MAJOR == 4 // esp-idf v4
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    // temp_sensor_get_config(&temp_sensor);
    // temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    // temp_sensor.dac_offset = TSENS_DAC_L3; /*!< offset =  1, measure range:-30℃ ~  50℃, error < 2℃. */
    ESP_ERROR_CHECK(temp_sensor_set_config(temp_sensor));
    ESP_ERROR_CHECK(temp_sensor_start());
    vTaskDelay(1); // Влияет на измерения
    ESP_ERROR_CHECK(temp_sensor_read_celsius(&tsens_out));
    ESP_ERROR_CHECK(temp_sensor_stop());
#endif

    wsbuf_handle = xRingbufferCreate(10 * WS_BUF_SIZE, RINGBUF_TYPE_NOSPLIT);

    char buf[32];
    int l = snprintf((char *)buf, sizeof(buf), "Temperature:  %.01f°C", tsens_out);
    xRingbufferSend(wsbuf_handle, buf, l + 1, 0);
    ESP_LOGI("temperature_sensor", "%s", buf);

#endif

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

    // Example of nvs_get_stats() to get the number of used entries and free entries:
    nvs_stats_t nvs_stats;
    nvs_get_stats(NULL, &nvs_stats);
    ESP_LOGI("NVS", "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);

    read_nvs_menu();

    gpio_config_t io_conf = {};

    // Check I2C bus resistor to +3v3 is connected
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << I2C_MASTER_SDA_PIN) | (1 << I2C_MASTER_SCL_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (gpio_get_level(I2C_MASTER_SDA_PIN) == 1 && gpio_get_level(I2C_MASTER_SCL_PIN) == 1)
    {

        int i2c_master_port = 0;

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_PIN,
            .scl_io_num = I2C_MASTER_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
        };

        i2c_param_config(i2c_master_port, &conf);

        if (i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0) == ESP_OK)
        {
            i2c_err = 0;
        }
        else
        {
            ESP_LOGE("i2c", "i2c_driver_install error!");
        }

        // init pcf outs
        pcf8575_set(0);
    }
    else
    {
        ESP_LOGE("i2c", "i2c bus error state!");
    }

    uicmd_queue = xQueueCreate(8, sizeof(cmd_t));

    send_queue = xQueueCreate(10, sizeof(result_t));

    ready_event_group = xEventGroupCreate();

#ifdef RECEIVER_ONLY
    xTaskCreate(radio_task, "radio_task", 1024 * 4, NULL, 5, &xHandleRadio);
    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
#else
    xTaskCreate(btn_task, "btn_task", 1024 * 3, NULL, 5, &xHandleBtn);

    xTaskCreate(radio_task, "radio_task", 1024 * 4, NULL, 5, &xHandleRadio);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) // reset
    {
        // vTaskDelay(3000 / portTICK_PERIOD_MS);
    };

    xTaskCreate(dual_adc, "dual_adc", 1024 * 4, NULL, 10, &xHandleADC);

    if (bootCount % (menu[19].val / menu[18].val) == 1 && BattLow < 10)
    {
        start_measure(0, 0);
    }
    else
    {
        start_measure(0, 2); // LV only
    }

#endif

    if (terminal_mode > -1)
        ESP_LOGI("info", "Free memory: %lu bytes", esp_get_free_heap_size());

    // BIT0 - окончание измерения
    // BIT1 - окончание передачи
    // BIT2 - timeout работы wifi
    /* Ждем окончания измерений */
    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_MEASURE,       /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        portMAX_DELAY);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED || wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) // reset
    {
        // для отладки схемы pulse -1
        if (menu[0].val != -1)
        {
            xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);
        }

        xEventGroupWaitBits(
            ready_event_group, /* The event group being tested. */
            END_WIFI_TIMEOUT,  /* The bits within the event group to wait for. */
            pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
            pdTRUE,
            portMAX_DELAY);
    }

    if (terminal_mode > -1)
    {

        ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());

        ESP_LOGI("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
        ESP_LOGI("dual_adc", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
        ESP_LOGI("radio_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleRadio));
        ESP_LOGI("btn_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleBtn));
    }
    xEventGroupWaitBits(
        ready_event_group,              /* The event group being tested. */
        END_TRANSMIT | END_RADIO_SLEEP, /* The bits within the event group to wait for. */
        pdFALSE,                        /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        100000 / portTICK_PERIOD_MS);

    xTaskNotify(xHandleRadio, SLEEP_BIT, eSetValueWithOverwrite);

    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_RADIO_SLEEP,   /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        4000 / portTICK_PERIOD_MS);

    if (terminal_mode > -1)
        ESP_LOGI("info", "Free memory: %lu bytes", esp_get_free_heap_size());

    // засыпаем...
    go_sleep();

    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    };
}
