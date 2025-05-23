/*
    espnow MAC0 8713987 MAC3 3784525
*/

#include "main.h"
#include <sys/time.h>

#include <stdio.h>
#include "esp_mac.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_timer.h"

#define PCF8575_I2C_ADDR_BASE 0x20

#include <string.h>

#include "driver/i2c_master.h"
#include "driver/temperature_sensor.h"

i2c_master_dev_handle_t dev_handle_pcf;

RTC_DATA_ATTR int bootCount = 0;

float tsens_out = 0;

uint8_t mac[6];

TaskHandle_t xHandleNB = NULL;
TaskHandle_t xHandleUI = NULL;
TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandleADC = NULL;
TaskHandle_t xHandleBtn = NULL;

int i2c_err = 1;

QueueHandle_t uicmd_queue;
QueueHandle_t send_queue;
QueueHandle_t set_lora_queue;
RingbufHandle_t wsbuf_handle;

EventGroupHandle_t status_event_group;

int pcf8575_read(int bit)
{
    if (i2c_err)
        return 0;

    static uint16_t port_val = UINT16_MAX;

    esp_err_t err = i2c_master_receive(dev_handle_pcf, (uint8_t *)&port_val, 2, 100);

    if (err == ESP_OK)
    {
        ESP_LOGD("pcf8575", "pcf8575 get: 0x%04X", port_val);

        if ((port_val & BIT(bit)) == 0)
            return 1;
    }

    return 0;
}

//  POWER_BIT - и NB_PWR_BIT - инверсные
//  POWER_BIT - всегда, кроме когда каналы выбраны
void pcf8575_set(int channel_cmd)
{
    if (i2c_err)
        return;

    static uint16_t current_mask = 0;
    uint16_t cmd_mask = 0;

    switch (channel_cmd)
    {
    case 0:
        current_mask = BIT(POWER_BIT);
        cmd_mask = current_mask;
        break;
    case NB_PWR_CMDON:
        current_mask |= BIT(NB_PWR_BIT);
        cmd_mask = current_mask;
        break;
    case NB_PWR_CMDOFF:
        current_mask &= ~BIT(NB_PWR_BIT);
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
    case LV_MEASUREON:
        current_mask |= BIT(LV_BIT);
        cmd_mask = current_mask;
        break;
    case LV_MEASUREOFF:
        current_mask &= ~BIT(LV_BIT);
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

        current_mask |= BIT(6); // ADC R
        current_mask |= BIT(7); // Uout + U0

        cmd_mask = current_mask;
        break;
    case 5:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV

        current_mask |= BIT(LV_BIT); // LV
        current_mask |= BIT(6);      // ADC R
        current_mask |= BIT(7);      // Uout + U0

        cmd_mask = current_mask;
        break;
    case 2:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV

        current_mask |= BIT(4); // ADC R
        current_mask |= BIT(5); // Uout + U0

        cmd_mask = current_mask;
        break;
    case 6:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV

        current_mask |= BIT(LV_BIT); // LV
        current_mask |= BIT(4);      // ADC R
        current_mask |= BIT(5);      // Uout + U0

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

        current_mask |= BIT(LV_BIT); // LV
        current_mask |= BIT(2);      // ADC R
        current_mask |= BIT(3);      // Uout + U0
        cmd_mask = current_mask;
        break;
    case 4:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV

        current_mask |= BIT(0); // ADC R
        current_mask |= BIT(1); // Uout + U0

        cmd_mask = current_mask;
        break;
    case 8:                              // LV Measure
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        current_mask &= ~(0x60ff);       // очищаем каналы 0..7 bit + LV

        current_mask |= BIT(LV_BIT); // LV
        current_mask |= BIT(0);      // ADC R
        current_mask |= BIT(1);      // Uout + U0

        cmd_mask = current_mask;
        break;
    }

    uint16_t port_val = ~(cmd_mask);

    if (i2c_master_transmit(dev_handle_pcf, (uint8_t *)&port_val, 2, 100) != ESP_OK)
    {
        i2c_err = 1;
        ESP_LOGE("pcf8575", "Error transmit");
    }
    else
    {
        // ESP_LOGD("pcf8575", "cmd: %3d set: 0x%04X", channel_cmd, port_val);
    }
}

void start_measure(int channel, int flag)
{
    cmd_t cmd = {.channel = 0, .cmd = 0};
    int pos;
    if (channel >= 1 && channel <= 8)
    {
        cmd.channel = channel;
        xQueueSend(uicmd_queue, &cmd, (TickType_t)0);
    }
    else
    {
        int channels = get_menu_val_by_id("chanord");

        if (flag != 1)
        {
            pos = 100000000;
            while (pos > 0) // First LV
            {
                cmd.channel = (channels % (pos * 10)) / pos;
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
                cmd.channel = (channels % (pos * 10)) / pos;
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
    char buf[40];

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

    ESP_LOGI("main", "Current date/time: %s", get_datetime(time(0)));

    esp_efuse_mac_get_default(mac);
    ESP_LOGI("main", "mac: %02x-%02x-%02x-%02x-%02x-%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    wsbuf_handle = xRingbufferCreate(10 * WS_BUF_SIZE, RINGBUF_TYPE_NOSPLIT);

    ESP_LOGD("main", "Initializing Temperature sensor");

    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_out));

    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_sensor));

    int l = snprintf((char *)buf, sizeof(buf), "Temperature:  %.01f°C", tsens_out);
    xRingbufferSend(wsbuf_handle, buf, l + 1, 0);
    ESP_LOGI("temperature_sensor", "%s", buf);

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

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8575_I2C_ADDR_BASE,
        .scl_speed_hz = 100000,
    };

    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle_pcf) == ESP_OK)
    {
        i2c_err = 0;
    }
    else
    {
        ESP_LOGE("i2c", "i2c_driver_install error!");
    }

    // init pcf outs
    pcf8575_set(0);

    pcf8575_read(0);

    uicmd_queue = xQueueCreate(8, sizeof(cmd_t));

    send_queue = xQueueCreate(10, sizeof(result_t));

    status_event_group = xEventGroupCreate();

    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xHandleWifi);

    xTaskCreate(console_task, "console_task", 1024 * 5, NULL, configMAX_PRIORITIES - 20, NULL);

    // xTaskCreate(btn_task, "btn_task", 1024 * 2, NULL, configMAX_PRIORITIES - 20, &xHandleBtn);

    xTaskCreate(modem_task, "modem_task", 1024 * 5, NULL, configMAX_PRIORITIES - 15, &xHandleNB);

    xTaskCreate(adc_task, "adc_task", 1024 * 3, NULL, configMAX_PRIORITIES - 5, &xHandleADC);

    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 7);

    const int Trepeatlv = get_menu_val_by_id("Trepeatlv");
    const int Trepeathv = get_menu_val_by_id("Trepeathv");

    if (bootCount % (Trepeathv / Trepeatlv) == 1 && BattLow < 100)
    {
        if (xHandleNB)
            xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль
        start_measure(0, 0);
    }
    else
    {
        start_measure(0, 2); // LV only
    }

    if (terminal_mode > -1)
        ESP_LOGI("info", "Free memory: %lu bytes", esp_get_free_heap_size());

    /* Ждем окончания измерений */
    xEventGroupWaitBits(
        status_event_group, /* The event group being tested. */
        END_MEASURE,        /* The bits within the event group to wait for. */
        pdFALSE,            /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        60000 / portTICK_PERIOD_MS);

    // время ожидания
    int wait = get_menu_val_by_id("waitwifi");

    // запускаем WiFi если подали питание или сработал концевик
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED || wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) // reset
    {
        // для отладки схемы pulse != -1
        if (get_menu_val_by_id("pulse") != -1)
        {
            if (xHandleWifi)
                xTaskNotifyGive(xHandleWifi); // включаем WiFi
        }

        // Ждем timeout WIFI_ACTIVE
        while ((xEventGroupWaitBits(status_event_group, WIFI_ACTIVE, pdTRUE, pdFALSE, wait * 60000 / portTICK_PERIOD_MS) & WIFI_ACTIVE) != 0)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        };
    }

    if (terminal_mode > -1)
    {
        ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
        ESP_LOGI("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
        ESP_LOGI("adc_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
        ESP_LOGI("modem_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleNB));
        ESP_LOGI("btn_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleBtn));
    }

    EventBits_t uxBits;

    uxBits = xEventGroupWaitBits(
        status_event_group,                               /* The event group being tested. */
        END_RADIO | NB_TERMINAL | SERIAL_TERMINAL_ACTIVE, /* The bits within the event group to wait for. */
        pdFALSE,                                          /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        wait * 60000 / portTICK_PERIOD_MS);

    // если serial terminal
    if (uxBits & (NB_TERMINAL | SERIAL_TERMINAL_ACTIVE))
    {
        while ((xEventGroupWaitBits(
                    status_event_group,     /* The event group being tested. */
                    SERIAL_TERMINAL_ACTIVE, /* The bits within the event group to wait for. */
                    pdTRUE,                 /* BIT_0 & BIT_1 should be cleared before returning. */
                    pdFALSE,
                    wait * 60000 / portTICK_PERIOD_MS) &
                SERIAL_TERMINAL_ACTIVE) != 0)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        };
    };

    vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 1);

    xTaskNotify(xHandleWifi, NOTYFY_WIFI_STOP, eSetValueWithOverwrite);
    vTaskDelay(1);

    if (terminal_mode > -1)
        ESP_LOGI("info", "Free memory: %lu bytes", esp_get_free_heap_size());

    // засыпаем...
    if (BattLow < 100)
    {
        pcf8575_read(0); // reset INT_PIN
                         // if (gpio_get_level(PCF_INT_PIN) == 1)
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(PCF_INT_PIN), ESP_GPIO_WAKEUP_GPIO_LOW));
    }

    // коррекция на время работы
    uint64_t time_in_us = StoUS(Trepeatlv * (BattLow + 1) * 60) - (esp_timer_get_time() % StoUS(Trepeatlv * 60));
    // uint64_t time_in_us = StoUS(Trepeatlv * (BattLow + 1) * 60);

    ESP_LOGW("main", "Go sleep: %lld мин, (k BattLow: %d)", time_in_us / 1000000 / 60, BattLow);
    fflush(stdout);

    esp_sleep_enable_timer_wakeup(time_in_us);
    esp_deep_sleep_start();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}
