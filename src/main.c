/*
    espnow MAC0 8713987 MAC3 3784525
*/

#include "main.h"
#include <sys/time.h>

#include <stdio.h>

#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_timer.h"

#define PCF8575_I2C_ADDR_BASE 0x20

#include <string.h>

#include "driver/i2c_master.h"
#include "driver/temperature_sensor.h"

i2c_master_dev_handle_t dev_handle_pcf;

RTC_DATA_ATTR unsigned int bootCount = 0;

float tsens_out = 0;

TaskHandle_t xHandleNB = NULL;
TaskHandle_t xHandleConsole = NULL;
TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandleADC = NULL;
TaskHandle_t xHandleBtn = NULL;

int i2c_err = 0;

QueueHandle_t uicmd_queue;
QueueHandle_t send_queue;
QueueHandle_t adc_queue;
QueueHandle_t set_lora_queue;
RingbufHandle_t wsbuf_handle;

EventGroupHandle_t status_event_group;

bool start_adc_init = false;

int pcf8575_read(int bit)
{
    if (i2c_err)
        return -1;

    static uint16_t port_val = UINT16_MAX;

    esp_err_t err = i2c_master_receive(dev_handle_pcf, (uint8_t *)&port_val, 2, 100);

    if (err == ESP_OK)
    {
        ESP_LOGD("pcf8575", "pcf8575 get: 0x%04X", port_val);

        if ((port_val & BIT(bit)) == 0)
            return 1;
        else
            return 0;
    }
    return -1;
}

//  POWER_BIT - и NB_PWR_BIT - инверсные
//  POWER_BIT - всегда, кроме когда каналы выбраны
esp_err_t pcf8575_set(int channel_cmd)
{
    if (i2c_err)
        return -1;

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
    case POWER_CMDON:
        current_mask &= ~BIT(POWER_BIT); // On HV PWM and ADC amplifier
        cmd_mask = current_mask;
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

    return i2c_master_transmit(dev_handle_pcf, (uint8_t *)&port_val, 2, 100);
}

void start_measure(int channel, int flag)
{
    cmd_t cmd;

    cmd.channel = channel;
    cmd.cmd = flag;

    if (channel == 0)
    {
        const int channels = get_menu_val_by_id("chanord");

        if (flag != 1)
        {
            int pos = 100000000;
            while (pos > 0) // First LV
            {
                cmd.channel = (channels % (pos * 10)) / pos;
                cmd.cmd = 2;
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
            int pos = 100000000;
            while (pos > 0) // HV
            {
                cmd.channel = (channels % (pos * 10)) / pos;
                cmd.cmd = 1;
                if (cmd.channel > 0 && cmd.channel <= 4)
                {
                    xQueueSend(uicmd_queue, &cmd, (TickType_t)0);
                }
                pos /= 10;
            }
        }
    }
    else
    {
        if (flag == 0)
        {
            if (channel > 0 && channel <= 4)
                cmd.cmd = 1;
            else if (channel >= 5)
                cmd.cmd = 2;
        }
        xQueueSend(uicmd_queue, &cmd, (TickType_t)0);
    }

    if (flag < 2)
        if (xHandleNB)
            xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль
}

esp_err_t start_adc_calibrate()
{
    calcdata_t adc_result;
    calcdata_t adc_corr = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_pulldown_en(0);
    gpio_pulldown_en(1);
    gpio_pulldown_en(2);
    gpio_pulldown_en(3);
    gpio_pulldown_en(4);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    gpio_pulldown_dis(0);
    gpio_pulldown_dis(1);
    gpio_pulldown_dis(2);
    gpio_pulldown_dis(3);
    gpio_pulldown_dis(4);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Запускаем опрос портов ADC без каналов и ВВ
    start_measure(-1, 4);
    /* Ждем окончания измерений */
    xEventGroupWaitBits(
        status_event_group, /* The event group being tested. */
        END_MEASURE,        /* The bits within the event group to wait for. */
        pdTRUE,             /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        60000 / portTICK_PERIOD_MS);

    if (pdTRUE == xQueueReceive(adc_queue, &adc_result, 1000 / portTICK_PERIOD_MS))
    {
        adc_corr.Ubatt = adc_result.Ubatt;

        if (adc_result.Ubatt > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 1", "Error Ubat!");
            return ESP_FAIL;
        }
        if (adc_result.R1 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 1", "Error R1!");
            return ESP_FAIL;
        }
        if (adc_result.R2 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 1", "Error R2!");
            return ESP_FAIL;
        }
        if (adc_result.U > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 1", "Error U!");
            return ESP_FAIL;
        }
        if (adc_result.U0 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 1", "Error U0!");
            return ESP_FAIL;
        }
    }
    else
        return ESP_FAIL;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Запускаем опрос портов ADC без каналов с POWER_ON
    start_measure(-1, 5);
    /* Ждем окончания измерений */
    xEventGroupWaitBits(
        status_event_group, /* The event group being tested. */
        END_MEASURE,        /* The bits within the event group to wait for. */
        pdTRUE,             /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        60000 / portTICK_PERIOD_MS);

    if (pdTRUE == xQueueReceive(adc_queue, &adc_result, 1000 / portTICK_PERIOD_MS))
    {
        adc_corr.R1 = adc_result.R1;
        adc_corr.R2 = adc_result.R2;
        adc_corr.U0 = adc_result.U0;
        adc_corr.U = adc_result.U;

        if (adc_result.Ubatt < 2000)
        {
            ESP_LOGE("AUTOCALIBRATE 2", "Error Ubat!");
            return ESP_FAIL;
        }
        if (adc_result.R1 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 2", "Error R1!");
            return ESP_FAIL;
        }
        if (adc_result.R2 > 100)
        {
            ESP_LOGE("AUTOCALIBRATE 2", "Error R2!");
            return ESP_FAIL;
        }
        if (adc_result.U > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 2", "Error U!");
            return ESP_FAIL;
        }
        if (adc_result.U0 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE 2", "Error U0!");
            return ESP_FAIL;
        }
    }
    else
        return ESP_FAIL;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_pulldown_en(0);
    gpio_pulldown_en(1);
    gpio_pulldown_en(2);
    gpio_pulldown_en(3);
    gpio_pulldown_en(4);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    gpio_pulldown_dis(0);
    gpio_pulldown_dis(1);
    gpio_pulldown_dis(2);
    gpio_pulldown_dis(3);
    gpio_pulldown_dis(4);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Запускаем опрос портов ADC без каналов и ВВ
    start_measure(-1, 4);
    /* Ждем окончания измерений */
    xEventGroupWaitBits(
        status_event_group, /* The event group being tested. */
        END_MEASURE,        /* The bits within the event group to wait for. */
        pdFALSE,            /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        60000 / portTICK_PERIOD_MS);

    if (pdTRUE == xQueueReceive(adc_queue, &adc_result, 1000 / portTICK_PERIOD_MS))
    {
        if (adc_result.Ubatt > 50)
        {
            ESP_LOGE("AUTOCALIBRATE", "Error Ubat!");
            return ESP_FAIL;
        }
        if (adc_result.R1 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE", "Error R1!");
            return ESP_FAIL;
        }
        if (adc_result.R2 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE", "Error R2!");
            return ESP_FAIL;
        }
        if (adc_result.U > 50)
        {
            ESP_LOGE("AUTOCALIBRATE", "Error U!");
            return ESP_FAIL;
        }
        if (adc_result.U0 > 50)
        {
            ESP_LOGE("AUTOCALIBRATE", "Error U0!");
            return ESP_FAIL;
        }
    }
    else
        return ESP_FAIL;

    if ((ESP_OK == set_menu_val_by_id("offstADC0", adc_corr.Ubatt)) &&
        (ESP_OK == set_menu_val_by_id("offstADC1", adc_corr.R1)) &&
        (ESP_OK == set_menu_val_by_id("offstADC2", adc_corr.R2)) &&
        (ESP_OK == set_menu_val_by_id("offstADC3", adc_corr.U0)) &&
        (ESP_OK == set_menu_val_by_id("offstADC4", adc_corr.U)))
        return ESP_OK;

    return ESP_FAIL;
}

void app_main(void)
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

    ESP_LOGI("main", "Current date/time: %s", get_datetime(time(0)));

    wsbuf_handle = xRingbufferCreate(10 * WS_BUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);

    ESP_LOGD("main", "Initializing Temperature sensor");

    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_out));

    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_sensor));

    char buf[24];
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

    err = read_nvs_menu();

    if (err != ESP_OK)
    {
        // Первый запуск
        start_adc_init = true;
    }

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
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
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle_pcf));

    // init pcf outs
    if (pcf8575_set(0) != ESP_OK)
    {
        i2c_err = 1;
        ESP_LOGE("i2c", "Error. pcf8575 disabled!");
    }

    uicmd_queue = xQueueCreate(8, sizeof(cmd_t));
    send_queue = xQueueCreate(10, sizeof(result_t));
    adc_queue = xQueueCreate(3, sizeof(calcdata_t));

    status_event_group = xEventGroupCreate();

    xTaskCreate(wifi_task, "wifi_task", 1024 * 3, NULL, configMAX_PRIORITIES - 10, &xHandleWifi);

    xTaskCreate(console_task, "console_task", 1024 * 3, NULL, configMAX_PRIORITIES - 20, &xHandleConsole);

    // xTaskCreate(btn_task, "btn_task", 1024 * 2, NULL, configMAX_PRIORITIES - 20, &xHandleBtn);

    xTaskCreate(modem_task, "modem_task", 1024 * 5, NULL, configMAX_PRIORITIES - 15, &xHandleNB);

    xTaskCreate(adc_task, "adc_task", 1024 * 3, (void *)&wakeup_reason, configMAX_PRIORITIES - 5, &xHandleADC);

    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 7);

    int Trepeatlv = get_menu_val_by_id("Trepeatlv");
    int Trepeathv = get_menu_val_by_id("Trepeathv");

    if (start_adc_init) // АВТОКАЛИБРОВКА ADC
    {
        if (ESP_OK == start_adc_calibrate())
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }
    }

    if ((bootCount % (Trepeathv / Trepeatlv) == 1 || wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) && BattLow < 100)
    {
        start_measure(0, 0);
    }
    else
    {
        start_measure(0, 2); // LV only
    }

    /* Ждем окончания измерений */
    xEventGroupWaitBits(
        status_event_group, /* The event group being tested. */
        END_MEASURE,        /* The bits within the event group to wait for. */
        pdFALSE,            /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,
        60000 / portTICK_PERIOD_MS);

    ESP_LOGD("info", "Free memory: %lu bytes", esp_get_free_heap_size());

    // время ожидания
    const int wait = get_menu_val_by_id("waitwifi");

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

    ESP_LOGD("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
    ESP_LOGD("main_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGD("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
    ESP_LOGD("adc_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
    ESP_LOGD("modem_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleNB));
    ESP_LOGD("console_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleConsole));

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

    ESP_LOGD("info", "Free memory: %lu bytes", esp_get_free_heap_size());

    // засыпаем...
    if (BattLow < 100)
    {
        // pcf8575_read(0); // reset INT_PIN
        //  if (gpio_get_level(PCF_INT_PIN) == 1)
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(PCF_INT_PIN), ESP_GPIO_WAKEUP_GPIO_LOW));
    }

    // коррекция на время работы
    Trepeatlv = get_menu_val_by_id("Trepeatlv"); // а вдруг уже настройки поменялись
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
