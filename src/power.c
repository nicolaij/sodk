#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_rom_sys.h"

#include <driver/dac.h>
#include "driver/ledc.h"

#include "main.h"

#define LED_GPIO 2

int bufferR[10000];
int bufferU[10000];
int buffer3[10000];

typedef struct
{
    adc1_channel_t channel;
    int k;
    int max;
} measure_t;

measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_5, .k = 90, .max = 5000},
    {.channel = ADC1_CHANNEL_7, .k = 1750, .max = 20000},
};

extern menu_t menu[];

/**
 * @brief debug buffer data
 */
void disp_buf(uint8_t *buf, int length)
{
    printf("======\n");
    for (int i = 0; i < length; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 32 == 0)
        {
            printf("\n");
        }
    }
    printf("======\n");
}

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

void dual_adc(void *arg)
{
    check_efuse();

    gpio_pad_select_gpio(POWER_PIN);
    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_PIN, 0);

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, 0);

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(chan_r[0].channel, ADC_ATTEN_11db);
    adc1_config_channel_atten(chan_r[1].channel, ADC_ATTEN_11db);

    adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_11db);

    //результат измерений
    result_t result;

    cmd_t cmd;
    cmd.cmd = 0;
    cmd.power = 0;

    while (1)
    {

        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        dac_output_voltage(DAC_CHANNEL_1, cmd.power);

        int64_t t1 = esp_timer_get_time();
        int64_t timeout = menu[0].val * 1000;

        int len = 0;
        int pos_off = 0;

        if (cmd.cmd >= 2) // Pulse
        {
            int u = 0;
            int r = 0;
            //int pre_u = 0;
            int sum_u = 0;
            int sum_count = 0;
            gpio_set_level(POWER_PIN, 1);
            while (esp_timer_get_time() - t1 < (timeout * 2))
            {
                if (esp_timer_get_time() - t1 > timeout || u > 4090 || r > 4090)
                {
                    gpio_set_level(POWER_PIN, 0);
                    if (pos_off == 0)
                    {
                        pos_off = len;
                    }
                }

                // adc_read[0] = adc1_get_raw(ADC1_CHANNEL_5);
                // adc_read[0] = adc1_get_raw(ADC1_CHANNEL_7);
                bufferR[len] = adc1_get_raw(chan_r[0].channel);
                adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &bufferU[len]);

                const int adc_avg_count = 10;

                if (len > 0)
                {
                    r = (bufferR[len - 1] + bufferR[len]) / 2;
                    /*
                    //запоминаем предыдущее среднее
                    if (len % adc_avg_count == 0)
                    {
                        //считаем скорость роста (<1%)
                        if (u < (pre_u + (pre_u / 100)))
                        {
                            gpio_set_level(POWER_PIN, 0);
                            if (pos_off == 0)
                            {
                                pos_off = len;
                            }
                        };
                        pre_u = u;
                    }
                    */
                    sum_u += bufferU[len];
                    sum_count++;

                    if (sum_count > adc_avg_count)
                    {
                        sum_u -= bufferU[len - adc_avg_count];
                        sum_count--;
                        u = sum_u / adc_avg_count;
                    }
                    else
                    {
                        u = sum_u / sum_count;
                    }
                }

                len++;
                if (len >= sizeof(bufferR) / sizeof(bufferR[0]))
                    break;

                //если низкие напряжения - то выходим
                if (pos_off > 0)
                {
                    if (r < 10 || u < 10)
                        break;
                }
            };
            gpio_set_level(POWER_PIN, 0);

            int sum_r = 0;
            int sum_res = 0;
            sum_u = 0;
            const int res_avg_count = 10;
            int avg_count = res_avg_count;
            int pos = pos_off;
            sum_count = 0;
            while (avg_count > 0)
            {
                sum_r += bufferR[pos];
                sum_u += bufferU[pos];
                sum_res += kOm(bufferU[pos], bufferR[pos]);
                sum_count++;

                avg_count--;

                if (bufferR[pos] > 4090)
                {
                    avg_count = res_avg_count;
                    sum_r = 0;
                    sum_res = 0;
                    sum_u = 0;
                    sum_count = 0;
                }

                pos++;
                if (pos >= len)
                    break;
            }

            if (sum_count > 0)
            {
                result.adc11 = sum_r / sum_count;
                result.adc12 = 0;

                result.adc2 = sum_u / sum_count;
                result.U = volt(result.adc2);
                result.R = sum_res / sum_count;
            }
            else
            {
                result.adc11 = 0;
                result.adc2 = 0;
                result.U = 0;
                result.R = 0;
            }

            xQueueSend(adc1_queue, (void *)&result, (portTickType)0);
            xQueueSend(send_queue, (void *)&result, (portTickType)0);

            printf("pos_off: %d; last u:%d\n", pos_off, u);
            printf("N, U adc, R adc, R kOm\n");
            vTaskDelay(5);
            // if (cmd.cmd == 2)
            for (int i = 0; i < len; i++)
            {
                // printf("%4d: %4d (%4d V), %4d (%4d kOm)\n", i, buffer1[i], volt(buffer1[i]), buffer2[i], kOm(buffer1[i], buffer2[i]));
                // printf("%4d, %4d, %4d\n", i, volt(buffer2[i]), kOm(buffer2[i], buffer1[i]));

                printf("%4d, %4d, %4d, %4d\n", i, bufferU[i], bufferR[i], kOm(bufferU[i], bufferR[i]));

                if (i % 1000 == 0)
                    vTaskDelay(1);
            }
            xEventGroupSetBits(ready_event_group, BIT0);
        }
        else if (cmd.cmd == 1) // ON
        {
            gpio_set_level(POWER_PIN, 1);
            while (cmd.cmd == 1)
            {
                len = 0;
                t1 = esp_timer_get_time();
                while (esp_timer_get_time() - t1 < timeout * 2)
                {
                    // adc_read[0] = adc1_get_raw(ADC1_CHANNEL_5);
                    bufferR[len] = adc1_get_raw(chan_r[0].channel);
                    adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &bufferU[len]);
                    len++;
                    if (len >= sizeof(bufferR) / sizeof(bufferR[0]))
                        break;
                };

                int s1 = 0;
                int s2 = 0;
                int r = 0;
                int offs = 0;
                int avg_count = len - offs;
                for (int i = 0; i < avg_count; i++)
                {
                    s1 += bufferR[offs + i];
                    s2 += bufferU[offs + i];
                    r += kOm(bufferU[offs + i], bufferR[offs + i]);
                }

                result.adc11 = s1 / avg_count;
                result.adc12 = 0;

                result.adc2 = s2 / avg_count;
                result.U = volt(result.adc2);
                result.R = r / avg_count;

                xQueueSend(adc1_queue, (void *)&result, (portTickType)0);

                printf("ADC2: U = %d V (%4d), ADC1: R = %d kOm (%d,%d) buffer:%d\n", result.U, result.adc2, result.R, result.adc11, result.adc12, len);

                if (pdTRUE == xQueueReceive(uicmd_queue, &cmd, (1000 - menu[0].val) / portTICK_PERIOD_MS))
                {
                    dac_output_voltage(DAC_CHANNEL_1, cmd.power);
                }
            }
            gpio_set_level(POWER_PIN, 0);
        }

        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

int volt(int adc)
{
    if (adc < 2)
        return 0;
    return adc * 1000 / menu[1].val + menu[3].val;
};

int kOm(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    int k = menu[2].val;
    if (adc_r < 2)
        return chan_r[0].max;
    if (adc_u < 2)
        return 0;
    int r = u * k / (adc_r + menu[4].val);
    if (r > chan_r[0].max)
        return chan_r[0].max;
    return r;
};
