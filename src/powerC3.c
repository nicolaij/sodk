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

#include "driver/ledc.h"

#include "main.h"

// int bufferR[DATALEN];
// int bufferU[DATALEN];

uint32_t bufferADC[DATALEN * 2];

typedef struct
{
    adc1_channel_t channel;
    int k;
    int max;
    int max0db;
} measure_t;

measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_4, .k = 1, .max = 5000, .max0db = 15000},
    {.channel = ADC2_CHANNEL_0, .k = 1, .max = 1000},
};

extern menu_t menu[];

static void continuous_adc_init(adc_atten_t adc1_atten)
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    // esp_err_t adc_digi_filter_set_config(adc_digi_filter_idx_tidx, adc_digi_filter_t *config);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = 256,
        .adc1_chan_mask = 1 << chan_r[0].channel,
        .adc2_chan_mask = 1 << chan_r[1].channel,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_pattern_table_t adc_pattern[10] = {0};

    // Do not set the sampling frequency out of the range between `SOC_ADC_SAMPLE_FREQ_THRES_LOW` and `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
    adc_digi_config_t dig_cfg = {
        .conv_limit_en = 0,
        .conv_limit_num = 200,
        .sample_freq_hz = 8000,
        .adc_pattern_len = 2,
    };

    adc_pattern[0].atten = adc1_atten;
    adc_pattern[0].channel = chan_r[0].channel;
    adc_pattern[0].unit = 0;

    adc_pattern[1].atten = ADC_ATTEN_DB_11;
    adc_pattern[1].channel = chan_r[1].channel;
    adc_pattern[1].unit = 1;

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_config(&dig_cfg));
}

void dual_adc(void *arg)
{
    uint32_t ret_num = 0;

    //результат измерений
    result_t result;

    cmd_t cmd;
    cmd.cmd = 0;
    cmd.power = 0;
    continuous_adc_init(ADC_ATTEN_DB_11);

    while (1)
    {
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        ESP_ERROR_CHECK(adc_digi_start());

        uint8_t *ptr = (uint8_t *)bufferADC;
        uint8_t *ptr_end = 0; //выключаем источник питания
        uint8_t *ptr_0db = 0; //переключаем чувствительность
        int trigger0db = 2;

        int64_t time_off = 0;
        int64_t t1 = esp_timer_get_time();
        int64_t timeout = menu[0].val * 1000;

        //(Лимит напряж - смещение) * коэфф. U
        int adcU_limit = ((menu[1].val - menu[3].val) * menu[2].val) / 1000;

        while (ptr < (uint8_t *)&bufferADC[DATALEN * 2] - 256)
        {
            ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, 256, &ret_num, ADC_MAX_DELAY));
            ptr = ptr + ret_num;

            if (ptr_end == 0)
            {
                //отсечка по времени
                if ((esp_timer_get_time() - t1) > timeout)
                {
                    //ВЫРУБАЕМ
                    gpio_set_level(POWER_PIN, 0);
                    ptr_end = ptr;
                    time_off = esp_timer_get_time();
                    printf("off time: %d\n", (ptr_end - (uint8_t *)bufferADC) / 8);
                }
                else
                {
                    //отсечка по току, напряжению
                    const int count_avg = 3;
                    int sum_avg_c = 0;
                    int sum_avg_u = 0;
                    int n = 0;
                    adc_digi_output_data_t *p = (void *)ptr;
                    while (n++ < count_avg)
                    {
                        //напряжение
                        do
                        {
                            p--;
                        } while (p->type2.unit != 1 && (uint8_t *)p > (uint8_t *)bufferADC);
                        sum_avg_u += p->type2.data;

                        //ток
                        do
                        {
                            p--;
                        } while (p->type2.unit != 0 && (uint8_t *)p > (uint8_t *)bufferADC);
                        sum_avg_c += p->type2.data;
                    };

                    if ((uint8_t *)p >= (uint8_t *)bufferADC)
                    {
                        if (sum_avg_c / count_avg > 4090 || sum_avg_u / count_avg > adcU_limit)
                        {
                            //ВЫРУБАЕМ
                            gpio_set_level(POWER_PIN, 0);
                            ptr_end = ptr;
                            time_off = esp_timer_get_time();

                            printf("off: %d\n", (ptr_end - (uint8_t *)bufferADC) / 8);
                        }
                    }
                };
            }
            else
            //Отключили питание
            {
                if (trigger0db > 0) //проверяем чувствительность
                {
                    // int64_t t1 = esp_timer_get_time();
                    const int count_avg = 3;
                    int sum_avg_c = 0;
                    int n = 0;
                    adc_digi_output_data_t *p = (void *)ptr;
                    while (n++ < count_avg)
                    {
                        //ток
                        do
                        {
                            p--;
                        } while (p->type2.unit != 0 && (uint8_t *)p > (uint8_t *)bufferADC);

                        sum_avg_c += p->type2.data;
                    };

                    if (sum_avg_c / count_avg < 1000)
                    {
                        //переключаем
                        ptr_0db = ptr;
                        trigger0db = -1;
                        // printf("stop\n");
                        ESP_ERROR_CHECK(adc_digi_stop());
                        ESP_ERROR_CHECK(adc_digi_deinitialize());
                        continuous_adc_init(ADC_ATTEN_DB_0);
                        // printf("start\n");
                        ESP_ERROR_CHECK(adc_digi_start());
                    };
                    // int64_t t2 = esp_timer_get_time();
                    // printf("dt: %lld\n", t2 - t1);
                    trigger0db--;
                }
            }
        }

        ESP_ERROR_CHECK(adc_digi_stop());

        adc_digi_output_data_t *p = (void *)bufferADC;
        int adc1 = 0;
        int adc2 = 0;
        int n = 0;
        uint8_t *p_process;
        while ((uint8_t *)p < ptr)
        {
            while (p->type2.unit != 0 && (uint8_t *)p < ptr)
            {
                p++;
            }
            p_process = p;
            adc1 = p->type2.data;
            p++;
            while (p->type2.unit != 1 && (uint8_t *)p < ptr)
            {
                p++;
            }
            adc2 = p->type2.data;
            p++;
            if ((uint8_t *)p > ptr)
                break;

            if (p_process < ptr_0db)
            { // 11db
                printf("%5d, %4d, %4d, %4d, %4d\n", n++, adc1, adc2, volt(adc2), kOm(adc2, adc1));
            }
            else
            { // 0db
                printf("%6d, %4d, %4d, %4d, %4d\n", n++, adc1, adc2, volt(adc2), kOm0db(adc2, adc1));
            }
            if (n % 2000 == 0)
                vTaskDelay(1);
        }
        // If you see task WDT in this task, it means the conversion is too fast for the task to handle

        result.adc11 = 0;
        result.adc2 = 0;
        result.U = 0;
        result.R = 0;

        xQueueSend(send_queue, (void *)&result, (portTickType)0);
        xEventGroupSetBits(ready_event_group, BIT0);
    };
}

int volt(int adc)
{
    if (adc < 2)
        return 0;
    return adc * 1000 / menu[2].val + menu[3].val;
};

int kOm(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    int k = menu[4].val;
    if (adc_r < 2)
        return chan_r[0].max;
    if (adc_u < 2)
        return 0;
    int r = u * k / (adc_r + menu[5].val);
    if (r > chan_r[0].max)
        return chan_r[0].max;
    return r;
};

int kOm0db(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    int k = menu[6].val;
    if (adc_r < 2)
        return chan_r[0].max0db;
    if (adc_u < 2)
        return 0;
    int r = u * k / (adc_r + menu[7].val);
    if (r > chan_r[0].max0db)
        return chan_r[0].max0db;
    return r;
};
