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
    {.channel = ADC1_CHANNEL_4, .k = 1, .max = 5000},
    {.channel = ADC1_CHANNEL_3, .k = 1, .max = 25000},
    {.channel = ADC2_CHANNEL_0, .k = 1, .max = 1000},
};

extern menu_t menu[];

static void continuous_adc_init(int chan)
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    // esp_err_t adc_digi_filter_set_config(adc_digi_filter_idx_tidx, adc_digi_filter_t *config);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = 256,
        .adc1_chan_mask = 1 << chan_r[chan].channel,
        .adc2_chan_mask = 1 << chan_r[2].channel,
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

    adc_pattern[0].unit = 0;
    if (chan == 0)
    {
        adc_pattern[0].atten = ADC_ATTEN_DB_11;
        adc_pattern[0].channel = chan_r[0].channel;
    }
    else
    {
        adc_pattern[0].atten = ADC_ATTEN_DB_0;
        adc_pattern[0].channel = chan_r[1].channel;
    }

    adc_pattern[1].atten = ADC_ATTEN_DB_11;
    adc_pattern[1].channel = chan_r[2].channel;
    adc_pattern[1].unit = 1;

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_config(&dig_cfg));
}

void dual_adc(void *arg)
{

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    uint32_t ret_num = 0;

    cmd_t cmd;
    cmd.cmd = 0;
    cmd.power = 0;

    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = (1 << ENABLE_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << POWER_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(POWER_PIN, 0));

    while (1)
    {
        continuous_adc_init(0);
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        ESP_ERROR_CHECK(gpio_set_level(POWER_PIN, 1));
        vTaskDelay(1);
        ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));

        uint8_t *ptr = (uint8_t *)bufferADC;
        uint8_t *ptr_off = 0; //выключаем источник питания
        uint8_t *ptr_chan = 0; //переключаем канал
        int triggerChan = 1;

        int64_t time_off = 0;
        int64_t t1 = esp_timer_get_time();
        int64_t timeout = menu[0].val * 1000;

        //(Лимит напряж - смещение) * коэфф. U
        int adcU_limit = ((menu[1].val - menu[3].val) * menu[2].val) / 1000;

        ESP_ERROR_CHECK(adc_digi_start());

        while (ptr < (uint8_t *)&bufferADC[DATALEN * 2] - 256)
        {
            ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, 256, &ret_num, ADC_MAX_DELAY));
            ptr = ptr + ret_num;

            if (ptr_off == 0)
            {
                //отсечка по времени
                if ((esp_timer_get_time() - t1) > timeout)
                {
                    //ВЫРУБАЕМ
                    gpio_set_level(ENABLE_PIN, 1);
                    gpio_set_level(POWER_PIN, 0);
                    ptr_off = ptr;
                    time_off = esp_timer_get_time();
                    printf("off count: %d\n", (ptr_off - (uint8_t *)bufferADC) / 8);
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
                            gpio_set_level(ENABLE_PIN, 1);
                            gpio_set_level(POWER_PIN, 0);
                            ptr_off = ptr;
                            time_off = esp_timer_get_time();

                            printf("off: %d\n", (ptr_off - (uint8_t *)bufferADC) / 8);
                        };
                    };
                };
            }
            else
            //Отключили питание
            {
                if (triggerChan > 0) //проверяем чувствительность
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

                    if (sum_avg_c / count_avg < 500)
                    {
                        //переключаем
                        ptr_chan = ptr;
                        triggerChan = -1;
                        ESP_ERROR_CHECK(adc_digi_stop());
                        ESP_ERROR_CHECK(adc_digi_deinitialize());
                        continuous_adc_init(1);
                        ESP_ERROR_CHECK(adc_digi_start());
                    };
                    // int64_t t2 = esp_timer_get_time();
                    // printf("dt: %lld\n", t2 - t1);
                    triggerChan--;
                };
            };
        };

        ESP_ERROR_CHECK(adc_digi_stop());
        ESP_ERROR_CHECK(adc_digi_deinitialize());

        processBuffer(ptr, ptr_chan, ptr_off);

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
        return chan_r[1].max;
    if (adc_u < 2)
        return 0;
    int r = u * k / (adc_r + menu[7].val);
    if (r > chan_r[1].max)
        return chan_r[1].max;
    return r;
};

void processBuffer(uint8_t *endptr, uint8_t *ptr_0db, uint8_t *ptr_off)
{
    //результат измерений
    result_t result = {};

    adc_digi_output_data_t *p = (void *)bufferADC;
    int n = 0;

    const int count_avg = 32;
    int sum_avg_c = 0;
    int sum_avg_u = 0;
    int sum_n = 0;

    int block_off = -1;

    while ((uint8_t *)(p + 1) < endptr)
    {
        if (p->type2.unit == 0 && (p + 1)->type2.unit == 1)
        {
            int adc1 = p->type2.data;
            int adc2 = (p + 1)->type2.data;

            sum_n++;
            sum_avg_u += volt(adc2);
            sum_avg_c += ((uint8_t *)p < ptr_0db) ? kOm(adc2, adc1) : kOm0db(adc2, adc1);

            if ((uint8_t *)p >= ptr_off)
            {
                block_off++;
            };

            if (sum_n == count_avg || block_off == 0)
            {
                if (block_off == count_avg) //первый блок после отключения
                {
                    result.adc11 = adc1;
                    result.adc2 = adc2;
                    result.U = sum_avg_u / sum_n;
                    result.R = sum_avg_c / sum_n;
                    // printf("-------------------------------------\n");
                };

                printf("%5d, %4d, %4d, %4d, %4d\n", n++, adc1, adc2, sum_avg_c / sum_n, sum_avg_u / sum_n);
                sum_avg_u = 0;
                sum_avg_c = 0;
                sum_n = 0;
            };

            if (n % 1000 == 0)
                vTaskDelay(1);
        }
        else
        {
            p++;
            continue;
        };

        p = p + 2;
    };

    xQueueSend(send_queue, (void *)&result, (portTickType)0);

    return;
}
