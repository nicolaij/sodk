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
} measure_t;

measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_1, .k = 90, .max = 5000},
};

extern menu_t menu[];

static void continuous_adc_init()
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = 256,
        .adc1_chan_mask = 1 << ADC1_CHANNEL_1,
        .adc2_chan_mask = 1 << ADC2_CHANNEL_0,
    };
    ret = adc_digi_initialize(&adc_dma_config);
    assert(ret == ESP_OK);

    adc_digi_pattern_table_t adc_pattern[10] = {0};

    // Do not set the sampling frequency out of the range between `SOC_ADC_SAMPLE_FREQ_THRES_LOW` and `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
    adc_digi_config_t dig_cfg = {
        .conv_limit_en = 0,
        .conv_limit_num = 255,
        .sample_freq_hz = 1000,
        .adc_pattern_len = 2,
    };

    adc_pattern[0].atten = ADC_ATTEN_DB_11;
    adc_pattern[0].channel = ADC1_CHANNEL_1;
    adc_pattern[0].unit = 0;

    adc_pattern[1].atten = ADC_ATTEN_DB_11;
    adc_pattern[1].channel = ADC2_CHANNEL_0;
    adc_pattern[1].unit = 1;

    dig_cfg.adc_pattern = adc_pattern;
    ret = adc_digi_controller_config(&dig_cfg);
    assert(ret == ESP_OK);
}

void dual_adc(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;

    //результат измерений
    result_t result;

    cmd_t cmd;
    cmd.cmd = 0;
    cmd.power = 0;
    int64_t t1 = esp_timer_get_time();
    int64_t timeout = menu[0].val * 1000;

    continuous_adc_init();
    adc_digi_start();

    while (1)
    {
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        int n = 1;
        while (n--)
        {
            ret = adc_digi_read_bytes((uint8_t *)bufferADC, 256, &ret_num, ADC_MAX_DELAY);
            printf("adc_digi_read_bytes: %d\n", ret_num);
            adc_digi_output_data_t *p = (void *)bufferADC;
            for (int i = 0; i < ret_num; i += 4)
            {
                printf("ADC%d_CH%d: %x\n", p->type2.unit + 1, p->type2.channel, p->type2.data);
                p++;
            }
            // If you see task WDT in this task, it means the conversion is too fast for the task to handle
        }

        result.adc11 = 0;
        result.adc2 = 0;
        result.U = 0;
        result.R = 0;

        xQueueSend(send_queue, (void *)&result, (portTickType)0);
        xEventGroupSetBits(ready_event_group, BIT0);
    };

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
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
