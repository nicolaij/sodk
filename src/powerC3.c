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

#define ADC_BLOCK 256

typedef struct
{
    adc1_channel_t channel;
    int k;
    int max;
    int max0db;
} measure_t;

const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC1_CHANNEL_4, .k = 1, .max = 29999}, //~х22
    {.channel = ADC2_CHANNEL_0, .k = 1, .max = 1000},  //Напряжение источника питания
    {.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .k = 1, .max = 1000},  //Напряжение 0 проводника
};

#define ADC_DMA 4

extern menu_t menu[];

static void continuous_adc_init(int chan)
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    // esp_err_t adc_digi_filter_set_config(adc_digi_filter_idx_tidx, adc_digi_filter_t *config);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = ADC_BLOCK * 4,
        .conv_num_each_intr = ADC_BLOCK,
        .adc1_chan_mask = (1 << chan_r[chan].channel) | (1 << chan_r[3].channel) | (1 << chan_r[4].channel),
        .adc2_chan_mask = 1 << chan_r[2].channel,
    };

    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    static adc_digi_pattern_table_t adc_pattern[10] = {0};

    // Do not set the sampling frequency out of the range between `SOC_ADC_SAMPLE_FREQ_THRES_LOW` and `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
    adc_digi_config_t dig_cfg = {
        .conv_limit_en = 0,
        .conv_limit_num = 200,
        .sample_freq_hz = 40000,
        .adc_pattern_len = ADC_DMA,
    };

    if (chan == 0)
    {
        adc_pattern[0].atten = ADC_ATTEN_DB_11;
        adc_pattern[0].channel = chan_r[0].channel;
        adc_pattern[0].unit = 0;
    }
    else
    {
        adc_pattern[0].atten = ADC_ATTEN_DB_11;
        adc_pattern[0].channel = chan_r[1].channel;
        adc_pattern[0].unit = 0;
    }

    adc_pattern[1].atten = ADC_ATTEN_DB_11;
    adc_pattern[1].channel = chan_r[2].channel;
    adc_pattern[1].unit = 1;

    adc_pattern[2].atten = ADC_ATTEN_DB_11;
    adc_pattern[2].channel = chan_r[3].channel;
    adc_pattern[2].unit = 0;

    adc_pattern[3].atten = ADC_ATTEN_DB_11;
    adc_pattern[3].channel = chan_r[4].channel;
    adc_pattern[3].unit = 0;

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

    // adc1_config_width(ADC_WIDTH_BIT_12);
    // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    while (1)
    {
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        reset_sleep_timeout();

        continuous_adc_init(0);

        //подаем питание на источник питания
        ESP_ERROR_CHECK(gpio_set_level(POWER_PIN, 1));
        vTaskDelay(1);
        /*
                //Измеряем акк перед включением силы
                uint32_t adc_batt[2];
                const uint32_t adc_samples = 32;
                adc_batt[0] = 0;
                for (int i = 0; i < adc_samples; i++)
                {
                    adc_batt[0] += adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_0);
                }
                adc_batt[0] /= adc_samples;
        */
        //включаем источник питания
        ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
        /*
                //Измеряем акк после включения силы
                adc_batt[1] = 0;
                for (int i = 0; i < adc_samples; i++)
                {
                    adc_batt[1] += adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_0);
                }
                adc_batt[1] /= adc_samples;

                printf("Ubat: %d/%d\n", adc_batt[0], adc_batt[1]);
        */

        uint8_t *ptr = (uint8_t *)bufferADC;
        uint8_t *ptr_off = (uint8_t *)bufferADC;  //выключаем источник питания
        uint8_t *ptr_chan = (uint8_t *)bufferADC; //переключаем канал
        int triggerChan = 1;

        int64_t timeout = menu[0].val * 1000;

        //(Лимит напряж - смещение) * коэфф. U
        int adcU_limit = menu[1].val * 1000 / menu[2].val;

        int64_t t1 = esp_timer_get_time();
        // int64_t time_off = 0;

        ESP_ERROR_CHECK(adc_digi_start());

        do
        {
            ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, ADC_BLOCK, &ret_num, ADC_MAX_DELAY));
            /*            adc_digi_output_data_t *p = (void *)ptr;
                        if (p->type2.data > 500)
                            p->type2.data = p->type2.data - 500;
                        else
                            p->type2.data = 0;
            */
            ptr = ptr + ret_num;

            // vTaskDelay(1);

            // memset(ptr, 0, ret_num);
            // printf("ret_num: %d\n", ret_num);

            if (ptr_off == (uint8_t *)bufferADC)
            {
                //отсечка по времени
                if ((esp_timer_get_time() - t1) > timeout)
                {
                    //ВЫРУБАЕМ
                    gpio_set_level(ENABLE_PIN, 1);
                    ptr_off = ptr;
                    // time_off = esp_timer_get_time();
                    // printf("off count: %d\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_DMA * 4));
                }
                else
                {
                    //отсечка по току, напряжению
                    const int count_avg = 4;
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
                        } while (p->type2.unit != 1 || p->type2.channel != 0);
                        sum_avg_u += p->type2.data;

                        //ток
                        do
                        {
                            p--;
                        } while (p->type2.unit != 0 || p->type2.channel != 1);
                        sum_avg_c += p->type2.data;
                    };

                    if (sum_avg_c / count_avg > 4060 || sum_avg_u / count_avg > adcU_limit)
                    {
                        //ВЫРУБАЕМ
                        gpio_set_level(ENABLE_PIN, 1);
                        ptr_off = ptr;
                        // time_off = esp_timer_get_time();
                        // printf("off: %d\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_DMA * 4));
                    };

                    //Переполнен буффер!
                    if (ptr >= (uint8_t *)&bufferADC[DATALEN * 2] - (ADC_BLOCK * 4))
                        ptr = ptr - ret_num;
                };
            }
            else
            //Отключили питание
            {
                if (triggerChan > 0) //проверяем чувствительность
                {
                    // int64_t t1 = esp_timer_get_time();
                    const int count_avg = 4;
                    int sum_avg_c = 0;
                    int n = 0;
                    adc_digi_output_data_t *p = (void *)ptr;
                    while (n++ < count_avg)
                    {
                        //ток
                        do
                        {
                            p--;
                        } while (p->type2.unit != 0 || p->type2.channel != 1);
                        sum_avg_c += p->type2.data;
                    };

                    // printf("sum: %d / %d\n", sum_avg_c, count_avg);

                    if ((sum_avg_c / count_avg) < 145)
                    {
                        //переключаем
                        ptr_chan = ptr;
                        triggerChan = -1;
                        ESP_ERROR_CHECK(adc_digi_stop());
                        ESP_ERROR_CHECK(adc_digi_deinitialize());
                        continuous_adc_init(1);
                        ESP_ERROR_CHECK(adc_digi_start());
                    };
                    int64_t t2 = esp_timer_get_time();
                    // printf("dt: %lld\n", t2 - t1);
                    triggerChan--;
                };
            };
        } while (ptr < (uint8_t *)&bufferADC[DATALEN * 2] - ADC_BLOCK);

        gpio_set_level(POWER_PIN, 0);

        ESP_ERROR_CHECK(adc_digi_stop());
        ESP_ERROR_CHECK(adc_digi_deinitialize());

        processBuffer(ptr, ptr_chan, ptr_off);

        xEventGroupSetBits(ready_event_group, END_MEASURE);
    };
}

int volt(int adc)
{
    if (adc < 2)
        return 0;
    return ((adc * menu[2].val) + menu[3].val);
};

int voltBatt(int adc)
{
    if (adc < 2)
        return 0;
    return ((adc * menu[8].val) + menu[9].val) / 1000;
};

int volt0(int adc)
{
    if (adc < 2)
        return 0;
    return ((adc * menu[10].val) + menu[11].val);
};

int current(int adc)
{
    if (adc < 2)
        return 0;
    return ((adc * menu[4].val) + menu[5].val);
};

int kOm(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    int k = menu[4].val;
    if (adc_r < 10)
        return chan_r[0].max;
    if (adc_u < 10)
        return 0;
    int r = u * 1000 / (k * adc_r + menu[5].val);
    if (r > chan_r[0].max || r < 0)
        return chan_r[0].max;
    return r;
};

int current0(int adc)
{
    if (adc < 2)
        return 0;
    return ((adc * menu[6].val) + menu[7].val);
};

int kOm0db(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    int k = menu[6].val;
    if ((adc_r) < 10)
        return chan_r[1].max;
    if ((adc_u) < 10)
        return 0;
    int r = u * 1000 / (k * adc_r + menu[7].val);
    if (r > chan_r[1].max || r < 0)
        return chan_r[1].max;
    return r;
};

void processBuffer(uint8_t *endptr, uint8_t *ptr_0db, uint8_t *ptr_off)
{
    //результат измерений
    result_t result = {};

    char buf[128];

    adc_digi_output_data_t *p = (void *)bufferADC;
    int n = 0;

    int count_avg = 16;

    int sum_avg_c = 0;
    int sum_avg_u = 0;
    int sum_avg_batt = 0;
    int sum_avg_u0 = 0;
    int sum_n = 0;

    int batt_counter = 32;
    int batt_min = voltBatt(4095);

    const int filter_avg = 5;
    int sum_adc1 = 0;
    int sum_adc2 = 0;
    int sum_n_adc = 0;

    int pre_adc1[32];
    int pre_adc2[32];
    int pre_pos = 0;

    int block_off = 0;
    int block_measure = 0;

    /*    while ((uint8_t *)p < endptr)
        {
            for (int i = 0; i < ADC_DMA; i++)
            {
                printf("%d,%d, %4d, ", p->type2.unit, p->type2.channel, p->type2.data);
                p++;
            }
            printf("\n");
        };
        p = (void *)bufferADC;
    */
    sprintf(buf, "off %d, adc %d", (ptr_off - (uint8_t *)bufferADC) / (ADC_DMA * 4), (ptr_0db - (uint8_t *)bufferADC) / (ADC_DMA * 4));
    xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
    printf("%s\n", buf);

    while ((uint8_t *)p < endptr)
    {
        if (p->type2.unit == 0 && (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == 0 && (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == 0 && (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == 3)
        {
            int adc1 = p->type2.data;
            int adc2 = (p + 1)->type2.data;
            int adc3 = (p + 2)->type2.data;
            int adc4 = (p + 3)->type2.data;

            if ((uint8_t *)p >= ptr_off)
            {
                //если перегрузка то сдвигаем расчет сопротивления
                //если переключали каналы - то блок после ptr_0db
                if (adc1 > 4090 || (uint8_t *)p < ptr_0db)
                {
                    block_measure = 0;
                };

                if (block_off >= 0)
                {
                    if (block_off == 0)
                        sum_n = 0;

                    block_off++;
                };

                if (block_measure >= 0)
                {
                    if (block_measure == 0)
                        sum_n = 0;

                    block_measure++;
                };
            };

            if (sum_n == 0)
            {
                sum_avg_u = 0;
                sum_avg_c = 0;
                sum_avg_batt = 0;
                sum_avg_u0 = 0;
            }

            sum_n++;
            sum_avg_u += volt(adc2);
            sum_avg_batt += voltBatt(adc3);
            sum_avg_u0 += volt0(adc4);

            if (ptr_0db > (uint8_t *)bufferADC && (uint8_t *)p >= ptr_0db)
            {
                sum_avg_c += kOm0db(adc2, adc1);
                // sum_avg_c += current0(adc1);
            }
            else
            {
                sum_avg_c += kOm(adc2, adc1);
                // sum_avg_c += current(adc1);
            }

            if (batt_counter > 0)
            {
                if (voltBatt(adc3) < batt_min)
                    batt_min = voltBatt(adc3);

                batt_counter--;
            }

            /*          if ((uint8_t *)p == ptr_off)
                            printf("off %d -------------------------------------\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_DMA * 4));
                        if ((uint8_t *)p == ptr_0db)
                            printf("adc1 %d ------------------------------------\n", (ptr_0db - (uint8_t *)bufferADC) / (ADC_DMA * 4));
            */

            // if ((uint8_t *)p == ptr_off || (uint8_t *)p == ptr_0db)
            //     printf("sum_n: %d\n", sum_n);

            if (sum_n == count_avg)
            {
                if (block_off >= count_avg) //первый блок после отключения или после ptr_0db
                {
                    result.adc1 = adc1;
                    result.adc2 = adc2;
                    result.R = sum_avg_c / sum_n;
                    result.U = sum_avg_u / sum_n / 1000;
                    result.U0 = sum_avg_u0 / sum_n / 1000;
                    result.Ubatt1 = batt_min;
                    result.Ubatt0 = sum_avg_batt / sum_n;
                    block_off = -1;
                    // printf("-------------------------------------\n");
                }

                if (block_measure >= count_avg) //первый блок после ptr_0db
                {
                    result.adc1 = adc1;
                    result.R = sum_avg_c / sum_n;
                    block_measure = -1;
                }

                // printf("%5d, %4d, %4d, %4d, %4d\n", n++, adc1, adc2, sum_avg_c / sum_n, sum_avg_u / sum_n);
                sprintf(buf, "%5d, %4d, %5d, %4d, %5d, %4d, %5d, %4d, %5d", n++, adc1, sum_avg_c / sum_n, adc2, sum_avg_u / sum_n / 1000, adc3, sum_avg_batt / sum_n, adc4, sum_avg_u0 / sum_n);
                // xQueueOverwrite(ws_send_queue, (char *)buf);
                xQueueSend(ws_send_queue, (char *)buf, (portTickType)1);
                printf("%s\n", buf);
                sum_n = 0;
            };

            if (n % 1000 == 0)
                vTaskDelay(1);
        }
        else
        {
            printf("skip %d-%d %d-%d %d-%d %d-%d\n", p->type2.unit, p->type2.channel, (p + 1)->type2.unit, (p + 1)->type2.channel, (p + 2)->type2.unit, (p + 2)->type2.channel, (p + 3)->type2.unit, (p + 3)->type2.channel);
            p++;
            continue;
        };

        p = p + ADC_DMA;
    };

    xQueueSend(send_queue, (void *)&result, (portTickType)0);

    return;
}

int getADC_Data(char *line, uint8_t **ptr_adc, int *num)
{
    adc_digi_output_data_t *p = (void *)(*ptr_adc);

    if ((*ptr_adc) == 0)
    {
        p = (void *)bufferADC;
        (*ptr_adc) = (uint8_t *)p;
        strcpy(line, "id, adc0, adc1, adc2, adc3\n");
        return 2;
    };

    while ((uint8_t *)(p + ADC_DMA) < (uint8_t *)bufferADC + sizeof(bufferADC))
    {
        if (p->type2.unit == 0 && (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == 0 && (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == 0 && (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == 3)
        {
            sprintf(line, "%5d, %4d, %4d, %4d, %4d\n", (*num)++, p->type2.data, (p + 1)->type2.data, (p + 2)->type2.data, (p + 3)->type2.data);
            // printf("%s", line);
        }
        else
        {
            p++;
            continue;
        };

        p = p + ADC_DMA;
        (*ptr_adc) = (uint8_t *)p;
        return 1;
    };

    return 0;
};
