#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"

#include "driver/adc.h"

//#include "esp_adc_cal.h"
#include "esp_rom_sys.h"

#include "main.h"

#include "esp_log.h"

#include "soc/apb_saradc_reg.h"

RTC_DATA_ATTR int last_chan = 0;

#define ADC_COUNT_READ 5
#define ADC_FREQ 50000
#define ADC_BLOCK (ADC_FREQ / 1000 * 4) // 1 mc

uint32_t bufferADC[DATALEN];
uint32_t bufferR[2][DATALEN / ADC_COUNT_READ];

//результат измерений
result_t result = {};

typedef struct
{
    adc1_channel_t channel;
    adc_atten_t k;
    int max;
} measure_t;

#define SWAP_ADC1_0 1

#ifdef SWAP_ADC1_0
const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_0, .k = ADC_ATTEN_DB_11, .max = 6999},  //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC2_CHANNEL_0, .k = ADC_ATTEN_DB_11, .max = 1000},  //Напряжение источника питания
    {.channel = ADC1_CHANNEL_4, .k = ADC_ATTEN_DB_11, .max = 99999}, //
    {.channel = ADC1_CHANNEL_1, .k = ADC_ATTEN_DB_11, .max = 10000}, //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .k = ADC_ATTEN_DB_11, .max = 1000},  //Напряжение 0 проводника
};
#else
const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_1, .k = ADC_ATTEN_DB_11, .max = 6999},  //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC2_CHANNEL_0, .k = ADC_ATTEN_DB_11, .max = 1000},  //Напряжение источника питания
    {.channel = ADC1_CHANNEL_4, .k = ADC_ATTEN_DB_11, .max = 99999}, //~х22
    {.channel = ADC1_CHANNEL_0, .k = ADC_ATTEN_DB_11, .max = 10000}, //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .k = ADC_ATTEN_DB_11, .max = 1000},  //Напряжение 0 проводника
};
#endif

//#define ADC_DMA 4

extern menu_t menu[];

extern int BattLow;

void adc_select(int chan)
{
    // printf("PATT1: %8X\n", REG_READ(APB_SARADC_SAR_PATT_TAB1_REG));
    // printf("M: %8X\n", ((chan_r[chan].channel << 2) | ADC_ATTEN_DB_11));

    REG_WRITE(APB_SARADC_SAR_PATT_TAB1_REG, (REG_READ(APB_SARADC_SAR_PATT_TAB1_REG) & ~(0b111111 << 18)) | (((chan_r[chan].channel << 2) | chan_r[chan].k) << 18));

    // REG_WRITE(APB_SARADC_SAR_PATT_TAB1_REG, 0x001e30cf);
    // REG_WRITE(APB_SARADC_SAR_PATT_TAB1_REG, 0x004e30cf);
    // printf("PATT1: %8X\n", REG_READ(APB_SARADC_SAR_PATT_TAB1_REG));
    // printf("PATT2: %8X\n", REG_READ(APB_SARADC_SAR_PATT_TAB2_REG));
};

int adc_cur_chan()
{
    return (REG_READ(APB_SARADC_SAR_PATT_TAB1_REG) >> 20) & 0b111;
};

void adc_info()
{
    printf("APB_SARADC_SAR_PATT_TAB1_REG: %8X\n", REG_READ(APB_SARADC_SAR_PATT_TAB1_REG));
    // printf("APB_SARADC_APB_ADC_CLKM_CONF_REG: %8X\n", REG_READ(APB_SARADC_APB_ADC_CLKM_CONF_REG));
    // printf("APB_SARADC_APB_TSENS_CTRL_REG: %8X\n", REG_READ(APB_SARADC_APB_TSENS_CTRL_REG));
    // printf("APB_SARADC_APB_TSENS_CTRL2_REG: %8X\n", REG_READ(APB_SARADC_APB_TSENS_CTRL2_REG));
};

// return mV
int volt(int adc)
{
    if (adc < 3)
        return 0;
    return ((adc * menu[2].val) / 10 + menu[3].val);
};

// return mV
int voltBatt(int adc)
{
    if (adc < 3)
        return 0;
    return ((adc * menu[8].val) + menu[9].val) / 1000;
};

// return mV
int volt0(int adc)
{
    if (adc < 3)
        return 0;
    return ((adc * menu[10].val) / 10 + menu[11].val);
};

int current(int adc)
{
    if (adc < 3)
        return 0;
    return ((adc * menu[4].val) / 10 + menu[5].val);
};

int kOm(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    if (adc_u < 10 || u < 6000)
        return 0;
    if (adc_r < 10)
        return chan_r[0].max;
    int r = u * 1000 / ((menu[4].val * adc_r) / 10 + menu[5].val);
    if (r > chan_r[0].max || r < 0)
        return chan_r[0].max;
    return r;
};

int current0(int adc)
{
    if (adc < 3)
        return 0;
    return ((menu[6].val * adc) / 10 + menu[7].val);
};

int kOm0db(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    if ((adc_u) < 10 || u < 6000)
        return 0;
    if ((adc_r) < 10)
        return chan_r[2].max;
    int r = u * 1000 / ((menu[6].val * adc_r) / 10 + menu[7].val);
    if (r > chan_r[2].max || r < 0)
        return chan_r[2].max;
    return r;
};
/*
static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW("ADC", "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW("ADC", "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);
    } else {
        ESP_LOGE("ADC", "Invalid arg");
    }

    return cali_enable;
}
*/
static void continuous_adc_init()
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);
    int64_t t1 = esp_timer_get_time();
    int64_t t2 = 0;
    int64_t t3 = 0;

    // esp_err_t adc_digi_filter_set_config(adc_digi_filter_idx_tidx, adc_digi_filter_t *config);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = ADC_BLOCK * 4,
        .conv_num_each_intr = ADC_BLOCK,
        .adc1_chan_mask = (1 << chan_r[0].channel) | (1 << chan_r[2].channel) | (1 << chan_r[3].channel) | (1 << chan_r[4].channel),
        .adc2_chan_mask = 1 << chan_r[1].channel,
    };

    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    t2 = esp_timer_get_time();
    /*
        static adc_digi_pattern_table_t adc_pattern[10] = {0};

        // Do not set the sampling frequency out of the range between `SOC_ADC_SAMPLE_FREQ_THRES_LOW` and `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
        adc_digi_config_t dig_cfg = {
            .conv_limit_en = 0,
            .conv_limit_num = 1,
            .sample_freq_hz = ADC_FREQ,
            .adc_pattern_len = ADC_COUNT_READ // sizeof(chan_r) / sizeof(chan_r[0]) // ADC_DMA,
        };

        int n = 0;
        // Note: Все atten при инициализации должны быть одинаковые!!!???
        adc_pattern[n].atten = ADC_ATTEN_DB_11;
        adc_pattern[n].channel = chan_r[0].channel;
        adc_pattern[n].unit = 0;
        n++;
        adc_pattern[n].atten = ADC_ATTEN_DB_11;
        adc_pattern[n].channel = chan_r[1].channel;
        adc_pattern[n].unit = 1;
        n++;
        adc_pattern[n].atten = ADC_ATTEN_DB_11;
        adc_pattern[n].channel = chan_r[2].channel;
        adc_pattern[n].unit = 0;
        n++;
        adc_pattern[n].atten = ADC_ATTEN_DB_11;
        adc_pattern[n].channel = chan_r[3].channel;
        adc_pattern[n].unit = 0;
        n++;
        adc_pattern[n].atten = ADC_ATTEN_DB_11;
        adc_pattern[n].channel = chan_r[4].channel;
        adc_pattern[n].unit = 0;

        dig_cfg.adc_pattern = adc_pattern;
        ESP_ERROR_CHECK(adc_digi_controller_config(&dig_cfg));
        t3 = esp_timer_get_time();
        // printf("digi_init: %lld digi_config: %lld\n", t1-t2, t2-t3);
    */
    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = false,
        .conv_limit_num = 255,
        .sample_freq_hz = ADC_FREQ,
        .conv_mode = ADC_CONV_BOTH_UNIT,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    int n = 0;
    // Note: Все atten при инициализации должны быть одинаковые!!!???
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[0].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[1].channel;
    adc_pattern[n].unit = 1;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[2].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[3].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[4].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    
    dig_cfg.pattern_num = n;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
    t3 = esp_timer_get_time();
}

void dual_adc(void *arg)
{

    // bool cali_enable = adc_calibration_init();

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
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        reset_sleep_timeout();

        continuous_adc_init();

        uint8_t *ptr = (uint8_t *)bufferADC;
        uint8_t *ptr_on = (uint8_t *)bufferADC;   //включаем источник питания
        uint8_t *ptr_off = (uint8_t *)bufferADC;  //выключаем источник питания
        uint8_t *ptr_chan = (uint8_t *)bufferADC; //переключаем канал
        int blocks = 0;

        int64_t timeout = menu[0].val * 1000;
        int64_t quiet_timeout = 10 * 1000;

        //(Лимит напряж - смещение) * коэфф. U
        int adcU_limit = (menu[1].val * 1000 - menu[3].val) * 10 / menu[2].val;
        // printf("adcU_limit = %d\n", adcU_limit);

        //((adc * menu[8].val) + menu[9].val) / 1000;
        int adcUbattLow = (menu[12].val * 1000 - menu[9].val) / menu[8].val;
        int adcUbattEnd = (menu[13].val * 1000 - menu[9].val) / menu[8].val;

        // int64_t time_off = 0;

        ESP_ERROR_CHECK(adc_digi_start());

        // adc_select(1);

        //подаем питание на источник питания, OpAmp, делитель АЦП батареи
        ESP_ERROR_CHECK(gpio_set_level(POWER_PIN, 1));

        int64_t t1 = esp_timer_get_time();
        int64_t t2 = 0, t3 = 0, time_off = 0;

        int repeate_test = menu[16].val;

        do
        {
            ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, ADC_BLOCK, &ret_num, ADC_MAX_DELAY));
            ptr = ptr + ret_num;
            blocks++;

            if (blocks == 10)
            {
                if (BattLow > 10)
                    break;

                // adc_select(0);
                if ((esp_timer_get_time() - t1) < timeout)
                {
                    //включаем источник питания
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                    ptr_on = ptr;
                }
            }

            if (blocks >= 10)
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
                        //отсечка по току, напряжению, Ubatt
                        const int count_avg = 4;
                        int sum_avg_c = 0;
                        int sum_avg_u = 0;
                        int sum_avg_batt = 0;
                        int n = 0;
                        adc_digi_output_data_t *p = (void *)ptr;
                        while (n++ < count_avg)
                        {
#if ADC_COUNT_READ > 4
                            // Ubatt
                            do
                            {
                                p--;
                            } while (p->type2.unit != 0 || p->type2.channel != chan_r[3].channel);
                            sum_avg_batt += p->type2.data;
#endif
                            //напряжение
                            do
                            {
                                p--;
                            } while (p->type2.unit != 1 || p->type2.channel != chan_r[1].channel);
                            sum_avg_u += p->type2.data;

                            //ток
                            do
                            {
                                p--;
                            } while (p->type2.unit != 0 || p->type2.channel != chan_r[0].channel);
                            sum_avg_c += p->type2.data;
                        };

                        if (sum_avg_batt / count_avg < adcUbattEnd)
                        {
                            //прекращаем измерения
                            //ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            printf("UbattEnd: %d(%d)\n", voltBatt(sum_avg_batt / count_avg), sum_avg_batt / count_avg);
                            BattLow += 100;
                            break;
                        }
                        else if (sum_avg_batt / count_avg < adcUbattLow)
                        {
                            BattLow += 1;
                            if (BattLow > 10)
                            {
                                //прекращаем измерения
                                printf("UbattLow: %d(%d):%d\n", voltBatt(sum_avg_batt / count_avg), sum_avg_batt / count_avg, BattLow);
                                //ВЫРУБАЕМ
                                gpio_set_level(ENABLE_PIN, 1);
                                break;
                            }
                        }

                        // "если напряжение > лимита" или "ток в зашкале при напряжении > половины лимита"
                        if ((sum_avg_c / count_avg > 4060 && sum_avg_u / count_avg > adcU_limit / 3) || sum_avg_u / count_avg > adcU_limit)
                        {
                            //ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            ptr_off = ptr;
                            time_off = esp_timer_get_time();
                            // printf("off: %d, adcU_limit: %d\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_BLOCK * 4), adcU_limit);
                        };

                        //Переполнен буффер! 
                        if (ptr >= (uint8_t *)&bufferADC[DATALEN] - (ADC_BLOCK * (menu[17].val + 1)))
                            ptr = ptr - ret_num;
                    };
                }
            /*
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
                        } while (p->type2.unit != 0 || p->type2.channel != chan_r[0].channel);
                        sum_avg_c += p->type2.data;
                    };

                    // printf("sum: %d / %d\n", sum_avg_c, count_avg);

                    if ((sum_avg_c / count_avg) < 145)
                    {
                        //переключаем
                        ptr_chan = ptr;
                        triggerChan = -1;
                        adc_select(1);
                    };
                    // int64_t t2 = esp_timer_get_time();
                    //  printf("dt: %lld\n", t2 - t1);
                    triggerChan--;
                };
            };
            */

            // TEST Start 2
            if (repeate_test > 0 && blocks % 50 == 0)
            {
                repeate_test--;
                //включаем источник питания
                ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                t1 = esp_timer_get_time();
                ptr_on = ptr;
                ptr_off = (uint8_t *)bufferADC;
            }
        } while (ptr < (uint8_t *)&bufferADC[DATALEN] - ADC_BLOCK);

        gpio_set_level(POWER_PIN, 0);

        ESP_ERROR_CHECK(adc_digi_stop());
        ESP_ERROR_CHECK(adc_digi_deinitialize());

        processBuffer(ptr, ptr_chan, ptr_off, ptr_on);

        xEventGroupSetBits(ready_event_group, END_MEASURE);

        // printf("Time:\n3 block: %lld\n3 block off: %lld\noff: %lld\n", t2 - t1, t3 - t2, time_off - t1);
    };
}

void processBuffer(uint8_t *endptr, uint8_t *ptr_chan, uint8_t *ptr_off, uint8_t *ptr_on)
{
    char buf[WS_BUF_LINE];

    adc_digi_output_data_t *p = (void *)bufferADC;
    int n = 0;
    int num_p = 0;

    int count_avg = ADC_BLOCK / ADC_COUNT_READ / 4;
    int result_count_avg = menu[17].val; //

    int sum_avg_c = 0;
    int sum_avg_r0 = 0;
    int sum_avg_r1 = 0;
    int sum_avg_u = 0;
    int sum_avg_batt = 0;
    int sum_avg_u0 = 0;
    int sum_n = 0;
    int sum_first_p = 0;
    int sum_adcI0 = 0;
    int sum_adcI1 = 0;
    int sum_adcU = 0;
    int sum_n_r0 = 0;
    int sum_n_r1 = 0;

    int result_sum_avg_r0 = 0;
    int result_sum_avg_r1 = 0;
    int result_sum_avg_u = 0;
    int result_sum_n = 0;

    int batt_min_counter = count_avg;
    int batt_min = 0;

    int block_off = 0;
    int block_on = 0;
    int block_0db = 0;
    int block_chan = 0;
    int block_overload = 0;

    int sum_n_adc = 0;
    int sum_full_adc0 = 0;
    int sum_full_adc1 = 0;
    int sum_full_adc2 = 0;
    int sum_full_adc3 = 0;
    int sum_full_adc4 = 0;
    // int block_measure = 0;

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
    // sprintf(buf, "off %d, adc %d", (ptr_off - (uint8_t *)bufferADC) / (sizeof(chan_r) / sizeof(chan_r[0]) * 4), (ptr_chan - (uint8_t *)bufferADC) / (sizeof(chan_r) / sizeof(chan_r[0]) * 4));
    //  xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
    // printf("%s\n", buf);
    printf("on: %d, off: %d, last_chan: %d\n", (ptr_on - (uint8_t *)bufferADC) / (ADC_COUNT_READ * 4), (ptr_off - (uint8_t *)bufferADC) / (ADC_COUNT_READ * 4), last_chan);

    int adc0 = 0; //Первый канал
    int adc1 = 0; //Напряжение
    int adc2 = 0; //Второй канал
    int adc3 = 0;
    int adc4 = 0;

    while ((uint8_t *)p < endptr)
    {
        while (sum_n < count_avg)
        {
            if ((uint8_t *)p >= endptr)
            {
                // printf("endptr (%d)-------------------------------------\n", num_p);
                break;
            }

            if ((uint8_t *)p >= ptr_on)
            {
                if (block_on >= 0)
                {
                    block_on++;
                    if (block_on == 1)
                    {
                        // printf("block_on (%d)-------------------------------------\n", num_p);
                        break;
                    }
                }
            }

            if ((uint8_t *)p >= ptr_off)
            {
                if (block_off >= 0)
                {
                    block_off++;
                    if (block_off == 1)
                    {
                        // printf("ptr_off (%d)-------------------------------------\n", num_p);
                        break;
                    }
                }
                /*
                if ((uint8_t *)p >= ptr_chan)
                {
                    if (block_0db >= 0)
                    {
                        block_0db++;
                        if (block_0db == 1)
                        {
                            block_off = 1; //для пересчета результата
                            // printf("block_0db (%d)-------------------------------------\n", num_p);
                            break;
                        }
                    }
                }
                */
            }
            if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
                (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel
#if ADC_COUNT_READ == 5
                && (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
                (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
                (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel
#endif
            )
            { /*
              if (block_chan != p->type2.channel)
              {
                  block_chan = p->type2.channel;
                  // printf("block_channel (%d)-------------------------------------\n", num_p);
                  break;
              }
              */

                if (p->type2.data > 4090)
                {
                    block_overload++;
                    if (block_overload == 1)
                    {
                        // printf("block_overload 1 (%d)-------------------------------------\n", num_p);
                        break;
                    }
                }

                if (sum_n == 0)
                {
                    sum_avg_u = 0;
                    sum_avg_c = 0;
                    sum_avg_batt = 0;
                    sum_avg_u0 = 0;
                    sum_first_p = num_p;
                    sum_adcI0 = 0;
                    sum_adcI1 = 0;
                    sum_adcU = 0;

                    sum_avg_r0 = 0;
                    sum_n_r0 = 0;
                    sum_avg_r1 = 0;
                    sum_n_r1 = 0;
                }

                adc0 = p->type2.data;
                adc1 = (p + 1)->type2.data;
#if ADC_COUNT_READ == 5
                adc2 = (p + 2)->type2.data;
                adc3 = (p + 3)->type2.data;
                adc4 = (p + 4)->type2.data;
#endif
                sum_n_adc++;
                sum_full_adc0 += adc0;
                sum_full_adc1 += adc1;
                sum_full_adc2 += adc2;
                sum_full_adc3 += adc3;
                sum_full_adc4 += adc4;

                sum_n++;
                sum_adcI0 += adc0;
                sum_adcI1 += adc2;
                sum_adcU += adc1;
                sum_avg_u += volt(adc1);
                sum_avg_batt += voltBatt(adc3);
                sum_avg_u0 += volt0(adc4);

                //Канал 0
                int r = kOm(adc1, adc0);
                if (r != 0 && r != chan_r[0].max)
                {
                    sum_avg_r0 += r;
                    sum_n_r0++;
                }
                bufferR[0][num_p] = r;
                // sum_avg_c += current(adc1);

                //Канал 1
                r = kOm0db(adc1, adc2);
                if (r != 0 && r != chan_r[1].max)
                {
                    sum_avg_r1 += r;
                    sum_n_r1++;
                }
                bufferR[1][num_p] = kOm0db(adc1, adc2);
                // sum_avg_c += current0(adc1);

                if ((uint8_t *)p > ptr_on && batt_min_counter > 0)
                {
                    if (voltBatt(adc3) < batt_min)
                        batt_min = voltBatt(adc3);

                    // printf("(%d) batt_min: %d\n", num_p, voltBatt(adc3));

                    batt_min_counter--;
                }
            }
            else
            {
                printf("skip %d-%d %d-%d %d-%d %d-%d %d-%d\n", p->type2.unit, p->type2.channel,
                       (p + 1)->type2.unit, (p + 1)->type2.channel, (p + 2)->type2.unit, (p + 2)->type2.channel,
                       (p + 3)->type2.unit, (p + 3)->type2.channel, (p + 4)->type2.unit, (p + 4)->type2.channel);
                // fflush(stdout);
                p++;
                sum_n = 0;
                continue;
            };

            num_p++;
            p = p + ADC_COUNT_READ; // sizeof(chan_r) / sizeof(chan_r[0]);
        }                           // end while

        if (sum_n > 0)
        {
            if (batt_min == 0)
            {
                batt_min = sum_avg_batt / sum_n;

                if (batt_min > 10000) // reset BattLow
                    BattLow = 0;
            }

            if (sum_n_r0 == 0)
                sum_n_r0 = 1;
            if (sum_n_r1 == 0)
                sum_n_r1 = 1;

            result.adc0 = sum_adcI0 / sum_n;
            result.adc2 = sum_adcI1 / sum_n;
            result.adc1 = sum_adcU / sum_n;

            // if (block_0db >= count_avg) //первый блок после ptr_0db
            //{
            //  result.adc2 = adc2;
            //  result.U = sum_avg_u / sum_n / 1000;
            //  result.U0 = sum_avg_u0 / sum_n / 1000;
            // result.adc1 = adc1;
            // result.R = sum_avg_c / sum_n;
            // block_0db = -1;
            //}

            // sprintf(buf, "%5d(%2d)(%2d;%2d), %4d(%d), %5d, %4d, %5d, %4d, %5d, %4d, %5d", sum_first_p, sum_n, block_off, block_0db, adc1, adc1ch, sum_avg_c / sum_n,
            //         adc2, sum_avg_u / sum_n / 1000, adc3, sum_avg_batt / sum_n, adc4, sum_avg_u0 / sum_n);
            //            f_id cnt  adc0:R0  adc1:U     adc2:R1  adc3:Ubat adc4:U0
            sprintf(buf, "%5d(%2d), %4d:%5d, %4d:%5.2f, %4d:%5d, %4d:%5.3f, %4d:%5.2f, (%d)", sum_first_p, sum_n, sum_adcI0 / sum_n, sum_avg_r0 / sum_n_r0, sum_adcU / sum_n, (float)sum_avg_u / sum_n / 1000,
                    sum_adcI1 / sum_n, sum_avg_r1 / sum_n_r1, adc3, (float)sum_avg_batt / sum_n / 1000, adc4, (float)sum_avg_u0 / sum_n / 1000, block_off);
            xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
            printf("%s\n", buf);

            if (block_off > 0)
            {
                if (result_sum_n == 0)
                {
#if ADC_COUNT_READ > 2
                    //Выбираем канал измерения
                    if (sum_adcI0 / sum_n < (75000 - menu[5].val) * 10 / menu[4].val) // < 75мкА >6,6MOm
                    {
                        last_chan = 1;
                    }

                    if (sum_adcI1 / sum_n > (250000 - menu[7].val) * 10 / menu[6].val) // > 250мкА <2MOm
                    {
                        last_chan = 0;
                    }
#endif
                    if (last_chan == 1)
                    {
                        result.R = sum_avg_r1 / sum_n_r1;
                    }
                    else
                    {
                        result.R = sum_avg_r0 / sum_n_r0;
                    }

                    result.U = sum_avg_u / sum_n / 1000;
                    result.U0 = sum_avg_u0 / sum_n / 1000;
                    result.Ubatt1 = batt_min;
                    result.Ubatt0 = sum_avg_batt / sum_n;

                    result_sum_avg_r0 = 0;
                    result_sum_avg_r1 = 0;
                }

                if (result_sum_n < result_count_avg && sum_avg_u / sum_n / 1000 > 6)
                {
                    result_sum_n++;
                    result_sum_avg_r0 += sum_avg_r0 / sum_n_r0;
                    result_sum_avg_r1 += sum_avg_r1 / sum_n_r1;

                    if (last_chan == 1)
                        result.R = result_sum_avg_r1 / result_sum_n;
                    else
                        result.R = result_sum_avg_r0 / result_sum_n;

                    if (result_sum_n == result_count_avg)
                    {
                        block_off = -1;
                    }

                    // printf("%d result: %d---------------------------------\n", result_sum_n, result.R);
                }
            }

            //перезапускаем
            if (block_off < count_avg * 2) //начиная со второго блока после отключения
            {
                result_sum_n = 0;
            }

            //перезапускаем результат
            if (block_overload > 0 ||            //если перегрузка по току
                ptr_off == (uint8_t *)bufferADC) //последний, если отключения небыло
            {
                result_sum_n = 0;
                block_overload = 0;
            }

            sum_n = 0;
        }
    };

    xQueueSend(send_queue, (void *)&result, (portTickType)0);

    printf("adc0: %d, adc1: %d, adc2: %d, adc3: %d, adc4: %d\n", sum_full_adc0 / sum_n_adc, sum_full_adc1 / sum_n_adc, sum_full_adc2 / sum_n_adc, sum_full_adc3 / sum_n_adc, sum_full_adc4 / sum_n_adc);

    return;
}

int getADC_Data(char *line, uint8_t **ptr_adc, int *num)
{
    adc_digi_output_data_t *p = (void *)(*ptr_adc);

    if ((*ptr_adc) == 0)
    {
        p = (void *)bufferADC;
        (*ptr_adc) = (uint8_t *)p;
#if ADC_COUNT_READ == 2
        strcpy(line, "id, adc0, adc1, adc2, res0, res1\n");
#elif ADC_COUNT_READ == 5
        strcpy(line, "id, adc0, adc1, adc2, adc3, adc4, res0, res1\n");
#endif
        return 2;
    };

    while ((uint8_t *)(p + ADC_COUNT_READ) < (uint8_t *)bufferADC + sizeof(bufferADC))
    {
#if ADC_COUNT_READ == 2
        if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
            (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel)
        {
            sprintf(line, "%5d, %4d, %4d, %4d, %5d, %5d\n", (*num), p->type2.data, (p + 1)->type2.data, 0, bufferR[0][(*num)], bufferR[1][(*num)]);
#elif ADC_COUNT_READ == 5
        if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
            (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel &&
            (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
            (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
            (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel)
        {
            //             id   adc0 adc1 adc2 adc3 adc4
            sprintf(line, "%5d, %4d, %4d, %4d, %4d, %4d, %5d, %5d\n", (*num), p->type2.data, (p + 1)->type2.data, (p + 2)->type2.data, (p + 3)->type2.data, (p + 4)->type2.data, bufferR[0][(*num)], bufferR[1][(*num)]);
#endif
            (*num)++;
            // printf("%s", line);
        }
        else
        {
            p++;
            continue;
        };

        p = p + ADC_COUNT_READ;
        (*ptr_adc) = (uint8_t *)p;
        return 1;
    };

    return 0;
};
