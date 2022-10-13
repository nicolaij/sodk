#include "main.h"
#include <stdio.h>
#include <string.h>
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_partition.h"

#include "driver/adc.h"

//#include "esp_adc_cal.h"
#include "esp_rom_sys.h"

#include "soc/apb_saradc_reg.h"

#include "esp_wifi.h"

// RTC_DATA_ATTR int last_chan;

uint32_t bufferADC[DATALEN];
uint32_t bufferR[2][DATALEN / ADC_COUNT_READ];
uint32_t bufferRb[6][DATALEN * 4 / ADC_BUFFER];

typedef struct
{
    adc1_channel_t channel;
    adc_atten_t k;
    int max;
} measure_t;

//#define SWAP_ADC1_0 1

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

// extern menu_t menu[];

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

static void continuous_adc_init()
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);
    int64_t t1 = esp_timer_get_time();
    int64_t t2 = 0;
    int64_t t3 = 0;

    // esp_err_t adc_digi_filter_set_config(adc_digi_filter_idx_tidx, adc_digi_filter_t *config);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = ADC_BUFFER * 2,
        .conv_num_each_intr = ADC_BUFFER,
        .adc1_chan_mask = (1 << chan_r[0].channel) | (1 << chan_r[2].channel) | (1 << chan_r[3].channel) | (1 << chan_r[4].channel),
        .adc2_chan_mask = 1 << chan_r[1].channel,
    };

    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    t2 = esp_timer_get_time();

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = false,
        .conv_limit_num = 200,
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

int terminal_mode = -1;
void btn_task(void *arg)
{
    /*
    gpio_config_t io_input_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE, // disable interrupt
        .mode = GPIO_MODE_INPUT,            // input mode
        .pin_bit_mask = (1 << BTN_PIN),     // bit mask of the input pins
        .pull_down_en = 0,                  // disable pull-down mode
        .pull_up_en = 0,                    // disable pull-down mode
    };
    gpio_config(&io_input_conf);
*/
    gpio_reset_pin(BTN_PIN);
    gpio_set_direction(BTN_PIN, GPIO_MODE_INPUT);

    int old_btn_state = gpio_get_level(BTN_PIN);
    int new_btn_state = old_btn_state;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        new_btn_state = gpio_get_level(BTN_PIN);

        if (old_btn_state != new_btn_state)
        {
            old_btn_state = new_btn_state;
            if (new_btn_state == 0)
            {
                // ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));
                // pcf8575_set(1 | (1 << NB_PWR_BIT));
                // ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
            }
            else
            {
                // ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                // pcf8575_set((1 << NB_PWR_BIT));
                // ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
                start_measure();
            }
            reset_sleep_timeout();
        }

        //работа с терминалом
        int ch = EOF;
        int cmd = EOF;
        do
        {
            cmd = ch;
            ch = fgetc(stdin);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } while (ch != EOF);

        if (cmd != EOF)
        {
            // ESP_LOGI("terminal", "read: %x", cmd);
            if (cmd == '\n')
            {
                if (terminal_mode == -1)
                {
                    terminal_mode = 0;
                    ESP_LOGI("terminal", "Вывод в терминал включен");
                }
                else
                {
                    terminal_mode = -1;
                    ESP_LOGW("terminal", "Вывод в терминал отключен");
                }
            }

            if (cmd == 'w' || cmd == 'W') // stop wifi
            {
                esp_wifi_stop();
            }

            if (cmd == 'r' || cmd == 'r') // print block result
            {
                for (int i = 0; i < DATALEN * 4 / ADC_BUFFER; i++)
                {
                    printf("%3d(%2d:%3d,%3d) %4d %4d %4d\n", i, bufferRb[3][i], bufferRb[4][i], bufferRb[5][i], bufferRb[2][i], bufferRb[0][i], bufferRb[1][i]);
                }
            }

            if (cmd == 'p' || cmd == 'P') // print errors in ADC buffer
            {
                adc_digi_output_data_t *p = (void *)bufferADC;
                int n = 0;
                int cnt = 0;

                while ((uint8_t *)p < (uint8_t *)&bufferADC[DATALEN])
                {
                    n = ((uint8_t *)p - (uint8_t *)bufferADC) / sizeof(adc_digi_output_data_t);

                    if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
                        (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel &&
                        (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
                        (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
                        (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel)
                    {
                        if (cnt > 0)
                        {
                            for (int i = (-5 - cnt); i < 5; i++)
                            {
                                printf("%4d %d:%d:%4d\n", n + i, (p + i)->type2.unit, (p + i)->type2.channel, (p + i)->type2.data);
                            }
                            printf("\n");
                        };
                        p = p + 5;
                        cnt = 0;
                    }
                    else
                    {
                        p++;
                        cnt++;
                    }
                }
            }
        }
    }
}

void dual_adc(void *arg)
{

    // bool cali_enable = adc_calibration_init();

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    cmd_t cmd;
    cmd.cmd = 0;
    cmd.channel = 0;

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
    io_conf.pin_bit_mask = (1 << LED_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));

    continuous_adc_init();

    while (1)
    {
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        //подаем питание на источник питания, OpAmp, делитель АЦП батареи
        //включаем канал
        pcf8575_set((1 << (cmd.channel - 1)) | (1 << NB_PWR_BIT));
        ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));

        uint8_t *ptr = (uint8_t *)bufferADC;
        uint8_t *ptr1 = (uint8_t *)bufferADC;
        uint8_t *ptr_on = (uint8_t *)bufferADC;   //включаем источник питания
        uint8_t *ptr_off = (uint8_t *)bufferADC;  //выключаем источник питания
        uint8_t *ptr_chan = (uint8_t *)bufferADC; //переключаем канал
        int blocks = 0;

        int64_t timeout = menu[0].val * 1000;
        // int64_t quiet_timeout = 10 * 1000;

        //(Лимит напряж - смещение) * коэфф. U
        int32_t adcU_limit = (menu[1].val * 1000 - menu[3].val) * 10 / menu[2].val;
        // printf("adcU_limit = %d\n", adcU_limit);

        //((adc * menu[8].val) + menu[9].val) / 1000;
        int32_t adcUbattLow = (menu[12].val * 1000 - menu[9].val) / menu[8].val;
        if (menu[12].val <= 0)
            adcUbattLow = INT32_MIN;
        int32_t adcUbattEnd = (menu[13].val * 1000 - menu[9].val) / menu[8].val;
        if (menu[13].val <= 0)
            adcUbattEnd = INT32_MIN;

        // int64_t time_off = 0;
        // adc_select(1);

        ESP_ERROR_CHECK(adc_digi_start());

        int64_t t1 = esp_timer_get_time();
        // int64_t t2 = 0, t3 = 0, time_off = 0;

        int repeate_test = menu[16].val;

        // adc_digi_output_data_t *p =

        do
        {
            uint32_t req = ADC_BUFFER;
            uint32_t ret = 0;
            int n1 = 0, n2 = 0;
            while (req > 0)
            {
                ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, req, &ret, ADC_MAX_DELAY));
                ptr = ptr + ret;
                req = req - ret;
                if (n1 == 0)
                    n1 = ret;
                else
                    n2 = ret;
            }

            // printf("%4d\n",blocks);
            // if (ret_num != ADC_BUFFER) printf("%3d: %4d\n",blocks, ret_num);

            int sum_avg_c = 0;
            int sum_avg_u = 0;
            int sum_avg_u_r = 0;
            int sum_avg_c2 = 0;
            int sum_avg_batt = 0;
            int sum_avg_r1 = 0;
            int sum_avg_r2 = 0;
            int n = 0;
            adc_digi_output_data_t *p = (void *)ptr1;
            while ((p + 4) < (adc_digi_output_data_t *)ptr)
            {
                if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
                    (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel &&
                    (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
                    (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
                    (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel)
                {
                    n++;
                    sum_avg_batt += (p + 3)->type2.data;
                    sum_avg_c2 += (p + 2)->type2.data;
                    sum_avg_u += (p + 1)->type2.data;
                    sum_avg_c += p->type2.data;

                    sum_avg_r1 += kOm((p + 1)->type2.data, p->type2.data);
                    sum_avg_r2 += kOm0db((p + 1)->type2.data, (p + 2)->type2.data);
                    sum_avg_u_r += volt((p + 1)->type2.data);
                    p = p + ADC_COUNT_READ;
                    ptr1 = (void *)p;
                }
                else
                {
                    //сбой повтора канала
                    if (p->type2.unit == (p + 1)->type2.unit && p->type2.channel == (p + 1)->type2.channel)
                    {
                        (p + 1)->val = p->val;
                    }
                    p++;
                }
            };

            if (n == 0)
            {
                ESP_LOGE("adc", "Error! n==0, block:%d\n", blocks);
                continue;
            }

            bufferRb[0][blocks] = sum_avg_r1 / n;
            bufferRb[1][blocks] = sum_avg_r2 / n;
            bufferRb[2][blocks] = sum_avg_u_r / n / 1000;
            bufferRb[3][blocks] = n;
            bufferRb[4][blocks] = n1;
            bufferRb[5][blocks] = n2;

            if (blocks == 9) //включаем источник ВВ
            {
                if (BattLow > 10)
                    break;

                // adc_select(0);
                if ((esp_timer_get_time() - t1) < timeout)
                {
                    //включаем источник питания
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                    // printf("on\n");
                    ptr_on = ptr;
                }
            }

            if (blocks >= 9)
            {
                if (ptr_off == (uint8_t *)bufferADC)
                {
                    //отсечка по времени
                    if ((esp_timer_get_time() - t1) > timeout)
                    {
                        //ВЫРУБАЕМ
                        gpio_set_level(ENABLE_PIN, 1);
                        // printf("off 1\n");
                        ptr_off = ptr;
                        // time_off = esp_timer_get_time();
                        // printf("off count: %d\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_DMA * 4));
                    }
                    else
                    {
                        //отсечка по току, напряжению, Ubatt
                        if (sum_avg_batt / n < adcUbattEnd)
                        {
                            //прекращаем измерения
                            //ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            // printf("off 2\n");
                            if (terminal_mode >= 0)
                                printf("UbattEnd: %d(%d)\n", voltBatt(sum_avg_batt / n), sum_avg_batt / n);
                            BattLow += 100;
                            break;
                        }
                        else if (sum_avg_batt / n < adcUbattLow)
                        {
                            BattLow += 1;
                            if (BattLow > 10)
                            {
                                //ВЫРУБАЕМ
                                gpio_set_level(ENABLE_PIN, 1);
                                // printf("off 3\n");
                                //прекращаем измерения
                                if (terminal_mode >= 0)
                                    printf("UbattLow: %d(%d):%d\n", voltBatt(sum_avg_batt / n), sum_avg_batt / n, BattLow);
                                break;
                            }
                        }

                        // "если напряжение > лимита" или "ток в зашкале при напряжении > половины лимита"
                        if ((sum_avg_c / n > 4060 && sum_avg_u / n > adcU_limit / 3) || sum_avg_u / n > adcU_limit)
                        {
                            //ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            // printf("off 4\n");
                            ptr_off = ptr;
                            // time_off = esp_timer_get_time();
                            //  printf("off: %d, adcU_limit: %d\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_BLOCK * 4), adcU_limit);
                        };

                        //Оставляем свободное место в буфере для вычисления сопротивления после отключения ВВ источника
                        if (ptr >= (uint8_t *)bufferADC + sizeof(bufferADC) - (ADC_BUFFER * (menu[17].val + 1)))
                        {
                            ptr = ptr - ADC_BUFFER;
                            blocks = sizeof(bufferADC) / ADC_BUFFER - (menu[17].val + 1);
                        }
                    };
                }
            }

            // TEST Start 2
            if (repeate_test > 0 && blocks % 50 == 0)
            {
                repeate_test--;
                //включаем источник питания
                ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                // printf("on r\n");
                t1 = esp_timer_get_time();
                ptr_on = ptr;
                ptr_off = (uint8_t *)bufferADC;
            }

            blocks++;
        } while (ptr < (uint8_t *)bufferADC + sizeof(bufferADC) - ADC_BUFFER);

        ESP_ERROR_CHECK(adc_digi_stop());

        //ВЫРУБАЕМ (на всякий случай)
        gpio_set_level(ENABLE_PIN, 1);

        //выключаем питание, если больше нет каналов в очереди
        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
        {
            pcf8575_set((1 << POWER_BIT) | (1 << NB_PWR_BIT));
            gpio_set_level(LED_PIN, 0);
        }

        // ESP_ERROR_CHECK(adc_digi_deinitialize());

        // printf("Total blocks: %d\n", blocks);
        // for (int i = 0; i < blocks; i++)
        //{
        //     printf("%3d %4d %4d %4d\n", i, bufferRb[2][i], bufferRb[0][i], bufferRb[1][i]);
        // }

        processBuffer(ptr, ptr_chan, ptr_off, ptr_on, cmd.channel);

        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
            xEventGroupSetBits(ready_event_group, END_MEASURE);

        // printf("Time:\n3 block: %lld\n3 block off: %lld\noff: %lld\n", t2 - t1, t3 - t2, time_off - t1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    };
}

void processBuffer(uint8_t *endptr, uint8_t *ptr_chan, uint8_t *ptr_off, uint8_t *ptr_on, int channel)
{
    char buf[WS_BUF_SIZE];

    adc_digi_output_data_t *p = (void *)bufferADC;
    int n = 0;
    int num_p = 0;

    int count_avg = ADC_BUFFER / ADC_COUNT_READ / 4;
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

    int last_chan = 0;

    int skip_data_count = 0;

    //результат измерений
    result_t result = {.channel = channel};

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
    if (terminal_mode >= 0)
        printf("on: %d, off: %d\n", (ptr_on - (uint8_t *)bufferADC) / (ADC_COUNT_READ * 4), (ptr_off - (uint8_t *)bufferADC) / (ADC_COUNT_READ * 4));

    int adc0 = 0; //Первый канал
    int adc1 = 0; //Напряжение
    int adc2 = 0; //Второй канал
    int adc3 = 0;
    int adc4 = 0;

    int pre_data0 = 0;

    //пропуск первых 50 значений для расчета среднего АЦП
    int skip_full_summ = 50;

    while ((uint8_t *)p < endptr)
    {
        while (sum_n < count_avg)
        {
            if ((uint8_t *)p >= endptr)
            {
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
            }

            if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
                (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel && (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
                (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
                (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel)
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
                        pre_data0 = p->type2.data;
                        break;
                    }
                }
                else
                {
                    if (pre_data0 > 4090)
                    {
                        pre_data0 = p->type2.data;
                        break;
                    }
                }

                pre_data0 = p->type2.data;

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
                if (skip_full_summ-- <= 0)
                {
                    sum_n_adc++;
                    sum_full_adc0 += adc0;
                    sum_full_adc1 += adc1;
                    sum_full_adc2 += adc2;
                    sum_full_adc3 += adc3;
                    sum_full_adc4 += adc4;
                }

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
                skip_data_count++;
                if (terminal_mode >= 0)
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
            snprintf(buf, sizeof(buf), "%5d(%2d), %4d:%5d, %4d:%5.2f, %4d:%5d, %4d:%5.3f, %4d:%5.2f, (%d)", sum_first_p, sum_n, sum_adcI0 / sum_n, sum_avg_r0 / sum_n_r0, sum_adcU / sum_n, (float)sum_avg_u / sum_n / 1000,
                     sum_adcI1 / sum_n, sum_avg_r1 / sum_n_r1, adc3, (float)sum_avg_batt / sum_n / 1000, adc4, (float)sum_avg_u0 / sum_n / 1000, block_off);
            // xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
            if (terminal_mode >= 0)
                printf("%s\n", buf);

            if (block_off > 0)
            {
                if (result_sum_n == 0)
                {
#if ADC_COUNT_READ > 2
                    //Выбираем канал измерения
                    if (sum_adcI0 / sum_n < (110000 - menu[5].val) * 10 / menu[4].val) // < 110мкА > 4.5MOm
                    {
                        last_chan = 1;
                        // printf("select chan 1: %d < %d\n", sum_adcI0 / sum_n, (50000 - menu[5].val) * 10 / menu[4].val);
                    }
#endif
                    if (last_chan == 1)
                    {
                        result.R = sum_avg_r1 / sum_n_r1;
                        // ESP_LOGI("processBuffer", "Chan: %d Result R:%d", last_chan, result.R);
                    }
                    else
                    {
                        result.R = sum_avg_r0 / sum_n_r0;
                        // ESP_LOGI("processBuffer", "Chan: %d Result R:%d", last_chan, result.R);
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

    printf("adc0: %d, adc1: %d, adc2: %d, adc3: %d, adc4: %d, skip error: %d\n", sum_full_adc0 / sum_n_adc, sum_full_adc1 / sum_n_adc, sum_full_adc2 / sum_n_adc, sum_full_adc3 / sum_n_adc, sum_full_adc4 / sum_n_adc, skip_data_count);

    snprintf((char *)buf, sizeof(buf), "\"channel\":%d,\"U\":%d,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%d", result.channel, result.U, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0);
    xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
    printf("Result: %s\n", buf);

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
