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

// uint32_t bufferR[2][DATALEN / ADC_COUNT_READ];
// uint32_t bufferR[6][DATALEN];
calcdata_t bufferR[DATALEN];
uint32_t buffer[3][DATALEN];
uint8_t buffer_ADC[ADC_BUFFER + sizeof(adc_digi_output_data_t) * 4];
uint8_t buffer_ADC_copy[ADC_BUFFER * 5];

#define ON_BLOCK 10

typedef struct
{
    adc1_channel_t channel;
    adc_atten_t k;
    int max;
} measure_t;

#ifdef SWAP_ADC1_0
const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_0, .max = 6999},  //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC2_CHANNEL_0, .max = 1000},  //Напряжение источника питания
    {.channel = ADC1_CHANNEL_4, .max = 99999}, //
    {.channel = ADC1_CHANNEL_1, .max = 10000}, //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .max = 1000},  //Напряжение 0 проводника
};
#else
const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_1, .max = 6999},  //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC2_CHANNEL_0, .max = 1000},  //Напряжение источника питания
    {.channel = ADC1_CHANNEL_4, .max = 99999}, //~х22
    {.channel = ADC1_CHANNEL_0, .max = 10000}, //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .max = 1000},  //Напряжение 0 проводника
};
#endif

const uint16_t pcf_output[5] = {0, BIT(3), BIT(2), BIT(1), BIT(0)};

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

int kOm2chan(int adc_u, int adc_r)
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
        .max_store_buf_size = ADC_BUFFER * 400,
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

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR pcf_int_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

int terminal_mode = -1;
void btn_task(void *arg)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BIT64(BTN_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

#if MULTICHAN
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = BIT64(INT_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));

    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(INT_PIN, pcf_int_handler, (void *)INT_PIN);
#endif

    int old_btn_state = gpio_get_level(BTN_PIN);
    int new_btn_state = old_btn_state;

    while (1)
    {
#if MULTICHAN
        uint32_t io_num;
        if (xQueueReceive(gpio_evt_queue, &io_num, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            char buf[10];
            int r = pcf8575_read(IN1_BIT);
            snprintf((char *)buf, sizeof(buf), "IN: %d", r);
            ESP_LOGI("pcf8575", "%s", buf);
            xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
        }
#else
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

        new_btn_state = gpio_get_level(BTN_PIN);
        if (old_btn_state != new_btn_state)
        {
            old_btn_state = new_btn_state;
            if (new_btn_state == 1)
                start_measure(1);

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

            if (cmd == 'r' || cmd == 'R') // print block result
            {
                int n = 0;
                for (int i = 0; i < DATALEN; i++)
                {
                    if (buffer[2][i] != 10)
                        printf("%4d %4d %4d %4d %4d\n", ++n, i, buffer[0][i], buffer[1][i], buffer[2][i]);
                }
            }

            if (cmd == 'p' || cmd == 'P') // print ADC buffer
            {
                adc_digi_output_data_t *p = (void *)buffer_ADC_copy;
                int n = 0;
                int cnt = 0;

                while ((uint8_t *)p < (uint8_t *)buffer_ADC_copy + sizeof(buffer_ADC_copy))
                {
                    n = ((uint8_t *)p - (uint8_t *)buffer_ADC_copy) / sizeof(adc_digi_output_data_t);

                    if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
                        (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel &&
                        (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
                        (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
                        (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel)
                    {
                        printf("%4d %4d %4d %4d %4d %4d\t", n, (p + 0)->type2.data, (p + 1)->type2.data, (p + 2)->type2.data, (p + 3)->type2.data, (p + 4)->type2.data);
                        printf("=%6d %6d %6d\n", kOm((p + 1)->type2.data, (p + 0)->type2.data), volt((p + 1)->type2.data), kOm2chan((p + 1)->type2.data, (p + 2)->type2.data));

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

    result_t result = {};

    cmd_t cmd = {};

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT64(ENABLE_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));

#ifndef NODEBUG
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT64(LED_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
#endif

    continuous_adc_init();

    while (1)
    {
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        //подаем питание на источник питания, OpAmp, делитель АЦП батареи
        //включаем канал

#ifdef MULTICHAN
        result.channel = cmd.channel;

        //для первого канала читаем вход
        if (cmd.cmd == 2) //сработал внешний вход
        {
            result.input = 2;
        }
        else
        {
            result.input = pcf8575_read(IN1_BIT);
        }

        pcf8575_set(pcf_output[cmd.channel] | BIT(NB_PWR_BIT));
#endif
#ifndef NODEBUG
        ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));
#endif
        uint8_t *ptr = (uint8_t *)buffer_ADC;
        uint8_t *ptr1 = (uint8_t *)buffer_ADC;
        uint8_t *ptre = (uint8_t *)buffer_ADC;

        int blocks = 0;
        int block_power_on = 0;
        int block_power_off = 0;
        int start_new_avg = 0;
        int end_new_avg = 0;
        int block_result = 0;
        int block_pre_result = 0;
        int next_block_result = 0;

        int u_max = 0;

        int overcurrent_counter = 50;

        int64_t timeout = menu[0].val * 1000LL;
        // int64_t quiet_timeout = 10 * 1000;

        //(Лимит напряж - смещение) * коэфф. U
        // int32_t adcU_limit = (menu[1].val * 1000 - menu[3].val) * 10 / menu[2].val;
        // printf("adcU_limit = %d\n", adcU_limit);

        //((adc * menu[8].val) + menu[9].val) / 1000;
        // int32_t adcUbattLow = (menu[12].val * 1000 - menu[9].val) / menu[8].val;
        // if (menu[12].val <= 0)
        //    adcUbattLow = INT32_MIN;
        // int32_t adcUbattEnd = (menu[13].val * 1000 - menu[9].val) / menu[8].val;
        // if (menu[13].val <= 0)
        //    adcUbattEnd = INT32_MIN;

        // int64_t time_off = 0;
        // adc_select(1);

        ESP_ERROR_CHECK(adc_digi_start());

        int64_t t1 = esp_timer_get_time();
        // int64_t t2 = 0, t3 = 0, time_off = 0;

        calcdata_t sum_bavg = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};
        calcdata_t bref = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        const int avg_len = menu[17].val;
        int compare_counter = menu[16].val;
        const int compare_delta = 1; //в %

        int data_errors = 0;

        // adc_digi_output_data_t *p =

        do
        {
            uint32_t req = ADC_BUFFER;
            uint32_t ret = 0;
            int i = 0;
            while (req > 0)
            {
                ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, req, &ret, ADC_MAX_DELAY));
                ptr = ptr + ret;
                req = req - ret;
                if (i == 0)
                    buffer[0][blocks] = ret;
                i++;
            }
            ptre = ptr;

            if (blocks == ON_BLOCK) //включаем источник ВВ
            {
                if (BattLow > 10)
                    break;

                // adc_select(0);
                if ((esp_timer_get_time() - t1) < timeout)
                {
                    //включаем источник питания
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                    // printf("on\n");
                    block_power_on = blocks;
                }
            }

            int sum_adc[5] = {0, 0, 0, 0, 0};

            //расчет среднего каждую 1 мс
            calcdata_t sum_avg01 = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

            int n = 0;
            adc_digi_output_data_t *p = (void *)buffer_ADC;
            while ((uint8_t *)p < ptr - sizeof(adc_digi_output_data_t) * 4)
            {
                if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel &&
                    (p + 1)->type2.unit == 1 && (p + 1)->type2.channel == chan_r[1].channel &&
                    (p + 2)->type2.unit == 0 && (p + 2)->type2.channel == chan_r[2].channel &&
                    (p + 3)->type2.unit == 0 && (p + 3)->type2.channel == chan_r[3].channel &&
                    (p + 4)->type2.unit == 0 && (p + 4)->type2.channel == chan_r[4].channel)
                {
                    n++;
                    sum_adc[3] += (p + 3)->type2.data;
                    sum_adc[2] += (p + 2)->type2.data;
                    sum_adc[1] += (p + 1)->type2.data;
                    sum_adc[0] += p->type2.data;

                    sum_avg01.R1 += kOm((p + 1)->type2.data, p->type2.data);
                    sum_avg01.R2 += kOm2chan((p + 1)->type2.data, (p + 2)->type2.data);
                    sum_avg01.U += volt((p + 1)->type2.data);
                    sum_avg01.Ubatt += voltBatt((p + 3)->type2.data);
                    sum_avg01.U0 += volt0((p + 4)->type2.data);
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
                    data_errors++;
                }
            };

            buffer[1][blocks] = (ptr - ptr1);
            buffer[2][blocks] = n;

            //обрывок данных
            size_t d = (ptr - ptr1) % (ADC_COUNT_READ * sizeof(adc_digi_output_data_t));
            if (d > 0)
            {
                memcpy(buffer_ADC, ptr - d, d);
                ptr = buffer_ADC + d;
            }
            else
            {
                ptr = buffer_ADC;
            }

            if (n == 0)
            {
                ESP_LOGE("adc", "%4d %d\n", blocks, n);
                continue;
            }

            if (n != 10)
            {
                // ESP_LOGE("adc", "%4d %d\n", blocks, n);
            }

            bufferR[blocks].U = sum_avg01.U / n / 1000;
            bufferR[blocks].R1 = sum_avg01.R1 / n;
            bufferR[blocks].R2 = sum_avg01.R2 / n;
            bufferR[blocks].Ubatt = sum_avg01.Ubatt / n;
            bufferR[blocks].U0 = sum_avg01.U0 / n / 1000;

            if (blocks > ON_BLOCK && block_power_off == 0)
            {
                //отсечка по времени
                if ((esp_timer_get_time() - t1) > timeout)
                {
                    //ВЫРУБАЕМr
                    gpio_set_level(ENABLE_PIN, 1);
                    // printf("off 1\n");
                    block_power_off = blocks;
                    // time_off = esp_timer_get_time();
                    // printf("off count: %d\n", (ptr_off - (uint8_t *)bufferADC) / (ADC_DMA * 4));
                }
                else
                {
                    //отсечка по напряжению, Ubatt
                    if (bufferR[blocks].Ubatt < menu[13].val)
                    {
                        //прекращаем измерения
                        //ВЫРУБАЕМ
                        gpio_set_level(ENABLE_PIN, 1);
                        // printf("off 2\n");
                        if (terminal_mode >= 0)
                            printf("UbattEnd: %d < %d)\n", bufferR[blocks].Ubatt, menu[13].val);
                        BattLow += 100;
                        break;
                    }
                    else if (bufferR[blocks].Ubatt < menu[12].val)
                    {
                        BattLow += 1;
                        if (BattLow > 10)
                        {
                            //ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            // printf("off 3\n");
                            //прекращаем измерения
                            if (terminal_mode >= 0)
                                printf("UbattLow: %d < %d (%d)\n", bufferR[blocks].Ubatt, menu[12].val, BattLow);
                            break;
                        }
                    }

                    //если напряжение >

                    // "если напряжение > лимита" или "ток в зашкале при напряжении > ???"
                    // if ((sum_avg_c / n > 4060 && sum_avg_u / n > adcU_limit / 3) || sum_avg_u / n > adcU_limit)
                    /*
                                        if (sum_avg_c1 / n > 4060)
                                        {
                                            overcurrent_counter--;
                                        }
                                        else
                                        {
                                            overcurrent_counter++;
                                        }

                                        // "если напряжение > лимита" или перегруз 50мс
                                        if (bufferR[0][blocks] > menu[1].val || overcurrent_counter == 0)
                                        {
                                            //ВЫРУБАЕМ
                                            gpio_set_level(ENABLE_PIN, 1);
                                            block_power_off = blocks;
                                        }
                                        */
                }
            }

            //считаем среднее
            if (blocks > (ON_BLOCK + 5))
            {
                sum_bavg.R1 += bufferR[blocks].R1;
                sum_bavg.R2 += bufferR[blocks].R2;
                sum_bavg.U += bufferR[blocks].U;
                sum_bavg.U0 += bufferR[blocks].U0;
                if (blocks > ((ON_BLOCK + 5) + avg_len))
                {
                    sum_bavg.R1 -= bufferR[blocks - avg_len].R1;
                    sum_bavg.R2 -= bufferR[blocks - avg_len].R2;
                    sum_bavg.U -= bufferR[blocks - avg_len].U;
                    sum_bavg.U0 -= bufferR[blocks - avg_len].U0;

                    int r1 = sum_bavg.R1 / avg_len;
                    int r2 = sum_bavg.R2 / avg_len;
                    int u = sum_bavg.U / avg_len;
                    int u0 = sum_bavg.U0 / avg_len;

                    int cmp_r1_max = r1 * 1000 / (1000 - compare_delta * 10);
                    int cmp_r1_min = r1 * 1000 / (1000 + compare_delta * 10);
                    if (cmp_r1_max == r1)
                        cmp_r1_max = cmp_r1_max + 1;

                    int cmp_r2_max = r2 * 1000 / (1000 - compare_delta * 10);
                    int cmp_r2_min = r2 * 1000 / (1000 + compare_delta * 10);
                    if (cmp_r2_max == r2)
                        cmp_r2_max = cmp_r2_max + 1;

                    int cmp_u_max = u * 1000 / (1000 - compare_delta * 10);
                    int cmp_u_min = u * 1000 / (1000 + compare_delta * 10);
                    if (cmp_u_max == u)
                        cmp_u_max = cmp_u_max + 1;

                    if (cmp_r1_min <= bref.R1 &&
                        cmp_r1_max >= bref.R1 &&
                        cmp_r2_min <= bref.R2 &&
                        cmp_r2_max >= bref.R2 &&
                        cmp_u_min <= bref.U &&
                        cmp_u_max >= bref.U)
                    {
                        compare_counter--;
                    }
                    else
                    {
                        bref.R1 = r1;
                        bref.R2 = r2;
                        bref.U = u;
                        bref.U0 = u0;
                        compare_counter = menu[16].val;
                    }

                    if (block_power_off == 0)
                    {
                        if (compare_counter == 0)
                        {
                            //ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            block_power_off = blocks;
                            block_pre_result = blocks;

                            if (sum_adc[0] / n < 4000) ///НЕТ ПЕРЕГРУЗКИ
                                block_result = blocks;
                        }
                    }
                    else
                    {
                        if (block_result == 0) //была перегрузка.
                        {
                            if (sum_adc[0] / n < 4000)
                            {
                                r1 = bufferR[blocks].R1;
                                r2 = bufferR[blocks].R2;
                                u = bufferR[blocks].U;
                                u0 = bufferR[blocks].U0;

                                if (block_pre_result + 1 == blocks)
                                {
                                    block_result = blocks;
                                }
                                else
                                    block_pre_result = blocks;
                            }
                        }
                    }

                    //запоминаем результат
                    if (block_result == blocks || block_pre_result == blocks)
                    {
                        result.adc0 = sum_adc[0] / n;
                        result.adc1 = sum_adc[1] / n;
                        result.adc2 = sum_adc[2] / n;

                        if (r1 < 4200)
                        {
                            result.R = r1;
                        }
                        else
                        {
                            result.R = r2;
                        }
                        result.U = u;
                        result.U0 = u0;
                        result.Ubatt1 = (bufferR[ON_BLOCK + 1].Ubatt + bufferR[ON_BLOCK + 2].Ubatt + bufferR[ON_BLOCK + 3].Ubatt) / 3;
                        result.Ubatt0 = (bufferR[ON_BLOCK - 1].Ubatt + bufferR[ON_BLOCK - 2].Ubatt + bufferR[ON_BLOCK - 3].Ubatt) / 3;

                        if (block_result == blocks)
                            xQueueSend(send_queue, (void *)&result, (portTickType)0);
                    }

                    //копируем данные АЦП в буфер АЦП
                    static int nb;
                    static int cb;
                    if (block_result == blocks)
                    {
                        nb = blocks;
                        cb = sizeof(buffer_ADC_copy) / ADC_BUFFER;
                    }
                    if (blocks >= nb && cb-- > 0)
                    {
                        memcpy(&buffer_ADC_copy[ADC_BUFFER * (blocks - nb)], ptre - ADC_BUFFER, ADC_BUFFER);
                    }
                }
            }

            //Оставляем свободное место (1/4) в буфере для вычисления сопротивления после отключения ВВ источника
            if (block_power_off == 0 && blocks > DATALEN * 3 / 4)
                blocks = DATALEN * 3 / 4;

        } while (++blocks < DATALEN);

        ESP_ERROR_CHECK(adc_digi_stop());

        //ВЫРУБАЕМ (на всякий случай)
        gpio_set_level(ENABLE_PIN, 1);

        //выключаем питание, если больше нет каналов в очереди
        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
        {
#ifdef MULTICHAN
            pcf8575_set(BIT(POWER_BIT) | BIT(NB_PWR_BIT));

            // int a = pcf8575_read(INT_PIN); //сброс INT
#endif
#ifndef NODEBUG
            ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
#endif
        }

        //РЕЗУЛЬТАТ

        char buf[WS_BUF_SIZE];
        snprintf((char *)buf, sizeof(buf), "(on:%3d off:%3d err:%3d) \"channel\":%d,\"U\":%d,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%d,\"in\":%d", block_power_on, block_power_off, data_errors, result.channel, result.U, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0, result.input);
        xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
        printf("%s\n", buf);

        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
            xEventGroupSetBits(ready_event_group, END_MEASURE);

        // printf("Time:\n3 block: %lld\n3 block off: %lld\noff: %lld\n", t2 - t1, t3 - t2, time_off - t1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    };
}

int getResult_Data(char *line, int data_pos)
{
    const char *header = {"id,adc0,adc1,adc2\n"};
    char *pos = line;
    int l = 0;
    if (data_pos == 0)
    {
        strcpy(line, header);
        l = strlen(header);
        pos = line + l;
    }
    l += sprintf(pos, "%d,%d,%d,%d\n", data_pos, bufferR[data_pos].R1, bufferR[data_pos].U, bufferR[data_pos].R2);
    return l;
}

int getADC_Data(char *line, uint8_t **ptr_adc, int *num)
{
    adc_digi_output_data_t *p = (void *)(*ptr_adc);

    if ((*ptr_adc) == 0)
    {
        p = (void *)buffer_ADC_copy;
        (*ptr_adc) = (uint8_t *)p;
#if ADC_COUNT_READ == 2
        strcpy(line, "id,adc0,adc1,adc2,res0,res1\n");
#elif ADC_COUNT_READ == 5
        strcpy(line, "id,adc0,adc1,adc2,adc3,adc4\n");
#endif
        return 2;
    };

    while ((uint8_t *)p < (uint8_t *)buffer_ADC_copy + sizeof(buffer_ADC_copy))
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
            sprintf(line, "%5d, %4d, %4d, %4d, %4d, %4d\n", (*num), p->type2.data, (p + 1)->type2.data, (p + 2)->type2.data, (p + 3)->type2.data, (p + 4)->type2.data);
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
