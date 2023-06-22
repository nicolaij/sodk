#include "main.h"
#include <stdio.h>
#include <string.h>
// #include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_partition.h"

#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"

#include "esp_timer.h"

// #include "esp_adc_cal.h"
#include "esp_rom_sys.h"

#include "soc/apb_saradc_reg.h"

#include "esp_wifi.h"

// RTC_DATA_ATTR int last_chan;

RTC_DATA_ATTR int8_t step_time[] = {0, 0, 0, 0, 0};
RTC_DATA_ATTR int8_t step_time_switch[] = {0, 0, 0, 0, 0};
const int step_time_const[] = {0, 125, 250, 500, 1000, 2000, 4000, 8000};
#define STEP_TIME_COUNT 3

// кольцевой буфер для визуализации
calcdata_t bufferR[RINGBUFLEN];
unsigned int bhead = 0;
unsigned int btail = 0;

uint8_t buffer_ADC[ADC_BUFFER + sizeof(adc_digi_output_data_t) * 4];
uint8_t buffer_ADC_copy[ADC_BUFFER * 200];

result_t result = {};

#define ON_BLOCK 10

typedef struct
{
    adc_channel_t channel;
    adc_atten_t k;
    int max;
} measure_t;

#ifdef SWAP_ADC1_0
const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_0, .max = 6999},   //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC2_CHANNEL_0, .max = 1000},   // Напряжение источника питания
    {.channel = ADC1_CHANNEL_4, .max = 299999}, //
    {.channel = ADC1_CHANNEL_1, .max = 10000},  //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .max = 1000},   // Напряжение 0 проводника
};
#else
const measure_t chan_r[] = {
    {.channel = ADC_CHANNEL_1, .max = 6999},   //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC_CHANNEL_0, .max = 1000},   // Напряжение источника питания
    {.channel = ADC_CHANNEL_4, .max = 299999}, //~х22
    {.channel = ADC_CHANNEL_0, .max = 10000},  //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC_CHANNEL_3, .max = 1000},   // Напряжение 0 проводника
};
#endif

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
    printf("APB_SARADC_SAR_PATT_TAB1_REG: %8lX\n", REG_READ(APB_SARADC_SAR_PATT_TAB1_REG));
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
    if (adc_u < 10 || u < 4000)
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

static void continuous_adc_init(adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_BUFFER * 4,
        .conv_frame_size = ADC_BUFFER * 4};
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
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
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static QueueHandle_t gpio_evt_queue = NULL;

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
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

#if MULTICHAN
    // result.input = pcf8575_read(IN1_BIT);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = BIT64(INT_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
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

    int d_in = -1;

    while (1)
    {
#if MULTICHAN
        uint32_t io_num;
        if (xQueueReceive(gpio_evt_queue, &io_num, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            char buf[10];
            result.input = pcf8575_read(IN1_BIT);
            if (result.input != d_in)
            {
                int l = snprintf((char *)buf, sizeof(buf), "IN: %d", result.input);
                ESP_LOGI("pcf8575", "%s", buf);
                xRingbufferSend(wsbuf_handle, buf, l + 1, 0);
                d_in = result.input;
            }
        }
#else
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

        new_btn_state = gpio_get_level(BTN_PIN);
        if (old_btn_state != new_btn_state)
        {
            old_btn_state = new_btn_state;
            if (new_btn_state == 1)
            {
                // для отладки схемы pulse -1
                if (menu[0].val == -1)
                {
                    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, NULL);
                }
                else
                {
                    d_in = -1;
                    start_measure(1);
                }
            }

            reset_sleep_timeout();
        }

        // работа с терминалом
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
                unsigned int p = btail;
                int i = 0;
                do
                {
                    printf("%4d %4d %4d %4d\n", i, bufferR[p].R1, bufferR[p].U, bufferR[p].R2);
                    p = (p + 1) & (RINGBUFLEN - 1);
                    i++;
                } while (p != bhead);
            }

            if (cmd == 'p' || cmd == 'P') // print ADC buffer
            {
                adc_digi_output_data_t *p = (void *)buffer_ADC_copy;
                adc_digi_output_data_t *p_good = p;
                int n = 0;

                calcdata_t current_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};
                uint8_t fill_data_line = 0;

                while ((uint8_t *)p < (uint8_t *)buffer_ADC_copy + sizeof(buffer_ADC_copy))
                {

                    n = ((uint8_t *)p - (uint8_t *)buffer_ADC_copy) / sizeof(adc_digi_output_data_t);

                    // fill data
                    if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel)
                    {
                        current_adc.R1 = p->type2.data;
                        p++;
                        fill_data_line |= 0b1;
                        if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel) // ПОВТОР пропускаем
                            p++;
                    }

                    if (p->type2.unit == 1 && p->type2.channel == chan_r[1].channel)
                    {
                        current_adc.U = p->type2.data;
                        p++;
                        fill_data_line |= 0b10;
                    }

                    if (p->type2.unit == 0 && p->type2.channel == chan_r[2].channel)
                    {
                        current_adc.R2 = p->type2.data;
                        p++;
                        fill_data_line |= 0b100;
                    }

                    if (p->type2.unit == 0 && p->type2.channel == chan_r[3].channel)
                    {
                        current_adc.Ubatt = p->type2.data;
                        p++;
                        fill_data_line |= 0b1000;
                    }

                    if (p->type2.unit == 0 && p->type2.channel == chan_r[4].channel)
                    {
                        current_adc.U0 = p->type2.data;
                        p++;
                        fill_data_line |= 0b10000;
                    }

                    if (fill_data_line != 0b11111)
                    {
                        if (fill_data_line == 0)
                            p++;

                        printf("error ADC data");
                        while (p_good < p)
                        {
                            printf(" %d:%d:%d", p_good->type2.unit, p_good->type2.channel, p_good->type2.data);
                            p_good++;
                        }
                        printf("\n");

                        continue;
                    }

                    fill_data_line = 0;
                    p_good = p;

                    printf("%4d %4d %4d %4d %4d %4d\t", n / ADC_COUNT_READ, current_adc.R1, current_adc.U, current_adc.R2, current_adc.Ubatt, current_adc.U0);
                    printf("= %6d %6d %6d\n", kOm(current_adc.U, current_adc.R1), volt(current_adc.U), kOm2chan(current_adc.U, current_adc.R2));

                    // избежать task_wdt: Task watchdog got triggered.
                    if ((n / ADC_COUNT_READ) % 500 == 0)
                        vTaskDelay(1);
                }
            }

            if (cmd == '1')
            {
                pcf8575_set(NB_PWR_CMDON);
            }
            if (cmd == '2')
            {
                pcf8575_set(NB_PWR_CMDOFF);
            }

        }
        // end работа с терминалом
    }
}

void dual_adc(void *arg)
{

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

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(&handle);

    // сбрасывем буфер
    bhead = 0;
    btail = 0;

    while (1)
    {

        xQueueReceive(uicmd_queue, &cmd, portMAX_DELAY);

        // подаем питание на источник питания, OpAmp, делитель АЦП батареи
        // включаем канал

        result.channel = cmd.channel;

        pcf8575_set(cmd.channel);

#ifndef NODEBUG
        ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));
#endif
        uint8_t *ptr = (uint8_t *)buffer_ADC;
        uint8_t *ptr_adc_begin = (uint8_t *)buffer_ADC;
        // uint8_t *ptre = (uint8_t *)buffer_ADC;
        uint8_t *ptrb = (uint8_t *)buffer_ADC;

        int blocks = 0;
        int block_power_on = 0;
        int block_power_off = 0;
        int block_power_off_step = step_time_const[step_time[result.channel]];
        int block_result = 0;
        int counter_block_ADC_buffer = 0;
        int overload = 0;

        int64_t timeout = menu[0].val * 1000LL;

        ESP_ERROR_CHECK(adc_continuous_start(handle));

        int64_t t1 = esp_timer_get_time();
        // int64_t t2 = 0, t3 = 0, time_off = 0;

        calcdata_t sum_bavg = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        // опорное значение
        calcdata_t bref = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        uint16_t avg_len = 0;
        int compare_counter = menu[16].val;
        const int compare_delta = 1;
        bool compare_ok = false;

        int exp_filter_k = menu[20].val;
        int exp_filter[5] = {0, 0, 0, 0, 0};
        // int sum_adc_full[5] = {0, 0, 0, 0, 0};

        // сумма на всем измерении для расчета среднего для калибровки
        calcdata_t sum_adc_full = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

        int count_adc_full = 0;
        bool filter_initialize = false;

        int data_errors = 0;

        do
        {
            uint32_t req = ADC_BUFFER;
            uint32_t ret = 0;
            ptrb = ptr;
            // printf("read adc: ");
            while (req > 0)
            {
                ESP_ERROR_CHECK(adc_continuous_read(handle, ptr, req, &ret, ADC_MAX_DELAY));
                ptr = ptr + ret;
                req = req - ret;
                // printf(" %ld", ret);
            }
            // printf(" %ld\n", ret);

            if (blocks == ON_BLOCK) // включаем источник ВВ
            {
                if (BattLow > 10)
                    break;

                // adc_select(0);
                if ((esp_timer_get_time() - t1) < timeout)
                {
                    // включаем источник питания
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                    // printf("on\n");
                    block_power_on = blocks;
                }
            }

            // int sum_adc[5] = {0, 0, 0, 0, 0};
            calcdata_t sum_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

            calcdata_t current_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

            // расчет среднего каждую 1 мс
            calcdata_t sum_avg01 = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

            int n = 0;
            int overvolt = 0;
            uint8_t fill_data_line = 0;
            adc_digi_output_data_t *p = (void *)buffer_ADC;
            while ((uint8_t *)p <= ptr - ADC_COUNT_READ * sizeof(adc_digi_output_data_t))
            {
                // fill data
                if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel)
                {
                    current_adc.R1 = p->type2.data;
                    p++;
                    fill_data_line |= 0b1;
                    if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel) // ПОВТОР пропускаем
                        p++;
                }

                if (p->type2.unit == 1 && p->type2.channel == chan_r[1].channel)
                {
                    current_adc.U = p->type2.data;
                    p++;
                    fill_data_line |= 0b10;
                }

                if (p->type2.unit == 0 && p->type2.channel == chan_r[2].channel)
                {
                    current_adc.R2 = p->type2.data;
                    p++;
                    fill_data_line |= 0b100;
                }

                if (p->type2.unit == 0 && p->type2.channel == chan_r[3].channel)
                {
                    current_adc.Ubatt = p->type2.data;
                    p++;
                    fill_data_line |= 0b1000;
                }

                if (p->type2.unit == 0 && p->type2.channel == chan_r[4].channel)
                {
                    current_adc.U0 = p->type2.data;
                    p++;
                    fill_data_line |= 0b10000;
                }

                if (fill_data_line != 0b11111)
                {
                    if (fill_data_line == 0)
                        p++;

                    data_errors++;
                    continue;
                }

                // все данные заполнены
                fill_data_line = 0;
                ptr_adc_begin = (void *)p;

                if (filter_initialize == true)
                {
                    exp_filter[0] = (current_adc.R1 * exp_filter_k + exp_filter[0] * (100 - exp_filter_k)) / 100;
                    exp_filter[1] = (current_adc.U * exp_filter_k + exp_filter[1] * (100 - exp_filter_k)) / 100;
                    exp_filter[2] = (current_adc.R2 * exp_filter_k + exp_filter[2] * (100 - exp_filter_k)) / 100;
                    exp_filter[3] = (current_adc.Ubatt * exp_filter_k + exp_filter[3] * (100 - exp_filter_k)) / 100;
                    exp_filter[4] = (current_adc.U0 * exp_filter_k + exp_filter[4] * (100 - exp_filter_k)) / 100;
                }
                else
                {
                    exp_filter[0] = current_adc.R1;
                    exp_filter[1] = current_adc.U;
                    exp_filter[2] = current_adc.R2;
                    exp_filter[3] = current_adc.Ubatt;
                    exp_filter[4] = current_adc.U0;
                    filter_initialize = true;
                }

                n++;
                sum_adc.R1 += exp_filter[0];
                sum_adc.U0 += exp_filter[1];
                sum_adc.R2 += exp_filter[2];
                sum_adc.Ubatt += exp_filter[3];
                sum_adc.U0 += exp_filter[4];

                sum_avg01.R1 += kOm(exp_filter[1], exp_filter[0]);
                sum_avg01.R2 += kOm2chan(exp_filter[1], exp_filter[2]);
                int v = volt(exp_filter[1]);
                sum_avg01.U += v;
                sum_avg01.Ubatt += voltBatt(exp_filter[3]);
                sum_avg01.U0 += volt0(exp_filter[4]);

                if (v > menu[1].val * 1000)
                {
                    overvolt++;
                    if (overvolt >= 3)
                    {
                        // ВЫРУБАЕМ
                        gpio_set_level(ENABLE_PIN, 1);
                        block_power_off = blocks;
                    }
                }
                else
                {
                    overvolt = 0;
                }
            };

            // обрывок данных
            size_t d = (ptr - ptr_adc_begin) % (ADC_COUNT_READ * sizeof(adc_digi_output_data_t));
            if (d > 0)
            {
                memcpy(buffer_ADC, ptr_adc_begin, d);
                ptr = buffer_ADC + d;
                // printf("Обрывок: %d\n", (int)d);
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

            bufferR[bhead].U = sum_avg01.U / n / 1000;
            bufferR[bhead].R1 = sum_avg01.R1 / n;
            bufferR[bhead].R2 = sum_avg01.R2 / n;
            bufferR[bhead].Ubatt = sum_avg01.Ubatt / n;
            bufferR[bhead].U0 = sum_avg01.U0 / n / 1000;

            if (blocks > ON_BLOCK && block_power_off == 0)
            {
                // отсечка по времени
                if ((esp_timer_get_time() - t1) > timeout)
                {
                    // ВЫРУБАЕМ
                    gpio_set_level(ENABLE_PIN, 1);
                    block_power_off = blocks;
                }
                else
                {
                    // отсечка по напряжению, Ubatt
                    if (bufferR[bhead].Ubatt < menu[13].val)
                    {
                        // прекращаем измерения
                        // ВЫРУБАЕМ
                        gpio_set_level(ENABLE_PIN, 1);
                        if (terminal_mode >= 0)
                            printf("UbattEnd: %d < %ld)\n", bufferR[bhead].Ubatt, menu[13].val);
                        BattLow += 100;
                        break;
                    }
                    else if (bufferR[bhead].Ubatt < menu[12].val)
                    {
                        BattLow += 1;
                        if (BattLow > 10)
                        {
                            // ВЫРУБАЕМ
                            gpio_set_level(ENABLE_PIN, 1);
                            // прекращаем измерения
                            if (terminal_mode >= 0)
                                printf("UbattLow: %d < %ld (%d)\n", bufferR[bhead].Ubatt, menu[12].val, BattLow);
                            break;
                        }
                    }
                }
            }

            // считаем среднее
            if (blocks > ON_BLOCK)
            {
                count_adc_full += n; // сумма на всем интервале измерения для калибровки
                sum_adc_full.R1 += sum_adc.R1;
                sum_adc_full.R2 += sum_adc.R2;
                sum_adc_full.U += sum_adc.U;
                sum_adc_full.Ubatt += sum_adc.Ubatt;
                sum_adc_full.U0 += sum_adc.U0;

                sum_bavg.R1 += bufferR[bhead].R1;
                sum_bavg.R2 += bufferR[bhead].R2;
                sum_bavg.U += bufferR[bhead].U;
                sum_bavg.U0 += bufferR[bhead].U0;
                avg_len++;

                if (avg_len > menu[17].val)
                {
                    avg_len--;
                    unsigned int delpos = (bhead - avg_len) & (RINGBUFLEN - 1);
                    sum_bavg.R1 -= bufferR[delpos].R1;
                    sum_bavg.R2 -= bufferR[delpos].R2;
                    sum_bavg.U -= bufferR[delpos].U;
                    sum_bavg.U0 -= bufferR[delpos].U0;
                }

                int r1 = sum_bavg.R1 / avg_len;
                int r2 = sum_bavg.R2 / avg_len;
                int u = sum_bavg.U / avg_len;
                int u0 = sum_bavg.U0 / avg_len;

                /*
                //Поиск максимума
                if (r1 <= bref.R1 &&
                    r2 <= bref.R2)
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
                */

                // Поиск наклона
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

                // окончание измерений
                if (block_power_off == 0)
                {
                    if (compare_counter == 0)
                    {
                        compare_ok = true;
                        if (step_time[result.channel] == 0) // первое измерение после старта
                        {
                            for (int i = 1; i < sizeof(step_time_const) / sizeof(step_time_const[0]); i++)
                            {
                                if (blocks <= step_time_const[i])
                                {
                                    step_time[result.channel] = i;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            // счетчик необходимости переключения диапазона
                            if (blocks <= step_time_const[step_time[result.channel] - 1])
                            {
                                step_time_switch[result.channel]--;
                            }
                            else if (blocks <= step_time_const[step_time[result.channel]])
                            {
                                step_time_switch[result.channel] = 0;
                            }

                            // переключаем диапазон вниз
                            if (step_time_switch[result.channel] <= STEP_TIME_COUNT * -1)
                            {
                                if (step_time[result.channel] > 1)
                                    step_time[result.channel]--;

                                step_time_switch[result.channel] = 0;
                            }
                        }

                        block_power_off_step = step_time_const[step_time[result.channel]];
                    }

                    if (blocks == block_power_off_step)
                    {
                        if (compare_ok == false) // если небыло успешного сравнения
                        {
                            step_time_switch[result.channel]++;

                            // переключаем диапазон вверх
                            if (step_time_switch[result.channel] >= STEP_TIME_COUNT)
                            {
                                if (step_time[result.channel] < sizeof(step_time_const) / sizeof(step_time_const[0]) - 1)
                                    step_time[result.channel]++;

                                step_time_switch[result.channel] = 0;

                                block_power_off_step = step_time_const[step_time[result.channel]];
                            }
                        }
                    }

                    if (blocks == block_power_off_step)
                    {
                        // ВЫРУБАЕМ
                        gpio_set_level(ENABLE_PIN, 1);
                        block_power_off = blocks;

                        if (sum_adc.R1 / n < 4000) // НЕТ ПЕРЕГРУЗКИ измерительного канала
                            block_result = blocks;
                        else
                            overload = 1;
                    }
                }
                else
                {
                    if (block_result == 0) // была перегрузка или отключились по timeout
                    {
                        if (sum_adc.R1 / n < 4000)
                        {
                            if (overload > 0) // при выходе из перегрузки среднее не действительно
                            {
                                r1 = bufferR[bhead].R1;
                                r2 = bufferR[bhead].R2;
                                u = bufferR[bhead].U;
                                u0 = bufferR[bhead].U0;
                            }

                            block_result = blocks;
                        }
                        else
                            overload = 1;
                    }
                }

                // запоминаем напряжение батареи
                if (blocks == ON_BLOCK + 5)
                {
                    result.Ubatt1 = (bufferR[(unsigned int)(bhead - 1) & (RINGBUFLEN - 1)].Ubatt + bufferR[(unsigned int)(bhead - 2) & (RINGBUFLEN - 1)].Ubatt + bufferR[(unsigned int)(bhead - 3) & (RINGBUFLEN - 1)].Ubatt) / 3;
                    result.Ubatt0 = (bufferR[(unsigned int)(bhead - blocks + ON_BLOCK - 1) & (RINGBUFLEN - 1)].Ubatt + bufferR[(unsigned int)(bhead - blocks + ON_BLOCK - 2) & (RINGBUFLEN - 1)].Ubatt + bufferR[(unsigned int)(bhead - blocks + ON_BLOCK - 3) & (RINGBUFLEN - 1)].Ubatt) / 3;
                }

                // запоминаем результат
                if (blocks == block_result)
                {
                    result.adc0 = sum_adc.R1 / n;
                    result.adc1 = sum_adc.U / n;
                    result.adc2 = sum_adc.R2 / n;

                    if (result.adc0 > 100) //~ 150 uA
                    {
                        result.R = r1;
                    }
                    else
                    {
                        result.R = r2;
                    }
                    result.U = u;
                    result.U0 = u0;
                    result.time = block_result;

                    xQueueSend(send_queue, (void *)&result, (TickType_t)0);
                }
            }

            // буферезируем после включения последнее измерение
            if (blocks >= ON_BLOCK)
            {
                if (uxQueueMessagesWaiting(uicmd_queue) == 0) // последнее измерение
                {
                    if (counter_block_ADC_buffer < sizeof(buffer_ADC_copy) / ADC_BUFFER) // копируем буфер
                    {
                        memcpy(&buffer_ADC_copy[ADC_BUFFER * counter_block_ADC_buffer], ptrb, ADC_BUFFER);
                        counter_block_ADC_buffer++;
                    }
                    else if (block_result > 0 && blocks > (block_result + menu[19].val))
                    {
                        // заканчиваем измерение
                        break;
                    }
                }
                else
                {
                    if (block_result > 0)
                    {
                        // заканчиваем измерение
                        break;
                    }
                }
            }

            bhead = (bhead + 1) & (RINGBUFLEN - 1);
            if (bhead == btail) // увеличиваем хвост
                btail = (btail + 1) & (RINGBUFLEN - 1);

            blocks++;
        } while (blocks < 10000); // ограничение 10 сек

        ESP_ERROR_CHECK(adc_continuous_stop(handle));
        // ESP_ERROR_CHECK(adc_continuous_deinit(handle));

        // ВЫРУБАЕМ (на всякий случай)
        gpio_set_level(ENABLE_PIN, 1);

        // выключаем питание, если больше нет каналов в очереди
        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
        {
            pcf8575_set(POWER_CMDOFF);
            xEventGroupSetBits(ready_event_group, END_MEASURE);
#ifndef NODEBUG
            ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
#endif
        }

        // РЕЗУЛЬТАТ
        char buf[WS_BUF_SIZE];
        int l = snprintf((char *)buf, sizeof(buf), "(on:%3d res:%3d err:%3d) \"channel\":%d,\"U\":%d,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%d,\"in\":%d", block_power_on, block_result, data_errors, result.channel, result.U, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0, result.input);
        xRingbufferSend(wsbuf_handle, buf, l + 1, 0);

        printf("%s\n", buf);
        printf("Avg ADC 0:%d, 1:%d, 2:%d, 3:%d, 4:%d\n", sum_adc_full.R1 / count_adc_full, sum_adc_full.U / count_adc_full, sum_adc_full.R2 / count_adc_full, sum_adc_full.Ubatt / count_adc_full, sum_adc_full.U0 / count_adc_full);

        // printf("Time:\n3 block: %lld\n3 block off: %lld\noff: %lld\n", t2 - t1, t3 - t2, time_off - t1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    };
}

int getResult_Data(char *line, int data_pos)
{
    const char *header = {"id,R1,U,R2\n"};
    char *pos = line;
    int l = 0;
    unsigned int ringpos = (unsigned int)(btail + data_pos) & (RINGBUFLEN - 1);

    if (data_pos == 0)
    {
        strcpy(line, header);
        l = strlen(header);
        pos = line + l;
    }

    if (data_pos >= ((bhead - btail) & (RINGBUFLEN - 1)))
    {
        return 0;
    }

    l += sprintf(pos, "%d,%d,%d,%d\n", data_pos, bufferR[ringpos].R1, bufferR[ringpos].U, bufferR[ringpos].R2);
    return l;
}

int getADC_Data(char *line, int data_pos, bool filter)
{
    const char *header = {"id,adc0,adc1,adc2,adc3,adc4\n"};
    char *pos = line;
    int l = 0;

    int exp_filter_k = menu[20].val;
    static int exp_filter[5] = {0, 0, 0, 0, 0};

    calcdata_t current_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};
    uint8_t fill_data_line = 0;

    adc_digi_output_data_t *p = (void *)(buffer_ADC_copy + data_pos * ADC_COUNT_READ * sizeof(adc_digi_output_data_t));

    if (data_pos == 0)
    {
        strcpy(line, header);
        l = strlen(header);
        pos = line + l;
    }

    while ((uint8_t *)p < (uint8_t *)buffer_ADC_copy + sizeof(buffer_ADC_copy))
    {
        // fill data
        if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel)
        {
            current_adc.R1 = p->type2.data;
            p++;
            fill_data_line |= 0b1;
            if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel) // ПОВТОР пропускаем
                p++;
        }

        if (p->type2.unit == 1 && p->type2.channel == chan_r[1].channel)
        {
            current_adc.U = p->type2.data;
            p++;
            fill_data_line |= 0b10;
        }

        if (p->type2.unit == 0 && p->type2.channel == chan_r[2].channel)
        {
            current_adc.R2 = p->type2.data;
            p++;
            fill_data_line |= 0b100;
        }

        if (p->type2.unit == 0 && p->type2.channel == chan_r[3].channel)
        {
            current_adc.Ubatt = p->type2.data;
            p++;
            fill_data_line |= 0b1000;
        }

        if (p->type2.unit == 0 && p->type2.channel == chan_r[4].channel)
        {
            current_adc.U0 = p->type2.data;
            p++;
            fill_data_line |= 0b10000;
        }

        if (fill_data_line != 0b11111)
        {
            if (fill_data_line == 0)
                p++;

            continue;
        }

        // все данные заполнены
        fill_data_line = 0;

        if (filter && data_pos > 0)
        {
            exp_filter[0] = (current_adc.R1 * exp_filter_k + exp_filter[0] * (100 - exp_filter_k)) / 100;
            exp_filter[1] = (current_adc.U * exp_filter_k + exp_filter[1] * (100 - exp_filter_k)) / 100;
            exp_filter[2] = (current_adc.R2 * exp_filter_k + exp_filter[2] * (100 - exp_filter_k)) / 100;
            exp_filter[3] = (current_adc.Ubatt * exp_filter_k + exp_filter[3] * (100 - exp_filter_k)) / 100;
            exp_filter[4] = (current_adc.U0 * exp_filter_k + exp_filter[4] * (100 - exp_filter_k)) / 100;
        }
        else
        {
            exp_filter[0] = current_adc.R1;
            exp_filter[1] = current_adc.U;
            exp_filter[2] = current_adc.R2;
            exp_filter[3] = current_adc.Ubatt;
            exp_filter[4] = current_adc.U0;
        }

        //                 id   adc0 adc1 adc2 adc3 adc4
        l += sprintf(pos, "%5d, %4d, %4d, %4d, %4d, %4d\n", data_pos, exp_filter[0], exp_filter[1], exp_filter[2], exp_filter[3], exp_filter[4]);

        return l;
    }

    return 0;
};
