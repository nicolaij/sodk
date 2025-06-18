#include "main.h"
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_partition.h"

#include "driver/gpio.h"

#include "esp_adc/adc_continuous.h" //esp-idf v5

#include "esp_timer.h"

#include "esp_rom_sys.h"

#include "esp_wifi.h"

#include <math.h>

static const char *TAG = "power";

// Счетчик разряда батареи
RTC_DATA_ATTR uint8_t BattLow = 0;

RTC_DATA_ATTR result_t history_data[HISTORY_SIZE];
RTC_DATA_ATTR uint16_t history_head = 0;
RTC_DATA_ATTR uint16_t history_tail = 0;

// величина перегрузки измерительного канала ед. АЦП
#define OVERLOADADC (int)(4095 - (0.03 * 4095.0))
#define UNDERVOLTAGE 4000 // min 4В

/*
    БИТ
    0 - пред. состояние входа in
    1 - пред. состояние петли 1-го канала, при высоковольт. измерении
    2 - пред. состояние петли 2-го канала, при высоковольт. измерении
    3 - пред. состояние петли 3-го канала, при высоковольт. измерении
    4 - пред. состояние петли 4-го канала, при высоковольт. измерении
    5 - пред. состояние петли 1-го канала, при низквольт. измерении
    6 - пред. состояние петли 2-го канала, при низквольт. измерении
    7 - пред. состояние петли 3-го канала, при низквольт. измерении
    8 - пред. состояние петли 4-го канала, при низквольт. измерении
*/
RTC_DATA_ATTR unsigned int measure_flags = 0;

/*
    Предыдущее значение сопротивления
*/
RTC_DATA_ATTR int measure_chan[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/*
    Переменные для переключения времени измерений
*/
RTC_DATA_ATTR int8_t step_time[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RTC_DATA_ATTR int8_t step_time_switch[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const int step_time_const[] = {0, 63, 125, 250, 500, 1000, 2000, 4000, 8000};
#define STEP_TIME_COUNT 3

// кольцевой буфер для визуализации
static calcdata_t bufferR[RINGBUFLEN];
uint16_t buffer_head = 0;
uint16_t buffer_tail = 0;

static uint8_t buffer_ADC[ADC_BUFFER + sizeof(adc_digi_output_data_t) * 4];
static uint8_t buffer_ADC_copy[ADC_BUFFER * 400];

int d_input; // состояние внешнего входа (-1 - состояние отправлено через модем)

#define ON_BLOCK 10

typedef struct
{
    adc_channel_t channel;
    adc_atten_t k;
    int max;
    int maxlv;
} measure_t;

#ifdef NODEBUG
int terminal_mode = -1;
#else
int terminal_mode = 1;
#endif

const measure_t chan_r[] = {
    {.channel = ADC_CHANNEL_0, .max = 15999, .maxlv = 15999},  // Напряжение акк
    {.channel = ADC_CHANNEL_1, .max = 5999, .maxlv = 159},     // Основной канал (1)
    {.channel = ADC_CHANNEL_4, .max = 599999, .maxlv = 5999},  // Усиленный канал (2)
    {.channel = ADC_CHANNEL_3, .max = 599999, .maxlv = 15999}, // Напряжение 0 проводника
    {.channel = ADC_CHANNEL_2, .max = 599999, .maxlv = 15999}, // Напряжение источника питания
};

adc_continuous_handle_t adc_handle = NULL;

// Коэффициенты каналов
// U, Ulv, I1, I2, U0, U0lv, Ubat
int koeff[7][2] = {0};

// return mV
int volt(int adc)
{
    int res = ((adc * koeff[0][0]) / 10 + koeff[0][1]);
    if (res < 0)
        return 0;
    else
        return res;
};

// return mV
int voltlv(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = ((adc * koeff[1][0]) / 1000 + koeff[1][1]);
    if (res < 0)
        return 0;
    else
        return res;
};

// return mV
int voltBatt(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = ((adc * koeff[6][0]) / 1000 + koeff[6][1]);
    if (res < 0)
        return 0;
    else
        return res;
};

// return mV
int volt0(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = ((adc * koeff[4][0]) / 10 + koeff[4][1]);
    if (res < 0)
        return 0;
    else
        return res;
};

// return mV
int volt0lv(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = ((adc * koeff[5][0]) / 1000 + koeff[5][1]);
    if (res < 0)
        return 0;
    else
        return res;
};

int current(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = (adc * koeff[2][0]) / 10 + koeff[2][1];

    return res;
};

int kOm(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    if (adc_u <= 1 || u < UNDERVOLTAGE)
        return 0;
    int c = current(adc_r);
    if (c <= 0)
        return chan_r[1].max;

    int r = u * 1000 / c;
    if (r > chan_r[1].max || r < 0)
        return chan_r[1].max;
    return r;
};

int kOmlv(int adc_u, int adc_r)
{
    int u = voltlv(adc_u);
    if (adc_u <= 1 || u < UNDERVOLTAGE)
        return 0;
    int c = current(adc_r);
    if (c <= 0)
        return chan_r[1].maxlv;

    int r = u * 1000 / c;
    if (r > chan_r[1].maxlv || r < 0)
        return chan_r[1].maxlv;
    return r;
};

// return nA
int current2chan(int adc)
{
    int res = (adc * koeff[3][0]) / 1000 + koeff[3][1];
    return res;
};

int kOm2chan(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    if (adc_u <= 1 || u < UNDERVOLTAGE)
        return 0;
    int c = current2chan(adc_r);
    if (c <= 0)
        return chan_r[2].max;
    int r = (u * 1000) / c;
    if (r > chan_r[2].max)
        return chan_r[2].max;
    return r;
};

int kOm2chanlv(int adc_u, int adc_r)
{
    int u = voltlv(adc_u);
    if (adc_u <= 1 || u < UNDERVOLTAGE)
        return 0;
    int c = current2chan(adc_r);
    if (c <= 0)
        return chan_r[2].maxlv;
    int r = u * 1000 / c;
    if (r > chan_r[2].maxlv || r < 0)
        return chan_r[2].maxlv;
    return r;
};

static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_BUFFER * 4,
        .conv_frame_size = ADC_BUFFER};

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t adc_pattern[5] = {0};
    for (int n = 0; n < 5; n++)
    {
#ifdef ADC12dB
        adc_pattern[n].atten = ADC_ATTEN_DB_12;
#else
        adc_pattern[n].atten = ADC_ATTEN_DB_6; // ADC_ATTEN_DB_12;
#endif
        adc_pattern[n].channel = chan_r[n].channel;
        adc_pattern[n].unit = ADC_UNIT_1;
        adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    };

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 5,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = ADC_FREQ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
}

static void IRAM_ATTR pcf_int_handler(void *arg)
{
    static cmd_t cmd = {.cmd = 3, .channel = 0};
    xQueueSendFromISR(uicmd_queue, &cmd, (TickType_t)0);
    // uint32_t gpio_num = (uint32_t)arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void btn_task(void *arg)
{
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BIT64(BTN_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    int old_btn_state = gpio_get_level(BTN_PIN);
    int new_btn_state = old_btn_state;

    const int pulse = get_menu_val_by_id("pulse");

    while (1)
    {
        new_btn_state = gpio_get_level(BTN_PIN);
        if (old_btn_state != new_btn_state)
        {
            old_btn_state = new_btn_state;
            if (new_btn_state == 1)
            {
                // для отладки схемы pulse -1
                if (pulse == -1)
                {
                    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, NULL);
                }
                else
                {
                    start_measure(1, 0);
                }
            }
            reset_sleep_timeout();
        }

        vTaskDelay(1);
    }
}

#define MEDIAN(a, p) (MAX(a[0].p, a[1].p) == MAX(a[1].p, a[2].p)) ? MAX(a[0].p, a[2].p) : MAX(a[1].p, MIN(a[0].p, a[2].p))

void adc_task(void *arg)
{

    cmd_t cmd_power = {};
    result_t result = {};

    unsigned int measure_current_flags = measure_flags;

    // признак высоковольтных измерений
    int hv_measure = 0;

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT64(ENABLE_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));

    // Interrupt from PCF8575
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BIT64(PCF_INT_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(PCF_INT_PIN, pcf_int_handler, (void *)PCF_INT_PIN);

    int d_in = -1;

    continuous_adc_init();

    // сбрасывем буфер
    buffer_head = 0;
    buffer_tail = 0;

    int timeoutCounter = 0;

    const int Trepeatlv = get_menu_val_by_id("Trepeatlv");
    const int Trepeathv = get_menu_val_by_id("Trepeathv");
    const int overvolt_value = get_menu_val_by_id("Overvolt");

    koeff[0][0] = get_menu_val_by_id("kU");
    koeff[0][1] = get_menu_val_by_id("offsU");
    koeff[1][0] = get_menu_val_by_id("kUlv");
    koeff[1][1] = get_menu_val_by_id("offsUlv");
    koeff[2][0] = get_menu_val_by_id("kR1");
    koeff[2][1] = get_menu_val_by_id("offsR1");
    koeff[3][0] = get_menu_val_by_id("kR2");
    koeff[3][1] = get_menu_val_by_id("offsR2");
    koeff[4][0] = get_menu_val_by_id("kU0");
    koeff[4][1] = get_menu_val_by_id("offsU0");
    koeff[5][0] = get_menu_val_by_id("kU0lv");
    koeff[5][1] = get_menu_val_by_id("offsU0lv");
    koeff[6][0] = get_menu_val_by_id("kUbat");
    koeff[6][1] = get_menu_val_by_id("offsUbat");

    const int moving_avg_length = get_menu_val_by_id("avgcnt");
    const int compare_counter_val = get_menu_val_by_id("avgcomp");
    // const int exp_filter_k = get_menu_val_by_id("Kfilter");

    const int UbatEnd = get_menu_val_by_id("UbatEnd");
    const int UbatLow = get_menu_val_by_id("UbatLow");

    const int percRlv = get_menu_val_by_id("percRlv");
    const int percU0lv = get_menu_val_by_id("percU0lv");

    // Ограничение времени импульса (0 - не вкл. ВВ источник)
    int pulse = get_menu_val_by_id("pulse");

    const int offsetADC0 = get_menu_val_by_id("offstADC0");
    const int offsetADC1 = get_menu_val_by_id("offstADC1");
    const int offsetADC2 = get_menu_val_by_id("offstADC2");
    const int offsetADC3 = get_menu_val_by_id("offstADC3");
    const int offsetADC4 = get_menu_val_by_id("offstADC4");

    while (1)
    {
        BaseType_t err = xQueueReceive(uicmd_queue, &cmd_power, pdMS_TO_TICKS(Trepeatlv * 60 * 1000));
        if (err != pdTRUE) // истек timeout в случае если интервал измерений > таймаута WiFi
        {
            if ((++timeoutCounter) % (Trepeathv / Trepeatlv) == 1)
            {
                start_measure(0, 0);
            }
            else
            {
                start_measure(0, 2); // LV only
            }
            continue;
        }

        if (cmd_power.cmd == 3)
        {
            ESP_LOGV("pcf8575", "read pcf8575");

            d_input = pcf8575_read(IN1_BIT);
            if (d_input != d_in)
            {
                char buf[10];
                d_in = d_input;
                int l = snprintf((char *)buf, sizeof(buf), "IN: %d", d_input);
                ESP_LOGI("pcf8575", "%s", buf);
                xRingbufferSend(wsbuf_handle, buf, l + 1, 0);
                if (d_input)
                {
                    measure_flags |= 0b1;
                }
                else
                {
                    measure_flags &= ~0b1;
                }
            }
            continue;
        };

        // отключаем обработку прерываний от pcf
        // gpio_isr_handler_remove(PCF_INT_PIN);
        ESP_ERROR_CHECK(gpio_intr_disable(PCF_INT_PIN));

        ESP_LOGV(TAG, "Start measure ch:%d cmd:%d", cmd_power.channel, cmd_power.cmd);

        // подаем питание на источник питания, OpAmp, делитель АЦП батареи
        // включаем канал
        if (cmd_power.channel > 0)
            result.channel = cmd_power.channel; // 1..4, 5..8(LV)
        else
            result.channel = 0;

        if (result.channel < 5)
            hv_measure = 1;

        result.ttime = time(0);

        if (cmd_power.channel > 0)
            pcf8575_set(cmd_power.channel);
        else if (cmd_power.cmd != 4)
        {
            pcf8575_set(POWER_CMDON);
        }

        if (cmd_power.cmd > 3) // Калибровка ADC
        {
            pulse = 0;
        }

        uint8_t *ptr = buffer_ADC;
        uint8_t *ptr_adc_begin = buffer_ADC;

        int block = 0;
        int block_power_off = 0;
        int block_power_off_step = step_time_const[step_time[result.channel]];
        int block_result = 0;
        int counter_block_ADC_buffer = 0;
        int overload = 0;

        calcdata_t sum_moving_avg = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        // опорное значение
        calcdata_t bref = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        uint16_t moving_avg_counter = 0;
        int compare_counter = 0;
        const int compare_delta = 2;
        bool compare_ok = false;

        calcdata_t digital_filter = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};
        calcdata_t median_filter[3] = {{.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0}, {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0}, {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0}};
        int median_filter_fill = 0;

        // сумма на всем измерении для расчета среднего для калибровки
        calcdata_t sum_adc_full = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

        int count_adc_full = 0;

        int data_errors = 0;

        int Ubatt0counter = 0;
        result.Ubatt0 = 0;
        int Ubatt1counter = 0;
        result.Ubatt1 = 0;

        ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
        int64_t t1 = esp_timer_get_time();
        static int64_t ta[50];

        do
        {
            uint32_t req = ADC_BUFFER;
            uint32_t ret = 0;

            while (req > 0)
            {
                ESP_ERROR_CHECK(adc_continuous_read(adc_handle, ptr, req, &ret, ADC_MAX_DELAY));

                ptr = ptr + ret;
                req = req - ret;
            }

            if (block < sizeof(ta) / sizeof(ta[0]))
            {
                ta[block] = esp_timer_get_time();
            }

            if (block == ON_BLOCK) // включаем источник ВВ
            {
                // t2 = esp_timer_get_time();
                // включаем источник питания
                if (pulse > 0)
                {
                    if (cmd_power.cmd == 1)
                    {
                        ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                    }
                    else if (cmd_power.cmd == 2)
                    {
                        pcf8575_set(LV_CMDON);
                    }
                }
            }

            calcdata_t sum_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};
            calcdata_t current_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

            // расчет среднего каждую 1 мс
            // calcdata_t sum_avg_block = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

            int n = 0;
            // int overvolt = 0;
            uint8_t fill_data_line = 0;
            adc_digi_output_data_t *p = (void *)buffer_ADC;
            while ((uint8_t *)p <= ptr - ADC_COUNT_READ * sizeof(adc_digi_output_data_t))
            {
                // fill data
                if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel)
                {
                    current_adc.Ubatt = p->type2.data;
                    p++;
                    fill_data_line |= 0b1;
                }

                if (p->type2.unit == 0 && p->type2.channel == chan_r[1].channel)
                {
                    current_adc.R1 = p->type2.data;
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
                    current_adc.U0 = p->type2.data;
                    p++;
                    fill_data_line |= 0b1000;
                }

                if (p->type2.unit == 0 && p->type2.channel == chan_r[4].channel)
                {
                    current_adc.U = p->type2.data;
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

                /*
                                current_adc.Ubatt -= offsetADC0;
                                current_adc.R1 -= offsetADC1;
                                current_adc.R2 -= offsetADC2;
                                current_adc.U0 -= offsetADC3;
                                current_adc.U -= offsetADC4;
                */
                median_filter[median_filter_fill % 3] = current_adc;
                median_filter_fill++;

                if (median_filter_fill >= 3)
                {
                    digital_filter.R1 = MEDIAN(median_filter, R1);
                    digital_filter.U = MEDIAN(median_filter, U);
                    digital_filter.R2 = MEDIAN(median_filter, R2);
                    digital_filter.Ubatt = MEDIAN(median_filter, Ubatt);
                    digital_filter.U0 = MEDIAN(median_filter, U0);
                }
                else
                {
                    digital_filter = current_adc;
                }

                n++;
                sum_adc.R1 += digital_filter.R1;
                sum_adc.U += digital_filter.U;
                sum_adc.R2 += digital_filter.R2;
                sum_adc.Ubatt += digital_filter.Ubatt;
                sum_adc.U0 += digital_filter.U0;
            }; // buffer_ADC

            // обрывок данных
            size_t d = (ptr - ptr_adc_begin) % (ADC_COUNT_READ * sizeof(adc_digi_output_data_t));
            if (d > 0)
            {
                memcpy(buffer_ADC, ptr_adc_begin, d);
                ptr = buffer_ADC + d;
                ESP_LOGE("adc", "Обрывок: %d\n", (int)d);
            }
            else
            {
                ptr = buffer_ADC;
            }

            if (n == 0)
            {
                ESP_LOGE("adc", "Data error: %4d %d\n", block, n);
                continue;
            }

            if (n != 10)
            {
                ESP_LOGE("adc", "%4d %d\n", block, n);
            }

            // Расчет по средним блокам 1 мс
            int v = 0;
            bufferR[buffer_head].Ubatt = voltBatt(sum_adc.Ubatt / n - offsetADC0);
            if (cmd_power.cmd == 1)
            {
                bufferR[buffer_head].R1 = kOm(sum_adc.U / n - offsetADC4, sum_adc.R1 / n - offsetADC1);
                bufferR[buffer_head].R2 = kOm2chan(sum_adc.U / n - offsetADC4, sum_adc.R2 / n - offsetADC2);
                v = volt(sum_adc.U / n - offsetADC4);
                bufferR[buffer_head].U = v;
                bufferR[buffer_head].U0 = volt0(sum_adc.U0 / n - offsetADC3);
            }
            else
            {
                bufferR[buffer_head].R1 = kOmlv(sum_adc.U / n - offsetADC4, sum_adc.R1 / n - offsetADC1);
                bufferR[buffer_head].R2 = kOm2chanlv(sum_adc.U / n - offsetADC4, sum_adc.R2 / n - offsetADC2);
                v = voltlv(sum_adc.U / n - offsetADC4);
                bufferR[buffer_head].U = v;
                bufferR[buffer_head].U0 = volt0lv(sum_adc.U0 / n - offsetADC3);
            }

            if (v > overvolt_value * 1000)
            {
                // ВЫРУБАЕМ
                if (cmd_power.cmd == 1)
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                else if (cmd_power.cmd == 2)
                    pcf8575_set(LV_CMDOFF);

                block_power_off = block;
            }

            if (block > ON_BLOCK && block_power_off == 0)
            {
                // отсечка по времени
                if (pulse > 0)
                {
                    if ((esp_timer_get_time() - t1) > (pulse * 1000LL))
                    {
                        // ВЫРУБАЕМ
                        if (cmd_power.cmd == 1)
                            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        else if (cmd_power.cmd == 2)
                            pcf8575_set(LV_CMDOFF);

                        block_power_off = block;
                    }
                }
                else
                {
                    block_power_off = 1000; // для калибровки
                    block_result = 1000;
                }
            }

            // считаем среднее для калибровки
            if (block > (ON_BLOCK + 10) && (block_result == 0 || block <= block_result))
            {
                count_adc_full += n; // сумма на всем интервале измерения для калибровки
                sum_adc_full.R1 += sum_adc.R1;
                sum_adc_full.U += sum_adc.U;
                sum_adc_full.R2 += sum_adc.R2;
                sum_adc_full.Ubatt += sum_adc.Ubatt;
                sum_adc_full.U0 += sum_adc.U0;
            };

            // запоминаем напряжение батареи
            if (block >= (ON_BLOCK - 5) && block < ON_BLOCK)
            {
                result.Ubatt0 += bufferR[buffer_head].Ubatt;
                Ubatt0counter++;
            }
            else if (block >= (ON_BLOCK + 3) && block < (ON_BLOCK + 8))
            {
                result.Ubatt1 += bufferR[buffer_head].Ubatt;
                Ubatt1counter++;
            }

            // считаем скользящее среднее
            if (block > ON_BLOCK)
            {
                sum_moving_avg.R1 += bufferR[buffer_head].R1;
                sum_moving_avg.R2 += bufferR[buffer_head].R2;
                sum_moving_avg.U += bufferR[buffer_head].U;
                sum_moving_avg.U0 += bufferR[buffer_head].U0;
                moving_avg_counter++;

                if (moving_avg_counter > moving_avg_length)
                {
                    moving_avg_counter--;
                    uint16_t delpos = (buffer_head - moving_avg_counter) & (RINGBUFLEN - 1);
                    sum_moving_avg.R1 -= bufferR[delpos].R1;
                    sum_moving_avg.R2 -= bufferR[delpos].R2;
                    sum_moving_avg.U -= bufferR[delpos].U;
                    sum_moving_avg.U0 -= bufferR[delpos].U0;
                }

                int r1 = sum_moving_avg.R1 / moving_avg_counter;
                int r2 = sum_moving_avg.R2 / moving_avg_counter;
                int u = sum_moving_avg.U / moving_avg_counter;
                int u0 = sum_moving_avg.U0 / moving_avg_counter;

                // ESP_DRAM_LOGD(TAG, "r2: %d / %d = %d; %d", sum_moving_avg.R2, moving_avg_counter, r2, bufferR[buffer_head].R2);

                // Поиск наклона
                int cmp_r1_max = (r1 * 1000) / (1000 - compare_delta * 10);
                int cmp_r1_min = (r1 * 1000) / (1000 + compare_delta * 10);
                if (cmp_r1_max == r1)
                    cmp_r1_max = cmp_r1_max + 1;

                int cmp_r2_max = (r2 * 1000) / (1000 - compare_delta * 10);
                int cmp_r2_min = (r2 * 1000) / (1000 + compare_delta * 10);
                if (cmp_r2_max == r2)
                    cmp_r2_max = cmp_r2_max + 1;

                int cmp_u_max = (u * 1000) / (1000 - compare_delta * 10);
                int cmp_u_min = (u * 1000) / (1000 + compare_delta * 10);
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
                    compare_counter = compare_counter_val;
                    // ESP_DRAM_LOGD("compare", "reset: %d", block);
                }

                // окончание измерений
                if (block_power_off == 0)
                {
                    if (compare_counter == 0)
                    {
                        compare_ok = true;
                        // ESP_DRAM_LOGD("compare", "OK: %d", block);
                        if (step_time[result.channel] == 0) // первое измерение после старта
                        {
                            for (int i = 1; i < sizeof(step_time_const) / sizeof(step_time_const[0]); i++)
                            {
                                if (block <= step_time_const[i])
                                {
                                    step_time[result.channel] = i;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            // счетчик необходимости переключения диапазона
                            if (block <= step_time_const[step_time[result.channel] - 1])
                            {
                                step_time_switch[result.channel]--;
                            }
                            else if (block <= step_time_const[step_time[result.channel]])
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

                    if (block == block_power_off_step)
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

                    if (block == block_power_off_step)
                    {
                        ESP_LOGV("adc", "compare off %d", block);

                        // ВЫРУБАЕМ
                        if (cmd_power.cmd == 1)
                            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        else if (cmd_power.cmd == 2)
                            pcf8575_set(LV_CMDOFF);

                        block_power_off = block;

                        if (sum_adc.R1 / n < OVERLOADADC) // НЕТ ПЕРЕГРУЗКИ измерительного канала
                            block_result = block;
                        else
                        {
                            overload = 1;
                            // counter_block_ADC_buffer = 0; //при перегрузке пишем копию после отключения
                        }
                    }
                }
                else
                {
                    if (block_result == 0) // была перегрузка или отключились по timeout
                    {
                        if (sum_adc.R1 / n < OVERLOADADC)
                        {
                            if (overload > 0) // при выходе из перегрузки среднее не действительно
                            {
                                r1 = bufferR[buffer_head].R1;
                                r2 = bufferR[buffer_head].R2;
                                u = bufferR[buffer_head].U;
                                u0 = bufferR[buffer_head].U0;
                            }

                            block_result = block;
                        }
                        else
                        {
                            overload = 1;
                        }
                    }
                }

                // запоминаем результат
                if (block == block_result)
                {
                    if (current(sum_adc.R1 / n - offsetADC1) >= 100000) // > 100мкА
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

                    sum_adc_full.Ubatt /= count_adc_full;
                    sum_adc_full.R1 /= count_adc_full;
                    sum_adc_full.R2 /= count_adc_full;
                    sum_adc_full.U0 /= count_adc_full;
                    sum_adc_full.U /= count_adc_full;

                    //  ESP_LOGD(TAG, "block: %d = adc 0:%d (%d), 1:%d (%d), 2:%d (%d)", block, sum_adc.R1 / n, r1, sum_adc.U / n, u, sum_adc.R2 / n, r2);
                    //   ESP_LOGD(TAG, "on result: Ubatt0 summ: %d, count: %d", result.Ubatt0, Ubatt0counter);
                    //   ESP_LOGD(TAG, "on result: Ubatt1 summ: %d, count: %d", result.Ubatt1, Ubatt1counter);

                    if (Ubatt0counter > 0)
                        result.Ubatt0 = result.Ubatt0 / Ubatt0counter;

                    if (Ubatt1counter > 0)
                        result.Ubatt1 = result.Ubatt1 / Ubatt1counter;

                    // увеличивем интервал при низком напряжении
                    if (result.Ubatt1 < UbatLow)
                    {
                        result.flags.d_batt_low = 1;
                        BattLow += 1;

                        ESP_LOGD(TAG, "Ubatt low: %d", result.Ubatt1);

                        // отключаем если при lv измерении напряжение упало ниже минимума
                        if (result.Ubatt1 < UbatEnd)
                        {
                            BattLow = 100;
                        }
                    }
                }
            }

            // буферизируем adc
            if (block >= 0) // ON_BLOCK)
            {
                if (counter_block_ADC_buffer < sizeof(buffer_ADC_copy) / ADC_BUFFER) // копируем буфер
                {
                    memcpy(buffer_ADC_copy + ADC_BUFFER * counter_block_ADC_buffer, buffer_ADC, ADC_BUFFER);
                    counter_block_ADC_buffer++;
                }
                else if (block_result > 0 && block >= block_result)
                {
                    // заканчиваем измерение
                    break;
                }
            }

            buffer_head = (buffer_head + 1) & (RINGBUFLEN - 1);
            if (buffer_head == buffer_tail) // увеличиваем хвост
                buffer_tail = (buffer_head + 1) & (RINGBUFLEN - 1);

            if (block < sizeof(ta) / sizeof(ta[0]))
            {
                ta[block] = esp_timer_get_time() - ta[block];
            }

            block++;
        } while (block < 9000); // ограничение 9 сек

        ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));

        // сравниваем с предыдущими измерениями измерения
        if (result.R > 0)
        {
            if (abs(measure_chan[result.channel] - result.R) * 100 / abs(result.R) > percRlv)
            {
                if (xHandleNB)
                    xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль

                ESP_LOGD(TAG, "Differences in measurements found.");

                // запоминаем данные для сравнения
                measure_chan[result.channel] = result.R;
            }
        }

        // определяем процент напряжение на обратном проводе
        int pr = (result.U0 * 100) / result.U;
        if (pr > percU0lv) // 75%
        {
            measure_current_flags |= BIT(result.channel);
        }
        else
        {
            measure_current_flags &= ~BIT(result.channel);
        }

        result.flags.d_in = measure_flags & 1;

        if (result.channel % 4 == 1)
            result.flags.d_changed_ch1 |= ((measure_current_flags & BIT(result.channel)) > 0);
        if (result.channel % 4 == 2)
            result.flags.d_changed_ch2 |= ((measure_current_flags & BIT(result.channel)) > 0);
        if (result.channel % 4 == 3)
            result.flags.d_changed_ch3 |= ((measure_current_flags & BIT(result.channel)) > 0);
        if (result.channel % 4 == 0)
            result.flags.d_changed_ch4 |= ((measure_current_flags & BIT(result.channel)) > 0);

        xQueueSend(send_queue, &result, (TickType_t)0);
        xQueueSend(adc_queue, &sum_adc_full, (TickType_t)0);

        // История
        history_data[history_head] = result;
        history_head = (history_head + 1) % HISTORY_SIZE;
        if (history_head == history_tail)
            history_tail = (history_tail + 1) % HISTORY_SIZE;

        // проверяем изменения U0lv и запускаем hv измерения
        // выключаем питание, если больше нет каналов в очереди
        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
        {
            if (hv_measure == 0 && measure_current_flags != measure_flags)
            {
                start_measure(0, 1);
            }
        };

        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
        {
            // завершаем
            measure_flags = measure_current_flags;
            // ВЫРУБАЕМ каналы
            pcf8575_set(POWER_CMDOFF);

            // читаем pcf, для сброса бита INT
            pcf_int_handler(NULL);
            // Подключаем прерывания от pcf
            ESP_ERROR_CHECK(gpio_intr_enable(PCF_INT_PIN));
            xEventGroupSetBits(status_event_group, END_MEASURE);
        };

        // РЕЗУЛЬТАТ
        static char buf[WS_BUF_SIZE];
        // int len_data = snprintf((char *)buf, sizeof(buf), "{\"channel\":\"%d\",\"num\":%d,\"dt\":\"%s\",\"U\":%.1f,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%.1f,\"time\":%d,\"Temp\":%.01f,\"Flags\":\"0x%04X\"}", result.channel, bootCount + timeoutCounter, get_datetime(result.ttime), result.U / 1000.0, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0 / 1000.0, result.time, result.flags.value);
        int len_data = snprintf(buf, WS_BUF_SIZE, OUT_CHANNEL, get_menu_val_by_id("idn"), result.channel, bootCount, get_datetime(result.ttime), OUT_DATA_CHANNEL(result));

        len_data += snprintf(buf + len_data, WS_BUF_SIZE - len_data, OUT_ADD_COMMON, tsens_out, result.flags.value);

        xRingbufferSend(wsbuf_handle, buf, len_data + 1, 0);

        ESP_LOGI("result", "%s", buf);
        ESP_LOGD("result", "Avg ADC (%d) 0:%d, 1:%d, 2:%d, 3:%d, 4:%d", count_adc_full, sum_adc_full.Ubatt, sum_adc_full.R1, sum_adc_full.R2, sum_adc_full.U0, sum_adc_full.U);

        for (int i = 0; i < sizeof(ta) / sizeof(ta[0]) - 1; i++)
        {
            if (ta[i] >= 1000)
                ESP_LOGE("result", "Block %i process time %lli us", i, ta[i]);
        }

        vTaskDelay(1);
    };
}

int getResult_Data(char *line, int data_pos)
{
    const char *header = {"id,R1,U,R2\n"};
    char *pos = line;
    int l = 0;
    unsigned int ringpos = (unsigned int)(buffer_tail + data_pos) & (RINGBUFLEN - 1);

    if (data_pos == 0)
    {
        strcpy(line, header);
        l = strlen(header);
        pos = line + l;
    }

    if (data_pos >= ((buffer_head - buffer_tail) & (RINGBUFLEN - 1)))
    {
        return 0;
    }

    l += sprintf(pos, "%d,%d,%d,%d\n", data_pos, bufferR[ringpos].R1, bufferR[ringpos].U, bufferR[ringpos].R2);
    return l;
}

int getADC_Data(char *line, int data_pos, const int mode)
{
    const char *header = {"id,adc0,adc1,adc2,adc3,adc4\n"};
    char *pos = line;
    int l = 0;

    // const int exp_filter_k = get_menu_val_by_id("Kfilter");

    calcdata_t digital_filter = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

    calcdata_t current_adc = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};
    static calcdata_t median_filter[3] = {{.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0}, {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0}, {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0}};
    static int median_filter_fill = 0;

    uint8_t fill_data_line = 0;

    adc_digi_output_data_t *p = (void *)(buffer_ADC_copy + data_pos * ADC_COUNT_READ * sizeof(adc_digi_output_data_t));

    if (data_pos == 0)
    {
        strcpy(line, header);
        l = strlen(header);
        pos = line + l;

        median_filter_fill = 0;
    }

    while ((uint8_t *)p < (uint8_t *)buffer_ADC_copy + sizeof(buffer_ADC_copy))
    {
        // fill data
        if (p->type2.unit == 0 && p->type2.channel == chan_r[0].channel)
        {
            current_adc.Ubatt = p->type2.data;
            p++;
            fill_data_line |= 0b1;
        }

        if (p->type2.unit == 0 && p->type2.channel == chan_r[1].channel)
        {
            current_adc.R1 = p->type2.data;
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
            current_adc.U0 = p->type2.data;
            p++;
            fill_data_line |= 0b1000;
        }

        if (p->type2.unit == 0 && p->type2.channel == chan_r[4].channel)
        {
            current_adc.U = p->type2.data;
            p++;
            fill_data_line |= 0b10000;
        }

        if (fill_data_line != 0b11111)
        {
            if (fill_data_line == 0)
                p++;

            ESP_LOGE("adc copy", "Error data pos: %i", data_pos);
            continue;
        }

        // все данные заполнены
        fill_data_line = 0;

        median_filter[median_filter_fill % 3] = current_adc;
        median_filter_fill++;

        if (mode == 1 && median_filter_fill >= 3)
        {
            digital_filter.R1 = MEDIAN(median_filter, R1);
            digital_filter.U = MEDIAN(median_filter, U);
            digital_filter.R2 = MEDIAN(median_filter, R2);
            digital_filter.Ubatt = MEDIAN(median_filter, Ubatt);
            digital_filter.U0 = MEDIAN(median_filter, U0);
        }
        else
        {
            digital_filter = current_adc;
        }

        //                 id adc0 adc1 adc2 adc3 adc4
        l += sprintf(pos, "%5d,%4d,%4d,%4d,%4d,%4d\n", data_pos, digital_filter.Ubatt, digital_filter.R1, digital_filter.R2, digital_filter.U0, digital_filter.U);

        return l;
    }

    return 0;
};

result_t *get_history_data(int id)
{
    int pos = (history_tail + id) % HISTORY_SIZE;
    if (pos == history_head)
        return NULL;
    return &history_data[pos];
};
