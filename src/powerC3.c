#include "main.h"
#include <stdio.h>
#include <string.h>
// #include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_partition.h"

#include "driver/gpio.h"

#if ESP_IDF_VERSION_MAJOR == 5
#include "esp_adc/adc_continuous.h" //esp-idf v5
#endif
#if ESP_IDF_VERSION_MAJOR == 4
#include "driver/adc.h" //esp-idf v4
#endif

#include "esp_timer.h"

// #include "esp_adc_cal.h"
#include "esp_rom_sys.h"

#include "soc/apb_saradc_reg.h"

#include "esp_wifi.h"

static const char *TAG = "power";

// Счетчик разряда батареи
RTC_DATA_ATTR unsigned int BattLow = 1;

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
const int step_time_const[] = {0, 50, 125, 250, 500, 1000, 2000, 4000, 8000};
#define STEP_TIME_COUNT 3

// кольцевой буфер для визуализации
calcdata_t bufferR[RINGBUFLEN];
unsigned int bhead = 0;
unsigned int btail = 0;

uint8_t buffer_ADC[ADC_BUFFER + sizeof(adc_digi_output_data_t) * 4];
uint8_t buffer_ADC_copy[ADC_BUFFER * 300];

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

#ifdef SWAP_ADC1_0
const measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_0, .max = 6999},   //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC2_CHANNEL_0, .max = 1000},   // Напряжение источника питания
    {.channel = ADC1_CHANNEL_4, .max = 299999}, //
    {.channel = ADC1_CHANNEL_1, .max = 10000},  //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC1_CHANNEL_3, .max = 1000},   // Напряжение 0 проводника
};
#elif ADC1_ONLY
const measure_t chan_r[] = {
    {.channel = ADC_CHANNEL_1, .max = 6999, .maxlv = 6999},   //{.channel = ADC1_CHANNEL_1, .k = 1, .max = 2999},  //Основной канал
    {.channel = ADC_CHANNEL_2, .max = 1000, .maxlv = 1000},   // Напряжение источника питания
    {.channel = ADC_CHANNEL_4, .max = 299999, .maxlv = 9999}, //
    {.channel = ADC_CHANNEL_0, .max = 15000, .maxlv = 15000}, //{.channel = ADC1_CHANNEL_0, .k = 1, .max = 10000}, //Напряжение акк
    {.channel = ADC_CHANNEL_3, .max = 1000, .maxlv = 15},     // Напряжение 0 проводника
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

// return mV
int volt(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = (adc * menu[2].val) / 10 + menu[3].val;
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
    int res = (adc * menu[4].val) / 10 + menu[5].val;
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
    int res = ((adc * menu[14].val) + menu[15].val) / 1000;
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
    int res = (adc * menu[10].val) / 10 + menu[11].val;
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
    int res = (adc * menu[12].val) / 10 + menu[13].val;
    if (res < 0)
        return 0;
    else
        return res;
};

int current(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = (adc * menu[6].val) / 10 + menu[7].val;

    return res;
};

int kOm(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    if (adc_u < 10 || u < 4000)
        return 0;
    int c = current(adc_r);
    if (c <= 0)
        return chan_r[0].max;

    int r = u * 1000 / c;
    if (r > chan_r[0].max || r < 0)
        return chan_r[0].max;
    return r;
};

int kOmlv(int adc_u, int adc_r)
{
    int u = voltlv(adc_u);
    if (adc_u < 10 || u < 4000)
        return 0;
    int c = current(adc_r);
    if (c <= 0)
        return chan_r[0].maxlv;

    int r = u * 1000 / c;
    if (r > chan_r[0].maxlv || r < 0)
        return chan_r[0].maxlv;
    return r;
};

int current2chan(int adc)
{
    //    if (adc < 3)
    //        return 0;
    int res = (menu[8].val * adc) / 10 + menu[9].val;

    return res;
};

int kOm2chan(int adc_u, int adc_r)
{
    int u = volt(adc_u);
    if ((adc_u) < 10 || u < 4000)
        return 0;
    int c = current2chan(adc_r);
    if (c <= 0)
        return chan_r[2].max;
    int r = u * 1000 / c;
    if (r > chan_r[2].max || r < 0)
        return chan_r[2].max;
    return r;
};

int kOm2chanlv(int adc_u, int adc_r)
{
    int u = voltlv(adc_u);
    if ((adc_u) < 10 || u < 4000)
        return 0;
    int c = current2chan(adc_r);
    if (c <= 0)
        return chan_r[2].maxlv;
    int r = u * 1000 / c;
    if (r > chan_r[2].maxlv || r < 0)
        return chan_r[2].maxlv;
    return r;
};
#if ESP_IDF_VERSION_MAJOR >= 5

static void continuous_adc_init(adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_BUFFER * 4,
        .conv_frame_size = ADC_BUFFER};

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_FREQ,
#ifdef ADC1_ONLY
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
#else
        .conv_mode = ADC_CONV_BOTH_UNIT,
#endif
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

    int n = 0;
    // Note: Все atten при инициализации должны быть одинаковые!!!???
    adc_pattern[n].atten = ADC_ATTEN_DB_12;
    adc_pattern[n].channel = chan_r[0].channel;
    adc_pattern[n].unit = ADC_UNIT_1;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_12;
    adc_pattern[n].channel = chan_r[1].channel;
#ifdef ADC1_ONLY
    adc_pattern[n].unit = ADC_UNIT_1;
#else
    adc_pattern[n].unit = ADC_UNIT_2;
#endif
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_12;
    adc_pattern[n].channel = chan_r[2].channel;
    adc_pattern[n].unit = ADC_UNIT_1;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_12;
    adc_pattern[n].channel = chan_r[3].channel;
    adc_pattern[n].unit = ADC_UNIT_1;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_12;
    adc_pattern[n].channel = chan_r[4].channel;
    adc_pattern[n].unit = ADC_UNIT_1;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;

    dig_cfg.pattern_num = n;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}
#else
static void continuous_adc_init()
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = ADC_BUFFER * 20, // ??? подобрано по количеству ошибок
        .conv_num_each_intr = ADC_BUFFER * 4,  // подобрано по количеству ошибок
#ifdef ADC1_ONLY
        .adc1_chan_mask = (1 << chan_r[0].channel) | (1 << chan_r[1].channel) | (1 << chan_r[2].channel) | (1 << chan_r[3].channel) | (1 << chan_r[4].channel),
        .adc2_chan_mask = 0,
#else
        .adc1_chan_mask = (1 << chan_r[0].channel) | (1 << chan_r[2].channel) | (1 << chan_r[3].channel) | (1 << chan_r[4].channel),
        .adc2_chan_mask = (1 << chan_r[1].channel),
#endif

    };

    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = false,
        .conv_limit_num = 0,
        .sample_freq_hz = ADC_FREQ,
#ifdef ADC1_ONLY
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
#else
        .conv_mode = ADC_CONV_BOTH_UNIT,
#endif
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    int n = 0;
    // Note: Все atten при инициализации должны быть одинаковые!!!???
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[n].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[n].channel;
#ifdef ADC1_ONLY
    adc_pattern[n].unit = 0;
#else
    adc_pattern[n].unit = 1;
#endif
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[n].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[n].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;
    adc_pattern[n].atten = ADC_ATTEN_DB_11;
    adc_pattern[n].channel = chan_r[n].channel;
    adc_pattern[n].unit = 0;
    adc_pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    n++;

    dig_cfg.pattern_num = n;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}
#endif

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR pcf_int_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
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

    // Interrupt from PCF8575
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = BIT64(PCF_INT_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));

    uint32_t io_num = 0;
    xQueueSend(gpio_evt_queue, &io_num, 0);

    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(PCF_INT_PIN, pcf_int_handler, (void *)PCF_INT_PIN);

    int old_btn_state = gpio_get_level(BTN_PIN);
    int new_btn_state = old_btn_state;

    result_t result = {.channel = 1, .input = -1};
    int d_in = -1;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, 200 / portTICK_PERIOD_MS) == pdTRUE)
        {
            ESP_LOGV("pcf8575", "read pcf8575");

            char buf[10];
            d_input = pcf8575_read(IN1_BIT);
            if (d_input != d_in)
            {
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
                result.input = d_input + 100;
                xQueueSend(send_queue, &result, 0);
            }
        }

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
                    // pcf8575_set(11);
                    start_measure(1, 2);
                }
            }

            reset_sleep_timeout();
        }

        // работа с терминалом
        int ch = EOF;
        int key_code = EOF;
        do
        {
            key_code = ch;
            ch = fgetc(stdin);
            vTaskDelay(10 / portTICK_PERIOD_MS);

        } while (ch != EOF);

        if (key_code != EOF)
        {
            if (key_code == '\n')
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

                if (terminal_mode > -1)
                {
                    ESP_LOGI("info", "Free memory: %lu bytes", esp_get_free_heap_size());
                    ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
                }
            }

            if (key_code == 'w' || key_code == 'W') // stop wifi
            {
                esp_wifi_stop();
            }

            if (key_code == 'r' || key_code == 'R') // print block result
            {
                unsigned int p = btail;
                int i = 0;
                do
                {
                    printf(" %4d %4d %4d %4d\n", i, bufferR[p].R1, bufferR[p].U, bufferR[p].R2);
                    p = (p + 1) & (RINGBUFLEN - 1);
                    i++;
                } while (p != bhead);
            }

            if (key_code == 'p' || key_code == 'P') // print ADC buffer
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
#ifdef ADC1_ONLY
                    if (p->type2.unit == 0 && p->type2.channel == chan_r[1].channel)
#else
                    if (p->type2.unit == 1 && p->type2.channel == chan_r[1].channel)
#endif
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

                    printf(" %4d %4d %4d %4d %4d %4d\t", n / ADC_COUNT_READ, current_adc.R1, current_adc.U, current_adc.R2, current_adc.Ubatt, current_adc.U0);
                    printf("= %6d %6d %6d\n", kOm(current_adc.U, current_adc.R1), volt(current_adc.U), kOm2chan(current_adc.U, current_adc.R2));

                    // избежать task_wdt: Task watchdog got triggered.
                    if ((n / ADC_COUNT_READ) % 100 == 0)
                        vTaskDelay(1);
                }
            }

            if (key_code == '1')
            {
                start_measure(1, 1);
            }
            if (key_code == '2')
            {
                start_measure(2, 1);
            }
            if (key_code == '3')
            {
                start_measure(3, 1);
            }
            if (key_code == '4')
            {
                start_measure(4, 1);
            }
            if (key_code == '5')
            {
                start_measure(5, 2);
            }
            if (key_code == '6')
            {
                start_measure(6, 2);
            }
            if (key_code == '7')
            {
                start_measure(7, 2);
            }
            if (key_code == '8')
            {
                start_measure(8, 2);
            }
            if (key_code == '0')
            {
                start_measure(0, 0);
            }

            if (key_code == 37) // Open LV mode for testing Shift-5
            {
                pcf8575_set(11);
                pcf8575_set(LV_CMDON);
            }
            if (key_code == 94) // Open LV mode for testing Shift-6
            {
                pcf8575_set(12);
                pcf8575_set(LV_CMDON);
            }
            if (key_code == 38) // Open LV mode for testing Shift-7
            {
                pcf8575_set(13);
                pcf8575_set(LV_CMDON);
            }
            if (key_code == 42) // Open LV mode for testing Shift-8
            {
                pcf8575_set(14);
                pcf8575_set(LV_CMDON);
            }
        }
        // end работа с терминалом

        vTaskDelay(1);
    }
}

void dual_adc(void *arg)
{

    cmd_t cmd_power = {};
    result_t result = {};

    unsigned int measure_current_flags = measure_flags;

    // признак высоковольтных измерений
    int hv_measure = 0;

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

#if ESP_IDF_VERSION_MAJOR >= 5
    adc_continuous_handle_t adc_handle = NULL;
    continuous_adc_init(&adc_handle);
#else
    continuous_adc_init();
#endif

    // сбрасывем буфер
    bhead = 0;
    btail = 0;

    while (1)
    {
        xQueueReceive(uicmd_queue, &cmd_power, portMAX_DELAY);

        // отключаем обработку прерываний от pcf
        gpio_isr_handler_remove(PCF_INT_PIN);

        // подаем питание на источник питания, OpAmp, делитель АЦП батареи
        // включаем канал
        result.channel = cmd_power.channel; // 1..4, 5..8(LV)

        pcf8575_set(cmd_power.channel);

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

#if ESP_IDF_VERSION_MAJOR >= 5
        ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
#else
        ESP_ERROR_CHECK(adc_digi_start());
#endif

        int64_t t1 = esp_timer_get_time();
        // int64_t t2 = 0, t3 = 0, time_off = 0;

        calcdata_t sum_bavg = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        // опорное значение
        calcdata_t bref = {.R1 = 0, .R2 = 0, .U = 0, .U0 = 0, .Ubatt = 0};

        uint16_t avg_len = 0;
        int compare_counter = menu[22].val;
        const int compare_delta = 1;
        bool compare_ok = false;

        const int exp_filter_k = menu[25].val;
        int exp_filter[5] = {0, 0, 0, 0, 0};

        // сумма на всем измерении для расчета среднего для калибровки
        calcdata_t sum_adc_full = {.R1 = 0, .U = 0, .R2 = 0, .Ubatt = 0, .U0 = 0};

        int count_adc_full = 0;
        bool filter_initialize = false;

        int data_errors = 0;

        int Ubatt0counter = 0;
        result.Ubatt0 = 0;
        int Ubatt1counter = 0;
        result.Ubatt1 = 0;

        do
        {
            uint32_t req = ADC_BUFFER;
            uint32_t ret = 0;
            ptrb = ptr;

            while (req > 0)
            {
#if ESP_IDF_VERSION_MAJOR >= 5
                ESP_ERROR_CHECK(adc_continuous_read(adc_handle, ptr, req, &ret, ADC_MAX_DELAY));
#else
                ESP_ERROR_CHECK(adc_digi_read_bytes(ptr, req, &ret, ADC_MAX_DELAY));
#endif
                ptr = ptr + ret;
                req = req - ret;
            }

            if (blocks == ON_BLOCK) // включаем источник ВВ
            {

                t1 = esp_timer_get_time();

                // включаем источник питания
                if (timeout > 0)
                {
                    if (cmd_power.channel < 5)
                    {
                        ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                        hv_measure = 1;
                    }
                    else
                        pcf8575_set(LV_CMDON);
                }

                block_power_on = blocks;
            }

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
#ifdef ADC1_ONLY
                if (p->type2.unit == 0 && p->type2.channel == chan_r[1].channel)
#else
                if (p->type2.unit == 1 && p->type2.channel == chan_r[1].channel)
#endif
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
                sum_adc.U += exp_filter[1];
                sum_adc.R2 += exp_filter[2];
                sum_adc.Ubatt += exp_filter[3];
                sum_adc.U0 += exp_filter[4];

                int v = voltBatt(exp_filter[3]);
                sum_avg01.Ubatt += v;

                if (cmd_power.channel >= 5)
                {
                    sum_avg01.R1 += kOmlv(exp_filter[1], exp_filter[0]);
                    sum_avg01.R2 += kOm2chanlv(exp_filter[1], exp_filter[2]);
                    v = voltlv(exp_filter[1]);
                    sum_avg01.U += v;
                    sum_avg01.U0 += volt0lv(exp_filter[4]);
                }
                else
                {
                    sum_avg01.R1 += kOm(exp_filter[1], exp_filter[0]);
                    sum_avg01.R2 += kOm2chan(exp_filter[1], exp_filter[2]);
                    v = volt(exp_filter[1]);
                    sum_avg01.U += v;
                    sum_avg01.U0 += volt0(exp_filter[4]);
                }

                if (v > menu[1].val * 1000)
                {
                    overvolt++;
                    if (overvolt >= 3)
                    {
                        // ВЫРУБАЕМ
                        if (cmd_power.channel < 5)
                            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        else
                            pcf8575_set(LV_CMDOFF);

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
                ESP_LOGE("adc", "Data error: %4d %d\n", blocks, n);
                continue;
            }

            if (n != 10)
            {
                // ESP_LOGE("adc", "%4d %d\n", blocks, n);
            }

            bufferR[bhead].U = sum_avg01.U / n;
            bufferR[bhead].R1 = sum_avg01.R1 / n;
            bufferR[bhead].R2 = sum_avg01.R2 / n;
            bufferR[bhead].Ubatt = sum_avg01.Ubatt / n;
            bufferR[bhead].U0 = sum_avg01.U0 / n;

            if (blocks > ON_BLOCK && block_power_off == 0)
            {
                // отсечка по времени
                if ((esp_timer_get_time() - t1) > timeout)
                {
                    if (timeout > 0)
                    {
                        // ВЫРУБАЕМ
                        if (cmd_power.channel < 5)
                            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        else
                            pcf8575_set(LV_CMDOFF);

                        block_power_off = blocks;
                    }
                    else
                    {
                        block_power_off = 1000; // для калибровки
                        block_result = 1000;
                    }
                }
                else
                {
                    /*
                    // отсечка по напряжению, Ubatt
                    if (bufferR[bhead].Ubatt < menu[17].val)
                    {
                        // прекращаем измерения
                        // ВЫРУБАЕМ
                        if (cmd_power.channel < 5)
                            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        else
                            pcf8575_set(LV_CMDOFF);

                        if (terminal_mode >= 0)
                            printf("UbattEnd: %d < %ld)\n", bufferR[bhead].Ubatt, menu[17].val);
                        BattLow += 100;
                        break;
                    }
                    else if (bufferR[bhead].Ubatt < menu[16].val)
                    {
                        BattLow += 1;
                        if (BattLow > 10)
                        {
                            // ВЫРУБАЕМ
                            if (cmd_power.channel < 5)
                                ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                            else
                                pcf8575_set(LV_CMDOFF);

                            // прекращаем измерения
                            if (terminal_mode >= 0)
                                printf("UbattLow: %d < %ld (%d)\n", bufferR[bhead].Ubatt, menu[16].val, BattLow);
                            break;
                        }
                    }*/
                }
            }

            // считаем среднее для калибровки
            if (blocks > ON_BLOCK + 10)
            {
                count_adc_full += n; // сумма на всем интервале измерения для калибровки
                sum_adc_full.R1 += sum_adc.R1;
                sum_adc_full.R2 += sum_adc.R2;
                sum_adc_full.U += sum_adc.U;
                sum_adc_full.Ubatt += sum_adc.Ubatt;
                sum_adc_full.U0 += sum_adc.U0;
            };

            // считаем среднее
            if (blocks > ON_BLOCK)
            {
                sum_bavg.R1 += bufferR[bhead].R1;
                sum_bavg.R2 += bufferR[bhead].R2;
                sum_bavg.U += bufferR[bhead].U;
                sum_bavg.U0 += bufferR[bhead].U0;
                avg_len++;

                if (avg_len > menu[23].val)
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
                    compare_counter = menu[22].val;
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
                        ESP_LOGV("adc", "compare off %d", blocks);

                        // ВЫРУБАЕМ
                        if (cmd_power.channel < 5)
                            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        else
                            pcf8575_set(LV_CMDOFF);

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
                        {
                            overload = 1;
                        }
                    }
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

                    result.Ubatt0 = result.Ubatt0 / Ubatt0counter;
                    result.Ubatt1 = result.Ubatt1 / Ubatt1counter;

                    // увеличивем интервал при низком напряжении
                    if (result.Ubatt1 < menu[16].val)
                    {
                        BattLow += 1;
        
                        ESP_LOGD(TAG, "Ubatt low: %d", result.Ubatt1);

                        // отключаем если при lv измерении напряжение упало ниже минимума
                        // увеличивем интервал при очень низком напряжении
                        if (cmd_power.channel > 4 && result.Ubatt1 < menu[17].val)
                        {
                            BattLow += 10;
                        }
                    }

                    xQueueSend(send_queue, &result, (TickType_t)0);
                }
            }

            // запоминаем напряжение батареи
            if (blocks >= (ON_BLOCK - 5) && blocks < ON_BLOCK)
            {
                result.Ubatt0 += bufferR[bhead].Ubatt;
                Ubatt0counter++;
            }
            else if (blocks >= (ON_BLOCK + 2) && blocks < (ON_BLOCK + 5))
            {
                result.Ubatt1 += bufferR[bhead].Ubatt;
                Ubatt1counter++;
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
                    else if (block_result > 0 && blocks > (block_result + menu[24].val))
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
        } while (blocks < 9000); // ограничение 9 сек

#if ESP_IDF_VERSION_MAJOR >= 5
        ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
        // ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
#else
        ESP_ERROR_CHECK(adc_digi_stop());
#endif

        // ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));

        // сравниваем с предыдущими измерениями измерения
        if (result.R > 0)
        {
            if (abs(measure_chan[cmd_power.channel] - result.R) * 100 / abs(result.R) > menu[27].val)
            {
                xEventGroupSetBits(ready_event_group, NEED_TRANSMIT);

                ESP_LOGD(TAG, "Differences in measurements found. Set NEED_TRANSMIT");

                // запоминаем данные для сравнения
                measure_chan[cmd_power.channel] = result.R;
            }
        }

        // передаем полюбому если проводим высоковольтные измерения
        if (hv_measure)
        {
            xEventGroupSetBits(ready_event_group, NEED_TRANSMIT);
        }

        // определяем процент напряжение на обратном проводе
        int pr = (result.U0 * 100) / result.U;
        if (pr > menu[26].val) // 75%
        {
            measure_current_flags |= BIT(cmd_power.channel);
        }
        else
        {
            measure_current_flags &= ~BIT(cmd_power.channel);
        }

        // проверяем изменения U0lv и запускаем hv измерения
        // выключаем питание, если больше нет каналов в очереди
        if (uxQueueMessagesWaiting(uicmd_queue) == 0)
        {
            if (hv_measure == 0 && measure_current_flags != measure_flags)
            {
                start_measure(0, 1);
            }
            else
            {
                // завершаем
                measure_flags = measure_current_flags;
                // ВЫРУБАЕМ каналы
                pcf8575_set(POWER_CMDOFF);

                // Подключаем прерывания от pcf
                // читаем pcf, для сброса бита INT
                ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(PCF_INT_PIN, pcf_int_handler, (void *)PCF_INT_PIN));
                uint32_t io_num = 0;
                xQueueSend(gpio_evt_queue, &io_num, 0);

                vTaskDelay(100 / portTICK_PERIOD_MS);

                xEventGroupSetBits(ready_event_group, END_MEASURE);
            };
        };

        // РЕЗУЛЬТАТ
        char buf[WS_BUF_SIZE];
        int l = snprintf((char *)buf, sizeof(buf), "(on:%3d res:%3d err:%3d) \"channel\":%d,\"U\":%.1f,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%.1f", block_power_on, block_result, data_errors, cmd_power.channel, result.U / 1000.0, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0 / 1000.0);
        xRingbufferSend(wsbuf_handle, buf, l + 1, 0);

        printf("%s\n", buf);
        printf("Avg ADC 0:%d, 1:%d, 2:%d, 3:%d, 4:%d\n", sum_adc_full.R1 / count_adc_full, sum_adc_full.U / count_adc_full, sum_adc_full.R2 / count_adc_full, sum_adc_full.Ubatt / count_adc_full, sum_adc_full.U0 / count_adc_full);

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

    const int exp_filter_k = menu[25].val;
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

#ifdef ADC1_ONLY
        if (p->type2.unit == 0 && p->type2.channel == chan_r[1].channel)
#else
        if (p->type2.unit == 1 && p->type2.channel == chan_r[1].channel)
#endif
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
