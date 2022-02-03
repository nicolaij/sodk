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

#include <driver/dac.h>
#include "driver/ledc.h"

#include "main.h"

#define LED_GPIO 2

int buffer1[10000];
int buffer2[10000];
int buffer3[10000];

static const char *TAG = "ad/da";
#define V_REF 1090

#define BTN_GPIO 0

typedef struct
{
    adc1_channel_t channel;
    int k;
    int max;
} measure_t;

measure_t chan_r[] = {
    {.channel = ADC1_CHANNEL_5, .k = 90, .max = 1000},
    {.channel = ADC1_CHANNEL_7, .k = 1750, .max = 20000},
};

int k_U = 5336;

/**
 * @brief I2S ADC/DAC mode init.
 */
void i2s_init(void)
{
    int i2s_num = I2S_NUM_0;
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .channel_format = I2S_CHANNEL_MONO,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = 1024,
        .use_apll = 1,
    };
    //install and start i2s driver
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    //init DAC pad
    //i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    //init ADC pad
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_5);
}

/**
 * @brief debug buffer data
 */
void disp_buf(uint8_t *buf, int length)
{
    printf("======\n");
    for (int i = 0; i < length; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 32 == 0)
        {
            printf("\n");
        }
    }
    printf("======\n");
}

void btn_task(void *arg)
{
    int btn = 0;
    gpio_pad_select_gpio(BTN_GPIO);
    gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_GPIO, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(POWER_PIN);
    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_PIN, 1);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 1000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = 25,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0};

    ledc_channel_config(&ledc_channel);

    while (1)
    {
        int s = gpio_get_level(BTN_GPIO);
        if (s == 0)
        {
            gpio_set_level(POWER_PIN, 0);

            btn++;
            if (btn > 5)
                btn = 0;

            ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 1024 * btn / 5);
            ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        else
            vTaskDelay(50 / portTICK_RATE_MS);
    }
    /*
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
*/
}

/**
 * @brief I2S ADC/DAC example
 *        1. Erase flash
 *        2. Record audio from ADC and save in flash
 *        3. Read flash and replay the sound via DAC
 *        4. Play an example audio file(file format: 8bit/8khz/single channel)
 *        5. Loop back to step 3

void example_i2s_adc_dac(void *arg)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                              ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    else
    {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }
    //1. Erase flash
    example_erase_flash();
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    int flash_wr_size = 0;
    size_t bytes_read, bytes_written;

    //2. Record audio from ADC and save in flash
#if RECORD_IN_FLASH_EN
    char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
    uint8_t *flash_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
    i2s_adc_enable(EXAMPLE_I2S_NUM);
    while (flash_wr_size < FLASH_RECORD_SIZE)
    {
        //read data from I2S bus, in this case, from ADC.
        i2s_read(EXAMPLE_I2S_NUM, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        example_disp_buf((uint8_t *)i2s_read_buff, 64);
        //save original data from I2S(ADC) into flash.
        esp_partition_write(data_partition, flash_wr_size, i2s_read_buff, i2s_read_len);
        flash_wr_size += i2s_read_len;
        esp_rom_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    }
    i2s_adc_disable(EXAMPLE_I2S_NUM);
    free(i2s_read_buff);
    i2s_read_buff = NULL;
    free(flash_write_buff);
    flash_write_buff = NULL;
#endif

    uint8_t *flash_read_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
    uint8_t *i2s_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
    while (1)
    {

        //3. Read flash and replay the sound via DAC
#if REPLAY_FROM_FLASH_EN
        for (int rd_offset = 0; rd_offset < flash_wr_size; rd_offset += FLASH_SECTOR_SIZE)
        {
            //read I2S(ADC) original data from flash
            esp_partition_read(data_partition, rd_offset, flash_read_buff, FLASH_SECTOR_SIZE);
            //process data and scale to 8bit for I2S DAC.
            example_i2s_adc_data_scale(i2s_write_buff, flash_read_buff, FLASH_SECTOR_SIZE);
            //send data
            i2s_write(EXAMPLE_I2S_NUM, i2s_write_buff, FLASH_SECTOR_SIZE, &bytes_written, portMAX_DELAY);
            printf("playing: %d %%\n", rd_offset * 100 / flash_wr_size);
        }
#endif

        //4. Play an example audio file(file format: 8bit/16khz/single channel)
        printf("Playing file example: \n");
        int offset = 0;
        int tot_size = sizeof(audio_table);
        example_set_file_play_mode();
        while (offset < tot_size)
        {
            int play_len = ((tot_size - offset) > (4 * 1024)) ? (4 * 1024) : (tot_size - offset);
            int i2s_wr_len = example_i2s_dac_data_scale(i2s_write_buff, (uint8_t *)(audio_table + offset), play_len);
            i2s_write(EXAMPLE_I2S_NUM, i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
            offset += play_len;
            example_disp_buf((uint8_t *)i2s_write_buff, 32);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        example_reset_play_mode();
    }
    free(flash_read_buff);
    free(i2s_write_buff);
    vTaskDelete(NULL);
}
 */

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}
/*
void i2s_adc(void *arg)
{
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_11db);
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);

    uint32_t voltage;
    esp_adc_cal_get_voltage(ADC1_CHANNEL_5, &characteristics, &voltage);
    ESP_LOGI(TAG, "%d mV", voltage);
    vTaskDelay(200 / portTICK_RATE_MS);

    i2s_init();
    int i2s_read_len = 1024 * 4;
    char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
    size_t bytes_read = 0;

    while (1)
    {
        i2s_read(I2S_NUM_0, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);

        ESP_LOGI("I2S", "read: %d", bytes_read);

        disp_buf((uint8_t *)i2s_read_buff, bytes_read);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
*/
void dual_adc(void *arg)
{

    check_efuse();
    /*
    esp_err_t status = adc_vref_to_gpio(ADC_UNIT_2, GPIO_NUM_25);
    if (status == ESP_OK) {
        printf("v_ref routed to GPIO\n");
    } else {
        printf("failed to route v_ref\n");
    }
*/

    gpio_pad_select_gpio(POWER_PIN);
    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_PIN, 1);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 10000,                     // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = 25,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0};

    ledc_channel_config(&ledc_channel);

    dac_output_enable(DAC_CHANNEL_2);
    dac_output_voltage(DAC_CHANNEL_2, 127);

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(chan_r[0].channel, ADC_ATTEN_11db);
    adc1_config_channel_atten(chan_r[1].channel, ADC_ATTEN_11db);

    adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_11db);

    esp_adc_cal_characteristics_t *adc1_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    printf("ADC1: ");
    esp_adc_cal_value_t val_type1 = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc1_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type1 == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("eFuse Vref %d", adc1_chars->vref);
    }
    else if (val_type1 == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Two Point");
    }
    else
    {
        printf("Default");
    }
    printf("\n");

    esp_adc_cal_characteristics_t *adc2_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    printf("ADC2: ");
    esp_adc_cal_value_t val_type2 = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc2_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type2 == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("eFuse Vref %d", adc2_chars->vref);
    }
    else if (val_type2 == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Two Point");
    }
    else
    {
        printf("Default");
    }
    printf("\n");

    /*
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_value_t r = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);
    ESP_LOGI(TAG, "esp_adc_cal_value_t %d ", r);

    uint32_t voltage;
    esp_adc_cal_get_voltage(ADC1_CHANNEL_5, &characteristics, &voltage);
    ESP_LOGI(TAG, "%d mV", voltage);
    */
    vTaskDelay(200 / portTICK_RATE_MS);

    //gpio_pad_select_gpio(LED_GPIO);
    //gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    //gpio_set_level(LED_GPIO, 0);

    //результат измерений
    result_t result;

    cmd_t cmd;
    cmd.cmd = 0;
    cmd.pwm = 0;

    while (1)
    {
        //int64_t t1 = esp_timer_get_time();
        //int64_t timeout = 100000;
        //gpio_set_level(LED_GPIO, 1);
        //gpio_set_level(POWER_PIN, 0);

        //while (esp_timer_get_time() - t1 < timeout)
        xQueueReceive(uicmd_queue, &cmd, (portTickType)portMAX_DELAY);

        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 1024 * cmd.pwm / 100);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

        int64_t t1 = esp_timer_get_time();
        int64_t timeout = 100000;

        int len = 0;
        int pos_off = 0;

        int chan = 0;

        /*
        Сначала меряем на ADC1_CHANNEL_5, если показания < ~250 adc
        дальше меряем ADC1_CHANNEL_7 (x20)
        */
        if (cmd.cmd == 2) //Pulse
        {
            gpio_set_level(POWER_PIN, 0);
            while (esp_timer_get_time() - t1 < (timeout * 2))
            {
                if (esp_timer_get_time() - t1 > timeout)
                {
                    gpio_set_level(POWER_PIN, 1);
                    if (pos_off == 0)
                    {
                        pos_off = len;
                        //Переключаем каналы
                        if (len > 5)
                            if ((buffer1[len - 5] + buffer1[len - 4] + buffer1[len - 3] + buffer1[len - 2] + buffer1[len - 1]) / 5 < 200)
                                chan = 1;
                    }
                }

                //adc_read[0] = adc1_get_raw(ADC1_CHANNEL_5);
                //adc_read[0] = adc1_get_raw(ADC1_CHANNEL_7);
                buffer1[len] = adc1_get_raw(chan_r[chan].channel);
                adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &buffer2[len]);
                len++;
                if (len >= sizeof(buffer1) / sizeof(buffer1[0]))
                    break;
            };
            gpio_set_level(POWER_PIN, 1);

            int s1 = 0;
            int s2 = 0;
            int r = 0;
            int avg_count = (len - pos_off) / 10; //10% усредняем
            for (int i = 0; i < avg_count; i++)
            {
                s1 += buffer1[pos_off + i];
                s2 += buffer2[pos_off + i];
                r += kOm(buffer2[pos_off + i], buffer1[pos_off + i], chan);
            }

            result.adc1 = s1 / avg_count;
            result.adc2 = s2 / avg_count;
            result.U = volt(result.adc2);
            result.R = r / avg_count;

            xQueueSend(adc1_queue, (void *)&result, (portTickType)0);

            int c = 0;

            for (int i = 0; i < len; i++)
            {
                //printf("%4d: %4d (%4d V), %4d (%4d kOm)\n", i, buffer1[i], volt(buffer1[i]), buffer2[i], kOm(buffer1[i], buffer2[i]));
                //printf("%4d, %4d, %4d\n", i, volt(buffer2[i]), kOm(buffer2[i], buffer1[i]));
                if (chan == 1)
                {
                    if (i >= pos_off)
                        c = 1;
                }

                printf("%4d, %4d, %4d, %4d\n", i, buffer2[i], buffer1[i], kOm(buffer2[i], buffer1[i], c));
                taskYIELD();
            }
        }
        else if (cmd.cmd == 1) //ON
        {
            gpio_set_level(POWER_PIN, 0);
            chan = 0;
            while (cmd.cmd == 1)
            {
                t1 = esp_timer_get_time();
                len = 0;
                while (esp_timer_get_time() - t1 < timeout * 2)
                {
                    //adc_read[0] = adc1_get_raw(ADC1_CHANNEL_5);
                    adc_read[0] = adc1_get_raw(chan_r[chan].channel);
                    adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &adc_read[1]);
                    buffer1[len] = adc_read[0];
                    buffer2[len] = adc_read[1];
                    len++;
                    if (len >= sizeof(buffer1) / sizeof(buffer1[0]))
                        break;
                };

                int s1 = 0;
                int s2 = 0;
                for (int i = 0; i < len; i++)
                {
                    s1 += buffer1[i];
                    s2 += buffer2[i];
                }
                adc_read[0] = s1 / len;
                adc_read[1] = s2 / len;
                int v = volt(adc_read[1]);
                int r = kOm(adc_read[1], adc_read[0], chan);

                xQueueSend(adc1_queue, adc_read, (portTickType)0);

                printf("ADC2: U = %d V (%4d), ADC1: R = %d kOm (%4d) buffer:%d\n", v, adc_read[1], r, adc_read[0], len);

                if (pdTRUE == xQueueReceive(uicmd_queue, &cmd, 800 / portTICK_PERIOD_MS))
                {
                    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 1024 * cmd.pwm / 100);
                    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
                }
            }
            gpio_set_level(POWER_PIN, 1);
        }

        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

/*
esp_err_t app_main(void)
{
    example_i2s_init();
    esp_log_level_set("I2S", ESP_LOG_INFO);
    xTaskCreate(example_i2s_adc_dac, "example_i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
    xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);
    return ESP_OK;
}
*/

int volt(int adc)
{
    return adc * 1000 / k_U;
};

int kOm(int adc_u, int adc_r, int channel_r)
{
    if (adc_r == 0)
        return chan_r[channel_r].max;
    int r = adc_u * chan_r[channel_r].k / adc_r;
    if (r > chan_r[channel_r].max)
        return chan_r[channel_r].max;
    return r;
};
