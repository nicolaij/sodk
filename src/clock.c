/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "driver/i2c.h"

#include "main.h"

static const char *TAG = "clock";

static void obtain_time(void);
static void initialize_sntp(void);

//#define DS3231_POWER_PIN 26

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define DS3231_ADDRESS 0x68   ///< I2C address for DS3231
#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_TEMPERATUREREG \
    0x11 ///< Temperature register (high byte - low byte is at 0x12), 10-bit
         ///< temperature value

/** Constants */
#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 \
    946684800 ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

bool osf = false;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_master_clock_read(i2c_port_t i2c_num, uint8_t *data, uint8_t len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, DS3231_TIME, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }
    // vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_clock_write(i2c_port_t i2c_num, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, DS3231_TIME, ACK_CHECK_EN);
    i2c_master_write(cmd, data, len, ACK_VAL);
    i2c_master_stop(cmd);
    return i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
}

static esp_err_t i2c_master_clock_write_byte(i2c_port_t i2c_num, uint8_t addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    return i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_RATE_MS);
}

/*!
    @brief  Convert a binary coded decimal value to binary. RTC stores
  time/date values as BCD.
    @param val BCD value
    @return Binary value
*/
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
/*!
    @brief  Convert a binary value to BCD format for the RTC registers
    @param val Binary value
    @return BCD value
*/
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }

void time_sync_notification_cb(struct timeval *tv)
{
    char strftime_buf[64];
    time_t now = tv->tv_sec;
    struct tm timeinfo = {0};

    time(&now);
    localtime_r(&now, &timeinfo);

    // sync to DS3231
    uint8_t buffer[8] = {bin2bcd(timeinfo.tm_sec),
                         bin2bcd(timeinfo.tm_min),
                         bin2bcd(timeinfo.tm_hour),
                         0,
                         bin2bcd(timeinfo.tm_mday),
                         bin2bcd(timeinfo.tm_mon),
                         bin2bcd(timeinfo.tm_year - 100)};

    xSemaphoreTake(i2c_mux, portMAX_DELAY);

    if (i2c_master_clock_write(I2C_MASTER_NUM, buffer, 8) == ESP_OK)
    {
        ESP_LOGI("DS3231", "Clock set");
        if (osf) //часы сброшены
        {
            ESP_ERROR_CHECK(i2c_master_clock_write_byte(I2C_MASTER_NUM, 0x0e, 0)); // Control Register (0Eh)
            ESP_ERROR_CHECK(i2c_master_clock_write_byte(I2C_MASTER_NUM, 0x0f, 0)); // Status Register (0Fh)
        }
    }

    xSemaphoreGive(i2c_mux);

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Minks is: %s", strftime_buf);

    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

void clock_task(void *arg)
{
    time_t now;
    struct tm timeinfo;
    uint8_t buf[64];

    time(&now);

    // Set timezone to BY
    setenv("TZ", "UTC-3", 1);
    tzset();

    localtime_r(&now, &timeinfo);

    strftime((char *)buf, sizeof(buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "ESP date/time: %s", (char *)buf);

    // init i2c DS3231
    // Power on DS3231
    // gpio_pad_select_gpio(DS3231_POWER_PIN);
    // gpio_set_direction(DS3231_POWER_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_level(DS3231_POWER_PIN, 1);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    xSemaphoreTake(i2c_mux, portMAX_DELAY);

    // ESP_ERROR_CHECK(i2c_master_init());
    if (ESP_OK == i2c_master_clock_read(I2C_MASTER_NUM, buf, 0x12))
    {
        ESP_LOGI("DS3231", "DS3231 read OK. Temperature %.2f°C", (float)((int16_t)(buf[DS3231_TEMPERATUREREG] << 8 | buf[DS3231_TEMPERATUREREG + 1]) / 64) * 0.25f);
        if (buf[DS3231_STATUSREG] & (1 << 7))
        {
            osf = true;
            ESP_LOGE("DS3231", "Oscillator Stop Flag (OSF)");
        }

        //Выводим время
        timeinfo.tm_sec = bcd2bin(buf[0]);
        timeinfo.tm_min = bcd2bin(buf[1]);
        timeinfo.tm_hour = bcd2bin(buf[2]);
        timeinfo.tm_mday = bcd2bin(buf[4]);
        timeinfo.tm_mon = bcd2bin(buf[5]);
        timeinfo.tm_year = bcd2bin(buf[6]) + 100; // tm_year:1900,  DS3231:2000

        time_t t = mktime(&timeinfo);
        struct timeval now = {.tv_sec = t};
        settimeofday(&now, NULL);

        ESP_LOGI("DS3231", "%s", asctime(&timeinfo));
    }
    else
    {
        ESP_LOGE("DS3231", "Not found");
    }

    xSemaphoreGive(i2c_mux);

    initialize_sntp();

    /*
        // Is time set? If not, tm_year will be (1970 - 1900).
        if (timeinfo.tm_year < (2016 - 1900))
        {
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            ESP_LOGW(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time();
            // update 'now' variable with current time
            time(&now);
        }
    */
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void initialize_sntp(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "by.pool.ntp.org");
    sntp_setservername(1, "time.windows.com");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

//Текущая дата-время в формате
void cur_time(char *buf)
{
    time_t now;
    struct tm timeinfo;

    time(&now);

    localtime_r(&now, &timeinfo);

    strftime((char *)buf, 32, "%y-%m-%d %H.%M", &timeinfo);
}