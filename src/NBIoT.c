#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <sys/time.h>

#include "esp_timer.h"

#include "cJSON.h"

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"

static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 512;
static const char *TAG = "NBIoT";

char pdp_ip[20];
char net_status_current[32];

RTC_DATA_ATTR bool first_run_completed = false;

extern float tsens_out;

extern TaskHandle_t xHandleWifi;

unsigned int fromActiveTime(uint8_t val)
{
    uint8_t unit = val >> 5;
    uint8_t value = val & 0b11111;

    switch (unit)
    {
    case 0:
        return value * 2;
        break;
    case 1:
        return value * 60;
        break;
    case 2:
        return value * 60 * 6;
        break;
    }

    return 0;
}

unsigned int fromPeriodicTAU(uint8_t val)
{
    uint8_t unit = val >> 5;
    uint8_t value = val & 0b11111;

    switch (unit)
    {
    case 0:
        return value * 60 * 10;
        break;
    case 1:
        return value * 60 * 60;
        break;
    case 2:
        return value * 60 * 60 * 10;
        break;
    case 3:
        return value * 2;
        break;
    case 4:
        return value * 30;
        break;
    case 5:
        return value * 60;
        break;
    case 6:
        return value * 60 * 60 * 320;
        break;
    }

    return 0;
}

void nbiot_power_pin(const TickType_t xTicksToDelay)
{
    gpio_set_level(MODEM_POWER, 0);
    vTaskDelay(xTicksToDelay);
    gpio_set_level(MODEM_POWER, 1);
};

esp_err_t print_atcmd(const char *cmd, char *buffer)
{
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_FAIL;
    }

    int len = uart_read_bytes(UART_NUM_1, buffer, (RX_BUF_SIZE - 1), 500 / portTICK_PERIOD_MS);
    if (len < 4)
    {
        return ESP_FAIL;
    };

    buffer[len] = '\0';
    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return ESP_OK;
}

esp_err_t wait_string(char *buffer, const char *wait, TickType_t ticks_to_wait)
{

    const char *err = "ERROR\r\n";

    int64_t start_time = esp_timer_get_time();
    char *pb = buffer;
    *pb = '\0';
    esp_err_t res = ESP_ERR_TIMEOUT;
    do
    {
        int len = uart_read_bytes(UART_NUM_1, pb, (RX_BUF_SIZE - 1), 10);
        // ESP_LOGD(TAG, "len: %d", len);
        if (len > 0)
        {
            pb += len;
            *pb = '\0';

            if (strnstr((const char *)buffer, wait, 100) != NULL)
            {
                res = ESP_OK;
                break;
            }
            else if (strnstr((const char *)buffer, err, 100) != NULL)
            {
                res = ESP_ERR_INVALID_STATE;
                break;
            }
        }
        else if (len == -1)
        {
            return ESP_FAIL;
        }
    } while ((esp_timer_get_time() - start_time) < ticks_to_wait * portTICK_PERIOD_MS * 1000);

    return res;
}

esp_err_t at_reply_wait(const char *cmd, const char *wait, char *buffer, TickType_t ticks_to_wait)
{
    esp_err_t res = ESP_ERR_TIMEOUT;
    int l = strlen(cmd);
    int txBytes = 0;
    txBytes = uart_write_bytes(UART_NUM_1, cmd, l);
    if (txBytes < l)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    res = wait_string(buffer, wait, ticks_to_wait);

    ESP_LOGD(TAG, "Reply wait...:\"%s\"", (char *)buffer);

    return res;
}

esp_err_t at_reply_wait_OK(const char *cmd, char *buffer, TickType_t ticks_to_wait)
{
    return at_reply_wait(cmd, "OK\r\n", buffer, ticks_to_wait);
}
/*
cmd "AT+CMUX?"
wait "+CMUX:"
получаем результат "+CMUX: 0,0,0,31,10,3,30,10,2"
дальше пробел " "
дальше цифры через запятую "0,0,0,31,10,3,30,10,2" (9 штук)
*/
esp_err_t at_reply_get(const char *cmd, const char *wait, char *buffer, int *resultdata, int resultcount, TickType_t ticks_to_wait)
{
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t res = wait_string(buffer, "OK\r\n", ticks_to_wait);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    char *s = strstr((const char *)buffer, wait);
    if (s == NULL)
    {
        return ESP_ERR_NOT_FOUND;
    }
    else
    {
        for (int i = 0; i < resultcount; i++)
        {
            if (i == 0)
                s = strchr(s + 1, ' ');
            else
                s = strchr(s + 1, ',');

            if (s == NULL)
                return ESP_OK;

            ESP_LOGV(TAG, "Found string:\"%s\"", s);

            if (*(s + 1) == '"') // HEX STRING (ex CREG?)
                resultdata[i] = strtol(s + 2, NULL, 16);
            else
                resultdata[i] = atoi(s + 1);
        }
    }

    return res;
}

/*
 Send Data to Remote Via Socket With Data Mode
*/
esp_err_t at_csosend(int socket, char *data, char *buffer)
{
    esp_err_t res = ESP_FAIL;
    char buf[14];
    int len_data = strlen(data);

    snprintf(buf, sizeof(buf), "AT+CSOSEND=%d,", socket);
    int txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    snprintf(buf, sizeof(buf), "%d,", len_data * 2);
    txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 3)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (int i = 0; i < len_data; i++)
    {
        snprintf(buf, 3, "%02x", data[i]);
        txBytes = uart_write_bytes(UART_NUM_1, buf, 2);
        if (txBytes < 2)
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    uart_write_bytes(UART_NUM_1, "\r", 1);

    res = wait_string(buffer, "OK\r\n", 30000 / portTICK_PERIOD_MS);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return res;
}

esp_err_t at_csosend_wait_SEND(int socket, char *data, char *buffer)
{
    esp_err_t res = ESP_FAIL;

    char buf[14];
    int len_data = strlen(data);

    snprintf(buf, sizeof(buf), "AT+CSOSEND=%d,", socket);
    int txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    snprintf(buf, sizeof(buf), "%d,", len_data * 2);
    txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 3)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (int i = 0; i < len_data; i++)
    {
        snprintf(buf, 3, "%02x", data[i]);
        txBytes = uart_write_bytes(UART_NUM_1, buf, 2);
        if (txBytes < 2)
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    uart_write_bytes(UART_NUM_1, "\r", 1);

    res = wait_string(buffer, "SEND:", 30000 / portTICK_PERIOD_MS);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return res;
}

char *get_datetime(time_t ttime)
{
    static char datetime[24];
    struct tm *localtm = localtime(&ttime);
    strftime(datetime, sizeof(datetime), "%Y-%m-%d %T", localtm);
    return datetime;
}

esp_err_t apply_command(const char *cmd, size_t len)
{
    cJSON *json = cJSON_ParseWithLength(cmd, len);
    if (json)
    {
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, json)
        {
            ESP_LOGI(TAG, "Setup %s: %d\n", item->string, item->valueint);
            if (get_menu_val_by_id(item->string) != item->valueint)
            {
                set_menu_val_by_id(item->string, item->valueint);
            }
        }
    }
    else
    {
        const char *er = cJSON_GetErrorPtr();
        ESP_LOGW(TAG, "Parse err: %c in \"%s\"", *er, cmd);
    }
    cJSON_Delete(json);

    return ESP_OK;
}

// Callback при отправке
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "Packet to " MACSTR ", status: %s",
             MAC2STR(mac_addr),
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Callback при получении
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "Received from " MACSTR ", len: %d", MAC2STR(recv_info->src_addr), len);
    apply_command((const char *)data, len);
}

esp_err_t init_espnow(uint8_t *peer_addr)
{
    // 1. Инициализация ESP-NOW
    esp_err_t err_rc = esp_now_init();

    // 2. Регистрация callback'ов
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // 3. Добавьте peer-устройства (если нужно)
    esp_now_peer_info_t peer_info = {
        .peer_addr = {MAC2STR(peer_addr)}, //
        .channel = 0,                      // Должен совпадать с каналом WiFi
        .ifidx = ESP_IF_WIFI_AP,
        .encrypt = false};
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    return err_rc;
}

void modem_task(void *arg)
{
    char data[RX_BUF_SIZE];
    char send_data[TX_BUF_SIZE];

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT64(MODEM_POWER);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(MODEM_POWER, 1));

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    int try_counter = 0;

    strcpy(net_status_current, "OFF");
    bool common_data_transmit = false;

    int protocol = 1; // ESPNOW = 0, TCP = 1, UDP =2

    int mac1 = get_menu_val_by_id("MAC1");
    int mac2 = get_menu_val_by_id("MAC2");
    uint8_t mac_addr[6] = {
        (mac1 >> 16) & 0xFF,
        (mac1 >> 8) & 0xFF,
        (mac1 >> 0) & 0xFF,
        (mac2 >> 16) & 0xFF,
        (mac2 >> 8) & 0xFF,
        (mac2 >> 0) & 0xFF,
    };

    if (mac2 > 0 || mac2 > 0)
    {
        protocol = 0;
    }

    while (1)
    {
        /* Ждем необходимости запуска передачи*/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления беcконечно, для повторного опроса

        bool cpin = false;
        int d_nbiot_error_counter = 0;

        while (protocol > 0) // повторы опроса модуля
        {
            esp_err_t ee = 0;

            switch (d_nbiot_error_counter++ % 4)
            {
            case 1:
            case 3:
                // power on
                ESP_LOGD(TAG, "Try %d. Power ON", d_nbiot_error_counter);
                nbiot_power_pin(1000 / portTICK_PERIOD_MS);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            case 2:
                // power off
                ESP_LOGD(TAG, "Try %d. Power OFF", d_nbiot_error_counter);
                nbiot_power_pin(1500 / portTICK_PERIOD_MS);
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                break;
            default:
                // sleep exit
                ESP_LOGD(TAG, "Try %d. Wakeup", d_nbiot_error_counter);
                nbiot_power_pin(100 / portTICK_PERIOD_MS);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            }

            ee = wait_string(data, "\"EXIT PSM\"", 1000 / portTICK_PERIOD_MS);

            ESP_LOGD(TAG, "Wait... %s", data);

            if (strnstr((const char *)data, "CPIN: READY", 100) != NULL)
            {
                cpin = true;
                ESP_LOGI(TAG, "CPIN: READY");
            }
            else
            {
                // check modem
                ee = at_reply_wait_OK("AT\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "Modem not reply");
                    strcpy(net_status_current, "Modem not reply");

                    if (d_nbiot_error_counter > 8)
                        break;

                    continue;
                }
            };

            if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT))
                break;

            strcpy(net_status_current, "ready");

            at_reply_wait_OK("ATE1;+IPR=115200\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            // Battery Charge
            int cbc[2] = {-1, -1};

            ee = at_reply_get("AT+CBC\r\n", "CBC:", (char *)data, cbc, 2, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGE(TAG, "AT+CBC");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            if (cpin == false)
            {
                if (strnstr(net_status_current, "Error", sizeof(net_status_current)) == NULL) // Если ошибка SIM - то не перезаписываем ее
                    strcpy(net_status_current, "Check SIM...");

                // Enter PIN
                ee = at_reply_wait("AT+CPIN?\r\n", "CPIN: READY", (char *)data, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK)
                {
                    strcpy(net_status_current, "SIM Error!");
                    try_counter++;
                    ESP_LOGW(TAG, "CPIN:\n%s", data);

                    if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT))
                        break;

                    // Reset and Set Phone Functionality
                    if ((try_counter % 3) == 0) // if fail restart sim
                    {
                        ESP_LOGI(TAG, "Modem CFUN Reset");
                        at_reply_wait_OK("AT+CFUN=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                        at_reply_wait_OK("AT+CFUN=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                    }

                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    if (try_counter > 4)
                    {
                        // power off
                        if (print_atcmd("AT+CPOWD=1\r\n", data) == ESP_OK)
                            ESP_LOGI(TAG, "Power DOWN");
                        break;
                    }
                    else
                        continue;
                }
                else
                {
                    ESP_LOGI(TAG, "PIN OK");
                }
            }

            strcpy(net_status_current, "Network search...");

            if (first_run_completed == false)
            {
                at_reply_wait_OK("AT+CEREG=5\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            }
            //+CEREG: 5,1,"00A0","0002920C",9,"00",0,0,"00100010","00010010"

            // Network Registration Status
            int try_network = 120;
            int tAT = 0;
            int tRT = 0;
            while (try_network > 0)
            {
                int creg[10] = {-1, -1};
                ee = at_reply_get("AT+CEREG?\r\n", "CEREG: 5", (char *)data, creg, 10, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CREG?");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    if (creg[0] != 5) // Исправится следующий раз
                    {
                        first_run_completed = false;
                    }

                    if (creg[1] == 1) // 1 Registered, home network.
                    {
                        char bf[10];
                        snprintf(bf, 9, "%8X", creg[8]);
                        tAT = fromActiveTime(strtol(bf, NULL, 2));
                        snprintf(bf, 9, "%8X", creg[9]);
                        tRT = fromPeriodicTAU(strtol(bf, NULL, 2));
                        ESP_LOGI(TAG, "Registered. Home network. TAC=%u, CI=%u Active-Time=%02d:%02d:%02d Periodic-TAU=%02d:%02d:%02d", creg[2], creg[3], (tAT / (60 * 60)), (tAT / 60) % 60, (tAT % 60), (tRT / (60 * 60)), (tRT / 60) % 60, (tRT % 60));
                        break;
                    }
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }

                try_network--;

                // если запускаем терминал - стоп работа с модулем
                if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
                {
                    break;
                }
            }

            // Signal Quality Report
            int csq[2] = {-1, -1};
            ee = at_reply_get("AT+CSQ\r\n", "CSQ:", (char *)data, csq, 2, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSQ");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                // result.measure.rssi = csq[0] * 2.0 + -113.0;
                ESP_LOGI(TAG, "RSSI: %i", csq[0] * 2 + -113);
            }

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            if (first_run_completed == false)
            {
                // AT+CURTC? AT+CTZR?
                // ee = at_reply_wait_OK("AT+CTZR=?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                at_reply_wait_OK("AT+CURTC=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS); // CCLK show UTC time after network time synchronization
                at_reply_wait_OK("AT+CTZU=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);  // Automatic time update via NITZ

                at_reply_wait_OK("AT+CPSMSTATUS=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                // #TAU 30sec * 3 , ACC 8 sec
                // at_reply_wait_OK("AT+CPSMS=1,,,\"10000011\",\"00000100\"\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                // TAU 25h 1*25, ACC 0 sec
                at_reply_wait_OK("AT+CPSMS=1,,,\"00111001\",\"00000000\"\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                first_run_completed = true;
            }

            ee = at_reply_wait_OK("AT+CCLK?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CCLK?");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                int dt[] = {0, 0, 0, 0, 0, 0, 0, 0};
                const char *pdata = strstr((const char *)data, "CCLK:");

                //+CCLK: 24/04/30,07:49:36+12
                char *s = strchr(pdata, ' ');
                if (s)
                {
                    // year
                    dt[0] = atoi(s + 1);
                    if (dt[0] >= 0 && dt[0] < 100)
                        dt[0] = dt[0] + 2000;
                    s = strchr(s + 1, '/');
                    if (s)
                    {
                        // month
                        dt[1] = atoi(s + 1);
                        s = strchr(s + 1, '/');
                        if (s)
                        {
                            // day
                            dt[2] = atoi(s + 1);
                            s = strchr(s + 1, ',');
                            if (s)
                            {
                                // hour
                                dt[3] = atoi(s + 1);
                                s = strchr(s + 1, ':');
                                if (s)
                                {
                                    // minute
                                    dt[4] = atoi(s + 1);
                                    s = strchr(s + 1, ':');
                                    if (s)
                                    {
                                        // second
                                        dt[5] = atoi(s + 1);

                                        // + - timezone
                                        dt[6] = 0;
                                        char *tzs = strchr(s + 1, '-');
                                        if (!tzs)
                                        {
                                            tzs = strchr(s + 1, '+');
                                        }

                                        if (tzs)
                                        {
                                            dt[6] = atoi(tzs);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                struct tm tm;
                tm.tm_year = dt[0] - 1900;
                tm.tm_mon = dt[1] - 1;
                tm.tm_mday = dt[2];
                tm.tm_hour = dt[3];
                tm.tm_min = dt[4];
                tm.tm_sec = dt[5];

                dt[6] = 3; // FORCE TIMEZONE

                time_t t = mktime(&tm) + dt[6] * 3600; // UNIX time + timezone offset
                struct timeval now = {.tv_sec = t};
                settimeofday(&now, NULL);
                ESP_LOGI(TAG, "Set date and time: %s", get_datetime(t));
            }

            // Show the Complete PDP Address
            ee = at_reply_wait_OK("AT+IPCONFIG\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+IPCONFIG");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                const char *pdata = strstr((const char *)data, "IPCONFIG:");
                const char *s = strchr(pdata, ' ');
                if (s)
                {
                    char *s_end = strchr(s, '\r');
                    if (s_end)
                        *s_end = 0;

                    strncpy(pdp_ip, s + 1, 18);
                };

                strncpy(pdp_ip, s + 1, 18);

                // если нет нормального IP - рестарт модуля
                if (atoi(pdp_ip) == 127)
                {
                    ESP_LOGE(TAG, "IP: %s", pdp_ip);
                    print_atcmd("AT+CPOWD=1\r\n", data);
                    first_run_completed = false;
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    continue;
                }
                else
                {
                    ESP_LOGI(TAG, "IP: %s", pdp_ip);
                }
            };

            int ip = get_menu_val_by_id("ip");
            /*
                                    // ping
                                    // AT+CIPPING

                                    snprintf(send_data, sizeof(send_data), "AT+CIPPING=\"%i.%i.%i.%i\"\r\n", (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, (ip) & 0xff);
                                    ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);
                                    if (ee != ESP_OK)
                                    {
                                        ESP_LOGW(TAG, "AT+CIPPING:%s", data);
                                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                                    }

                                    vTaskDelay(10000 / portTICK_PERIOD_MS);

                                    ee = at_reply_wait_OK("AT+CIPPING?\r\n", (char *)data, 60000 / portTICK_PERIOD_MS);
                                    if (ee != ESP_OK)
                                    {
                                        ESP_LOGW(TAG, "AT+CIPPING?:%s", data);
                                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                                    }
            */
            strcpy(net_status_current, "Send data...");

            int socket = 0;

            int tcpport = get_menu_val_by_id("tcpport");
            int udpport = get_menu_val_by_id("udpport");

            if (tcpport > 0)
            {
                protocol = 1;
                ee = at_reply_wait_OK("AT+CSOSENDFLAG=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            }
            else if (udpport > 0)
            {
                protocol = 2;
                tcpport = udpport;
            }
            else
            {
                break;
            }

            snprintf(send_data, sizeof(send_data), "AT+CSOC=1,%i,1\r\n", protocol);
            at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // Create socket

            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSOC");
            }
            else
            {
                const char *pdata = strnstr((const char *)data, "CSOC: ", 100);
                if (pdata)
                {
                    socket = atoi(pdata + 6);

                    try_counter = 3;
                    while (try_counter--)
                    {
                        snprintf(send_data, sizeof(send_data), "AT+CSOCON=%i,%i,\"%i.%i.%i.%i\"\r\n", socket, tcpport, (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, (ip) & 0xff);
                        ESP_LOGI(TAG, "%i Socket %i connect...", 3 - try_counter, socket);
                        ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);

                        if (ee == ESP_OK || ee == ESP_ERR_INVALID_STATE) // respond OK or ERROR
                        {
                            // результат измерений
                            result_t result;
                            while (pdTRUE == xQueueReceive(send_queue, &result, 5000 / portTICK_PERIOD_MS))
                            {
                                int l = snprintf(send_data, sizeof(send_data), OUT_JSON_CHANNEL, get_menu_val_by_id("idn"), result.channel, bootCount, get_datetime(result.ttime), OUT_DATA_CHANNEL(result));

                                if (common_data_transmit == false)
                                {
                                    snprintf(send_data + l - 1, sizeof(send_data) - l, OUT_JSON_ADD_NBCOMMON, cbc[1] / 1000.0, csq[0] * 2 + -113, tsens_out);
                                }

                                ESP_LOGD(TAG, "Send... %s", send_data);

                                if (protocol == 1) // TCP
                                    ee = at_csosend_wait_SEND(socket, send_data, (char *)data);

                                if (protocol == 2) // UDP
                                    ee = at_csosend(socket, send_data, (char *)data);

                                if (ee == ESP_OK)
                                {
                                    ESP_LOGI(TAG, "Send OK");
                                    strcpy(net_status_current, "Send OK");
                                }

                                if (common_data_transmit == false)
                                {
                                    // wait 10s for reply from server
                                    int64_t start_time = esp_timer_get_time();
                                    do
                                    {
                                        if (wait_string(data, "\r\n", 1000 / portTICK_PERIOD_MS) == ESP_OK)
                                        {
                                            // ESP_LOGI(TAG, "Modem: %s", data);
                                            const char *pdata = strstr((const char *)data, "+CSONMI: ");
                                            if (pdata)
                                            { //+CSONMI: 0,20,5468616E6B20796F7521
                                                char *s = strchr(pdata, ',');
                                                if (s)
                                                {
                                                    // len
                                                    int l = atoi(s + 1);
                                                    s = strchr(s + 1, ',');
                                                    if (s++)
                                                    {
                                                        // message
                                                        for (int i = 0; i < l; i = i + 2)
                                                        {
                                                            char c[3] = {*(s++), *(s++), 0};
                                                            send_data[i / 2] = (char)strtol(c, NULL, 16);
                                                        }
                                                        send_data[l / 2] = '\0';

                                                        ee = apply_command((const char *)send_data, l / 2);
                                                        break;
                                                    }
                                                }
                                            }
                                        }

                                    } while ((esp_timer_get_time() - start_time) < StoUS(10));
                                    common_data_transmit = true;
                                }
                            }
                            break;
                        }
                        else
                        {
                            ESP_LOGW(TAG, "AT+CSOCON:%s", data);
                            continue;
                        }
                    }
                }
                else
                {
                    // Скорее всего сокеты закончились...
                    socket = 4;
                }
            };

            while (protocol == 1 && socket >= 0) // TCP
            {
                snprintf(send_data, sizeof(send_data), "AT+CSOCL=%i\r\n", socket--);
                at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // CLOSE socket
            }
            break;
        }; // end while

        if (protocol == 0 && get_menu_val_by_id("pulse") != -1) // ESPNOW
        {
            xEventGroupWaitBits(
                status_event_group, /* The event group being tested. */
                END_MEASURE,        /* The bits within the event group to wait for. */
                pdFALSE,            /* BIT_0 & BIT_1 should be cleared before returning. */
                pdFALSE,
                portMAX_DELAY);

            if (xHandleWifi)
                xTaskNotify(xHandleWifi, NOTYFY_WIFI_ESPNOW, eSetValueWithOverwrite); // включаем WiFi для ESPNOW

            vTaskDelay(1000 / portTICK_PERIOD_MS);
            init_espnow(mac_addr);

            // результат измерений
            result_t result;
            while (pdTRUE == xQueueReceive(send_queue, &result, 10000 / portTICK_PERIOD_MS))
            {
                int l = snprintf(send_data, sizeof(send_data), OUT_JSON_CHANNEL, get_menu_val_by_id("idn"), result.channel, bootCount, get_datetime(result.ttime), OUT_DATA_CHANNEL(result));

                int try = 2;
                esp_err_t result;
                do
                {
                    result = esp_now_send(mac_addr, (uint8_t *)send_data, l);
                    if (result == ESP_OK || --try == 0)
                        break;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                } while (result != ESP_OK);

                if (result == ESP_OK)
                {
                    ESP_LOGI("ESPNOW", "Message sent successfully");
                }
                else
                {
                    ESP_LOGE("ESPNOW", "Error sending message: %s to " MACSTR, esp_err_to_name(result), MAC2STR(mac_addr));
                }

                // wait 1s for reply from server
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            ESP_ERROR_CHECK(esp_now_deinit());
        }

        // clear notify
        ulTaskNotifyTake(pdTRUE, 0);
        xEventGroupSetBits(status_event_group, END_RADIO);
    }
}
