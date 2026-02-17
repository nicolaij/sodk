#include "NBIoTlib.h"

#include "esp_wifi.h"
#include "esp_timer.h"

static const char *TAG = "NBIoT";
esp_ip4_addr_t pdp_ip;
char net_status_current[32];

RTC_DATA_ATTR bool first_run_completed = false;

RTC_DATA_ATTR time_t last_sync_time = 0;

extern int32_t timezone;

void modem_task(void *arg)
{
    char data[RX_BUF_SIZE];
    char send_data[TX_BUF_SIZE];
    time_t realtime = 0;

    bool cpsms0 = false;

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT64(MODEM_POWER);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(MODEM_POWER, 1));

    ESP_ERROR_CHECK(uart_init(UART_NUM_1));

    int try_counter = 0;

    strcpy(net_status_current, "OFF");
    bool common_data_transmit = false;

    int protocol = 1; // ESPNOW = 0, TCP = 1, UDP =2

    const int mac1 = get_menu_val_by_id("MAC1");
    const int mac2 = get_menu_val_by_id("MAC2");
    uint8_t mac_addr[6] = {
        (mac1 >> 16) & 0xFF,
        (mac1 >> 8) & 0xFF,
        (mac1 >> 0) & 0xFF,
        (mac2 >> 16) & 0xFF,
        (mac2 >> 8) & 0xFF,
        (mac2 >> 0) & 0xFF,
    };

    if (mac1 > 0 || mac2 > 0)
    {
        protocol = 0;
    }

    const int pulse = get_menu_val_by_id("pulse");

    xEventGroupSetBits(status_event_group, END_RADIO);

    while (1)
    {
        /* Ждем необходимости запуска передачи*/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления беcконечно, для повторного опроса

        xEventGroupClearBits(status_event_group, END_RADIO);

        bool cpin = false;
        int d_nbiot_error_counter = 0;

        while (protocol > 0) // повторы опроса модуля
        {
            esp_err_t ee = 0;

            bool network_registration = false;

            int tAT = 0;
            int tRT = 0;

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

            if (check_cereg((const char *)data) == ESP_OK)
            {
                network_registration = true;
            }

            if (strstr((const char *)data, "CPIN: READY") != NULL)
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

                /*                if (first_run_completed == false)
                                    at_reply_wait_OK("ATE1;+IPR=115200\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                */
            };

            if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT))
                break;

            strcpy(net_status_current, "ready");

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
                ee = at_reply_wait_OK("AT+CPIN?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK || strstr(data, "CPIN: READY") == NULL)
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
            if (network_registration == false)
            {
                while (try_network > 0)
                {
                    at_reply_wait_OK("AT+CEREG?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                    // ee = at_reply_get("AT+CEREG?\r\n", "CEREG: 5", (char *)data, creg, 10, 1000 / portTICK_PERIOD_MS);
                    if (check_cereg(data) == ESP_OK)
                    {

                        if (creg[0] != 5) // Исправится следующий раз
                        {
                            first_run_completed = false;
                        }

                        if (creg[1] == 1) // 1 Registered, home network.
                        {
                            network_registration = true;
                            break;
                        }
                    }

                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    try_network--;

                    // если запускаем терминал - стоп работа с модулем
                    if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
                    {
                        // Disable the use of PSM and discard all parameters for PSM or, if available reset to the manufacturer specific default values.
                        print_atcmd("AT+CPSMS=2\r\n", data);
                        cpsms0 = true;
                        break;
                    }
                }
            }

            if (network_registration)
            {
                char bf[10];
                snprintf(bf, 9, "%8X", creg[8]);
                tAT = fromActiveTime(strtol(bf, NULL, 2));
                snprintf(bf, 9, "%8X", creg[9]);
                tRT = fromPeriodicTAU(strtol(bf, NULL, 2));
                ESP_LOGI(TAG, "Registered. Home network. TAC=%u, CI=%u Active-Time=%02d:%02d:%02d Periodic-TAU=%02d:%02d:%02d", creg[2], creg[3], (tAT / (60 * 60)), (tAT / 60) % 60, (tAT % 60), (tRT / (60 * 60)), (tRT / 60) % 60, (tRT % 60));
            }

            // Signal Quality Report
            int csq[2] = {-1, -1};
            ee = at_reply_get("AT+CSQ\r\n", "+CSQ: ", (char *)data, csq, 2, 1000 / portTICK_PERIOD_MS);
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
                // Disable the use of PSM and discard all parameters for PSM or, if available reset to the manufacturer specific default values.
                print_atcmd("AT+CPSMS=2\r\n", data);
                cpsms0 = true;
                break;
            }

            if (first_run_completed == false || cpsms0)
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
                cpsms0 = false;

                at_reply_wait_OK("AT+CSOSENDFLAG=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                first_run_completed = true;
            }

            // sync time 1 / 24 hours
            if (last_sync_time == 0 || (time(0) - last_sync_time) > (24 * 60 * 60))
            {
                ee = at_reply_wait_OK("AT+CCLK?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CCLK?");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    //+CCLK: 24/04/30,07:49:36+12
                    int dt[6] = {0, 0, 0, 0, 0, 0};
                    const char *pdata = strstr((const char *)data, "CCLK: ");
                    int parsed = sscanf((const char *)(pdata + 6), "%d/%d/%d,%d:%d:%d", &dt[0], &dt[1], &dt[2], &dt[3], &dt[4], &dt[5]);
                    if (parsed == 6)
                    {
                        struct tm tm;
                        tm.tm_year = (dt[0] > 1900) ? dt[0] - 1900 : dt[0] + 100;
                        tm.tm_mon = dt[1] - 1;
                        tm.tm_mday = dt[2];
                        tm.tm_hour = dt[3];
                        tm.tm_min = dt[4];
                        tm.tm_sec = dt[5];

                        last_sync_time = mktime(&tm) + timezone * 3600; // UNIX time + timezone offset
                        struct timeval now = {.tv_sec = last_sync_time};
                        settimeofday(&now, NULL);
                        ESP_LOGI(TAG, "Set date and time: %s", get_datetime(last_sync_time));
                    }
                }
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
                const char *s = strstr((const char *)data, "IPCONFIG: ");
                int parsed = sscanf((const char *)(s + 10), "%hhu.%hhu.%hhu.%hhu", &((uint8_t *)(&pdp_ip.addr))[0], &((uint8_t *)(&pdp_ip.addr))[1], &((uint8_t *)(&pdp_ip.addr))[2], &((uint8_t *)(&pdp_ip.addr))[3]);
                if (parsed == 4)
                {
                    // если нет нормального IP - рестарт модуля
                    if (esp_ip4_addr1(&pdp_ip) == 127)
                    {
                        ESP_LOGE(TAG, "IP: " IPSTR, IP2STR(&pdp_ip));
                        print_atcmd("AT+CPOWD=1\r\n", data);
                        first_run_completed = false;
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        continue;
                    }
                    else
                    {
                        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&pdp_ip));
                    }
                }
            };

            esp_ip4_addr_t ip;
            ip.addr = (unsigned int)get_menu_val_by_id("ip");

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
            const int tcpport = get_menu_val_by_id("tcpport");
            const int udpport = get_menu_val_by_id("udpport");

            int port = tcpport;

            if (tcpport > 0)
            {
                protocol = 1;
            }

            if (udpport > 0)
            {
                port = udpport;
                protocol = 2;
            }

            if (tcpport == 0 && udpport == 0)
            {
                break;
            }

            snprintf(send_data, sizeof(send_data), "AT+CSOC=1,%i,1\r\n", protocol);
            ee = at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // Create socket

            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSOC");
            }
            else
            {
                const char *pdata = strstr((const char *)data, "CSOC: ");
                if (pdata)
                {
                    socket = atoi(pdata + 6);

                    try_counter = 3;
                    while (try_counter--)
                    {
                        snprintf(send_data, sizeof(send_data), "AT+CSOCON=%i,%i,\"" IPSTR "\"\r\n", socket, port, IP2STR(&ip));
                        ESP_LOGI(TAG, "%i Socket %i connect...", 3 - try_counter, socket);
                        ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);

                        if (ee != ESP_OK)
                        {
                            ESP_LOGW(TAG, "AT+CSOCON:%s", data);
                        }
                        else
                        {
                            // результат измерений
                            result_t result;
                            if (realtime < 1754900000LL)
                                realtime = time(0);

                            int64_t start_time = esp_timer_get_time();
                            do
                            {
                                while (pdTRUE == xQueueReceive(send_queue, &result, 0))
                                {
                                    // ESP_LOGD(TAG, "time: %lli", result.ttime);
                                    if (result.ttime < 1754900000LL) // 2025-08-11
                                    {
                                        result.ttime = realtime;
                                    }

                                    int l = snprintf(send_data, sizeof(send_data), "{" OUT_CHANNEL, get_menu_val_by_id("idn"), result.channel, bootCount, get_datetime(result.ttime), OUT_DATA_CHANNEL(result));

                                    if (common_data_transmit == false)
                                    {
                                        l += snprintf(send_data + l, sizeof(send_data) - l, OUT_ADD_NBCOMMON, cbc[1] / 1000.0, csq[0] * 2 + -113, creg[2], creg[3]);
                                        l += snprintf(send_data + l, sizeof(send_data) - l, OUT_ADD_COMMON, tsens_out);
                                    }

                                    l += snprintf(send_data + l, sizeof(send_data) - l, "}");

                                    do
                                    {
                                        ESP_LOGD(TAG, "Send... %s", send_data);

                                        if (protocol == 1) // TCP
                                            ee = at_csosend_wait_SEND(socket, send_data, (char *)data, l);
                                        if (protocol == 2) // UDP
                                            ee = at_csosend(socket, send_data, (char *)data, l);

                                        if (ee == ESP_OK)
                                        {
                                            common_data_transmit = true;
                                            strcpy(net_status_current, "Send OK");
                                            ESP_LOGI(TAG, "Send OK");

                                            // обрабатываем если нет сети или приход команды
                                            ee = check_received_message(data, send_data);
                                        }

                                    } while (ee == ESP_ERR_NOT_FINISHED);

                                    start_time = esp_timer_get_time(); // reset timer
                                }
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                check_received_message(data, send_data);
                            } while ((esp_timer_get_time() - start_time) < (pulse * 1000LL));
                            break;
                        }
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
                else
                {
                    // Скорее всего сокеты закончились...
                    socket = 4;
                }
            };

            // wait 5s for reply from server
            while (wait_string(data, "\r\n", 10000 / portTICK_PERIOD_MS) == ESP_OK)
            {
                ESP_LOGI(TAG, "Modem: %s", data);
                check_received_message(data, send_data);
            }

            while (socket >= 0)
            {
                snprintf(send_data, sizeof(send_data), "AT+CSOCL=%i\r\n", socket--);
                at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // CLOSE socket
            }

            download_firmware(data, send_data);

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                // Disable the use of PSM and discard all parameters for PSM or, if available reset to the manufacturer specific default values.
                print_atcmd("AT+CPSMS=2\r\n", data);
                cpsms0 = true;
            }
            break;
        }; // end while

        if (protocol == 0 && pulse != -1) // ESPNOW
        {
            // результат измерений
            result_t result;
            while (pdTRUE == xQueueReceive(send_queue, &result, 10000 / portTICK_PERIOD_MS))
            {
                int l = snprintf(send_data, sizeof(send_data), "{" OUT_CHANNEL, get_menu_val_by_id("idn"), result.channel, bootCount, get_datetime(result.ttime), OUT_DATA_CHANNEL(result));

                if (common_data_transmit == false)
                {
                    l += snprintf(send_data + l, sizeof(send_data) - l, OUT_ADD_COMMON, tsens_out);

                    common_data_transmit = true;
                }
                l += snprintf(send_data + l, sizeof(send_data) - l, "}");

                int try = 2;
                esp_err_t err;
                do
                {
                    err = send_by_espnow(mac_addr, send_data);

                    if (err == ESP_OK || --try == 0)
                        break;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                } while (err != ESP_OK);

                // wait 1s for reply from server
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        // clear notify
        ulTaskNotifyTake(pdTRUE, 0);
        xEventGroupSetBits(status_event_group, END_RADIO);
    }
}
