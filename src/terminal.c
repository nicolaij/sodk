#include "main.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_now.h"

uint8_t serialbuffer[256];

static const char *TAG = "terminal";

nvs_handle_t my_handle;
extern TaskHandle_t xHandleWifi;

menu_t menu[] = {
    {.id = "idn", .name = "Номер датчика", .izm = "", .val = 1, .min = 1, .max = 1000000},
    {.id = "waitwifi", .name = "Ожидание WiFi", .izm = "мин", .val = 3, .min = 1, .max = 1000000},
    {.id = "Trepeatlv", .name = "Интервал измер. низ.", .izm = "мин", .val = 60, .min = 1, .max = 1000000},
    {.id = "Trepeathv", .name = "Интервал измер. выс.", .izm = "мин", .val = 720, .min = 1, .max = 1000000},
    {.id = "ip", .name = "IP сервера", .izm = "", .val = ((172 << 24) | (30 << 16) | (239 << 8) | (20)), .min = INT32_MIN, .max = INT32_MAX}, // 172.30.239.20
    {.id = "tcpport", .name = "TCP порт сервера (0: не исп.)", .izm = "", .val = 48884, .min = 0, .max = UINT16_MAX},
    {.id = "udpport", .name = "UDP порт сервера (0: не исп.)", .izm = "", .val = 0, .min = 0, .max = UINT16_MAX},
    {.id = "MAC1", .name = "ESPNOW! Target MAC[0,1,2]", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
    {.id = "MAC2", .name = "ESPNOW! Target MAC[3,4,5]", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
    {.id = "pulse", .name = "Макс. время импульса", .izm = "мс", .val = 5000, .min = -1, .max = 10000},
    {.id = "chanord", .name = "Порядок опроса каналов", .izm = "", .val = 1234, .min = 0, .max = 999999999},
    {.id = "Overvolt", .name = "Ограничение U", .izm = "В", .val = 600, .min = 0, .max = 1000},
    {.id = "kU", .name = "коэф. U", .izm = "", .val = 228273, .min = 1, .max = 1000000},
    {.id = "offsU", .name = "смещ. U", .izm = "", .val = 0, .min = -1000000, .max = 1000000},
    {.id = "kUlv", .name = "коэф. U низ.", .izm = "", .val = 6909, .min = 1, .max = 1000000},
    {.id = "offsUlv", .name = "смещ. U низ.", .izm = "", .val = 0, .min = -100000, .max = 100000},
    {.id = "kR1", .name = "коэф. R (ch 1)", .izm = "", .val = 15000, .min = 1, .max = 1000000},
    {.id = "offsAR1", .name = "смещ. R (ch 1)", .izm = "", .val = 0, .min = -100000, .max = 100000},
    {.id = "kR2", .name = "коэф. R (ch 2)", .izm = "", .val = 319, .min = 1, .max = 100000},
    {.id = "offsAR2", .name = "смещ. R (ch 2)", .izm = "", .val = 0, .min = -100000, .max = 100000},
    {.id = "kU0", .name = "коэф. U петли", .izm = "", .val = 228273, .min = 1, .max = 1000000},
    {.id = "offsU0", .name = "смещ. U петли", .izm = "", .val = 0, .min = -100000, .max = 100000},
    {.id = "kU0lv", .name = "коэф. U петли низ.", .izm = "", .val = 228273, .min = 1, .max = 1000000},
    {.id = "offsU0lv", .name = "смещ. U петли низ.", .izm = "", .val = 0, .min = -100000, .max = 100000},
    {.id = "kUbat", .name = "коэф. U bat", .izm = "", .val = 7060, .min = 1, .max = 100000},
    {.id = "offsUbat", .name = "смещ. U bat", .izm = "", .val = 0, .min = -1000000, .max = 1000000},
    {.id = "UbatLow", .name = "Нижн. U bat под нагр", .izm = "В", .val = 0, .min = 0, .max = 12000},
    {.id = "UbatEnd", .name = "U bat отключения", .izm = "В", .val = 0, .min = 0, .max = 12000},
    {.id = "Kfilter", .name = "Коэф. фильтрации АЦП", .izm = "", .val = 10, .min = 1, .max = 100},
    {.id = "WiFichan", .name = "WiFi channel", .izm = "", .val = 11, .min = 1, .max = 20},
    {.id = "avgcomp", .name = "Кол-во совпад. сравн.", .izm = "", .val = 25, .min = 1, .max = 10000},
    {.id = "avgcnt", .name = "Кол-во усред. сравн.", .izm = "", .val = 25, .min = 1, .max = 10000},
    {.id = "percU0lv", .name = "\% U петли низ.", .izm = "\%", .val = 75, .min = 0, .max = 100}, /*Процент от Ubatt, ниже которого - обрыв 0 провода, > - цел. 100% - не проводим высоковольные измерения от изменения*/
    {.id = "percRlv", .name = "\% R низ.", .izm = "\%", .val = 10, .min = 0, .max = 100},        /*Процент изменения от предыдущего значения сопротивления, ниже которого не передаем изменения*/
};

esp_err_t init_nvs()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // Example of nvs_get_stats() to get the number of used entries and free entries:
    nvs_stats_t nvs_stats;
    nvs_get_stats(NULL, &nvs_stats);
    ESP_LOGD("NVS", "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
    return err;
}

esp_err_t read_nvs_menu()
{
    // Open
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
        {
            err = nvs_get_i32(my_handle, menu[i].id, (int32_t *)&menu[i].val);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGD("NVS", "Read \"%s\" = %i", menu[i].name, menu[i].val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", menu[i].name);
                break;
            default:
                ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
            }
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}

esp_err_t read_nvs_id(const char *key, uint64_t *out_value)
{
    // Open
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_u64(my_handle, key, out_value);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD("NVS", "Read \"%s\" = %016llX", key, *out_value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", key);
            break;
        default:
            ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}

int get_menu_pos_by_id(const char *id)
{
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
            return i;
    }
    return -1;
}

int get_menu_val_by_id(const char *id)
{
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
            return menu[i].val;
    }
    return 0;
}

esp_err_t set_menu_val_by_id(const char *id, int value)
{
    esp_err_t err = ESP_OK;
    int ll = strlen(id);

    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
        {
            if (menu[i].val != value)
            {
                err = nvs_open("storage", NVS_READWRITE, &my_handle);

                ESP_LOGD("NVS", "Write  \"%s\" : \"%i\"", menu[i].id, value);
                err = nvs_set_i32(my_handle, id, value);
                menu[i].val = value;
                nvs_close(my_handle);
            }
            break;
        }
    }

    return err;
}

int get_menu_json(char *buf)
{
    int pos = 0;
    buf[pos++] = '{';
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        pos += sprintf(&buf[pos], "\"%s\":[\"%s\",%i,\"%s\"]", menu[i].id, menu[i].name, menu[i].val, menu[i].izm);
        if (i < sizeof(menu) / sizeof(menu_t) - 1)
            buf[pos++] = ',';
        else
            buf[pos++] = '}';

        buf[pos] = '\0';
    }
    return pos;
}

int get_menu_html(char *buf)
{
    int pos = 0;
    static int index = 0;

    if (index == 0)
        pos += sprintf(&buf[pos], "<table>");

    for (int i = index; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        if (strlen(menu[i].name) > 0)
        {
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"%i\"/>%s</td></tr>\n", menu[i].id, menu[i].name, menu[i].id, menu[i].id, menu[i].val, menu[i].izm);
        }
        else
        {
            pos += sprintf(&buf[pos], "<input type=\"hidden\" id=\"%s\" name=\"%s\" value=\"%i\">", menu[i].id, menu[i].id, menu[i].val);
        }

        if (pos > CONFIG_LWIP_TCP_MSS - 256)
        {
            index = i + 1;
            return pos;
        }
    }

    if (pos > 0)
    {
        index = sizeof(menu) / sizeof(menu_t);
        pos += sprintf(&buf[pos], "</table><br>");
    }
    else
    {
        index = 0;
    }

    return pos;
}

void console_task(void *arg)
{
    uint8_t *data = serialbuffer;

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, sizeof(serialbuffer), 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_flush(UART_NUM_0));

    int enter_value = 0;

    bool NB_terminal_mode = 0;

    bool espnow_send = false;
    esp_now_peer_info_t peerInfo = {.ifidx = WIFI_IF_AP, .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, .channel = 0, .lmk = "", .encrypt = false};

    int n = 0;

    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, 1, 50 / portTICK_PERIOD_MS);

        // if (rxBytes > 0)
        // ESP_LOGE(TAG, "%c(%02x)", *data, *data);

        if (NB_terminal_mode)
        {
            if (rxBytes > 0)
            {
                uart_write_bytes(UART_NUM_1, data, rxBytes);
                // ESP_LOGE(TAG, "%c(%02x)", *data, *data);
                // print_atcmd("ATI", (char*)data);
                if (data[rxBytes - 1] == '\n')
                {
                    xEventGroupSetBits(status_event_group, SERIAL_TERMINAL_ACTIVE);
                }
            }

            while (uart_read_bytes(UART_NUM_1, data, 1, 50 / portTICK_PERIOD_MS) > 0)
            {
                putchar(*data);
            }
            continue;
        }

        if (rxBytes > 0)
        {
            if (data[rxBytes - 1] == '\n')
            {
                xEventGroupSetBits(status_event_group, SERIAL_TERMINAL_ACTIVE);

                if (data[rxBytes - 2] == '\r')
                {
                    data[rxBytes - 2] = 0;
                };

                data[rxBytes - 1] = 0;
                ESP_LOGD(TAG, "Read bytes: '%s'", serialbuffer);
                // ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes, ESP_LOG_INFO);
                data = serialbuffer;
                n = atoi((const char *)data);
                if (enter_value > 0)
                {
                    if (n >= menu[enter_value - 1].min && n <= menu[enter_value - 1].max)
                    {
                        menu[enter_value - 1].val = n;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
                        }
                        else
                        {
                            err = nvs_set_i32(my_handle, menu[enter_value - 1].id, menu[enter_value - 1].val);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                            }
                            else
                            {
                                ESP_LOGI("menu", "-------------------------------------------");
                                ESP_LOGI("menu", "%2i. %s: %i %s.", enter_value, menu[enter_value - 1].name, menu[enter_value - 1].val, menu[enter_value - 1].izm);
                                ESP_LOGI("menu", "-------------------------------------------");
                            }
                        }

                        // Commit written value.
                        // After setting any values, nvs_commit() must be called to ensure changes are written
                        // to flash storage. Implementations may write to storage at other times,
                        // but this is not guaranteed.
                        ESP_LOGD(TAG, "Committing updates in NVS ... ");
                        err = nvs_commit(my_handle);
                        if (err != ESP_OK)
                            ESP_LOGE(TAG, "Committing updates in NVS ... - Failed!");

                        // Close
                        nvs_close(my_handle);
                    }
                    enter_value = 0;
                }
                else
                {
                    if (n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %i %s. Введите новое значение: ", n, menu[n - 1].name, menu[n - 1].val, menu[n - 1].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                        enter_value = n;
                    }
                    else if (n == sizeof(menu) / sizeof(menu_t) + 1) // выводим историю
                    {
                        /*                        int pos = history_pos + HISTORY_SIZE;
                                                int end = history_pos;
                                                ESP_LOGI("menu", "-------------------------------------------");
                                                ESP_LOGI("menu", "Datetime, Bootcount, " OUT_MEASURE_HEADERS);
                                                while (pos > end)
                                                {
                                                    int indx = pos % HISTORY_SIZE;

                                                    ESP_LOGI("menu", "%s, %3u, " OUT_MEASURE_FORMATS, get_datetime(history[indx].ttime), history[indx].measure.bootcount, OUT_MEASURE_VARS(history[indx].measure));
                                                    pos--;
                                                }

                                                ESP_LOGI("menu", "-------------------------------------------");
                                                enter_value = 0;
                        */
                    }
                    else if (n == sizeof(menu) / sizeof(menu_t) + 2) // AT терминал NBIoT
                    {
                        NB_terminal_mode = 1;
                        xEventGroupSetBits(status_event_group, NB_TERMINAL);
                        if (xHandleNB)
                            xTaskNotifyGive(xHandleNB); // если уже уснули
                        // vTaskSuspend(xHandleNB); // Suspend NBIot task
                        // wait_max_counter = 3;
                        enter_value = 0;
                    }else if (n == sizeof(menu) / sizeof(menu_t) + 3) // WiFi
                    {
                        if (xHandleWifi)
                            xTaskNotifyGive(xHandleWifi); // включаем WiFi
                        enter_value = 0;
                    }
                    else if (n == 100)
                    {
                        pcf8575_set(0);
                        ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                        enter_value = 0;
                    }
                    else if (n > 100 && n <= 108) // Тест каналов
                    {
                        pcf8575_set(n - 100);
                        enter_value = 0;
                    }
                    else if (n == 110) // Включить LV_measure
                    {
                        pcf8575_set(LV_MEASUREON);
                        enter_value = 0;
                    }
                    else if (n == 111) // Включить LV_sw (LV_POWER)
                    {
                        pcf8575_set(LV_CMDON);
                        enter_value = 0;
                    }
                    else if (n == 112) // Включить Enable_PWM
                    {
                        ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                        enter_value = 0;
                    }
                    else if (n == 120) // Непрерывная передача ESP-NOW
                    {
                        if (esp_now_init() == ESP_OK)
                        {
                            // ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
                            esp_now_add_peer(&peerInfo);
                            esp_err_t result = esp_now_send(peerInfo.peer_addr, serialbuffer, ESP_NOW_MAX_DATA_LEN);
                            if (result == ESP_OK)
                            {
                                ESP_LOGI("ESPNOW", "Start sending espnow..");
                                espnow_send = true;
                            }
                            else
                            {
                                ESP_LOGE("ESPNOW", "esp_now_send status 0x%04x", result);
                            }
                        }
                        enter_value = 0;
                    }
                    else if (n > 200 && n <= 208) // Измерения каналов
                    {
                        start_measure(n - 200, 1);
                        enter_value = 0;
                    }
                    else
                    {

                        // ESP_LOGI("result", OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, get_datetime(result.ttime), OUT_MEASURE_VARS(result.measure));

                        ESP_LOGI("menu", "-------------------------------------------");
                        int i = 0;
                        for (i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
                        {
                            if (strlen(menu[i].name) == 0)
                                ESP_LOGI("menu", "%2i. %s: %i %s", i + 1, menu[i].id, menu[i].val, menu[i].izm);
                            else
                                ESP_LOGI("menu", "%2i. %s: %i %s", i + 1, menu[i].name, menu[i].val, menu[i].izm);
                        }
                        ESP_LOGI("menu", "%2i. История: %i", ++i, bootCount);
                        ESP_LOGI("menu", "%2i. AT терминал NBIoT", ++i);
                        ESP_LOGI("menu", "%2i. Start WiFi", ++i);
                        ESP_LOGI("menu", "100. Сброс выходов");
                        ESP_LOGI("menu", "101 - 108. Включить канал");
                        ESP_LOGI("menu", "110. Включить LV_measure");
                        ESP_LOGI("menu", "111. Включить LV_sw");
                        ESP_LOGI("menu", "112. Включить Enable_PWM");
                        ESP_LOGI("menu", "120. ESP-NOW transmit");
                        ESP_LOGI("menu", "201 - 208. Измерение канала 1 - 8");
                        ESP_LOGI("menu", "-------------------------------------------");
                        enter_value = 0;
                    }
                }
            }
            else
            {
                data = data + rxBytes;
                if (data >= serialbuffer + sizeof(serialbuffer))
                    data = serialbuffer;

                if (espnow_send)
                {
                    esp_now_deinit();
                    espnow_send = false;
                }

                if (n > 100)
                {
                    //pcf8575_set(0);
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                }
            }
        }

        if (espnow_send)
        {
            esp_now_send(peerInfo.peer_addr, serialbuffer, ESP_NOW_MAX_DATA_LEN);
        }
    }
}
