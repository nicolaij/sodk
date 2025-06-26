#include "main.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_now.h"
#include "esp_mac.h"

#include <arpa/inet.h>
#include "esp_netif.h"

static const char *TAG = "terminal";

nvs_handle_t my_handle;
extern TaskHandle_t xHandleWifi;

menu_t menu[] = {
    /*0*/ {.id = "idn", .name = "Номер датчика", .izm = "", .val = 1, .min = 1, .max = 1000000},
    /*1*/ {.id = "waitwifi", .name = "Ожидание WiFi", .izm = "мин", .val = 3, .min = 1, .max = 1000000},
    /*2*/ {.id = "Trepeatlv", .name = "Интервал измер. низ.", .izm = "мин", .val = 60, .min = 1, .max = 1000000},
    /*3*/ {.id = "Trepeathv", .name = "Интервал измер. выс.", .izm = "мин", .val = 720, .min = 1, .max = 1000000},
    /*4*/ {.id = "ip", .name = "IP сервера", .izm = "", .val = ((172 << 0) | (30 << 8) | (239 << 16) | (20 << 24)), .min = INT32_MIN, .max = INT32_MAX}, // 172.30.239.20
    /*5*/ {.id = "tcpport", .name = "TCP порт сервера (0: не исп.)", .izm = "", .val = 48884, .min = 0, .max = UINT16_MAX},
    /*6*/ {.id = "udpport", .name = "UDP порт сервера (0: не исп.)", .izm = "", .val = 0, .min = 0, .max = UINT16_MAX},
    /*7*/ {.id = "MAC1", .name = "ESPNOW! Target MAC", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
    /*8*/ {.id = "MAC2", .name = "", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
    /*9*/ {.id = "pulse", .name = "Макс. время импульса", .izm = "мс", .val = 5000, .min = -1, .max = 10000},
    /*10*/ {.id = "chanord", .name = "Порядок опроса каналов", .izm = "", .val = 1234, .min = 0, .max = 999999999},
    /*11*/ {.id = "Overvolt", .name = "Ограничение U", .izm = "В", .val = 600, .min = 0, .max = 1000},
    /*12*/ {.id = "k2U", .name = "коэф. 2 U", .izm = "", .val = 0, .min = -1000000, .max = 1000000},
    /*13*/ {.id = "kU", .name = "коэф. U", .izm = "", .val = 1432, .min = 1, .max = 1000000},
    /*14*/ {.id = "offsU", .name = "смещ. U", .izm = "", .val = 0, .min = -1000000, .max = 1000000},
    /*15*/ {.id = "k2Ulv", .name = "коэф. 2 U низ.", .izm = "", .val = 0, .min = -100, .max = 100},
    /*16*/ {.id = "kUlv", .name = "коэф. U низ.", .izm = "", .val = 3784, .min = 1, .max = 1000000},
    /*17*/ {.id = "offsUlv", .name = "смещ. U низ.", .izm = "", .val = 122, .min = -1000000, .max = 1000000},
    /*18*/ {.id = "k2R1", .name = "коэф. 2 R (ch 1)", .izm = "", .val = 0, .min = -100, .max = 100},
    /*19*/ {.id = "kR1", .name = "коэф. R (ch 1)", .izm = "", .val = 8086, .min = 1, .max = 1000000},
    /*20*/ {.id = "offsR1", .name = "смещ. R (ch 1)", .izm = "", .val = -15160, .min = -1000000, .max = 1000000},
    /*21*/ {.id = "k2R2", .name = "коэф. 2 R (ch 2)", .izm = "", .val = 0, .min = -100, .max = 100},
    /*22*/ {.id = "kR2", .name = "коэф. R (ch 2)", .izm = "", .val = 30560, .min = 1, .max = 100000},
    /*23*/ {.id = "offsR2", .name = "смещ. R (ch 2)", .izm = "", .val = -239, .min = -1000000, .max = 1000000},
    /*24*/ {.id = "k2U0", .name = "коэф. 2 U петли", .izm = "", .val = 0, .min = -100, .max = 100},
    /*25*/ {.id = "kU0", .name = "коэф. U петли", .izm = "", .val = 1418, .min = 1, .max = 1000000},
    /*26*/ {.id = "offsU0", .name = "смещ. U петли", .izm = "", .val = 0, .min = -1000000, .max = 1000000},
    /*27*/ {.id = "k2U0lv", .name = "коэф. 2 U петли низ.", .izm = "", .val = 0, .min = -100, .max = 100},
    /*28*/ {.id = "kU0lv", .name = "коэф. U петли низ.", .izm = "", .val = 124535, .min = 1, .max = 1000000},
    /*29*/ {.id = "offsU0lv", .name = "смещ. U петли низ.", .izm = "", .val = 0, .min = -1000000, .max = 1000000},
    /*30*/ {.id = "k2Ubat", .name = "коэф. 2 U bat", .izm = "", .val = 0, .min = -100, .max = 100},
    /*31*/ {.id = "kUbat", .name = "коэф. U bat", .izm = "", .val = 3784, .min = 1, .max = 1000000},
    /*32*/ {.id = "offsUbat", .name = "смещ. U bat", .izm = "", .val = 122, .min = -1000000, .max = 1000000},
    /*33*/ {.id = "UbatLow", .name = "Нижн. U bat под нагр", .izm = "В", .val = 0, .min = 0, .max = 12000},
    /*34*/ {.id = "UbatEnd", .name = "U bat отключения", .izm = "В", .val = 0, .min = 0, .max = 12000},
    /*35*/ {.id = "Kfilter", .name = "Коэф. фильтрации АЦП", .izm = "", .val = 10, .min = 1, .max = 100},
    /*36*/ {.id = "WiFichan", .name = "WiFi channel", .izm = "", .val = 11, .min = 1, .max = 20},
    /*37*/ {.id = "avgcomp", .name = "Кол-во совпад. сравн.", .izm = "", .val = 25, .min = 1, .max = 10000},
    /*38*/ {.id = "avgcnt", .name = "Кол-во усред. сравн.", .izm = "", .val = 25, .min = 1, .max = 10000},
    /*39*/ {.id = "percU0lv", .name = "\% U петли низ.", .izm = "\%", .val = 75, .min = 0, .max = 100}, /*Процент от Ubatt, ниже которого - обрыв 0 провода, > - цел. 100% - не проводим высоковольные измерения от изменения*/
    /*40*/ {.id = "percRlv", .name = "\% R низ.", .izm = "\%", .val = 10, .min = 0, .max = 100},        /*Процент изменения от предыдущего значения сопротивления, ниже которого не передаем изменения*/
    /*41*/ {.id = "offstADC0", .name = "Смещение 0 ADC0", .izm = "", .val = 0, .min = 0, .max = 200},
    /*42*/ {.id = "offstADC1", .name = "Смещение 0 ADC1", .izm = "", .val = 0, .min = 0, .max = 200},
    /*43*/ {.id = "offstADC2", .name = "Смещение 0 ADC2", .izm = "", .val = 0, .min = 0, .max = 200},
    /*44*/ {.id = "offstADC3", .name = "Смещение 0 ADC3", .izm = "", .val = 0, .min = 0, .max = 200},
    /*45*/ {.id = "offstADC4", .name = "Смещение 0 ADC4", .izm = "", .val = 0, .min = 0, .max = 200},
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
    esp_err_t erro = nvs_open("storage", NVS_READONLY, &my_handle);
    if (erro != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(erro));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
        {
            esp_err_t err = nvs_get_i32(my_handle, menu[i].id, (int32_t *)&menu[i].val);
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
    return erro;
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
    esp_err_t err = ESP_FAIL;
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
        pos = sprintf(buf, "<table>");

    while (index < sizeof(menu) / sizeof(menu_t))
    {
        if (pos > CONFIG_LWIP_TCP_MSS - 256)
        {
            return pos;
        }

        if (index == 4) // IP
        {
            esp_ip4_addr_t ip_addr;
            ip_addr.addr = (unsigned int)menu[index].val;
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"" IPSTR "\"/></td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, IP2STR(&ip_addr));
        }
        else if (index == 7) // MAC
        {
            uint8_t mac_addr[6];
            mac_addr[0] = (menu[index].val >> 16) & 0xFF;
            mac_addr[1] = (menu[index].val >> 8) & 0xFF;
            mac_addr[2] = (menu[index].val >> 0) & 0xFF;
            mac_addr[3] = (menu[index + 1].val >> 16) & 0xFF;
            mac_addr[4] = (menu[index + 1].val >> 8) & 0xFF;
            mac_addr[5] = (menu[index + 1].val >> 0) & 0xFF;
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"" MACSTR "\"/></td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, MAC2STR(mac_addr));
        }
        else if (strlen(menu[index].name) > 0)
        {
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"%d\"/>%s</td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, menu[index].val, menu[index].izm);
        }
        else // hidden
        {
            pos += sprintf(&buf[pos], "<input type=\"hidden\" id=\"%s\" name=\"%s\" value=\"%d\">", menu[index].id, menu[index].id, menu[index].val);
        }

        index++;
    }

    if (pos > 0)
    {
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
    uint8_t serialbuffer[256];

    int selected_menu_id = 0;

    bool NB_terminal_mode = 0;

    bool espnow_send = false;
    esp_now_peer_info_t peerInfo = {.ifidx = WIFI_IF_AP, .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, .channel = 0, .lmk = "", .encrypt = false};

    uint8_t mac_addr[6];
    esp_ip4_addr_t ip_addr;

    char *data = (char *)serialbuffer;
    int pos = 0;

    while (1)
    {
        const int c = fgetc(stdin);
        if (c > 0) // EOF = -1
        {
            if (c == '\n')
            {
                data[pos] = 0;

                const int nc = fgetc(stdin); // remove CRLF
                if (nc != '\n' && nc != '\r')
                    ungetc(nc, stdin);
            }
            else
            {
                if (pos < sizeof(serialbuffer))
                    data[pos++] = c;
            }

            if (espnow_send)
            {
                esp_now_deinit();
                espnow_send = false;
            }

            ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
        }
        else
        {
            if (espnow_send)
            {
                esp_now_send(peerInfo.peer_addr, serialbuffer, ESP_NOW_MAX_DATA_LEN);
            }

            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        xEventGroupSetBits(status_event_group, SERIAL_TERMINAL_ACTIVE);

        if (NB_terminal_mode)
        {
            if (c == '\n')
            {
                const char cl_return = '\r';
                uart_write_bytes(UART_NUM_1, &cl_return, 1);
            }
            else
            {
                uart_write_bytes(UART_NUM_1, &c, 1);
            }

            while (uart_read_bytes(UART_NUM_1, data, 1, 50 / portTICK_PERIOD_MS) > 0)
            {
                putchar(*data);
            }
            continue;
        }

        if (c == '\n')
        {
            // ESP_LOG_BUFFER_HEXDUMP(TAG, data, pos + 1, ESP_LOG_INFO);
            // ESP_LOGD(TAG, "Read bytes: '%s'", data);
            int n = atoi((const char *)data);
            switch (selected_menu_id)
            {
            case 0:
                switch (n)
                {
                case 0: // Выводим меню
                    ESP_LOGI("menu", "-------------------------------------------");
                    int i = 0;
                    for (i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
                    {
                        if (i == 4) // IP сервера
                        {
                            ip_addr.addr = (unsigned int)menu[i].val;
                            ESP_LOGI("menu", "%2i. %s: " IPSTR, i + 1, menu[i].name, IP2STR(&ip_addr));
                        }
                        else if (i == 7) // MAC
                        {
                            mac_addr[0] = (menu[7].val >> 16) & 0xFF;
                            mac_addr[1] = (menu[7].val >> 8) & 0xFF;
                            mac_addr[2] = (menu[7].val >> 0) & 0xFF;
                            mac_addr[3] = (menu[8].val >> 16) & 0xFF;
                            mac_addr[4] = (menu[8].val >> 8) & 0xFF;
                            mac_addr[5] = (menu[8].val >> 0) & 0xFF;
                            ESP_LOGI("menu", "%2i. %s: " MACSTR, i + 1, menu[i].name, MAC2STR(mac_addr));
                        }
                        else if (strlen(menu[i].name) > 0)
                            ESP_LOGI("menu", "%2i. %s: %i %s", i + 1, menu[i].name, menu[i].val, menu[i].izm);
                    }

                    ESP_LOGI("menu", "51. История: %u", bootCount);
                    ESP_LOGI("menu", "52. AT терминал NBIoT");
                    ESP_LOGI("menu", "53. Start WiFi");
                    ESP_LOGI("menu", "54. FreeRTOS INFO");
                    ESP_LOGI("menu", "55. Reboot");
                    ESP_LOGI("menu", "100. Сброс выходов");
                    ESP_LOGI("menu", "101 - 108. Включить канал");
                    ESP_LOGI("menu", "110. Включить LV_measure");
                    ESP_LOGI("menu", "111. Включить LV_sw");
                    ESP_LOGI("menu", "112. ! Включить Enable_PWM !");
                    ESP_LOGI("menu", "113. Включить Power_ON");
                    ESP_LOGI("menu", "120. ESP-NOW transmit");
                    ESP_LOGI("menu", "200. Измерение ВВ без подключ. канала");
                    ESP_LOGI("menu", "201 - 208. Измерение канала 1 - 8");
                    ESP_LOGI("menu", "209. Измерение c POWER_ON и подключ. канала");
                    ESP_LOGI("menu", "210. Измерение без ВВ и подключ. канала");
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 5: // IP сервера
                    ip_addr.addr = (unsigned int)menu[n - 1].val;
                    ESP_LOGI("menu", "-------------------------------------------");
                    ESP_LOGI("menu", "%2i. %s: " IPSTR ". Введите новое значение: ", n, menu[n - 1].name, IP2STR(&ip_addr));
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 8: // MAC Address
                    mac_addr[0] = (menu[7].val >> 16) & 0xFF;
                    mac_addr[1] = (menu[7].val >> 8) & 0xFF;
                    mac_addr[2] = (menu[7].val >> 0) & 0xFF;
                    mac_addr[3] = (menu[8].val >> 16) & 0xFF;
                    mac_addr[4] = (menu[8].val >> 8) & 0xFF;
                    mac_addr[5] = (menu[8].val >> 0) & 0xFF;

                    ESP_LOGI("menu", "-------------------------------------------");
                    ESP_LOGI("menu", "%2i. %s: " MACSTR ". Введите новое значение: ", n, menu[n - 1].name, MAC2STR(mac_addr));
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 51: // выводим историю
                    break;
                case 52: // AT терминал NBIoT
                    NB_terminal_mode = 1;
                    xEventGroupSetBits(status_event_group, NB_TERMINAL);
                    if (xHandleNB)
                        xTaskNotifyGive(xHandleNB); // если уже уснули
                    break;
                case 53: // WiFi
                    if (xHandleWifi)
                        xTaskNotifyGive(xHandleWifi); // включаем WiFi
                    break;
                case 54: // FreeRTOS INFO
                    ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
                    ESP_LOGI("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
                    ESP_LOGI("adc_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
                    ESP_LOGI("modem_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleNB));
                    ESP_LOGI("console_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleConsole));
                    /*
                                        char statsbuf[600];
                                        vTaskGetRunTimeStats(statsbuf);
                                        printf(statsbuf);
                    */
                    break;
                case 55: // Reboot
                    if (xHandleWifi)
                        xTaskNotify(xHandleWifi, NOTYFY_WIFI_REBOOT, eSetValueWithOverwrite);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    esp_restart();
                    break;
                case 100:
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 1));
                    pcf8575_set(0);
                    break;
                case 101: // Тест каналов
                    pcf8575_set(1);
                    break;
                case 102:
                    pcf8575_set(2);
                    break;
                case 103:
                    pcf8575_set(3);
                    break;
                case 104:
                    pcf8575_set(4);
                    break;
                case 105:
                    pcf8575_set(5);
                    break;
                case 106:
                    pcf8575_set(6);
                    break;
                case 107:
                    pcf8575_set(7);
                    break;
                case 108:
                    pcf8575_set(8);
                    break;
                case 110: // Включить LV_measure
                    pcf8575_set(LV_MEASUREON);
                    break;
                case 111: // Включить LV_sw (LV_POWER)
                    pcf8575_set(LV_CMDON);
                    break;
                case 112: // Включить Enable_PWM
                    ESP_ERROR_CHECK(gpio_set_level(ENABLE_PIN, 0));
                    break;
                case 113: // Включить Power_ON
                    pcf8575_set(POWER_CMDON);
                    break;
                case 120: // Непрерывная передача ESP-NOW
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
                    break;
                case 200: // Измерения каналов c ВВ
                    start_measure(-1, 1);
                    break;
                case 201: // Измерения каналов
                    start_measure(1, 0);
                    break;
                case 202: // Измерения каналов
                    start_measure(2, 0);
                    break;
                case 203: // Измерения каналов
                    start_measure(3, 0);
                    break;
                case 204: // Измерения каналов
                    start_measure(4, 0);
                    break;
                case 205: // Измерения каналов
                    start_measure(5, 0);
                    break;
                case 206: // Измерения каналов
                    start_measure(6, 0);
                    break;
                case 207: // Измерения каналов
                    start_measure(7, 0);
                    break;
                case 208: // Измерения каналов
                    start_measure(8, 0);
                    break;
                case 209: // Измерения каналов с POWER ON
                    start_measure(-1, 5);
                    break;
                case 210: // Измерение без ВВ и подключ. канала
                    start_measure(-1, 4);
                    break;
                default:
                    if (n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %i %s. Введите новое значение: ", n, menu[n - 1].name, menu[n - 1].val, menu[n - 1].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                    }
                    break;
                }
                break;
            case 5: // IP сервера
                if (sscanf((const char *)serialbuffer, "%hhu.%hhu.%hhu.%hhu", &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3]) == 4)
                {
                    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                    if (err == ESP_OK)
                    {
                        esp_ip4_addr_t ip_addr;
                        ip_addr.addr = (mac_addr[0] << 0) | (mac_addr[1] << 8) | (mac_addr[2] << 16) | (mac_addr[3] << 24);
                        ESP_LOGD("NVS", "Write  \"%s\" : \"" IPSTR "\"", menu[4].id, IP2STR(&ip_addr));
                        menu[4].val = (int)ip_addr.addr;
                        nvs_set_i32(my_handle, menu[4].id, menu[4].val);
                        nvs_close(my_handle);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Error IP address format");
                }
                break;
            case 8: // MAC ESPNOW!
                if (sscanf((const char *)serialbuffer, "%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx",
                           &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4], &mac_addr[5]) == 6)
                {
                    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                    if (err == ESP_OK)
                    {
                        ESP_LOGD("NVS", "Write  \"%s\" : \"" MACSTR "\"", menu[7].id, MAC2STR(mac_addr));
                        menu[7].val = (mac_addr[0] << 16) | (mac_addr[1] << 8) | (mac_addr[2]);
                        nvs_set_i32(my_handle, menu[7].id, menu[7].val);
                        menu[8].val = (mac_addr[3] << 16) | (mac_addr[4] << 8) | (mac_addr[5]);
                        nvs_set_i32(my_handle, menu[8].id, menu[8].val);
                        nvs_close(my_handle);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Error MAC format");
                }
                break;
            default:
                if (selected_menu_id > 0 && selected_menu_id <= sizeof(menu) / sizeof(menu_t)) // selected_menu_id - номер пункта меню, n - value
                {
                    if (n >= menu[selected_menu_id - 1].min && n <= menu[selected_menu_id - 1].max)
                    {
                        menu[selected_menu_id - 1].val = n;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
                        }
                        else
                        {
                            err = nvs_set_i32(my_handle, menu[selected_menu_id - 1].id, menu[selected_menu_id - 1].val);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                            }
                            else
                            {
                                ESP_LOGI("menu", "-------------------------------------------");
                                ESP_LOGI("menu", "%2i. %s: %i %s.", selected_menu_id, menu[selected_menu_id - 1].name, menu[selected_menu_id - 1].val, menu[selected_menu_id - 1].izm);
                                ESP_LOGI("menu", "-------------------------------------------");
                            }
                        }

                        ESP_LOGD(TAG, "Committing updates in NVS ... ");
                        err = nvs_commit(my_handle);
                        if (err != ESP_OK)
                            ESP_LOGE(TAG, "Committing updates in NVS ... - Failed!");

                        // Close
                        nvs_close(my_handle);
                    }
                }
                break;
            }

            if (selected_menu_id == 0 && n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                selected_menu_id = n;
            else
                selected_menu_id = 0;

            pos = 0;
        } // if (c == '\n')
        vTaskDelay(1);
    }
}
