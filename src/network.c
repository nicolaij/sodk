/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "main.h"

#include "esp_partition.h"
#include <esp_ota_ops.h>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <esp_http_server.h>

#include "esp_spiffs.h"

#include "driver/uart.h"

extern uint8_t mac[6];
extern char modem_status[128];

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define CLIENT_WIFI_SSID "Nadtocheeva 5"
#define CLIENT_WIFI_PASS "123123123"
#define AP_WIFI_SSID "MegaOm"
#define AP_WIFI_PASS "123123123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 0

#define EXAMPLE_MAX_STA_CONN 5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define PAGE_LORA_SET "/"

#ifdef NB
extern uint16_t port;
extern char apn[32];
extern char serverip[17];
extern int32_t proto;
extern int32_t timezone;
#else
extern int fr;
extern int bw;
extern int sf;
extern int op;
#endif

extern int32_t id;

static const char *TAG = "wifi";

static const char *TAGH = "httpd";

static int s_retry_num = 0;

static char buf[CONFIG_LWIP_TCP_MSS];
size_t buf_len;

int64_t timeout_begin;

bool need_ws_send = false;

bool restart = false;

typedef struct
{
    char filepath[32];
    char content[32];
} down_data_t;

void reset_sleep_timeout()
{
    timeout_begin = esp_timer_get_time();
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        reset_sleep_timeout();
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        reset_sleep_timeout();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        reset_sleep_timeout();
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(uint8_t channel)
{

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .ssid_len = strlen(AP_WIFI_SSID),
            .channel = channel,
            .password = AP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };

    if (strlen(AP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    static char wifi_name[sizeof(wifi_config.ap.ssid)] = AP_WIFI_SSID;
    int l = strlen(wifi_name);
    itoa(id, &wifi_name[l], 10);

    strlcpy((char *)wifi_config.ap.ssid, wifi_name, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(wifi_name);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); //влияние на измерения АЦП незначительные
    
    int8_t power = 0;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d power:%d", AP_WIFI_SSID, AP_WIFI_PASS, wifi_config.ap.channel, power);
}

int wifi_init_sta(void)
{

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CLIENT_WIFI_SSID,
            .password = CLIENT_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        }};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
        return 1;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    return 0;
}

/* An HTTP GET handler */
static esp_err_t settings_handler(httpd_req_t *req)
{
    reset_sleep_timeout();

    /*                       "<script src=\"/d3.min.js\"></script>"
                           "<link rel=\"shortcut icon\" href=\"/favicon.ico\" type=\"image/x-icon\">"
                       "<link rel=\"icon\" href=\"/favicon.ico\" type=\"image/x-icon\">"
*/
    const char *head = "<!DOCTYPE html><html><head>"
                       "<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\">"
                       "<meta name=\"viewport\" content=\"width=device-width\">"
                       "<link rel=\"shortcut icon\" href=\"/favicon.ico\">"
                       /*"<link rel=\"icon\" type=\"image/png\" sizes=\"32x32\" href=\"/favicon-32x32.png\">"
                       "<link rel=\"icon\" type=\"image/png\" sizes=\"16x16\" href=\"/favicon-16x16.png\">"*/
                       "<title>Settings</title></head><body>";
    const char *sodkstartform1 = "<form><fieldset><legend>"; // СОДК</legend><table>";
    const char *sodkstartform2 = "</legend><table>";

    const char *sodkend = "</table><input type=\"submit\" value=\"Сохранить\" />&nbsp;&nbsp;<input type=\"checkbox\" name=\"save\" id=\"save\"><label for=\"save\">save to flash</label></fieldset></form>";

    const char *nbiotstart = "<form><fieldset><legend>NB-IoT</legend><table>";
    const char *nbiotend = "</table><input type=\"submit\" value=\"Сохранить\" /></fieldset></form>";
    const char *tail = "<p><a href=\"/d?mode=2\">Буфер данных</a>&nbsp;&nbsp;<a href=\"/d3\">График АЦП</a>&nbsp;&nbsp;<a href=\"/d3?mode=1\">График АЦП(фильтр)</a>&nbsp;&nbsp;<a href=\"/d3?mode=2\">График R</a></p>"
                       "<p><button onclick=\"measure(1)\">Измерить 1</button>&nbsp;<button onclick=\"measure(2)\">Измерить 2</button>&nbsp;<button onclick=\"measure(3)\">Измерить 3</button>&nbsp;<button onclick=\"measure(4)\">Измерить 4</button>&nbsp;"
                       "<button onclick=\"measure(5)\">Измерить 5</button>&nbsp;<button onclick=\"measure(6)\">Измерить 6</button>&nbsp;<button onclick=\"measure(7)\">Измерить 7</button>&nbsp;<button onclick=\"measure(8)\">Измерить 8</button></p>"
                       "<p><textarea id=\"text\" style=\"width:98\%;height:400px;\"></textarea></p>\n"
                       "<a href=\"?restart=true\">Restart</a>\n"
                       "<script>var socket = new WebSocket(\"ws://\" + location.host + \"/ws\");\n"
                       "socket.onopen = function(){socket.send(\"openws:\" + String(Date.now() / 1000));};\n"
                       "socket.onmessage = function(e){document.getElementById(\"text\").value += e.data + \"\\n\";};\n"
                       "function measure(chan){socket.send(\"start:\"+ String(chan));};\n"
                       "</script>"
                       "</body></html>";

    const char *lora_set_id[] = {
        "<tr><td><label for=\"id\">ID transceiver:</label></td><td><input type=\"text\" name=\"id\" id=\"id\" size=\"7\" value=\"",
        "\" /></td></tr>\n"};

    const char *nb_set_ip[] = {
        "<tr><td><label for=\"ip\">IP adress:</label></td><td><input type=\"text\" name=\"ip\" id=\"ip\" size=\"16\" value=\"",
        "\" /></td></tr>\n"};

    const char *nb_set_port[] = {
        "<tr><td><label for=\"port\">Port:</label></td><td><input type=\"text\" name=\"port\" id=\"port\" size=\"16\" value=\"",
        "\" /></td></tr>\n"};

    const char *nb_set_timezone[] = {
        "<tr><td><label for=\"timezone\">Timezone:</label></td><td><input type=\"text\" name=\"timezone\" id=\"timezone\" size=\"16\" value=\"",
        "\" /></td></tr>\n"};

#ifndef NB
    const char *lorastart = "<form><fieldset><legend>LoRa</legend><table>";
    const char *loraend = "</table><input type=\"submit\" value=\"Сохранить\" /></fieldset></form>";

    const char *lora_set_bw[] = {
        "<tr><td><label for='bw'>Signal bandwidth:</label></td><td><select name='bw' id='bw'>",
        "</select></td></tr>"};

    const char *lora_set_bw_options[][3] = {
        {"<option value=", "0", ">7.8 kHz</option>"},
        {"<option value=", "1", ">10.4 kHz</option>"},
        {"<option value=", "2", ">15.6 kHz</option>"},
        {"<option value=", "3", ">20.8 kHz</option>"},
        {"<option value=", "4", ">31.25 kHz</option>"},
        {"<option value=", "5", ">41.7 kHz</option>"},
        {"<option value=", "6", ">62.5 kHz</option>"},
        {"<option value=", "7", ">125 kHz</option>"},
        {"<option value=", "8", ">250 kHz</option>"},
        {"<option value=", "9", ">500 kHz</option>"}};

    const char *lora_set_fr[] = {
        "<tr><td><label for=\"fr\">Signal frequency:</label></td><td><input type=\"text\" name=\"fr\" id=\"fr\" size=\"7\" value=\"",
        "\" />  kHz</td></tr>"};

    const char *lora_set_sf[] = {
        "<tr><td><label for=\"sf\">Spreading Factor(6-12):</label></td><td><input type=\"text\" name=\"sf\" id=\"sf\" size=\"7\" value=\"",
        "\" /></td></tr>"};

    const char *lora_set_op[] = {
        "<tr><td><label for=\"op\">Output Power(2-17):</label></td><td><input type=\"text\" name=\"op\" id=\"op\" size=\"7\" value=\"",
        "\" /></td></tr>"};
#endif

    char param[32];

    /* Read URL query string length */
    buf_len = httpd_req_get_url_query_len(req);

    if (buf_len > 1 && buf_len < sizeof(buf))
    {
        if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
        {
            ESP_LOGI(TAGH, "Found URL query => %s", buf);

            bool param_change = false;
            bool save = false;

            if (httpd_query_key_value(buf, "restart", param, 7) == ESP_OK)
            {
                restart = true;
            }

            if (httpd_query_key_value(buf, "save", param, 7) == ESP_OK)
            {
                save = true;
            }

            for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
            {
                if (httpd_query_key_value(buf, menu[i].id, param, 10) == ESP_OK)
                {
                    int p = atoi(param);
                    if (menu[i].val != p && p >= menu[i].min && p <= menu[i].max)
                    {
                        menu[i].val = p;

                        // printf("Param %s = %d\n", menu[i].id, menu[i].val);

                        nvs_handle_t my_handle;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
                        }
                        else if (save == true)
                        {
                            // Write
                            ESP_LOGD(TAG, "Write: \"%s\" ", menu[i].name);
                            err = nvs_set_i32(my_handle, menu[i].id, menu[i].val);
                            if (err != ESP_OK)
                                ESP_LOGE(TAG, "Write: \"%s\" - Failed!", menu[i].name);

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
                    }
                }
            }
#ifdef NB
            if (httpd_query_key_value(buf, "id", param, 7) == ESP_OK)
            {
                int p = atoi(param);
                if (id != p && p > 0)
                {
                    param_change = true;
                    id = p;
                }
            }

            if (httpd_query_key_value(buf, "ip", param, 17) == ESP_OK)
            {
                if (strncmp(param, serverip, 16) != 0)
                {
                    param_change = true;
                    strlcpy(serverip, param, 16);
                }
            }

            if (httpd_query_key_value(buf, "port", param, 6) == ESP_OK)
            {
                int p = atoi(param);
                if (port != p && p > 0)
                {
                    param_change = true;
                    port = p;
                }
            }

            if (httpd_query_key_value(buf, "proto", param, 2) == ESP_OK)
            {
                int p = atoi(param);
                if (proto != p)
                {
                    param_change = true;
                    proto = p;
                }
            }

            if (httpd_query_key_value(buf, "timezone", param, 2) == ESP_OK)
            {
                int p = atoi(param);
                if (timezone != p)
                {
                    param_change = true;
                    timezone = p;
                }
            }

#else
            if (httpd_query_key_value(buf, "id", param, 7) == ESP_OK)
            {
                int p = atoi(param);
                if (id != p && p > 0 && p < 1000000)
                {
                    param_change = true;
                    id = p;
                    write_nvs_lora("id", id);
                }
            }

            if (httpd_query_key_value(buf, "fr", param, 7) == ESP_OK)
            {
                int p = atoi(param);
                if (fr != p && p > 800000 && p < 1000000)
                {
                    param_change = true;
                    fr = p;
                    write_nvs_lora("fr", fr);
                }
            }

            if (httpd_query_key_value(buf, "op", param, 3) == ESP_OK)
            {
                int p = atoi(param);
                if (op != p && p >= 2 && p <= 17)
                {
                    param_change = true;
                    op = p;
                    write_nvs_lora("op", op);
                }
            }

            if (httpd_query_key_value(buf, "bw", param, 2) == ESP_OK)
            {
                int p = atoi(param);
                if (bw != p && p >= 0 && p <= 9)
                {
                    param_change = true;
                    bw = p;
                    write_nvs_lora("bw", bw);
                }
            }

            if (httpd_query_key_value(buf, "sf", param, 3) == ESP_OK)
            {
                int p = atoi(param);
                if (sf != p && p >= 6 && p <= 12)
                {
                    param_change = true;
                    sf = p;
                    write_nvs_lora("sf", sf);
                }
            }
#endif

            if (param_change)
            {
                write_nvs_nbiot();
            };

            httpd_resp_set_status(req, "307 Temporary Redirect");
            httpd_resp_set_hdr(req, "Location", PAGE_LORA_SET);
            httpd_resp_send(req, NULL, 0); // Response body can be empty
            return ESP_OK;
        }
    }

    // httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr_chunk(req, head);

    //-----------------------------СОДК------------------------------
    strlcpy(buf, sodkstartform1, sizeof(buf));
    snprintf(param, sizeof(param), "СОДК %02x-%02x-%02x-%02x-%02x-%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, sodkstartform2, sizeof(buf));

    httpd_resp_sendstr_chunk(req, buf);
    buf[0] = '\0';

    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        if (i % 2 == 0)
            strlcat(buf, "\n<tr>", sizeof(buf));

        strlcat(buf, "<td><label for=\"", sizeof(buf));
        strlcat(buf, menu[i].id, sizeof(buf));
        strlcat(buf, "\">", sizeof(buf));
        strlcat(buf, menu[i].name, sizeof(buf));
        strlcat(buf, "</label></td><td><input type=\"text\" name=\"", sizeof(buf));
        strlcat(buf, menu[i].id, sizeof(buf));
        strlcat(buf, "\" id=\"", sizeof(buf));
        strlcat(buf, menu[i].id, sizeof(buf));
        strlcat(buf, "\" size=\"7\" value=\"", sizeof(buf));
        itoa(menu[i].val, param, 10);
        strlcat(buf, param, sizeof(buf));
        strlcat(buf, "\" /></td>", sizeof(buf));

        if (i % 2 == 1)
            strlcat(buf, "</tr>", sizeof(buf));

        if (strlen(buf) > sizeof(buf) / 2)
        {
            httpd_resp_sendstr_chunk(req, buf);
            buf[0] = '\0';
        }
    }
    strlcat(buf, sodkend, sizeof(buf));
    strlcat(buf, "\n", sizeof(buf));
    httpd_resp_sendstr_chunk(req, buf);
#ifdef NB
    //-----------------------------NB-IoT------------------------------
    // ESP_LOG_BUFFER_HEXDUMP(TAG, serverip, 10, ESP_LOG_INFO);

    strlcpy(buf, nbiotstart, sizeof(buf));

    // Generate id
    strlcat(buf, lora_set_id[0], sizeof(buf));
    itoa(id, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, lora_set_id[1], sizeof(buf));

    // tcp / udp
    const char *nb_set_proto[] = {
        "<tr><td><label for=\"proto\">Data protocol:</label></td><td>",
        "</td></tr>\n"};
    const char *proto_select = "<label><input type=\"radio\" name=\"proto\" value=\"";

    strlcat(buf, nb_set_proto[0], sizeof(buf));
    strlcat(buf, proto_select, sizeof(buf));
    if (proto == 0)
        strlcat(buf, "0\" checked/>", sizeof(buf));
    else
        strlcat(buf, "0\"/>", sizeof(buf));
    strlcat(buf, "TCP</label>", sizeof(buf));

    strlcat(buf, proto_select, sizeof(buf));
    if (proto != 0)
        strlcat(buf, "1\" checked/>", sizeof(buf));
    else
        strlcat(buf, "1\"/>", sizeof(buf));
    strlcat(buf, "UDP</label>", sizeof(buf));

    strlcat(buf, nb_set_proto[1], sizeof(buf));

    // Generate ip
    strlcat(buf, nb_set_ip[0], sizeof(buf));
    strlcat(buf, serverip, sizeof(buf));
    strlcat(buf, nb_set_ip[1], sizeof(buf));

    // Generate port
    strlcat(buf, nb_set_port[0], sizeof(buf));
    itoa(port, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, nb_set_port[1], sizeof(buf));

    // Generate timezone
    strlcat(buf, nb_set_timezone[0], sizeof(buf));
    itoa(timezone, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, nb_set_timezone[1], sizeof(buf));

    // status
    strlcat(buf, "<tr><td>Status:</td><td>", sizeof(buf));
    strlcat(buf, modem_status, sizeof(buf));
    strlcat(buf, "</tr>", sizeof(buf));

    strlcat(buf, nbiotend, sizeof(buf));

    httpd_resp_sendstr_chunk(req, buf);

    // httpd_resp_sendstr_chunk(req, nbiotend);
#else
    //-----------------------------LoRa------------------------------
    httpd_resp_sendstr_chunk(req, lorastart);

    // Generate id
    strlcpy(buf, lora_set_id[0], sizeof(buf));
    itoa(id, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, lora_set_id[1], sizeof(buf));
    httpd_resp_sendstr_chunk(req, buf);

    // Generate fr
    strlcpy(buf, lora_set_fr[0], sizeof(buf));
    itoa(fr, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, lora_set_fr[1], sizeof(buf));
    httpd_resp_sendstr_chunk(req, buf);

    // Generate bw
    strlcpy(buf, lora_set_bw[0], sizeof(buf));
    for (int i = 0; i < 10; i++)
    {
        strlcat(buf, lora_set_bw_options[i][0], sizeof(buf));
        param[0] = '"';
        param[1] = *lora_set_bw_options[i][1];
        param[2] = '"';
        param[3] = 0;
        int n = atoi(lora_set_bw_options[i][1]);
        if (n == bw)
        {
            strcpy(&param[3], " selected");
        }

        strlcat(buf, param, sizeof(buf));
        strlcat(buf, lora_set_bw_options[i][2], sizeof(buf));
    }
    strlcat(buf, lora_set_bw[1], sizeof(buf));
    httpd_resp_sendstr_chunk(req, buf);

    // Generate sf
    strlcpy(buf, lora_set_sf[0], sizeof(buf));
    itoa(sf, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, lora_set_sf[1], sizeof(buf));
    httpd_resp_sendstr_chunk(req, buf);

    // Generate op
    strlcpy(buf, lora_set_op[0], sizeof(buf));
    itoa(op, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, lora_set_op[1], sizeof(buf));
    httpd_resp_sendstr_chunk(req, buf);

    httpd_resp_sendstr_chunk(req, loraend);
#endif

    httpd_resp_sendstr_chunk(req, tail);
    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

static esp_err_t download_get_handler(httpd_req_t *req)
{
    reset_sleep_timeout();

    char *filepath = ((down_data_t *)(req->user_ctx))->filepath;
    FILE *fd = NULL;
    struct stat file_stat;

    if (stat(filepath, &file_stat) == -1)
    {
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filepath, file_stat.st_size);

    httpd_resp_set_type(req, ((down_data_t *)(req->user_ctx))->content);
    int l = strlen(filepath);
    if (filepath[l - 3] == '.' && filepath[l - 2] == 'g' && filepath[l - 1] == 'z')
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    size_t chunksize;
    // int n = 0;
    do
    {
        // memset(buf, 0, sizeof(buf));
        chunksize = fread(buf, 1, sizeof(buf), fd);
        // printf("fread %d\n", chunksize);

        if (chunksize > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, buf, chunksize) != ESP_OK)
            {
                fclose(fd);
                ESP_LOGE(TAG, "File %s sending failed!", filepath);
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    // ESP_LOGI(TAG, "File sending complete");

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
};

/* Handler to download a file kept on the server */
static esp_err_t download_ADCdata_handler(httpd_req_t *req)
{
    reset_sleep_timeout();

    char line[128];
    int limitData = RINGBUFLEN;
    int mode = 0; // 0 - ADC, 1 - фильтр ADC, 2 - СОДК результаты

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
        if (httpd_query_key_value(buf, "mode", line, 6) == ESP_OK)
        {
            mode = atoi(line);
        }
    }
    // printf("mode:%d\n", mode);

    strlcpy(line, "attachment; filename=\"", sizeof(line));
    if (mode == 2)
        strlcat(line, "SODKdata.txt\"", sizeof(line));
    else
        strlcat(line, "ADCdata.txt\"", sizeof(line));

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", line);
    httpd_resp_set_hdr(req, "Connection", "close");

    // uint8_t *ptr_adc = 0;

    int l = 0;
    int ll = 0;
    int n = 0;
    buf[0] = '\0';

    while (n < limitData)
    {
        if (mode == 2)
            ll = getResult_Data(&buf[l], n);
        else if (mode == 1)
            ll = getADC_Data(&buf[l], n, true);
        else
            ll = getADC_Data(&buf[l], n, false);

        if (ll == 0) // data end
            break;

        l = l + ll;
        if (l > (sizeof(buf) - 128))
        {
            // printf("l: %d, ", l);

            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, buf, l) != ESP_OK)
            {
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_sendstr_chunk(req, NULL));
                /* Respond with 500 Internal Server Error */
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file"));
                return ESP_FAIL;
            }

            l = 0;
        }
        n++;
    };

    if (l > 0)
    {
        if (httpd_resp_send_chunk(req, buf, l) != ESP_OK)
        {
            ESP_LOGE(TAG, "File sending failed!");
            /* Abort sending file */
            ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_sendstr_chunk(req, NULL));
            /* Respond with 500 Internal Server Error */
            ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file"));
            return ESP_FAIL;
        }
    }

    /* Respond with an empty chunk to signal HTTP response completion */
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
};

#define ESP_IMAGE_HEADER_MAGIC 0xE9 /*!< The magic word for the esp_image_header_t structure. */

/*
 * Handle OTA file upload
 */
esp_err_t update_post_handler(httpd_req_t *req)
{
    int file_id = -1;
    esp_ota_handle_t ota_handle;
    int remaining = req->content_len;

    reset_sleep_timeout();

    /* Пишем в next_ota и прошивку и spiffs.bin*/
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);

    while (remaining > 0)
    {
        int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

        // Timeout Error: Just retry
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            continue;

            // Serious Error: Abort OTA
        }
        else if (recv_len <= 0)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
            return ESP_FAIL;
        }

        if (file_id == -1) // first data block
        {
            file_id = buf[0];

            if (file_id == ESP_IMAGE_HEADER_MAGIC)
            {
                ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));
            }
            else if (remaining == 0x80000)
            {
                file_id = remaining;
                ESP_ERROR_CHECK(esp_partition_erase_range(ota_partition, 0, 0x80000));
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File Error");
                return ESP_FAIL;
            }
        }

        // firmware.bin
        if (file_id == ESP_IMAGE_HEADER_MAGIC)
        {
            // Successful Upload: Flash firmware chunk
            if (esp_ota_write(ota_handle, (const void *)buf, recv_len) != ESP_OK)
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
                return ESP_FAIL;
            }
            vTaskDelay(1);
        }
        else
            // spiffs.bin
            if (file_id == 0x80000)
            {
                ESP_ERROR_CHECK(esp_partition_write(ota_partition, (req->content_len - remaining), (const void *)buf, recv_len));
                vTaskDelay(1);
            }

        remaining -= recv_len;
    }

    if (file_id == ESP_IMAGE_HEADER_MAGIC)
    {
        // Validate and switch to new OTA image and reboot
        if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
            return ESP_FAIL;
        }

        httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");
        ESP_LOGW(TAG, "Firmware update complete, rebooting now!");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        restart = true;
    }
    else if (file_id == 0x80000)
    {
        const esp_partition_t *storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
        ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, 0x80000));

        remaining = req->content_len;
        int recv_len = sizeof(buf);
        while (remaining > 0)
        {
            recv_len = MIN(remaining, sizeof(buf));
            if (esp_partition_read(ota_partition, (req->content_len - remaining), (void *)buf, recv_len) == ESP_OK)
            {
                ESP_ERROR_CHECK(esp_partition_write(storage_partition, (req->content_len - remaining), (const void *)buf, recv_len));
                vTaskDelay(1);
            }
            remaining -= recv_len;
        }

        httpd_resp_sendstr(req, "SPIFFS update complete, rebooting now!\n");
        ESP_LOGW(TAG, "SPIFFS update complete, rebooting now!");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        restart = true;
    }

    return ESP_OK;
}

httpd_handle_t ws_hd;
int ws_fd[5] = {0, 0, 0, 0, 0};

/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_send(char *msg)
{
    if (ws_fd[0] == 0)
        return;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)msg;
    ws_pkt.len = strlen(msg);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    for (int i = 0; i < (sizeof(ws_fd) / sizeof(ws_fd[0])); i++)
    {
        if (ws_fd[i] > 0)
            httpd_ws_send_frame_async(ws_hd, ws_fd[i], &ws_pkt);
    }
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "WS handshake done, the new connection was opened");
        return ESP_OK;
    }

    uint8_t bf[WS_BUF_SIZE] = {0};
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = bf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(bf));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAGH, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }
    ESP_LOGD(TAGH, "Got packet with message: \"%s\"", ws_pkt.payload);
    ESP_LOGD(TAGH, "Packet type: %d", ws_pkt.type);

    ws_hd = req->handle;
    int ws_sock = httpd_req_to_sockfd(req);
    for (int i = 0; i < (sizeof(ws_fd) / sizeof(ws_fd[0])); i++)
    {
        if (ws_fd[i] == ws_sock)
            break;

        if (ws_fd[i] == 0)
        {
            ws_fd[i] = ws_sock;
            break;
        }
    }

    ESP_LOGI(TAGH, "ws_handler: httpd_handle_t=%p, sockfd=%d, client_info:%d", req->handle, httpd_req_to_sockfd(req), httpd_ws_get_fd_info(req->handle, httpd_req_to_sockfd(req)));

    const char *initstring = "openws:";
    if (strncmp(initstring, (const char *)ws_pkt.payload, strlen(initstring)) == 0)
    {
        long long ts = atoll((const char *)ws_pkt.payload + strlen(initstring));
        if (ts > 1715923962LL) // 17 May 2024 05:32:42
        {
            struct timeval now = {.tv_sec = ts + timezone * 3600}; // UNIX time + timezone offset
            settimeofday(&now, NULL);

            time_t t = time(0);
            struct tm *localtm = localtime(&t);
            ESP_LOGD(TAG, "WS set date and time: %s", asctime(localtm));
        }
        need_ws_send = true;
    }

    const char *startstring = "start:";
    if (strncmp(startstring, (const char *)ws_pkt.payload, strlen(startstring)) == 0)
    {
        int ch = atoi((const char *)ws_pkt.payload + strlen(startstring));
        if (ch > 0)
        {
            start_measure(ch, 0);
            ESP_LOGD(TAG, "Start measure channel %i", ch);
        }
    }

    return ret;
}

static void ws_close_fn(httpd_handle_t hd, int sockfd)
{
    for (int i = 0; i < (sizeof(ws_fd) / sizeof(ws_fd[0])); i++)
    {
        if (ws_fd[i] == sockfd)
        {
            ws_fd[i] = 0;
            break;
        }
    }
    ESP_LOGI(TAG, "Handle hd=%p: closing websocket fd=%d", hd, sockfd);
}

static const httpd_uri_t main_page = {
    .uri = PAGE_LORA_SET,
    .method = HTTP_GET,
    .handler = settings_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "Hello World!",
    .is_websocket = false};

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static const httpd_uri_t file_download = {
    .uri = "/d",
    .method = HTTP_GET,
    .handler = download_ADCdata_handler,
    .is_websocket = false,
};

httpd_uri_t update_get = {
    .uri = "/update",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/update.html", .content = "text/html"}),
    .is_websocket = false};

httpd_uri_t update_post = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = update_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t favicon_ico = {
    .uri = "/favicon.ico",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/favicon.ico", .content = "image/x-icon"}),
    .is_websocket = false};
/*
static const httpd_uri_t favicon_png16 = {
    .uri = "/favicon-16x16.png",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/favicon-16x16.png", .content = "image/png"}),
    .is_websocket = false};
static const httpd_uri_t favicon_png32 = {
    .uri = "/favicon-32x32.png",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/favicon-32x32.png", .content = "image/png"}),
    .is_websocket = false};
*/
static const httpd_uri_t d3_get = {
    .uri = "/d3",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/testD3.html", .content = "text/html"}),
    .is_websocket = false};

static const httpd_uri_t d3_get_gz = {
    .uri = "/d3.min.js",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/d3.min.js.gz", .content = "application/javascript"}),
    .is_websocket = false};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // config.max_open_sockets = 5;
    // config.stack_size = 1024 * 10;
    config.lru_purge_enable = true;
    // config.send_wait_timeout = 30;
    // config.recv_wait_timeout = 30;
    // config.task_priority = 6;
    // config.close_fn = ws_close_fn;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &main_page);
        httpd_register_uri_handler(server, &ws);
        httpd_register_uri_handler(server, &file_download);
        httpd_register_uri_handler(server, &d3_get);
        httpd_register_uri_handler(server, &d3_get_gz);
        httpd_register_uri_handler(server, &favicon_ico);
        // httpd_register_uri_handler(server, &favicon_png16);
        // httpd_register_uri_handler(server, &favicon_png32);

        httpd_register_uri_handler(server, &update_post);
        httpd_register_uri_handler(server, &update_get);

        for (int i = 0; i < (sizeof(ws_fd) / sizeof(ws_fd[0])); i++)
        {
            ws_fd[i] = 0;
        }

        return server;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return NULL;
}

void wifi_task(void *arg)
{
    ESP_LOGI("SPIFFS", "Initializing SPIFFS");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
    }

    // int wifi_on = 1;

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_softap(menu[23].val); // WiFi channel

    /* Start the server for the first time */
    start_webserver();

    /* Mark current app as valid */
    const esp_partition_t *partition = esp_ota_get_running_partition();
    // printf("Currently running partition: %s\r\n", partition->label);
    ESP_LOGI(TAG, "Currently running partition: %s", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    reset_sleep_timeout();

    while (1)
    {
        if (ws_fd[0] > 0)
        {
            // Receive an item from no-split ring buffer
            size_t item_size;
            char *item = (char *)xRingbufferReceive(wsbuf_handle, &item_size, pdMS_TO_TICKS(0));
            // Check received item
            if (item != NULL)
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_queue_work(ws_hd, (httpd_work_fn_t)ws_async_send, item));
                reset_sleep_timeout();
                // Return Item
                vRingbufferReturnItem(wsbuf_handle, (void *)item);
            }
        }
        /*
        if (pdTRUE == xQueueReceive(ws_send_queue, msg, 1000 / portTICK_PERIOD_MS))
        {
            need_ws_send = true;
        }

        if (need_ws_send && ws_fd > 0)
        {
            httpd_queue_work(ws_hd, (httpd_work_fn_t)ws_async_send, msg);
            reset_sleep_timeout();
            need_ws_send = false;
        }
        */
        // WiFi timeout
        if (esp_timer_get_time() - timeout_begin > StoUS(menu[22].val))
        {
            esp_wifi_stop();
            xEventGroupSetBits(ready_event_group, END_WIFI_TIMEOUT);
        }

        if (restart == true)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_wifi_stop();
            esp_restart();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
