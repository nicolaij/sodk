/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "main.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"

#include <esp_http_server.h>

#include "esp_spiffs.h"

#include "driver/uart.h"

// extern menu_t menu[];
extern uint8_t mac[6];
extern char modem_status[128];

#if CONFIG_IDF_TARGET_ESP32
extern int bufferR[DATALEN];
extern int bufferU[DATALEN];
#elif CONFIG_IDF_TARGET_ESP32C3
extern int bufferADC[DATALEN];
#endif

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define CLIENT_WIFI_SSID "Nadtocheeva 5"
#define CLIENT_WIFI_PASS "123123123"
#define AP_WIFI_SSID "MegaOm"
#define AP_WIFI_PASS "123123123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 0

#define EXAMPLE_ESP_WIFI_CHANNEL 1
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

char buf[1440];
size_t buf_len;

int64_t timeout_start;

bool need_ws_send = false;

typedef struct
{
    char filepath[32];
    char content[32];
} down_data_t;

void reset_sleep_timeout()
{
    timeout_start = esp_timer_get_time();
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

void wifi_init_softap(void)
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
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = AP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };

    if (strlen(AP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    static char wifi_name[32] = AP_WIFI_SSID;
    int l = strlen(wifi_name);
    itoa(id, &wifi_name[l], 10);

    strlcpy((char *)wifi_config.ap.ssid, wifi_name, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(wifi_name);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", AP_WIFI_SSID, AP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
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

    /*                       "<script src=\"/d3.min.js\"></script>"*/
    const char *head = "<!DOCTYPE html><html><head>"
                       "<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\">"
                       "<meta name=\"viewport\" content=\"width=device-width\">"
                       "<title>Settings</title></head><body>";
    const char *sodkstartform1 = "<form><fieldset><legend>"; //СОДК</legend><table>";
    const char *sodkstartform2 = "</legend><table>";

    const char *sodkend = "</table><input type=\"submit\" value=\"Сохранить\" />&nbsp;&nbsp;<input type=\"checkbox\" name=\"save\" id=\"save\"><label for=\"save\">save to flash</label></fieldset></form>";

    const char *lorastart = "<form><fieldset><legend>LoRa</legend><table>";
    const char *loraend = "</table><input type=\"submit\" value=\"Сохранить\" /></fieldset></form>";
    const char *nbiotstart = "<form><fieldset><legend>NB-IoT</legend><table>";
    const char *nbiotend = "</table><input type=\"submit\" value=\"Сохранить\" /></fieldset></form>";
    const char *tail = "<p><a href=\"/d?end=10000\">Буфер данных</a>&nbsp;&nbsp;<a href=\"/d3?end=500\">График АЦП</a></p>"
                       "<p><textarea id=\"text\" style=\"width:98\%;height:400px;\"></textarea></p>\n"
                       "<script>var socket = new WebSocket(\"ws://\" + location.host + \"/ws\");\n"
                       "socket.onopen = function(){socket.send(\"open ws\");};\n"
                       "socket.onmessage = function(e){document.getElementById(\"text\").value += e.data + \"\\n\";}"
                       "</script>"
                       "</body></html>";

    const char *lora_set_id[] = {
        "<tr><td><label for=\"id\">ID transceiver:</label></td><td><input type=\"text\" name=\"id\" id=\"id\" size=\"7\" value=\"",
        "\" /></td></tr>"};

    const char *nb_set_ip[] = {
        "<tr><td><label for=\"ip\">IP adress:</label></td><td><input type=\"text\" name=\"ip\" id=\"ip\" size=\"16\" value=\"",
        "\" /></td></tr>"};

    const char *nb_set_port[] = {
        "<tr><td><label for=\"ip\">UDP port:</label></td><td><input type=\"text\" name=\"port\" id=\"port\" size=\"16\" value=\"",
        "\" /></td></tr>"};

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

            if (httpd_query_key_value(buf, "save", param, 7) == ESP_OK)
            {
                save = true;
            }

            for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
            {
                if (httpd_query_key_value(buf, menu[i].id, param, 7) == ESP_OK)
                {
                    int p = atoi(param);
                    if (menu[i].val != p && p >= menu[i].min && p <= menu[i].max)
                    {
                        param_change = true;

                        menu[i].val = p;

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

            if (httpd_query_key_value(buf, "ip", param, 16) == ESP_OK)
            {
                if (strncmp(param, serverip, 16) != 0)
                {
                    param_change = true;
                    strlcpy(serverip, param, 16);
                }
            }

            if (httpd_query_key_value(buf, "port", param, 5) == ESP_OK)
            {
                int p = atoi(param);
                if (port != p && p > 0)
                {
                    param_change = true;
                    port = p;
                }
            }

            if (param_change)
            {
                write_nvs_nbiot(&id, apn, serverip, &port);
            };
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
            // if (param_change)
            //{
            httpd_resp_set_status(req, "307 Temporary Redirect");
            httpd_resp_set_hdr(req, "Location", PAGE_LORA_SET);
            httpd_resp_send(req, NULL, 0); // Response body can be empty
            return ESP_OK;
            //}
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

    // Generate ip
    strlcat(buf, nb_set_ip[0], sizeof(buf));
    strlcat(buf, serverip, sizeof(buf));
    strlcat(buf, nb_set_ip[1], sizeof(buf));

    // Generate port
    strlcat(buf, nb_set_port[0], sizeof(buf));
    itoa(port, param, 10);
    strlcat(buf, param, sizeof(buf));
    strlcat(buf, nb_set_port[1], sizeof(buf));

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
    int n = 0;
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
    int limitADCData = 0;

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
        if (httpd_query_key_value(buf, "end", line, 6) == ESP_OK)
        {
            limitADCData = atoi(line);
        }

    //по умолчанию 500 строк
    if (limitADCData == 0)
        limitADCData = 500;

    char header[96] = "attachment; filename=\"";
    // strlcat(header, line, sizeof(header));
    strlcat(header, "ADCdata.txt\"", sizeof(header));

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", header);
    httpd_resp_set_hdr(req, "Connection", "close");

    uint8_t *ptr_adc = 0;

    buf[0] = 0;
    int l = 0;
    int n = 0;

#if CONFIG_IDF_TARGET_ESP32C3
    while (getADC_Data(line, &ptr_adc, &n) > 0)
    {
        l = strlcat(buf, line, sizeof(buf));
        if (l > (sizeof(buf) - sizeof(line)))
        {
            // printf("l: %d, ", l);

            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, buf, l) != ESP_OK)
            {
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }

            buf[0] = 0;
            l = 0;
        }

        // if (n++ % 100 == 0)
        //{
        //     vTaskDelay(1);
        // }

        if (n > limitADCData)
        {
            break;
        }
    }
#endif

    if (l > 0)
    {
        httpd_resp_send_chunk(req, buf, l);
    }

    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_handle_t ws_hd;
int ws_fd = 0;

/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_send(char *msg)
{
    if (ws_fd == 0)
        return;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)msg;
    ws_pkt.len = strlen(msg);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(ws_hd, ws_fd, &ws_pkt);
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
    ESP_LOGI(TAGH, "Got packet with message: \"%s\"", ws_pkt.payload);
    ESP_LOGI(TAGH, "Packet type: %d", ws_pkt.type);

    ws_hd = req->handle;
    ws_fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAGH, "ws_hd/fd: %d/%d", *(int *)ws_hd, ws_fd);

    if (strcmp("open ws", (const char *)ws_pkt.payload) == 0)
        need_ws_send = true;

    /*
        strlcpy((char*)buf, "Test!!!", sizeof(buf));

        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAGH, "httpd_ws_send_frame failed with %d", ret);
        }
        */
    return ret;
}

/* URI handler for getting uploaded files */
static const httpd_uri_t file_download = {
    .uri = "/d",
    .method = HTTP_GET,
    .handler = download_ADCdata_handler,
    .is_websocket = false,
};

static const httpd_uri_t lora_set = {
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
    // config.max_open_sockets = 2;
    // config.stack_size = 1024 * 10;
    config.lru_purge_enable = true;
    // config.send_wait_timeout = 30;
    // config.recv_wait_timeout = 30;
    // config.task_priority = 6;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &lora_set);
        httpd_register_uri_handler(server, &ws);
        httpd_register_uri_handler(server, &file_download);
        httpd_register_uri_handler(server, &d3_get);
        httpd_register_uri_handler(server, &d3_get_gz);

        ws_fd = 0;

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void wifi_task(void *arg)
{
    char msg[WS_BUF_SIZE];

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

    int wifi_on = 1;

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (wifi_init_sta() == 0)
        wifi_init_softap();

    /* Start the server for the first time */
    start_webserver();

    reset_sleep_timeout();

    while (1)
    {
        if (ws_fd > 0)
        {
            if (pdTRUE == xQueueReceive(ws_send_queue, msg, 0))
            {
                httpd_queue_work(ws_hd, (httpd_work_fn_t)ws_async_send, msg);
                reset_sleep_timeout();
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
        if (esp_timer_get_time() - timeout_start > (int64_t)menu[15].val * 1000000)
        {
            esp_wifi_stop();
            xEventGroupSetBits(ready_event_group, END_WIFI_TIMEOUT);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
