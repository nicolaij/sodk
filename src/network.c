/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "main.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"
#include <esp_http_server.h>

extern menu_t menu[];

extern int bufferR[DATALEN];
extern int bufferU[DATALEN];

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define CLIENT_WIFI_SSID "MiMikalai"
#define CLIENT_WIFI_PASS "123123123"
#define AP_WIFI_SSID "MegaOm"
#define AP_WIFI_PASS "123123123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 3

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

static const char *TAG = "wifi";

static const char *TAGH = "httpd";

static int s_retry_num = 0;

char buf[512];
size_t buf_len;

extern int fr;
extern int bw;
extern int sf;
extern int op;
extern int id;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
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
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
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
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
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
static esp_err_t lora_set_handler(httpd_req_t *req)
{
    const char *head = "<!DOCTYPE html><html><head>"
                       "<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\">"
                       "<meta name=\"viewport\" content=\"width=device-width\">"
                       "<title>Lora setting</title></head><body><form><fieldset>"
                       "<legend>LoRa</legend><table>";

    const char *tail = "</table><input type=\"submit\" value=\"Сохранить\" /></fieldset></form>"
                       "<p><a href=\"/d\">Буфер данных</a></p>"
                       "<p><textarea id=\"text\" style=\"width:98\%;height:400px;\"></textarea></p>\n"
                       "<script>var socket = new WebSocket(\"ws://\" + location.host + \"/ws\");\n"
                       "socket.onopen = function(){socket.send(\"open ws\");};\n"
                       "socket.onmessage = function(e){document.getElementById(\"text\").value += e.data + \"\\n\";}"
                       "</script>"
                       "</body></html>";

    const char *lora_set_id[] = {
        "<tr><td><label for=\"id\">ID transceiver:</label></td><td><input type=\"text\" name=\"id\" id=\"id\" size=\"7\" value=\"",
        "\" /></td></tr>"};

    const char *lora_set_bw[] = {
        "<tr><td><label for='bw'>Signal bandwidth:</label></td><td><select name='bw' id='bw'>",
        "</select></td></tr>"};

    const char *lora_set_bw_options[10][3] = {
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

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1 && buf_len < sizeof(buf))
    {
        // buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            ESP_LOGI(TAGH, "Found URL query => %s", buf);

            bool param_change = false;
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

            if (param_change)
            {
                httpd_resp_set_status(req, "307 Temporary Redirect");
                httpd_resp_set_hdr(req, "Location", PAGE_LORA_SET);
                httpd_resp_send(req, NULL, 0); // Response body can be empty
                return ESP_OK;
            }
        }
    }

    httpd_resp_sendstr_chunk(req, head);

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

    httpd_resp_sendstr_chunk(req, tail);
    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);

    reset_sleep_timeout();

    return ESP_OK;
}

/* Handler to download a file kept on the server */
static esp_err_t download_get_handler(httpd_req_t *req)
{
    int pos = 0;
    char line[32];
    char header[96] = "attachment; filename=\"";

    cur_time(line);

    strlcat(header, line, sizeof(header));
    strlcat(header, ".txt\"", sizeof(header));

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", header);

    while (pos < sizeof(bufferR) / sizeof(bufferR[0]))
    {
        buf[0] = 0;
        int str_len = 0;
        for (int i = 0; i < 20; i++)
        {
            int l = sprintf(line, "%4d, %4d, %4d, %4d\n", pos, bufferU[pos], bufferR[pos], kOm(bufferU[pos], bufferR[pos]));
            strlcpy(&buf[str_len], line, sizeof(buf));
            str_len += l;
            if (str_len >= sizeof(buf))
            {
                ESP_LOGE(TAG, "Buffer owerflow!!!");
                return ESP_FAIL;
            }
            pos++;
        }

        /* Send the buffer contents as HTTP response chunk */
        if (httpd_resp_send_chunk(req, buf, str_len) != ESP_OK)
        {
            ESP_LOGE(TAG, "File sending failed!");
            /* Abort sending file */
            httpd_resp_sendstr_chunk(req, NULL);
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }
    }

    reset_sleep_timeout();

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
    uint8_t bf[128] = {0};
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = bf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 128);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAGH, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }
    ESP_LOGI(TAGH, "Got packet with message: %s", ws_pkt.payload);
    ESP_LOGI(TAGH, "Packet type: %d", ws_pkt.type);

    ws_hd = req->handle;
    ESP_LOGI(TAGH, "ws_hd: %d", *(int *)ws_hd);
    ws_fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAGH, "ws_fd: %d", ws_fd);

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
    .handler = download_get_handler,
};

static const httpd_uri_t lora_set = {
    .uri = PAGE_LORA_SET,
    .method = HTTP_GET,
    .handler = lora_set_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "Hello World!"};

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &lora_set);
        httpd_register_uri_handler(server, &ws);
        httpd_register_uri_handler(server, &file_download);

        ws_fd = 0;

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void wifi_task(void *arg)
{
    char msg[256];

    int wifi_on = 1;

    if (wifi_init_sta() == 0)
        wifi_init_softap();

    /* Start the server for the first time */
    start_webserver();

    menu[5].val = 1;

    while (1)
    {
        if (menu[5].val == 0 && wifi_on == 1)
        {
            ESP_ERROR_CHECK(esp_wifi_stop());
            wifi_on = 0;
        }
        else if (menu[5].val == 1 && wifi_on == 0)
        {
            ESP_ERROR_CHECK(esp_wifi_start());
            wifi_on = 1;
        }

        if (pdTRUE == xQueueReceive(ws_send_queue, msg, (portTickType)1000 / portTICK_PERIOD_MS))
        {
            if (ws_fd > 0)
                httpd_queue_work(ws_hd, ws_async_send, msg);
        }
    }
}
