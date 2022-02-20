/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "main.h"

#include <string.h>
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


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define CLIENT_WIFI_SSID "Test"
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
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", AP_WIFI_SSID, AP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

void wifi_init_sta(void)
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
        },
        .ap = {.ssid = AP_WIFI_SSID, .ssid_len = strlen(AP_WIFI_SSID), .channel = EXAMPLE_ESP_WIFI_CHANNEL, .password = AP_WIFI_PASS, .max_connection = EXAMPLE_MAX_STA_CONN, .authmode = WIFI_AUTH_WPA_WPA2_PSK},

    };
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
        return;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    esp_netif_create_default_wifi_ap();

    if (strlen(AP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", AP_WIFI_SSID, AP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
    /* The event will not be processed after unregister */
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    // vEventGroupDelete(s_wifi_event_group);
}

/* An HTTP GET handler */
static esp_err_t lora_set_handler(httpd_req_t *req)
{

    const char *head = "<!DOCTYPE html><html><head>\
        <meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\">\
        <meta name=\"viewport\" content=\"width=device-width\">\
        <title>Lora setting</title></head><body><form><table>";

    const char *tail = "</table><input type=\"submit\" value=\"Сохранить\" /></body></html>";

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

    int fr = 867500; // frequency kHz
    int bw = 7;      // Номер полосы
    int sf = 7;      // SpreadingFactor
    int op = 17;     // OutputPower
    read_nvs_lora(&fr, &bw, &sf, &op);

    char buf[512];
    size_t buf_len;
    char param[32];

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1 && buf_len < sizeof(buf))
    {
        // buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            bool param_change = false;
            ESP_LOGI(TAGH, "Found URL query => %s", buf);
            /* Get value of expected key from query string */
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

    return ESP_OK;
}

static const httpd_uri_t lora_set = {
    .uri = PAGE_LORA_SET,
    .method = HTTP_GET,
    .handler = lora_set_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "Hello World!"};

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
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void wifi_task(void *arg)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    /* Start the server for the first time */
    start_webserver();

    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
