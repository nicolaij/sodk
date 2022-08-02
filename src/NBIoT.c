#include <stdio.h>
#include "driver/gpio.h"
#include <sys/time.h>
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_check.h"

#include "esp_modem.h"
#include "esp_modem_netif.h"
#include "bc26.h"

#include <main.h>

#define TX_GPIO 6
#define RX_GPIO 7
#define POWER_GPIO 2

//#define RX_BUF_SIZE 1024

static EventGroupHandle_t event_group = NULL;

//результат измерений
result_t result;

// char rx_buf[RX_BUF_SIZE];

// uint8_t buf[WS_BUF_LINE];

char apn[32] = {"mogenergo"};

char serverip[17] = {"10.179.40.11"};

uint16_t port = 48884;

int32_t id = 1; // ID передатчика

static const char *TAG = "nbiot";

extern float tsens_out;

char modem_status[128];
uint32_t rssi = 0, ber = 0;

int read_nvs_nbiot(int32_t *pid, char *apn, char *server, uint16_t *port)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("nbiot", NVS_READONLY, &my_handle);
    size_t l;
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_i32(my_handle, "id", pid);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "Read \"%s\" = %d", "id", *pid);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "The value  \"%s\" is not initialized yet!", "id");
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }

        err = nvs_get_str(my_handle, "apn", apn, &l);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "Read \"%s\" = %s", "apn", apn);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "The value  \"%s\" is not initialized yet!", "apn");
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }

        err = nvs_get_str(my_handle, "server", server, &l);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "Read \"%s\" = %s", "server", server);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "The value  \"%s\" is not initialized yet!", "server");
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }

        int32_t i = 0;
        err = nvs_get_i32(my_handle, "port", &i);
        switch (err)
        {
        case ESP_OK:
            *port = i;
            ESP_LOGD(TAG, "Read \"%s\" = %d", "port", *port);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "The value  \"%s\" is not initialized yet!", "port");
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }

        // Close
        nvs_close(my_handle);

        return 1;
    }
    return 0;
};

int write_nvs_nbiot(int32_t *pid, const char *apn, const char *server, const uint16_t *port)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("nbiot", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        // Write
        ESP_LOGD(TAG, "Write id: %d ", *pid);
        err = nvs_set_i32(my_handle, "id", *pid);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Write id: %d - Failed!", *pid);

        ESP_LOGD(TAG, "Write APN: \"%s\" ", apn);
        err = nvs_set_str(my_handle, "apn", apn);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Write APN: \"%s\" - Failed!", apn);

        ESP_LOGD(TAG, "Write server: \"%s\" ", server);
        err = nvs_set_str(my_handle, "server", server);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Write server: \"%s\" - Failed!", server);

        ESP_LOGD(TAG, "Write port: %d ", *port);
        int32_t i = *port;
        err = nvs_set_i32(my_handle, "port", i);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Write port: %d - Failed!", i);

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
    return 1;
};

esp_err_t esp_modem_dce_handle_cclk(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

    const char *ss = "+CCLK:";

    if (!strncmp(line, ss, strlen(ss)))
    {
        int32_t *data = esp_dce->priv_resource;

        char *s = strchr(line, ' ');
        if (s)
        {
            // year (two last digits)
            data[0] = atoi(s + 1);
            s = strchr(s + 1, '/');
            if (s)
            {
                // month
                data[1] = atoi(s + 1);
                s = strchr(s + 1, '/');
                if (s)
                {
                    // day
                    data[2] = atoi(s + 1);
                    s = strchr(s + 1, ',');
                    if (s)
                    {
                        // hour
                        data[3] = atoi(s + 1);
                        s = strchr(s + 1, ':');
                        if (s)
                        {
                            // minute
                            data[4] = atoi(s + 1);
                            s = strchr(s + 1, ':');
                            if (s)
                            {
                                // second
                                data[5] = atoi(s + 1);

                                // + - timezone
                                data[6] = 0;
                                char *tzs = strchr(s + 1, '-');
                                if (!tzs)
                                {
                                    tzs = strchr(s + 1, '+');
                                }

                                if (tzs)
                                {
                                    data[6] = atoi(tzs);
                                }

                                err = ESP_OK;
                            }
                        }
                    }
                }
            }
        }
    }
    else if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }

    return err;
}

esp_err_t esp_modem_dce_handle_quistate(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

    const char *ss = "+QISTATE:";

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (!strncmp(line, ss, strlen(ss)))
    {
        // printf("===================================================================");
        int32_t *data = esp_dce->priv_resource;

        char *s = strchr(line, ' ');
        if (s)
        {
            data[0] = atoi(s + 1);
            int n = 2;
            do
            {
                s = strchr(s + 1, ',');
            } while (s && n-- > 0);

            if (s)
            {
                data[1] = atoi(s + 1);
                s = strchr(s + 1, ',');
                if (s)
                {
                    data[2] = atoi(s + 1);
                    s = strchr(s + 1, ',');
                    if (s)
                    {
                        data[3] = atoi(s + 1);
                        s = strchr(s + 1, ',');
                        err = ESP_OK;
                    }
                }
            }
        }
    }
    return err;
}

esp_err_t esp_modem_dce_handle_response_cgpaddr(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

    const char *ss = "+CGPADDR:";

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (!strncmp(line, ss, strlen(ss)))
    {
        int32_t *data = esp_dce->priv_resource;

        char *s = strchr(line, ',');
        if (s)
        {
            data[0] = atoi(s + 1);
            s = strchr(s + 1, '.');
            if (s)
            {
                data[1] = atoi(s + 1);
                s = strchr(s + 1, '.');
                if (s)
                {
                    data[2] = atoi(s + 1);
                    s = strchr(s + 1, '.');
                    if (s)
                    {
                        data[3] = atoi(s + 1);
                        err = ESP_OK;
                    }
                }
            }
        }
    }
    return err;
}

esp_err_t esp_modem_dce_handle_response_qisend(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

    const char *ss = "+QISEND:";

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (!strncmp(line, ss, strlen(ss)))
    {
        int32_t *data = esp_dce->priv_resource;

        char *s = strchr(line, ' ');
        if (s)
        {
            data[0] = atoi(s + 1);
            s = strchr(s + 1, ',');
            if (s)
            {
                data[1] = atoi(s + 1);
                s = strchr(s + 1, ',');
                if (s)
                {
                    data[2] = atoi(s + 1);
                    err = ESP_OK;
                }
            }
        }
    }
    return err;
}

esp_err_t esp_modem_dce_handle_response_all(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

    // printf("LINE: %s\nRESOURCE: %s\n", line, (const char *)esp_dce->priv_resource);

    if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (!strncmp(line, (const char *)esp_dce->priv_resource, strlen((const char *)esp_dce->priv_resource)))
    {
        // err = esp_modem_process_command_done(dce, MODEM_STATE_PROCESSING);
        // printf("===================================================================");
        err = ESP_OK;
    }
    return err;
}

esp_err_t esp_modem_dce_handle_response_wait(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

    // printf("LINE: %s\nRESOURCE: %s\n", line, (const char *)esp_dce->priv_resource);

    if (!strncmp(line, (const char *)esp_dce->priv_resource, strlen((const char *)esp_dce->priv_resource)))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    }
    else if (strstr(line, MODEM_RESULT_CODE_ERROR))
    {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    else if (strstr(line, MODEM_RESULT_CODE_SUCCESS))
    {
        err = ESP_OK;
    }

    return err;
}

void radio_task(void *arg)
{

    BaseType_t xResult;
    uint32_t ulNotifiedValue;

    esp_err_t ret = ESP_OK;

    read_nvs_nbiot(&id, apn, serverip, &port);

    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "start");

    /*
     * Configure CPU hardware to communicate with the radio chip
     */
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << POWER_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* create dte object */
    esp_modem_dte_config_t config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    config.event_task_priority = 6,
    config.event_task_stack_size = 2048 * 2;
    config.tx_io_num = TX_GPIO;
    config.rx_io_num = RX_GPIO;
    config.rts_io_num = -1;
    config.cts_io_num = -1;

    modem_dte_t *dte = esp_modem_dte_init(&config);

    while (1)
    {
        ESP_LOGI(TAG, "init module NB-IoT");

        modem_status[0] = 0;

        gpio_set_level(POWER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(600));
        gpio_set_level(POWER_GPIO, 0);
        // vTaskDelay(pdMS_TO_TICKS(400));

        modem_dce_t *dce = NULL;
        dce = bc26_init(dte);

        if (dce != NULL)
        {
            // dce->handle_line = esp_modem_dce_handle_response_default;
            // dte->send_cmd(dte, "AT+QLEDMODE=0\r", MODEM_COMMAND_TIMEOUT_DEFAULT);
            // dce->handle_line = esp_modem_dce_handle_response_default;
            // dte->send_cmd(dte, "AT+CEREG=3\r", MODEM_COMMAND_TIMEOUT_DEFAULT);

            /* Print Module ID, Operator, IMEI, IMSI */
            ESP_LOGI(TAG, "Module: %s", dce->name);
            ESP_LOGI(TAG, "Operator: %s", dce->oper);
            ESP_LOGI(TAG, "IMEI: %s", dce->imei);
            ESP_LOGI(TAG, "IMSI: %s", dce->imsi);

            /* Get signal quality */
            ESP_ERROR_CHECK_WITHOUT_ABORT(dce->get_signal_quality(dce, &rssi, &ber));
            if (rssi == 99 || ber == 99)
            {
                ESP_LOGE(TAG, "rssi: Not know, ber: Not know");
            }
            else
            {
                ESP_LOGI(TAG, "rssi: %ddBm, ber: %d", (int)rssi * 2 + -113, ber);
            }

            /* Get battery voltage */
            uint32_t voltage = 0, bcs = 0, bcl = 0;
            ESP_ERROR_CHECK_WITHOUT_ABORT(dce->get_battery_status(dce, &bcs, &bcl, &voltage));
            ESP_LOGI(TAG, "Battery voltage: %d mV", voltage);

            // vTaskDelay(pdMS_TO_TICKS(1000));

            // dce->handle_line = esp_modem_dce_handle_response_print;
            // dte->send_cmd(dte, "AT+CGPADDR\r", MODEM_COMMAND_TIMEOUT_DEFAULT);

            // dce->handle_line = esp_modem_dce_handle_response_print;
            // dte->send_cmd(dte, "AT+CPSMS?\r", MODEM_COMMAND_TIMEOUT_DEFAULT);

            // uint32_t dat[8];
            // dat[0] = 55;
            // dce->handle_line = esp_modem_dce_handle_response_default;
            // dte->send_cmd(dte, "AT+CCLK?\r", 1000);

            // printf("1test: %ld\n", dat[0]);
            /*
                        uint32_t dat[8];
                        dat[0] = 1;
                        printf("0test: %ld\n", dat[0]);
                        esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);
                        esp_dce->priv_resource = dat;
                        printf("1test: %ld\n", dat[0]);
                        dce->handle_line = esp_modem_dce_handle_cclk;
                        dte->send_cmd(dte, "AT+CCLK?\r", MODEM_COMMAND_TIMEOUT_DEFAULT);
                        //printf("%4d/%2d/%2d,%2d:%2d:%2dGMT%+2d", dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6]);
            */
            // dce->handle_line = esp_modem_dce_handle_response_print;
            // dte->send_cmd(dte, "AT+QPING=1,\"10.179.40.11\"\r", 15000);

            // ESP_LOGI(TAG, "AT+QISTATE?");

            esp_modem_dce_t *esp_dce = __containerof(dce, esp_modem_dce_t, parent);

            // Show PDP Addresses
            int32_t pdpaddr[4] = {0, 0, 0, 0};
            esp_dce->priv_resource = pdpaddr;
            dce->handle_line = esp_modem_dce_handle_response_cgpaddr;
            dte->send_cmd(dte, "AT+CGPADDR?\r", MODEM_COMMAND_TIMEOUT_DEFAULT);

            snprintf(modem_status, sizeof(modem_status), "Operator: %s, IMEI: %s, IMSI: %s, rssi: %ddBm, ber: %d, IP:%d.%d.%d.%d", dce->oper, dce->imei, dce->imsi, (int)rssi * 2 + -113, ber, pdpaddr[0], pdpaddr[1], pdpaddr[2], pdpaddr[3]);

            char tx_buf[1024];

            char buf[1024];

            // connectID, <local_port>,<socket_state
            int32_t connectID[4] = {-1, 0, 0, 0};

            // vTaskDelay(5000 / portTICK_PERIOD_MS);

            int counter = 10;

            // wait connect
            do
            {
                dce->sync(dce);

                connectID[0] = -1;
                esp_dce->priv_resource = connectID;
                dce->handle_line = esp_modem_dce_handle_quistate;
                dte->send_cmd(dte, "AT+QISTATE?\r", MODEM_COMMAND_TIMEOUT_DEFAULT);

                ESP_LOGI(TAG, "try %d, connectID: %d, %d, %d, %d", counter, connectID[0], connectID[1], connectID[2], connectID[3]);

                if (connectID[0] == -1)
                {
                    ESP_LOGI(TAG, "Open port");
                    esp_dce->priv_resource = "+QIOPEN:";
                    dce->handle_line = esp_modem_dce_handle_response_wait;
                    snprintf(tx_buf, sizeof(tx_buf), "AT+QIOPEN=1,0,\"UDP\",\"%s\",%d,0,0\r", serverip, port);

                    // 2. It is recommended to wait for 60 seconds for URC response “+QIOPEN: <connectID>,<err>”.
                    dte->send_cmd(dte, tx_buf, 30000);
                }

                if (dce->state != MODEM_STATE_SUCCESS)
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }

            } while (connectID[3] != 2 && counter-- > 0);

            int32_t dt[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            char datetime[22] = "";
            esp_dce->priv_resource = dt;
            dce->handle_line = esp_modem_dce_handle_cclk;
            dte->send_cmd(dte, "AT+CCLK?\r", MODEM_COMMAND_TIMEOUT_DEFAULT);
            if (dce->state == MODEM_STATE_SUCCESS)
            {
                snprintf((char *)datetime, sizeof(datetime), "%04d-%02d-%02d %02d:%02d:%02d", dt[0], dt[1], dt[2], dt[3] + dt[6], dt[4], dt[5]);
                ESP_LOGI(TAG, "Current date/time: %s", datetime);
            }

            int len_data = 0;
            if (pdTRUE == xQueueReceive(send_queue, &result, 5000 / portTICK_PERIOD_MS))
            {
                len_data = snprintf((char *)buf, sizeof(buf), "{\"id\":%d,\"num\":%d,\"dt\":\"%s\",\"U\":%d,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%d,\"T\":%.1f,\"rssi\":%d}", id, bootCount, datetime, result.U, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0, tsens_out, (int)rssi * 2 + -113);

                ESP_LOGI(TAG, "Send data: %s", buf);

                xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);

                esp_dce->priv_resource = "SEND";
                dce->handle_line = esp_modem_dce_handle_response_wait;
                snprintf(tx_buf, sizeof(tx_buf), "AT+QISEND=0,%d,%s\r", len_data, buf);
                dte->send_cmd(dte, tx_buf, 60000);

                /*
                                //ждем окончания передачи
                                counter = 10;
                                while (counter--)
                                {
                                    int32_t bytes_len[3] = {-1, 0, 0};
                                    esp_dce->priv_resource = bytes_len;
                                    dce->handle_line = esp_modem_dce_handle_response_qisend;
                                    dte->send_cmd(dte, "AT+QISEND=0,0\r", MODEM_COMMAND_TIMEOUT_DEFAULT);

                                    if (bytes_len[0] == bytes_len[2])
                                    {
                                        ESP_LOGI(TAG, "Send data OK");
                                        break;
                                    }
                                }
                */
            }

            xEventGroupSetBits(ready_event_group, END_TRANSMIT);

            /*
                        xResult = xTaskNotifyWait(pdFALSE,
                                                  ULONG_MAX,
                                                  &ulNotifiedValue,
                                                  (portTickType)1000 / portTICK_PERIOD_MS);
            */

            // dce->sync(dce);

            ESP_LOGI(TAG, "Power down");
            ESP_ERROR_CHECK_WITHOUT_ABORT(dce->power_down(dce));

            vTaskDelay(pdMS_TO_TICKS(300000));

            ESP_ERROR_CHECK_WITHOUT_ABORT(dce->deinit(dce));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}