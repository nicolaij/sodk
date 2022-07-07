#include <stdio.h>
#include "driver/gpio.h"
#include <sys/time.h>
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <main.h>

#define TX_GPIO 2
#define RX_GPIO 7
#define POWER_GPIO 6

#define RX_BUF_SIZE 1024

char rx_buf[RX_BUF_SIZE];

uint8_t buf[WS_BUF_LINE];

char apn[32] = {"mogenergo"};
char server[32] = {"10.179.40.11"};
uint16_t port = 48884;
int32_t id = 1; // ID передатчика

static const char *TAG = "nbiot";

extern float tsens_out;

int read_nvs_nbiot(int32_t *pid, char *apn, char *server, uint16_t *port)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("nbiot", NVS_READONLY, &my_handle);
    size_t l;
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_i32(my_handle, "id", pid);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "id", *pid);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "id");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_str(my_handle, "apn", apn, &l);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %s\n", "apn", apn);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "apn");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_str(my_handle, "server", server, &l);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %s\n", "server", server);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "server");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        int32_t i = 0;
        err = nvs_get_i32(my_handle, "port", &i);
        switch (err)
        {
        case ESP_OK:
            *port = i;
            printf("Read \"%s\" = %d\n", "port", *port);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "port");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
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
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        // Write
        printf("Write id: %d ", *pid);
        err = nvs_set_i32(my_handle, "id", *pid);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Write APN: \"%s\" ", apn);
        err = nvs_set_str(my_handle, "apn", apn);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Write server: \"%s\" ", server);
        err = nvs_set_str(my_handle, "server", server);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Write port: %d ", *port);
        int32_t i = *port;
        err = nvs_set_i32(my_handle, "port", i);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }
    return 1;
};

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_delete(UART_NUM_1);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TX_GPIO, RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int ATcmd(char *cmd, char *resbuf, uint32_t length, TickType_t ticks_to_wait)
{
    char atcmd[32];
    strlcpy(atcmd, cmd, sizeof(atcmd));
    strlcat(atcmd, "\r\n", sizeof(atcmd));
    const int txBytes = uart_write_bytes(UART_NUM_1, atcmd, strlen(atcmd));
    // ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
    resbuf[0] = 0;
    int read = uart_read_bytes(UART_NUM_1, resbuf, length, pdMS_TO_TICKS(ticks_to_wait));
    if (read > 0)
    {
        resbuf[read] = 0;
        ESP_LOGI(TAG, "Read: \"%s\"", rx_buf);
        char *ptr = strnstr(resbuf, "\nOK\r", read);
        if (ptr == NULL)
            return -1;
        else
            return (ptr - resbuf);
    }

    return -10;
}

void radio_task(void *arg)
{
    int x;

    read_nvs_nbiot(&id, apn, server, &port);

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

    while (1)
    {
        ESP_LOGI(TAG, "init module NB-IoT");

        while (1)
        {
            gpio_set_level(POWER_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(1100));
            gpio_set_level(POWER_GPIO, 0);

            uart_init();
            uart_read_bytes(UART_NUM_1, rx_buf, RX_BUF_SIZE, pdMS_TO_TICKS(500));

            if (ATcmd("AT", rx_buf, RX_BUF_SIZE, 500) > 0)
                break;
            if (ATcmd("AT", rx_buf, RX_BUF_SIZE, 500) > 0)
                break;
            if (ATcmd("AT", rx_buf, RX_BUF_SIZE, 500) > 0)
                break;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        //результат измерений
        result_t result;
        BaseType_t xResult;
        uint32_t ulNotifiedValue;

        // work witch module
        while (1)
        {
            /* Ожидание оповещения от прерываний. */
            xResult = xTaskNotifyWait(pdFALSE,          /* Не очищать биты на входе. */
                                      ULONG_MAX,        /* На выходе очищаются все биты. */
                                      &ulNotifiedValue, /* Здесь хранится значение оповещения. */
                                      (portTickType)2); /* Время таймаута на блокировке. */
            if (xResult == pdPASS)
            {
                /* Было получено оповещение. Проверка, какие биты установлены. */
                if ((ulNotifiedValue & RESET_BIT) != 0)
                {
                }

                if ((ulNotifiedValue & SLEEP_BIT) != 0)
                {
                    ESP_LOGI(TAG, "lora_sleep");
                    // lora_sleep();
                    xEventGroupSetBits(ready_event_group, END_LORA_SLEEP);
                    vTaskDelay(10000 / portTICK_PERIOD_MS);
                }
            }
            /*
                    lora_receive(); // put into receive mode

                    while (lora_received())
                    {
                        x = lora_receive_packet(buf, sizeof(buf));
                        buf[x] = 0;
                        // printf("Received: \"%s\" Len: %d RSSI: %d\n", buf, x, rssi);
                        if (x > 10 && x < 200)
                        {
                            if (buf[x - 1] == '}')
                            {
                                sprintf((char *)&buf[x - 1], ",\"rssi\":%d,\"snr\":%.2f}", lora_packet_rssi(), lora_packet_snr());
                            }
                            else
                            {
                                sprintf((char *)&buf[x], " - RSSI:%d; - SNR:%.2f", lora_packet_rssi(), lora_packet_snr());
                            }
                            printf("%s\n", buf);
                            // fflush(stdout);
                        }

                        // xQueueSend(ws_send_queue, (char *)buf, (portTickType)1);
                        lora_receive();
                        // lora_idle();
                        // vTaskDelay(100 / portTICK_PERIOD_MS);
                        // lora_send_packet((uint8_t *)buf, strlen((char*)buf));
                        // vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }

                    if (pdTRUE == xQueueReceive(send_queue, &result, (portTickType)0))
                    {
                        int l = sprintf((char *)buf, "{\"id\":%d,\"num\":%d,\"U\":%d,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%d,\"T\":%.1f}", id, bootCount, result.U, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0, tsens_out);
                        printf("%s\n", buf);
                        xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
                        if (ver == 0x12)
                            lora_send_packet((uint8_t *)buf, l);
                        xEventGroupSetBits(ready_event_group, END_TRANSMIT);
                    }

                    gpio_set_pull_mode(BTN_GPIO, GPIO_PULLUP_ONLY);
                    gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);

                    if (gpio_get_level(BTN_GPIO) == 0)
                    {
                        time_t now;
                        struct tm timeinfo;

                        time(&now);
                        localtime_r(&now, &timeinfo);
                        size_t l = strftime((char *)buf, sizeof(buf), "%c", &timeinfo);
                        // lora_send_packet((uint8_t *)buf, l);
                        printf("Press BTN:\"%s\"\n", buf);

                        cmd_t cmd;
                        cmd.cmd = 10;
                        cmd.power = 255;

                        xQueueSend(uicmd_queue, &cmd, (portTickType)0);

                        vTaskDelay(1000 / portTICK_PERIOD_MS);

                        // xQueueSend(ws_send_queue, "Проверка связи...", (portTickType)0);
                    }

                    gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
            */
            if (ATcmd("ATI", rx_buf, RX_BUF_SIZE, 500) <= 0) break;
            ATcmd("AT+CFUN?", rx_buf, RX_BUF_SIZE, 500); //Value is 1 (full functionality)
            ATcmd("AT+CIMI", rx_buf, RX_BUF_SIZE, 500); //Query the IMSI number.
            ATcmd("AT+CESQ", rx_buf, RX_BUF_SIZE, 500); //Extended Signal Quality

            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
