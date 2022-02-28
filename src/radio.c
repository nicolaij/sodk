#include <stdio.h>
#include "lora.h"
#include "driver/gpio.h"
#include <sys/time.h>

#include "nvs_flash.h"
#include "nvs.h"

#include <esp_log.h>

#include <main.h>

#define BTN_GPIO 0

uint8_t buf[256];

// 4. Передача разрешена только в полосах 865,6-865,8 МГц, 866,2-866,4 МГц, 866,8-867,0 МГц и 867,4-867,6 МГц.
int fr = 867500; // frequency kHz
int bw = 7;      // Номер полосы
int sf = 7;      // SpreadingFactor
int op = 17;     // OutputPower
int id = 1;      // ID передатчика

static const char *TAG = "radio";

/*
params int32 - список параметров
return кол-во элементов
*/
int read_nvs_lora(int *id, int *fr, int *bw, int *sf, int *op)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("lora", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("lora", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_i32(my_handle, "id", id);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "id", *id);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "id");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_i32(my_handle, "fr", fr);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "fr", *fr);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "fr");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_i32(my_handle, "bw", bw);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "bw", *bw);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "bw");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_i32(my_handle, "sf", sf);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "sf", *sf);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "sf");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_i32(my_handle, "op", op);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "op", *op);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "op");
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

int write_nvs_lora(const char *key, int value)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("lora", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("lora", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        // Write
        printf("Write: \"%s\" ", key);
        err = nvs_set_i32(my_handle, key, value);
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

void radio_task(void *arg)
{
    int x;

    read_nvs_lora(&id, &fr, &bw, &sf, &op);

    lora_init();

    lora_set_frequency(fr * 1000);
    lora_set_bandwidth_n(bw);
    lora_set_spreading_factor(sf);
    lora_set_tx_power(op);

    lora_enable_crc();
    // lora_disable_crc();
    //lora_dump_registers();

    gpio_pad_select_gpio(BTN_GPIO);
    gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_GPIO, GPIO_PULLUP_ONLY);

    //результат измерений
    result_t result;
    BaseType_t xResult;
    uint32_t ulNotifiedValue;

    while (1)
    {
        /* Ожидание оповещения от прерываний. */
        xResult = xTaskNotifyWait(pdFALSE,          /* Не очищать биты на входе. */
                                  ULONG_MAX,        /* На выходе очищаются все биты. */
                                  &ulNotifiedValue, /* Здесь хранится значение оповещения. */
                                  0);               /* Время таймаута на блокировке. */
        if (xResult == pdPASS)
        {
            /* Было получено оповещение. Проверка, какие биты установлены. */
            if ((ulNotifiedValue & RESET_BIT) != 0)
            {
                lora_set_frequency(fr * 1000);
                lora_set_bandwidth_n(bw);
                lora_set_spreading_factor(sf);
                lora_set_tx_power(op);
            }

            if ((ulNotifiedValue & SLEEP_BIT) != 0)
            {
                ESP_LOGI(TAG, "lora_sleep");
                lora_sleep();
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
        }

        lora_receive(); // put into receive mode
        while (lora_received())
        {
            x = lora_receive_packet(buf, sizeof(buf));
            buf[x] = 0;
            int rssi = lora_packet_rssi();
            printf("Received: \"%s\" Len: %d, RSSI: %d\n", buf, x, rssi);
            if (strlen((char *)buf) < 200)
            {
                sprintf((char *)&buf[x], " - RSSI: %d", rssi);
            }
            xQueueSend(ws_send_queue, (char *)buf, (portTickType)0);
            lora_receive();
            // lora_idle();
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // lora_send_packet((uint8_t *)buf, strlen((char*)buf));
            // vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (pdTRUE == xQueueReceive(send_queue, &result, (portTickType)1))
        {
            int l = sprintf((char *)buf, "{\"id\":%d,\"num\":%d,\"U\":%d,\"R\":%d}", id, bootCount, result.U, result.R);
            lora_send_packet((uint8_t *)buf, l);
            xEventGroupSetBits(ready_event_group, BIT1);
        }

        if (gpio_get_level(BTN_GPIO) == 0)
        {
            time_t now;
            struct tm timeinfo;
            int l;

            time(&now);
            localtime_r(&now, &timeinfo);
            l = strftime((char *)buf, sizeof(buf), "%c", &timeinfo);
            lora_send_packet((uint8_t *)buf, l);
            printf("Send:\"%s\"\n", buf);

            vTaskDelay(1000 / portTICK_PERIOD_MS); 

            // xQueueSend(ws_send_queue, "Проверка связи...", (portTickType)0);
        }

        vTaskDelay(1);
    }
}
