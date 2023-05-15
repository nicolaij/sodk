#include <stdio.h>
#include "lora.h"
#include "driver/gpio.h"
#include <sys/time.h>

#include "nvs_flash.h"
#include "nvs.h"

#include <main.h>

uint8_t buf[WS_BUF_SIZE];

// 4. Передача разрешена только в полосах 865,6-865,8 МГц, 866,2-866,4 МГц, 866,8-867,0 МГц и 867,4-867,6 МГц.
int32_t fr = 867500; // frequency kHz
int32_t bw = 7;      // Номер полосы
int32_t sf = 7;      // SpreadingFactor
int32_t op = 17;     // OutputPower
int32_t id = 1;      // ID передатчика

static const char *TAG = "radio";

extern float tsens_out;

/*
params int32 - список параметров
return кол-во элементов
*/
int read_nvs_lora(int32_t *id, int32_t *fr, int32_t *bw, int32_t *sf, int32_t *op)
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
    int ver = 0;

    read_nvs_lora(&id, &fr, &bw, &sf, &op);

    ver = lora_init();

    lora_set_frequency(fr * 1000);
    lora_set_bandwidth_n(bw);
    lora_set_spreading_factor(sf);
    lora_set_tx_power(op);

    lora_enable_crc();
    // lora_disable_crc();
    // lora_dump_registers();

    ESP_LOGI(TAG, "lora start");

    //результат измерений
    result_t result;
    BaseType_t xResult;
    uint32_t ulNotifiedValue;

    lora_receive(); // put into receive mode

    while (1)
    {
        /* Ожидание оповещения от прерываний. */
        xResult = xTaskNotifyWait(pdFALSE,          /* Не очищать биты на входе. */
                                  ULONG_MAX,        /* На выходе очищаются все биты. */
                                  &ulNotifiedValue, /* Здесь хранится значение оповещения. */
                                  (TickType_t)0); /* Время таймаута на блокировке. */
        if (xResult == pdPASS)
        {
            /* Было получено оповещение. Проверка, какие биты установлены. */
            if ((ulNotifiedValue & RESET_BIT) != 0)
            {
                lora_set_frequency(fr * 1000);
                lora_set_bandwidth_n(bw);
                lora_set_spreading_factor(sf);
                lora_set_tx_power(op);
                
                lora_receive(); // put into receive mode
            }

            if ((ulNotifiedValue & SLEEP_BIT) != 0)
            {
                ESP_LOGI(TAG, "lora_sleep");
                lora_sleep();
                xEventGroupSetBits(ready_event_group, END_RADIO_SLEEP);
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
        }

        while (lora_received())
        {
            x = lora_receive_packet(buf, sizeof(buf) - 28);
            buf[x] = 0;
            // printf("Received: \"%s\" Len: %d RSSI: %d\n", buf, x, rssi);
            if (x > 10 && x < sizeof(buf))
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
            vTaskDelay(10 / portTICK_PERIOD_MS);
            // lora_send_packet((uint8_t *)buf, strlen((char*)buf));
            // vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (pdTRUE == xQueueReceive(send_queue, &result, (TickType_t)0))
        {
            int l = sprintf((char *)buf, "{\"id\":%d,\"num\":%d,\"U\":%d,\"R\":%d,\"Ub1\":%.3f,\"Ub0\":%.3f,\"U0\":%d,\"T\":%.1f}", id, bootCount, result.U, result.R, result.Ubatt1 / 1000.0, result.Ubatt0 / 1000.0, result.U0, tsens_out);
            printf("%s\n", buf);
            xQueueSend(ws_send_queue, (char *)buf, (TickType_t)0);
            if (ver == 0x12)
                lora_send_packet((uint8_t *)buf, l);
                
            lora_receive(); // put into receive mode
            xEventGroupSetBits(ready_event_group, END_TRANSMIT);
        }
       
        vTaskDelay(10);
    }
}
