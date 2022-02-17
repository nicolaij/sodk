#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "driver/gpio.h"
#include <sys/time.h>

#include "nvs_flash.h"
#include "nvs.h"

#define BTN_GPIO 0

uint8_t buf[256];

void radio_task(void *arg)
{
    int x;
    char *str = "SSSSS";

    lora_init();
    // lora_dump_registers();
    // lora_set_frequency(915e6);
    // 4. Передача разрешена только в полосах 865,6-865,8 МГц, 866,2-866,4 МГц, 866,8-867,0 МГц и 867,4-867,6 МГц.
    // lora_set_frequency(867.5E6);
    // lora_set_bandwidth(125E3);
    //
    int fr = 867500; // frequency kHz
    int bw = 7;      // Номер полосы
    int sf = 7;      // SpreadingFactor

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("lora", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_i32(my_handle, "fr", &fr);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "fr", fr);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "fr");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_i32(my_handle, "bw", &bw);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "bw", bw);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "bw");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        err = nvs_get_i32(my_handle, "sf", &sf);
        switch (err)
        {
        case ESP_OK:
            printf("Read \"%s\" = %d\n", "sf", sf);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value  \"%s\" is not initialized yet!\n", "sf");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        // Close
        nvs_close(my_handle);
    }

    lora_set_frequency(fr * 1000);
    lora_set_bandwidth_n(bw);
    lora_set_spreading_factor(sf);

    lora_enable_crc();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    lora_dump_registers();
    // xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);

    gpio_pad_select_gpio(BTN_GPIO);
    gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_GPIO, GPIO_PULLUP_ONLY);

    while (1)
    {
        lora_receive(); // put into receive mode
        while (lora_received())
        {
            x = lora_receive_packet(buf, sizeof(buf));
            buf[x] = 0;
            printf("Received: \"%s\" Len: %d, RSSI: %d\n", buf, x, lora_packet_rssi());
            lora_receive();
            // lora_send_packet((uint8_t *)"Reply.", 5);
        }

        if (gpio_get_level(BTN_GPIO) == 0)
        {

            time_t now;
            struct tm timeinfo;
            uint8_t buf[64];
            int l;

            lora_set_tx_power(2);
            time(&now);
            localtime_r(&now, &timeinfo);
            l = strftime((char *)buf, sizeof(buf), "%c", &timeinfo);
            lora_send_packet((uint8_t *)buf, l);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            lora_set_tx_power(10);
            time(&now);
            localtime_r(&now, &timeinfo);
            l = strftime((char *)buf, sizeof(buf), "%c", &timeinfo);
            lora_send_packet((uint8_t *)buf, l);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            lora_set_tx_power(17);
            time(&now);
            localtime_r(&now, &timeinfo);
            l = strftime((char *)buf, sizeof(buf), "%c", &timeinfo);
            lora_send_packet((uint8_t *)buf, l);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            printf("Send:\"%s\"\n", str);
        }

        vTaskDelay(1);
    }
}
