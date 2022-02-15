#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "driver/gpio.h"
#include <sys/time.h>

#define BTN_GPIO 0

uint8_t buf[256];

void task_rx(void *p)
{
    int x;

    char *str = "SSSSS";

    for (;;)
    {
        lora_receive(); // put into receive mode
        while (lora_received())
        {
            x = lora_receive_packet(buf, sizeof(buf));
            buf[x] = 0;
            printf("Received: \"%s\"\n", buf);
            lora_receive();
            // lora_send_packet((uint8_t *)"Reply.", 5);
        }

        vTaskDelay(1);

        if (gpio_get_level(BTN_GPIO) == 0)
        {
            printf("Send:\"%s\"\n", str);
            lora_send_packet((uint8_t *)str, 5);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
}

void radio_task(void *arg)
{
    int x;
    char *str = "SSSSS";

    lora_init();
    //lora_dump_registers();
    //lora_set_frequency(915e6);
    //4. Передача разрешена только в полосах 865,6-865,8 МГц, 866,2-866,4 МГц, 866,8-867,0 МГц и 867,4-867,6 МГц.
    lora_set_frequency(867.5E6);
    lora_set_bandwidth(125E3);
    //
    lora_enable_crc();
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
