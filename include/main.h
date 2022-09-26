#pragma once

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

#define DATALEN 20000

#define I2C_MASTER_NUM 0          //  I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 400000 //  I2C master clock frequency

#define RESET_BIT 0x1
#define SLEEP_BIT 0x2

#if CONFIG_IDF_TARGET_ESP32

#define PIN_SDA 22
#define PIN_SCL 23

#define PIN_ENCODER_A 19
#define PIN_ENCODER_B 18
#define PIN_ENCODER_BTN 21

#define POWER_PIN 32
#define BTN_GPIO 0

#elif CONFIG_IDF_TARGET_ESP32C3

#define ENABLE_PIN 19

#if MULTICHAN
    #define I2C_MASTER_SDA_PIN 8
    #define I2C_MASTER_SCL_PIN 10
    #define POWER_BIT 8
    #define NB_PWR_BIT 9
    #define PSM_BIT 10
#else
    #define POWER_PIN 18
    #define BTN_GPIO 9
#endif

#endif

#define WS_BUF_LINE 128

typedef struct
{
    int cmd;
    int power;
} cmd_t;

typedef struct
{
    int adc0;
    int adc1;
    int R;
    int adc2;
    int U;
    int Ubatt0;
    int Ubatt1;
    int U0;
} result_t;

#define PWM_MIN 0
#define PWM_MAX 255

typedef enum
{
    KEY_NONE = 0,
    KEY_CLICK,
    PRE_KEY_CLICK,
    KEY_DOUBLECLICK,
    KEY_LONG_PRESS,
    KEY_DOUBLELONG_PRESS
} keys_events_t;

typedef struct
{
    const char id[10];
    const char name[32];
    int32_t val;
    int32_t min;
    int32_t max;
} menu_t;

QueueHandle_t uicmd_queue;
QueueHandle_t adc1_queue;
QueueHandle_t send_queue;
QueueHandle_t set_lora_queue;

QueueHandle_t ws_send_queue;

// SemaphoreHandle_t i2c_mux;

EventGroupHandle_t ready_event_group;

//Конец измерений
#define END_MEASURE BIT0
#define END_TRANSMIT BIT1
#define END_WIFI_TIMEOUT BIT2
#define END_LORA_SLEEP BIT3
#define END_UI_SLEEP BIT4

extern RTC_DATA_ATTR int bootCount;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);
void clock_task(void *arg);
void wifi_task(void *arg);
void radio_task(void *arg);

int current(int adc);
int volt(int adc);
int kOm(int adc_u, int adc_r);
int kOm0db(int adc_u, int adc_r);

int read_nvs_lora(int32_t *id, int32_t *fr, int32_t *bw, int32_t *sf, int32_t *op);
int write_nvs_lora(const char *key, int value);
int write_nvs_nbiot(int32_t *pid, const char *apn, const char *server, const uint16_t *port);

void go_sleep(void);

int read_nvs_menu(void);

void reset_sleep_timeout(void);

void cur_time(char *buf);

void processBuffer(uint8_t *endptr, uint8_t *ptr_0db, uint8_t *ptr_off, uint8_t *ptr_on);

int getADC_Data(char *line, uint8_t **ptr_adc, int *num);

void power_on(int channel);
void power_off(void);

