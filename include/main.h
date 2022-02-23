#pragma once

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#define POWER_PIN 32

#define I2C_MASTER_NUM 0          //  I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 400000 //  I2C master clock frequency
#define PIN_SDA 22
#define PIN_SCL 23

#define PIN_ENCODER_A 19
#define PIN_ENCODER_B 18
#define PIN_ENCODER_BTN 21

#define RESET_BIT 0x1
#define SLEEP_BIT 0x2

typedef struct
{
    int cmd;
    int power;
} cmd_t;

typedef struct
{
    int adc11;
    int adc12;
    int adc2;
    int R;
    int U;
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
    char name[32];
    int val;
    int min;
    int max;
} menu_t;

QueueHandle_t uicmd_queue;
QueueHandle_t adc1_queue;
QueueHandle_t send_queue;
QueueHandle_t set_lora_queue;

QueueHandle_t ws_send_queue;

SemaphoreHandle_t i2c_mux;

EventGroupHandle_t ready_event_group;

extern RTC_DATA_ATTR int bootCount;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);
void clock_task(void *arg);
void wifi_task(void *arg);
void radio_task(void *arg);

int volt(int adc);
int kOm(int adc_u, int adc_r);

int read_nvs_lora(int *id, int *fr, int *bw, int *sf, int *op);
int write_nvs_lora(const char *key, int value);

void go_sleep(void);
