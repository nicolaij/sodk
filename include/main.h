#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define POWER_PIN 32

#define PIN_SDA 22
#define PIN_SCL 23

#define PIN_ENCODER_A 19
#define PIN_ENCODER_B 18
#define PIN_ENCODER_BTN 21

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
#define PWM_MAX 100

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

xQueueHandle uicmd_queue;
xQueueHandle adc1_queue;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);
void clock_task(void *arg);
void wifi_task(void *arg);
void radio_task(void *arg);

int volt(int adc);
int kOm(int adc_u, int adc_r, int channel_r);
