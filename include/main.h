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
    int pwm;
} cmd_t;

typedef enum
{
    KEY_NONE = 0,
    KEY_CLICK,
    KEY_LONG_PRESS,
    KEY_DOUBLELONG_PRESS
} keys_events_t;

xQueueHandle uicmd_queue;
xQueueHandle adc1_queue;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);

int volt(int adc);
int kOm(int adc1, int adc2);
