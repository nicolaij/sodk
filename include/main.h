#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define POWER_PIN 32

typedef struct
{
    int cmd;
    int pwm;
} cmd_t;

xQueueHandle uicmd_queue;
xQueueHandle adc1_queue;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);

int volt(int adc);
int kOm(int adc1, int adc2);
