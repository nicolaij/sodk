#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

xQueueHandle uicmd_queue;
xQueueHandle adc1_queue;
xQueueHandle adc2_queue;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);



