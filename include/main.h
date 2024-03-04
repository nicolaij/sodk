#ifndef MAIN_H_
#define MAIN_H_

#include "esp_idf_version.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG - не работает
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "freertos/ringbuf.h"

// размер кольцевого буфера (только степень 2)
#define RINGBUFLEN 4096

#define ADC_COUNT_READ 5
#define ADC_FREQ 50000
#define ADC_BUFFER (ADC_FREQ / 1000 * 4) // размер буфера данных для выборки 1 mc

#define RESET_BIT 0x1
#define SLEEP_BIT 0x2

#define ENABLE_PIN GPIO_NUM_19

#define BTN_PIN GPIO_NUM_9

#ifdef ADC1_ONLY
    #define INT_PIN GPIO_NUM_5
#else
    #define INT_PIN GPIO_NUM_2
#endif

#define I2C_MASTER_SDA_PIN GPIO_NUM_8
#define I2C_MASTER_SCL_PIN GPIO_NUM_10
#define POWER_BIT 8
#define NB_PWR_BIT 9
#define PSM_BIT 10
#define LV_BIT 13
#define LV_POWER_BIT 14
#define IN1_BIT 15
//#define NB_RESET_BIT 12

#define NB_PWR_CMDON 200
#define NB_PWR_CMDOFF 201
#define POWER_CMDON 202
#define POWER_CMDOFF 203
#define NB_RESET_CMD 205
#define LV_CMDON 206
#define LV_CMDOFF 207

#define WS_BUF_SIZE 160

#define StoUS(ms) ((ms)*1000000LL)

typedef struct
{
    int cmd;     // причина измерения 1 - по таймеру низковольные, 2 - по таймеру высоковольные, 
    int channel; // канал измерения 1-4, 5-8
} cmd_t;

typedef struct
{
    int R1;
    int R2;
    int U;
    int Ubatt;
    int U0;
} calcdata_t;

typedef struct
{
    int channel; // канал измерения 1..4(HV), 5..8(LV)
    int adc0;
    int adc1;
    int adc2;
    int R;
    int U;
    int Ubatt0;
    int Ubatt1;
    int U0;
    int time;
    int input;
} result_t;

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
    const char name[48];
    int32_t val;
    const int32_t min;
    const int32_t max;
} menu_t;

extern menu_t menu[28];

extern QueueHandle_t uicmd_queue;
extern QueueHandle_t send_queue;
extern QueueHandle_t set_lora_queue;
// QueueHandle_t ws_send_queue;
extern RingbufHandle_t wsbuf_handle;

// SemaphoreHandle_t i2c_mux;

extern EventGroupHandle_t ready_event_group;

// Конец измерений
#define END_MEASURE BIT0
#define END_TRANSMIT BIT1
#define END_WIFI_TIMEOUT BIT2
#define END_RADIO_SLEEP BIT3
#define END_UI_SLEEP BIT4

extern int bootCount;
extern int terminal_mode;

extern unsigned int BattLow;

void ui_task(void *arg);
void dual_adc(void *arg);
void btn_task(void *arg);
void clock_task(void *arg);
void wifi_task(void *arg);
void radio_task(void *arg);

void btn_task(void *arg);

int current(int adc);
int volt(int adc);
int kOm(int adc_u, int adc_r);
int kOm2chan(int adc_u, int adc_r);

int read_nvs_lora(int32_t *id, int32_t *fr, int32_t *bw, int32_t *sf, int32_t *op);
int write_nvs_lora(const char *key, int value);
int write_nvs_nbiot();

void go_sleep(void);

int read_nvs_menu(void);

void reset_sleep_timeout(void);

void cur_time(char *buf);

void processBuffer(uint8_t *endptr, uint8_t *ptr_0db, uint8_t *ptr_off, uint8_t *ptr_on, int channel);

void pcf8575_set(int channel_cmd);
int pcf8575_read(uint16_t bit);

/*
channel - номер канала, если 0 - то по списку menu[20]
flag - 0: lv,hv; 1:hv only, 2:lv only; 
*/
void start_measure(int channel, int flag);

// получаем 1 строку данных data_pos
// возвращает длину строки
int getResult_Data(char *line, int data_pos);

// получаем 1 строку данных data_pos
// возвращает длину строки
int getADC_Data(char *line, int data_pos, bool filter);

#endif
