#ifndef MAIN_H_
#define MAIN_H_

#include "esp_idf_version.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG - не работает
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "freertos/ringbuf.h"

#include <sys/time.h>

// размер кольцевого буфера (только степень 2)
#define RINGBUFLEN (1 << 11)

#define ADC_COUNT_READ 5
#define ADC_FREQ (10000 * ADC_COUNT_READ)
#define ADC_BUFFER (ADC_FREQ / 1000 * 4) // размер буфера данных для выборки 1 mc

#define RESET_BIT 0x1
#define SLEEP_BIT 0x2

#define TXD_PIN GPIO_NUM_6
#define RXD_PIN GPIO_NUM_7

#define ENABLE_PIN GPIO_NUM_19
#define BTN_PIN GPIO_NUM_9

#define MODEM_POWER GPIO_NUM_18

#define PCF_INT_PIN GPIO_NUM_5

#define I2C_MASTER_SDA_PIN GPIO_NUM_8
#define I2C_MASTER_SCL_PIN GPIO_NUM_10

#define POWER_BIT 8
#define NB_PWR_BIT 9
#define PSM_BIT 10
#define LV_BIT 13
#define LV_POWER_BIT 12
#define IN1_BIT 15

#define NB_PWR_CMDON 200
#define NB_PWR_CMDOFF 201
#define POWER_CMDON 202
#define POWER_CMDOFF 203
#define NB_RESET_CMD 205
#define LV_CMDON 302
#define LV_CMDOFF 303
#define LV_MEASUREON 304
#define LV_MEASUREOFF 305

#define WS_BUF_SIZE 160

#define StoUS(s) (s * 1000000LL)

typedef struct
{
    short int cmd;     // причина измерения 1 - по таймеру низковольные, 2 - по таймеру высоковольные, 3- прерывание от PCF
    short int channel; // канал измерения 1-4, 5-8
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
    int U0;
    int Ubatt0;
    int Ubatt1;
    int time;
    int input;
    time_t ttime;
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
    const char name[64];
    const char izm[8];
    int val;
    const int min;
    const int max;
} menu_t;

extern QueueHandle_t uicmd_queue;
extern QueueHandle_t send_queue;
extern QueueHandle_t set_lora_queue;
extern RingbufHandle_t wsbuf_handle;

extern EventGroupHandle_t status_event_group;

extern TaskHandle_t xHandleNB;

// Конец измерений и флаги
#define END_MEASURE BIT0
#define END_TRANSMIT BIT1
#define END_WIFI_TIMEOUT BIT2
#define END_RADIO BIT3
#define END_UI_SLEEP BIT4
#define NEED_TRANSMIT BIT5
#define END_WORK_NBIOT BIT6
#define END_WIFI BIT7
#define SERIAL_TERMINAL_ACTIVE BIT8
#define NB_TERMINAL BIT9
#define WIFI_ACTIVE BIT10

#define NOTYFY_WIFI BIT0
#define NOTYFY_WIFI_STOP BIT1
#define NOTYFY_WIFI_ESPNOW BIT2
#define NOTYFY_WIFI_REBOOT BIT3

#define OUT_JSON_ADD_COMMON ",\"NBPower\":%.3f,\"RSSI\":%i,\"Temp\":%.01f}"

#define OUT_JSON_CHANNEL "{\"id\":\"sodk%d.%d\",\"num\":%u,\"dt\":\"%s\",\"U\":%d,\"R\":%d,\"U0\":%d,\"Ubatt1\":%d}"
#define OUT_DATA_CHANNEL(prefix) prefix.U, prefix.R, prefix.U0, prefix.Ubatt1

extern int bootCount;
extern int terminal_mode;
extern unsigned int BattLow;

void adc_task(void *arg);
void btn_task(void *arg);
void wifi_task(void *arg);
void modem_task(void *arg);
void console_task(void *arg);

void btn_task(void *arg);

int current(int adc);
int volt(int adc);
int kOm(int adc_u, int adc_r);
int kOm2chan(int adc_u, int adc_r);

int read_nvs_lora(int32_t *id, int32_t *fr, int32_t *bw, int32_t *sf, int32_t *op);
int write_nvs_lora(const char *key, int value);
int write_nvs_nbiot();

int read_nvs_menu(void);

void reset_sleep_timeout(void);

void cur_time(char *buf);

void processBuffer(uint8_t *endptr, uint8_t *ptr_0db, uint8_t *ptr_off, uint8_t *ptr_on, int channel);

void pcf8575_set(int channel_cmd);

// bit 0..15 - return bit
// bit -1 return all bits
int pcf8575_read(int bit);

/*
channel - номер канала, если 0 - то по списку menu[20]
flag - 0: lv,hv; 1:hv only, 2:lv only;
*/
void start_measure(int channel, int flag);

// получаем 1 строку данных data_pos
// возвращает длину строки
int getResult_Data(char *line, int data_pos);

/* получаем 1 строку данных data_pos
 возвращает длину строки
 mode 0 - adc, 1 - adc filter, 3 - adc to mV
*/
int getADC_Data(char *line, int data_pos, const int mode);

int get_menu_val_by_id(const char *id);
esp_err_t set_menu_val_by_id(const char *id, int value);

char *get_datetime(time_t ttime);
int get_menu_html(char *buf);

#endif
