
#ifndef __LORA_H__
#define __LORA_H__

#include "stdint.h"

#if CONFIG_IDF_TARGET_ESP32
#define CONFIG_RST_GPIO 2
#define CONFIG_CS_GPIO 5
#define CONFIG_MISO_GPIO 17
#define CONFIG_MOSI_GPIO 16
#define CONFIG_SCK_GPIO 15

#elif CONFIG_IDF_TARGET_ESP32C3
#define CONFIG_RST_GPIO 10
#define CONFIG_CS_GPIO 9
#define CONFIG_MISO_GPIO 2
#define CONFIG_MOSI_GPIO 7
#define CONFIG_SCK_GPIO 6

#endif

void lora_reset(void);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(int size);
void lora_idle(void);
void lora_sleep(void); 
void lora_receive(void);
void lora_set_tx_power(int level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(int sf);
void lora_set_bandwidth(long sbw);
void lora_set_bandwidth_n(long bw);
void lora_set_coding_rate(int denominator);
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc(void);
void lora_disable_crc(void);
int lora_init(void);
void lora_send_packet(uint8_t *buf, int size);
int lora_receive_packet(uint8_t *buf, int size);
int lora_received(void);
int lora_packet_rssi(void);
float lora_packet_snr(void);
void lora_close(void);
int lora_initialized(void);
void lora_dump_registers(void);

#endif
