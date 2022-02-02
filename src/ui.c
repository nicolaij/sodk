#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "u8g2_esp32_hal.h"
#include "ui.h"
#include "main.h"

#include "rotary_encoder.h"

#define PIN_SDA 22
#define PIN_SCL 23

#define PIN_ENCODER_A 19
#define PIN_ENCODER_B 18
#define PIN_ENCODER_BTN 21

static const char *TAG = "ui";

void ui_task(void *arg)
{
	int screen = 0;
	int menu_pos = 0;
	int menu_sel = 0;

	char buf[64];

	cmd_t cmd;
	cmd.cmd = 0;
	cmd.pwm = 0;

	// Rotary encoder underlying device is represented by a PCNT unit in this example
	uint32_t pcnt_unit = 0;

	// Create rotary encoder instance
	rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, PIN_ENCODER_A, PIN_ENCODER_B);
	rotary_encoder_t *encoder = NULL;
	ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

	// Filter out glitch (1us)
	ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

	// Start encoder
	ESP_ERROR_CHECK(encoder->start(encoder));

	int encoder_min = 0;
	int encoder_max = 100;
	int encoder_cor = 0;

	int enc_btn = 0;
	gpio_pad_select_gpio(PIN_ENCODER_BTN);
	gpio_set_direction(PIN_ENCODER_BTN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(PIN_ENCODER_BTN, GPIO_PULLUP_ONLY);

	// initialize the u8g2 hal
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda = PIN_SDA;
	u8g2_esp32_hal.scl = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	// initialize the u8g2 library
	u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);

	// set the display address
	u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

	// initialize the display
	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2);

	// wake up the display
	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0);

	// draw the hourglass animation, full-half-empty
	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_full);
	u8g2_SendBuffer(&u8g2);
	vTaskDelay(500 / portTICK_RATE_MS);

	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_half);
	u8g2_SendBuffer(&u8g2);
	vTaskDelay(500 / portTICK_RATE_MS);

	u8g2_ClearBuffer(&u8g2);
	u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_empty);
	u8g2_SendBuffer(&u8g2);
	vTaskDelay(500 / portTICK_RATE_MS);

	int old_val = -1;
	int val = 0;
	bool update = true;
	int adc[2] = {0, 0};

	int64_t click_time = 0;

	// loop
	while (1)
	{
		int s = gpio_get_level(PIN_ENCODER_BTN);
		if (s == 0) //down
		{
			enc_btn++;
			if (enc_btn > 50) //long press
			{
				cmd.cmd = 1;
				enc_btn = 50;
				update = true;
				xQueueSend(uicmd_queue, &cmd, (portTickType)0);
			}
		}
		else //up
		{
			if (enc_btn > 0)
			{
				if (enc_btn < 10) //short click
				{
					if (cmd.cmd == 1) //ON
					{
						cmd.cmd = 0; //OFF
					}
					else
					{
						if (click_time == 0)
						{
							click_time = esp_timer_get_time();
							cmd.cmd = 2; //PULSE
						}
						else
						{
							if (esp_timer_get_time() - click_time < 400000)
							{
								click_time = 0;
								screen++;
								if (screen == 2)
									screen = 0;
							}
						}
					}

					enc_btn = 0;
					update = true;
					xQueueSend(uicmd_queue, &cmd, (portTickType)0);
				}
			}
			enc_btn = 0;
		}

		if (encoder->get_counter_value(encoder) != val)
		{
			val = encoder->get_counter_value(encoder);

			int v = val / 4 + encoder_cor;

			if (v <= encoder_max && v >= encoder_min)
			{
				cmd.pwm = v;
			}
			else if (v > encoder_max)
			{
				encoder_cor = (encoder_max - (val / 4));
				cmd.pwm = encoder_max;
			}
			else if (v < encoder_min)
			{
				encoder_cor = (encoder_min - (val / 4));
				cmd.pwm = encoder_min;
			}
			xQueueSend(uicmd_queue, &cmd, (portTickType)0);
			update = true;
			//printf("encoder: %d, val: %d, corr: %d\n", pwm, val, encoder_cor);
		}

		if (pdTRUE == xQueueReceive(adc1_queue, &adc, (portTickType)0))
		{
			update = true;
			if (cmd.cmd == 2)
				cmd.cmd = 0;
		}

		if (update)
		{
			if (screen == 0)
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);

				u8g2_DrawStr(&u8g2, 2, 15, "POWER: ");

				if (cmd.cmd == 0)
					u8g2_DrawStr(&u8g2, 60, 15, "  OFF");
				if (cmd.cmd == 1)
					u8g2_DrawStr(&u8g2, 60, 15, "   ON");
				if (cmd.cmd == 2)
					u8g2_DrawStr(&u8g2, 60, 15, "PULSE");

				u8g2_DrawStr(&u8g2, 2, 31, "PWM: ");
				//u8g2_DrawStr(&u8g2, 60, 31, u8x8_u16toa(pwm, 3));
				sprintf(buf, "%3d", cmd.pwm);
				u8g2_DrawStr(&u8g2, 60, 31, buf);

				int v = volt(adc[0]);
				int r = kOm(adc[0], adc[1]);

				sprintf(buf, "U=%3d V (%d)", v, adc[0]);
				u8g2_DrawStr(&u8g2, 2, 47, buf);
				sprintf(buf, "R=%d kOm (%d)", r, adc[1]);
				u8g2_DrawStr(&u8g2, 2, 63, buf);

				u8g2_SendBuffer(&u8g2);
			}
			if (screen == 1) //Меню настроек
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);

				u8g2_DrawStr(&u8g2, 2, 15, "Pulse: ");
				u8g2_DrawStr(&u8g2, 2, 31, "k U: ");
				u8g2_DrawStr(&u8g2, 2, 47, "k R: ");
				u8g2_SendBuffer(&u8g2);
			}
		}

		update = false;
		vTaskDelay(40 / portTICK_RATE_MS);
	}
}
