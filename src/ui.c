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

#include <U8g2.h>

#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ui";

typedef struct
{
	char name[32];
	int val;
	int min;
	int max;
} menu_t;

menu_t menu[] = {
	{.name = "Импульс", .val = 100, .min = 1, .max = 1000},
	{.name = "коэф. U", .val = 5336, .min = 1000, .max = 10000},
	{.name = "коэф. R1", .val = 90, .min = 1, .max = 1000},
	{.name = "коэф. R2", .val = 1750, .min = 100, .max = 5000},
	{.name = "Выход  ", .val = 0, .min = 0, .max = 0},
};

#define MENU_LINES sizeof(menu) / sizeof(menu[0])
#define DISPLAY_LINES 2

int menu_current_selection = -1;
int menu_current_position = 0;
int menu_current_display = 0;

int limits(int val, int min, int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

void ui_task(void *arg)
{
	int screen = 0;
	int encoder_cor = 0;

	char buf[64];

	cmd_t cmd;
	cmd.cmd = 0;
	cmd.pwm = 0;

	// Rotary encoder underlying device is represented by a PCNT unit in this example
	uint32_t pcnt_unit = 0;

	//only for test. Power encoder
	gpio_pad_select_gpio(5);
	gpio_set_direction(5, GPIO_MODE_OUTPUT);
	gpio_set_level(5, 1);

	// Create rotary encoder instance
	rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, PIN_ENCODER_A, PIN_ENCODER_B);
	rotary_encoder_t *encoder = NULL;
	ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

	// Filter out glitch (1us)
	ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

	// Start encoder
	ESP_ERROR_CHECK(encoder->start(encoder));

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
	//u8g2_Setup_ssd1306_i2c_128x64_noname_f(
	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
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

	int encoder_val = 0;
	bool update = true;

	//результат измерений
	result_t result;

	keys_events_t encoder_key = KEY_NONE;

	// loop
	while (1)
	{
		int s = gpio_get_level(PIN_ENCODER_BTN);
		if (s == 0) //down
		{
			enc_btn++;
			if (enc_btn == 50) //long press
			{
				encoder_key = KEY_LONG_PRESS;
			}

			if (enc_btn == 100) //super long press
			{
				encoder_key = KEY_DOUBLELONG_PRESS;
			}
		}
		else //up
		{
			if (enc_btn > 0)
			{
				if (enc_btn < 10) //short click
				{
					encoder_key = KEY_CLICK;
				}
			}
			enc_btn = 0;
		}

		if (encoder->get_counter_value(encoder) != encoder_val)
		{
			encoder_val = encoder->get_counter_value(encoder);

			int v = encoder_val / 4 + encoder_cor;

			if (screen == 0)
			{
				cmd.pwm = limits(v, PWM_MIN, PWM_MAX);
				encoder_cor = cmd.pwm - encoder_val / 4;
				xQueueSend(uicmd_queue, &cmd, (portTickType)0);
			}
			else if (screen == 1)
			{
				if (menu_current_selection < 0) //навигация по меню
				{
					menu_current_position = limits(v, 0, MENU_LINES - 1);
					encoder_cor = menu_current_position - encoder_val / 4;
				}
				else //изменения значений
				{
					menu[menu_current_selection].val = limits(v, menu[menu_current_selection].min, menu[menu_current_selection].max);
				}
			};
			update = true;
		}

		if (screen == 0)
		{
			switch (encoder_key)
			{
			case KEY_CLICK:
				if (cmd.cmd == 1) //ON
				{
					cmd.cmd = 0; //OFF
				}
				else
				{
					if (cmd.cmd == 0) //OFF
					{
						cmd.cmd = 2; //PULSE
					}
				}
				update = true;
				xQueueSend(uicmd_queue, &cmd, (portTickType)0);
				break;
			case KEY_LONG_PRESS:
				cmd.cmd = 1;
				update = true;
				xQueueSend(uicmd_queue, &cmd, (portTickType)0);
				break;
			case KEY_DOUBLELONG_PRESS:
				//Вызываем меню
				screen++;
				if (screen == 2)
					screen = 0;

				if (screen == 1)
				{
					menu_current_position = 0;
					encoder_cor = menu_current_position - encoder_val / 4;
					menu_current_selection = -1;
				}
				break;

			default:
				break;
			}
		}
		else if (screen == 1) //Меню
		{
			switch (encoder_key)
			{
			case KEY_CLICK:
				if (menu_current_selection < 0)
				{
					if (menu_current_position == MENU_LINES - 1) //Последний пункт меню - Выход на главный экран
					{
						screen = 0;
						encoder_cor = cmd.pwm - encoder_val / 4;
					}
					else
					{
						menu_current_selection = menu_current_position;
						encoder_cor = menu[menu_current_selection].val - encoder_val / 4;
					}
				}
				else
				{
					//Сохраняем введенное значение

					// Initialize NVS
					esp_err_t err = nvs_flash_init();
					if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
					{
						// NVS partition was truncated and needs to be erased
						// Retry nvs_flash_init
						ESP_ERROR_CHECK(nvs_flash_erase());
						err = nvs_flash_init();
					}
					ESP_ERROR_CHECK(err);

					menu_current_selection = -1;
					encoder_cor = menu_current_position - encoder_val / 4;
				}
				break;

			default:
				break;
			}
		}
		encoder_key = 0;

		if (pdTRUE == xQueueReceive(adc1_queue, &result, (portTickType)0))
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
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_cyrillic);

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

				sprintf(buf, "U=%3d V (%d)", result.U, result.adc2);
				u8g2_DrawStr(&u8g2, 2, 47, buf);
				sprintf(buf, "R=%d kOm (%d)", result.R, result.adc1);
				u8g2_DrawStr(&u8g2, 2, 63, buf);

				u8g2_SendBuffer(&u8g2);
			}
			if (screen == 1) //Меню настроек
			{

				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFontMode(&u8g2, 1);
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_cyrillic);
				//u8g2_SetDrawColor(&u8g2, 1);
				u8g2_SetDrawColor(&u8g2, 2);

				if (menu_current_position - menu_current_display > (DISPLAY_LINES - 1))
					menu_current_display = menu_current_position - (DISPLAY_LINES - 1);

				if (menu_current_position < menu_current_display)
					menu_current_display = menu_current_position;

				if (menu_current_selection < 0) //Навигация по меню
				{
					u8g2_DrawBox(&u8g2, 0, 16 * (menu_current_position - menu_current_display), u8g2_GetDisplayWidth(&u8g2), 16);
				}

				for (int l = 0; l < DISPLAY_LINES; l++)
				{
					int p = menu_current_display + l;
					if (p < MENU_LINES)
					{
						u8g2_DrawUTF8(&u8g2, 3, 16 * (l + 1) - 3, menu[p].name);
						sprintf(buf, "%d", menu[p].val);
						int w = u8g2_GetStrWidth(&u8g2, buf);
						if (menu_current_selection == p)
						{
							u8g2_DrawBox(&u8g2, u8g2_GetDisplayWidth(&u8g2) - w - 2, 16 * l, w + 2, 16);
						}
						u8g2_DrawUTF8(&u8g2, u8g2_GetDisplayWidth(&u8g2) - w, 16 * (l + 1) - 3, buf);
					}
				}

				u8g2_SendBuffer(&u8g2);
			}
		}
		update = false;
		vTaskDelay(40 / portTICK_RATE_MS);
	}
}
