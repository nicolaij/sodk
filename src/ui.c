#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
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

menu_t menu[] = {
	{.name = "Импульс", .val = 100, .min = 1, .max = 1000},
	{.name = "коэф. U", .val = 5336, .min = 1000, .max = 10000},
	{.name = "коэф. R", .val = 90, .min = 1, .max = 1000},
	{.name = "смещ. U", .val = 0, .min = -500, .max = 500},
	{.name = "смещ. R", .val = 0, .min = -500, .max = 500},
	{.name = "Выход  ", .val = 0, .min = 0, .max = 0},
};

#define MENU_LINES sizeof(menu) / sizeof(menu[0])
#define DISPLAY_LINES 4

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
	cmd.power = 0;

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

	int enc_btn = 0;
	gpio_pad_select_gpio(PIN_ENCODER_BTN);
	gpio_set_direction(PIN_ENCODER_BTN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(PIN_ENCODER_BTN, GPIO_PULLUP_ONLY);

	xSemaphoreTake(i2c_mux, portMAX_DELAY);

	// initialize the u8g2 hal
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda = PIN_SDA;
	u8g2_esp32_hal.scl = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	// initialize the u8g2 library
	u8g2_t u8g2;
	u8g2_Setup_sh1106_i2c_128x64_noname_f(
		// u8g2_Setup_ssd1306_i2c_128x32_univision_f(
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

	xSemaphoreGive(i2c_mux);

	vTaskDelay(500 / portTICK_RATE_MS);

	int encoder_val = 0;
	bool update = true;

	// Open
	nvs_handle_t my_handle;
	esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE("storage","Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	}
	else
	{
		for (int i = 0; i < MENU_LINES; i++)
		{
			err = nvs_get_i32(my_handle, menu[i].name, &menu[i].val);
			switch (err)
			{
			case ESP_OK:
				printf("Read \"%s\" = %d\n", menu[i].name, menu[i].val);
				break;
			case ESP_ERR_NVS_NOT_FOUND:
				printf("The value  \"%s\" is not initialized yet!\n", menu[i].name);
				break;
			default:
				printf("Error (%s) reading!\n", esp_err_to_name(err));
			}
		}

		// Close
		nvs_close(my_handle);
	}

	//результат измерений
	result_t result;

	keys_events_t encoder_key = KEY_NONE;

	int64_t t1 = 0; // Для определения Double Click

	// loop
	while (1)
	{
		int s = gpio_get_level(PIN_ENCODER_BTN);
		if (s == 0) // down
		{
			enc_btn++;
			if (enc_btn == 50) // long press
			{
				if (t1 == 0)
					encoder_key = KEY_LONG_PRESS;
			}

			if (enc_btn == 100) // super long press
			{
				if (t1 == 0)
					encoder_key = KEY_DOUBLELONG_PRESS;
			}
		}
		else // up
		{
			if (enc_btn > 0)
			{
				if (enc_btn < 10) // short click
				{
					if (t1 > 0)
					{
						// printf("Double t :%lld, c=%d\n", esp_timer_get_time() - t1, enc_btn);
						encoder_key = KEY_DOUBLECLICK;
						t1 = 0;
					}
					else
						t1 = esp_timer_get_time();
				}
				else
					t1 = 0;
			}
			else if (t1 > 0)
				if (esp_timer_get_time() - t1 > 400000)
				{
					encoder_key = KEY_CLICK;
					t1 = 0;
				}

			enc_btn = 0;
		}

		if (encoder->get_counter_value(encoder) != encoder_val)
		{
			encoder_val = encoder->get_counter_value(encoder);

			int v = encoder_val / 4 + encoder_cor;

			if (screen == 0)
			{
				cmd.power = limits(v, PWM_MIN, PWM_MAX);
				encoder_cor = cmd.power - encoder_val / 4;
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
				if (cmd.cmd == 1) // ON
				{
					cmd.cmd = 0; // OFF
				}
				else
				{
					if (cmd.cmd == 0) // OFF
					{
						cmd.cmd = 2; // PULSE
					}
				}
				update = true;
				xQueueSend(uicmd_queue, &cmd, (portTickType)0);
				break;
			case KEY_DOUBLECLICK: //Включаем
				if (cmd.cmd == 1) // ON
				{
					cmd.cmd = 0; // OFF
				}
				else
					cmd.cmd = 1;
				update = true;
				xQueueSend(uicmd_queue, &cmd, (portTickType)0);
				break;
			case KEY_LONG_PRESS:
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
				update = true;
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
				if (menu_current_selection < 0) //Выбираем пункт меню
				{
					if (menu_current_position == MENU_LINES - 1) //Последний пункт меню - Выход на главный экран
					{
						screen = 0;
						encoder_cor = cmd.power - encoder_val / 4;
					}
					else
					{
						menu_current_selection = menu_current_position;
						encoder_cor = menu[menu_current_selection].val - encoder_val / 4;
					}
				}
				else //Сохраняем введенное значение
				{
					nvs_handle_t my_handle;
					err = nvs_open("storage", NVS_READWRITE, &my_handle);
					if (err != ESP_OK)
					{
						printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
					}
					else
					{
						// Write
						printf("Write: \"%s\" ", menu[menu_current_selection].name);
						err = nvs_set_i32(my_handle, menu[menu_current_selection].name, menu[menu_current_selection].val);
						printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

						// Commit written value.
						// After setting any values, nvs_commit() must be called to ensure changes are written
						// to flash storage. Implementations may write to storage at other times,
						// but this is not guaranteed.
						printf("Committing updates in NVS ... ");
						err = nvs_commit(my_handle);
						printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

						// Close
						nvs_close(my_handle);
					}

					menu_current_selection = -1;
					encoder_cor = menu_current_position - encoder_val / 4;
				}
				update = true;
				break;
			case KEY_LONG_PRESS: //Выход на главный экран
				screen = 0;
				encoder_cor = cmd.power - encoder_val / 4;
				update = true;
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
			xSemaphoreTake(i2c_mux, portMAX_DELAY);

			if (screen == 0)
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_cyrillic);

				u8g2_DrawStr(&u8g2, 2, 16 - 3, "POWER: ");

				if (cmd.cmd == 0)
					u8g2_DrawStr(&u8g2, 53, 16 - 3, "OFF");
				if (cmd.cmd == 1)
					u8g2_DrawStr(&u8g2, 53, 16 - 3, "ON");
				if (cmd.cmd == 2)
					u8g2_DrawStr(&u8g2, 53, 16 - 3, "PULSE");

				// u8g2_DrawStr(&u8g2, 2, 31, "PWM: ");
				// u8g2_DrawStr(&u8g2, 60, 31, u8x8_u16toa(pwm, 3));
				sprintf(buf, "%d", cmd.power);
				int w = u8g2_GetStrWidth(&u8g2, buf);
				u8g2_DrawStr(&u8g2, u8g2_GetDisplayWidth(&u8g2) - w, 16 - 3, buf);

				sprintf(buf, "U = %d V", result.U);
				u8g2_DrawStr(&u8g2, 2, 16 * 2 - 3, buf);
				sprintf(buf, "R = %d kOm", result.R);
				u8g2_DrawStr(&u8g2, 2, 16 * 3 - 3, buf);

				u8g2_SetFont(&u8g2, u8g2_font_6x13_t_cyrillic);
				sprintf(buf, "1:%4d 2:%4d U:%4d", result.adc11, result.adc12, result.adc2);
				u8g2_DrawStr(&u8g2, 2, 16 * 4 - 3, buf);

				u8g2_SendBuffer(&u8g2);
			}
			if (screen == 1) //Меню настроек
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFontMode(&u8g2, 1);
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_cyrillic);
				// u8g2_SetDrawColor(&u8g2, 1);
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
			xSemaphoreGive(i2c_mux);
		}
		update = false;
		vTaskDelay(40 / portTICK_RATE_MS);
	}
}
