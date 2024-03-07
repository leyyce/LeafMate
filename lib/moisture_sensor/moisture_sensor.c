/*
 * LeafMate - Plant monitoring for the ESP32
 * Copyright (C) 2024  Leya Wehner
 * Copyright (C) 2024  Julian Frank
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_oneshot.h>

#include "config.h"
#include "moisture_sensor.h"

adc_oneshot_unit_handle_t handle;
adc_channel_t channel;

int map(int value, int from_low, int from_high, int to_low, int to_high) {
    int result = (value - from_low) * (to_high - to_low);
    result /= (from_high - from_low);
    result += to_low;

    if (result < to_low) result = to_low;
    else if (result > to_high) result = to_high;

    return result;
}

int moisture_sensor_read_raw() {
    int reads = 10;

    int avg = 0;
    int raw;

    // Perform multiple ADC readings for better accuracy
    for (int i = 0; i < reads; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(handle, channel, &raw));
        avg += raw;
        vTaskDelay(pdMS_TO_TICKS(10));  // Short delay between readings
    }

    return avg / reads;  // Calculate average reading
}

int moisture_sensor_read() {
    int raw = moisture_sensor_read_raw();

    // int voltage = adc_reading * 2450 / 4095;
    // Convert ADC reading to moisture level based on sensor characteristics
    // Adjust this calculation based on the sensor's behavior
    int moisture = map(raw, MOISTURE_SENSOR_DRY_VAL, MOISTURE_SENSOR_WET_VAL, 0, 100);

    return moisture;
}

void moisture_sensor_init(adc_unit_t adc_unit, adc_channel_t adc_channel) {
    adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = adc_unit,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &handle));

    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, adc_channel, &config));
    channel = adc_channel;
}
