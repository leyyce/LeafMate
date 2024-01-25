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
#include <driver/adc.h>
#include "moisture_sensor.h"

int map(int value, int from_low, int from_high, int to_low, int to_high) {
    int result = (value - from_low) * (to_high - to_low);
    result /= (from_high - from_low);
    result += to_low;

    if (result < to_low) result = to_low;
    else if (result > to_high) result = to_high;

    return result;
}

int moisture_sensor_read(adc1_channel_t adc1_channel) {
    int adc_reading = 0;

    // Perform multiple ADC readings for better accuracy
    for (int i = 0; i < 10; i++) {
        adc_reading += adc1_get_raw(adc1_channel);
        vTaskDelay(pdMS_TO_TICKS(10));  // Short delay between readings
    }

    adc_reading /= 10;  // Calculate average reading

    // printf("R: %d\n", adc_reading);

    // int voltage = adc_reading * 2450 / 4095;

    // Convert ADC reading to moisture level based on sensor characteristics
    // Adjust this calculation based on the sensor's behavior
    int moisture = map(adc_reading, 2250, 1500, 0, 100);

    return moisture;
}

void moisture_sensor_init(adc1_channel_t adc1_channel) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc1_channel, ADC_ATTEN_DB_11);
}