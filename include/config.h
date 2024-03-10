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

#ifndef LEAFMATE_CONFIG_H
#define LEAFMATE_CONFIG_H


// WiFi setup
#define WIFI_SSID "Your WiFi SSID"
#define WIFI_PASS "your-wifi-pass"


// Uncomment the line below to start in config mode.
// Config mode is used to configure the moisture sensor for correct measurements.
// The MOISTURE_SENSOR_DRY_VAL and MOISTURE_SENSOR_WET_VAL should be set according to your measurements done in config mode.
// Measure once in air and once with the sensor submerged in water and set values accordingly.

// #define CONFIG_MODE
#define MOISTURE_SENSOR_DRY_VAL 2295
#define MOISTURE_SENSOR_WET_VAL 1430


// Defines after what time the soil moisture should be rechecked for watering needs
#define PUMP_RECHECK_TIME_MS 900000
// Defines how long the pump will pump water when watering is needed
#define PUMP_WATERING_TIME_MS 30000

// Defines the initial ideal temperature range (in °C)
#define IDEAL_TEMPERATURE_DEFAULT_MIN 21
#define IDEAL_TEMPERATURE_DEFAULT_MAX 27

// Defines the initial ideal light level range (in lux)
#define IDEAL_LIGHT_LEVEL_DEFAULT_MIN 2000
#define IDEAL_LIGHT_LEVEL_DEFAULT_MAX 4000

// Defines the initial ideal humidity range (in %)
#define IDEAL_HUMIDITY_DEFAULT_MIN 40
#define IDEAL_HUMIDITY_DEFAULT_MAX 60

// Defines the initial ideal soil moisture range (in %)
#define IDEAL_MOISTURE_DEFAULT_MIN 20
#define IDEAL_MOISTURE_DEFAULT_MAX 60

#endif //LEAFMATE_CONFIG_H
