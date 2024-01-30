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

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include <driver/adc.h>
#include <esp_http_server.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "bme680.h"
#include "moisture_sensor.h"
#include "tsl2561.h"
#include "wifi_config.h"

#define GREETING "\
\n\n Welcome to\n \
...........................:=*+:............................\n \
.........................:*#*=*#*:..........................\n \
.....................::-+#*-+#+-*#+-::......................\n \
..................:+###%#==##-##==#%%%#+-...................\n \
................=###+*#*-*###-#*%*=*%#+*%%+:................\n \
..............+%#+*%%#+=*#=*#-%**%+=+%%%#+#%*...............\n \
............-##+##+##==+=-=*=-+**%*#*+##***+%%=.............\n \
...........+#***=#%#=+#=-::-#=***#*#%*+#%%++*#%*............\n \
..........+#**-:#%#=+%#=---=#=%+###***++%%#--=-%*:..........\n \
.........=#***#%%%+=#######**-+###%###*+*%%%-#*#%*::........\n \
........:*%#%*%%%#=*+======+*=%%#%#+-###+#%%%-=-#%=::.......\n \
........=##%%%%%%*=#******#*%=#+*#*+**#%+*%%%+-==%*-::......\n \
........+%=#+#%#%++#+*****#+#=*##%*****%**%%%%*#+%#-:::.....\n \
.......:*%=+:##*%*+##########=%*+%#**#%%+#%*#@++-%#=-:::....\n \
........+%=**%#*%%+********+#=%%%##%##***%@%%@#%*%#=-::::...\n \
........=%=+-*%+*%#+*###****#=%****###**%@##@*=#=%#=--::::..\n \
..:...:.-##+#%%*+*%%#****##%#=%%#*****#%@@%%@#%+#%*=--:::..:\n \
.:......:=%#%%#%*##%%%%%###*%+%*###%%@@@%%%#*%@@@#+=--::::..\n \
:.::.:..::+%#%%%%%%%######%%%*@@@@@@@@@%#=..:#%@%+==--:::.::\n \
.:.::.::.::*%%%%#=.............::-+%@%#=.:-=%%@%++=--::::::.\n \
:.::.::::=#%%*:..::::::::---------+#%+::-=*%%%*++==--::::.::\n \
::::::::*%+:..::::-------+#######%%%*=-=+#%%**++==--::::::::\n \
.:::::::-#%+=--------------::-----===+*#%@#*++==---:::::::::\n \
:.::::::::*%#+=---================+*#%@%#*++===---::::::::.:\n \
:::::::::::+%%**+++*******++++++*#%@%#**++===----:::::::::::\n \
::::::::::::=%%%##%%@@@@@@@@@@@@@%#**+++===---::::::::::::::\n \
:::::::::::::-#@@@%#*************++++====----:::::::::::::::\n \
 _                __ __  __       _ \n \
| |    ___  __ _ / _|  \\/  | __ _| |_ ___\n \
| |   / _ \\/ _` | |_| |\\/| |/ _` | __/ _ \\\n \
| |__|  __/ (_| |  _| |  | | (_| | ||  __/\n \
|_____\\___|\\__,_|_| |_|  |_|\\__,_|\\__\\___|\n \
Where Every Leaf Counts!\n\n \
by Leya Wehner and Julian Frank\n"

#define GPIO_OFF 0
#define GPIO_ON 1

#define TASK_STACK_DEPTH 2048

#define I2C_BUS       0
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define I2C_FREQ      I2C_FREQ_100K

#define MOISTURE_SENSOR_CHANNEL ADC1_CHANNEL_0

#define PUMP_GPIO 26

static const char *JSON_SENSOR_DATA = "{ \n"
                                      "\t\"temperature\": %d,\n"
                                      "\t\"light_level\": %d,\n"
                                      "\t\"humidity\": %d,\n"
                                      "\t\"moisture\": %d\n"
                                      "}\n";

static const char *JSON_RANGE_DATA = "{ \n"
                                     "\t\"temperature_min\": %d,\n"
                                     "\t\"temperature_max\": %d,\n"
                                     "\t\"light_level_min\": %d,\n"
                                     "\t\"light_level_max\": %d,\n"
                                     "\t\"humidity_min\": %d,\n"
                                     "\t\"humidity_max\": %d,\n"
                                     "\t\"moisture_min\": %d,\n"
                                     "\t\"moisture_max\": %d\n"
                                     "}\n";

extern const char index_html[] asm("_binary_index_html_start");
extern const char config_html[] asm("_binary_config_html_start");

static const char *MAIN_TAG = "Main";
static const char *WIFI_TAG = "WiFi";
static const char *WEBSERVER_TAG = "Server";
static const char *SENSOR_READOUT_TAG = "Sensor Readout";
static const char *PUMP_TAG = "Pump Task";

typedef struct {
    int temperature;
    int light_level;
    int humidity;
    int moisture;
} sensor_data_t;

typedef struct {
    int temperature_min;
    int temperature_max;

    int light_level_min;
    int light_level_max;

    int humidity_min;
    int humidity_max;

    int moisture_min;
    int moisture_max;
} range_config_t;

typedef struct {
    int recheck_time; // in Minutes
    int pump_time; // in Seconds
} pump_config_t;

sensor_data_t sensor_data = {
        .temperature = 0,
        .light_level = 0,
        .humidity = 0,
        .moisture = 0,
};

range_config_t range_config = {
        .temperature_min = 21,
        .temperature_max = 27,

        .light_level_min = 2000,
        .light_level_max = 4000,

        .humidity_min = 40,
        .humidity_max = 60,

        .moisture_min = 20,
        .moisture_max = 60,
};

pump_config_t pump_config = {
        .recheck_time = 15,
        .pump_time = 30,
};

bool wifi_established;

static bme680_sensor_t *sensor = 0;

// WiFi stuff

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void) arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_established = false;
        ESP_LOGI(WIFI_TAG, "Connecting to the AP!");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_established = false;
        ESP_LOGI(WIFI_TAG, "Retrying to connect to the AP!");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(WIFI_TAG, "Got IP:" IPSTR, IP2STR(&(event->ip_info.ip)));
        wifi_established = true;
    } else {
        ESP_LOGI (WIFI_TAG, "Unhandled event (%s) with ID %ld!", event_base, event_id);
    }
}

void wifi_init_sta() {
    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&config));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
//  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL));
    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = WIFI_SSID,
                    .password = WIFI_PASS,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// --------------------------------------------------------------------------------------------------------------------
// Webserver stuff

esp_err_t receive_and_parse_data(httpd_req_t *req, long *min_val, long *max_val, char *kind) {
    ESP_LOGI(WEBSERVER_TAG, "Handling post config %s request", kind);
    /* Read the content length of the request */
    size_t content_length = req->content_len;

    char buffer[content_length + 1];  // Adjust the buffer size as needed

    ESP_LOGI(WEBSERVER_TAG, "Content length: %d", content_length);

    /* Check if there's any data to read */
    if (content_length > 0) {
        /* Read the body of the POST request */
        int read_len = httpd_req_recv(req, buffer, sizeof(buffer));

        if (read_len <= 0) {
            /* Handle error */
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        /* Null-terminate the received data */
        buffer[read_len] = '\0';

        /* Print the received data to the console */
        ESP_LOGI(WEBSERVER_TAG, "Received data: %s", buffer);
        char *temp_min_str = strtok(buffer, ":");
        char *temp_max_str = strtok(NULL, ":");

        *min_val = strtol(temp_min_str, NULL, 10);
        *max_val = strtol(temp_max_str, NULL, 10);
    }

    /* Send a response back to the client */
    httpd_resp_send(req, "Data received successfully", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler functions

esp_err_t get_root_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling root request");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_config_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling config request");
    httpd_resp_send(req, config_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_sensor_readout_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling sensor readout request");
    httpd_resp_set_type(req, "application/json");

    char buff[strlen(JSON_SENSOR_DATA) + 512];

    sprintf(buff, JSON_SENSOR_DATA, sensor_data.temperature, sensor_data.light_level, sensor_data.humidity,
            sensor_data.moisture);

    httpd_resp_send(req, buff, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_range_data_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling range data request");
    httpd_resp_set_type(req, "application/json");

    char buff[strlen(JSON_RANGE_DATA) + 512];

    sprintf(buff, JSON_RANGE_DATA,
            range_config.temperature_min, range_config.temperature_max,
            range_config.light_level_min, range_config.light_level_max,
            range_config.humidity_min, range_config.humidity_max,
            range_config.moisture_min, range_config.moisture_max);

    httpd_resp_send(req, buff, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t post_config_temperature_handler(httpd_req_t *req) {
    long min, max;
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &min, &max, "temperature")) != ESP_OK) {
        return ret;
    }

    range_config.temperature_min = min;
    range_config.temperature_max = max;

    return ret;
}

esp_err_t post_config_light_level_handler(httpd_req_t *req) {
    long min, max;
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &min, &max, "light_level")) != ESP_OK) {
        return ret;
    }

    range_config.light_level_min = min;
    range_config.light_level_max = max;

    return ret;
}

esp_err_t post_config_humidity_handler(httpd_req_t *req) {
    long min, max;
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &min, &max, "humidity")) != ESP_OK) {
        return ret;
    }

    range_config.humidity_min = min;
    range_config.humidity_max = max;

    return ret;
}

esp_err_t post_config_moisture_handler(httpd_req_t *req) {
    long min, max;
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &min, &max, "moisture")) != ESP_OK) {
        return ret;
    }

    range_config.moisture_min = min;
    range_config.moisture_max = max;

    return ret;
}

// Handler functions end


// URI configs

httpd_uri_t uri_get_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_root_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_get_config = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = get_config_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_get_sensor_readout = {
        .uri = "/api/v1/vitality",
        .method = HTTP_GET,
        .handler = get_sensor_readout_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_get_range_data = {
        .uri = "/api/v1/ranges",
        .method = HTTP_GET,
        .handler = get_range_data_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_post_config_temperature = {
        .uri = "/api/v1/config/temperature",
        .method = HTTP_POST,
        .handler = post_config_temperature_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_post_config_light_level = {
        .uri = "/api/v1/config/light_level",
        .method = HTTP_POST,
        .handler = post_config_light_level_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_post_config_humidity_level = {
        .uri = "/api/v1/config/humidity",
        .method = HTTP_POST,
        .handler = post_config_humidity_handler,
        .user_ctx = NULL
};

httpd_uri_t uri_post_config_moisture_level = {
        .uri = "/api/v1/config/moisture",
        .method = HTTP_POST,
        .handler = post_config_moisture_handler,
        .user_ctx = NULL
};

// URI configs end

httpd_handle_t start_webserver(void) {
    ESP_LOGI(WEBSERVER_TAG, "Starting server");
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG ();
    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;
    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_config);
        httpd_register_uri_handler(server, &uri_get_sensor_readout);
        httpd_register_uri_handler(server, &uri_get_range_data);

        httpd_register_uri_handler(server, &uri_post_config_temperature);
        httpd_register_uri_handler(server, &uri_post_config_light_level);
        httpd_register_uri_handler(server, &uri_post_config_humidity_level);
        httpd_register_uri_handler(server, &uri_post_config_moisture_level);
    }
    return server;
}
// --------------------------------------------------------------------------------------------------------------------

/*
 * User task that triggers measurements of sensor every second. It uses
 * function *vTaskDelay* to wait for measurement results. Busy wating
 * alternative is shown in comments
 */
_Noreturn void update_sensor_data(void *pvParameters) {
    (void) pvParameters;

    bme680_values_float_t values;

    TickType_t last_wakeup = xTaskGetTickCount();

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration = bme680_get_measurement_duration(sensor);

    while (1) {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(sensor)) {
            // passive waiting until measurement results are available
            vTaskDelay(duration);

            // alternatively: busy waiting until measurement results are available
            // while (bme680_is_measuring (sensor)) ;

            // get the results and do something with them
            if (bme680_get_results_float(sensor, &values)) {
                float lux = tsl2561_read_sensor_value(I2C_BUS);
                int moisture = moisture_sensor_read(MOISTURE_SENSOR_CHANNEL);

                ESP_LOGI(SENSOR_READOUT_TAG, "Temp: %.2f °C, Light: %.2f lux, Hum: %.2f %%, Moist: %d %%",
                       values.temperature, lux, values.humidity, moisture);

                sensor_data.temperature = (int) roundf(values.temperature);
                sensor_data.light_level = (int) roundf(lux);
                sensor_data.humidity = (int) roundf(values.humidity);
                sensor_data.moisture = moisture;

                // values.pressure, values.gas_resistance);
                // bme680_set_ambient_temperature(sensor, (int16_t) values.temperature);
            }
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS)
    }
}

void water_plant(void *pvParameters) {
    (void) pvParameters;

    // Give the sensors some time for accurate readings before entering the loop
    vTaskDelay(pdMS_TO_TICKS(30 * 1000));

    while (1) {
        int ideal = (int) roundf(range_config.moisture_min + (( (float) (range_config.moisture_max - range_config.moisture_min) ) / 2));
        ESP_LOGI(PUMP_TAG, "Checking if plant needs watering...");
        ESP_LOGI(PUMP_TAG, "Measured soil moisture: %d %%; Ideal soil moisture: %d %%", sensor_data.moisture, ideal);
        if (sensor_data.moisture < ideal) {
            ESP_LOGI(PUMP_TAG, "Plant needs watering! Starting pump for %d seconds...", pump_config.pump_time);
            gpio_set_level(PUMP_GPIO, GPIO_ON);
            vTaskDelay(pdMS_TO_TICKS(pump_config.pump_time * 1000));
            ESP_LOGI(PUMP_TAG, "Turning pump off and giving the water some time to settle...");
            gpio_set_level(PUMP_GPIO, GPIO_OFF);
        } else ESP_LOGI(PUMP_TAG, "Plant doesn't need to watered right now.");

        ESP_LOGI(PUMP_TAG, "Rechecking soil moisture in %d minutes", pump_config.recheck_time);
        vTaskDelay(pdMS_TO_TICKS(pump_config.recheck_time * 60000));
    }
}

void bme680_init() {
    // init the sensor with slave address BME680_I2C_ADDRESS_1 connected to I2C_BUS.
    sensor = bme680_init_sensor(I2C_BUS, BME680_I2C_ADDRESS_1, 0);

    if (sensor) {
        /** -- SENSOR CONFIGURATION --- */

        // Changes the oversampling rates to 4x oversampling for temperature
        // and 2x oversampling for humidity. Pressure measurement is skipped.
        bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);

        // Change the IIR filter size for temperature and pressure to 7.
        bme680_set_filter_size(sensor, iir_size_7);

        // Change the heater profile 0 to 200 degree Celcius for 100 ms.
        // bme680_set_heater_profile(sensor, 0, 200, 100);
        bme680_use_heater_profile(sensor, BME680_HEATER_NOT_USED);
    }
}

void pump_init() {
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_GPIO, GPIO_OFF);
}


void app_main() {
    puts(GREETING);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    while (!wifi_established);

    start_webserver();

    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    // Init I2C bus interfaces
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    // Init BME680 sensor
    bme680_init();

    // TSL2561 init
    tsl2561_sensor_init(I2C_BUS);

    // Init moisture sensor
    moisture_sensor_init(MOISTURE_SENSOR_CHANNEL);

    // Init pump
    pump_init();

    /** -- TASK CREATION --- */
    if (sensor) {
        xTaskCreate(update_sensor_data, "update_sensor_data", TASK_STACK_DEPTH, NULL, 2, NULL);
        xTaskCreate(water_plant, "water_plant", TASK_STACK_DEPTH, NULL, 2, NULL);
    }
    else
        ESP_LOGE(MAIN_TAG, "Could not initialize BME680 sensor");
}
