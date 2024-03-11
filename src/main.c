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
#include <sys/cdefs.h>

#include <hal/adc_types.h>
#include <esp_http_server.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include "bme680.h"
#include "config.h"
#include "moisture_sensor.h"
#include "tsl2561.h"


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

#define MOISTURE_SENSOR_UNIT ADC_UNIT_1
#define MOISTURE_SENSOR_CHANNEL ADC_CHANNEL_0

#define PUMP_GPIO 26

#define POST_REQUEST_BUFF_SIZE 128
#define JSON_BUFF_INCREASE 512

/*
 * JSON-Object for sensor data
 * Used to transmit data over API-Endpoint
 */
static const char *JSON_SENSOR_DATA = "{\n"
                                      "\t\"temperature\": %d,\n"
                                      "\t\"light_level\": %d,\n"
                                      "\t\"humidity\": %d,\n"
                                      "\t\"moisture\": %d\n"
                                      "}\n";
/*
 * JSON-Object for the acceptable range of the sensors
 * Used to transmit data over API-Endpoint
 */
static const char *JSON_RANGE_DATA = "{\n"
                                     "\t\"temperature_min\": %d,\n"
                                     "\t\"temperature_max\": %d,\n"
                                     "\t\"light_level_min\": %d,\n"
                                     "\t\"light_level_max\": %d,\n"
                                     "\t\"humidity_min\": %d,\n"
                                     "\t\"humidity_max\": %d,\n"
                                     "\t\"moisture_min\": %d,\n"
                                     "\t\"moisture_max\": %d\n"
                                     "}\n";

/* Pointers to the gzip compressed HTML files embedded in the firmware binary */
extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const unsigned char index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const unsigned char config_html_gz_start[] asm("_binary_config_html_gz_start");
extern const unsigned char config_html_gz_end[] asm("_binary_config_html_gz_end");

/* Tags for the esp-log functions. Used for categorizing the logs */
static const char *MAIN_TAG = "Main";
static const char *WIFI_TAG = "WiFi";
static const char *WEBSERVER_TAG = "Server";
static const char *SENSOR_READOUT_TAG = "Sensor Readout";
static const char *PUMP_TAG = "Pump Task";

/* Struct for the sensor data */
typedef struct {
    int temperature;    // in °C
    int light_level;    // in lux
    int humidity;       // in %
    int moisture;       // in %
} sensor_data_t;

/* Struct for the ideal range of the sensors values */
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

/* Struct for pump time and checks */
typedef struct {
    int recheck_time_ms;
    int pump_watering_time_ms;
} pump_config_t;

/* Initialising a sensor data object with values of 0 */
static sensor_data_t sensor_data = {
        .temperature = 0,
        .light_level = 0,
        .humidity = 0,
        .moisture = 0,
};

/* Initialising a sensor range object with the default values set in the config */
static range_config_t range_config = {
        .temperature_min = IDEAL_TEMPERATURE_DEFAULT_MIN,
        .temperature_max = IDEAL_TEMPERATURE_DEFAULT_MAX,

        .light_level_min = IDEAL_LIGHT_LEVEL_DEFAULT_MIN,
        .light_level_max = IDEAL_LIGHT_LEVEL_DEFAULT_MAX,

        .humidity_min = IDEAL_HUMIDITY_DEFAULT_MIN,
        .humidity_max = IDEAL_HUMIDITY_DEFAULT_MAX,

        .moisture_min = IDEAL_MOISTURE_DEFAULT_MIN,
        .moisture_max = IDEAL_MOISTURE_DEFAULT_MAX,
};

/* Initialising the pump object with values set in the config */
static pump_config_t pump_config = {
        .recheck_time_ms = PUMP_RECHECK_TIME_MS,
        .pump_watering_time_ms = PUMP_WATERING_TIME_MS,
};

/* Has a Wifi-Connection been established? */
static bool wifi_established;

/* pointer for our bme680_sensor_t object to be used later when the sensor gets initialized! */
static bme680_sensor_t *sensor = 0;

/* Declaring functions so that they can be used before they are implemented */
static void update_sensor_data();


/* ------------------------------------------------------------------------------------------------------------------ */
/* Start of WiFi initialization and event handling */

/*
 * Handles the connection to WiFi
 * Sets the value of the wifi_established boolean
 * also logs esp WiFi-events
*/
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

/*
 * Initiates the network-interface of the esp
 * Utilizes the default, provided WiFi initialization config
 * Uses the values defined inside of config.h for the SSID and password of the network
 * Uses ESP_ERROR_CHECK to provide error-handling
*/
static void wifi_init_sta() {
    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&config));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

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

/* End of WiFi initialization and event handling */
/* ------------------------------------------------------------------------------------------------------------------ */


/* ------------------------------------------------------------------------------------------------------------------ */
/* Start of webserver section */


/*
 * The receive_and_parse_data function handles POST requests to the webserver
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * int *min_val:        Pointer to the minimum value of the sensor range that should be updated
 *
 * int *max_val:        Pointer to the maximum value of the sensor range that should be updated
 *
 * char *kind:          String which declares what sensor value is updated, e.g. "temperature"
 *
 * returns:             ESP_OK on success or ESP_FAIL on failed read from request body
 */
static esp_err_t receive_and_parse_data(httpd_req_t *req, int *min_val, int *max_val, char *kind) {
    ESP_LOGI(WEBSERVER_TAG, "Handling %s config post request", kind);

    /* Read the content length of the request */
    size_t content_length = req->content_len;

    /* Buffer size + 1 to accommodate the null terminator */
    char buffer[POST_REQUEST_BUFF_SIZE + 1];

    ESP_LOGI(WEBSERVER_TAG, "Content length: %d", content_length);
    if (content_length > POST_REQUEST_BUFF_SIZE) return ESP_FAIL;

    /* Check if there is any data to read */
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

        /* Split received data at ':' */
        char *temp_min_str = strtok(buffer, ":");
        char *temp_max_str = strtok(NULL, ":");

        /* Update min and max values based on the received user input */
        *min_val = strtol(temp_min_str, NULL, 10);
        *max_val = strtol(temp_max_str, NULL, 10);
    }

    /* Send a response back to the client */
    httpd_resp_send(req, "Data received successfully", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static inline void send_html_response(httpd_req_t *req, const unsigned char *start, const unsigned char *end) {
    /* Adds the header field for "Content-Encoding" to our http response so */
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    /*
     * Computes the length of the gzipped HTML file
     * Necessary because the files are embedded in binary mode and not in text mode, so they are not a null-terminated
     */
    const ssize_t size = (end - start);

    /* Responds to the request of with the corresponding HTML page */
    httpd_resp_send(req, (const char *) start, size);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* start of handler functions */

/*
 *
 * The get_root_handler function handles GET requests to the webserver root
 * Replies with the gzip compressed index.html
 *
 * httpd_req_t *req:        Pointer to the received HTTP request
 *
 * returns:                 ESP_OK
 */
static esp_err_t get_root_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling root request");

    send_html_response(req, index_html_gz_start, index_html_gz_end);

    return ESP_OK;
}

/*
 *
 * The get_config_handler function handles GET requests to the /config path
 * Replies with the gzip compressed config.html
 *
 * httpd_req_t *req:        Pointer to the received HTTP request
 *
 * returns:                 ESP_OK
 */
static esp_err_t get_config_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling config request");

    send_html_response(req, config_html_gz_start, config_html_gz_end);

    return ESP_OK;
}

/*
 *
 * The get_sensor_readout_handler replies to the GET request with the JSON formatted sensor data
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t get_sensor_readout_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling sensor readout request");
    httpd_resp_set_type(req, "application/json");

    char buff[strlen(JSON_SENSOR_DATA) + JSON_BUFF_INCREASE];

    update_sensor_data();
    sprintf(buff, JSON_SENSOR_DATA, sensor_data.temperature, sensor_data.light_level, sensor_data.humidity,
            sensor_data.moisture);

    httpd_resp_send(req, buff, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 *
 * The post_config_temperature_handler replies to the GET request with the JSON formatted range data
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t get_range_data_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling range data request");
    httpd_resp_set_type(req, "application/json");

    char buff[strlen(JSON_RANGE_DATA) + JSON_BUFF_INCREASE];

    sprintf(buff, JSON_RANGE_DATA,
            range_config.temperature_min, range_config.temperature_max,
            range_config.light_level_min, range_config.light_level_max,
            range_config.humidity_min, range_config.humidity_max,
            range_config.moisture_min, range_config.moisture_max);

    httpd_resp_send(req, buff, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 *
 * The post_config_temperature_handler function handles POST requests to update the ideal temperature range
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             e.g. ESP_OK on success or ESP_FAIL on failed read from request body
 */
static esp_err_t post_config_temperature_handler(httpd_req_t *req) {
    esp_err_t ret;

    /* Updates the range values for the temperature */
    if ((ret = receive_and_parse_data(req, &range_config.temperature_min, &range_config.temperature_max,
                                      "temperature")) != ESP_OK) {
        return ret;
    }

    return ret;
}

/*
 *
 * The post_config_light_level_handler function handles POST requests to update the ideal light level range
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             e.g. ESP_OK on success or ESP_FAIL on failed read from request body
 */
static esp_err_t post_config_light_level_handler(httpd_req_t *req) {
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &range_config.light_level_min, &range_config.light_level_max,
                                      "light_level")) != ESP_OK) {
        return ret;
    }

    return ret;
}

/*
 *
 * The post_config_humidity_handler function handles POST requests to update the ideal humidity range
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             e.g. ESP_OK on success or ESP_FAIL on failed read from request body
 */
static esp_err_t post_config_humidity_handler(httpd_req_t *req) {
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &range_config.humidity_min, &range_config.humidity_max, "humidity")) !=
        ESP_OK) {
        return ret;
    }

    return ret;
}

/*
 *
 * The post_config_moisture_handler function handles POST requests to update the ideal moisture range
 *
 * httpd_req_t *req:    pointer to the received HTTP request
 *
 * returns:             e.g. ESP_OK on success or ESP_FAIL on failed read from request body
 */
static esp_err_t post_config_moisture_handler(httpd_req_t *req) {
    esp_err_t ret;

    if ((ret = receive_and_parse_data(req, &range_config.moisture_min, &range_config.moisture_max, "moisture")) !=
        ESP_OK) {
        return ret;
    }

    return ret;
}
/* End of handler functions */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Start of URI configs */

static httpd_uri_t uri_get_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_root_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_get_config = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = get_config_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_get_sensor_readout = {
        .uri = "/api/v1/vitality",
        .method = HTTP_GET,
        .handler = get_sensor_readout_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_get_range_data = {
        .uri = "/api/v1/ranges",
        .method = HTTP_GET,
        .handler = get_range_data_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_post_config_temperature = {
        .uri = "/api/v1/config/temperature",
        .method = HTTP_POST,
        .handler = post_config_temperature_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_post_config_light_level = {
        .uri = "/api/v1/config/light_level",
        .method = HTTP_POST,
        .handler = post_config_light_level_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_post_config_humidity_level = {
        .uri = "/api/v1/config/humidity",
        .method = HTTP_POST,
        .handler = post_config_humidity_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_post_config_moisture_level = {
        .uri = "/api/v1/config/moisture",
        .method = HTTP_POST,
        .handler = post_config_moisture_handler,
        .user_ctx = NULL
};

/* End of URI config */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Start of webserver and registering handlers */
static httpd_handle_t start_webserver() {
    ESP_LOGI(WEBSERVER_TAG, "Starting server");
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
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
/* End of Webserver section*/
/* ------------------------------------------------------------------------------------------------------------------ */


/*
 * The update_sensor_data function tries to update the sensor data and also calls the update_oled function to update the
 * display accordingly
 */
static void update_sensor_data() {
    bme680_values_float_t values;

    /* Gets an approximation of the time the measurement will take  */
    uint32_t duration = bme680_get_measurement_duration(sensor);

    /* Start a measurement of humidity and temperature via the bme680 sensor */
    if (bme680_force_measurement(sensor)) {
        /* Passive wating for the duration of the measurement
         * Utilization of active waiting via a while-loop and the bme680_is_measuring function possible too */
        vTaskDelay(duration);
        if (bme680_get_results_float(sensor, &values)) {
            /* start a measurement of the light level via the tsl2561 sensor*/
            float lux = tsl2561_read_sensor_value(I2C_BUS);
            /* start a measurement of moisture via the moisture sensor*/
            int moisture = moisture_sensor_read();

            /* Log the values for the ESP  */
            ESP_LOGI(SENSOR_READOUT_TAG, "Temp: %.2f °C, Light: %.2f lux, Hum: %.2f %%, Moist: %d %%",
                     values.temperature, lux, values.humidity, moisture);

            /* set the attributes of the sensor data object accordingly */
            sensor_data.temperature = (int) roundf(values.temperature);
            sensor_data.light_level = (int) roundf(lux);
            sensor_data.humidity = (int) roundf(values.humidity);
            sensor_data.moisture = moisture;
        }
    }
}

/*
 * The water_plant function periodically checks if the plant needs watering and pumps water to the plant accordingly.
 * The function also updates the sensor data beforehand make the check as accurate as possible
 *
 * void *pvParameters:  required Parameter for the xTaskCreate function, always NULL
 *
 * notes:               _Noreturn tells the compiler that the function should never return and makes it act accordingly
 */

static _Noreturn void water_plant() {
    // Wait 5 minutes before checking the pump for the first time to make sure the hardware is set up correctly.
    vTaskDelay(pdMS_TO_TICKS(5 * 60000));

    while (1) {
        int ideal = (int) roundf(
                range_config.moisture_min + (((float) (range_config.moisture_max - range_config.moisture_min)) / 2));
        ESP_LOGI(PUMP_TAG, "Checking if plant needs watering...");
        /* Update sensor data for an accurate check */
        update_sensor_data();

        ESP_LOGI(PUMP_TAG, "Measured soil moisture: %d %%; Ideal soil moisture: %d %%", sensor_data.moisture, ideal);
        if (sensor_data.moisture < ideal) {
            ESP_LOGI(PUMP_TAG, "Plant needs watering! Starting pump for %d seconds...",
                     pump_config.pump_watering_time_ms / 1000);
            /* activate the water pump by setting the GPIO-level to ON for the specified pump time then turn OFF again*/
            gpio_set_level(PUMP_GPIO, GPIO_ON);
            vTaskDelay(pdMS_TO_TICKS(pump_config.pump_watering_time_ms));
            ESP_LOGI(PUMP_TAG, "Turning pump off and giving the water some time to settle...");
            gpio_set_level(PUMP_GPIO, GPIO_OFF);
        } else
            ESP_LOGI(PUMP_TAG, "Plant doesn't need to be watered right now.");

        ESP_LOGI(PUMP_TAG, "Rechecking soil moisture in %d minutes", pump_config.recheck_time_ms / 60000);
        /* Check */
        vTaskDelay(pdMS_TO_TICKS(pump_config.recheck_time_ms));
    }
}

static void bme680_init() {
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

static void pump_init() {
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_GPIO, GPIO_OFF);
}

void app_main() {
    puts(GREETING);

    /* Initializing moisture sensor */
    moisture_sensor_init(MOISTURE_SENSOR_UNIT, MOISTURE_SENSOR_CHANNEL);

#ifdef CONFIG_MODE
    puts("LEAFMATE IS RUNNING IN CONFIG MODE!");
    puts("After you're finished setting up the moisture sensor comment out the\n\n#define CONFIG_MODE\n\n"
         "line in include/config.h and recompile LeafMate.\n\n");
    while (1) {
        printf("Raw ADC Reading: %d\n", moisture_sensor_read_raw());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
#else
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    /* Initializing WiFi-Connection */
    wifi_init_sta();

    while (!wifi_established) { vTaskDelay(1); }

    /* Start the ESP-Webserver */
    start_webserver();

    /* Setting the baud rate of our Universal Asynchronous Receiver/Transmitter to the same as our  */
    // uart_set_baud(0, 115200);
    /* Give the UART some time to settle */
    vTaskDelay(1);

    /* Initializing the I2C-BUS-Interfaces*/
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    /* Initialize the bme680 sensor */
    bme680_init();

    /* Initialize the tsl2561 sensor */
    tsl2561_sensor_init(I2C_BUS);

    /* Initialize the pump */
    pump_init();

    /*  TASK CREATION  */
    if (sensor) {
        /* creates a Task that runs in a separate execution context */
        xTaskCreate(water_plant, "water_plant", TASK_STACK_DEPTH, NULL, 2, NULL);
    } else
        ESP_LOGE(MAIN_TAG, "Could not initialize BME680 sensor");
#endif //CONFIG_MODE
}
