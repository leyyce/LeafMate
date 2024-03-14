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
#include <freertos/semphr.h>
#include <nvs_flash.h>

#include "bme680.h"
#include "config.h"
#include "moisture_sensor.h"
#include "tsl2561.h"

#define xQueueHandle QueueHandle_t

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

#define POST_REQUEST_BUF_SIZE 128
#define JSON_BUF_INCREASE 512

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

static const char *JSON_PUMP_STATUS_DATA = "{\n"
                                           "\t\"pump_status\": %s\n"
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

/* Initialising a sensor data object with all values set to 0 */
static sensor_data_t sensor_data = {
        .temperature = 0,
        .light_level = 0,
        .humidity = 0,
        .moisture = 0,
};

SemaphoreHandle_t sensor_data_mutex = NULL;

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

/* Pointer to bme680 sensor struct */
static bme680_sensor_t *sensor = NULL;

TaskHandle_t pump_task = NULL;
bool pump_task_toggle = PUMP_TOGGLE_DEFAULT;

/* Declaring functions so that they can be used before they are implemented */
static void update_sensor_data();


/* ------------------------------------------------------------------------------------------------------------------ */
/* Start of Wi-Fi initialization and event handling */

/*
 * Handles the connection to Wi-Fi
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
 * Utilizes the default, provided Wi-Fi initialization config
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

/* End of Wi-Fi initialization and event handling */
/* ------------------------------------------------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------------------------------------------------ */
/* Start of webserver section */

static esp_err_t read_request_content(httpd_req_t *req, char *buffer, size_t buf_len, char *kind) {
    /* Read the content length of the request */
    size_t content_length = req->content_len;

    ESP_LOGI(WEBSERVER_TAG, "Content length: %d", content_length);
    if (content_length > POST_REQUEST_BUF_SIZE) {
        ESP_LOGE(WEBSERVER_TAG, "Request body is to long. %s config values not updated.", kind);

        httpd_resp_set_status(req, "400");
        httpd_resp_send(req, "Request body is to long!", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    /* Check if there is any data to read */
    if (content_length > 0) {
        /* Read the body of the POST request */
        int read_len = httpd_req_recv(req, buffer, buf_len);

        if (read_len <= 0) {
            /* Handle error */
            httpd_resp_send_500(req);
            ESP_LOGE(WEBSERVER_TAG, "Failed to read from request body. %s config values not updated.", kind);
            return ESP_FAIL;
        }

        /* Null-terminate the received data */
        buffer[read_len] = '\0';

        /* Print the received data to the console */
        ESP_LOGI(WEBSERVER_TAG, "Received data: %s", buffer);

        return ESP_OK;
    }

    ESP_LOGE(WEBSERVER_TAG, "Request body is empty. %s config values not updated.", kind);
    return ESP_FAIL;
}

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

    /* Buffer size + 1 to accommodate the null terminator */
    char buffer[POST_REQUEST_BUF_SIZE + 1];

    if (read_request_content(req, buffer, sizeof(buffer), kind) == ESP_OK) {
        /* Split received data at ':' */
        char *temp_min_str = strtok(buffer, ":");
        char *temp_max_str = strtok(NULL, ":");

        /* Update min and max values based on the received user input */
        if (temp_min_str && temp_max_str) {
            char *min_end_ptr = NULL, *max_end_ptr = NULL;

            int min_val_temp = strtol(temp_min_str, &min_end_ptr, 10);
            int max_val_temp = strtol(temp_max_str, &max_end_ptr, 10);

            if (! *min_end_ptr && ! *max_end_ptr) {

                if (min_val_temp > max_val_temp) {
                    ESP_LOGE(WEBSERVER_TAG, "%s_min can't be greater than %s_max. %s config values not updated!", kind, kind, kind);
                    httpd_resp_set_status(req, "400");
                    httpd_resp_send(req, "Min val can't be greater than max val. Request invalid!", HTTPD_RESP_USE_STRLEN);
                    return ESP_FAIL;
                }

                *min_val = min_val_temp;
                *max_val = max_val_temp;

                ESP_LOGI(WEBSERVER_TAG, "Config updated successfully => %s_min=%d ; %s_max=%d", kind, *min_val, kind,
                         *max_val);

                /* Send a response back to the client */
                httpd_resp_send(req, "Data received successfully", HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            } else
                ESP_LOGE(WEBSERVER_TAG, "%s config values not updated as received data contains invalid characters!", kind);
        } else
            ESP_LOGE(WEBSERVER_TAG, "%s config not updated as received data is malformed!", kind);
    }
    httpd_resp_set_status(req, "400");
    httpd_resp_send(req, "Request body is invalid!", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}

static void send_html_response(httpd_req_t *req, const unsigned char *start, const unsigned char *end) {
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
 * The get_sensor_readout_handler replies to the GET request with the JSON formatted sensor data
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t get_sensor_readout_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling sensor readout request");
    httpd_resp_set_type(req, "application/json");

    char buf[strlen(JSON_SENSOR_DATA) + JSON_BUF_INCREASE];

    update_sensor_data();

    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    sprintf(buf, JSON_SENSOR_DATA, sensor_data.temperature, sensor_data.light_level, sensor_data.humidity,
            sensor_data.moisture);
    xSemaphoreGive(sensor_data_mutex);

    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 * The post_config_temperature_handler replies to the GET request with the JSON formatted range data
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t get_range_data_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling range data request");
    httpd_resp_set_type(req, "application/json");

    char buf[strlen(JSON_RANGE_DATA) + JSON_BUF_INCREASE];

    sprintf(buf, JSON_RANGE_DATA,
            range_config.temperature_min, range_config.temperature_max,
            range_config.light_level_min, range_config.light_level_max,
            range_config.humidity_min, range_config.humidity_max,
            range_config.moisture_min, range_config.moisture_max);

    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 * The post_config_temperature_handler replies to the GET request with the JSON formatted range data
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t get_pump_status_handler(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling pump status request");
    httpd_resp_set_type(req, "application/json");

    char buf[strlen(JSON_PUMP_STATUS_DATA) + JSON_BUF_INCREASE];

    sprintf(buf, JSON_PUMP_STATUS_DATA, pump_task_toggle ? "true" : "false");

    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 * The post_config_temperature_handler function handles POST requests to update the ideal temperature range
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t post_config_temperature_handler(httpd_req_t *req) {
    receive_and_parse_data(req, &range_config.temperature_min, &range_config.temperature_max, "temperature");

    return ESP_OK;
}

/*
 * The post_config_light_level_handler function handles POST requests to update the ideal light level range
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t post_config_light_level_handler(httpd_req_t *req) {
    receive_and_parse_data(req, &range_config.light_level_min, &range_config.light_level_max, "light_level");

    return ESP_OK;
}

/*
 * The post_config_humidity_handler function handles POST requests to update the ideal humidity range
 *
 * httpd_req_t *req:    Pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t post_config_humidity_handler(httpd_req_t *req) {
    receive_and_parse_data(req, &range_config.humidity_min, &range_config.humidity_max, "humidity");

    return ESP_OK;
}

/*
 * The post_config_moisture_handler function handles POST requests to update the ideal moisture range
 *
 * httpd_req_t *req:    pointer to the received HTTP request
 *
 * returns:             ESP_OK
 */
static esp_err_t post_config_moisture_handler(httpd_req_t *req) {
    receive_and_parse_data(req, &range_config.moisture_min, &range_config.moisture_max, "moisture");

    return ESP_OK;
}

static esp_err_t post_config_pump_status(httpd_req_t *req) {
    ESP_LOGI(WEBSERVER_TAG, "Handling pump toggle config post request");

    /* Buffer size + 1 to accommodate the null terminator */
    char buffer[POST_REQUEST_BUF_SIZE + 1];

    if (read_request_content(req, buffer, sizeof(buffer), "pump_status") == ESP_OK) {
        char *end_ptr = NULL;

        int temp = strtol(buffer, &end_ptr, 2);

        if (! *end_ptr) {
            pump_task_toggle = temp;

            if (pump_task_toggle) vTaskResume(pump_task);

            ESP_LOGI(WEBSERVER_TAG, "Pump task %s successfully!", pump_task_toggle ? "resumed" : "suspended");

            /* Send a response back to the client */
            httpd_resp_send(req, "Data received successfully", HTTPD_RESP_USE_STRLEN);
        } else {
            ESP_LOGE(WEBSERVER_TAG, "Pump task config value not updated as received data contains invalid characters!");
        }
    }

    httpd_resp_set_status(req, "400");
    httpd_resp_send(req, "Request body is invalid!", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
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

static httpd_uri_t uri_get_pump_status = {
        .uri = "/api/v1/pump",
        .method = HTTP_GET,
        .handler = get_pump_status_handler,
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

static httpd_uri_t uri_post_config_humidity = {
        .uri = "/api/v1/config/humidity",
        .method = HTTP_POST,
        .handler = post_config_humidity_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_post_config_moisture = {
        .uri = "/api/v1/config/moisture",
        .method = HTTP_POST,
        .handler = post_config_moisture_handler,
        .user_ctx = NULL
};

static httpd_uri_t uri_post_config_pump_status = {
        .uri = "/api/v1/config/pump",
        .method = HTTP_POST,
        .handler = post_config_pump_status,
        .user_ctx = NULL
};

/* End of URI config */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Starts the webserver and registers handlers */
static httpd_handle_t start_webserver() {
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;
    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_config);
        httpd_register_uri_handler(server, &uri_get_sensor_readout);
        httpd_register_uri_handler(server, &uri_get_range_data);
        httpd_register_uri_handler(server, &uri_get_pump_status);

        httpd_register_uri_handler(server, &uri_post_config_temperature);
        httpd_register_uri_handler(server, &uri_post_config_light_level);
        httpd_register_uri_handler(server, &uri_post_config_humidity);
        httpd_register_uri_handler(server, &uri_post_config_moisture);
        httpd_register_uri_handler(server, &uri_post_config_pump_status);

        ESP_LOGI(WEBSERVER_TAG, "Web server started successfully!");
    } else {
        ESP_LOGE(WEBSERVER_TAG, "Failed to start web server. Resetting device...");
        esp_restart();
    }
    return server;
}

/* End of Webserver section*/
/* ------------------------------------------------------------------------------------------------------------------ */

/* Updates all sensor data and sets the values in sensor_data struct accordingly */
static void update_sensor_data() {
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    bme680_values_float_t values;

    /* Gets an approximation of the time the measurement will take  */
    uint32_t duration = bme680_get_measurement_duration(sensor);
    /* Start a measurement of humidity and temperature via the bme680 sensor */
    if (bme680_force_measurement(sensor)) {
        /* Passive waiting for the duration of the measurement
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
    xSemaphoreGive(sensor_data_mutex);
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
    ESP_LOGI(PUMP_TAG, "Pump task started successfully!");

    if (!pump_task_toggle)
        ESP_LOGI(PUMP_TAG, "Pump task will be suspended now because automatic watering is turned off.\n"
                           "You can turn watering on until next startup via the webinterface or "
                           "enable it permanently in the config file.");

    while (true) {
        if (!pump_task_toggle) vTaskSuspend(NULL);

        ESP_LOGI(PUMP_TAG, "Checking if plant needs watering...");
        /* Update sensor data for an accurate check */
        update_sensor_data();

        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        int measured_moisture = sensor_data.moisture;
        xSemaphoreGive(sensor_data_mutex);

        ESP_LOGI(PUMP_TAG, "Measured soil moisture: %d %%; Minimum set soil moisture: %d %%", measured_moisture,
                 range_config.moisture_min);
        if (measured_moisture < range_config.moisture_min) {
            ESP_LOGI(PUMP_TAG, "Plant needs watering! Starting pump for %d seconds...",
                     pump_config.pump_watering_time_ms / 1000);
            /* Activate the water pump by setting the GPIO-level to ON for the specified pump time then turn OFF again*/
            gpio_set_level(PUMP_GPIO, GPIO_ON);
            vTaskDelay(pdMS_TO_TICKS(pump_config.pump_watering_time_ms));
            ESP_LOGI(PUMP_TAG, "Turning pump off and giving the water some time to settle...");
            gpio_set_level(PUMP_GPIO, GPIO_OFF);
        } else
            ESP_LOGI(PUMP_TAG, "Plant doesn't need to be watered right now.");
        if (pump_task_toggle) {
            ESP_LOGI(PUMP_TAG, "Rechecking soil moisture in %d minutes", pump_config.recheck_time_ms / 60000);
            /* Wait the configured amount of time before rechecking again */
            vTaskDelay(pdMS_TO_TICKS(pump_config.recheck_time_ms));
        }
    }
}

static void bme680_init() {
    /* Init the sensor with default slave address connected to I2C_BUS. */
    sensor = bme680_init_sensor(I2C_BUS, BME680_I2C_ADDRESS_1, 0);

    if (sensor) {
        /* -- SENSOR CONFIGURATION --- */

        /* Disables the heater because it's only used for gas measurements that we don't take. */
        bme680_use_heater_profile(sensor, BME680_HEATER_NOT_USED);
    }
}

static void pump_init() {
    /* Set pump GPIO to output mode */
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);

    /* Make sure the pump is turned off on start */
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

    if (!(sensor_data_mutex = xSemaphoreCreateMutex())) {
        ESP_LOGE(WEBSERVER_TAG, "Couldn't allocate memory for the sensor data mutex. Resetting device...");
        esp_restart();
    }

    /* Initializing WiFi-Connection */
    wifi_init_sta();

    while (!wifi_established) { vTaskDelay(1); }

    /* Start the ESP-Webserver */
    ESP_LOGI(MAIN_TAG, "Starting web server...");
    start_webserver();

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
        /* creates a Task that periodically checks if the plant needs watering */
        ESP_LOGI(MAIN_TAG, "Starting pump task...");
        xTaskCreate(water_plant, "water_plant", TASK_STACK_DEPTH, NULL, 2, &pump_task);
    } else
        ESP_LOGE(MAIN_TAG, "Could not initialize BME680 sensor");
#endif //CONFIG_MODE
}
