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

#include "driver/adc.h"
#include "bme680.h"
#include "moisture_sensor.h"

#define LOGO "\
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

#define TASK_STACK_DEPTH 2048

#define I2C_BUS       0
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define I2C_FREQ      I2C_FREQ_100K

#define MOISTURE_SENSOR_CHANNEL ADC1_CHANNEL_0

static bme680_sensor_t *sensor = 0;

/*
 * User task that triggers measurements of sensor every second. It uses
 * function *vTaskDelay* to wait for measurement results. Busy wating
 * alternative is shown in comments
 */
_Noreturn void user_task(void *pvParameters) {
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
                int moist = moisture_sensor_read(MOISTURE_SENSOR_CHANNEL);

                printf("[%.3f] Temp: %.2f °C, Hum: %.2f %%, Moist: %d %%\n",
                       (double) sdk_system_get_time() * 1e-3,
                       values.temperature, values.humidity, moist);
                       // values.pressure, values.gas_resistance);
                bme680_set_ambient_temperature(sensor, (int16_t) values.temperature);
            }
        }
        // passive waiting until 1 second is over
        xTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
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


void app_main() {
    puts(LOGO);

    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    // Init I2C bus interfaces
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    // Init BME680 sensor
    bme680_init();

    // Init moisture sensor
    moisture_sensor_init(MOISTURE_SENSOR_CHANNEL);


    /** -- TASK CREATION --- */

    // must be done last to avoid concurrency situations with the sensor configuration

    // Create a task that uses the sensor
    if (sensor)
        xTaskCreate(user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, NULL);
    else
        printf("Could not initialize BME680 sensor\n");
}
