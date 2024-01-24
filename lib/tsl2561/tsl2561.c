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
#include "esp8266_wrapper.h"
#include "tsl2561.h"

uint8_t prefix_register_address(uint8_t reg_addr) {
    return (0b1000 << 4) | reg_addr;
}

void tsl2561_sensor_init(uint8_t bus) {
    uint8_t data = 0x03;
    uint8_t reg = prefix_register_address(0x0);
    i2c_slave_write(bus, 0x29, &reg, &data, 1);

    data = 0b10010;
    reg = prefix_register_address(0x1);
    i2c_slave_write(bus, 0x29, &reg, &data, 1);
}

float tsl2561_read_sensor_value(uint8_t bus) {
    uint8_t reg;

    uint8_t ch0[2];
    uint8_t ch1[2];

    float CH0;
    float CH1;
    float div;

    float lux;

    reg = prefix_register_address(0xC);
    i2c_slave_read(bus, 0x29, &reg, ch0, 2);

    reg = prefix_register_address(0xE);
    i2c_slave_read(bus, 0x29, &reg, ch1, 2);

    // Prevent division by zero for the next step
    if (((ch0[1] << 8) | ch0[0]) == 0) {
        return 0;
    }

    CH0 = (float) ((ch0[1] << 8) | ch0[0]);
    CH1 = (float) ((ch1[1] << 8) | ch1[0]);

    div = CH1 / CH0;

    // printf("CH0: %d, CH1: %d; DIV: %f\n", ch0[1] << 8 | ch0[0], ch1[1] << 8 | ch1[0], div);

    if (0 < div && div <= 0.5) {
        lux = (float) (0.0304 * CH0 - 0.062 * CH0 * pow(div, 1.4));
    } else if (0.5 < div && div <= 0.61) {
        lux = (float) (0.0224 * CH0 - 0.031 * CH1);
    } else if (0.61 < div && div <= 0.8) {
        lux = (float) (0.0128 * CH0 - 0.0153 * CH1);
    } else if (0.8 < div && div <= 1.3) {
        lux = (float) (0.00146 * CH0 - 0.00112 * CH1);
    } else if (div > 1.3) {
        lux = 0;
    } else lux = -1;

    return lux;
}
